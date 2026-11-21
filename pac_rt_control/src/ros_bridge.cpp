#include "rt_control/ros_bridge.hpp"

#include <errno.h>
#include <pthread.h>
#include <sys/resource.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <cstring>
#include <vector>

#include "rt_control/rt_control_loop.hpp"

namespace rt_control {

using intelion_interfaces::msg::HalCommand;
using intelion_interfaces::msg::HalError;
using intelion_interfaces::msg::HalState;

namespace {

constexpr int64_t kNsPerSec = 1'000'000'000;

void thread_usage(rusage& ru) { getrusage(RUSAGE_THREAD, &ru); }

timespec to_timespec(int64_t ns) {
  timespec ts;
  ts.tv_sec = static_cast<time_t>(ns / kNsPerSec);
  ts.tv_nsec = static_cast<long>(ns % kNsPerSec);
  return ts;
}

}  // namespace

RosBridge::RosBridge(rclcpp::Node& node, const Seqlock<HalCommandSnapshot>& command_buffer,
                     const BridgeConfig& config)
    : command_buffer_(command_buffer),
      config_(config),
      clock_(node.get_clock()),
      logger_(node.get_logger().get_child("ros_bridge")) {
  // 발행 스레드 CPU가 1ms 루프 코어나 그 짝꿍(같은 물리 코어)이면 경고
  if (config_.cpu >= 0 && config_.loop_cpu >= 0) {
    if (config_.cpu == config_.loop_cpu) {
      RCLCPP_WARN(logger_, "bridge_cpu=%d 가 1ms 루프 CPU와 같음. 루프 지터가 커질 수 있음", config_.cpu);
    } else if (const std::vector<int> sib = thread_siblings(config_.loop_cpu);
               std::find(sib.begin(), sib.end(), config_.cpu) != sib.end()) {
      RCLCPP_WARN(logger_, "bridge_cpu=%d 는 1ms 루프 CPU %d 의 짝꿍 코어(같은 물리 코어). 비워 두는 것을 권장",
                  config_.cpu, config_.loop_cpu);
    }
  }

  // /hal/final_command: BestEffort / Volatile / KeepLast 1
  const rclcpp::QoS cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  command_pub_ = node.create_publisher<HalCommand>(kCommandTopic, cmd_qos);

  // /hal/state: BestEffort / KeepLast 1
  const rclcpp::QoS state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  state_sub_ = node.create_subscription<HalState>(
      kStateTopic, state_qos, [this](const HalState::ConstSharedPtr msg) { on_state(*msg); });

  // /hal/error: Reliable / Volatile / KeepLast 32 (켜짐·꺼짐 이벤트를 놓치지 않게)
  const rclcpp::QoS error_qos = rclcpp::QoS(rclcpp::KeepLast(kErrorQosDepth)).reliable().durability_volatile();
  error_sub_ = node.create_subscription<HalError>(
      kErrorTopic, error_qos, [this](const HalError::ConstSharedPtr msg) { on_error(*msg); });
}

RosBridge::~RosBridge() { stop(); }

bool RosBridge::publish_profile(PublishProfile out[kProfiledPublishes]) const {
  if (profile_count_.load(std::memory_order_acquire) < kProfiledPublishes) return false;
  for (int i = 0; i < kProfiledPublishes; ++i) out[i] = profile_[i];
  return true;
}

void RosBridge::start() {
  if (publish_thread_.joinable()) return;
  stop_requested_.store(false, std::memory_order_relaxed);
  publish_thread_ = std::thread(&RosBridge::publish_main, this);
}

void RosBridge::stop() {
  stop_requested_.store(true, std::memory_order_relaxed);
  if (publish_thread_.joinable()) publish_thread_.join();
}

void RosBridge::publish_main() {
  if (config_.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(config_.cpu, &set);
    const int err = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (err == 0) {
      RCLCPP_INFO(logger_, "발행 스레드 CPU %d 고정: 성공", config_.cpu);
    } else {
      RCLCPP_WARN(logger_, "발행 스레드 CPU %d 고정 실패 (%s), 고정 없이 계속", config_.cpu, std::strerror(err));
    }
  }
  // 늦게 깨면 1ms 루프가 쓴 명령을 못 보고 건너뛰게 되므로, 실시간 옵션이 켜져 있으면 FIFO로
  if (config_.fifo_priority > 0) {
    sched_param param{};
    param.sched_priority = config_.fifo_priority;
    const int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (err == 0) {
      RCLCPP_INFO(logger_, "발행 스레드 SCHED_FIFO 우선순위 %d: 성공", config_.fifo_priority);
    } else {
      RCLCPP_WARN(logger_, "발행 스레드 SCHED_FIFO 설정 실패 (%s), 일반 스케줄링으로 계속", std::strerror(err));
    }
  }

  // 메시지는 한 번 만들어 두고 값만 바꿔 씀 (frame_id 문자열 할당을 매번 하지 않게)
  HalCommand msg;
  msg.header.frame_id = kFrameId;
  msg.mode = HalCommand::MODE_POSITION;

  const int64_t period = config_.period_ns;
  int64_t next = now_ns() + period;
  bool phase_aligned = false;
  uint64_t last_session = 0;
  uint64_t last_seq = 0;
  HalCommandSnapshot cmd{};

  while (!stop_requested_.load(std::memory_order_relaxed)) {
    const timespec ts = to_timespec(next);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
      if (stop_requested_.load(std::memory_order_relaxed)) return;
    }
    const int64_t woke_ns = now_ns();
    ticks_.fetch_add(1, std::memory_order_relaxed);

    // 1ms 루프가 아직 RUN이 아니면(INIT) 버퍼가 비어 있음 -> 발행하지 않음
    if (command_buffer_.ever_written()) {
      if (!command_buffer_.try_read(cmd)) {
        read_failures_.fetch_add(1, std::memory_order_relaxed);
      } else if (cmd.session_id != last_session || cmd.command_seq != last_seq) {
        // 같은 세션에서 번호가 2 이상 뛰었으면 그 사이 명령은 한 번도 못 내보낸 것
        if (cmd.session_id == last_session && cmd.command_seq > last_seq + 1) {
          skipped_seq_.fetch_add(cmd.command_seq - last_seq - 1, std::memory_order_relaxed);
        }
        if (cmd.session_id != last_session) phase_aligned = false;  // 새 세션이면 위상을 다시 맞춤
        last_session = cmd.session_id;
        last_seq = cmd.command_seq;

        const int pi = profile_count_.load(std::memory_order_relaxed);
        rusage ru0{}, ru1{};
        if (pi < kProfiledPublishes) thread_usage(ru0);
        const int64_t t0 = now_ns();
        msg.header.stamp = clock_->now();
        const int64_t t1 = now_ns();
        msg.session_id = cmd.session_id;
        msg.command_seq = cmd.command_seq;
        msg.created_mono_ns = cmd.created_mono_ns;
        msg.expires_mono_ns = cmd.expires_mono_ns;
        msg.target_pose.position.x = cmd.pos[0];
        msg.target_pose.position.y = cmd.pos[1];
        msg.target_pose.position.z = cmd.pos[2];
        msg.target_pose.orientation.x = cmd.quat[0];
        msg.target_pose.orientation.y = cmd.quat[1];
        msg.target_pose.orientation.z = cmd.quat[2];
        msg.target_pose.orientation.w = cmd.quat[3];
        msg.source_result_id = cmd.source_result_id;
        command_pub_->publish(msg);
        const int64_t t2 = now_ns();
        if (t2 - t1 > max_publish_ns_.load(std::memory_order_relaxed)) {
          max_publish_ns_.store(t2 - t1, std::memory_order_relaxed);
        }
        if (pi < kProfiledPublishes) {
          thread_usage(ru1);
          profile_[pi] = PublishProfile{t1 - t0,
                                        t2 - t1,
                                        ru1.ru_minflt - ru0.ru_minflt,
                                        ru1.ru_majflt - ru0.ru_majflt,
                                        ru1.ru_nvcsw - ru0.ru_nvcsw,
                                        ru1.ru_nivcsw - ru0.ru_nivcsw,
                                        woke_ns - next,
                                        cmd.command_seq};
          profile_count_.store(pi + 1, std::memory_order_release);
        }
        published_.fetch_add(1, std::memory_order_relaxed);
        last_published_seq_.store(cmd.command_seq, std::memory_order_relaxed);

        // 처음 명령을 봤을 때 한 번: 다음 깨어날 시각을 "명령을 쓴 시각 + 반 주기"에 맞춤.
        // 두 타이머 모두 CLOCK_MONOTONIC 1ms 절대 시각이라 이후로는 위상이 어긋나지 않는다.
        if (!phase_aligned) {
          next = cmd.created_mono_ns + kPublishPhaseNs;
          phase_aligned = true;
        }
      }
    }

    next += period;
    const int64_t now = now_ns();
    if (now >= next) {  // 늦어서 다음 예정 시각도 지났으면 몰아서 돌지 않고 건너뜀
      const int64_t missed = (now - next) / period + 1;
      next += missed * period;
      overruns_.fetch_add(static_cast<uint64_t>(missed), std::memory_order_relaxed);
    }
  }
}

void RosBridge::on_state(const HalState& msg) {
  const int64_t recv_ns = now_ns();  // 가장 먼저 받은 시각부터 찍음
  state_received_.fetch_add(1, std::memory_order_relaxed);

  HalStateSnapshot s{};
  s.recv_mono_ns = recv_ns;
  s.received_mono_ns = msg.received_mono_ns;
  s.state_seq = msg.state_seq;
  s.active_session_id = msg.active_session_id;
  s.last_command_seq = msg.last_command_seq;
  s.tcp_pos[0] = msg.tcp_pose.position.x;
  s.tcp_pos[1] = msg.tcp_pose.position.y;
  s.tcp_pos[2] = msg.tcp_pose.position.z;
  s.tcp_quat[0] = msg.tcp_pose.orientation.x;
  s.tcp_quat[1] = msg.tcp_pose.orientation.y;
  s.tcp_quat[2] = msg.tcp_pose.orientation.z;
  s.tcp_quat[3] = msg.tcp_pose.orientation.w;
  size_t n = msg.active_errors.size();
  if (n > static_cast<size_t>(kMaxActiveErrors)) {
    state_error_overflow_.fetch_add(1, std::memory_order_relaxed);
    n = kMaxActiveErrors;
  }
  for (size_t i = 0; i < n; ++i) s.active_errors[i] = msg.active_errors[i];
  s.active_error_count = static_cast<uint8_t>(n);
  s.hal_state = msg.hal_state;
  s.last_command_result = msg.last_command_result;
  s.stale = msg.stale;
  s.pose_valid = msg.pose_valid;
  state_buffer_.write(s);
}

void RosBridge::on_error(const HalError& msg) {
  const int64_t recv_ns = now_ns();
  error_received_.fetch_add(1, std::memory_order_relaxed);

  // 지금 켜져 있는 오류 목록 갱신 (이 콜백만 쓰므로 load -> store로 충분)
  if (msg.code <= kMaxErrorCode) {
    const uint64_t bit = uint64_t{1} << msg.code;
    const uint64_t mask = active_error_mask_.load(std::memory_order_relaxed);
    active_error_mask_.store(msg.active ? (mask | bit) : (mask & ~bit), std::memory_order_relaxed);
  }

  const HalErrorEvent ev{msg.event_mono_ns, recv_ns, msg.session_id, msg.command_seq, msg.code, msg.active};
  if (!error_ring_.push(ev)) error_overflow_.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace rt_control
