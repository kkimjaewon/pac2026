// B <-> C: ROS 2 다리 (ros_bridge)
//
// B의 1ms 루프는 ROS를 모른다. 이 다리가 공유 버퍼와 ROS 토픽 사이를 이어 준다.
//
//   [발행] 1ms 루프 --Seqlock<HalCommandSnapshot>--> 발행 스레드 --> /hal/final_command
//     발행 스레드는 자기만의 1ms 절대 시각 타이머(clock_nanosleep)로 깨어나서 버퍼를 읽고,
//     command_seq가 지난번과 다르면 발행한다. 깨어나는 위상은 1ms 루프가 명령을 쓰는 시각의
//     중간(kPublishPhaseNs 뒤)에 맞춘다. 두 타이머가 같은 순간에 깨면 지터 때문에
//     어떤 주기는 새 명령을 못 보고(같은 seq), 다음 주기는 두 개를 넘게 되기(건너뛴 seq) 때문.
//
//   [구독] /hal/state --> 콜백(통신 스레드) --Seqlock<HalStateSnapshot>--> 1ms 루프
//          /hal/error --> 콜백(통신 스레드) --SpscRing<HalErrorEvent>--> 소비자(지금은 요약 로그)
//                                          + 지금 켜져 있는 오류 목록(비트마스크)
//
// 어느 쪽도 1ms 루프를 기다리게 하지 않는다 (Seqlock·링버퍼 모두 기다림 없음).
#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include <intelion_interfaces/msg/hal_command.hpp>
#include <intelion_interfaces/msg/hal_error.hpp>
#include <intelion_interfaces/msg/hal_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rt_control/hal_command_snapshot.hpp"
#include "rt_control/hal_state_snapshot.hpp"
#include "rt_control/seqlock.hpp"
#include "rt_control/spsc_ring.hpp"

namespace rt_control {

// 첫 발행 몇 번의 구간별 측정 (원인 확인용: 메모리 할당·DDS 첫 전송·잠금 대기를 구분)
//   now_ns     : clock_->now() 시간
//   publish_ns : publish() 시간
//   minflt     : 그 동안 이 스레드의 minor page fault 수 (메모리 새로 할당·처음 접근)
//   majflt     : major page fault 수 (디스크)
//   nvcsw      : 자발적 컨텍스트 스위치 수 (잠금·소켓 등에서 기다림)
//   gap_ns     : 이번 발행 끝 ~ 다음 깨어남 사이에 늦어진 시간 (발행 뒤 다른 스레드에 밀렸는지)
struct PublishProfile {
  int64_t now_ns;
  int64_t publish_ns;
  int64_t minflt;
  int64_t majflt;
  int64_t nvcsw;
  int64_t nivcsw;
  int64_t late_ns;     // 이번에 깨어난 시각 - 예정 시각
  uint64_t command_seq;
};
constexpr int kProfiledPublishes = 5;

struct BridgeConfig {
  int64_t period_ns = 1'000'000;  // 발행 주기 1ms
  int cpu = -1;                   // 발행 스레드를 고정할 CPU (-1 = 고정 안 함)
  int loop_cpu = -1;              // 1ms 루프의 CPU (겹치는지 경고용)
  int fifo_priority = 0;          // 발행 스레드 SCHED_FIFO 우선순위 (0 = 일반 스케줄링). 1ms 루프보다 낮게
};

class RosBridge {
 public:
  static constexpr const char* kCommandTopic = "/hal/final_command";
  static constexpr const char* kStateTopic = "/hal/state";
  static constexpr const char* kErrorTopic = "/hal/error";
  static constexpr const char* kFrameId = "map";
  static constexpr size_t kErrorQosDepth = 32;
  static constexpr size_t kErrorRingSize = 256;
  // [제안] 1ms 루프가 명령을 쓴 시각(created)에서 이만큼 뒤에 발행 스레드가 깨어남 (주기의 절반)
  static constexpr int64_t kPublishPhaseNs = 500'000;

  // 오류 코드는 비트마스크(1 << code)로 들고 있음. 이보다 큰 코드는 개수만 셈
  static constexpr int kMaxErrorCode = 63;

  RosBridge(rclcpp::Node& node, const Seqlock<HalCommandSnapshot>& command_buffer,
            const BridgeConfig& config);
  ~RosBridge();

  RosBridge(const RosBridge&) = delete;
  RosBridge& operator=(const RosBridge&) = delete;

  // 발행 스레드 시작 / 정지 (stop은 여러 번 불러도 됨)
  void start();
  void stop();

  // 1ms 루프가 읽는 최신 C 상태 버퍼
  const Seqlock<HalStateSnapshot>& state_buffer() const { return state_buffer_; }

  // /hal/error 이벤트 링버퍼에서 하나 꺼냄 (소비자 한 곳에서만 호출. 지금은 요약 로그 스레드)
  bool pop_error_event(HalErrorEvent& out) { return error_ring_.pop(out); }

  // 발행 쪽 누적 값
  uint64_t published_count() const { return published_.load(std::memory_order_relaxed); }
  uint64_t skipped_seq_count() const { return skipped_seq_.load(std::memory_order_relaxed); }
  uint64_t read_failure_count() const { return read_failures_.load(std::memory_order_relaxed); }
  uint64_t last_published_seq() const { return last_published_seq_.load(std::memory_order_relaxed); }
  uint64_t publish_overrun_count() const { return overruns_.load(std::memory_order_relaxed); }
  // 발행 스레드가 깨어난 누적 수 (INIT에서 발행하지 않아도 셈. warmup의 발행 안정 판단용)
  uint64_t publish_tick_count() const { return ticks_.load(std::memory_order_relaxed); }
  // 가장 오래 걸린 publish() 시간 [ns] (누적 최고)
  int64_t max_publish_ns() const { return max_publish_ns_.load(std::memory_order_relaxed); }
  // 첫 kProfiledPublishes번 발행 측정. 다 모였으면 true (메인 스레드가 한 번 출력)
  bool publish_profile(PublishProfile out[kProfiledPublishes]) const;
  // /hal/final_command 구독자 수. rmw를 부르므로(잠금 있음) 1ms 루프에서 부르지 말 것 (메인 스레드 전용)
  size_t command_subscriber_count() const { return command_pub_->get_subscription_count(); }

  // 구독 쪽 누적 값
  uint64_t state_received_count() const { return state_received_.load(std::memory_order_relaxed); }
  uint64_t error_received_count() const { return error_received_.load(std::memory_order_relaxed); }
  uint64_t error_ring_overflow_count() const { return error_overflow_.load(std::memory_order_relaxed); }
  uint64_t state_error_overflow_count() const { return state_error_overflow_.load(std::memory_order_relaxed); }
  // 지금 켜져 있는 오류 코드 (/hal/error 이벤트 기준). bit n = 코드 n
  uint64_t active_error_mask() const { return active_error_mask_.load(std::memory_order_relaxed); }

 private:
  void publish_main();
  void on_state(const intelion_interfaces::msg::HalState& msg);
  void on_error(const intelion_interfaces::msg::HalError& msg);

  const Seqlock<HalCommandSnapshot>& command_buffer_;
  BridgeConfig config_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  rclcpp::Publisher<intelion_interfaces::msg::HalCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<intelion_interfaces::msg::HalState>::SharedPtr state_sub_;
  rclcpp::Subscription<intelion_interfaces::msg::HalError>::SharedPtr error_sub_;

  Seqlock<HalStateSnapshot> state_buffer_;
  SpscRing<HalErrorEvent, kErrorRingSize> error_ring_;

  std::atomic<bool> stop_requested_{false};
  std::thread publish_thread_;

  std::atomic<uint64_t> published_{0};
  std::atomic<uint64_t> skipped_seq_{0};       // 발행이 늦어서 한 번도 못 내보낸 command_seq 수
  std::atomic<uint64_t> read_failures_{0};     // Seqlock 3회 재시도 모두 실패
  std::atomic<uint64_t> last_published_seq_{0};
  std::atomic<uint64_t> overruns_{0};          // 발행 스레드가 늦어서 건너뛴 자기 주기 수
  std::atomic<uint64_t> ticks_{0};             // 발행 스레드가 깨어난 수
  std::atomic<int64_t> max_publish_ns_{0};
  PublishProfile profile_[kProfiledPublishes] = {};  // 발행 스레드만 씀, profile_count_로 공개
  std::atomic<int> profile_count_{0};

  std::atomic<uint64_t> state_received_{0};
  std::atomic<uint64_t> state_error_overflow_{0};  // active_errors가 kMaxActiveErrors를 넘은 횟수
  std::atomic<uint64_t> error_received_{0};
  std::atomic<uint64_t> error_overflow_{0};        // 링버퍼가 가득 차서 버린 이벤트 수
  std::atomic<uint64_t> active_error_mask_{0};
};

}  // namespace rt_control
