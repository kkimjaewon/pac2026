#include "rt_control/ai_command_receiver.hpp"

#include <chrono>

#include "rt_control/rt_control_loop.hpp"

namespace rt_control {

using intelion_interfaces::msg::ControlCommand;

namespace {

constexpr auto kLoggerSleep = std::chrono::milliseconds(50);  // 파일 저장 스레드가 링버퍼를 비우는 간격

}  // namespace

AiCommandReceiver::AiCommandReceiver(rclcpp::Node& node, const std::string& log_path) : log_path_(log_path) {
  log_file_ = std::fopen(log_path_.c_str(), "w");
  if (log_file_) {
    std::fprintf(log_file_, "seq,writer_boot_id,status,age_ns,recv_mono_ns,dropped\n");
  } else {
    RCLCPP_WARN(node.get_logger(), "수신 기록 파일을 열 수 없음: %s (기록 없이 계속)",
                log_path_.c_str());
  }
  logger_thread_ = std::thread(&AiCommandReceiver::logger_main, this);

  // BestEffort / Volatile / KeepLast 5
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(kQosDepth)).best_effort().durability_volatile();
  sub_ = node.create_subscription<ControlCommand>(
      kTopic, qos, [this](const ControlCommand::ConstSharedPtr msg) { on_message(*msg); });
}

AiCommandReceiver::~AiCommandReceiver() {
  sub_.reset();
  stop_logger_.store(true, std::memory_order_relaxed);
  if (logger_thread_.joinable()) logger_thread_.join();
  if (log_file_) std::fclose(log_file_);
}

void AiCommandReceiver::on_message(const ControlCommand& msg) {
  const int64_t recv_ns = now_ns();  // 가장 먼저 받은 시각부터 찍음
  received_.fetch_add(1, std::memory_order_relaxed);

  // 새 명령 판정: boot_id가 바뀌면 A가 재시작된 것 -> 기준을 리셋하고 새 명령으로 인정.
  // 같은 boot_id에서는 seq가 마지막 값보다 클 때만 새 명령.
  if (has_last_ && msg.writer_boot_id != last_boot_id_) {
    boot_changes_.fetch_add(1, std::memory_order_relaxed);
    has_last_ = false;
  }
  const bool is_new = !has_last_ || msg.seq > last_seq_;

  if (is_new) {
    has_last_ = true;
    last_boot_id_ = msg.writer_boot_id;
    last_seq_ = msg.seq;

    AiCommandSnapshot snap{};
    snap.seq = msg.seq;
    snap.writer_boot_id = msg.writer_boot_id;
    snap.layout_version = msg.layout_version;
    snap.command_type = msg.command_type;
    snap.status = msg.status;
    for (size_t i = 0; i < 4; ++i) snap.target[i] = msg.target[i];
    snap.confidence = msg.confidence;
    snap.age_ns = msg.age_ns;
    snap.recv_mono_ns = recv_ns;
    buffer_.write(snap);
  } else {
    dropped_.fetch_add(1, std::memory_order_relaxed);
  }

  const RxLogRow row{msg.seq,    msg.writer_boot_id, msg.status, static_cast<uint8_t>(!is_new),
                     msg.age_ns, recv_ns};
  if (!log_ring_.push(row)) log_overflow_.fetch_add(1, std::memory_order_relaxed);
}

void AiCommandReceiver::logger_main() {
  RxLogRow row;
  for (;;) {
    // stop 신호를 먼저 읽고 나서 비워야, 종료 직전에 들어온 줄까지 다 저장됨
    const bool stopping = stop_logger_.load(std::memory_order_relaxed);
    bool wrote = false;
    while (log_ring_.pop(row)) {
      if (!log_file_) continue;
      std::fprintf(log_file_, "%llu,%u,%u,%lld,%lld,%u\n", (unsigned long long)row.seq,
                   row.writer_boot_id, row.status, (long long)row.age_ns,
                   (long long)row.recv_mono_ns, row.dropped);
      wrote = true;
    }
    if (wrote) std::fflush(log_file_);
    if (stopping) break;
    std::this_thread::sleep_for(kLoggerSleep);
  }
}

}  // namespace rt_control
