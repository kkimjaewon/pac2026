// A -> B: /ai/control_command 수신기
//
// ROS 콜백(통신 스레드)에서 메시지를 받아
//   1) CLOCK_MONOTONIC으로 받은 시각(recv_mono_ns)을 찍고
//   2) (writer_boot_id, seq)로 새 명령인지 판정한 뒤
//   3) 새 명령이면 Seqlock 버퍼에 쓴다.
// 값 검사(gate: 상태·TTL·범위·NaN)는 여기서 하지 않는다. 그건 1ms 루프의 몫.
//
// 받은 메시지마다 한 줄씩 ai_rx.csv(노드가 정한 실행 폴더 ~/pac_logs/<run_id>/ 안)에 기록한다.
// 파일 쓰기는 별도 스레드가 하고, 콜백은 링버퍼에 넣기만 한다(기다리지 않음).
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include <intelion_interfaces/msg/control_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rt_control/ai_command_snapshot.hpp"
#include "rt_control/seqlock.hpp"
#include "rt_control/spsc_ring.hpp"

namespace rt_control {

class AiCommandReceiver {
 public:
  static constexpr const char* kTopic = "/ai/control_command";
  static constexpr size_t kQosDepth = 5;
  static constexpr size_t kLogRingSize = 4096;  // 파일 저장 대기 줄 수 (15Hz면 수 분치)

  // log_path에 수신 기록 CSV를 만든다 (폴더는 이미 있어야 함)
  AiCommandReceiver(rclcpp::Node& node, const std::string& log_path);
  ~AiCommandReceiver();

  AiCommandReceiver(const AiCommandReceiver&) = delete;
  AiCommandReceiver& operator=(const AiCommandReceiver&) = delete;

  // 1ms 루프가 읽는 최신 명령 버퍼
  const Seqlock<AiCommandSnapshot>& buffer() const { return buffer_; }

  uint64_t received_count() const { return received_.load(std::memory_order_relaxed); }
  uint64_t dropped_count() const { return dropped_.load(std::memory_order_relaxed); }
  uint64_t boot_change_count() const { return boot_changes_.load(std::memory_order_relaxed); }
  uint64_t log_overflow_count() const { return log_overflow_.load(std::memory_order_relaxed); }
  const std::string& log_path() const { return log_path_; }

 private:
  // CSV 한 줄
  struct RxLogRow {
    uint64_t seq;
    uint32_t writer_boot_id;
    uint8_t status;
    uint8_t dropped;  // 1 = 오래된·중복 번호라 버림
    int64_t age_ns;
    int64_t recv_mono_ns;
  };

  void on_message(const intelion_interfaces::msg::ControlCommand& msg);
  void logger_main();

  Seqlock<AiCommandSnapshot> buffer_;
  rclcpp::Subscription<intelion_interfaces::msg::ControlCommand>::SharedPtr sub_;

  // 새 명령 판정 기준 (콜백 스레드만 사용)
  bool has_last_ = false;
  uint32_t last_boot_id_ = 0;
  uint64_t last_seq_ = 0;

  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> boot_changes_{0};
  std::atomic<uint64_t> log_overflow_{0};  // 링버퍼가 가득 차서 기록 못 한 줄 수

  SpscRing<RxLogRow, kLogRingSize> log_ring_;
  std::string log_path_;
  std::FILE* log_file_ = nullptr;
  std::atomic<bool> stop_logger_{false};
  std::thread logger_thread_;
};

}  // namespace rt_control
