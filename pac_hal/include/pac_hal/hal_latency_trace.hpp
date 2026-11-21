// Copyright 2026 Jaewon Kim
#pragma once
#include <array>
#include <atomic>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include "pac_hal/hal_types.hpp"
namespace pac_hal
{
// 단일 C executor가 고정 크기 큐에 기록하고 별도 스레드가 CSV/ACK map을 처리한다.
// 기록이 밀려도 제어 콜백은 대기하지 않는다. 누락 개수는 trace_stats.yaml에 남긴다.
class HalLatencyTrace
{
public:
  explicit HalLatencyTrace(const Config & config);
  ~HalLatencyTrace();
  void command(const TraceRecord & record);
  void applied(const Receiver & receiver, int64_t received_ns);
  void error(const Error & event);
  void receiver_diagnostic(const std::string & payload, int64_t received_ns);
  void feedback_gap(const char * source, int64_t callback_ns, int64_t sample_ns,
    int64_t callback_gap_ns, int64_t sample_gap_ns);
private:
  struct Event {
    enum Kind {CommandRecord, Ack, JointGap, ReceiverGap, ErrorTransition, ReceiverDiagnostic} kind{CommandRecord};
    TraceRecord record{};
    uint64_t session{0}, seq{0};
    int64_t sample{0}, received{0}, callback_gap{0}, sample_gap{0};
    uint16_t error_code{0};bool active{false}, truncated{false};
    std::array<char,512> detail{};
  };
  static constexpr size_t kCapacity = 8192;
  void enqueue(const Event & event);
  void drain();
  void consume(const Event & event);
  void write(const TraceRecord & record, const char * result);
  std::array<Event, kCapacity> queue_{};
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
  std::atomic<bool> stopping_{false};
  std::atomic<uint64_t> dropped_{0};
  uint64_t enqueued_{0};  // Single producer; read only after worker join.
  std::thread worker_;
  std::ofstream file_, feedback_, errors_, diagnostics_;
  std::string stats_path_;
  std::map<uint64_t, TraceRecord> pending_;
  uint64_t session_, last_ack_{0};
  bool ack_seen_{false};
  // Worker-owned counters; read after join. A 10ms receiver status coalesces
  // command identities, so queue loss=0 does not mean every sent target has ACK.
  uint64_t sent_{0}, applied_{0}, rejected_{0}, superseded_{0};
  uint64_t pending_evicted_{0}, older_without_ack_{0}, unmatched_ack_{0}, invalid_ack_time_{0};
  uint64_t error_records_{0}, diagnostic_records_{0}, truncated_records_{0};
};
}
