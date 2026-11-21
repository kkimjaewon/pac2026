#include "rt_control/cycle_tracer.hpp"

#include <chrono>
#include <string>

#include "rt_control/gate.hpp"
#include "rt_control/safety_mode.hpp"

namespace rt_control {

const char* const kTraceCsvHeader =
    "cycle,wake_mono_ns,compute_ns,overrun,mode,used_ai_seq,weight,goal_x,goal_y,goal_z,y_x,y_y,y_z,"
    "command_seq,ai_recv_mono_ns,ai_age_ns,ai_apply_mono_ns,reject_reason,"
    "jitter_ns,mode_reason,speed,has_used,arrived,tcp_error_m,used_ai_boot_id,rejected_wait";

namespace {

const char* trace_mode_name(uint8_t m) {
  switch (m) {
    case TRACE_RUN: return "RUN";
    case TRACE_HOLD: return "HOLD";
    case TRACE_DECEL_STOP: return "DECEL_STOP";
    case TRACE_FAULT: return "FAULT";
    case TRACE_INIT: return "INIT";
    default: return "?";
  }
}

// 이유 비트 -> "EXPIRED|RANGE" (CSV 쉼표와 섞이지 않게 |로 이음). 없으면 빈 칸
void reason_text(uint32_t reasons, char* out, size_t size) {
  size_t n = 0;
  out[0] = '\0';
  for (int i = 0; i < kSafetyReasonCount && n + 1 < size; ++i) {
    if (!((reasons >> i) & 1u)) continue;
    const int w = std::snprintf(out + n, size - n, "%s%s", n ? "|" : "", kSafetyReasonNames[i]);
    if (w < 0) break;
    n += static_cast<size_t>(w);
  }
}

}  // namespace

int trace_row_to_csv(const TraceRow& r, char* buf, size_t size) {
  char reason[160];
  reason_text(r.mode_reason, reason, sizeof(reason));
  return std::snprintf(buf, size,
                       "%llu,%lld,%lld,%u,%s,%llu,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                       "%llu,%lld,%lld,%lld,0x%02x,"
                       "%lld,%s,%.6f,%u,%u,%.9f,%u,%u\n",
                       (unsigned long long)r.cycle, (long long)r.wake_mono_ns, (long long)r.compute_ns,
                       r.overrun, trace_mode_name(r.mode), (unsigned long long)r.used_ai_seq, r.weight,
                       r.goal[0], r.goal[1], r.goal[2], r.y[0], r.y[1], r.y[2],
                       (unsigned long long)r.command_seq, (long long)r.ai_recv_mono_ns, (long long)r.ai_age_ns,
                       (long long)r.ai_apply_mono_ns, r.reject_reason,
                       (long long)r.jitter_ns, reason, r.speed, r.has_used, r.arrived, r.tcp_error_m,
                       r.used_ai_boot_id, r.rejected_wait);
}

CycleTracer::CycleTracer() : ring_(std::make_unique<SpscRing<TraceRow, kRingSize>>()) {}

CycleTracer::~CycleTracer() { close(); }

bool CycleTracer::open(const std::string& path) {
  file_ = std::fopen(path.c_str(), "w");
  if (!file_) return false;
  std::fprintf(file_, "%s\n", kTraceCsvHeader);
  stop_.store(false, std::memory_order_relaxed);
  writer_ = std::thread(&CycleTracer::writer_main, this);
  return true;
}

void CycleTracer::close() {
  stop_.store(true, std::memory_order_relaxed);
  if (writer_.joinable()) writer_.join();
  if (file_) {
    std::fclose(file_);
    file_ = nullptr;
  }
}

void CycleTracer::writer_main() {
  TraceRow row;
  char line[512];
  for (;;) {
    // stop 신호를 먼저 읽고 나서 비워야, 종료 직전에 들어온 줄까지 다 저장됨
    const bool stopping = stop_.load(std::memory_order_relaxed);
    uint64_t n = 0;
    while (ring_->pop(row)) {
      const int len = trace_row_to_csv(row, line, sizeof(line));
      if (len > 0) std::fwrite(line, 1, static_cast<size_t>(len), file_);
      ++n;
    }
    if (n) {
      std::fflush(file_);
      written_.fetch_add(n, std::memory_order_relaxed);
    }
    if (stopping) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(kFlushIntervalMs));
  }
}

}  // namespace rt_control
