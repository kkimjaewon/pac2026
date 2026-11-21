// cycle_tracer: 1ms 루프의 매 주기 기록을 rt.csv로 저장 (계약 v0.3 인터페이스 5-2)
//
//   1ms 루프 --push(TraceRow)--> SpscRing --기록 스레드(50ms마다)--> rt.csv
//
// 루프는 숫자만 있는 TraceRow를 링버퍼에 넣기만 한다. 가득 차면 기다리지 않고 버리고 버린 수를 센다.
// 글자 변환(모드 이름 등)과 파일 쓰기는 모두 기록 스레드가 한다. 루프 안 할당·파일 쓰기·printf 없음.
// 기록 스레드는 open()을 부른 스레드의 CPU 설정을 물려받는다 (노드는 루프 코어를 비운 뒤에 open할 것).
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "rt_control/spsc_ring.hpp"

namespace rt_control {

// TraceRow.mode 값. 0~3은 SafetyMode와 같은 번호, INIT은 4
enum TraceMode : uint8_t {
  TRACE_RUN = 0,
  TRACE_HOLD = 1,
  TRACE_DECEL_STOP = 2,
  TRACE_FAULT = 3,
  TRACE_INIT = 4,
};

// 한 주기 기록 (숫자만, memcpy 가능)
struct TraceRow {
  uint64_t cycle;
  int64_t wake_mono_ns;      // 실제로 깨어난 시각
  int64_t compute_ns;        // 깨어난 시각부터 이번 주기 계산이 끝날 때까지
  uint32_t overrun;          // 이번 주기 직전에 건너뛴 주기 수 (정상 0)
  uint8_t mode;              // TraceMode
  uint8_t has_used;          // blend가 한 번이라도 반영했는지
  uint64_t used_ai_seq;      // 마지막으로 반영한 A seq
  double weight;             // 이번 주기에 반영했으면 w, 아니면 0
  double goal[3];            // blend 목표 (map [m])
  double y[3];               // limiter 출력 위치 (map [m])
  uint64_t command_seq;      // 이번 주기에 쓴 HalCommand 번호 (INIT이면 0)
  int64_t ai_recv_mono_ns;   // 지금 보고 있는 A 명령을 받은 시각 (없으면 0)
  int64_t ai_age_ns;         // 그 명령의 현재 나이 (gate 계산값)
  int64_t ai_apply_mono_ns;  // 이번 주기에 반영했으면 그 시각, 아니면 0
  uint32_t reject_reason;    // gate 차단 이유 비트 (GateReason)
  int64_t jitter_ns;         // 깨어난 시각 - 예정 시각
  uint32_t mode_reason;      // safety 모드에 들어온 이유 비트 (SafetyReason)
  double speed;              // limiter 속도 크기 [m/s]
  uint8_t arrived;           // 도착 판정 (|목표 - 측정 TCP| <= arrive_tol 이 arrive_hold_s 이상)
  double tcp_error_m;        // |blend 목표 - 측정 TCP| [m]. 측정이 없거나 INIT이면 NaN
  uint32_t used_ai_boot_id;  // 마지막으로 반영한 A 명령의 writer_boot_id (has_used가 false면 0)
  uint8_t rejected_wait;     // 1 = 거부된 목표 대기 중 (C가 거부한 AI 목표와 비슷한 새 목표는 반영 안 함)
};

// rt.csv 첫 줄. 계약 v0.3 필수 컬럼(이름·순서 그대로) 뒤에 추가 컬럼
extern const char* const kTraceCsvHeader;

// TraceRow -> CSV 한 줄 (끝에 줄바꿈 포함). 돌려주는 값 = 쓴 글자 수 (buf가 작으면 snprintf처럼 잘림)
// 기록 스레드에서만 부를 것 (snprintf 사용)
int trace_row_to_csv(const TraceRow& row, char* buf, size_t size);

class CycleTracer {
 public:
  static constexpr size_t kRingSize = 16384;  // 약 16초분. 디스크가 잠깐 멈춰도 버티게
  static constexpr int kFlushIntervalMs = 50;

  CycleTracer();
  ~CycleTracer();
  CycleTracer(const CycleTracer&) = delete;
  CycleTracer& operator=(const CycleTracer&) = delete;

  // path에 헤더를 쓰고 기록 스레드를 시작. 파일을 못 열면 false (push는 그냥 버림으로 셈)
  bool open(const std::string& path);

  // 1ms 루프 전용 (기다리지 않음). 가득 차면 버리고 dropped를 셈
  void push(const TraceRow& row) {
    if (!ring_->push(row)) dropped_.fetch_add(1, std::memory_order_relaxed);
  }

  // 남은 줄까지 다 쓰고 파일을 닫음 (여러 번 불러도 됨)
  void close();

  uint64_t written() const { return written_.load(std::memory_order_relaxed); }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  void writer_main();

  std::unique_ptr<SpscRing<TraceRow, kRingSize>> ring_;  // 크기가 커서(약 2.6MB) 힙에 둠
  std::FILE* file_ = nullptr;
  std::thread writer_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> written_{0};
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace rt_control
