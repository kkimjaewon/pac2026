// 1ms 고정 주기 실시간 루프 (B 계층 구현 순서 1단계)
//
// clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)으로 "절대 시각"까지 잠들었다가
// 깨어나서 콜백을 한 번 호출하는 것을 반복한다. 상대 시간(1ms 후)이 아니라
// 절대 시각(시작 + n × 1ms)을 쓰기 때문에 오차가 쌓이지 않는다.
//
// 루프 안(run 내부, 콜백 호출 사이)에서는 메모리 할당·printf·파일 쓰기를 하지 않는다.
// 콜백 안에서도 같은 규칙을 지켜야 한다.
#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace rt_control {

// CLOCK_MONOTONIC 현재 시각 [ns]
int64_t now_ns();

// cpu와 같은 물리 코어를 쓰는 CPU 번호들 (자기 자신 포함, 예: 6 -> {6, 7}). 읽을 수 없으면 {cpu}
std::vector<int> thread_siblings(int cpu);

// 이 프로세스에 지금 있는 모든 스레드를 cpu와 그 짝꿍 코어에서 내보낸다 (나머지 CPU 어디서든 돌게).
// 호출한 스레드도 포함되므로, 이후에 이 스레드에서 만든 스레드(DDS·발행·기록)도 그 코어를 피한다.
// 1ms 루프 스레드를 만들기 전에 메인 스레드에서 부를 것. 돌려주는 값 = 옮긴 스레드 수 (실패 -1)
int keep_threads_off_cpu(int cpu);

// 실시간 설정 옵션. 기본값은 모두 꺼짐 (일반 커널·일반 권한에서 그대로 돌게)
struct RtOptions {
  bool lock_memory = false;   // mlockall(MCL_CURRENT | MCL_FUTURE) + 스택 미리 건드리기
  bool sched_fifo = false;    // SCHED_FIFO 실시간 스케줄링
  int fifo_priority = 80;     // SCHED_FIFO 우선순위 (1~99). 발행 스레드(bridge_prio)보다 높게
  int cpu = -1;               // 고정할 CPU 번호. -1이면 고정하지 않음
};

struct LoopConfig {
  int64_t period_ns = 1'000'000;  // 주기 1ms
  RtOptions rt;
};

// 매 주기 콜백에 넘겨주는 정보
struct CycleInfo {
  uint64_t cycle;        // 주기 번호 (0부터)
  int64_t scheduled_ns;  // 이번 주기에 깨어나야 했던 예정 시각
  int64_t wakeup_ns;     // 실제로 깨어난 시각
  uint32_t overrun;      // 이번 주기 직전에 건너뛴 주기 수 (정상 0)
};

// 콜백: true를 돌려주면 계속, false면 루프 종료.
// std::function 대신 함수 포인터 + user 포인터를 써서 할당이 생기지 않게 한다.
using CycleCallback = bool (*)(const CycleInfo& info, void* user);

class RtControlLoop {
 public:
  explicit RtControlLoop(const LoopConfig& config);

  // 실시간 설정을 적용한다. 실패한 항목은 경고만 출력하고 계속한다.
  // run()을 호출할 스레드에서 run() 전에 호출할 것 (스케줄링·CPU 고정은 호출한 스레드에 적용됨).
  void apply_rt_settings();

  // 루프 실행. 콜백이 false를 돌려주거나 request_stop()이 불리면 돌아온다.
  void run(CycleCallback callback, void* user);

  // 다른 스레드나 시그널 핸들러에서 종료 요청 (lock-free atomic이라 시그널 핸들러에서도 안전)
  void request_stop() { stop_requested_.store(true, std::memory_order_relaxed); }

  // 늦게 깨어나 다음 예정 시각까지 지나쳐 버려서 건너뛴 주기 수 (다른 스레드에서 읽어도 됨)
  uint64_t overrun_count() const { return overruns_.load(std::memory_order_relaxed); }

 private:
  LoopConfig config_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<uint64_t> overruns_{0};
};

}  // namespace rt_control
