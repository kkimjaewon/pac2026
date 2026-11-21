// rt_control_loop 지터 측정 데모
//
// 사용법: rt_loop_demo [--seconds N] [--mlock] [--fifo] [--prio P] [--cpu C]
//   --seconds N  실행 시간(초), 기본 10
//   --mlock      mlockall 켜기
//   --fifo       SCHED_FIFO 켜기 (권한이 없으면 경고 후 일반 스케줄링으로 계속)
//   --prio P     SCHED_FIFO 우선순위, 기본 80
//   --cpu C      CPU C번에 고정
//
// 측정값
//   주기     : 이번에 깨어난 시각 - 지난번에 깨어난 시각 (목표 1000us)
//   지터     : 실제 깨어난 시각 - 예정 시각 (늦게 깬 정도, 0에 가까울수록 좋음)
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "rt_control/rt_control_loop.hpp"

namespace {

constexpr int64_t kJitterLimitNs = 100'000;  // 목표 지터 ±100us

rt_control::RtControlLoop* g_loop = nullptr;

void on_signal(int) {
  if (g_loop) g_loop->request_stop();
}

// 콜백 안에서는 할당·출력 없이 숫자만 누적
struct Stats {
  uint64_t total_cycles = 0;  // 이 값에 도달하면 종료
  uint64_t cycles = 0;
  int64_t prev_wakeup_ns = 0;
  int64_t period_sum_ns = 0;
  uint64_t period_count = 0;
  int64_t period_min_ns = INT64_MAX;
  int64_t period_max_ns = 0;
  int64_t jitter_sum_ns = 0;
  int64_t jitter_max_ns = 0;
  uint64_t jitter_over_limit = 0;
};

bool on_cycle(const rt_control::CycleInfo& info, void* user) {
  Stats& s = *static_cast<Stats*>(user);

  const int64_t jitter = info.wakeup_ns - info.scheduled_ns;
  s.jitter_sum_ns += jitter;
  if (jitter > s.jitter_max_ns) s.jitter_max_ns = jitter;
  if (jitter > kJitterLimitNs) ++s.jitter_over_limit;

  if (s.cycles > 0) {
    const int64_t period = info.wakeup_ns - s.prev_wakeup_ns;
    s.period_sum_ns += period;
    ++s.period_count;
    if (period < s.period_min_ns) s.period_min_ns = period;
    if (period > s.period_max_ns) s.period_max_ns = period;
  }
  s.prev_wakeup_ns = info.wakeup_ns;

  ++s.cycles;
  return s.cycles < s.total_cycles;
}

double us(int64_t ns) { return static_cast<double>(ns) / 1000.0; }

}  // namespace

int main(int argc, char** argv) {
  rt_control::LoopConfig config;
  double seconds = 10.0;

  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    const bool has_value = i + 1 < argc;
    if (!std::strcmp(a, "--seconds") && has_value) {
      seconds = std::atof(argv[++i]);
    } else if (!std::strcmp(a, "--mlock")) {
      config.rt.lock_memory = true;
    } else if (!std::strcmp(a, "--fifo")) {
      config.rt.sched_fifo = true;
    } else if (!std::strcmp(a, "--prio") && has_value) {
      config.rt.fifo_priority = std::atoi(argv[++i]);
    } else if (!std::strcmp(a, "--cpu") && has_value) {
      config.rt.cpu = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr,
                   "사용법: %s [--seconds N] [--mlock] [--fifo] [--prio P] [--cpu C]\n",
                   argv[0]);
      return 1;
    }
  }

  rt_control::RtControlLoop loop(config);
  g_loop = &loop;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  loop.apply_rt_settings();
  std::fflush(stderr);

  Stats stats;
  stats.total_cycles =
      static_cast<uint64_t>(seconds * 1e9 / static_cast<double>(config.period_ns));

  std::printf("주기 %.0f us로 %.1f초 실행 중... (Ctrl+C로 중단)\n", us(config.period_ns),
              seconds);
  std::fflush(stdout);

  loop.run(on_cycle, &stats);

  // ---- 루프가 끝난 뒤에만 출력 ----
  if (stats.period_count == 0) {
    std::printf("측정된 주기가 없습니다.\n");
    return 0;
  }
  std::printf("\n===== 결과 =====\n");
  std::printf("실행한 주기 수        : %llu\n", (unsigned long long)stats.cycles);
  std::printf("주기 평균             : %.3f us (목표 %.0f us)\n",
              us(stats.period_sum_ns) / static_cast<double>(stats.period_count),
              us(config.period_ns));
  std::printf("주기 최소 / 최대      : %.3f / %.3f us\n", us(stats.period_min_ns),
              us(stats.period_max_ns));
  std::printf("지터 평균             : %.3f us\n",
              us(stats.jitter_sum_ns) / static_cast<double>(stats.cycles));
  std::printf("최대 지터             : %.3f us\n", us(stats.jitter_max_ns));
  std::printf("지터 %.0fus 초과 횟수  : %llu (%.4f%%)\n", us(kJitterLimitNs),
              (unsigned long long)stats.jitter_over_limit,
              100.0 * static_cast<double>(stats.jitter_over_limit) /
                  static_cast<double>(stats.cycles));
  std::printf("건너뛴 주기(overrun)  : %llu\n", (unsigned long long)loop.overrun_count());
  return 0;
}
