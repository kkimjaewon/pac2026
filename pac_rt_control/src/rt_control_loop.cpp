#include "rt_control/rt_control_loop.hpp"

#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace rt_control {

static_assert(std::atomic<bool>::is_always_lock_free,
              "request_stop()을 시그널 핸들러에서 쓰려면 lock-free여야 함");

namespace {

constexpr int64_t kNsPerSec = 1'000'000'000;
// mlockall 뒤에 미리 건드려 둘 스택 크기. 루프 중 스택이 처음 늘어날 때 생기는
// 페이지 폴트(수십 us 지연)를 시작 전에 다 일으켜 두기 위함
constexpr size_t kPrefaultStackBytes = 512 * 1024;

timespec to_timespec(int64_t ns) {
  timespec ts;
  ts.tv_sec = static_cast<time_t>(ns / kNsPerSec);
  ts.tv_nsec = static_cast<long>(ns % kNsPerSec);
  return ts;
}

// 최적화로 지워지지 않게 noinline + volatile
__attribute__((noinline)) void prefault_stack() {
  [[maybe_unused]] volatile unsigned char buf[kPrefaultStackBytes];
  for (size_t i = 0; i < kPrefaultStackBytes; i += 4096) buf[i] = 0;
}

}  // namespace

int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

std::vector<int> thread_siblings(int cpu) {
  std::vector<int> out;
  // 예: "6-7" 또는 "6,7"
  std::ifstream f("/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/thread_siblings_list");
  std::string text;
  if (std::getline(f, text)) {
    std::stringstream ss(text);
    std::string part;
    while (std::getline(ss, part, ',')) {
      const size_t dash = part.find('-');
      const int lo = std::atoi(part.substr(0, dash).c_str());
      const int hi = dash == std::string::npos ? lo : std::atoi(part.substr(dash + 1).c_str());
      for (int i = lo; i <= hi; ++i) out.push_back(i);
    }
  }
  if (std::find(out.begin(), out.end(), cpu) == out.end()) out.push_back(cpu);
  return out;
}

int keep_threads_off_cpu(int cpu) {
  cpu_set_t set;
  if (sched_getaffinity(0, sizeof(set), &set) != 0) return -1;
  for (int c : thread_siblings(cpu)) CPU_CLR(c, &set);
  if (CPU_COUNT(&set) == 0) return -1;  // 남는 CPU가 없으면 아무것도 안 함

  DIR* dir = opendir("/proc/self/task");
  if (!dir) return -1;
  int moved = 0;
  while (dirent* e = readdir(dir)) {
    const pid_t tid = static_cast<pid_t>(std::atoi(e->d_name));
    if (tid > 0 && sched_setaffinity(tid, sizeof(set), &set) == 0) ++moved;
  }
  closedir(dir);
  return moved;
}

RtControlLoop::RtControlLoop(const LoopConfig& config) : config_(config) {}

void RtControlLoop::apply_rt_settings() {
  const RtOptions& rt = config_.rt;

  if (rt.lock_memory) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      prefault_stack();
      std::printf("[rt] mlockall: 성공\n");
    } else {
      std::fprintf(stderr, "[rt] 경고: mlockall 실패 (%s), 메모리 잠금 없이 계속\n",
                   std::strerror(errno));
    }
  }

  if (rt.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(rt.cpu, &set);
    // pthread_* 함수는 errno 대신 오류 번호를 반환값으로 돌려줌
    int err = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (err == 0) {
      std::printf("[rt] CPU %d 고정: 성공\n", rt.cpu);
    } else {
      std::fprintf(stderr, "[rt] 경고: CPU %d 고정 실패 (%s), 고정 없이 계속\n", rt.cpu,
                   std::strerror(err));
    }
  }

  if (rt.sched_fifo) {
    sched_param param{};
    param.sched_priority = rt.fifo_priority;
    int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (err == 0) {
      std::printf("[rt] SCHED_FIFO 우선순위 %d: 성공\n", rt.fifo_priority);
    } else {
      std::fprintf(stderr, "[rt] 경고: SCHED_FIFO 설정 실패 (%s), 일반 스케줄링으로 계속\n",
                   std::strerror(err));
    }
  }
}

void RtControlLoop::run(CycleCallback callback, void* user) {
  // ---- 여기부터 루프 끝까지: 메모리 할당·printf·파일 쓰기 금지 ----
  const int64_t period = config_.period_ns;
  int64_t next = now_ns() + period;  // 첫 예정 시각
  uint64_t cycle = 0;
  uint32_t missed_before = 0;  // 이번 주기 직전에 건너뛴 주기 수

  while (!stop_requested_.load(std::memory_order_relaxed)) {
    const timespec ts = to_timespec(next);
    // 시그널로 깨면 EINTR. 절대 시각이라 같은 값으로 다시 자면 됨
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
      if (stop_requested_.load(std::memory_order_relaxed)) return;
    }

    const CycleInfo info{cycle, next, now_ns(), missed_before};
    missed_before = 0;
    if (!callback(info, user)) break;
    ++cycle;

    next += period;
    // 너무 늦어서 다음 예정 시각도 이미 지났다면, 밀린 주기를 몰아서 돌리지 않고 건너뜀
    // (몰아서 돌리면 1ms 간격이 아닌 연속 호출이 생김)
    const int64_t now = now_ns();
    if (now >= next) {
      const int64_t missed = (now - next) / period + 1;
      next += missed * period;
      overruns_.fetch_add(static_cast<uint64_t>(missed), std::memory_order_relaxed);
      missed_before = static_cast<uint32_t>(missed);
    }
  }
}

}  // namespace rt_control
