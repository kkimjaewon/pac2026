// 한 생산자 · 한 소비자용 고정 크기 링버퍼 (lock-free)
//
// 기록을 파일로 저장할 때 사용한다. 생산자(콜백·제어 루프)는 push만 하고,
// 버퍼가 가득 차면 기다리지 않고 false를 돌려준다(그 기록은 버려짐).
// 소비자(파일 저장 스레드)가 pop으로 꺼내 파일에 쓴다. 동적 할당 없음.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace rt_control {

template <typename T, size_t N>
class SpscRing {
  static_assert(N > 0 && (N & (N - 1)) == 0, "SpscRing: N은 2의 거듭제곱이어야 함");

 public:
  // 생산자 스레드 전용. 가득 차 있으면 false
  bool push(const T& value) {
    const uint64_t h = head_.load(std::memory_order_relaxed);
    const uint64_t t = tail_.load(std::memory_order_acquire);
    if (h - t == N) return false;
    buf_[h & (N - 1)] = value;
    head_.store(h + 1, std::memory_order_release);
    return true;
  }

  // 소비자 스레드 전용. 비어 있으면 false
  bool pop(T& out) {
    const uint64_t t = tail_.load(std::memory_order_relaxed);
    const uint64_t h = head_.load(std::memory_order_acquire);
    if (t == h) return false;
    out = buf_[t & (N - 1)];
    tail_.store(t + 1, std::memory_order_release);
    return true;
  }

 private:
  // 배열을 맨 앞에 둠 (atomic 뒤에 두면 GCC 13이 배열 쓰기를 atomic 넘침으로 잘못 경고함)
  T buf_[N];
  // head와 tail을 다른 캐시 라인에 두어 두 스레드가 서로 캐시를 빼앗지 않게 함
  alignas(64) std::atomic<uint64_t> head_{0};
  alignas(64) std::atomic<uint64_t> tail_{0};
};

}  // namespace rt_control
