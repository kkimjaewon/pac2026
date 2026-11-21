// 한 쓰기 스레드 · 한 읽기 스레드용 Seqlock (최신값 버퍼)
//
// 쓰는 쪽은 절대 기다리지 않고, 읽는 쪽도 기다리지 않는다.
//   - 쓰기: 번호를 홀수로 올림(쓰는 중) -> 데이터 복사 -> 번호를 짝수로 올림(완료)
//   - 읽기: 번호 읽기 -> 데이터 복사 -> 번호 다시 읽기.
//           두 번호가 같고 짝수면 "쓰는 도중에 읽지 않았다"는 뜻이라 성공.
//
// 데이터는 std::atomic<uint64_t> 칸에 나눠 담는다. 쓰기와 읽기가 겹쳐도
// C++ 규칙상 데이터 경합(undefined behavior)이 생기지 않게 하기 위함.
// 동적 할당 없음. T는 memcpy로 복사할 수 있는 타입(trivially copyable)이어야 한다.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace rt_control {

template <typename T>
class Seqlock {
  static_assert(std::is_trivially_copyable_v<T>, "Seqlock<T>: T는 memcpy 가능한 타입이어야 함");

 public:
  static constexpr int kDefaultMaxTries = 3;  // 읽기 재시도 횟수 (CLAUDE.md: 최대 3회)

  // 쓰기 스레드 한 곳에서만 호출할 것
  void write(const T& value) {
    uint64_t tmp[kWords] = {};
    std::memcpy(tmp, &value, sizeof(T));

    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);  // 홀수 = 쓰는 중
    std::atomic_thread_fence(std::memory_order_release);
    for (size_t i = 0; i < kWords; ++i) words_[i].store(tmp[i], std::memory_order_relaxed);
    seq_.store(s + 2, std::memory_order_release);  // 짝수 = 완료
  }

  // 읽기 스레드 한 곳에서 호출. 최대 max_tries번 시도한다.
  // 성공하면 out에 값을 넣고 true, 모두 실패하면 out을 건드리지 않고 false.
  bool try_read(T& out, int max_tries = kDefaultMaxTries) const {
    for (int attempt = 0; attempt < max_tries; ++attempt) {
      const uint32_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u) continue;  // 쓰는 중

      uint64_t tmp[kWords];
      for (size_t i = 0; i < kWords; ++i) tmp[i] = words_[i].load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);

      const uint32_t s2 = seq_.load(std::memory_order_relaxed);
      if (s1 == s2) {
        std::memcpy(&out, tmp, sizeof(T));
        return true;
      }
    }
    return false;
  }

  // 지금까지 write()가 한 번이라도 불렸는지
  bool ever_written() const { return seq_.load(std::memory_order_acquire) != 0; }

 private:
  static constexpr size_t kWords = (sizeof(T) + 7) / 8;
  std::atomic<uint32_t> seq_{0};
  std::atomic<uint64_t> words_[kWords] = {};
};

}  // namespace rt_control
