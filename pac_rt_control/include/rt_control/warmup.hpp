// 시작 준비(warmup): INIT -> RUN으로 넘어가도 되는지 판단
//
// B를 켠 직후에는 DDS 연결·메모리 잠금 등으로 C 상태가 띄엄띄엄 오고 발행 스레드도 늦게 깬다.
// 이때 바로 RUN으로 가면 첫 수십 ms 안에 STATE_TIMEOUT으로 DECEL_STOP이 한 번 난다.
// 그래서 아래 조건이 모두 맞을 때까지 INIT에 머문다 (INIT에서는 safety 판단을 하지 않음).
//   WARM_STATES     : 새 HalState를 states번 이상 받음 (0 = 조건 끔)
//   WARM_PUBLISH    : 발행 스레드가 stable_ns 동안 overrun 없이 계속 깨어남
//   WARM_SUBSCRIBER : /hal/final_command 구독자가 1개 이상 연결됨
//   WARM_HOLD       : INIT 유지 명령(init_hold.hpp)을 hold_ns 동안 끊김·overrun 없이 계속 발행함
//                     (첫 publish()가 DDS 안에서 수백 ms 막히는 비용을 RUN 전에 치르게. 0 = 조건 끔)
// max_ns가 지나도 준비가 안 되면 timed_out()이 true (노드가 경고. 시작은 계속 기다림)
//
// 시계 호출·로그·메모리 할당이 없다 (1ms 루프에서 부름).
#pragma once

#include <cstdint>

namespace rt_control {

// [제안] 바꿀 곳은 여기 한 곳 + ROS 파라미터 warmup_states, warmup_stable_ms, warmup_max_s
constexpr int kDefaultWarmupStates = 5;
constexpr int64_t kDefaultWarmupStableNs = 100'000'000;  // 100ms
constexpr int64_t kDefaultWarmupMaxNs = 3'000'000'000;   // 3초
constexpr int64_t kDefaultWarmupHoldNs = 100'000'000;    // 100ms. ROS 파라미터 warmup_hold_ms

enum WarmupFlag : uint32_t {
  WARM_STATES = 1u << 0,
  WARM_PUBLISH = 1u << 1,
  WARM_SUBSCRIBER = 1u << 2,
  WARM_HOLD = 1u << 3,
};
constexpr uint32_t kWarmupAll = WARM_STATES | WARM_PUBLISH | WARM_SUBSCRIBER | WARM_HOLD;

struct WarmupConfig {
  int states = kDefaultWarmupStates;
  int64_t stable_ns = kDefaultWarmupStableNs;
  int64_t max_ns = kDefaultWarmupMaxNs;
  int64_t hold_ns = kDefaultWarmupHoldNs;
  int64_t publish_period_ns = 1'000'000;  // 발행 스레드 주기 (stable_ns 동안 최소 절반은 깨어나야 함)
};

struct WarmupInput {
  int64_t now_ns;
  bool new_state;             // 이번 주기에 새 HalState가 들어왔는지
  uint64_t publish_ticks;     // 발행 스레드가 깨어난 누적 수
  uint64_t publish_overruns;  // 발행 스레드 overrun 누적 수
  uint64_t subscribers;       // /hal/final_command 구독자 수
  bool holding;               // INIT 유지 명령을 발행 중인지 (지난 주기까지 끊김 없이)
  uint64_t published;         // 발행 스레드가 실제로 내보낸 누적 수
};

class StartupWarmup {
 public:
  StartupWarmup() = default;
  explicit StartupWarmup(const WarmupConfig& config) : config_(config) {}

  // 매 INIT 주기 호출. 돌려주는 값 = 지금 맞은 조건 (WarmupFlag 비트)
  uint32_t update(const WarmupInput& in);

  bool ready() const { return met_ == kWarmupAll; }
  uint32_t met() const { return met_; }
  int states_seen() const { return states_; }
  // 첫 update부터 지난 시간 (아직 update 전이면 0)
  int64_t elapsed_ns(int64_t now_ns) const { return started_ ? now_ns - start_ns_ : 0; }
  bool timed_out(int64_t now_ns) const { return !ready() && elapsed_ns(now_ns) > config_.max_ns; }
  const WarmupConfig& config() const { return config_; }

 private:
  WarmupConfig config_;
  bool started_ = false;
  int64_t start_ns_ = 0;
  int states_ = 0;
  // 발행 안정 판단: overrun이 마지막으로 바뀐 시각과 그때의 깨어난 수
  uint64_t last_overruns_ = 0;
  int64_t stable_since_ns_ = 0;
  uint64_t ticks_at_stable_ = 0;
  // 유지 명령 안정 판단: 유지 명령이 (다시) 시작됐거나 overrun이 난 시각과 그때의 발행 수
  bool was_holding_ = false;
  int64_t hold_since_ns_ = 0;
  uint64_t published_at_hold_ = 0;
  uint32_t met_ = 0;
};

}  // namespace rt_control
