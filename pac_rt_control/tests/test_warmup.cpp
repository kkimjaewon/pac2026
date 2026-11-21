// warmup(시작 준비) 단위 테스트: 준비가 끝나기 전에는 INIT에 머묾 (= STATE_TIMEOUT 등 safety 판단 안 함)
#include <gtest/gtest.h>

#include "rt_control/warmup.hpp"

namespace {

using rt_control::StartupWarmup;
using rt_control::WarmupConfig;
using rt_control::WarmupInput;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kT0 = 50'000 * kMs;

// 1ms마다 부르는 흉내: 발행 스레드는 매 ms 깨어남, HalState는 state_every_ms마다 하나
struct Sim {
  StartupWarmup w;
  int64_t t = kT0;
  uint64_t ticks = 0;
  uint64_t overruns = 0;
  uint64_t subscribers = 1;
  int state_every_ms = 4;  // 가짜 C 250Hz
  bool holding = true;     // INIT 유지 명령 발행 중
  uint64_t published = 0;  // 발행 수 (유지 명령 중이면 매 ms +1)
  int n = 0;
  explicit Sim(const WarmupConfig& c = WarmupConfig{}) : w(c) {}
  uint32_t step() {
    t += kMs;
    ++ticks;
    ++n;
    if (holding) ++published;
    return w.update(
        WarmupInput{t, state_every_ms > 0 && n % state_every_ms == 0, ticks, overruns, subscribers, holding, published});
  }
  // 준비될 때까지 돌린 시간 [ms] (limit_ms 안에 안 되면 -1)
  int run_until_ready(int limit_ms) {
    for (int i = 1; i <= limit_ms; ++i) {
      step();
      if (w.ready()) return i;
    }
    return -1;
  }
};

TEST(Warmup, Defaults) {
  const WarmupConfig c;
  EXPECT_EQ(c.states, 5);
  EXPECT_EQ(c.stable_ns, 100 * kMs);
  EXPECT_EQ(c.max_ns, 3000 * kMs);
}

// 정상: 상태 5개(20ms)와 발행 100ms 안정이 모두 되면 준비 끝 (그 전에는 아님).
// 시간은 첫 update부터 셈 -> 101번째 주기
TEST(Warmup, ReadyAfterStatesAndStablePublish) {
  Sim s;
  for (int i = 0; i < 100; ++i) {
    s.step();
    EXPECT_FALSE(s.w.ready()) << i;
  }
  EXPECT_EQ(s.w.states_seen(), 5);
  EXPECT_EQ(s.w.met(), rt_control::WARM_STATES | rt_control::WARM_SUBSCRIBER);  // 발행·유지 명령 100ms 전
  s.step();
  EXPECT_TRUE(s.w.ready());
}

// B 시작 직후 상태가 늦게 오면(문제 상황: 첫 상태 뒤 75ms 안에 STATE_TIMEOUT) 5개가 될 때까지 준비 안 됨
TEST(Warmup, NotReadyUntilEnoughStates) {
  Sim s;
  s.state_every_ms = 0;  // 상태 안 옴
  for (int i = 0; i < 500; ++i) s.step();
  EXPECT_FALSE(s.w.ready());
  EXPECT_EQ(s.w.met() & rt_control::WARM_STATES, 0u);
  s.state_every_ms = 30;  // 30ms마다 하나: 5개는 150ms 뒤
  int ready_at = s.run_until_ready(1000);
  EXPECT_GE(ready_at, 120);
  EXPECT_LE(ready_at, 150);
}

// 발행 overrun이 계속 나는 동안은 준비 안 됨, 멈추고 stable_ns 뒤에 준비
TEST(Warmup, PublishOverrunsRestartStableWindow) {
  Sim s;
  for (int i = 0; i < 300; ++i) {
    if (i % 10 == 0) ++s.overruns;  // 시작 직후 322번 같은 overrun
    s.step();
    EXPECT_FALSE(s.w.ready());
  }
  const int ready_at = s.run_until_ready(1000);
  EXPECT_GE(ready_at, 91);  // 마지막 overrun(i=290)에서 100ms
  EXPECT_LE(ready_at, 101);
}

// 발행 스레드가 아예 안 깨어나면(ticks 그대로) 안정으로 보지 않음
TEST(Warmup, PublishThreadNotRunning) {
  Sim s;
  for (int i = 0; i < 500; ++i) {
    s.t += kMs;
    ++s.n;
    s.w.update(WarmupInput{s.t, s.n % 4 == 0, 0, 0, 1, true, 0});
  }
  EXPECT_EQ(s.w.met() & rt_control::WARM_PUBLISH, 0u);
  EXPECT_FALSE(s.w.ready());
}

// 구독자가 없으면 준비 안 됨, 최대 시간이 지나면 timed_out (경고용)
TEST(Warmup, NoSubscriberTimesOut) {
  WarmupConfig c;
  c.max_ns = 500 * kMs;
  Sim s(c);
  s.subscribers = 0;
  for (int i = 0; i < 501; ++i) s.step();
  EXPECT_FALSE(s.w.ready());
  EXPECT_FALSE(s.w.timed_out(s.t));  // 첫 update부터 딱 500ms (넘어야 함)
  s.step();
  EXPECT_TRUE(s.w.timed_out(s.t));
  EXPECT_EQ(s.w.met(), rt_control::WARM_STATES | rt_control::WARM_PUBLISH | rt_control::WARM_HOLD);
  s.subscribers = 1;
  s.step();
  EXPECT_TRUE(s.w.ready());
  EXPECT_FALSE(s.w.timed_out(s.t));
}

// warmup_states 0 = 상태 수 조건 끔
TEST(Warmup, ZeroStatesDisablesStateCondition) {
  WarmupConfig c;
  c.states = 0;
  Sim s(c);
  s.state_every_ms = 0;
  EXPECT_EQ(s.run_until_ready(1000), 101);
}

// ---------------- 유지 명령 조건 (WARM_HOLD) ----------------
// 유지 명령을 발행하지 않으면 다른 조건이 다 맞아도 준비 안 됨
TEST(WarmupHold, NotReadyWithoutHoldCommands) {
  Sim s;
  s.holding = false;
  for (int i = 0; i < 500; ++i) s.step();
  EXPECT_FALSE(s.w.ready());
  EXPECT_EQ(s.w.met(), rt_control::WARM_STATES | rt_control::WARM_PUBLISH | rt_control::WARM_SUBSCRIBER);
  // 유지 명령을 시작하면 그때부터 100ms 뒤
  s.holding = true;
  EXPECT_EQ(s.run_until_ready(1000), 101);
}

// 첫 유지 명령 publish()가 막혀 overrun이 몰리면(측정 230ms) 그 뒤 100ms를 다시 세야 준비
TEST(WarmupHold, OverrunDuringHoldRestarts) {
  Sim s;
  for (int i = 0; i < 50; ++i) s.step();
  // 230ms 동안 발행 스레드가 막힘: 깨어남·발행 없음, 끝나고 overrun 230
  for (int i = 0; i < 230; ++i) {
    s.t += kMs;
    ++s.n;
    s.w.update(rt_control::WarmupInput{s.t, s.n % 4 == 0, s.ticks, s.overruns, 1, true, s.published});
    EXPECT_FALSE(s.w.ready());
  }
  s.overruns += 230;
  EXPECT_EQ(s.run_until_ready(1000), 101);
}

// 발행 스레드가 실제로 내보내지 못하면(published 그대로) 시간이 지나도 준비 안 됨
TEST(WarmupHold, RequiresActualPublishes) {
  Sim s;
  for (int i = 0; i < 300; ++i) {
    s.t += kMs;
    ++s.n;
    ++s.ticks;
    s.w.update(rt_control::WarmupInput{s.t, s.n % 4 == 0, s.ticks, 0, 1, true, 0});
  }
  EXPECT_EQ(s.w.met() & rt_control::WARM_HOLD, 0u);
}

// 유지 명령이 끊겼다 다시 시작하면 처음부터
TEST(WarmupHold, InterruptionRestarts) {
  Sim s;
  for (int i = 0; i < 90; ++i) s.step();
  s.holding = false;
  s.step();
  s.holding = true;
  for (int i = 0; i < 100; ++i) {  // 다시 시작한 주기부터 0ms, 99ms까지는 아직
    s.step();
    EXPECT_FALSE(s.w.ready()) << i;
  }
  s.step();
  EXPECT_TRUE(s.w.ready());
}

TEST(WarmupHold, DefaultAndZeroDisables) {
  EXPECT_EQ(WarmupConfig{}.hold_ns, 100 * kMs);
  WarmupConfig c;
  c.hold_ns = 0;
  Sim s(c);
  s.holding = false;
  EXPECT_EQ(s.run_until_ready(1000), 101);
}

}  // namespace
