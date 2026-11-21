// safety_mode 단위 테스트
#include <gtest/gtest.h>

#include "rt_control/gate.hpp"
#include "rt_control/hal_state_snapshot.hpp"
#include "rt_control/safety_mode.hpp"

namespace {

using rt_control::ResetResult;
using rt_control::SafetyConfig;
using rt_control::SafetyInput;
using rt_control::SafetyMode;
using rt_control::SafetyOutput;
using rt_control::SafetyStateMachine;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kSec = 1'000'000'000;
constexpr int64_t kT0 = 100 * kSec;

// 모두 정상인 입력: gate 통과(새 명령 아님), C 상태 1ms 전, READY, 오류 없음, 멈춰 있음
SafetyInput good(int64_t now) {
  SafetyInput in{};
  in.now_ns = now;
  in.gate_pass = true;
  in.gate_reasons = 0;
  in.is_new_command = false;
  in.c_has_state = true;
  in.c_state_age_ns = 1 * kMs;
  in.hal_state = rt_control::HAL_READY;
  in.c_error_mask = 0;
  in.reset_request = false;
  in.limiter_speed = 0.0;
  return in;
}

SafetyInput gate_block(int64_t now, uint32_t reasons, bool is_new = false) {
  SafetyInput in = good(now);
  in.gate_pass = false;
  in.gate_reasons = reasons;
  in.is_new_command = is_new;
  return in;
}

SafetyInput new_pass(int64_t now) {
  SafetyInput in = good(now);
  in.is_new_command = true;
  return in;
}

struct Fixture : ::testing::Test {
  SafetyStateMachine sm;
  int64_t t = kT0;
  void SetUp() override { sm.reset(t); }
  SafetyOutput step(SafetyInput in) {
    t = in.now_ns;
    return sm.update(in);
  }
  int64_t tick() { return t += kMs; }
};

// ---------------- 기본 ----------------
TEST_F(Fixture, AllGoodIsRun) {
  const SafetyOutput o = step(good(tick()));
  EXPECT_EQ(o.mode, SafetyMode::kRun);
  EXPECT_TRUE(o.allow_blend);
  EXPECT_FALSE(o.stop);
  EXPECT_FALSE(o.changed);
  EXPECT_EQ(o.reasons, 0u);
}

// ---------------- AI 쪽 ----------------
// gate가 EXPIRED·NO_DATA·STATUS·LOW_CONF로 막으면 HOLD (blend 안 함, 멈추지 않음)
TEST_F(Fixture, AiUnavailableGoesHold) {
  const struct {
    uint32_t gate;
    uint32_t reason;
  } cases[] = {{rt_control::GATE_EXPIRED, rt_control::SAFE_AI_EXPIRED},
               {rt_control::GATE_NO_DATA, rt_control::SAFE_AI_NO_DATA},
               {rt_control::GATE_STATUS, rt_control::SAFE_AI_STATUS},
               {rt_control::GATE_LOW_CONF, rt_control::SAFE_AI_LOW_CONF}};
  for (const auto& c : cases) {
    sm.reset(tick());
    const SafetyOutput o = step(gate_block(tick(), c.gate));
    EXPECT_EQ(o.mode, SafetyMode::kHold);
    EXPECT_TRUE(o.changed);
    EXPECT_EQ(o.prev_mode, SafetyMode::kRun);
    EXPECT_FALSE(o.allow_blend);
    EXPECT_FALSE(o.stop);
    EXPECT_EQ(o.reasons, c.reason);
  }
}

// 다른 gate 이유(NAN·TYPE, RANGE 한 번)만으로는 HOLD가 아님
TEST_F(Fixture, OtherGateReasonsStayRun) {
  EXPECT_EQ(step(gate_block(tick(), rt_control::GATE_TYPE, true)).mode, SafetyMode::kRun);
  EXPECT_EQ(step(gate_block(tick(), rt_control::GATE_RANGE, true)).mode, SafetyMode::kRun);
}

// HOLD -> RUN: 새 명령이 연속 resume_k(3)번 통과. 3번째 주기에 바로 blend 허용
TEST_F(Fixture, ResumeAfterKConsecutivePasses) {
  step(gate_block(tick(), rt_control::GATE_EXPIRED));
  EXPECT_FALSE(step(new_pass(tick())).allow_blend);  // 1
  EXPECT_EQ(step(good(tick())).mode, SafetyMode::kHold);  // 새 명령이 아닌 주기는 세지 않음
  EXPECT_FALSE(step(new_pass(tick())).allow_blend);  // 2
  const SafetyOutput o = step(new_pass(tick()));     // 3
  EXPECT_EQ(o.mode, SafetyMode::kRun);
  EXPECT_TRUE(o.allow_blend);
  EXPECT_TRUE(o.changed);
  EXPECT_FALSE(o.reset_limiter);  // HOLD에서는 멈춘 적이 없으니 reset 안 함
}

// 중간에 한 번 막히면 처음부터 다시 셈
TEST_F(Fixture, ResumeCountRestartsAfterBlock) {
  step(gate_block(tick(), rt_control::GATE_EXPIRED));
  step(new_pass(tick()));
  step(new_pass(tick()));
  step(gate_block(tick(), rt_control::GATE_RANGE, true));  // 새 명령이 막힘 (RANGE 1번이라 HOLD 조건은 아님)
  step(new_pass(tick()));
  EXPECT_EQ(step(new_pass(tick())).mode, SafetyMode::kHold);  // 2번째
  EXPECT_EQ(step(new_pass(tick())).mode, SafetyMode::kRun);   // 3번째
}

// HOLD 조건이 남아 있는 동안은 세지 않음 (예: 막힌 명령이 아직 최신)
TEST_F(Fixture, ResumeNeedsConditionGone) {
  step(gate_block(tick(), rt_control::GATE_STATUS));
  for (int i = 0; i < 5; ++i) step(gate_block(tick(), rt_control::GATE_STATUS, true));
  EXPECT_EQ(step(new_pass(tick())).mode, SafetyMode::kHold);
}

// 새 명령이 RANGE로 연속 range_reject_n(5)번 막히면 HOLD
TEST_F(Fixture, RangeRejectN) {
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(step(gate_block(tick(), rt_control::GATE_RANGE, true)).mode, SafetyMode::kRun);
    step(gate_block(tick(), rt_control::GATE_RANGE, false));  // 같은 명령이 계속 보이는 주기는 세지 않음
  }
  const SafetyOutput o = step(gate_block(tick(), rt_control::GATE_RANGE, true));
  EXPECT_EQ(o.mode, SafetyMode::kHold);
  EXPECT_EQ(o.reasons, rt_control::SAFE_AI_RANGE);
  // 정상 명령이 오면 RANGE 조건이 풀리고, 3번 통과 후 RUN
  step(new_pass(tick()));
  step(new_pass(tick()));
  EXPECT_EQ(step(new_pass(tick())).mode, SafetyMode::kRun);
}

// 중간에 정상 명령이 끼면 RANGE 횟수는 처음부터
TEST_F(Fixture, RangeCountResetsOnGoodCommand) {
  for (int i = 0; i < 4; ++i) step(gate_block(tick(), rt_control::GATE_RANGE, true));
  step(new_pass(tick()));
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(step(gate_block(tick(), rt_control::GATE_RANGE, true)).mode, SafetyMode::kRun);
  }
}

// ai_decel_after_s = 0(기본): HOLD가 아무리 길어도 DECEL_STOP으로 안 감
TEST_F(Fixture, HoldNeverDecelsWhenDisabled) {
  step(gate_block(tick(), rt_control::GATE_EXPIRED));
  const SafetyOutput o = step(gate_block(t + 100 * kSec, rt_control::GATE_EXPIRED));
  EXPECT_EQ(o.mode, SafetyMode::kHold);
  EXPECT_FALSE(o.stop);
  EXPECT_NEAR(o.mode_duration_ns, 100 * kSec, kMs);
}

// ai_decel_after_s = 2: HOLD가 2초를 넘으면 DECEL_STOP(HOLD_TIMEOUT)
TEST_F(Fixture, HoldDecelsAfterTimeout) {
  SafetyConfig c;
  c.ai_decel_after_ns = 2 * kSec;
  sm = SafetyStateMachine(c);
  sm.reset(t);
  const int64_t start = tick();
  step(gate_block(start, rt_control::GATE_EXPIRED));
  EXPECT_EQ(step(gate_block(start + 2 * kSec, rt_control::GATE_EXPIRED)).mode, SafetyMode::kHold);
  const SafetyOutput o = step(gate_block(start + 2 * kSec + kMs, rt_control::GATE_EXPIRED));
  EXPECT_EQ(o.mode, SafetyMode::kDecelStop);
  EXPECT_TRUE(o.stop);
  EXPECT_EQ(o.reasons, rt_control::SAFE_HOLD_TIMEOUT);
}

TEST_F(Fixture, VersionMismatchFaults) {
  const SafetyOutput o = step(gate_block(tick(), rt_control::GATE_VERSION, true));
  EXPECT_EQ(o.mode, SafetyMode::kFault);
  EXPECT_TRUE(o.stop);
  EXPECT_EQ(o.reasons, rt_control::SAFE_AI_VERSION);
}

// ---------------- C 쪽 ----------------
TEST_F(Fixture, StateTimeout) {
  SafetyInput in = good(tick());
  in.c_state_age_ns = 20 * kMs;  // 경계: 같으면 아직 괜찮음
  EXPECT_EQ(step(in).mode, SafetyMode::kRun);
  in = good(tick());
  in.c_state_age_ns = 20 * kMs + 1;
  const SafetyOutput o = step(in);
  EXPECT_EQ(o.mode, SafetyMode::kDecelStop);
  EXPECT_TRUE(o.stop);
  EXPECT_FALSE(o.allow_blend);
  EXPECT_EQ(o.reasons, rt_control::SAFE_C_STATE_TIMEOUT);
}

TEST_F(Fixture, NoStateIsDecel) {
  SafetyInput in = good(tick());
  in.c_has_state = false;
  const SafetyOutput o = step(in);
  EXPECT_EQ(o.mode, SafetyMode::kDecelStop);
  EXPECT_EQ(o.reasons, rt_control::SAFE_C_NO_STATE);
}

TEST_F(Fixture, HalStateMustBeReadyOrActive) {
  for (uint8_t s : {rt_control::HAL_READY, rt_control::HAL_ACTIVE}) {
    SafetyInput in = good(tick());
    in.hal_state = s;
    EXPECT_EQ(step(in).mode, SafetyMode::kRun) << int(s);
  }
  for (uint8_t s : {rt_control::HAL_INITIALIZING, rt_control::HAL_CONNECTING, rt_control::HAL_FAULT,
                    rt_control::HAL_RECONNECTING}) {
    sm.reset(tick());
    SafetyInput in = good(tick());
    in.hal_state = s;
    const SafetyOutput o = step(in);
    EXPECT_EQ(o.mode, SafetyMode::kDecelStop) << int(s);
    EXPECT_EQ(o.reasons, rt_control::SAFE_C_NOT_READY);
  }
}

// 오류 코드별 대응: 4·5·8·10 -> DECEL_STOP, 1·2·6·9 -> FAULT, 3·7 -> 그대로 RUN
TEST_F(Fixture, ErrorCodeMapping) {
  const struct {
    uint16_t code;
    SafetyMode mode;
  } cases[] = {{1, SafetyMode::kFault},     {2, SafetyMode::kFault},     {3, SafetyMode::kRun},
               {4, SafetyMode::kDecelStop}, {5, SafetyMode::kDecelStop}, {6, SafetyMode::kFault},
               {7, SafetyMode::kRun},       {8, SafetyMode::kDecelStop}, {9, SafetyMode::kFault},
               {10, SafetyMode::kDecelStop}};
  for (const auto& c : cases) {
    // HalState.active_errors로 알게 된 경우
    SafetyStateMachine a;
    a.reset(t);
    SafetyInput in = good(tick());
    in.c_error_mask = rt_control::err_bit(c.code);
    EXPECT_EQ(a.update(in).mode, c.mode) << "state code " << c.code;
    // /hal/error 이벤트로만 알게 된 경우 (예: disconnect는 상태 발행이 멈춤)
    SafetyStateMachine b;
    b.reset(t);
    b.on_error_event(c.code, true, t, 0);
    EXPECT_EQ(b.update(good(tick())).mode, c.mode) << "event code " << c.code;
  }
}

// 꺼짐 이벤트가 오면 이벤트 쪽 오류는 사라짐
TEST_F(Fixture, ErrorEventOffClears) {
  sm.on_error_event(5, true, t, 0);
  EXPECT_EQ(step(good(tick())).mode, SafetyMode::kDecelStop);
  sm.on_error_event(5, false, t, 0);
  EXPECT_EQ(step(good(tick())).active_reasons, 0u);
}

// E_EXPIRED가 최근 1초에 10번 이상 -> HOLD(C_EXPIRED)
TEST_F(Fixture, ExpiredBurstHolds) {
  for (int i = 0; i < 9; ++i) sm.on_error_event(3, true, tick(), 0);
  EXPECT_EQ(step(good(tick())).mode, SafetyMode::kRun);
  sm.on_error_event(3, true, tick(), 0);  // 10번째
  const SafetyOutput o = step(good(tick()));
  EXPECT_EQ(o.mode, SafetyMode::kHold);
  EXPECT_EQ(o.reasons, rt_control::SAFE_C_EXPIRED);
  // 1초가 지나 창에서 빠져도 resume_k번 통과해야 RUN
  t += kSec;
  EXPECT_EQ(step(good(t)).mode, SafetyMode::kHold);
  step(new_pass(tick()));
  step(new_pass(tick()));
  EXPECT_EQ(step(new_pass(tick())).mode, SafetyMode::kRun);
}

// 1초보다 넓게 퍼진 10번은 HOLD 아님
TEST_F(Fixture, ExpiredSpreadOutIsFine) {
  for (int i = 0; i < 10; ++i) {
    t += 150 * kMs;
    sm.on_error_event(3, true, t, 0);
  }
  EXPECT_EQ(step(good(tick())).mode, SafetyMode::kRun);
}

// ---------------- 우선순위 ----------------
TEST_F(Fixture, FaultBeatsDecelBeatsHold) {
  SafetyInput in = gate_block(tick(), rt_control::GATE_EXPIRED);  // HOLD 조건
  in.c_state_age_ns = 50 * kMs;                                   // DECEL 조건
  in.c_error_mask = rt_control::err_bit(9);                       // FAULT 조건
  SafetyOutput o = step(in);
  EXPECT_EQ(o.mode, SafetyMode::kFault);
  EXPECT_EQ(o.active_reasons,
            rt_control::SAFE_AI_EXPIRED | rt_control::SAFE_C_STATE_TIMEOUT | rt_control::SAFE_C_ERROR_FAULT);

  sm.reset(tick());
  in = gate_block(tick(), rt_control::GATE_EXPIRED);
  in.c_state_age_ns = 50 * kMs;
  o = step(in);
  EXPECT_EQ(o.mode, SafetyMode::kDecelStop);
  EXPECT_EQ(o.reasons, rt_control::SAFE_C_STATE_TIMEOUT);
}

// DECEL_STOP이 풀렸는데 AI가 아직 안 되면 HOLD로 내려감
TEST_F(Fixture, DecelReleasesIntoHoldIfAiStillBad) {
  SafetyInput in = gate_block(tick(), rt_control::GATE_EXPIRED);
  in.c_error_mask = rt_control::err_bit(5);
  EXPECT_EQ(step(in).mode, SafetyMode::kDecelStop);
  const int64_t clear = tick();
  step(gate_block(clear, rt_control::GATE_EXPIRED));
  const SafetyOutput o = step(gate_block(clear + kSec, rt_control::GATE_EXPIRED));
  EXPECT_EQ(o.mode, SafetyMode::kHold);
  EXPECT_TRUE(o.reset_limiter);
}

// ---------------- 재개 ----------------
// DECEL_STOP -> RUN: 원인이 풀린 상태가 1초 계속 + 멈춰 있음. 그때 limiter reset 요청
TEST_F(Fixture, DecelResumeDelay) {
  SafetyInput bad = good(tick());
  bad.c_error_mask = rt_control::err_bit(5);
  step(bad);
  const int64_t clear = tick();
  EXPECT_EQ(step(good(clear)).mode, SafetyMode::kDecelStop);
  EXPECT_EQ(step(good(clear + kSec - 1)).mode, SafetyMode::kDecelStop);
  const SafetyOutput o = step(good(clear + kSec));
  EXPECT_EQ(o.mode, SafetyMode::kRun);
  EXPECT_TRUE(o.changed);
  EXPECT_TRUE(o.reset_limiter);
  EXPECT_FALSE(o.new_session);
}

// 기다리는 중에 원인이 다시 생기면 1초를 처음부터
TEST_F(Fixture, DecelResumeTimerRestarts) {
  SafetyInput bad = good(tick());
  bad.c_error_mask = rt_control::err_bit(4);
  step(bad);
  step(good(tick()));
  t += 800 * kMs;
  bad.now_ns = t;
  step(bad);  // 다시 생김
  const int64_t clear = tick();
  step(good(clear));
  EXPECT_EQ(step(good(clear + 900 * kMs)).mode, SafetyMode::kDecelStop);
  EXPECT_EQ(step(good(clear + kSec)).mode, SafetyMode::kRun);
}

// 아직 움직이고 있으면 1초가 지나도 안 풀림
TEST_F(Fixture, DecelWaitsForStop) {
  SafetyInput bad = good(tick());
  bad.c_error_mask = rt_control::err_bit(5);
  step(bad);
  const int64_t clear = tick();
  step(good(clear));
  SafetyInput moving = good(clear + 2 * kSec);
  moving.limiter_speed = 0.01;
  EXPECT_EQ(step(moving).mode, SafetyMode::kDecelStop);
  EXPECT_EQ(step(good(clear + 2 * kSec + kMs)).mode, SafetyMode::kRun);
}

// FAULT는 원인이 풀려도 해제 요청 없이는 안 풀림
TEST_F(Fixture, FaultNeedsResetRequest) {
  sm.on_error_event(9, true, t, 0);
  EXPECT_EQ(step(good(tick())).mode, SafetyMode::kFault);
  sm.on_error_event(9, false, t, 0);
  EXPECT_EQ(step(good(t + 10 * kSec)).mode, SafetyMode::kFault);

  SafetyInput req = good(tick());
  req.reset_request = true;
  req.limiter_speed = 0.02;
  SafetyOutput o = step(req);
  EXPECT_EQ(o.reset_result, ResetResult::kMoving);
  EXPECT_EQ(o.mode, SafetyMode::kFault);

  req = good(tick());
  req.reset_request = true;
  o = step(req);
  EXPECT_EQ(o.reset_result, ResetResult::kAccepted);
  EXPECT_EQ(o.mode, SafetyMode::kRun);
  EXPECT_TRUE(o.new_session);
  EXPECT_TRUE(o.reset_limiter);
}

// 원인이 남아 있으면 해제 요청을 거절
TEST_F(Fixture, FaultResetRejectedWhileCauseActive) {
  SafetyInput req = good(tick());
  req.c_error_mask = rt_control::err_bit(1);
  step(req);
  req.now_ns = tick();
  req.reset_request = true;
  const SafetyOutput o = step(req);
  EXPECT_EQ(o.reset_result, ResetResult::kCauseActive);
  EXPECT_EQ(o.mode, SafetyMode::kFault);
  EXPECT_FALSE(o.new_session);
}

TEST_F(Fixture, ResetRequestWhenNotFault) {
  SafetyInput req = good(tick());
  req.reset_request = true;
  const SafetyOutput o = step(req);
  EXPECT_EQ(o.reset_result, ResetResult::kNotFault);
  EXPECT_EQ(o.mode, SafetyMode::kRun);
}

// ---------------- require_c_state = false ----------------
TEST_F(Fixture, RequireCStateFalseIgnoresC) {
  SafetyConfig c;
  c.require_c_state = false;
  sm = SafetyStateMachine(c);
  sm.reset(t);
  for (int i = 0; i < 20; ++i) sm.on_error_event(3, true, tick(), 0);
  sm.on_error_event(9, true, t, 0);
  SafetyInput in = good(tick());
  in.c_has_state = false;
  in.hal_state = rt_control::HAL_FAULT;
  in.c_error_mask = rt_control::err_bit(1) | rt_control::err_bit(5);
  const SafetyOutput o = step(in);
  EXPECT_EQ(o.mode, SafetyMode::kRun);
  EXPECT_EQ(o.active_reasons, 0u);
  // AI 규칙은 그대로
  EXPECT_EQ(step(gate_block(tick(), rt_control::GATE_EXPIRED)).mode, SafetyMode::kHold);
}

// 모드 지속 시간은 바뀐 순간부터
TEST_F(Fixture, ModeDuration) {
  const int64_t enter = tick();
  step(gate_block(enter, rt_control::GATE_EXPIRED));
  EXPECT_EQ(step(gate_block(enter + 1500 * kMs, rt_control::GATE_EXPIRED)).mode_duration_ns, 1500 * kMs);
}

// ---------------- HalError 세션 확인 ----------------
// 다른 세션의 오류 이벤트는 모드 판단에 쓰지 않고 개수만 셈
TEST_F(Fixture, OtherSessionErrorIgnored) {
  sm.set_session(1000);
  EXPECT_FALSE(sm.on_error_event(5, true, t, 999));   // 옛 세션
  EXPECT_FALSE(sm.on_error_event(9, true, t, 0));     // 세션 없음
  EXPECT_EQ(sm.ignored_error_events(), 2u);
  SafetyOutput o = step(good(tick()));
  EXPECT_EQ(o.mode, SafetyMode::kRun);
  EXPECT_EQ(o.c_error_mask, 0u);
  for (int i = 0; i < 20; ++i) sm.on_error_event(3, true, tick(), 999);  // E_EXPIRED 몰림도 무시
  EXPECT_EQ(step(good(tick())).mode, SafetyMode::kRun);

  // 같은 세션이면 씀
  EXPECT_TRUE(sm.on_error_event(5, true, t, 1000));
  o = step(good(tick()));
  EXPECT_EQ(o.mode, SafetyMode::kDecelStop);
  EXPECT_EQ(o.c_error_mask, rt_control::err_bit(5));
  EXPECT_EQ(sm.ignored_error_events(), 22u);
}

// 세션이 바뀌면 이벤트로 켜진 오류는 비움 (옛 세션의 꺼짐 이벤트는 무시되므로)
TEST_F(Fixture, NewSessionClearsEventErrors) {
  sm.set_session(1000);
  sm.on_error_event(5, true, t, 1000);
  EXPECT_EQ(step(good(tick())).c_error_mask, rt_control::err_bit(5));
  sm.set_session(2000);
  EXPECT_FALSE(sm.on_error_event(5, false, t, 1000));  // 옛 세션의 꺼짐 -> 무시
  EXPECT_EQ(step(good(tick())).c_error_mask, 0u);
  // HalState가 알려 주는 오류는 세션과 상관없이 그대로 씀
  SafetyInput in = good(tick());
  in.c_error_mask = rt_control::err_bit(4);
  EXPECT_EQ(step(in).c_error_mask, rt_control::err_bit(4));
}

TEST(SafetyTargetReject, MaskIsOutOfRangeAndIkFailed) {
  EXPECT_EQ(rt_control::kTargetRejectErrorMask, rt_control::err_bit(4) | rt_control::err_bit(5));
  EXPECT_EQ(rt_control::kTargetRejectErrorMask & ~rt_control::kDecelErrorMask, 0u);  // 둘 다 DECEL_STOP 대상
}

}  // namespace
