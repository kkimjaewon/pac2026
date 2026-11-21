// INIT 유지 명령(init_hold) 단위 테스트
//   신선한 HalState 전엔 발행 0 / 발행 위치·자세 = 측정 TCP / INIT 중 AI 목표 무시 / 세션 42 유지, 0이면 발급
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rt_control/blend_policy.hpp"
#include "rt_control/gate.hpp"
#include "rt_control/hal_command_snapshot.hpp"
#include "rt_control/hal_state_snapshot.hpp"
#include "rt_control/init_hold.hpp"
#include "rt_control/motion_limiter.hpp"

namespace {

using rt_control::HalCommandConfig;
using rt_control::HalCommandSnapshot;
using rt_control::HalStateSnapshot;
using rt_control::InitHold;
using rt_control::InitHoldConfig;
using rt_control::InitHoldEvent;
using rt_control::InitHoldResult;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kT0 = 10'000 * kMs;

// 신선한 HalState: 1ms 전에 받음, pose_valid, stale 아님
HalStateSnapshot state_at(int64_t recv_ns, double x = 0.45, double y = 0.01, double z = 0.75) {
  HalStateSnapshot s{};
  s.recv_mono_ns = recv_ns;
  s.pose_valid = true;
  s.stale = false;
  s.hal_state = rt_control::HAL_READY;
  s.tcp_pos[0] = x;
  s.tcp_pos[1] = y;
  s.tcp_pos[2] = z;
  s.tcp_quat[0] = 0.0;  // 정규화 전 (0, 0, 2, 0) -> (0, 0, 1, 0)
  s.tcp_quat[1] = 0.0;
  s.tcp_quat[2] = 2.0;
  s.tcp_quat[3] = 0.0;
  return s;
}

struct Fixture : ::testing::Test {
  InitHold hold;
  HalCommandSnapshot cmd{};
  HalCommandConfig cfg;  // 기본 자세 (1, 0, 0, 0), TTL 5ms
  uint64_t session = 0;
  int64_t t = kT0;
  InitHoldResult step(bool has_state, const HalStateSnapshot& s) { return hold.step(has_state, s, t, session, cmd, cfg); }
};

// ---------------- 신선한 HalState 전엔 발행 0 ----------------
TEST_F(Fixture, NothingBeforeFreshState) {
  HalStateSnapshot s = state_at(t - 1 * kMs);
  // 받은 적 없음
  EXPECT_FALSE(step(false, s).publish);
  // pose_valid false
  s.pose_valid = false;
  EXPECT_FALSE(step(true, s).publish);
  // stale 표시
  s = state_at(t - 1 * kMs);
  s.stale = true;
  EXPECT_FALSE(step(true, s).publish);
  // 나이 = state_timeout(20ms) 이상
  s = state_at(t - 20 * kMs);
  EXPECT_FALSE(step(true, s).publish);
  // 위치 NaN, 자세 0
  s = state_at(t - 1 * kMs, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(step(true, s).publish);
  s = state_at(t - 1 * kMs);
  for (double& q : s.tcp_quat) q = 0.0;
  EXPECT_FALSE(step(true, s).publish);

  EXPECT_FALSE(hold.holding());
  EXPECT_EQ(hold.published(), 0u);
  EXPECT_EQ(session, 0u);  // 세션도 아직 안 정함
  EXPECT_EQ(cmd.command_seq, 0u);
}

// ---------------- 발행 위치·자세 = 측정 TCP ----------------
TEST_F(Fixture, PublishesMeasuredPoseEveryCycle) {
  for (int i = 0; i < 5; ++i) {
    t += kMs;
    const double x = 0.45 + 0.001 * i;  // 로봇이 조금씩 움직여도 그때 측정 위치 그대로
    const InitHoldResult r = step(true, state_at(t - 2 * kMs, x, 0.01, 0.75));
    ASSERT_TRUE(r.publish);
    EXPECT_EQ(r.event, i == 0 ? InitHoldEvent::kStarted : InitHoldEvent::kNone);
    EXPECT_EQ(cmd.pos[0], x);
    EXPECT_EQ(cmd.pos[1], 0.01);
    EXPECT_EQ(cmd.pos[2], 0.75);
    EXPECT_DOUBLE_EQ(cmd.quat[0], 0.0);
    EXPECT_DOUBLE_EQ(cmd.quat[1], 0.0);
    EXPECT_DOUBLE_EQ(cmd.quat[2], 1.0);  // 측정 자세 (정규화), tool_orientation 아님
    EXPECT_DOUBLE_EQ(cmd.quat[3], 0.0);
    EXPECT_EQ(cmd.command_seq, static_cast<uint64_t>(i + 1));
    EXPECT_EQ(cmd.created_mono_ns, t);
    EXPECT_EQ(cmd.expires_mono_ns, t + cfg.hal_ttl_ns);
    EXPECT_EQ(cmd.source_result_id, 0u);  // AI 반영 없음
    EXPECT_EQ(hold.hold_pos()[0], x);
  }
  EXPECT_EQ(hold.published(), 5u);
}

// ---------------- HalState가 끊기면 멈추고 다시 기다림 ----------------
TEST_F(Fixture, StopsWhenStateLostAndResumes) {
  ASSERT_TRUE(step(true, state_at(t - kMs)).publish);
  const uint64_t s0 = session;
  const HalStateSnapshot last = state_at(t - kMs);
  // 같은 상태가 20ms 넘게 갱신 안 됨 -> 멈춤 (kLost 한 번)
  t += 25 * kMs;
  InitHoldResult r = step(true, last);
  EXPECT_FALSE(r.publish);
  EXPECT_EQ(r.event, InitHoldEvent::kLost);
  EXPECT_FALSE(hold.holding());
  const uint64_t seq_at_loss = cmd.command_seq;
  t += kMs;
  r = step(true, last);
  EXPECT_FALSE(r.publish);
  EXPECT_EQ(r.event, InitHoldEvent::kNone);
  EXPECT_EQ(cmd.command_seq, seq_at_loss);  // 멈춘 동안 명령을 새로 만들지 않음
  // 다시 신선해지면 같은 세션으로 이어서 발행
  r = step(true, state_at(t - kMs, 0.46));
  EXPECT_TRUE(r.publish);
  EXPECT_EQ(r.event, InitHoldEvent::kStarted);
  EXPECT_FALSE(r.session_issued);
  EXPECT_EQ(session, s0);
  EXPECT_EQ(cmd.command_seq, seq_at_loss + 1);
  EXPECT_EQ(cmd.pos[0], 0.46);
}

// ---------------- 세션 ----------------
TEST(InitHoldSession, ZeroIssuesAtFirstFreshState) {
  InitHold hold;  // registered_session_id 0
  HalCommandSnapshot cmd{};
  HalCommandConfig cfg;
  uint64_t session = 0;
  // 신선하지 않으면 안 정함
  HalStateSnapshot old = state_at(kT0 - 50 * kMs);
  EXPECT_FALSE(hold.step(true, old, kT0, session, cmd, cfg).session_issued);
  EXPECT_EQ(session, 0u);
  // 첫 신선한 HalState 시각으로 발급, command_seq 1부터
  const int64_t t1 = kT0 + 7 * kMs;
  InitHoldResult r = hold.step(true, state_at(t1 - kMs), t1, session, cmd, cfg);
  EXPECT_TRUE(r.session_issued);
  EXPECT_EQ(session, static_cast<uint64_t>(t1));
  EXPECT_EQ(cmd.session_id, static_cast<uint64_t>(t1));
  EXPECT_EQ(cmd.command_seq, 1u);
  // 다음 주기부터는 그대로
  r = hold.step(true, state_at(t1), t1 + kMs, session, cmd, cfg);
  EXPECT_FALSE(r.session_issued);
  EXPECT_EQ(session, static_cast<uint64_t>(t1));
  EXPECT_EQ(cmd.command_seq, 2u);
}

TEST(InitHoldSession, RegisteredKeeps42AndContinuesCSeq) {
  InitHoldConfig c;
  c.registered_session_id = 42;
  InitHold hold(c);
  HalCommandSnapshot cmd{};
  HalCommandConfig cfg;
  uint64_t session = 0;
  HalStateSnapshot s = state_at(kT0 - kMs);
  s.active_session_id = 42;     // C가 세션 42의 마지막 순번 900을 보고 (B 재시작)
  s.last_command_seq = 900;
  InitHoldResult r = hold.step(true, s, kT0, session, cmd, cfg);
  EXPECT_TRUE(r.session_issued);
  EXPECT_EQ(session, 42u);
  EXPECT_EQ(cmd.session_id, 42u);
  EXPECT_EQ(cmd.command_seq, 901u);
  // C가 다른 세션을 보고하면 1부터
  InitHold hold2(c);
  HalCommandSnapshot cmd2{};
  uint64_t session2 = 0;
  s.active_session_id = 7;
  hold2.step(true, s, kT0, session2, cmd2, cfg);
  EXPECT_EQ(session2, 42u);
  EXPECT_EQ(cmd2.command_seq, 1u);
}

TEST(InitHoldSession, ChooseSessionId) {
  EXPECT_EQ(rt_control::choose_session_id(42, 0, 123), 42u);
  EXPECT_EQ(rt_control::choose_session_id(42, 99, 123), 42u);
  EXPECT_EQ(rt_control::choose_session_id(0, 99, 123), 99u);
  EXPECT_EQ(rt_control::choose_session_id(0, 0, 123), 123u);
}

// ---------------- INIT 중 AI 목표 무시, RUN은 유지 위치에서 출발 ----------------
// 노드가 하는 순서 그대로: INIT 동안은 InitHold만 (AI 명령은 읽지 않음), RUN 시작 때
// blend 시작 목표·limiter를 마지막 유지 명령 위치로 둔 뒤부터 AI를 반영
TEST(InitHoldRun, AiIgnoredDuringInitAndRunStartsAtHoldPosition) {
  InitHold hold;
  HalCommandSnapshot cmd{};
  HalCommandConfig cfg;
  uint64_t session = 0;
  int64_t t = kT0;

  // A는 INIT 동안 계속 x = 0.50을 보냄 (로봇은 0.45에 있음)
  rt_control::AiCommandSnapshot ai{};
  ai.writer_boot_id = 1;
  ai.confidence = 1.0;
  ai.target[0] = 0.50;
  ai.target[1] = 0.01;
  ai.target[2] = 0.75;

  for (int i = 0; i < 200; ++i) {
    t += kMs;
    ai.seq = static_cast<uint64_t>(i);
    ASSERT_TRUE(hold.step(true, state_at(t - kMs), t, session, cmd, cfg).publish);
    EXPECT_EQ(cmd.pos[0], 0.45);  // INIT 발행은 AI 0.50이 아니라 측정 TCP
    EXPECT_EQ(cmd.source_result_id, 0u);
  }

  // RUN 시작: 마지막 유지 명령 위치에서
  rt_control::BlendConfig bc;
  for (int i = 0; i < 3; ++i) bc.initial_goal[i] = hold.hold_pos()[i];
  rt_control::BlendPolicy blend(bc);
  rt_control::MotionLimiter lim;
  lim.reset(hold.hold_pos());
  const uint64_t last_hold_seq = cmd.command_seq;

  // RUN 첫 주기: 새 AI 명령을 처음 반영하지만 limiter 출력은 유지 위치에서 연속 (튀지 않음)
  t += kMs;
  ai.seq = 1000;
  const rt_control::GateResult g{true, 0u, 0};
  const rt_control::BlendResult b = blend.update(ai, g);
  EXPECT_TRUE(b.applied);
  const rt_control::LimiterResult m = lim.update(b.goal, 0.001, false);
  rt_control::fill_hal_command(cmd, session, t, m.pos, b.has_used, b.used_ai_seq, cfg);
  EXPECT_EQ(cmd.command_seq, last_hold_seq + 1);  // seq도 이어짐
  EXPECT_NEAR(cmd.pos[0], 0.45, 1e-6);           // 1ms 동안 거의 안 움직임
  EXPECT_GT(cmd.pos[0], 0.45);                    // AI 목표 쪽으로 출발
}

}  // namespace
