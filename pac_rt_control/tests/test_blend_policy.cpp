// blend_policy 단위 테스트 (시나리오 1, blend_mode single)
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rt_control/blend_policy.hpp"
#include "rt_control/gate.hpp"
#include "rt_control/motion_limiter.hpp"

namespace {

using rt_control::AiCommandSnapshot;
using rt_control::BlendConfig;
using rt_control::BlendPolicy;
using rt_control::BlendResult;
using rt_control::GateResult;

constexpr int64_t kTtl = 200'000'000;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// 시작 목표가 원점인 설정 (계산을 보기 쉽게)
BlendConfig origin_config() {
  BlendConfig c;
  c.ttl_ns = kTtl;
  c.initial_goal[0] = c.initial_goal[1] = c.initial_goal[2] = 0.0;
  return c;
}

AiCommandSnapshot cmd(uint64_t seq, double x, double conf = 1.0, uint32_t boot = 7) {
  AiCommandSnapshot s{};
  s.seq = seq;
  s.writer_boot_id = boot;
  s.target[0] = x;
  s.confidence = conf;
  return s;
}

GateResult pass(int64_t age_ns) { return GateResult{true, 0u, age_ns}; }
GateResult block(int64_t age_ns = 0) { return GateResult{false, rt_control::GATE_RANGE, age_ns}; }

// 나이 64ms, 신뢰도 1 -> w = 1 - 64/200 = 0.68
constexpr int64_t kAge068 = 64'000'000;

TEST(BlendSingle, PureFunctionNoApplyKeepsGoal) {
  double goal[3] = {0.1, 0.2, 0.3};
  const double w = rt_control::blend_single(goal, cmd(1, 0.5), false, 0, kTtl);
  EXPECT_EQ(w, 0.0);
  EXPECT_EQ(goal[0], 0.1);
  EXPECT_EQ(goal[1], 0.2);
  EXPECT_EQ(goal[2], 0.3);
}

TEST(Blend, NormalConvergence) {
  BlendPolicy b(origin_config());
  const double expect[3] = {0.0204, 0.026928, 0.029017};
  for (uint64_t seq = 1; seq <= 3; ++seq) {
    const BlendResult r = b.update(cmd(seq, 0.03), pass(kAge068));
    EXPECT_TRUE(r.applied);
    EXPECT_NEAR(r.w, 0.68, 1e-12);
    EXPECT_NEAR(r.goal[0], expect[seq - 1], 1e-6) << "seq " << seq;
    EXPECT_EQ(r.used_ai_seq, seq);
    EXPECT_TRUE(r.has_used);
  }
}

TEST(Blend, SameSeqAppliedOnce) {
  BlendPolicy b(origin_config());
  const BlendResult first = b.update(cmd(5, 0.03), pass(kAge068));
  EXPECT_TRUE(first.applied);
  for (int i = 0; i < 100; ++i) {  // 같은 명령이 100주기 동안 버퍼에 남아 있음
    const BlendResult r = b.update(cmd(5, 0.03), pass(kAge068 + i * 1'000'000));
    EXPECT_FALSE(r.applied);
    EXPECT_EQ(r.w, 0.0);
    EXPECT_EQ(r.goal[0], first.goal[0]);
    EXPECT_EQ(r.used_ai_seq, 5u);
  }
}

TEST(Blend, BlockedSeqNeverAppliedLater) {
  BlendPolicy b(origin_config());
  BlendResult r = b.update(cmd(3, 0.03), block());
  EXPECT_FALSE(r.applied);
  EXPECT_EQ(r.goal[0], 0.0);
  // 같은 seq 3이 이번엔 gate 통과로 바뀌어도 이미 "본" 명령이라 반영 안 함
  r = b.update(cmd(3, 0.03), pass(kAge068));
  EXPECT_FALSE(r.applied);
  EXPECT_EQ(r.goal[0], 0.0);
  EXPECT_EQ(r.used_ai_seq, 0u);
  EXPECT_FALSE(r.has_used);
  // 다음 seq는 반영
  r = b.update(cmd(4, 0.03), pass(kAge068));
  EXPECT_TRUE(r.applied);
  EXPECT_EQ(r.used_ai_seq, 4u);
}

TEST(Blend, OlderMeansSmallerW) {
  BlendPolicy b(origin_config());
  EXPECT_NEAR(b.update(cmd(1, 0.03, 0.8), pass(50'000'000)).w, 0.75 * 0.8, 1e-12);
  EXPECT_NEAR(b.update(cmd(2, 0.03, 0.8), pass(150'000'000)).w, 0.25 * 0.8, 1e-12);
  EXPECT_EQ(b.update(cmd(3, 0.03, 0.8), pass(kTtl)).w, 0.0);  // 나이 = TTL이면 신선도 0
}

TEST(Blend, SpikeThenRecover) {
  BlendConfig c = origin_config();
  c.initial_goal[0] = 0.5;
  BlendPolicy b(c);
  uint64_t seq = 1;
  b.update(cmd(seq++, 0.5), pass(kAge068));
  const BlendResult spike = b.update(cmd(seq++, 0.54), pass(kAge068));  // 4cm 튐 (gate는 통과)
  EXPECT_NEAR(spike.goal[0], 0.5 + 0.68 * 0.04, 1e-12);
  BlendResult r{};
  for (int i = 0; i < 5; ++i) r = b.update(cmd(seq++, 0.5), pass(kAge068));
  EXPECT_NEAR(r.goal[0], 0.5, 1e-3);
  EXPECT_LT(std::fabs(r.goal[0] - 0.5), std::fabs(spike.goal[0] - 0.5));
}

TEST(Blend, NanNeverReachesGoal) {
  BlendPolicy b(origin_config());
  AiCommandSnapshot s = cmd(1, kNaN);
  s.target[1] = kNaN;
  s.target[2] = std::numeric_limits<double>::infinity();
  s.confidence = kNaN;
  // gate가 막은 경우 (정상 흐름)
  BlendResult r = b.update(s, block());
  // gate가 실수로 통과시킨 경우에도 목표는 유한해야 함
  s.seq = 2;
  r = b.update(s, pass(kAge068));
  for (int i = 0; i < 3; ++i) EXPECT_TRUE(std::isfinite(r.goal[i]));
  EXPECT_EQ(r.goal[0], 0.0);
  EXPECT_FALSE(r.applied);
  EXPECT_EQ(r.w, 0.0);
}

TEST(Blend, BootChangeWithSmallerSeqContinues) {
  // 같은 곳으로 수렴하면 5번째 변화가 0.2mm라 deadband에 걸림. 이 테스트는 seq·boot 처리만 보므로 끔
  BlendConfig cfg = origin_config();
  cfg.deadband_m = 0.0;
  BlendPolicy b(cfg);
  BlendResult r{};
  for (uint64_t seq = 100; seq < 103; ++seq) r = b.update(cmd(seq, 0.03, 1.0, 7), pass(kAge068));
  const double before = r.goal[0];
  // A 재시작: boot_id 바뀌고 seq는 0부터
  r = b.update(cmd(0, 0.03, 1.0, 8), pass(kAge068));
  EXPECT_TRUE(r.applied);
  EXPECT_EQ(r.used_ai_seq, 0u);
  EXPECT_TRUE(r.has_used);  // seq 0을 반영한 것 ("없음"과 구분됨)
  EXPECT_GT(r.goal[0], before);       // 이어서 0.03 쪽으로 진행
  EXPECT_LT(r.goal[0], 0.03 + 1e-12);  // 처음부터 다시 하지 않음 (튀지 않음)
  r = b.update(cmd(1, 0.03, 1.0, 8), pass(kAge068));
  EXPECT_TRUE(r.applied);
  EXPECT_EQ(r.used_ai_seq, 1u);
}

TEST(Blend, NoDataKeepsInitialGoal) {
  BlendPolicy b;  // 기본 설정: initial_goal = planned_position
  const GateResult no_data{false, rt_control::GATE_NO_DATA, 0};
  for (int i = 0; i < 10; ++i) {
    const BlendResult r = b.update(AiCommandSnapshot{}, no_data);
    EXPECT_FALSE(r.applied);
    EXPECT_EQ(r.w, 0.0);
    EXPECT_EQ(r.used_ai_seq, 0u);
    EXPECT_FALSE(r.has_used);
    for (int k = 0; k < 3; ++k) EXPECT_EQ(r.goal[k], rt_control::kDefaultPlannedPosition[k]);
  }
}

TEST(Blend, HasUsedDistinguishesSeqZero) {
  BlendPolicy b(origin_config());
  BlendResult r = b.update(cmd(0, 0.03), block());  // seq 0이 막힘 -> 아직 반영 없음
  EXPECT_EQ(r.used_ai_seq, 0u);
  EXPECT_FALSE(r.has_used);
  r = b.update(cmd(1, 0.03), pass(kAge068));
  EXPECT_TRUE(r.has_used);
  r = b.update(cmd(2, 0.03), block());  // 한 번 반영한 뒤에는 막혀도 계속 true
  EXPECT_TRUE(r.has_used);
  EXPECT_EQ(r.used_ai_seq, 1u);
}

TEST(Blend, ParseMode) {
  rt_control::BlendMode m{};
  EXPECT_TRUE(rt_control::parse_blend_mode("single", &m));
  EXPECT_EQ(m, rt_control::BlendMode::kSingle);
  EXPECT_FALSE(rt_control::parse_blend_mode("median3", &m));  // 아직 없음
}

// ---------------- target_offset (철판 표면 -> TCP 작업 높이) ----------------

constexpr double kInf = std::numeric_limits<double>::infinity();

BlendConfig offset_config(double ox, double oy, double oz) {
  BlendConfig c = origin_config();
  c.target_offset[0] = ox;
  c.target_offset[1] = oy;
  c.target_offset[2] = oz;
  return c;
}

AiCommandSnapshot cmd3(uint64_t seq, double x, double y, double z) {
  AiCommandSnapshot s = cmd(seq, x);
  s.target[1] = y;
  s.target[2] = z;
  return s;
}

TEST(BlendOffset, AddedToBlendGoalExactly) {
  // w = 1 (나이 0, 신뢰도 1)이면 목표 = AI 위치 + 오프셋
  BlendPolicy b(offset_config(0.01, -0.02, 0.10));
  const BlendResult r = b.update(cmd3(1, 0.5, 0.0, 0.71), pass(0));
  ASSERT_TRUE(r.applied);
  EXPECT_DOUBLE_EQ(r.w, 1.0);
  EXPECT_DOUBLE_EQ(r.goal[0], 0.5 + 0.01);
  EXPECT_DOUBLE_EQ(r.goal[1], 0.0 - 0.02);
  EXPECT_DOUBLE_EQ(r.goal[2], 0.71 + 0.10);
}

TEST(BlendOffset, AddedBeforeWeighting) {
  // w = 0.68: 목표 = 0 + 0.68 × (0.71 + 0.10 - 0)
  BlendPolicy b(offset_config(0.0, 0.0, 0.10));
  const BlendResult r = b.update(cmd3(1, 0.0, 0.0, 0.71), pass(kAge068));
  EXPECT_NEAR(r.goal[2], 0.68 * 0.81, 1e-12);
  EXPECT_DOUBLE_EQ(r.goal[0], 0.0);
}

TEST(BlendOffset, PureFunctionNullptrMeansZero) {
  double g1[3] = {0.1, 0.2, 0.3};
  double g2[3] = {0.1, 0.2, 0.3};
  const double zero[3] = {0.0, 0.0, 0.0};
  const AiCommandSnapshot c = cmd3(1, 0.5, 0.0, 0.71);
  const double w1 = rt_control::blend_single(g1, c, true, kAge068, kTtl);
  const double w2 = rt_control::blend_single(g2, c, true, kAge068, kTtl, zero);
  EXPECT_EQ(w1, w2);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(g1[i], g2[i]);
}

TEST(BlendOffset, GateChecksRawValueBeforeOffset) {
  // 계획 위치 = 철판 표면 [0.5, 0, 0.71], 허용 반경 5cm
  rt_control::GateConfig gc;
  gc.planned_position[0] = 0.5;
  gc.planned_position[1] = 0.0;
  gc.planned_position[2] = 0.71;
  gc.accept_radius = 0.05;
  const int64_t now = 10'000'000'000;
  AiCommandSnapshot s = cmd3(1, 0.5, 0.0, 0.75);  // 철판에서 4cm
  s.layout_version = gc.expected_layout_version;
  s.command_type = gc.expected_command_type;
  s.status = rt_control::kStatusOk;
  s.age_ns = 0;
  s.recv_mono_ns = now;

  // gate는 AI 원래 값(0.75)으로 검사 -> 통과. 오프셋 10cm는 gate 설정에 없으므로 영향 없음
  const GateResult g = rt_control::gate_check(s, true, now, gc);
  EXPECT_TRUE(g.pass) << "reasons " << g.reasons;

  // blend는 0.75 + 0.10 = 0.85를 목표로 (계획 위치에서 14cm지만 gate를 다시 보지 않음)
  BlendConfig bc = offset_config(0.0, 0.0, 0.10);
  for (int i = 0; i < 3; ++i) bc.initial_goal[i] = gc.planned_position[i];
  BlendPolicy b(bc);
  const BlendResult r = b.update(s, g);
  ASSERT_TRUE(r.applied);
  EXPECT_DOUBLE_EQ(r.goal[2], 0.75 + 0.10);

  // 원래 값이 6cm 밖이면 오프셋과 상관없이 막힘
  AiCommandSnapshot far = s;
  far.seq = 2;
  far.target[2] = 0.77;
  EXPECT_FALSE(rt_control::gate_check(far, true, now, gc).pass);
}

TEST(BlendOffset, NotAddedToStartGoalNoDataOrHold) {
  // 시작 목표(측정 위치)에는 안 더함: 명령이 없으면 목표 = initial_goal 그대로
  BlendConfig bc = offset_config(0.0, 0.0, 0.10);
  bc.initial_goal[0] = 0.45;
  bc.initial_goal[1] = 0.0;
  bc.initial_goal[2] = 0.75;
  BlendPolicy b(bc);
  const GateResult no_data{false, rt_control::GATE_NO_DATA, 0};
  for (int i = 0; i < 5; ++i) {
    const BlendResult r = b.update(AiCommandSnapshot{}, no_data);
    EXPECT_FALSE(r.applied);
    EXPECT_EQ(r.goal[0], 0.45);
    EXPECT_EQ(r.goal[1], 0.0);
    EXPECT_EQ(r.goal[2], 0.75);
  }
  // gate에서 막힌 명령도 목표를 바꾸지 않음
  BlendResult r = b.update(cmd3(1, 0.5, 0.0, 0.71), block());
  EXPECT_EQ(r.goal[2], 0.75);
  // HOLD(allow=false): 통과한 새 명령도 반영 안 함 -> 유지 목표에 오프셋이 더해지지 않음
  r = b.update(cmd3(2, 0.5, 0.0, 0.71), pass(0), false);
  EXPECT_FALSE(r.applied);
  EXPECT_EQ(r.goal[0], 0.45);
  EXPECT_EQ(r.goal[2], 0.75);
  // 같은 명령을 다시 봐도(새 명령 아님) 그대로
  r = b.update(cmd3(2, 0.5, 0.0, 0.71), pass(0));
  EXPECT_EQ(r.goal[2], 0.75);
}

TEST(BlendOffset, ZeroOffsetMatchesDefaultBitForBit) {
  BlendPolicy a(origin_config());              // 오프셋 필드 기본값
  BlendPolicy z(offset_config(0.0, 0.0, 0.0));  // 명시적으로 0
  const double xs[] = {0.03, 0.031, -0.02, 0.0123456789, 0.5};
  const int64_t ages[] = {kAge068, 0, 150'000'000, 10'000'000, 199'000'000};
  for (uint64_t seq = 1; seq <= 5; ++seq) {
    const AiCommandSnapshot c = cmd3(seq, xs[seq - 1], 0.001 * seq, 0.71 - 0.003 * seq);
    const GateResult g = (seq == 3) ? block() : pass(ages[seq - 1]);
    const BlendResult ra = a.update(c, g);
    const BlendResult rz = z.update(c, g);
    EXPECT_EQ(ra.applied, rz.applied);
    EXPECT_EQ(ra.w, rz.w);
    for (int i = 0; i < 3; ++i) EXPECT_EQ(ra.goal[i], rz.goal[i]) << "seq " << seq << " axis " << i;
  }
}

TEST(BlendOffset, NonFiniteOffsetTreatedAsZero) {
  double off[3] = {kNaN, 0.10, kInf};
  EXPECT_TRUE(rt_control::sanitize_offset(off));
  EXPECT_EQ(off[0], 0.0);
  EXPECT_EQ(off[1], 0.10);
  EXPECT_EQ(off[2], 0.0);
  double ok[3] = {0.0, 0.0, 0.10};
  EXPECT_FALSE(rt_control::sanitize_offset(ok));

  // BlendPolicy도 직접 넣은 NaN·무한대를 0으로 씀 (목표가 NaN이 되지 않음)
  BlendPolicy b(offset_config(kNaN, -kInf, kNaN));
  const BlendResult r = b.update(cmd3(1, 0.5, 0.0, 0.71), pass(0));
  ASSERT_TRUE(r.applied);
  EXPECT_DOUBLE_EQ(r.goal[0], 0.5);
  EXPECT_DOUBLE_EQ(r.goal[1], 0.0);
  EXPECT_DOUBLE_EQ(r.goal[2], 0.71);
}

// ---------------- deadband ----------------

BlendConfig deadband_config(double db) {
  BlendConfig c = origin_config();
  c.deadband_m = db;
  return c;
}

TEST(BlendDeadband, DefaultIsHalfMillimeter) { EXPECT_DOUBLE_EQ(BlendConfig{}.deadband_m, 0.0005); }

TEST(BlendDeadband, SmallChangeIgnoredLargeChangeApplied) {
  BlendPolicy b(deadband_config(0.0005));
  // 1) 10cm -> 반영
  BlendResult r = b.update(cmd(1, 0.10), pass(0));
  ASSERT_TRUE(r.applied);
  EXPECT_EQ(r.goal[0], 0.10);
  // 2) 0.4mm 옮기는 명령 (w = 1) -> deadband 안이라 무시, 목표 비트 단위로 그대로
  r = b.update(cmd(2, 0.1004), pass(0));
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.deadband_skipped);
  EXPECT_EQ(r.w, 0.0);
  EXPECT_EQ(r.goal[0], 0.10);
  EXPECT_EQ(r.used_ai_seq, 1u);  // 반영한 명령은 여전히 1
  // 3) 0.6mm -> 반영
  r = b.update(cmd(3, 0.1006), pass(0));
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.deadband_skipped);
  EXPECT_EQ(r.goal[0], 0.1006);
  EXPECT_EQ(r.used_ai_seq, 3u);
}

TEST(BlendDeadband, UsesActualGoalChangeNotRawDistance) {
  BlendPolicy b(deadband_config(0.0005));
  ASSERT_TRUE(b.update(cmd(1, 0.10), pass(0)).applied);
  // AI 위치는 1mm 떨어졌지만 w = 0.3 이라 목표는 0.3mm만 움직임 -> 무시
  BlendResult r = b.update(cmd(2, 0.101, 0.3), pass(0));
  EXPECT_FALSE(r.applied);
  EXPECT_EQ(r.goal[0], 0.10);
  // 같은 1mm라도 w = 1이면 1mm 움직임 -> 반영
  r = b.update(cmd(3, 0.101, 1.0), pass(0));
  EXPECT_TRUE(r.applied);
  EXPECT_EQ(r.goal[0], 0.101);
}

TEST(BlendDeadband, VectorDistanceAcrossAxes) {
  BlendPolicy b(deadband_config(0.0005));
  ASSERT_TRUE(b.update(cmd3(1, 0.1, 0.1, 0.1), pass(0)).applied);
  // 축마다 0.4mm (각각은 deadband 안) -> 벡터 거리 0.69mm는 밖이라 반영
  const BlendResult r = b.update(cmd3(2, 0.1004, 0.1004, 0.1004), pass(0));
  EXPECT_TRUE(r.applied);
}

TEST(BlendDeadband, ConvergingCommandsStopChangingGoal) {
  // 같은 곳을 가리키는 명령이 계속 와도(w < 1이면 목표가 조금씩 다가감) 결국 목표가 고정됨
  BlendPolicy b(deadband_config(0.0005));
  double prev = 0.0;
  int changes_after_settle = 0;
  for (uint64_t seq = 1; seq <= 200; ++seq) {
    const BlendResult r = b.update(cmd(seq, 0.05, 0.8), pass(0));
    if (seq > 20 && r.goal[0] != prev) ++changes_after_settle;
    prev = r.goal[0];
  }
  EXPECT_EQ(changes_after_settle, 0);
  EXPECT_NEAR(prev, 0.05, 0.0005 / 0.8);
}

TEST(BlendDeadband, ZeroDisables) {
  BlendPolicy b(deadband_config(0.0));
  ASSERT_TRUE(b.update(cmd(1, 0.10), pass(0)).applied);
  const BlendResult r = b.update(cmd(2, 0.1000001), pass(0));
  EXPECT_TRUE(r.applied);
  EXPECT_EQ(r.goal[0], 0.1000001);
}

// ---------------- used_ai_boot_id ----------------

TEST(BlendBootId, ZeroBeforeFirstApply) {
  BlendPolicy b(origin_config());
  BlendResult r = b.update(cmd(1, 0.01, 1.0, 42), block());  // gate 차단
  EXPECT_FALSE(r.has_used);
  EXPECT_EQ(r.used_ai_boot_id, 0u);
  r = b.update(cmd(2, 0.01, 1.0, 42), pass(0), false);  // HOLD 등으로 막힘
  EXPECT_EQ(r.used_ai_boot_id, 0u);
  r = b.update(cmd(3, 0.0001, 1.0, 42), pass(0));  // deadband 안이라 무시
  EXPECT_FALSE(r.has_used);
  EXPECT_EQ(r.used_ai_boot_id, 0u);
}

TEST(BlendBootId, MatchesAppliedCommand) {
  BlendPolicy b(origin_config());
  BlendResult r = b.update(cmd(5, 0.01, 1.0, 1111), pass(0));
  ASSERT_TRUE(r.applied);
  EXPECT_EQ(r.used_ai_boot_id, 1111u);
  EXPECT_EQ(r.used_ai_seq, 5u);
  // A 재시작 (새 boot, seq 0부터) -> 반영되면 새 boot_id
  r = b.update(cmd(0, 0.03, 1.0, 2222), pass(0));
  ASSERT_TRUE(r.applied);
  EXPECT_EQ(r.used_ai_boot_id, 2222u);
  EXPECT_EQ(r.used_ai_seq, 0u);
  // 다른 boot의 명령이 gate에 막히면 반영한 boot_id는 그대로
  r = b.update(cmd(1, 0.05, 1.0, 3333), block());
  EXPECT_EQ(r.used_ai_boot_id, 2222u);
  // deadband로 무시된 명령도 boot_id를 바꾸지 않음
  r = b.update(cmd(2, 0.0301, 1.0, 4444), pass(0));
  EXPECT_FALSE(r.applied);
  EXPECT_EQ(r.used_ai_boot_id, 2222u);
}

// ---------------- 거부된 목표 대기 (C가 목표를 거부한 뒤 교착 해결) ----------------
TEST(BlendRejected, DefaultTolIsOneCentimeter) { EXPECT_DOUBLE_EQ(BlendConfig{}.rejected_goal_tol_m, 0.01); }

// 반영한 적이 없으면 기억할 목표도 없음
TEST(BlendRejected, NothingToRejectBeforeFirstApply) {
  BlendPolicy b(origin_config());
  EXPECT_FALSE(b.reject_last_applied());
  EXPECT_FALSE(b.rejected_wait());
}

// 거부 뒤: limiter·blend 목표를 측정 TCP로 맞추면 그 자리를 그대로 유지하고,
// 같은 목표(오차 1cm 안)가 계속 와도 반영하지 않음
TEST(BlendRejected, HoldsMeasuredPositionAndIgnoresSameGoal) {
  BlendPolicy b(origin_config());
  uint64_t seq = 1;
  ASSERT_TRUE(b.update(cmd(seq++, 0.10), pass(0)).applied);  // C가 거부할 목표 x = 0.10
  ASSERT_TRUE(b.reject_last_applied());
  EXPECT_DOUBLE_EQ(b.rejected_target()[0], 0.10);

  // DECEL_STOP에서 재개: 측정 TCP(로봇은 거부 때문에 x = 0.03에 멈춰 있음)로 limiter·목표를 맞춤
  const double tcp[3] = {0.03, 0.0, 0.0};
  b.reset_goal(tcp);
  rt_control::MotionLimiter lim;
  lim.reset(tcp);

  // 같은 목표(노이즈 ±5mm)가 계속 와도 목표·출력이 측정 위치에서 비트 단위로 그대로
  for (int i = 0; i < 200; ++i) {
    const double x = 0.10 + ((i % 2) ? 0.005 : -0.005);
    const BlendResult r = b.update(cmd(seq++, x), pass(0));
    EXPECT_FALSE(r.applied);
    EXPECT_TRUE(r.rejected_skipped);
    EXPECT_TRUE(r.rejected_wait);
    EXPECT_EQ(r.goal[0], 0.03);
    const rt_control::LimiterResult m = lim.update(r.goal, 0.001, false);
    EXPECT_EQ(m.pos[0], 0.03);
    EXPECT_EQ(m.pos[1], 0.0);
    EXPECT_EQ(m.pos[2], 0.0);
    EXPECT_EQ(m.speed, 0.0);
  }
  EXPECT_EQ(b.update(cmd(1, 0.10), pass(0)).used_ai_seq, 1u);  // 거부 뒤로 반영된 명령 없음
}

// 1cm 이상 다른 목표가 오면 대기 끝, 그 목표로 다시 움직임
TEST(BlendRejected, DifferentGoalResumes) {
  BlendPolicy b(origin_config());
  ASSERT_TRUE(b.update(cmd(1, 0.10), pass(0)).applied);
  ASSERT_TRUE(b.reject_last_applied());
  const double tcp[3] = {0.03, 0.0, 0.0};
  b.reset_goal(tcp);

  BlendResult r = b.update(cmd(2, 0.0905), pass(0));  // 9.5mm 차이: 아직 같은 목표로 봄
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected_wait);

  r = b.update(cmd(3, 0.085), pass(0));  // 15mm 차이: 새 목표
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected_skipped);
  EXPECT_FALSE(r.rejected_wait);
  EXPECT_DOUBLE_EQ(r.goal[0], 0.085);
  EXPECT_EQ(r.used_ai_seq, 3u);

  // 대기가 끝났으니 처음 목표(0.10)도 다시 반영할 수 있음
  r = b.update(cmd(4, 0.10), pass(0));
  EXPECT_TRUE(r.applied);
  EXPECT_DOUBLE_EQ(r.goal[0], 0.10);
}

// 비교는 target_offset을 더한 목표끼리 (거부된 것은 C에 보낸 TCP 목표)
TEST(BlendRejected, ComparesWithOffsetApplied) {
  BlendPolicy b(offset_config(0.0, 0.0, 0.05));
  ASSERT_TRUE(b.update(cmd3(1, 0.0, 0.0, 0.70), pass(0)).applied);
  ASSERT_TRUE(b.reject_last_applied());
  EXPECT_DOUBLE_EQ(b.rejected_target()[2], 0.75);
  EXPECT_FALSE(b.update(cmd3(2, 0.0, 0.0, 0.70), pass(0)).applied);
  EXPECT_TRUE(b.update(cmd3(3, 0.0, 0.0, 0.72), pass(0)).applied);  // 2cm 다름
}

// HOLD 등으로 반영이 막힌 주기에는 대기 상태가 그대로 (다른 목표가 와도 반영 허용될 때까지 유지)
TEST(BlendRejected, NotClearedWhileBlendNotAllowed) {
  BlendPolicy b(origin_config());
  ASSERT_TRUE(b.update(cmd(1, 0.10), pass(0)).applied);
  ASSERT_TRUE(b.reject_last_applied());
  BlendResult r = b.update(cmd(2, 0.05), pass(0), false);
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected_wait);
  r = b.update(cmd(3, 0.05), block());
  EXPECT_TRUE(r.rejected_wait);
  r = b.update(cmd(4, 0.05), pass(0));
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected_wait);
}

}  // namespace
