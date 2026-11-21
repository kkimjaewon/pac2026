// gate_check 단위 테스트 (시나리오 1)
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rt_control/gate.hpp"

namespace {

using rt_control::AiCommandSnapshot;
using rt_control::gate_check;
using rt_control::GateConfig;
using rt_control::GateResult;

constexpr int64_t kNow = 10'000'000'000;  // 지금 시각 10s (임의)
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// 모든 검사를 통과하는 기본 명령: 계획 위치 그대로, 신뢰도 1, 나이 10ms
AiCommandSnapshot good_cmd(const GateConfig& c) {
  AiCommandSnapshot s{};
  s.seq = 1;
  s.writer_boot_id = 123;
  s.layout_version = c.expected_layout_version;
  s.command_type = c.expected_command_type;
  s.status = rt_control::kStatusOk;
  s.target[0] = c.planned_position[0];
  s.target[1] = c.planned_position[1];
  s.target[2] = c.planned_position[2];
  s.target[3] = 0.0;
  s.confidence = 1.0;
  s.age_ns = 5'000'000;           // A 쪽에서 5ms
  s.recv_mono_ns = kNow - 5'000'000;  // B가 받은 지 5ms
  return s;
}

GateResult run(const AiCommandSnapshot& s, const GateConfig& c = GateConfig{}) {
  return gate_check(s, true, kNow, c);
}

// 현재 나이가 정확히 age가 되도록 recv 시각을 맞춤
AiCommandSnapshot with_age(AiCommandSnapshot s, int64_t age) {
  s.age_ns = 0;
  s.recv_mono_ns = kNow - age;
  return s;
}

const GateConfig kCfg{};

// ---- 1. 정상 ----
TEST(Gate, GoodCommandPasses) {
  const GateResult r = run(good_cmd(kCfg));
  EXPECT_TRUE(r.pass);
  EXPECT_EQ(r.reasons, 0u);
  EXPECT_EQ(r.age_ns, 10'000'000);  // 5ms + 5ms
}

// ---- 2. 검사 8가지 각각 통과 / 차단 ----
TEST(Gate, NoData) {
  EXPECT_TRUE(gate_check(good_cmd(kCfg), true, kNow, kCfg).pass);
  // 받은 적 없으면 내용이 전부 0이어도 NO_DATA만 켜짐
  const GateResult r = gate_check(AiCommandSnapshot{}, false, kNow, kCfg);
  EXPECT_FALSE(r.pass);
  EXPECT_EQ(r.reasons, rt_control::GATE_NO_DATA);
  EXPECT_EQ(r.age_ns, 0);
}

TEST(Gate, Version) {
  AiCommandSnapshot s = good_cmd(kCfg);
  EXPECT_TRUE(run(s).pass);
  s.layout_version = 2;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_VERSION);
}

TEST(Gate, Type) {
  AiCommandSnapshot s = good_cmd(kCfg);
  EXPECT_TRUE(run(s).pass);
  s.command_type = 2;  // path_adjust
  EXPECT_EQ(run(s).reasons, rt_control::GATE_TYPE);
}

TEST(Gate, Status) {
  AiCommandSnapshot s = good_cmd(kCfg);
  EXPECT_TRUE(run(s).pass);
  s.status = 1;  // ST_NOT_DETECTED
  EXPECT_EQ(run(s).reasons, rt_control::GATE_STATUS);
}

TEST(Gate, Expired) {
  EXPECT_TRUE(run(with_age(good_cmd(kCfg), 100'000'000)).pass);
  EXPECT_EQ(run(with_age(good_cmd(kCfg), 300'000'000)).reasons, rt_control::GATE_EXPIRED);
}

TEST(Gate, ExpiredUsesMessageAgePlusElapsed) {
  // A 쪽 나이 150ms + 받은 지 60ms = 210ms -> 만료
  AiCommandSnapshot s = good_cmd(kCfg);
  s.age_ns = 150'000'000;
  s.recv_mono_ns = kNow - 60'000'000;
  const GateResult r = run(s);
  EXPECT_EQ(r.reasons, rt_control::GATE_EXPIRED);
  EXPECT_EQ(r.age_ns, 210'000'000);
}

TEST(Gate, NanAndInf) {
  AiCommandSnapshot s = good_cmd(kCfg);
  EXPECT_TRUE(run(s).pass);
  s.confidence = kInf;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_NAN);
}

TEST(Gate, LowConfidence) {
  AiCommandSnapshot s = good_cmd(kCfg);
  s.confidence = 0.9;
  EXPECT_TRUE(run(s).pass);
  s.confidence = 0.3;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_LOW_CONF);
}

TEST(Gate, Range) {
  AiCommandSnapshot s = good_cmd(kCfg);
  s.target[0] += 0.03;
  EXPECT_TRUE(run(s).pass);
  s.target[0] += 0.03;  // 6cm
  EXPECT_EQ(run(s).reasons, rt_control::GATE_RANGE);
}

TEST(Gate, RangeIgnoresTarget3) {
  AiCommandSnapshot s = good_cmd(kCfg);
  s.target[3] = 100.0;  // 시나리오 1에서는 쓰지 않는 칸
  EXPECT_TRUE(run(s).pass);
}

TEST(Gate, PlannedPositionFromConfig) {
  // 가짜 A가 보내는 [0.5, 0.0, 0.71]: 기본 계획 위치에서는 RANGE, 계획 위치를 바꾸면 통과
  AiCommandSnapshot s = good_cmd(kCfg);
  s.target[0] = 0.5;
  s.target[1] = 0.0;
  s.target[2] = 0.71;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_RANGE);
  GateConfig c;
  c.planned_position[2] = 0.71;
  EXPECT_TRUE(run(s, c).pass);
}

// ---- 3. 경계값 ----
TEST(GateBoundary, AgeTtl) {
  EXPECT_TRUE(run(with_age(good_cmd(kCfg), kCfg.ttl_ns - 1)).pass);
  EXPECT_TRUE(run(with_age(good_cmd(kCfg), kCfg.ttl_ns)).pass);  // "초과"만 차단
  EXPECT_EQ(run(with_age(good_cmd(kCfg), kCfg.ttl_ns + 1)).reasons, rt_control::GATE_EXPIRED);
}

TEST(GateBoundary, DistanceRadius) {
  // y축만 움직여서 거리가 정확히 0.05가 되게 함 (계획 y = 0.0)
  AiCommandSnapshot s = good_cmd(kCfg);
  s.target[1] = 0.05;
  EXPECT_TRUE(run(s).pass);
  s.target[1] = -0.05;
  EXPECT_TRUE(run(s).pass);
  s.target[1] = 0.0501;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_RANGE);
}

TEST(GateBoundary, ConfidenceMin) {
  AiCommandSnapshot s = good_cmd(kCfg);
  s.confidence = 0.5;
  EXPECT_TRUE(run(s).pass);
  s.confidence = std::nextafter(0.5, 0.0);  // 0.5보다 아주 조금 작음
  EXPECT_EQ(run(s).reasons, rt_control::GATE_LOW_CONF);
}

// ---- 4. NaN이면 NAN과 RANGE 둘 다 ----
TEST(GateNan, NanTargetBlocksNanAndRange) {
  for (int i = 0; i < 3; ++i) {
    AiCommandSnapshot s = good_cmd(kCfg);
    s.target[i] = kNaN;
    EXPECT_EQ(run(s).reasons, rt_control::GATE_NAN | rt_control::GATE_RANGE) << "target[" << i << "]";
  }
}

TEST(GateNan, InfTargetBlocksNanAndRange) {
  AiCommandSnapshot s = good_cmd(kCfg);
  s.target[2] = -kInf;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_NAN | rt_control::GATE_RANGE);
}

TEST(GateNan, NanConfidenceBlocksNanAndLowConf) {
  AiCommandSnapshot s = good_cmd(kCfg);
  s.confidence = kNaN;
  EXPECT_EQ(run(s).reasons, rt_control::GATE_NAN | rt_control::GATE_LOW_CONF);
}

// ---- 5. 여러 이유 동시 ----
TEST(GateMulti, AllSevenBitsAtOnce) {
  AiCommandSnapshot s{};
  s.layout_version = 9;
  s.command_type = 2;
  s.status = 3;
  s.target[0] = kNaN;
  s.confidence = 0.1;
  s.age_ns = 0;
  s.recv_mono_ns = kNow - 1'000'000'000;
  const GateResult r = run(s);
  EXPECT_FALSE(r.pass);
  EXPECT_EQ(r.reasons, rt_control::GATE_VERSION | rt_control::GATE_TYPE | rt_control::GATE_STATUS |
                           rt_control::GATE_EXPIRED | rt_control::GATE_NAN |
                           rt_control::GATE_LOW_CONF | rt_control::GATE_RANGE);
}

TEST(GateMulti, NotDetectedLikeFakeA) {
  // 가짜 A의 not_detected: status=1, confidence=0, target=0
  AiCommandSnapshot s = good_cmd(kCfg);
  s.status = 1;
  s.confidence = 0.0;
  s.target[0] = s.target[1] = s.target[2] = 0.0;
  EXPECT_EQ(run(s).reasons,
            rt_control::GATE_STATUS | rt_control::GATE_LOW_CONF | rt_control::GATE_RANGE);
}

}  // namespace
