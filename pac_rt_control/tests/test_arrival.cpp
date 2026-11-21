// 도착 판정(ArrivalDetector)과 재개 기준(choose_resume_position) 단위 테스트
#include <gtest/gtest.h>

#include <cmath>

#include "rt_control/arrival.hpp"

namespace {

using rt_control::ArrivalConfig;
using rt_control::ArrivalDetector;
using rt_control::ArrivalResult;
using rt_control::ResumeChoice;
using rt_control::ResumeSource;

constexpr int64_t kMs = 1'000'000;
constexpr double kGoal[3] = {0.5, 0.0, 0.7};

// 측정 TCP를 goal에서 x로 dx만큼 떨어뜨려 t0부터 t_end까지 1ms마다 넣고 마지막 결과를 돌려줌
ArrivalResult run_until(ArrivalDetector& det, double dx, int64_t t0, int64_t t_end) {
  const double tcp[3] = {kGoal[0] + dx, kGoal[1], kGoal[2]};
  ArrivalResult r{};
  for (int64_t t = t0; t <= t_end; t += kMs) r = det.update(kGoal, tcp, true, t);
  return r;
}

TEST(Arrival, DefaultsAgreedWithC) {
  const ArrivalConfig c;
  EXPECT_DOUBLE_EQ(c.tol_m, 0.002);
  EXPECT_EQ(c.hold_ns, 300 * kMs);
  EXPECT_DOUBLE_EQ(rt_control::kDefaultResumeMatchTolM, 0.002);
}

TEST(Arrival, NotArrivedAt299msArrivedAt301ms) {
  ArrivalDetector det;
  const int64_t t0 = 10'000 * kMs;
  // 0.299초 머문 시점: 아직 아님
  const double tcp[3] = {kGoal[0] + 0.001, kGoal[1], kGoal[2]};
  EXPECT_FALSE(det.update(kGoal, tcp, true, t0).arrived);
  EXPECT_FALSE(det.update(kGoal, tcp, true, t0 + 299 * kMs).arrived);
  // 0.301초: 도착
  EXPECT_TRUE(det.update(kGoal, tcp, true, t0 + 301 * kMs).arrived);
}

TEST(Arrival, NotArrivedAt299msWithEveryCycle) {
  ArrivalDetector det;
  const int64_t t0 = 5'000 * kMs;
  EXPECT_FALSE(run_until(det, 0.0015, t0, t0 + 299 * kMs).arrived);
  const double tcp[3] = {kGoal[0] + 0.0015, kGoal[1], kGoal[2]};
  EXPECT_TRUE(det.update(kGoal, tcp, true, t0 + 301 * kMs).arrived);
}

TEST(Arrival, TwoMillimeterBoundary) {
  // 정확히 2mm는 "안" (<=), 그보다 조금 크면 밖
  {
    // 목표 원점, 측정 x = 0.002 -> 오차가 정확히 0.002
    ArrivalDetector det;
    const double origin[3] = {0.0, 0.0, 0.0};
    const double tcp[3] = {0.002, 0.0, 0.0};
    ArrivalResult r{};
    for (int64_t t = 0; t <= 400 * kMs; t += kMs) r = det.update(origin, tcp, true, t);
    EXPECT_EQ(r.error_m, 0.002);
    EXPECT_TRUE(r.arrived);
  }
  {
    ArrivalDetector det;
    EXPECT_TRUE(run_until(det, 0.0019999, 0, 400 * kMs).arrived);
  }
  {
    ArrivalDetector det;
    const ArrivalResult r = run_until(det, 0.0020001, 0, 400 * kMs);
    EXPECT_NEAR(r.error_m, 0.0020001, 1e-12);
    EXPECT_FALSE(r.arrived);
  }
}

TEST(Arrival, LeavingToleranceRestartsTimer) {
  ArrivalDetector det;
  EXPECT_TRUE(run_until(det, 0.001, 0, 350 * kMs).arrived);
  // 한 주기라도 밖으로 나가면 처음부터 다시 셈
  EXPECT_FALSE(run_until(det, 0.003, 351 * kMs, 351 * kMs).arrived);
  EXPECT_FALSE(run_until(det, 0.001, 352 * kMs, 352 * kMs + 299 * kMs).arrived);
  EXPECT_TRUE(run_until(det, 0.001, 352 * kMs + 300 * kMs, 352 * kMs + 300 * kMs).arrived);
}

TEST(Arrival, InvalidTcpGivesNaNAndRestartsTimer) {
  ArrivalDetector det;
  EXPECT_TRUE(run_until(det, 0.0, 0, 400 * kMs).arrived);
  const double junk[3] = {0, 0, 0};
  const ArrivalResult r = det.update(kGoal, junk, false, 401 * kMs);
  EXPECT_FALSE(r.arrived);
  EXPECT_TRUE(std::isnan(r.error_m));
  EXPECT_FALSE(run_until(det, 0.0, 402 * kMs, 402 * kMs + 299 * kMs).arrived);
}

TEST(Arrival, ResetClearsTimer) {
  ArrivalDetector det;
  EXPECT_TRUE(run_until(det, 0.0, 0, 400 * kMs).arrived);
  det.reset();
  EXPECT_FALSE(run_until(det, 0.0, 401 * kMs, 401 * kMs + 299 * kMs).arrived);
}

// ---------------- 재개 기준 ----------------

constexpr double kLast[3] = {0.40, 0.10, 0.60};
constexpr double kOut[3] = {0.41, 0.11, 0.61};

TEST(Resume, WithinTolUsesLastCommand) {
  const double robot[3] = {kLast[0] + 0.0015, kLast[1], kLast[2]};
  const ResumeChoice c = rt_control::choose_resume_position(true, 0.002, true, kLast, true, robot, kOut);
  EXPECT_EQ(c.source, ResumeSource::kLastCommand);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(c.pos[i], kLast[i]);
  EXPECT_NEAR(c.diff_m, 0.0015, 1e-12);
}

TEST(Resume, ExactlyTolUsesLastCommand) {
  // 한 축 차이 0.002는 sqrt(0.002²) = 0.002 로 정확히 계산됨 -> 경계값은 "안"
  const double robot[3] = {0.0, 0.0, 0.0};
  const double last[3] = {0.002, 0.0, 0.0};
  const ResumeChoice c = rt_control::choose_resume_position(true, 0.002, true, last, true, robot, kOut);
  EXPECT_EQ(c.source, ResumeSource::kLastCommand);
  EXPECT_EQ(c.diff_m, 0.002);
}

TEST(Resume, OutsideTolUsesMeasuredAndFlagsMismatch) {
  const double robot[3] = {kLast[0], kLast[1] + 0.0025, kLast[2]};
  const ResumeChoice c = rt_control::choose_resume_position(true, 0.002, true, kLast, true, robot, kOut);
  EXPECT_EQ(c.source, ResumeSource::kRobotMismatch);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(c.pos[i], robot[i]);
  EXPECT_NEAR(c.diff_m, 0.0025, 1e-12);
}

TEST(Resume, DisabledUsesMeasured) {
  const double robot[3] = {kLast[0] + 0.0005, kLast[1], kLast[2]};
  const ResumeChoice c = rt_control::choose_resume_position(false, 0.002, true, kLast, true, robot, kOut);
  EXPECT_EQ(c.source, ResumeSource::kRobot);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(c.pos[i], robot[i]);
  EXPECT_TRUE(std::isnan(c.diff_m));
}

TEST(Resume, NoCommandSentYetUsesMeasured) {
  const double robot[3] = {kLast[0], kLast[1], kLast[2]};
  const ResumeChoice c = rt_control::choose_resume_position(true, 0.002, false, kLast, true, robot, kOut);
  EXPECT_EQ(c.source, ResumeSource::kRobot);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(c.pos[i], robot[i]);
}

TEST(Resume, NoRobotUsesOutputLikeBefore) {
  const double robot[3] = {0, 0, 0};
  const ResumeChoice c = rt_control::choose_resume_position(true, 0.002, true, kLast, false, robot, kOut);
  EXPECT_EQ(c.source, ResumeSource::kOutput);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(c.pos[i], kOut[i]);
}

}  // namespace
