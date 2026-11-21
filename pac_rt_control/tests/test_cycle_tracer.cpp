// cycle_tracer 단위 테스트: rt.csv 헤더 순서와 TraceRow -> CSV 한 줄 변환
#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "rt_control/cycle_tracer.hpp"
#include "rt_control/gate.hpp"
#include "rt_control/safety_mode.hpp"

namespace {

using rt_control::TraceRow;

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string cell;
  while (std::getline(ss, cell, ',')) out.push_back(cell);
  if (!s.empty() && s.back() == ',') out.push_back("");  // 마지막 빈 칸
  return out;
}

std::string to_csv(const TraceRow& r) {
  char buf[512];
  const int n = rt_control::trace_row_to_csv(r, buf, sizeof(buf));
  EXPECT_GT(n, 0);
  EXPECT_LT(n, static_cast<int>(sizeof(buf)));
  return std::string(buf, static_cast<size_t>(n));
}

// 계약 v0.3 rt.csv 필수 컬럼 (이름·순서 그대로)
const std::vector<std::string> kContractColumns = {
    "cycle",       "wake_mono_ns", "compute_ns",      "overrun",   "mode",      "used_ai_seq",
    "weight",      "goal_x",       "goal_y",          "goal_z",    "y_x",       "y_y",
    "y_z",         "command_seq",  "ai_recv_mono_ns", "ai_age_ns", "ai_apply_mono_ns", "reject_reason"};
const std::vector<std::string> kExtraColumns = {"jitter_ns", "mode_reason", "speed",          "has_used",
                                                "arrived",   "tcp_error_m", "used_ai_boot_id", "rejected_wait"};

TEST(CycleTracer, HeaderStartsWithContractColumnsInOrder) {
  const std::vector<std::string> cols = split(rt_control::kTraceCsvHeader);
  ASSERT_EQ(cols.size(), kContractColumns.size() + kExtraColumns.size());
  for (size_t i = 0; i < kContractColumns.size(); ++i) EXPECT_EQ(cols[i], kContractColumns[i]) << i;
  for (size_t i = 0; i < kExtraColumns.size(); ++i) {
    EXPECT_EQ(cols[kContractColumns.size() + i], kExtraColumns[i]) << i;
  }
}

TEST(CycleTracer, RowToCsv) {
  TraceRow r{};
  r.cycle = 1234;
  r.wake_mono_ns = 5'000'000'123;
  r.compute_ns = 4567;
  r.overrun = 2;
  r.mode = rt_control::TRACE_HOLD;
  r.used_ai_seq = 77;
  r.weight = 0.68;
  r.goal[0] = 0.5;
  r.goal[1] = -0.012345678;
  r.goal[2] = 0.71;
  r.y[0] = 0.45;
  r.y[1] = 0.0;
  r.y[2] = 0.75;
  r.command_seq = 999;
  r.ai_recv_mono_ns = 4'990'000'000;
  r.ai_age_ns = 64'000'000;
  r.ai_apply_mono_ns = 5'000'000'123;
  r.reject_reason = rt_control::GATE_EXPIRED;
  r.jitter_ns = 12'345;
  r.mode_reason = rt_control::SAFE_AI_EXPIRED | rt_control::SAFE_AI_RANGE;
  r.speed = 0.1;
  r.has_used = 1;
  r.arrived = 1;
  r.tcp_error_m = 0.000312;
  r.used_ai_boot_id = 4'000'000'001u;
  r.rejected_wait = 1;

  const std::string line = to_csv(r);
  ASSERT_EQ(line.back(), '\n');
  const std::vector<std::string> c = split(line.substr(0, line.size() - 1));
  ASSERT_EQ(c.size(), kContractColumns.size() + kExtraColumns.size());
  EXPECT_EQ(c[0], "1234");
  EXPECT_EQ(c[1], "5000000123");
  EXPECT_EQ(c[2], "4567");
  EXPECT_EQ(c[3], "2");
  EXPECT_EQ(c[4], "HOLD");
  EXPECT_EQ(c[5], "77");
  EXPECT_DOUBLE_EQ(std::stod(c[6]), 0.68);
  EXPECT_DOUBLE_EQ(std::stod(c[7]), 0.5);
  EXPECT_NEAR(std::stod(c[8]), -0.012345678, 1e-9);
  EXPECT_DOUBLE_EQ(std::stod(c[9]), 0.71);
  EXPECT_DOUBLE_EQ(std::stod(c[10]), 0.45);
  EXPECT_DOUBLE_EQ(std::stod(c[11]), 0.0);
  EXPECT_DOUBLE_EQ(std::stod(c[12]), 0.75);
  EXPECT_EQ(c[13], "999");
  EXPECT_EQ(c[14], "4990000000");
  EXPECT_EQ(c[15], "64000000");
  EXPECT_EQ(c[16], "5000000123");
  EXPECT_EQ(c[17], "0x10");  // GATE_EXPIRED = 1 << 4
  EXPECT_EQ(c[18], "12345");
  EXPECT_EQ(c[19], "EXPIRED|RANGE");
  EXPECT_DOUBLE_EQ(std::stod(c[20]), 0.1);
  EXPECT_EQ(c[21], "1");
  EXPECT_EQ(c[22], "1");
  EXPECT_DOUBLE_EQ(std::stod(c[23]), 0.000312);
  EXPECT_EQ(c[24], "4000000001");
  EXPECT_EQ(c[25], "1");
}

// 측정 TCP가 없으면 tcp_error_m은 nan, 반영 전이면 used_ai_boot_id 0
TEST(CycleTracer, NoMeasurementAndNoUsedCommand) {
  TraceRow r{};
  r.tcp_error_m = std::numeric_limits<double>::quiet_NaN();
  const std::string line = to_csv(r);
  const std::vector<std::string> c = split(line.substr(0, line.size() - 1));
  ASSERT_EQ(c.size(), kContractColumns.size() + kExtraColumns.size());
  EXPECT_EQ(c[22], "0");
  EXPECT_EQ(c[23], "nan");
  EXPECT_EQ(c[24], "0");
  EXPECT_EQ(c[25], "0");
}

// 모드 글자, 이유 없음은 빈 칸, 반영 안 한 주기는 weight·apply 0
TEST(CycleTracer, ModeNamesAndEmptyReason) {
  const struct {
    uint8_t mode;
    const char* name;
  } cases[] = {{rt_control::TRACE_RUN, "RUN"},
               {rt_control::TRACE_HOLD, "HOLD"},
               {rt_control::TRACE_DECEL_STOP, "DECEL_STOP"},
               {rt_control::TRACE_FAULT, "FAULT"},
               {rt_control::TRACE_INIT, "INIT"}};
  for (const auto& m : cases) {
    TraceRow r{};
    r.mode = m.mode;
    const std::vector<std::string> c = split(to_csv(r).substr(0, to_csv(r).size() - 1));
    ASSERT_EQ(c.size(), 26u);
    EXPECT_EQ(c[4], m.name);
    EXPECT_EQ(c[6], "0.000000");
    EXPECT_EQ(c[16], "0");
    EXPECT_EQ(c[17], "0x00");
    EXPECT_EQ(c[19], "");
  }
}

// TraceMode 번호가 SafetyMode와 같아서 그대로 넣어도 됨
TEST(CycleTracer, TraceModeMatchesSafetyMode) {
  EXPECT_EQ(rt_control::TRACE_RUN, static_cast<uint8_t>(rt_control::SafetyMode::kRun));
  EXPECT_EQ(rt_control::TRACE_HOLD, static_cast<uint8_t>(rt_control::SafetyMode::kHold));
  EXPECT_EQ(rt_control::TRACE_DECEL_STOP, static_cast<uint8_t>(rt_control::SafetyMode::kDecelStop));
  EXPECT_EQ(rt_control::TRACE_FAULT, static_cast<uint8_t>(rt_control::SafetyMode::kFault));
}

}  // namespace
