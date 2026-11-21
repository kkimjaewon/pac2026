// 시작 환경 점검(env_warnings) 단위 테스트
#include <gtest/gtest.h>

#include <string>

#include "rt_control/env_check.hpp"

namespace {

using rt_control::EnvInfo;
using rt_control::env_warnings;

// 시연 PC 같은 정상 환경
EnvInfo good() {
  EnvInfo e;
  e.kernel_known = true;
  e.preempt_rt = true;
  e.kernel_release = "6.8.1-1061-realtime";
  e.power_profile = "performance";
  e.rtprio_known = true;
  e.rtprio_limit = 99;
  e.fifo_requested = true;
  return e;
}

bool has(const std::vector<std::string>& w, const char* word) {
  for (const auto& s : w) {
    if (s.find(word) != std::string::npos) return true;
  }
  return false;
}

TEST(EnvCheck, GoodEnvironmentHasNoWarnings) { EXPECT_TRUE(env_warnings(good()).empty()); }

TEST(EnvCheck, NoPreemptRt) {
  EnvInfo e = good();
  e.preempt_rt = false;
  e.kernel_release = "6.8.0-146-generic";
  const auto w = env_warnings(e);
  ASSERT_EQ(w.size(), 1u);
  EXPECT_TRUE(has(w, "PREEMPT_RT"));
  EXPECT_TRUE(has(w, "6.8.0-146-generic"));
}

TEST(EnvCheck, PowerProfileNotPerformance) {
  for (const char* p : {"balanced", "power-saver", "low-power"}) {
    EnvInfo e = good();
    e.power_profile = p;
    const auto w = env_warnings(e);
    ASSERT_EQ(w.size(), 1u) << p;
    EXPECT_TRUE(has(w, p));
    EXPECT_TRUE(has(w, "powerprofilesctl set performance"));
  }
}

// 전원 프로파일을 못 읽으면 건너뜀
TEST(EnvCheck, UnknownPowerProfileIsSkipped) {
  EnvInfo e = good();
  e.power_profile = "";
  EXPECT_TRUE(env_warnings(e).empty());
}

TEST(EnvCheck, FifoWithoutRtprio) {
  EnvInfo e = good();
  e.rtprio_limit = 0;
  const auto w = env_warnings(e);
  ASSERT_EQ(w.size(), 1u);
  EXPECT_TRUE(has(w, "ulimit -r"));
}

// fifo를 안 켰거나 root면 rtprio 0이어도 괜찮음
TEST(EnvCheck, RtprioZeroIsFineWithoutFifoOrAsRoot) {
  EnvInfo e = good();
  e.rtprio_limit = 0;
  e.fifo_requested = false;
  EXPECT_TRUE(env_warnings(e).empty());
  e.fifo_requested = true;
  e.is_root = true;
  EXPECT_TRUE(env_warnings(e).empty());
}

TEST(EnvCheck, AllProblemsAtOnce) {
  EnvInfo e = good();
  e.preempt_rt = false;
  e.power_profile = "balanced";
  e.rtprio_limit = 0;
  EXPECT_EQ(env_warnings(e).size(), 3u);
}

// 지금 PC에서 읽기가 되는지만 확인 (값은 PC마다 다름)
TEST(EnvCheck, ReadDoesNotCrash) {
  const EnvInfo e = rt_control::read_env_info(true);
  EXPECT_TRUE(e.kernel_known);
  EXPECT_FALSE(e.kernel_release.empty());
  EXPECT_TRUE(e.fifo_requested);
}

}  // namespace
