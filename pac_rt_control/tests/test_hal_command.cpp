// HalCommand 스냅샷 채우기(fill_hal_command) 단위 테스트
#include <gtest/gtest.h>

#include "rt_control/hal_command_snapshot.hpp"

namespace {

using rt_control::fill_hal_command;
using rt_control::HalCommandConfig;
using rt_control::HalCommandSnapshot;

constexpr uint64_t kSession = 123'456'789;
constexpr double kPos[3] = {0.45, 0.01, 0.75};

// 기본 설정: TTL 5ms, 자세 (1, 0, 0, 0)
TEST(HalCommand, DefaultConfigIs5msAndFixedOrientation) {
  const HalCommandConfig c;
  EXPECT_EQ(c.hal_ttl_ns, 5'000'000);
  EXPECT_DOUBLE_EQ(c.tool_orientation[0], 1.0);
  EXPECT_DOUBLE_EQ(c.tool_orientation[1], 0.0);
  EXPECT_DOUBLE_EQ(c.tool_orientation[2], 0.0);
  EXPECT_DOUBLE_EQ(c.tool_orientation[3], 0.0);
}

// 새 세션의 첫 명령은 1, 그다음부터 주기마다 1씩
TEST(HalCommand, SeqStartsAt1AndIncrementsByOne) {
  HalCommandSnapshot cmd{};
  const HalCommandConfig c;
  for (uint64_t i = 1; i <= 5; ++i) {
    fill_hal_command(cmd, kSession, 1'000'000 * static_cast<int64_t>(i), kPos, false, 0, c);
    EXPECT_EQ(cmd.command_seq, i);
  }
}

// expires = created + 5ms, created = 넘겨준 주기 시각
TEST(HalCommand, ExpiresIsCreatedPlusTtl) {
  HalCommandSnapshot cmd{};
  const HalCommandConfig c;
  const int64_t now = 987'654'321'000;
  fill_hal_command(cmd, kSession, now, kPos, false, 0, c);
  EXPECT_EQ(cmd.created_mono_ns, now);
  EXPECT_EQ(cmd.expires_mono_ns, now + 5'000'000);
  EXPECT_EQ(cmd.expires_mono_ns - cmd.created_mono_ns, 5'000'000);
}

// 설정한 TTL을 그대로 씀
TEST(HalCommand, CustomTtl) {
  HalCommandSnapshot cmd{};
  HalCommandConfig c;
  c.hal_ttl_ns = 10'000'000;
  fill_hal_command(cmd, kSession, 1000, kPos, false, 0, c);
  EXPECT_EQ(cmd.expires_mono_ns, 1000 + 10'000'000);
}

// has_used가 false면 used_ai_seq가 무엇이든 source_result_id = 0
TEST(HalCommand, SourceIsZeroWhenNotUsed) {
  HalCommandSnapshot cmd{};
  const HalCommandConfig c;
  fill_hal_command(cmd, kSession, 1000, kPos, false, 42, c);
  EXPECT_EQ(cmd.source_result_id, 0u);
}

// has_used가 true면 source_result_id = used_ai_seq
TEST(HalCommand, SourceIsUsedSeqWhenUsed) {
  HalCommandSnapshot cmd{};
  const HalCommandConfig c;
  fill_hal_command(cmd, kSession, 1000, kPos, true, 42, c);
  EXPECT_EQ(cmd.source_result_id, 42u);
}

// 위치·자세·세션을 그대로 복사
TEST(HalCommand, CopiesPositionOrientationSession) {
  HalCommandSnapshot cmd{};
  HalCommandConfig c;
  c.tool_orientation[0] = 0.0;
  c.tool_orientation[3] = 1.0;
  fill_hal_command(cmd, kSession, 1000, kPos, false, 0, c);
  EXPECT_EQ(cmd.session_id, kSession);
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(cmd.pos[i], kPos[i]);
  EXPECT_DOUBLE_EQ(cmd.quat[0], 0.0);
  EXPECT_DOUBLE_EQ(cmd.quat[3], 1.0);
}

}  // namespace
