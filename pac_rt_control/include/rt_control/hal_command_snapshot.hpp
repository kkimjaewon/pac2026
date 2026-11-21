// B -> C 최종 명령 한 개를 B 안에서 들고 다니는 형식 (ROS 의존 없음)
//
// 1ms 루프가 매 주기 motion_limiter 다음에 fill_hal_command()로 채워 Seqlock 버퍼에 쓰고,
// ros_bridge의 발행 스레드가 그 버퍼를 읽어 /hal/final_command(HalCommand)로 내보낸다.
// 1ms 루프 안에서는 ROS publish를 하지 않는다 (계약: "B의 1ms 루프 안에서 ROS publish 금지").
#pragma once

#include <cstdint>

namespace rt_control {

// [제안] B -> C 유효기한 (C의 계산·전송 예산을 보고 조정). 바꿀 곳은 여기 한 곳 + ROS 파라미터 hal_ttl_ns
constexpr int64_t kDefaultHalTtlNs = 5'000'000;  // 5ms
// [제안] 고정 TCP 자세 쿼터니언 (x, y, z, w). 초기 시연은 자세 고정·위치만 변경
constexpr double kDefaultToolOrientation[4] = {1.0, 0.0, 0.0, 0.0};

struct HalCommandConfig {
  int64_t hal_ttl_ns = kDefaultHalTtlNs;
  double tool_orientation[4] = {kDefaultToolOrientation[0], kDefaultToolOrientation[1],
                                kDefaultToolOrientation[2], kDefaultToolOrientation[3]};
};

// Seqlock 버퍼에 들어가는 HalCommand 한 개 (HalCommand.msg에서 header·mode를 뺀 것)
struct HalCommandSnapshot {
  uint64_t session_id;       // B가 첫 신선한 HalState 때 한 번 발급 (그 순간 CLOCK_MONOTONIC ns), 또는 설정값
  uint64_t command_seq;      // 1ms마다 +1. 세션의 첫 명령이 1 (0 = 아직 없음)
  int64_t created_mono_ns;   // 이 명령을 만든 1ms 주기 시각 (CLOCK_MONOTONIC)
  int64_t expires_mono_ns;   // created + hal_ttl_ns
  double pos[3];             // TCP 목표 위치 (map 기준 [m]) = limiter 출력
  double quat[4];            // TCP 목표 자세 (x, y, z, w)
  uint64_t source_result_id; // 이 목표에 마지막으로 반영된 A 명령 seq (반영한 적 없으면 0)
};

// 한 주기분 명령을 채운다 (순수 함수, 분기 없음). cmd에는 지난 주기 값이 들어 있어야 한다.
//   command_seq를 1 올리고, created = now_ns, expires = now_ns + hal_ttl_ns,
//   pos = limiter 출력, quat = tool_orientation,
//   source_result_id = has_used면 used_ai_seq, 아니면 0
// (A의 seq는 0부터라 "seq 0을 반영함"과 "반영 없음"이 둘 다 0이 된다. 구분은 B 로그의 has_used로)
inline void fill_hal_command(HalCommandSnapshot& cmd, uint64_t session_id, int64_t now_ns,
                             const double pos[3], bool has_used, uint64_t used_ai_seq,
                             const HalCommandConfig& config) {
  cmd.session_id = session_id;
  cmd.command_seq += 1;
  cmd.created_mono_ns = now_ns;
  cmd.expires_mono_ns = now_ns + config.hal_ttl_ns;
  for (int i = 0; i < 3; ++i) cmd.pos[i] = pos[i];
  for (int i = 0; i < 4; ++i) cmd.quat[i] = config.tool_orientation[i];
  cmd.source_result_id = used_ai_seq * static_cast<uint64_t>(has_used);
}

}  // namespace rt_control
