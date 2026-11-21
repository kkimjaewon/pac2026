// 시나리오 1 gate: A 명령을 반영해도 되는지 1ms마다 검사한다 (1단계 유효성 검사)
//
// 순수 함수: 시계 호출·로그·메모리 할당이 없다. 지금 시각은 호출하는 쪽이 넘겨준다.
// 검사 8가지를 모두 계산한 뒤 비트로 합친다. if 분기 없이 비교 결과만 합치므로
// 어떤 입력이 와도 실행 시간이 거의 같다.
//
// path_adjust(시나리오 3)용 범위 검사는 아직 없다. 지금은 command_type이 position이 아니면 TYPE으로 막는다.
#pragma once

#include <cstdint>

#include "rt_control/ai_command_snapshot.hpp"

namespace rt_control {

// intelion_interfaces/msg/ControlCommand의 상수와 같은 값 (gate가 ROS 헤더 없이 빌드되도록 다시 적음)
constexpr uint8_t kCmdPosition = 0;  // CMD_POSITION
constexpr uint8_t kStatusOk = 0;     // ST_OK

// gate와 blend_policy가 함께 쓰는 기본값 (이 한 곳에만 둔다)
constexpr int64_t kDefaultTtlNs = 200'000'000;                    // 200ms
constexpr double kDefaultPlannedPosition[3] = {0.50, 0.00, 0.20};  // 계약 v0.3, map 기준 [m]. weld_cell 철판 위치 확정 시 변경

// gate 설정값. 기본값은 이 한 곳에만 둔다 (rt_control_node가 ROS 파라미터 기본값으로 그대로 씀)
struct GateConfig {
  uint16_t expected_layout_version = 1;
  uint8_t expected_command_type = kCmdPosition;
  int64_t ttl_ns = kDefaultTtlNs;                     // 현재 나이가 이 값을 "넘으면" 만료
  double confidence_min = 0.5;                        // 이 값 "이상"이면 통과
  double planned_position[3] = {kDefaultPlannedPosition[0], kDefaultPlannedPosition[1],
                                kDefaultPlannedPosition[2]};  // map 기준 계획 위치 [m]
  double accept_radius = 0.05;                        // 계획 위치에서 이 거리 "이하"면 통과 [m]
};

// 막힌 이유 비트 (여러 개가 동시에 켜질 수 있음)
enum GateReason : uint32_t {
  GATE_NO_DATA = 1u << 0,   // 아직 한 번도 받은 적 없음
  GATE_VERSION = 1u << 1,   // layout_version 불일치
  GATE_TYPE = 1u << 2,      // command_type 불일치 (시나리오 1은 position만)
  GATE_STATUS = 1u << 3,    // status != ST_OK
  GATE_EXPIRED = 1u << 4,   // 현재 나이 > ttl_ns
  GATE_NAN = 1u << 5,       // target[0..2] 또는 confidence가 NaN·무한대
  GATE_LOW_CONF = 1u << 6,  // confidence < confidence_min (NaN도 여기 걸림)
  GATE_RANGE = 1u << 7,     // 계획 위치와 거리 > accept_radius (NaN도 여기 걸림)
};
constexpr int kGateReasonCount = 8;

// 비트 번호(0~7) -> 이름 (로그용)
constexpr const char* kGateReasonNames[kGateReasonCount] = {
    "NO_DATA", "VERSION", "TYPE", "STATUS", "EXPIRED", "NAN", "LOW_CONF", "RANGE"};

struct GateResult {
  bool pass;         // reasons == 0
  uint32_t reasons;  // GateReason 비트 합
  int64_t age_ns;    // 현재 나이 = age_ns + (now - recv_mono_ns). 받은 적 없으면 0
};

// has_data == false면 NO_DATA 비트만 켜고 나머지 검사 결과는 버린다 (cmd 내용이 의미 없으므로)
GateResult gate_check(const AiCommandSnapshot& cmd, bool has_data, int64_t now_mono_ns,
                      const GateConfig& config);

}  // namespace rt_control
