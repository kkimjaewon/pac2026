#include "rt_control/gate.hpp"

#include <cmath>

namespace rt_control {

namespace {

// 조건이 참이면 bit, 거짓이면 0 (분기 없이 곱셈으로)
inline uint32_t bit_if(bool cond, uint32_t bit) { return static_cast<uint32_t>(cond) * bit; }

}  // namespace

GateResult gate_check(const AiCommandSnapshot& cmd, bool has_data, int64_t now_mono_ns,
                      const GateConfig& config) {
  const int64_t age = cmd.age_ns + (now_mono_ns - cmd.recv_mono_ns);

  const bool finite = std::isfinite(cmd.target[0]) & std::isfinite(cmd.target[1]) &
                      std::isfinite(cmd.target[2]) & std::isfinite(cmd.confidence);

  // 거리는 제곱끼리 비교 (sqrt 없이). NaN이 섞이면 dist2도 NaN이 되고,
  // NaN과의 비교는 항상 거짓이므로 "!(dist2 <= r2)" 꼴로 써야 NaN이 차단된다.
  const double dx = cmd.target[0] - config.planned_position[0];
  const double dy = cmd.target[1] - config.planned_position[1];
  const double dz = cmd.target[2] - config.planned_position[2];
  const double dist2 = dx * dx + dy * dy + dz * dz;
  const double r2 = config.accept_radius * config.accept_radius;

  const uint32_t checks =
      bit_if(cmd.layout_version != config.expected_layout_version, GATE_VERSION) |
      bit_if(cmd.command_type != config.expected_command_type, GATE_TYPE) |
      bit_if(cmd.status != kStatusOk, GATE_STATUS) |
      bit_if(age > config.ttl_ns, GATE_EXPIRED) |
      bit_if(!finite, GATE_NAN) |
      bit_if(!(cmd.confidence >= config.confidence_min), GATE_LOW_CONF) |
      bit_if(!(dist2 <= r2), GATE_RANGE);

  // 받은 적 없으면 검사 결과를 모두 지우고 NO_DATA만 남김 (mask = 0 또는 전부 1)
  const uint32_t data_mask = 0u - static_cast<uint32_t>(has_data);
  const uint32_t reasons = (checks & data_mask) | bit_if(!has_data, GATE_NO_DATA);

  return GateResult{reasons == 0, reasons, age * static_cast<int64_t>(has_data)};
}

}  // namespace rt_control
