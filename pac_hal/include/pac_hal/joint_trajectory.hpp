// Copyright 2026 Jaewon Kim
#pragma once

#include <cmath>
#include <sstream>
#include "pac_hal/hal_types.hpp"

namespace pac_hal
{
// 구간 평균 속도는 각 구간의 중간 시각에 해당한다.
// 명령 누락/주기 변동 때도 가속도 분모는 두 구간 길이의 평균이어야 한다.
// 현재 dt만 사용하면 부드러운 경로도 가속도 초과로 오판할 수 있다.
inline Result check_joint_trajectory(const Config & config, const Joints & lower,
  const Joints & upper, const Joints & candidate, const Joints & previous,
  const Joints & previous_velocity, double dt, double previous_dt)
{
  if (!std::isfinite(dt) || !std::isfinite(previous_dt) || dt <= 0 || previous_dt <= 0) {
    return {Error::E_FORMAT, "target creation time must increase"};
  }
  for (size_t i = 0; i < config.joint_count; ++i) {
    if (!std::isfinite(candidate[i]) || candidate[i] < lower[i] || candidate[i] > upper[i]) {
      return {Error::E_OUT_OF_RANGE, "joint position limit exceeded"};
    }
    const double velocity = (candidate[i] - previous[i]) / dt;
    const double acceleration = 2 * (velocity - previous_velocity[i]) / (dt + previous_dt);
    if (std::abs(velocity) > config.axes[i].velocity + 1e-6 ||
      std::abs(acceleration) > config.axes[i].acceleration + 1e-5)
    {
      std::ostringstream detail;
      detail << "joint trajectory limit exceeded: joint=" << i + 1 << " dt=" << dt
             << " previous_dt=" << previous_dt << " velocity=" << velocity
             << " acceleration=" << acceleration << " previous_velocity="
             << previous_velocity[i] << " delta_q=" << candidate[i] - previous[i];
      return {Error::E_OUT_OF_RANGE, detail.str()};
    }
  }
  return {};
}
}  // namespace pac_hal
