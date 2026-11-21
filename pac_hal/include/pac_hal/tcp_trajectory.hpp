// Copyright 2026 Jaewon Kim
#pragma once
#include <array>
#include <cmath>
#include <sstream>
#include "pac_hal/hal_types.hpp"
namespace pac_hal {
// Derivative baseline for the next target. Once the gap since the last sent
// target reaches the receiver watchdog, the receiver has stopped following the
// old stream: restart from rest and bound the interval, so a long idle cannot
// admit an arbitrary jump and an old speed cannot block a stationary hold.
struct MotionBaseline {double dt; double previous_dt; bool restarted;};
inline MotionBaseline motion_baseline(bool has_previous,int64_t created_ns,int64_t last_created_ns,
  double last_dt,int64_t period_ns,int64_t watchdog_ns) {
  const double period=period_ns*1e-9;
  if(!has_previous) {return {period,period,false};}
  const int64_t gap=created_ns-last_created_ns;
  if(gap>=watchdog_ns) {return {watchdog_ns*1e-9,period,true};}
  return {gap*1e-9,last_dt,false};
}
// Cartesian limits are vector norms in map/metres, independent of robot DOF.
// Use original command creation times and only the last successfully sent target.
inline Result check_tcp_trajectory(const Config & config,
  const geometry_msgs::msg::Pose & candidate,const geometry_msgs::msg::Pose & previous,
  const std::array<double,3> & previous_velocity,double dt,double previous_dt,
  std::array<double,3> & velocity) {
  if(!std::isfinite(dt) || !std::isfinite(previous_dt) || dt<=0 || previous_dt<=0) {
    return {Error::E_FORMAT,"TCP target creation time must increase"};
  }
  const auto & a=candidate.position;const auto & b=previous.position;
  velocity={(a.x-b.x)/dt,(a.y-b.y)/dt,(a.z-b.z)/dt};
  std::array<double,3> acceleration{};
  for(size_t i=0;i<3;++i) {acceleration[i]=2*(velocity[i]-previous_velocity[i])/(dt+previous_dt);}
  const double speed=std::hypot(velocity[0],velocity[1],velocity[2]);
  const double accel=std::hypot(acceleration[0],acceleration[1],acceleration[2]);
  if(!std::isfinite(speed) || !std::isfinite(accel) || speed>config.max_tcp_speed+1e-6 ||
    accel>config.max_tcp_acceleration+1e-5) {
    std::ostringstream detail;detail<<"TCP trajectory limit exceeded: speed_m_s="<<speed
      <<" acceleration_m_s2="<<accel<<" max_speed_m_s="<<config.max_tcp_speed
      <<" max_acceleration_m_s2="<<config.max_tcp_acceleration;
    return {Error::E_OUT_OF_RANGE,detail.str()};
  }
  return {};
}
}
