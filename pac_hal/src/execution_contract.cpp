// Copyright 2026 Jaewon Kim
#include "pac_hal/execution_contract.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
namespace pac_hal {
std::string json_string(const std::string & value) {
  std::ostringstream out;out<<'"';
  for (const unsigned char byte : value) {
    switch (byte) {
      case '"':out<<"\\\"";break;
      case '\\':out<<"\\\\";break;
      case '\b':out<<"\\b";break;
      case '\f':out<<"\\f";break;
      case '\n':out<<"\\n";break;
      case '\r':out<<"\\r";break;
      case '\t':out<<"\\t";break;
      default:
        if (byte<0x20) {out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned>(byte)<<std::dec;}
        else {out<<static_cast<char>(byte);}
    }
  }
  out<<'"';return out.str();
}
std::string describe_robot_contract(const Config & c) {
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  auto transform=[&out](const geometry_msgs::msg::Pose & pose) {
    const auto & p=pose.position;const auto & q=pose.orientation;
    out<<"{\"translation_m\":["<<p.x<<','<<p.y<<','<<p.z
       <<"],\"rotation_xyzw\":["<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<"]}";
  };
  out<<"{\"reference_frame\":"<<json_string(c.reference)
     <<",\"base_frame\":"<<json_string(c.base)<<",\"flange_frame\":"<<json_string(c.flange)
     <<",\"declared_tcp_frame\":"<<json_string(c.tcp)<<",\"reference_to_base\":";
  transform(c.reference_to_base);out<<",\"flange_to_tcp\":";transform(c.flange_to_tcp);
  out<<",\"motion_limits\":{\"max_tcp_speed_m_s\":"<<c.max_tcp_speed
     <<",\"max_tcp_acceleration_m_s2\":"<<c.max_tcp_acceleration<<"}"
     <<",\"capabilities\":{\"absolute_tcp_position\":"<<c.position_capability
     <<",\"measured_joint_feedback\":"<<c.feedback_capability
     <<",\"receiver_watchdog\":"<<c.watchdog_capability<<"}}";
  return out.str();
}
std::string describe_task_contract(const Config & c) {
  std::ostringstream out;
  out<<"{\"frame\":"<<json_string(c.reference)<<",\"input_semantics\":\"absolute_tcp_pose\"}";
  return out.str();
}
Result check_task(const Config & c) {
  if (!c.position_capability || !c.feedback_capability || !c.watchdog_capability ||
    c.task.min_dof > c.joint_count || c.backend_period_ns > c.task.maximum_period_ns ||
    c.backend_period_ns < c.period_ns || c.backend_period_ns >= c.ttl_ns) {
    return {Error::E_UNSUPPORTED, "CAPABILITY_MISMATCH: task exceeds validated backend"};
  }
  return {};
}
Result preflight(const Config & c, HalConverter & converter, SceneGuard & scene,
  const Measurement & measured) {
  if (auto result = check_task(c); !result) {return result;}
  if (!converter.ready() || !scene.ready()) {return {Error::E_TF, "preflight model/scene not initialized"};}
  geometry_msgs::msg::Pose current;geometry_msgs::msg::Twist twist;
  if (auto result = converter.forward(measured, current, twist); !result) {return result;}
  // Dedicated startup solver: a 20ms computation budget, never a production motion budget.
  // The converter is initialized from the same immutable model by the caller.
  // Runtime and startup use separate converters; caller passes the startup instance.
  auto check = [&](const geometry_msgs::msg::Pose & pose) -> Result {
    const auto & p = pose.position;const auto & q = pose.orientation;
    const std::array<double,3> xyz{p.x,p.y,p.z};
    const double norm=std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w));
    if (!std::isfinite(norm) || std::abs(norm-1.)>1e-6) {return {Error::E_FORMAT,"preflight invalid quaternion"};}
    for(size_t i=0;i<3;++i) {
      if(!std::isfinite(xyz[i])) {return {Error::E_FORMAT,"preflight nonfinite target"};}
      if(xyz[i]<c.workspace_min[i] || xyz[i]>c.workspace_max[i]) {return {Error::E_OUT_OF_RANGE,"preflight task sample outside workspace"};}
    }
    Joints solution{};
    if (auto result=converter.inverse(pose,measured.position,solution);!result) {return result;}
    return scene.check(solution); // Samples are feasibility checks, not a planned approach path.
  };
  if(auto result=scene.check(measured.position);!result) {return result;}
  if(auto result=check(current);!result) {return result;}
  for(const auto & pose:c.task.samples) {if(auto result=check(pose);!result) {return result;}}
  return {};
}
}
