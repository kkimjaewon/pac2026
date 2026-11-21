// Robot profile validation is separate from IK and transport.
#include "pac_hal/hal_types.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "yaml-cpp/yaml.h"
namespace pac_hal {
namespace {
bool finite(double x) {return std::isfinite(x);}
geometry_msgs::msg::Pose yaml_pose(const YAML::Node & n)
{
  geometry_msgs::msg::Pose p;
  auto t = n["translation"], q = n["rotation"];
  p.position.x = t["x"].as<double>();
  p.position.y = t["y"].as<double>();
  p.position.z = t["z"].as<double>();
  p.orientation.x = q["x"].as<double>();
  p.orientation.y = q["y"].as<double>();
  p.orientation.z = q["z"].as<double>();
  p.orientation.w = q["w"].as<double>();
  const double norm = std::hypot(std::hypot(p.orientation.x, p.orientation.y),
    std::hypot(p.orientation.z, p.orientation.w));
  if (!finite(norm) || std::abs(norm - 1.) > 1e-6 || !finite(p.position.x) ||
    !finite(p.position.y) || !finite(p.position.z))
  {
    throw std::runtime_error("installation/TCP transform must be finite and normalized");
  }
  return p;
}
}
Config load_robot_config(const std::string & path)
{
  const auto n = YAML::LoadFile(path);
  Config c;
  c.model = n["robot"]["model"].as<std::string>();
  c.adapter = n["robot"]["adapter"].as<std::string>();
  c.backend = n["robot"]["execution_backend"].as<std::string>();
  const bool fr3 = c.model == "fr3" && c.adapter == "franka_fr3";
  const bool ur5e = c.model == "ur5e" && c.adapter == "ur5e";
  const bool generic = c.adapter == "generic_gazebo" && n["robot"]["joint_count"] && n["execution"];
  if (n["contract_version"].as<std::string>() != "0.3" ||
    c.backend != "gazebo" || (!fr3 && !ur5e && !generic)) {
    throw std::runtime_error("supported Gazebo v0.3 profile required; generic models require explicit joint_count/execution");
  }
  c.joint_count = generic ? n["robot"]["joint_count"].as<size_t>() : (fr3 ? 7 : 6);
  if (c.model.empty() || (c.joint_count != 6 && c.joint_count != 7)) {
    throw std::runtime_error("this KDL position backend supports validated 6/7-axis arms only");
  }
  if (n["robot"]["joint_count"] && n["robot"]["joint_count"].as<size_t>() != c.joint_count) {
    throw std::runtime_error("declared joint_count differs from robot profile");
  }
  const auto f = n["frames"], t = n["timing"], tr = n["transport"], topics = n["topics"];
  c.reference = f["reference"].as<std::string>();
  c.base = f["base"].as<std::string>();
  c.flange = f["flange"].as<std::string>();
  c.tcp = f["tcp"].as<std::string>();
  c.reference_to_base = yaml_pose(f["reference_to_base"]);
  c.flange_to_tcp = yaml_pose(f["flange_to_tcp"]);
  c.session = n["runtime"]["active_session_id"].as<uint64_t>();
  c.heartbeat_hz = n["runtime"]["state_heartbeat_hz"].as<double>();
  c.ttl_ns = t["command_ttl_ns"].as<int64_t>();
  c.period_ns = t["command_period_ns"].as<int64_t>();
  c.stale_ns = t["state_stale_ns"].as<int64_t>();
  c.lost_ns = t["state_lost_ns"].as<int64_t>();
  c.ik_budget_ns = t["ik_budget_ns"].as<int64_t>();
  c.receiver_watchdog_ns = t["receiver_watchdog_ns"].as<int64_t>();
  c.ik_iterations = n["ik"]["max_iterations"].as<unsigned int>();
  c.position_tolerance = n["ik"]["position_tolerance_m"].as<double>();
  c.orientation_tolerance = n["ik"]["orientation_tolerance_rad"].as<double>();
  c.description_topic = "/" + n["robot"]["robot_description_parameter"].as<std::string>();
  c.command_topic = topics["command"].as<std::string>();
  c.state_topic = topics["state"].as<std::string>();
  c.error_topic = topics["error"].as<std::string>();
  c.joint_topic = tr["joint_state_topic"].as<std::string>();
  c.transport_topic = tr["joint_command_topic"].as<std::string>();
  c.receiver_topic = tr["receiver_status_topic"].as<std::string>();
  c.log_directory = n["logging"]["directory"].as<std::string>();
  c.run_id = n["logging"]["run_id"].as<std::string>();
  c.logging = n["logging"]["enabled"].as<bool>();
  const auto state_period = t["state_period_ns"].as<int64_t>();
  if (c.session == 0 || c.reference != "map" || c.ttl_ns != 5000000 || c.period_ns != 1000000 ||
    state_period <= 0 || c.stale_ns != std::max<int64_t>(5000000, 3 * state_period) ||
    c.lost_ns != std::max<int64_t>(20000000, 10 * state_period) || c.ik_budget_ns <= 0 ||
    c.ik_budget_ns > 500000 || c.receiver_watchdog_ns < c.stale_ns ||
    c.receiver_watchdog_ns > 1000000000 || c.ik_iterations == 0 || c.ik_iterations > 100 ||
    !finite(c.heartbeat_hz) || c.heartbeat_hz <= 0 || c.heartbeat_hz > 1000 ||
    !finite(c.position_tolerance) || c.position_tolerance <= 0 ||
    !finite(c.orientation_tolerance) || c.orientation_tolerance <= 0)
  {
    throw std::runtime_error("invalid session/frame/timing/IK configuration");
  }
  for (const auto * name : {"command", "state", "error"}) {
    const auto qos = n["qos"][name];
    if (qos["durability"].as<std::string>() != "volatile" ||
      qos["reliability"].as<std::string>() != (std::string(name) == "error" ?
      "reliable" : "best_effort") || qos["depth"].as<int>() !=
      (std::string(name) == "error" ? 32 : 1))
    {
      throw std::runtime_error("QoS differs from v0.3");
    }
  }
  if (n["joints"].size() != c.joint_count) {throw std::runtime_error("robot joint count mismatch");}
  std::array<bool, 7> indices{};
  for (size_t i = 0; i < c.joint_count; ++i) {
    const auto j = n["joints"][i];
    auto & a = c.axes[i];
    a = {j["name"].as<std::string>(), j["device_index"].as<size_t>(),
      j["direction"].as<double>(), j["zero_offset_rad"].as<double>(),
      j["lower_rad"].as<double>(), j["upper_rad"].as<double>(),
      j["max_velocity_rad_s"].as<double>(), j["max_acceleration_rad_s2"].as<double>()};
    if (a.name.empty() || a.device_index >= c.joint_count ||
      indices[a.device_index] || std::abs(a.direction) != 1. || !finite(a.offset) ||
      !finite(a.lower) || !finite(a.upper) || a.lower >= a.upper || !finite(a.velocity) ||
      !finite(a.acceleration) || a.velocity <= 0 || a.acceleration <= 0)
    {
      throw std::runtime_error("invalid joint mapping/limits");
    }
    indices[a.device_index] = true;
  }
  for (size_t i = 0; i < 3; ++i) {
    c.workspace_min[i] = n["workspace"]["min_m"][i].as<double>();
    c.workspace_max[i] = n["workspace"]["max_m"][i].as<double>();
    if (!finite(c.workspace_min[i]) || !finite(c.workspace_max[i]) ||
      c.workspace_min[i] >= c.workspace_max[i])
    {
      throw std::runtime_error("invalid workspace");
    }
  }
  c.max_tcp_speed=n["workspace"]["max_tcp_speed_m_s"].as<double>();
  c.max_tcp_acceleration=n["workspace"]["max_tcp_acceleration_m_s2"].as<double>();
  if(!finite(c.max_tcp_speed) || c.max_tcp_speed<=0 ||
    !finite(c.max_tcp_acceleration) || c.max_tcp_acceleration<=0) {
    throw std::runtime_error("invalid TCP motion limits");
  }
  if (c.run_id.empty() || c.run_id == "." || c.run_id == ".." ||
    c.run_id.find_first_of("/\\") != std::string::npos)
  {
    throw std::runtime_error("run_id must be a single directory name");
  }
  for (size_t i = 0; i < c.joint_count; ++i) {
    for (size_t j = 0; j < i; ++j) {
      if (c.axes[i].name == c.axes[j].name) {throw std::runtime_error("duplicate joint name");}
    }
  }
  if (const auto execution = n["execution"]) {
    c.backend_period_ns = execution["backend_period_ns"].as<int64_t>();
    const auto cap = execution["capabilities"];
    if (cap["control_modes"].size() != 1 || cap["control_modes"][0].as<std::string>() != "position" ||
      cap["stop_strategy"].as<std::string>() != "decelerate_then_hold") {
      throw std::runtime_error("CAPABILITY_MISMATCH: backend implements position/decelerate_then_hold only");
    }
    c.position_capability = true;
    c.feedback_capability = cap["state_feedback"].as<bool>();
    c.watchdog_capability = cap["independent_watchdog"].as<bool>();
    const auto task = execution["task"];
    if (task["control_mode"].as<std::string>() != "position") {
      throw std::runtime_error("CAPABILITY_MISMATCH: task control mode unsupported");
    }
    c.task.min_dof = task["min_dof"].as<size_t>();
    c.task.preferred_period_ns = task["preferred_period_ns"].as<int64_t>();
    c.task.maximum_period_ns = task["maximum_period_ns"].as<int64_t>();
    // Process-corridor task profiles are not part of this build. Refuse them
    // before start instead of running without the protection they declare.
    if (task["tcp_corridor"]) {
      throw std::runtime_error("CAPABILITY_MISMATCH: tcp_corridor task profile unsupported");
    }
    for (const auto sample : task["sample_tcp_poses"]) {
      if (sample.size() != 7) {throw std::runtime_error("task sample must be xyz+xyzw");}
      geometry_msgs::msg::Pose pose;
      pose.position.x = sample[0].as<double>();pose.position.y = sample[1].as<double>();pose.position.z = sample[2].as<double>();
      pose.orientation.x = sample[3].as<double>();pose.orientation.y = sample[4].as<double>();
      pose.orientation.z = sample[5].as<double>();pose.orientation.w = sample[6].as<double>();
      c.task.samples.push_back(pose);
    }
  }
  if (!c.feedback_capability || !c.watchdog_capability || c.task.min_dof < 6 ||
    c.task.min_dof > c.joint_count || c.backend_period_ns < c.period_ns ||
    c.backend_period_ns >= c.ttl_ns || c.backend_period_ns % c.period_ns ||
    c.task.preferred_period_ns <= 0 || c.task.maximum_period_ns < c.task.preferred_period_ns ||
    c.task.maximum_period_ns >= c.ttl_ns ||
    c.backend_period_ns > c.task.maximum_period_ns || c.task.samples.size() > 16) {
    throw std::runtime_error("CAPABILITY_MISMATCH: feedback/watchdog/DOF/transport timing incompatible");
  }
  return c;
}

Config load_fr3_config(const std::string & path)
{
  auto c = load_robot_config(path);
  if (c.model != "fr3") {throw std::runtime_error("FR3 profile required");}
  return c;
}

}
