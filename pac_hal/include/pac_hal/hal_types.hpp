// Copyright 2026 Jaewon Kim

#pragma once

#include <time.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "intelion_interfaces/msg/hal_command.hpp"
#include "intelion_interfaces/msg/hal_error.hpp"
#include "intelion_interfaces/msg/hal_joint_status.hpp"
#include "intelion_interfaces/msg/hal_state.hpp"
#include "std_msgs/msg/header.hpp"

namespace pac_hal
{
using Command = intelion_interfaces::msg::HalCommand;
using State = intelion_interfaces::msg::HalState;
using Error = intelion_interfaces::msg::HalError;
using Receiver = intelion_interfaces::msg::HalJointStatus;
// B never sees joint storage. Fixed capacity avoids allocation; joint_count selects active axes.
constexpr size_t MAX_DOF = 7;
// wall/thread_cpu: ik_budget_ns on that clock. iterations: only the profile's
// max_iterations bounds the work; scheduling stalls then surface as EXPIRED.
enum class IkBudget {Wall, ThreadCpu, Iterations};
inline const char * ik_budget_name(IkBudget b) {
  return b == IkBudget::Wall ? "wall" : b == IkBudget::ThreadCpu ? "thread_cpu" : "iterations";
}
using Joints = std::array<double, MAX_DOF>;

// B와 같은 PC의 CLOCK_MONOTONIC을 사용한다. ROS/Gazebo 시간은 TTL 계산에 사용하지 않는다.
inline int64_t monotonic_ns()
{
  timespec t{};
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

struct Axis
{
  std::string name;
  size_t device_index;
  double direction, offset, lower, upper, velocity, acceleration;
};

struct TaskRequirement
{
  size_t min_dof{6};
  int64_t preferred_period_ns{1000000}, maximum_period_ns{1000000};
  std::vector<geometry_msgs::msg::Pose> samples;
};

struct Config
{
  size_t joint_count{7};
  std::string model, adapter, backend;
  std::array<Axis, MAX_DOF> axes{};
  std::string reference, base, flange, tcp, description_topic, command_topic, state_topic;
  std::string error_topic, joint_topic, receiver_topic, transport_topic, log_directory, run_id;
  geometry_msgs::msg::Pose reference_to_base, flange_to_tcp;
  std::array<double, 3> workspace_min, workspace_max;
  double max_tcp_speed{0.1}, max_tcp_acceleration{0.2};
  uint64_t session;
  int64_t ttl_ns, period_ns, stale_ns, lost_ns, ik_budget_ns;
  int64_t receiver_watchdog_ns{100000000};
  unsigned int ik_iterations;
  double position_tolerance, orientation_tolerance, heartbeat_hz;
  bool logging;
  int64_t backend_period_ns{1000000};
  bool position_capability{true}, feedback_capability{true}, watchdog_capability{true};
  TaskRequirement task;
  // What bounds IK work. The 5ms wall command TTL is always enforced on top.
  IkBudget ik_budget{IkBudget::Iterations};
};
Config load_robot_config(const std::string & path);
// Compatibility entry point for FR3 callers; rejects other robot profiles.
Config load_fr3_config(const std::string & path);

struct Result
{
  uint16_t code{0};
  std::string detail;
  explicit operator bool() const {return code == 0;}
};

// 실제 로봇 측정만 담는다. commanded_position/ACK를 measured_position으로 바꾸면 안 된다.
struct Measurement
{
  Joints position{}, velocity{};
  bool velocity_valid{false};
  bool layout_valid{true};
  std_msgs::msg::Header header;
};

// 미수행 단계는 0으로 두고 CSV에서는 빈 칸으로 쓴다. 거짓 ACK/지연 숫자를 만들지 않는다.
struct TraceRecord
{
  uint64_t session{0}, seq{0};
  int64_t t1{0}, t2{0}, t3{0}, t4{0}, t6{0};
  uint64_t source_result_id{0};
  int64_t created{0}, validated{0}, ik_end{0}, transport_start{0}, transport_end{0};
  int64_t receiver_applied{0};
  int64_t ik_wall_ns{0}, ik_cpu_ns{0};
  IkBudget ik_budget{IkBudget::Wall};
  uint16_t error{0};
  bool sent{false}, superseded{false};
};
}  // namespace pac_hal
