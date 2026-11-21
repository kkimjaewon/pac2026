// Copyright 2026 Jaewon Kim

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <set>
#include "pac_hal/hal_latency_trace.hpp"
#include "pac_hal/hal_state_monitor.hpp"
#include "pac_hal/robot_adapter.hpp"
#include "pac_hal/scene_guard.hpp"
#include "pac_hal/execution_contract.hpp"
#include "pac_hal/tcp_trajectory.hpp"
#include "std_msgs/msg/string.hpp"

namespace pac_hal
{
// 실행 순서를 한곳에서 관리한다. 세션/TTL 검사 없이 converter/adapter를 직접 호출하지 않는다.
class HalCommandGateway
{
public:
  HalCommandGateway(rclcpp::Node & node, const Config & config,
    const std::string & scene_file, const std::string & expected_xml);
  void command(const Command & command);
  void heartbeat();
  void dispatch();

private:
  Result validate(const Command & command, int64_t now, bool sequence = true);
  // active_attempt is used only for queued rejects; it is computed before validate()
  // consumes the sequence watermark.
  void execute(const Command & command, TraceRecord record, bool queued = false, Result previous = {},
    bool active_attempt = true);
  void update_preflight();
  void publish_preflight();
  Result joint_limits(const Joints & candidate, const MotionBaseline & baseline);
  void transition(uint16_t code, bool active, const Command & command, const std::string & detail);
  void publish_state();
  rclcpp::Node & node_;
  Config config_;
  HalConverter converter_;
  std::unique_ptr<HalConverter> startup_converter_;
  LatestCommandRate rate_;
  bool preflight_done_{false};
  Result preflight_result_;
  ModelDescriptionGuard description_guard_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preflight_pub_;
  SceneGuard scene_;
  HalStateMonitor monitor_;
  HalLatencyTrace trace_;
  std::unique_ptr<RobotAdapter> adapter_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Publisher<Error>::SharedPtr error_pub_;
  std::set<uint16_t> errors_;
  std::optional<uint64_t> observed_seq_;
  std::optional<Joints> last_q_;
  std::optional<geometry_msgs::msg::Pose> last_tcp_target_;
  std::array<double,3> last_tcp_velocity_{};
  Joints last_velocity_{};
  int64_t last_created_ns_{0};
  double last_dt_{0.};
  uint64_t last_command_seq_{0};
  uint8_t last_command_result_{State::CMD_REJECTED};
  Command last_command_;
  bool command_seen_{false};
  bool measured_collision_{false};
  std::string measured_protection_detail_;
  int64_t last_measurement_callback_{0}, last_measurement_stamp_{0};
  int64_t last_receiver_callback_{0}, last_receiver_stamp_{0};
};
}  // namespace pac_hal
