// Copyright 2026 Jaewon Kim

#include <algorithm>
#include <cmath>
#include <utility>
#include "pac_hal/robot_adapter.hpp"
#include "intelion_interfaces/msg/hal_joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace pac_hal
{
class GazeboPositionAdapter final : public RobotAdapter
{
public:
  GazeboPositionAdapter(rclcpp::Node & node, const Config & config, AdapterCallbacks callbacks)
  : config_(config), callbacks_(std::move(callbacks))
  {
    auto latest = rclcpp::QoS(1).best_effort().durability_volatile();
    publisher_ = node.create_publisher<intelion_interfaces::msg::HalJointCommand>(
      config_.transport_topic, latest);
    // URDF는 시작 때 한 번 발행될 수 있어 transient_local로 기존 설명도 받는다.
    description_ = node.create_subscription<std_msgs::msg::String>(config_.description_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {callbacks_.description(msg->data);});
    joints_ = node.create_subscription<sensor_msgs::msg::JointState>(config_.joint_topic, latest,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        Measurement sample;
        sample.header = msg->header;
        sample.velocity_valid = true;
        if(msg->position.size()!=msg->name.size() ||
          (!msg->velocity.empty() && msg->velocity.size()!=msg->name.size())) {
          sample.layout_valid=false;callbacks_.measurement(sample);return;
        }
        for (size_t i = 0; i < config_.joint_count; ++i) {
          const auto found = std::find(msg->name.begin(), msg->name.end(), config_.axes[i].name);
          if (found == msg->name.end() ||
            std::find(std::next(found),msg->name.end(),config_.axes[i].name)!=msg->name.end()) {
            sample.layout_valid=false;callbacks_.measurement(sample);return;
          }
          const auto index = static_cast<size_t>(found - msg->name.begin());
          sample.position[i] = msg->position[index];
          if (index >= msg->velocity.size()) {
            sample.velocity_valid = false;
          } else {
            sample.velocity[i] = msg->velocity[index];
          }
        }
        // joint_states는 이미 URDF 기준이다. direction/offset을 또 적용하면 이중 변환된다.
        callbacks_.measurement(sample);
      });
    receiver_ = node.create_subscription<Receiver>(config_.receiver_topic, latest,
      [this](Receiver::ConstSharedPtr msg) {callbacks_.receiver(*msg);});
    diagnostic_ = node.create_subscription<std_msgs::msg::String>(
      config_.transport_topic.substr(0,config_.transport_topic.find_last_of('/'))+"/diagnostics",
      rclcpp::QoS(32).reliable(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if(callbacks_.receiver_diagnostic) {callbacks_.receiver_diagnostic(msg->data,monotonic_ns());}
      });
  }

  bool ready() const override {return publisher_->get_subscription_count() > 0;}

  Result send(const Command & command, const Joints & canonical) override
  {
    intelion_interfaces::msg::HalJointCommand output;
    output.session_id = command.session_id;
    output.command_seq = command.command_seq;
    output.created_mono_ns = command.created_mono_ns;
    output.expires_mono_ns = command.expires_mono_ns;
    output.joint_count = config_.joint_count;
    for (size_t i = 0; i < config_.joint_count; ++i) {
      const auto & axis = config_.axes[i];
      output.positions[axis.device_index] = axis.direction * canonical[i] + axis.offset;
    }
    // IK가 오래 걸렸거나 상대가 사라졌다면 전송하지 않는다. 수신 controller도 만료를 재검사한다.
    if (monotonic_ns() >= command.expires_mono_ns) {
      return {Error::E_EXPIRED, "expired before Gazebo transport"};
    }
    if (!ready()) {
      return {Error::E_SEND_FAILED, "no watchdog receiver subscription"};
    }
    publisher_->publish(output);
    return {};
  }

private:
  Config config_;
  AdapterCallbacks callbacks_;
  rclcpp::Publisher<intelion_interfaces::msg::HalJointCommand>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<Receiver>::SharedPtr receiver_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostic_;
};

std::unique_ptr<RobotAdapter> make_gazebo_adapter(
  rclcpp::Node & node, const Config & config, AdapterCallbacks callbacks)
{
  return std::make_unique<GazeboPositionAdapter>(node, config, std::move(callbacks));
}
}  // namespace pac_hal
