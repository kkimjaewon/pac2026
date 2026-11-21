#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include "std_msgs/msg/string.hpp"
#include "pac_hal/receiver_contract.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "intelion_interfaces/msg/hal_joint_command.hpp"
#include "intelion_interfaces/msg/hal_joint_status.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace pac_hal
{
using CallbackReturn = controller_interface::CallbackReturn;
using Command = intelion_interfaces::msg::HalJointCommand;
using Status = intelion_interfaces::msg::HalJointStatus;

static int64_t mono_ns()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

struct Slot
{
  uint64_t session{0}, seq{0};
  int64_t created{0}, expires{0};
  bool valid{false};
  std::array<double, 7> q{};
};

class WatchdogPositionController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {});
    for (const auto * name : {"lower", "upper", "max_velocity", "max_acceleration", "direction", "zero_offset"}) {
      auto_declare<std::vector<double>>(name, {});
    }
    auto_declare<int64_t>("session_id", 0);
    auto_declare<int64_t>("command_ttl_ns", 5000000);
    auto_declare<int64_t>("watchdog_ns", 100000000);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration c;
    c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & j : joints_) {c.names.push_back(j + "/position");}
    return c;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration c;
    c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & j : joints_) {c.names.push_back(j + "/position");}
    return c;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto n = get_node();
    joints_ = n->get_parameter("joints").as_string_array();
    lower_ = n->get_parameter("lower").as_double_array();
    upper_ = n->get_parameter("upper").as_double_array();
    vmax_ = n->get_parameter("max_velocity").as_double_array();
    amax_ = n->get_parameter("max_acceleration").as_double_array();
    direction_ = n->get_parameter("direction").as_double_array();
    offset_ = n->get_parameter("zero_offset").as_double_array();
    const auto sid = n->get_parameter("session_id").as_int();
    ttl_ = n->get_parameter("command_ttl_ns").as_int();
    watchdog_ns_ = n->get_parameter("watchdog_ns").as_int();
    if (joints_.empty() || joints_.size() > 7 || sid <= 0 || ttl_ != 5000000 ||
      watchdog_ns_ <= 0 || lower_.size() != joints_.size() ||
      upper_.size() != joints_.size() || vmax_.size() != joints_.size() ||
      amax_.size() != joints_.size() || direction_.size() != joints_.size() ||
      offset_.size() != joints_.size())
    {
      return CallbackReturn::ERROR;
    }
    session_ = static_cast<uint64_t>(sid);
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) ||
        !std::isfinite(vmax_[i]) || !std::isfinite(amax_[i]) ||
        lower_[i] >= upper_[i] || vmax_[i] <= 0 || amax_[i] <= 0 ||
        !std::isfinite(offset_[i]) || std::abs(direction_[i]) != 1.0)
      {
        return CallbackReturn::ERROR;
      }
    }
    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    // Passive diagnostics use the non-RT command callback. Never serialize JSON
    // or publish a second HalError source from the real-time update loop.
    diagnostics_ = n->create_publisher<std_msgs::msg::String>("~/diagnostics",
      rclcpp::QoS(32).reliable().durability_volatile());
    sub_ = n->create_subscription<Command>("~/commands", qos,
      [this](Command::ConstSharedPtr msg) {
        const auto now = mono_ns();
        const auto checked=check_receiver_packet(*msg,session_,joints_.size(),now,ttl_,lower_,upper_);
        if(checked.reason!=ReceiverReject::None) {
          // A late packet is a transport expiry, not a device fault: C reports it
          // per packet as EXPIRED from this diagnostic. Others stay hard rejections.
          if(checked.reason!=ReceiverReject::Expired) {rejected_.store(true);}
          std::ostringstream out;
          out<<"{\"active_session_id\":"<<session_<<",\"packet_session_id\":"<<msg->session_id
             <<",\"command_seq\":"<<msg->command_seq<<",\"received_mono_ns\":"<<now
             <<",\"created_mono_ns\":"<<msg->created_mono_ns<<",\"expires_mono_ns\":"<<msg->expires_mono_ns
             <<",\"reason\":\""<<receiver_reason(checked.reason)<<"\",\"axis\":"<<checked.axis<<"}";
          std_msgs::msg::String diagnostic;diagnostic.data=out.str();diagnostics_->publish(diagnostic);
          return;
        }
        Slot next;
        next.session = msg->session_id;
        next.seq = msg->command_seq;
        next.created = msg->created_mono_ns;
        next.expires = msg->expires_mono_ns;
        next.valid = true;
        next.q = msg->positions;
        pending_.writeFromNonRT(next);
      });
    auto publisher = n->create_publisher<Status>("~/status", qos);
    publisher_ = std::make_unique<realtime_tools::RealtimePublisher<Status>>(publisher);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pending_.initRT(Slot{});
    has_command_ = false;
    applied_seq_ = 0;
    applied_ns_ = 0;
    last_feedback_ = 0;
    valid_until_ = 0;
    rejected_.store(false);
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto current = state_interfaces_[i].get_optional<double>();
      if (!current || !std::isfinite(*current) ||
        direction_[i] * *current + offset_[i] < lower_[i] ||
        direction_[i] * *current + offset_[i] > upper_[i]) {
        return CallbackReturn::ERROR;
      }
      q_[i] = goal_[i] = direction_[i] * *current + offset_[i];
      v_[i] = 0.0;
      if (!command_interfaces_[i].set_value((q_[i] - offset_[i]) / direction_[i])) {return CallbackReturn::ERROR;}
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration & period)
  override
  {
    const auto now = mono_ns();
    const auto * command = pending_.readFromRT();
    // Sequence and expiry checks also happen at actual receiver adoption.
    if (command->valid && command->session == session_ &&
      (!has_command_ || command->seq > applied_seq_) && command->created <= now &&
      now < command->expires)
    {
      goal_ = command->q;
      applied_seq_ = command->seq;
      applied_ns_ = now;
      valid_until_ = command->expires;
      has_command_ = true;
      rejected_.store(false);
    }
    const bool stopping = !has_command_ || now >= valid_until_ ||
      now - applied_ns_ >= watchdog_ns_;
    const double dt = period.seconds();
    if (!std::isfinite(dt)) {return controller_interface::return_type::ERROR;}
    // Gazebo may call the first update with a zero simulation interval. Hold that cycle.
    if (dt <= 0.0) {return controller_interface::return_type::OK;}
    for (size_t i = 0; i < joints_.size(); ++i) {
      const double error = goal_[i] - q_[i];
      const double dv = amax_[i] * dt;
      // Discrete braking-speed bound includes one control interval of margin.
      const double braking = std::max(0.0, std::sqrt(dv * dv +
        2 * amax_[i] * std::abs(error)) - dv);
      const double wanted = stopping ? 0.0 : std::copysign(std::min(vmax_[i], braking), error);
      v_[i] = std::clamp(wanted, v_[i] - dv, v_[i] + dv);
      const double next = q_[i] + v_[i] * dt;
      // A hard range boundary is a final guard, separate from normal smooth stopping.
      q_[i] = std::clamp(next, lower_[i], upper_[i]);
      if (q_[i] != next) {v_[i] = 0.0; rejected_.store(true);}
      if (!command_interfaces_[i].set_value((q_[i] - offset_[i]) / direction_[i])) {
        return controller_interface::return_type::ERROR;
      }
    }
    if (now - last_feedback_ >= 10000000 && publisher_->trylock()) {
      auto & msg = publisher_->msg_;
      msg.session_id = session_;
      msg.last_applied_command_seq = applied_seq_;
      msg.applied_mono_ns = applied_ns_;
      msg.sampled_mono_ns = now;
      msg.active = true;
      msg.watchdog_active = stopping;
      msg.rejected = rejected_.load();
      msg.joint_count = static_cast<uint8_t>(joints_.size());
      msg.commanded_positions = q_;
      msg.commanded_velocities = v_;
      publisher_->unlockAndPublish();
      last_feedback_ = now;
    }
    return controller_interface::return_type::OK;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  std::vector<std::string> joints_;
  std::vector<double> lower_, upper_, vmax_, amax_, direction_, offset_;
  std::array<double, 7> q_{}, v_{}, goal_{};
  uint64_t session_{0}, applied_seq_{0};
  int64_t ttl_{5000000}, watchdog_ns_{100000000}, valid_until_{0};
  int64_t applied_ns_{0}, last_feedback_{0};
  bool has_command_{false};
  std::atomic<bool> rejected_{false};
  realtime_tools::RealtimeBuffer<Slot> pending_;
  rclcpp::Subscription<Command>::SharedPtr sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Status>> publisher_;
};
}  // namespace pac_hal

PLUGINLIB_EXPORT_CLASS(pac_hal::WatchdogPositionController, controller_interface::ControllerInterface)
