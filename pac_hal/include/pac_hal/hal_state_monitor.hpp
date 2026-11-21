// Copyright 2026 Jaewon Kim

#pragma once

#include <optional>
#include <set>
#include "pac_hal/hal_converter.hpp"

namespace pac_hal
{
// 측정 신선도와 통신 신선도는 다르다. 살아있는 heartbeat가 오래된 pose를 살려내면 안 된다.
class HalStateMonitor
{
public:
  explicit HalStateMonitor(const Config & config) : config_(config) {}
  bool observe(const Measurement & sample, const HalConverter & converter, int64_t now);
  bool observe_receiver(const Receiver & receiver, int64_t now);
  bool healthy(int64_t now) const;
  bool measurement_lost(int64_t now) const;
  bool receiver_lost(int64_t now) const;
  bool receiver_rejected() const;
  bool measurement_rejected() const {return !measurement_failure_;}
  const std::string & measurement_failure_detail() const {return measurement_failure_.detail;}
  bool receiver_feedback_rejected() const {return !receiver_feedback_failure_;}
  const std::string & receiver_feedback_failure_detail() const {return receiver_feedback_failure_.detail;}
  std::string health_detail(int64_t now) const;
  const Measurement & measurement() const {return *measurement_;}
  const geometry_msgs::msg::Pose & measured_pose() const {return pose_;}
  State state(int64_t now, uint64_t command_seq, uint8_t command_result,
    const std::set<uint16_t> & errors) const;

private:
  Config config_;
  std::optional<Measurement> measurement_;
  std::optional<Receiver> receiver_;
  geometry_msgs::msg::Pose pose_;
  geometry_msgs::msg::Twist twist_;
  int64_t received_ns_{0}, receiver_received_ns_{0};
  uint64_t state_seq_{0};
  Result measurement_failure_;
  Result receiver_feedback_failure_;
};
}  // namespace pac_hal
