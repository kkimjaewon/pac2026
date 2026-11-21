// Copyright 2026 Jaewon Kim

#pragma once

#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "pac_hal/hal_types.hpp"

namespace pac_hal
{
struct AdapterCallbacks
{
  std::function<void(const std::string &)> description;
  std::function<void(const Measurement &)> measurement;
  std::function<void(const Receiver &)> receiver;
  std::function<void(const std::string &, int64_t)> receiver_diagnostic;
};

// 제조사별 통신을 여기 뒤로 숨긴다. Gateway는 ROS controller 이름/Franka 패킷을 몰라도 된다.
// send 성공은 '전송 시도 완료'다. 수신단 적용 ACK와 TCP 도달은 각각 별도 확인한다.
class RobotAdapter
{
public:
  virtual ~RobotAdapter() = default;
  virtual bool ready() const = 0;
  virtual Result send(const Command & command, const Joints & canonical) = 0;
};
std::unique_ptr<RobotAdapter> make_gazebo_adapter(
  rclcpp::Node & node, const Config & config, AdapterCallbacks callbacks);
}  // namespace pac_hal
