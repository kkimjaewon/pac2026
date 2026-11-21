// Copyright 2026 Jaewon Kim

#include <chrono>
#include <exception>
#include <memory>
#include <sched.h>
#include <cstring>
#include <cerrno>
#include <fstream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "pac_hal/hal_command_gateway.hpp"

namespace pac_hal
{
// 노드는 조립과 수명 관리만 맡는다. IK/통신/모니터링 코드를 main에 섞지 않는다.
class HalNode final : public rclcpp::Node
{
public:
  HalNode() : Node("hal_command_gateway")
  {
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    const auto path = declare_parameter<std::string>("config_file", "", immutable);
    // launch는 session/run/log를 반영한 검증 snapshot YAML을 만든다. 실행 중 값 변경은 금지한다.
    declare_parameter<int64_t>("session_id", 0, immutable);
    declare_parameter<std::string>("run_id", "", immutable);
    declare_parameter<std::string>("log_directory", "", immutable);
    const auto cpu = declare_parameter<int>("executor_cpu", -1, immutable);
    const auto priority = declare_parameter<int>("executor_fifo_priority", 0, immutable);
    if (cpu < -1 || cpu >= CPU_SETSIZE || priority < 0 || priority > 80) {
      throw std::runtime_error("invalid executor scheduling settings");
    }
    auto config = load_robot_config(path);
    const auto budget_clock = declare_parameter<std::string>("ik_budget_clock", "iterations", immutable);
    if (budget_clock != "wall" && budget_clock != "thread_cpu" && budget_clock != "iterations") {
      throw std::runtime_error("ik_budget_clock must be wall, thread_cpu or iterations");
    }
    config.ik_budget = budget_clock == "wall" ? IkBudget::Wall :
      budget_clock == "thread_cpu" ? IkBudget::ThreadCpu : IkBudget::Iterations;
    RCLCPP_INFO(get_logger(), "IK budget clock=%s; command wall TTL remains 5ms", budget_clock.c_str());
    if (get_parameter("session_id").as_int() != static_cast<int64_t>(config.session)) {
      throw std::runtime_error("session parameter and snapshot YAML disagree");
    }
    const auto scene = declare_parameter<std::string>("collision_scene_file", "", immutable);
    if (scene.empty()) {
      throw std::runtime_error("collision scene is required before enabling HAL");
    }
    const auto model_path=declare_parameter<std::string>("expected_model_file","",immutable);
    std::ifstream model_file(model_path);
    if(model_path.empty() || !model_file) {
      throw std::runtime_error("prepared model snapshot missing; launch through hal_system/controller preparation");
    }
    std::ostringstream model_text;model_text<<model_file.rdbuf();
    if(model_text.str().empty() || model_file.bad()) {
      throw std::runtime_error("prepared model snapshot empty or unreadable");
    }
    gateway_ = std::make_unique<HalCommandGateway>(*this, config, scene, model_text.str());
    subscription_ = create_subscription<Command>(config.command_topic,
      rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](Command::ConstSharedPtr command) {gateway_->command(*command);});
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / config.heartbeat_hz),
      [this]() {gateway_->heartbeat();});
    dispatch_timer_ = create_wall_timer(std::chrono::nanoseconds(config.period_ns),
      [this]() {gateway_->dispatch();});
    // DDS/기록 스레드를 만든 뒤 현재 executor에만 적용한다.
    // 선택한 C executor 스레드만 고정한다. DDS/Gazebo/B 설정은 별도로 유지한다.
    if (cpu >= 0) {
      cpu_set_t allowed, selected;
      if (sched_getaffinity(0, sizeof(allowed), &allowed) || !CPU_ISSET(cpu, &allowed)) {
        throw std::runtime_error("executor CPU is unavailable");
      }
      CPU_ZERO(&selected); CPU_SET(cpu, &selected);
      if (sched_setaffinity(0, sizeof(selected), &selected)) {
        throw std::runtime_error(std::string("executor affinity failed: ") + std::strerror(errno));
      }
    }
    if (priority > 0) {
      sched_param param{}; param.sched_priority = priority;
      if (sched_setscheduler(0, SCHED_FIFO, &param)) {
        throw std::runtime_error(std::string("FIFO permission/setup failed: ") + std::strerror(errno));
      }
    }
    RCLCPP_INFO(get_logger(), "C executor cpu=%d FIFO priority=%d", static_cast<int>(cpu), static_cast<int>(priority));
    RCLCPP_INFO(get_logger(), "C++ Gazebo HAL active, robot=%s axes=%zu session=%s; waiting for fresh state",
      config.model.c_str(), config.joint_count, std::to_string(config.session).c_str());
  }

private:
  std::unique_ptr<HalCommandGateway> gateway_;
  rclcpp::Subscription<Command>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_, dispatch_timer_;
};
}  // namespace pac_hal

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<pac_hal::HalNode>();
    // 단일 executor로 명령·측정·ACK의 shared state를 직렬 처리한다.
    // 이 노드는 B의 RT thread가 아니다.
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("hal_node"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
