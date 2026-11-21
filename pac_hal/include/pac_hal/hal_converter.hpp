// Copyright 2026 Jaewon Kim

#pragma once

#include <memory>
#include <string>
#include <utility>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "pac_hal/hal_types.hpp"

namespace pac_hal
{
// ROS 토픽을 모르는 순수 변환 모듈: 어댑터를 바꿔도 같은 map TCP 계약을 유지한다.
class HalConverter
{
public:
  explicit HalConverter(const Config & config);
  Result initialize(const std::string & urdf);
  bool ready() const {return ready_;}
  Result inverse(const geometry_msgs::msg::Pose & target, const Joints & seed, Joints & output, int64_t expires_ns = 0);
  Result forward(const Measurement & sample, geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::Twist & twist) const;
  int64_t last_ik_wall_ns() const {return last_ik_wall_ns_;}
  int64_t last_ik_cpu_ns() const {return last_ik_cpu_ns_;}
  unsigned int last_ik_steps() const {return last_ik_steps_;}
  const Joints & lower() const {return lower_;}
  const Joints & upper() const {return upper_;}

private:
  Config config_;
  int newton_step(KDL::JntArray & q, const KDL::Frame & desired);
  int64_t last_ik_wall_ns_{0}, last_ik_cpu_ns_{0};
  unsigned int last_ik_steps_{0};
  bool ready_{false};
  KDL::Chain chain_;
  KDL::Frame installation_, tool_;
  Joints lower_{}, upper_{};
  KDL::JntArray seed_buffer_, solution_buffer_, step_buffer_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_;
  std::unique_ptr<KDL::ChainIkSolverVel> velocity_;
};
}  // namespace pac_hal
