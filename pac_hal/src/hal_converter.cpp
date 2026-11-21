// Copyright 2026 Jaewon Kim

#include "pac_hal/hal_converter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <Eigen/SVD>
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

namespace pac_hal
{
namespace
{
// Use fixed-size 6x6/6x7 SVD selected once when the model initializes.
// 동적 배열/SVD 반복에 드는 비용을 줄이고 기존 최소 노름 해와 동일한 절단 기준을 쓴다.
template<int DOF>
class FixedVelocity final : public KDL::ChainIkSolverVel
{
public:
  explicit FixedVelocity(const KDL::Chain & chain) : jacobian_(DOF), solver_(chain) {}
  int CartToJnt(const KDL::JntArray & q, const KDL::Twist & target,
    KDL::JntArray & output) override
  {
    const auto rc = solver_.JntToJac(q, jacobian_);
    if (rc < 0) {return rc;}
    matrix_ = jacobian_.data;
    svd_.compute(matrix_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 6, 1> twist, inverse;
    for (int i = 0; i < 6; ++i) {
      twist[i] = target[i];
      const auto singular = svd_.singularValues()[i];
      inverse[i] = singular > 1e-7 ? 1. / singular : 0.;
    }
    const Eigen::Matrix<double, DOF, 1> result = svd_.matrixV().template leftCols<6>() *
      inverse.asDiagonal() * svd_.matrixU().transpose() * twist;
    output.data = result;
    return 0;
  }
  int CartToJnt(const KDL::JntArray &, const KDL::FrameVel &, KDL::JntArrayVel &) override
  {return -1;}
  void updateInternalDataStructures() override {solver_.updateInternalDataStructures();}
private:
  KDL::Jacobian jacobian_;
  KDL::ChainJntToJacSolver solver_;
  Eigen::Matrix<double, 6, DOF> matrix_;
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, DOF>> svd_;
};
bool finite(double x) {return std::isfinite(x);}
KDL::Frame frame(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return {KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w), {p.x, p.y, p.z}};
}
KDL::JntArray array(const Joints & values, size_t count)
{
  KDL::JntArray a(count);
  for (size_t i = 0; i < count; ++i) {a(i) = values[i];}
  return a;
}
}  // namespace

HalConverter::HalConverter(const Config & c)
: config_(c), installation_(frame(c.reference_to_base)), tool_(frame(c.flange_to_tcp)),
  seed_buffer_(c.joint_count), solution_buffer_(c.joint_count), step_buffer_(c.joint_count) {}

Result HalConverter::initialize(const std::string & xml)
{
  if (ready_) {return {};}
  // Gazebo 모델만 허용한다. 실물 Franka 제어는 별도 libfranka 어댑터가 준비되어야 한다.
  if (xml.find("gz_ros2_control") == std::string::npos) {
    return {Error::E_UNSUPPORTED, "Gazebo ros2_control URDF required"};
  }
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initString(xml) || !kdl_parser::treeFromUrdfModel(model, tree) ||
    !tree.getChain(config_.base, config_.flange, chain_) || chain_.getNrOfJoints() != config_.joint_count)
  {
    return {Error::E_TF, "configured base-to-flange URDF chain missing"};
  }
  size_t i = 0;
  for (const auto & segment : chain_.segments) {
    const auto & joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::Fixed) {continue;}
    if (joint.getName() != config_.axes[i].name) {
      return {Error::E_TF, "configured joint order differs from URDF chain"};
    }
    const auto original = model.getJoint(joint.getName());
    if (!original || !original->limits) {return {Error::E_TF, "URDF limits missing"};}
    // YAML로 하드웨어 허용 범위를 넓힐 수 없게 URDF와 교집합을 사용한다.
    lower_[i] = std::max(config_.axes[i].lower, original->limits->lower);
    upper_[i] = std::min(config_.axes[i].upper, original->limits->upper);
    if (lower_[i] >= upper_[i] || config_.axes[i].velocity > original->limits->velocity) {
      return {Error::E_OUT_OF_RANGE, "configured limits exceed official URDF"};
    }
    ++i;
  }
  fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  if (config_.joint_count == 7) {velocity_ = std::make_unique<FixedVelocity<7>>(chain_);}
  else {velocity_ = std::make_unique<FixedVelocity<6>>(chain_);}
  // IK uses newton_step(): joint-limited NR with a 1e-9 task tolerance and an
  // exact small-angle rotation error. Joint limits and the IK budget are unchanged.
  ready_ = true;
  return {};
}

int HalConverter::newton_step(KDL::JntArray & q, const KDL::Frame & desired)
{
  // Same joint-limited Newton-Raphson step as KDL NR_JL, but with an exact small
  // rotation error. KDL Vector::Norm() (and so diff()/GetRot()) returns 0 when
  // every component is below KDL::epsilon=1e-6, so the solver "converged" with an
  // orientation error that drifted to ~1e-6rad and was then fixed in one step:
  // a joint jump that failed the 1kHz acceleration check with no real motion.
  KDL::Frame current;
  if (fk_->JntToCart(q, current) < 0) {return KDL::ChainIkSolverPos_NR_JL::E_FKSOLVERPOS_FAILED;}
  const KDL::Rotation e = desired.M * current.M.Inverse();
  const KDL::Vector s(.5 * (e(2, 1) - e(1, 2)), .5 * (e(0, 2) - e(2, 0)), .5 * (e(1, 0) - e(0, 1)));
  const double c = .5 * (e(0, 0) + e(1, 1) + e(2, 2) - 1.);
  const double sine = std::sqrt(KDL::dot(s, s));  // not Norm(): see above
  KDL::Vector rotation = sine > 0. ? s * (std::atan2(sine, c) / sine) : KDL::Vector::Zero();
  if (c < 0.) {rotation = KDL::diff(current.M, desired.M);}  // far from target: KDL is exact enough
  const KDL::Twist error(desired.p - current.p, rotation);
  if (KDL::Equal(error, KDL::Twist::Zero(), 1e-9)) {return KDL::SolverI::E_NOERROR;}
  if (velocity_->CartToJnt(q, error, step_buffer_) < 0) {
    return KDL::ChainIkSolverPos_NR_JL::E_IKSOLVERVEL_FAILED;
  }
  for (size_t i = 0; i < config_.joint_count; ++i) {
    q(i) = std::clamp(q(i) + step_buffer_(i), lower_[i], upper_[i]);
  }
  return KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED;
}

Result HalConverter::inverse(const geometry_msgs::msg::Pose & pose, const Joints & seed,
  Joints & output, int64_t expires_ns)
{
  last_ik_wall_ns_ = last_ik_cpu_ns_ = 0;
  last_ik_steps_ = 0;
  if (!ready_) {return {Error::E_TF, "robot model not initialized"};}
  // map_T_tcp = map_T_base * base_T_flange * flange_T_tcp 이므로 양끝 변환을 역으로 제거한다.
  const auto desired = installation_.Inverse() * frame(pose) * tool_.Inverse();
  // 단일 gateway executor에서 호출하므로 배열을 재사용해 1kHz 할당을 줄인다.
  // 같은 converter에 대한 동시 inverse 호출은 지원하지 않는다.
  auto & result = solution_buffer_;
  for (size_t i = 0; i < config_.joint_count; ++i) {seed_buffer_(i) = seed[i];}
  timespec cpu_start{}, cpu_end{};
  // The wall window encloses the thread-CPU window. Without IRQ time accounting,
  // interrupt work is charged to this thread, so CPU time is not pure IK compute.
  const auto start = monotonic_ns();
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_start);
  // Advance the same joint-limited KDL solver one step at a time, checking
  // deadlines between steps instead of spending all iterations on a late job.
  // This cannot interrupt a running SVD or an OS scheduling pause.
  result = seed_buffer_;
  int rc = KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED;
  auto converge = [&]() {
    rc = KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED;
    for (unsigned int step = 0; step < config_.ik_iterations; ++step) {
      const auto now = monotonic_ns();
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end);
      const auto cpu_elapsed = (cpu_end.tv_sec - cpu_start.tv_sec) * 1000000000LL +
        cpu_end.tv_nsec - cpu_start.tv_nsec;
      const bool over_budget = config_.ik_budget != IkBudget::Iterations &&
        (config_.ik_budget == IkBudget::ThreadCpu ? cpu_elapsed : now - start) > config_.ik_budget_ns;
      if ((expires_ns && now >= expires_ns) || over_budget) {break;}
      rc = newton_step(result, desired);
      ++last_ik_steps_;
      if (rc != KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED) {break;}
    }
  };
  converge();
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end);
  const auto elapsed = monotonic_ns() - start;
  last_ik_wall_ns_ = elapsed;
  last_ik_cpu_ns_ = (cpu_end.tv_sec - cpu_start.tv_sec) * 1000000000LL +
    cpu_end.tv_nsec - cpu_start.tv_nsec;
  // CPU 예산 비교 시험에서도 발행 명령의 wall TTL은 그대로 검사한다.
  if (expires_ns && monotonic_ns() >= expires_ns) {
    return {Error::E_EXPIRED, "expired during IK; never transported"};
  }
  const auto budget_time = config_.ik_budget == IkBudget::ThreadCpu ? last_ik_cpu_ns_ : elapsed;
  if (config_.ik_budget != IkBudget::Iterations && budget_time > config_.ik_budget_ns) {
    std::ostringstream detail;
    detail << "IK deadline exceeded: wall_us=" << elapsed / 1000.
           << " thread_cpu_us=" << last_ik_cpu_ns_ / 1000. << " solver_rc=" << rc
           << " steps=" << last_ik_steps_ << " budget_clock=" << ik_budget_name(config_.ik_budget);
    return {Error::E_IK_FAILED, detail.str()};
  }
  if (rc < 0) {
    std::ostringstream detail;
    detail << "IK did not converge: steps=" << last_ik_steps_ << "/" << config_.ik_iterations
           << " wall_us=" << elapsed / 1000.;
    return {Error::E_IK_FAILED, detail.str()};
  }
  output.fill(0.);  // Inactive capacity is never treated as a physical joint.
  for (size_t i = 0; i < config_.joint_count; ++i) {
    output[i] = result(i);
    if (!finite(output[i]) || output[i] < lower_[i] || output[i] > upper_[i]) {
      return {Error::E_OUT_OF_RANGE, "IK solution exceeds joint limits"};
    }
  }
  KDL::Frame actual;
  fk_->JntToCart(result, actual);
  const auto residual = KDL::diff(installation_ * actual * tool_, frame(pose));
  if (residual.vel.Norm() > config_.position_tolerance ||
    residual.rot.Norm() > config_.orientation_tolerance)
  {
    return {Error::E_IK_FAILED, "IK residual exceeds tolerance"};
  }
  return {};
}

Result HalConverter::forward(const Measurement & sample, geometry_msgs::msg::Pose & pose,
  geometry_msgs::msg::Twist & twist) const
{
  if (!ready_) {return {Error::E_TF, "robot model not initialized"};}
  KDL::Frame flange;
  const auto q = array(sample.position, config_.joint_count);
  if (fk_->JntToCart(q, flange) < 0) {return {Error::E_TF, "FK failed"};}
  const auto tcp = installation_ * flange * tool_;
  pose.position.x = tcp.p.x();
  pose.position.y = tcp.p.y();
  pose.position.z = tcp.p.z();
  tcp.M.GetQuaternion(pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);
  twist = geometry_msgs::msg::Twist{};
  if (sample.velocity_valid) {
    KDL::Jacobian jac(config_.joint_count);
    if (jacobian_->JntToJac(q, jac) < 0) {return {Error::E_TF, "Jacobian failed"};}
    KDL::Twist v;
    for (size_t row = 0; row < 6; ++row) {
      for (size_t col = 0; col < config_.joint_count; ++col) {v(row) += jac(row, col) * sample.velocity[col];}
    }
    // flange 속도에 omega × tool_offset을 더해야 TCP가 떨어져 있을 때도 정확하다.
    const auto linear = installation_.M * (v.vel + v.rot * (flange.M * tool_.p));
    const auto angular = installation_.M * v.rot;
    twist.linear.x = linear.x();
    twist.linear.y = linear.y();
    twist.linear.z = linear.z();
    twist.angular.x = angular.x();
    twist.angular.y = angular.y();
    twist.angular.z = angular.z();
  }
  return {};
}
}  // namespace pac_hal
