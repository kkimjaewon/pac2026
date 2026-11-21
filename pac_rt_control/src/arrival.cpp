#include "rt_control/arrival.hpp"

#include <cmath>
#include <limits>

namespace rt_control {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline double dist3(const double a[3], const double b[3]) {
  const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

ArrivalResult ArrivalDetector::update(const double goal[3], const double tcp[3], bool tcp_valid,
                                      int64_t now_ns) {
  const double err = tcp_valid ? dist3(goal, tcp) : kNaN;
  const bool inside = err <= config_.tol_m;  // NaN이면 false
  since_ns_ = (inside & !inside_) ? now_ns : since_ns_;
  inside_ = inside;
  const bool held = inside & (now_ns - since_ns_ >= config_.hold_ns);
  return ArrivalResult{held, err};
}

ResumeChoice choose_resume_position(bool resume_from_last_cmd, double match_tol_m, bool has_last_cmd,
                                    const double last_cmd[3], bool robot_valid, const double robot[3],
                                    const double output[3]) {
  ResumeChoice c{};
  c.diff_m = kNaN;
  const double* src = output;
  c.source = ResumeSource::kOutput;
  if (robot_valid) {
    src = robot;
    c.source = ResumeSource::kRobot;
    if (resume_from_last_cmd && has_last_cmd) {
      c.diff_m = dist3(last_cmd, robot);
      const bool match = c.diff_m <= match_tol_m;  // NaN이면 false
      src = match ? last_cmd : robot;
      c.source = match ? ResumeSource::kLastCommand : ResumeSource::kRobotMismatch;
    }
  }
  for (int i = 0; i < 3; ++i) c.pos[i] = src[i];
  return c;
}

}  // namespace rt_control
