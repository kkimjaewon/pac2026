#include "rt_control/init_hold.hpp"

#include <cmath>

namespace rt_control {

bool fresh_pose(bool has_state, const HalStateSnapshot& s, int64_t now_ns, int64_t state_timeout_ns, double pos[3],
                double quat[4]) {
  if (!has_state || !s.pose_valid || s.stale) return false;
  const int64_t age = now_ns - s.recv_mono_ns;
  if (age < 0 || age >= state_timeout_ns) return false;
  if (!std::isfinite(s.tcp_pos[0]) || !std::isfinite(s.tcp_pos[1]) || !std::isfinite(s.tcp_pos[2])) return false;
  const double* q = s.tcp_quat;
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(n) || n < 1e-6) return false;
  for (int i = 0; i < 3; ++i) pos[i] = s.tcp_pos[i];
  for (int i = 0; i < 4; ++i) quat[i] = q[i] / n;
  return true;
}

InitHoldResult InitHold::step(bool has_state, const HalStateSnapshot& state, int64_t now_ns, uint64_t& session_id,
                              HalCommandSnapshot& cmd, const HalCommandConfig& base) {
  InitHoldResult r{false, false, InitHoldEvent::kNone};
  double pos[3], quat[4];
  if (!fresh_pose(has_state, state, now_ns, config_.state_timeout_ns, pos, quat)) {
    if (holding_) r.event = InitHoldEvent::kLost;
    holding_ = false;
    return r;
  }
  if (session_id == 0) {
    session_id = choose_session_id(config_.registered_session_id, 0, now_ns);
    cmd = HalCommandSnapshot{};
    // 고정 세션이고 C가 같은 세션의 마지막 순번을 보고하면 그 다음부터 (B 재시작 시 옛 순번으로 거부되지 않게)
    if (config_.registered_session_id != 0 && state.active_session_id == config_.registered_session_id) {
      cmd.command_seq = state.last_command_seq;
    }
    r.session_issued = true;
  }
  HalCommandConfig c = base;
  for (int i = 0; i < 4; ++i) c.tool_orientation[i] = quat[i];
  fill_hal_command(cmd, session_id, now_ns, pos, false, 0, c);
  if (!holding_) r.event = InitHoldEvent::kStarted;
  holding_ = true;
  for (int i = 0; i < 3; ++i) hold_pos_[i] = pos[i];
  for (int i = 0; i < 4; ++i) hold_quat_[i] = quat[i];
  ++published_;
  r.publish = true;
  return r;
}

}  // namespace rt_control
