#include "rt_control/blend_policy.hpp"

#include <cmath>
#include <cstring>

namespace rt_control {

namespace {

// [0, 1]로 자르기. fmax/fmin은 NaN을 버리므로 NaN이 들어오면 0이 된다
inline double clamp01(double v) { return std::fmin(std::fmax(v, 0.0), 1.0); }

}  // namespace

bool parse_blend_mode(const char* name, BlendMode* out) {
  if (std::strcmp(name, "single") == 0) {
    *out = BlendMode::kSingle;
    return true;
  }
  return false;
}

bool sanitize_offset(double offset[3]) {
  bool changed = false;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(offset[i])) {
      offset[i] = 0.0;
      changed = true;
    }
  }
  return changed;
}

double blend_single(double goal_inout[3], const AiCommandSnapshot& cmd, bool is_new_and_pass,
                    int64_t age_ns, int64_t ttl_ns, const double* offset) {
  static constexpr double kZero[3] = {0.0, 0.0, 0.0};
  const double* off = offset ? offset : kZero;
  const double freshness =
      clamp01(1.0 - static_cast<double>(age_ns) / static_cast<double>(ttl_ns));
  const bool finite = std::isfinite(cmd.target[0]) & std::isfinite(cmd.target[1]) &
                      std::isfinite(cmd.target[2]);
  const bool use = is_new_and_pass & finite;
  const double w = clamp01(cmd.confidence) * freshness * static_cast<double>(use);

  for (int i = 0; i < 3; ++i) {
    // NaN × 0 = NaN 이므로, 쓰지 않을 때는 AI 위치를 지금 목표로 바꿔치기 (차이 = 정확히 0)
    const double src = use ? cmd.target[i] + off[i] : goal_inout[i];
    goal_inout[i] += w * (src - goal_inout[i]);
  }
  return w;
}

BlendPolicy::BlendPolicy(const BlendConfig& config) : config_(config) {
  sanitize_offset(config_.target_offset);
  for (int i = 0; i < 3; ++i) goal_[i] = config.initial_goal[i];
}

bool BlendPolicy::is_new(const AiCommandSnapshot& cmd, const GateResult& gate) const {
  const bool has_data = (gate.reasons & GATE_NO_DATA) == 0;
  return has_data & (!has_seen_ | (cmd.writer_boot_id != last_boot_id_) | (cmd.seq != last_seq_));
}

void BlendPolicy::reset_goal(const double pos[3]) {
  for (int i = 0; i < 3; ++i) goal_[i] = pos[i];
}

bool BlendPolicy::reject_last_applied() {
  if (!has_used_) return false;
  for (int i = 0; i < 3; ++i) rejected_target_[i] = last_applied_target_[i];
  rejected_active_ = true;
  return true;
}

BlendResult BlendPolicy::update(const AiCommandSnapshot& cmd, const GateResult& gate, bool allow) {
  const bool is_new = this->is_new(cmd, gate);
  // 거부된 목표 대기 중이면, 그 목표와 rejected_goal_tol_m 안인 새 명령은 반영하지 않음
  // (NaN 위치면 거리도 NaN -> 비교가 false라 막지 않지만, 어차피 blend_single이 쓰지 않음)
  const double rx = cmd.target[0] + config_.target_offset[0] - rejected_target_[0];
  const double ry = cmd.target[1] + config_.target_offset[1] - rejected_target_[1];
  const double rz = cmd.target[2] + config_.target_offset[2] - rejected_target_[2];
  const bool near_rejected = std::sqrt(rx * rx + ry * ry + rz * rz) < config_.rejected_goal_tol_m;
  const bool rejected_skip = rejected_active_ & near_rejected & is_new & gate.pass & allow;
  const bool apply = is_new & gate.pass & allow & !rejected_skip;

  // 새 명령이면 gate 결과와 상관없이 "봤다"고 표시 (차단된 명령이 나중에 다시 반영되지 않게)
  has_seen_ = has_seen_ | is_new;
  last_boot_id_ = is_new ? cmd.writer_boot_id : last_boot_id_;
  last_seq_ = is_new ? cmd.seq : last_seq_;

  // 후보 목표를 따로 계산한 뒤, 움직이는 거리가 deadband 이상일 때만 지금 목표로 받아들임
  double cand[3] = {goal_[0], goal_[1], goal_[2]};
  double w = 0.0;
  switch (config_.mode) {
    case BlendMode::kSingle:
      w = blend_single(cand, cmd, apply, gate.age_ns, config_.ttl_ns, config_.target_offset);
      break;
  }
  const double dx = cand[0] - goal_[0], dy = cand[1] - goal_[1], dz = cand[2] - goal_[2];
  const bool in_deadband = std::sqrt(dx * dx + dy * dy + dz * dz) < config_.deadband_m;
  const bool candidate = w > 0.0;  // 위치가 NaN·무한대여서 못 쓴 경우도 "반영 안 함"
  const bool applied = candidate & !in_deadband;
  rejected_active_ = rejected_active_ & !candidate;  // 거부된 목표와 다른 목표가 들어오면 대기 끝
  for (int i = 0; i < 3; ++i) {
    goal_[i] = applied ? cand[i] : goal_[i];
    last_applied_target_[i] = applied ? cmd.target[i] + config_.target_offset[i] : last_applied_target_[i];
  }
  used_ai_seq_ = applied ? cmd.seq : used_ai_seq_;
  used_ai_boot_id_ = applied ? cmd.writer_boot_id : used_ai_boot_id_;
  has_used_ = has_used_ | applied;

  const bool skipped = candidate & in_deadband;
  return BlendResult{{goal_[0], goal_[1], goal_[2]}, applied ? w : 0.0, applied, used_ai_seq_, has_used_,
                     used_ai_boot_id_, skipped, rejected_active_, rejected_skip};
}

}  // namespace rt_control
