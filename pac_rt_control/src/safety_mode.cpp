#include "rt_control/safety_mode.hpp"

#include <algorithm>

#include "rt_control/gate.hpp"
#include "rt_control/hal_state_snapshot.hpp"

namespace rt_control {

const char* safety_mode_name(SafetyMode m) {
  switch (m) {
    case SafetyMode::kRun: return "RUN";
    case SafetyMode::kHold: return "HOLD";
    case SafetyMode::kDecelStop: return "DECEL_STOP";
    case SafetyMode::kFault: return "FAULT";
  }
  return "?";
}

SafetyStateMachine::SafetyStateMachine(const SafetyConfig& config) : config_(config) {
  config_.c_expired_n = std::clamp(config_.c_expired_n, 1, kMaxExpiredCount);
}

void SafetyStateMachine::reset(int64_t now_ns) {
  mode_ = SafetyMode::kRun;
  mode_since_ns_ = now_ns;
  hold_latched_ = false;
  hold_reasons_ = 0;
  resume_count_ = 0;
  range_reject_count_ = 0;
  decel_latched_ = false;
  decel_reasons_ = 0;
  decel_clear_ = false;
  fault_latched_ = false;
  fault_reasons_ = 0;
  // 켜진 오류 목록·E_EXPIRED 기록은 C 쪽 사실이라 그대로 둔다
}

void SafetyStateMachine::set_session(uint64_t session_id) {
  if (session_id == session_id_) return;
  session_id_ = session_id;
  event_error_mask_ = 0;
  expired_head_ = 0;
  expired_total_ = 0;
}

bool SafetyStateMachine::on_error_event(uint16_t code, bool active, int64_t now_ns, uint64_t session_id) {
  if (session_id != session_id_) {
    ++ignored_error_events_;
    return false;
  }
  if (code < 64) {
    const uint64_t bit = err_bit(code);
    event_error_mask_ = active ? (event_error_mask_ | bit) : (event_error_mask_ & ~bit);
  }
  if (code == kErrExpired && active) {
    expired_times_[expired_head_] = now_ns;
    expired_head_ = (expired_head_ + 1) % kMaxExpiredCount;
    expired_total_ = std::min(expired_total_ + 1, kMaxExpiredCount);
  }
  return true;
}

uint32_t SafetyStateMachine::hold_conditions(const SafetyInput& in, uint64_t /*errors*/) {
  const uint32_t g = in.gate_reasons;
  uint32_t r = 0;
  if (g & GATE_NO_DATA) r |= SAFE_AI_NO_DATA;
  if (g & GATE_EXPIRED) r |= SAFE_AI_EXPIRED;
  if (g & GATE_STATUS) r |= SAFE_AI_STATUS;
  if (g & GATE_LOW_CONF) r |= SAFE_AI_LOW_CONF;

  // 새 명령이 RANGE로 막히면 +1, 새 명령이 RANGE 없이 오면 0부터 다시
  if (in.is_new_command) {
    range_reject_count_ = (g & GATE_RANGE) ? std::min(range_reject_count_ + 1, config_.range_reject_n) : 0;
  }
  if (range_reject_count_ >= config_.range_reject_n) r |= SAFE_AI_RANGE;

  // E_EXPIRED가 최근 c_expired_window 안에 c_expired_n번 이상: n번째로 최근 것이 창 안에 있으면 됨
  const int n = config_.c_expired_n;
  if (config_.require_c_state && expired_total_ >= n) {
    const int64_t t = expired_times_[(expired_head_ - n + kMaxExpiredCount) % kMaxExpiredCount];
    if (in.now_ns - t < config_.c_expired_window_ns) r |= SAFE_C_EXPIRED;
  }
  return r;
}

uint32_t SafetyStateMachine::decel_conditions(const SafetyInput& in, uint64_t errors) const {
  uint32_t r = 0;
  if (hold_latched_ && config_.ai_decel_after_ns > 0 && in.now_ns - hold_since_ns_ > config_.ai_decel_after_ns) {
    r |= SAFE_HOLD_TIMEOUT;
  }
  if (config_.require_c_state) {
    if (!in.c_has_state) {
      r |= SAFE_C_NO_STATE;
    } else {
      if (in.c_state_age_ns > config_.c_state_timeout_ns) r |= SAFE_C_STATE_TIMEOUT;
      if (in.hal_state != HAL_READY && in.hal_state != HAL_ACTIVE) r |= SAFE_C_NOT_READY;
    }
    if (errors & kDecelErrorMask) r |= SAFE_C_ERROR_DECEL;
  }
  return r;
}

uint32_t SafetyStateMachine::fault_conditions(const SafetyInput& in, uint64_t errors) const {
  uint32_t r = 0;
  if (in.gate_reasons & GATE_VERSION) r |= SAFE_AI_VERSION;
  if (config_.require_c_state && (errors & kFaultErrorMask)) r |= SAFE_C_ERROR_FAULT;
  return r;
}

SafetyOutput SafetyStateMachine::update(const SafetyInput& in) {
  const uint64_t errors = config_.require_c_state ? (in.c_error_mask | event_error_mask_) : 0;
  const bool stopped = in.limiter_speed <= config_.stopped_speed;

  // ---- HOLD 래치 ----
  const uint32_t hold = hold_conditions(in, errors);
  if (hold) {
    if (!hold_latched_) {
      hold_latched_ = true;
      hold_since_ns_ = in.now_ns;
      hold_reasons_ = 0;
    }
    hold_reasons_ |= hold;
    resume_count_ = 0;  // 조건이 남아 있는 동안은 세지 않음
  } else if (hold_latched_ && in.is_new_command) {
    // 새 명령이 통과하면 +1, 막히면 처음부터 다시
    resume_count_ = in.gate_pass ? resume_count_ + 1 : 0;
    if (resume_count_ >= config_.resume_k) {
      hold_latched_ = false;
      resume_count_ = 0;
    }
  } else if (hold_latched_ && config_.resume_k <= 0) {
    hold_latched_ = false;
  }

  // ---- DECEL_STOP 래치 (HOLD_TIMEOUT은 위의 HOLD 래치 결과를 씀) ----
  const uint32_t decel = decel_conditions(in, errors);
  if (decel) {
    if (!decel_latched_) {
      decel_latched_ = true;
      decel_reasons_ = 0;
    }
    decel_reasons_ |= decel;
    decel_clear_ = false;
  } else if (decel_latched_) {
    if (!decel_clear_) {
      decel_clear_ = true;
      decel_clear_since_ns_ = in.now_ns;
    }
    if (in.now_ns - decel_clear_since_ns_ >= config_.resume_delay_ns && stopped) decel_latched_ = false;
  }

  // ---- FAULT 래치 ----
  const uint32_t fault = fault_conditions(in, errors);
  if (fault) {
    if (!fault_latched_) {
      fault_latched_ = true;
      fault_reasons_ = 0;
    }
    fault_reasons_ |= fault;
  }
  ResetResult reset_result = ResetResult::kNone;
  bool new_session = false;
  if (in.reset_request) {
    if (!fault_latched_) {
      reset_result = ResetResult::kNotFault;
    } else if (fault) {
      reset_result = ResetResult::kCauseActive;
    } else if (!stopped) {
      reset_result = ResetResult::kMoving;
    } else {
      fault_latched_ = false;
      new_session = true;
      reset_result = ResetResult::kAccepted;
    }
  }

  // ---- 지금 모드 = 걸린 래치 중 가장 높은 것 ----
  const SafetyMode prev = mode_;
  mode_ = fault_latched_   ? SafetyMode::kFault
          : decel_latched_ ? SafetyMode::kDecelStop
          : hold_latched_  ? SafetyMode::kHold
                           : SafetyMode::kRun;
  const bool changed = mode_ != prev;
  if (changed) mode_since_ns_ = in.now_ns;
  const bool was_stopping = prev == SafetyMode::kDecelStop || prev == SafetyMode::kFault;
  const bool now_moving = mode_ == SafetyMode::kRun || mode_ == SafetyMode::kHold;

  SafetyOutput out{};
  out.mode = mode_;
  out.prev_mode = prev;
  out.changed = changed;
  out.allow_blend = mode_ == SafetyMode::kRun;
  out.stop = mode_ == SafetyMode::kDecelStop || mode_ == SafetyMode::kFault;
  switch (mode_) {
    case SafetyMode::kFault: out.reasons = fault_reasons_; break;
    case SafetyMode::kDecelStop: out.reasons = decel_reasons_; break;
    case SafetyMode::kHold: out.reasons = hold_reasons_; break;
    case SafetyMode::kRun: out.reasons = 0; break;
  }
  out.active_reasons = hold | decel | fault;
  out.mode_duration_ns = in.now_ns - mode_since_ns_;
  out.reset_limiter = was_stopping && now_moving;
  out.new_session = new_session;
  out.reset_result = reset_result;
  out.c_error_mask = errors;
  return out;
}

}  // namespace rt_control
