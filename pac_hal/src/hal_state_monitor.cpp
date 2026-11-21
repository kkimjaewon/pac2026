// Copyright 2026 Jaewon Kim

#include "pac_hal/hal_state_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <sstream>

namespace pac_hal
{
namespace
{
int64_t sim_stamp(const std_msgs::msg::Header & h)
{
  return static_cast<int64_t>(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
}
}  // namespace

bool HalStateMonitor::observe(const Measurement & sample,
  const HalConverter & converter, int64_t now)
{
  if (measurement_ && sim_stamp(sample.header) <= sim_stamp(measurement_->header)) {
    // pause/중복/역순 측정은 신선도를 갱신하지 않는다. sim reset은 새 세션으로 시작한다.
    return false;
  }
  if(!converter.ready()) {return false;}
  if(!sample.layout_valid) {
    measurement_failure_={Error::E_DEVICE,"joint feedback layout missing, partial or ambiguous"};
    return false;
  }
  for (size_t i = 0; i < config_.joint_count; ++i) {
    const char * detail=nullptr;
    if(!std::isfinite(sample.position[i])) {detail="nonfinite measured joint position";}
    else if(sample.position[i]<converter.lower()[i]-1e-6 || sample.position[i]>converter.upper()[i]+1e-6) {
      detail="measured joint position outside model/profile limits";
    } else if(sample.velocity_valid && !std::isfinite(sample.velocity[i])) {
      detail="nonfinite measured joint velocity";
    } else if(sample.velocity_valid && std::abs(sample.velocity[i])>config_.axes[i].velocity+1e-5) {
      detail="measured joint velocity exceeds profile limit";
    }
    if(detail) {
      measurement_failure_={Error::E_DEVICE,std::string(detail)+": joint="+std::to_string(i+1)};
      return false;
    }
  }
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  if (auto result=converter.forward(sample,pose,twist); !result) {
    measurement_failure_={Error::E_DEVICE,"measured FK failed: "+result.detail};return false;
  }
  const auto & p=pose.position;const auto & q=pose.orientation;
  const double qnorm=std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w));
  if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(qnorm) || std::abs(qnorm-1.)>1e-6) {
    measurement_failure_={Error::E_DEVICE,"invalid measured TCP pose from FK"};return false;
  }
  if(sample.velocity_valid) {
    const auto & v=twist.linear;const auto & w=twist.angular;
    const double speed=std::hypot(v.x,v.y,v.z);
    if(!std::isfinite(speed) || !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z) ||
      speed>config_.max_tcp_speed+1e-6) {
      std::ostringstream detail;detail<<"measured TCP speed exceeds profile limit or twist is nonfinite"
        <<": speed_m_s="<<speed<<" max_speed_m_s="<<config_.max_tcp_speed;
      measurement_failure_={Error::E_DEVICE,detail.str()};
      return false;
    }
  }
  measurement_failure_={};
  measurement_ = sample;
  pose_ = pose;
  twist_ = twist;
  received_ns_ = now;
  ++state_seq_;
  return true;
}

bool HalStateMonitor::observe_receiver(const Receiver & receiver, int64_t now)
{
  if (receiver.session_id != config_.session ||
    receiver.sampled_mono_ns <= 0 || receiver.sampled_mono_ns > now ||
    now - receiver.sampled_mono_ns >= config_.lost_ns ||
    (receiver_ && receiver.sampled_mono_ns <= receiver_->sampled_mono_ns)) {return false;}
  // Only a fresh own-session sample can diagnose a current device failure.
  // Foreign/replayed samples cannot poison or clear this health condition.
  auto reject=[this,&receiver](const std::string & reason) {
    receiver_feedback_failure_={Error::E_DEVICE,"invalid receiver feedback: "+reason+
      "; reported_seq="+std::to_string(receiver.last_applied_command_seq)};
    return false;
  };
  if(receiver.joint_count!=config_.joint_count) {return reject("joint_count mismatch");}
  // Bad device feedback must immediately disarm HAL without refreshing its cache.
  for (size_t i = 0; i < config_.joint_count; ++i) {
    if (!std::isfinite(receiver.commanded_positions[i]) ||
      !std::isfinite(receiver.commanded_velocities[i])) {
      return reject("nonfinite commanded position/velocity; device_axis="+std::to_string(i+1));
    }
  }
  if (receiver.applied_mono_ns < 0 || receiver.applied_mono_ns > receiver.sampled_mono_ns ||
    (receiver.last_applied_command_seq != 0 && receiver.applied_mono_ns == 0)) {
    return reject("invalid adoption timestamp");
  }
  if (receiver_ && (receiver.last_applied_command_seq < receiver_->last_applied_command_seq ||
    receiver.applied_mono_ns < receiver_->applied_mono_ns)) {return reject("adoption sequence/time regressed");}
  receiver_feedback_failure_={};
  receiver_ = receiver;
  receiver_received_ns_ = now;
  return true;
}

bool HalStateMonitor::measurement_lost(int64_t now) const
{
  return !measurement_ || now - received_ns_ >= config_.lost_ns;
}
bool HalStateMonitor::receiver_lost(int64_t now) const
{
  // 늦게 도착한 상태에 수신 시각 기준으로 신선도 기간을 다시 부여하지 않는다.
  // B/C와 수신단이 같은 PC의 monotonic clock을 사용한다는 계약에 따른다.
  return !receiver_ || now - receiver_->sampled_mono_ns >= config_.lost_ns;
}
bool HalStateMonitor::healthy(int64_t now) const
{
  return !measurement_rejected() && !receiver_feedback_rejected() && measurement_ && now - received_ns_ < config_.stale_ns &&
         !receiver_lost(now) && receiver_->active;
}
bool HalStateMonitor::receiver_rejected() const
{
  return receiver_ && receiver_->rejected;
}

std::string HalStateMonitor::health_detail(int64_t now) const
{
  std::ostringstream text;
  text << "state unavailable: measurement_age_us=" << (measurement_ ? (now - received_ns_) / 1000 : -1)
       << " receiver_age_us=" << (receiver_ ? (now - receiver_->sampled_mono_ns) / 1000 : -1)
       << " receiver_active=" << (receiver_ && receiver_->active);
  if(measurement_rejected()) {text<<" invalid_feedback="<<measurement_failure_.detail;}
  if(receiver_feedback_rejected()) {text<<" invalid_receiver="<<receiver_feedback_failure_.detail;}
  return text.str();
}

State HalStateMonitor::state(int64_t now, uint64_t command_seq, uint8_t command_result,
  const std::set<uint16_t> & errors) const
{
  State state;
  state.header.frame_id = config_.reference;
  state.hal_state = State::HAL_CONNECTING;
  state.active_session_id = config_.session;
  state.last_command_seq = command_seq;
  state.last_command_result = command_result;
  state.state_seq = state_seq_;
  state.received_mono_ns = received_ns_;
  state.stale = !measurement_ || now - received_ns_ >= config_.stale_ns;
  if (measurement_) {
    state.header.stamp = measurement_->header.stamp;
    state.tcp_pose = pose_;
    state.tcp_twist = twist_;
    state.pose_valid = !state.stale && !measurement_rejected();
    state.twist_valid = state.pose_valid && measurement_->velocity_valid;
    if (state.stale || measurement_rejected() || receiver_feedback_rejected() || receiver_lost(now) || !receiver_->active) {
      state.hal_state = State::HAL_FAULT;
    }
  }
  if (healthy(now)) {
    const bool moving = !measurement_->velocity_valid ||
      std::any_of(measurement_->velocity.begin(), measurement_->velocity.begin() + config_.joint_count,
        [](double v) {return !std::isfinite(v) || std::abs(v) > 1e-5;}) ||
      std::any_of(receiver_->commanded_velocities.begin(),
      receiver_->commanded_velocities.begin() + config_.joint_count, [](double v) {return std::abs(v) > 1e-5;});
    state.hal_state = moving ? State::HAL_ACTIVE : State::HAL_READY;
  }
  state.active_errors.assign(errors.begin(), errors.end());
  if(measurement_rejected() || receiver_feedback_rejected()) {state.hal_state=State::HAL_FAULT;}
  // 이 함수는 heartbeat에서도 호출되지만 received/state_seq를 바꾸지 않는다.
  return state;
}
}  // namespace pac_hal
