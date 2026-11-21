// Copyright 2026 Jaewon Kim

#include "pac_hal/hal_command_gateway.hpp"
#include "pac_hal/receiver_contract.hpp"

#include <cmath>
#include <utility>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include "pac_hal/joint_trajectory.hpp"
#include "pac_hal/tcp_trajectory.hpp"

namespace pac_hal
{
HalCommandGateway::HalCommandGateway(rclcpp::Node & node, const Config & config,
  const std::string & scene_file, const std::string & expected_xml)
: node_(node), config_(config), converter_(config), rate_(config.backend_period_ns), scene_(config, scene_file),
  monitor_(config), trace_(config)
{
  if (scene_file.empty()) {throw std::runtime_error("executable HAL requires a collision scene");}
  if (expected_xml.empty()) {throw std::runtime_error("executable HAL requires a prepared model snapshot");}
  description_guard_.bind(expected_xml);
  state_pub_ = node_.create_publisher<State>(config_.state_topic,
    rclcpp::QoS(1).best_effort().durability_volatile());
  error_pub_ = node_.create_publisher<Error>(config_.error_topic,
    rclcpp::QoS(32).reliable().durability_volatile());
  preflight_pub_ = node_.create_publisher<std_msgs::msg::String>("/hal/preflight",
    rclcpp::QoS(1).reliable().transient_local());
  auto startup = config_;startup.ik_budget_ns = 20000000;
  startup_converter_ = std::make_unique<HalConverter>(startup);
  AdapterCallbacks callbacks;
  callbacks.receiver_diagnostic=[this](const std::string & payload,int64_t now) {
    trace_.receiver_diagnostic(payload,now);
    // Receiver expiry uses the same EXPIRED contract as a C-side late command,
    // with the exact rejected packet identity. The next sent command clears it.
    uint64_t session=0,seq=0;
    if(parse_receiver_expiry(payload,session,seq) && session==config_.session) {
      Command context=last_command_;context.session_id=session;context.command_seq=seq;
      context.header.frame_id=config_.reference;
      transition(Error::E_EXPIRED,true,context,"expired at receiver; packet not adopted");
      publish_state();
    }
  };
  callbacks.description = [this](const std::string & xml) {
      if (!description_guard_.observe(xml)) {
        preflight_done_=true;
        preflight_result_={Error::E_TF,"robot description changed after model binding; restart and repeat preflight"};
        publish_preflight();
        Command context=last_command_;context.session_id=config_.session;context.header.frame_id=config_.reference;
        transition(Error::E_TF,true,context,preflight_result_.detail);
        publish_state();
        return;
      }
      if (description_guard_.bound() && converter_.ready() && scene_.ready() &&
        startup_converter_->ready()) {return;}
      auto result = converter_.initialize(xml);
      // Bind as soon as one immutable component initializes. If collision/model
      // preparation later fails, never mix it with a different retry XML.
      if (result) {description_guard_.bind(xml);}
      if (result) {result = scene_.initialize(xml);}
      if (result) {result = startup_converter_->initialize(xml);}
      if (!result) {
        preflight_done_=true;
        preflight_result_={result.code,"model initialization failed; restart and repeat preflight: "+result.detail};
        publish_preflight();
        Command context=last_command_;context.session_id=config_.session;context.header.frame_id=config_.reference;
        transition(result.code,true,context,preflight_result_.detail);
        RCLCPP_ERROR(node_.get_logger(), "%s", preflight_result_.detail.c_str());
        publish_state();
      }
    };
  callbacks.measurement = [this](const Measurement & sample) {
      const auto now = monotonic_ns();
      if (monitor_.observe(sample, converter_, now)) {
        const int64_t stamp = static_cast<int64_t>(sample.header.stamp.sec) * 1000000000LL +
          sample.header.stamp.nanosec;
        if (last_measurement_callback_) {
          trace_.feedback_gap("joint_sim_time", now, stamp,
            now - last_measurement_callback_, stamp - last_measurement_stamp_);
        }
        last_measurement_callback_ = now;
        last_measurement_stamp_ = stamp;
        const auto collision = scene_.check(sample.position);
        measured_collision_ = !collision;
        if(measured_collision_) {measured_protection_detail_=collision.detail;}
        else {measured_protection_detail_.clear();}
        // 현재 실제 자세가 충돌하면 READY를 표시하지 않고 명령 수락을 막는다.
        Command context = last_command_;
        context.session_id = config_.session;
        context.header.frame_id = config_.reference;
        transition(Error::E_DEVICE, measured_collision_ || monitor_.receiver_rejected() || monitor_.receiver_feedback_rejected(), context,
          !measured_protection_detail_.empty() ? measured_protection_detail_ :
          monitor_.receiver_feedback_rejected() ? monitor_.receiver_feedback_failure_detail() :
          monitor_.receiver_rejected() ? "receiver rejected a joint packet; see receiver diagnostics" :
          "measured protection cleared; receiver status applied");
        // 유효한 실제 측정과 수신단이 복구되면 heartbeat를 기다리지 않고 해제한다.
        // 중복/역순 측정은 observe에서 제외되므로 오류 해제 근거가 될 수 없다.
        if (command_seen_ && monitor_.healthy(now)) {
          transition(Error::E_STATE_STALE, false, last_command_, "fresh state restored");
        }
        publish_state();
      } else if(monitor_.measurement_rejected()) {
        Command context=last_command_;context.session_id=config_.session;context.header.frame_id=config_.reference;
        transition(Error::E_DEVICE,true,context,monitor_.measurement_failure_detail());
        publish_state();
      }
    };
  callbacks.receiver = [this](const Receiver & receiver) {
      const auto now = monotonic_ns();
      if (!monitor_.observe_receiver(receiver, now)) {
        if(monitor_.receiver_feedback_rejected()) {
          Command context=last_command_;context.session_id=config_.session;context.header.frame_id=config_.reference;
          transition(Error::E_DEVICE,true,context,monitor_.receiver_feedback_failure_detail());
          publish_state();
        }
        return;
      }
      if (last_receiver_callback_) {
        trace_.feedback_gap("receiver_mono_time", now, receiver.sampled_mono_ns,
          now - last_receiver_callback_, receiver.sampled_mono_ns - last_receiver_stamp_);
      }
      last_receiver_callback_ = now;
      last_receiver_stamp_ = receiver.sampled_mono_ns;
      trace_.applied(receiver, now);
      if (command_seen_) {
        // ACK만으로 측정 신선도를 복구하지 않는다. healthy일 때만 통신 오류를 해제한다.
        if (monitor_.healthy(now)) {
          transition(Error::E_STATE_STALE, false, last_command_, "fresh state restored");
          transition(Error::E_DISCONNECTED, false, last_command_, "receiver restored");
        }
      }
      Command context=last_command_;context.session_id=config_.session;context.header.frame_id=config_.reference;
      transition(Error::E_DEVICE, receiver.rejected || monitor_.measurement_rejected() || monitor_.receiver_feedback_rejected() || measured_collision_, context,
          monitor_.measurement_rejected() ? monitor_.measurement_failure_detail() :
          monitor_.receiver_feedback_rejected() ? monitor_.receiver_feedback_failure_detail() :
          !measured_protection_detail_.empty() ? measured_protection_detail_ : receiver.rejected ?
          "receiver rejected a joint packet; see receiver diagnostics" : "receiver and measured protection restored");
    };
  adapter_ = make_gazebo_adapter(node_, config_, std::move(callbacks));
}

Result HalCommandGateway::validate(const Command & c, int64_t now, bool sequence)
{
  if (c.session_id == 0 || c.session_id != config_.session) {
    return {Error::E_FORMAT, "B and C session_id must match"};
  }
  if (sequence) {
    if (auto result=observe_command_sequence(c,config_.session,observed_seq_);!result) {
      return result;
    }
  }
  if (c.mode != Command::MODE_POSITION || c.header.frame_id != config_.reference) {
    return {Error::E_UNSUPPORTED, "only absolute map TCP position mode is supported"};
  }
  if (c.created_mono_ns < 0 || c.created_mono_ns > now || c.expires_mono_ns <= c.created_mono_ns ||
    c.expires_mono_ns - c.created_mono_ns != config_.ttl_ns)
  {
    return {Error::E_FORMAT, "invalid monotonic clock or 5ms TTL"};
  }
  if (now >= c.expires_mono_ns) {return {Error::E_EXPIRED, "expired command"};}
  const auto & p = c.target_pose.position;
  const auto & q = c.target_pose.orientation;
  const std::array<double, 3> xyz{p.x, p.y, p.z};
  const auto norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || std::abs(norm - 1.) > 1e-6) {
    return {Error::E_FORMAT, "quaternion xyzw must be finite and normalized"};
  }
  for (size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(xyz[i])) {return {Error::E_FORMAT, "nonfinite TCP position"};}
    if (xyz[i] < config_.workspace_min[i] || xyz[i] > config_.workspace_max[i]) {
      return {Error::E_OUT_OF_RANGE, "TCP outside configured workspace"};
    }
  }
  // All own-session attempts already consumed their identity before checks.
  return {};
}

Result HalCommandGateway::joint_limits(const Joints & q, const MotionBaseline & b)
{
  const auto & previous = last_q_ ? *last_q_ : monitor_.measurement().position;
  return check_joint_trajectory(config_, converter_.lower(), converter_.upper(), q,
    previous, b.restarted ? Joints{} : last_velocity_, b.dt, b.previous_dt);
}

void HalCommandGateway::command(const Command & c)
{
  TraceRecord r;
  r.session=c.session_id;r.seq=c.command_seq;r.source_result_id=c.source_result_id;
  r.created=c.created_mono_ns;r.t1=monotonic_ns();
  if (config_.backend_period_ns == config_.period_ns) {execute(c,r);return;}
  const bool new_active_attempt=is_new_active_attempt(c,config_.session,observed_seq_);
  const auto result=validate(c,r.t1);r.t2=monotonic_ns();r.validated=r.t2;
  if (!result) {
    // A new own-session invalid target cancels its earlier target. Foreign,
    // duplicate and replayed packets cannot discard a verified pending command.
    if (new_active_attempt) {
      if (auto pending=rate_.cancel()) {pending->trace.superseded=true;trace_.command(pending->trace);}
    }
    execute(c,r,true,result,new_active_attempt);return;
  }
  if (auto replaced=rate_.offer(c,r)) {replaced->trace.superseded=true;trace_.command(replaced->trace);}
  record_command_progress(c,config_.session,State::CMD_ACCEPTED,last_command_seq_,last_command_result_);
  dispatch();
}

void HalCommandGateway::dispatch()
{
  if (auto pending=rate_.take(monotonic_ns())) {
    execute(pending->command,pending->trace,true);
    rate_.completed(monotonic_ns());
  }
}

void HalCommandGateway::execute(const Command & c, TraceRecord r, bool queued, Result previous,
  bool active_attempt)
{
  // Foreign-session, zero, duplicate or replayed packets are not the active
  // session's latest outcome. They must not clear its per-command errors.
  if (!queued) {active_attempt = is_new_active_attempt(c,config_.session,observed_seq_);}
  record_active_command_context(c,config_.session,config_.reference,last_command_,command_seen_);
  auto result = previous ? validate(c, monotonic_ns(), !queued) : previous;
  r.t2 = monotonic_ns();
  update_preflight();
  if (result && !preflight_done_) {result={Error::E_STATE_STALE,"preflight waiting for fresh device/model feedback"};}
  if (result && !preflight_result_) {result=preflight_result_;}
  if (result && (!converter_.ready() || !scene_.ready())) {
    result = {Error::E_TF, "kinematic/collision model unavailable"};
  }
  if (result && (monitor_.measurement_rejected() || monitor_.receiver_feedback_rejected())) {
    result={Error::E_DEVICE,monitor_.measurement_rejected() ? monitor_.measurement_failure_detail() :
      monitor_.receiver_feedback_failure_detail()};
  }
  if (result && !monitor_.healthy(r.t2)) {
    result = {Error::E_STATE_STALE, monitor_.health_detail(r.t2)};
  }
  std::array<double,3> tcp_velocity{};
  // last_q_ and last_tcp_target_ are always updated together on a successful send.
  const auto baseline=motion_baseline(last_q_.has_value(),c.created_mono_ns,last_created_ns_,
    last_dt_,config_.period_ns,config_.receiver_watchdog_ns);
  if(result) {
    result=check_tcp_trajectory(config_,c.target_pose,
      last_tcp_target_ ? *last_tcp_target_ : monitor_.measured_pose(),
      baseline.restarted ? std::array<double,3>{} : last_tcp_velocity_,
      baseline.dt,baseline.previous_dt,tcp_velocity);
  }
  r.validated = monotonic_ns();
  Joints solution{};
  // Receiver rejection is reported as DEVICE/FAULT. Keep measured collision a
  // hard block, but let a fresh fully validated command clear a transient rejected
  // packet at the receiver. Blocking every command here makes recovery impossible.
  if (result && (measured_collision_)) {
    result = {Error::E_DEVICE, "measured protection violation: "+measured_protection_detail_};
  }
  if (result) {
    // Seed from the last solution to keep either robot on a continuous IK branch.
    const auto & seed = last_q_ ? *last_q_ : monitor_.measurement().position;
    result = converter_.inverse(c.target_pose, seed, solution, c.expires_mono_ns);
    r.ik_end = monotonic_ns();
    r.ik_wall_ns = converter_.last_ik_wall_ns();
    r.ik_cpu_ns = converter_.last_ik_cpu_ns();
    r.ik_budget = config_.ik_budget;
    if (result) {result = joint_limits(solution, baseline);}
    if (result) {
      // 실제 측정→직전 목표→새 목표를 검사한다. endpoint만 검사해 경유 충돌을 놓치지 않는다.
      result = scene_.check_path(monitor_.measurement().position, seed, c.expires_mono_ns);
      if (result) {result = scene_.check_path(seed, solution, c.expires_mono_ns);}
    }
    r.t3 = monotonic_ns();
  }
  if (result) {
    r.t4 = monotonic_ns();
    if (!adapter_->ready()) {
      result = {Error::E_SEND_FAILED, "backend command channel unavailable"};
    } else if (!monitor_.healthy(r.t4)) {
      result = {Error::E_STATE_STALE, "measurements became stale before transport"};
    } else {
      r.transport_start = monotonic_ns();
      result = adapter_->send(c, solution);
      r.transport_end = monotonic_ns();
    }
    if (result) {
      const double dt = baseline.dt;
      const auto & previous = last_q_ ? *last_q_ : monitor_.measurement().position;
      for (size_t i = 0; i < config_.joint_count; ++i) {last_velocity_[i] = (solution[i] - previous[i]) / dt;}
      last_q_ = solution;
      last_tcp_target_=c.target_pose;last_tcp_velocity_=tcp_velocity;
      // 전송 성공 구간만 저장한다. 거부된 명령으로 다음 가속도의 기준을 바꾸지 않는다.
      last_dt_ = dt;
      last_created_ns_ = c.created_mono_ns;
      r.sent = true;
    }
  }
  r.error = result.code;
  record_command_progress(c,config_.session,r.sent ? State::CMD_SENT : State::CMD_REJECTED,
    last_command_seq_,last_command_result_);
  // 한 명령의 오류 변화만 정리한다. 측정 중단/장치 오류는 별도 건강 상태로 유지한다.
  if (!active_attempt && !result) {
    // Report the identity rejection only; keep the active session's other errors.
    transition(result.code, true, c, result.detail);
    trace_.command(r);
    return;
  }
  for (const auto code : {Error::E_FORMAT, Error::E_UNSUPPORTED, Error::E_EXPIRED,
      Error::E_OUT_OF_RANGE, Error::E_IK_FAILED, Error::E_TF, Error::E_SEND_FAILED})
  {
    if (code==Error::E_TF && description_guard_.changed()) {
      transition(code,true,last_command_,preflight_result_.detail);
      continue;
    }
    transition(code, code == result.code, c,
      code == result.code ? result.detail : "condition cleared");
  }
  if (result.code == Error::E_STATE_STALE) {transition(result.code, true, c, result.detail);}
  trace_.command(r);
}

void HalCommandGateway::transition(uint16_t code, bool active, const Command & c,
  const std::string & detail)
{
  if ((errors_.count(code) != 0) == active) {return;}
  if (active) {errors_.insert(code);} else {errors_.erase(code);}
  Error event;
  event.header = c.header;
  event.code = code;
  event.active = active;
  event.event_mono_ns = monotonic_ns();
  event.session_id = c.session_id;
  event.command_seq = c.command_seq;
  event.source = "hal_gateway";
  event.detail = detail;
  error_pub_->publish(event);
  trace_.error(event);
}

void HalCommandGateway::heartbeat()
{
  const auto now = monotonic_ns();
  if (command_seen_) {
    // 명령 수락과 같은 건강 조건을 사용해 stale 상태의 오류를 조기에 지우지 않는다.
    transition(Error::E_STATE_STALE, !monitor_.healthy(now), last_command_,
      "fresh measured pose and active receiver are required");
    transition(Error::E_DISCONNECTED, monitor_.receiver_lost(now), last_command_,
      "receiver status missing or stopped");
  }
  publish_state();
}
void HalCommandGateway::update_preflight()
{
  if (preflight_done_ || !monitor_.healthy(monotonic_ns()) || !adapter_->ready() ||
    !startup_converter_->ready() || !scene_.ready()) {return;}
  preflight_result_ = preflight(config_, *startup_converter_, scene_, monitor_.measurement());
  preflight_done_=true;
  publish_preflight();
  if (!preflight_result_) {
    Command context;context.session_id=config_.session;context.header.frame_id=config_.reference;
    transition(preflight_result_.code,true,context,"preflight: "+preflight_result_.detail);
  }
}

void HalCommandGateway::publish_preflight()
{
  std::ostringstream report;
  report << "{\"status\":" << json_string(preflight_result_ ?
    (config_.backend_period_ns > config_.task.preferred_period_ns ? "READY_WITH_LIMIT" : "COMPATIBLE") : "INCOMPATIBLE")
    << ",\"model\":" << json_string(config_.model) << ",\"backend\":" << json_string(config_.backend)
    << ",\"joint_count\":" << config_.joint_count << ",\"session_id\":" << config_.session
    << ",\"run_id\":" << json_string(config_.run_id)
    << ",\"model_change_policy\":\"restart_and_repeat_preflight\""
    << ",\"model_binding\":\"receiver_prepared_snapshot\""
    << ",\"input_period_ns\":" << config_.period_ns << ",\"backend_period_ns\":" << config_.backend_period_ns
    << ",\"sample_count\":" << config_.task.samples.size()+1
    << ",\"error_code\":" << preflight_result_.code << ",\"detail\":" << json_string(preflight_result_.detail)
    << ",\"task_contract\":" << describe_task_contract(config_)
    << ",\"robot_contract\":" << describe_robot_contract(config_) << "}";
  std_msgs::msg::String message;message.data=report.str();preflight_pub_->publish(message);
  RCLCPP_INFO(node_.get_logger(), "Pre-flight %s", message.data.c_str());
}

void HalCommandGateway::publish_state()
{
  update_preflight();
  auto state = monitor_.state(monotonic_ns(), last_command_seq_, last_command_result_, errors_);
  if (description_guard_.changed()) {state.pose_valid=false;state.twist_valid=false;}
  if ((state.hal_state == State::HAL_READY || state.hal_state == State::HAL_ACTIVE) &&
    !adapter_->ready()) {state.hal_state = State::HAL_CONNECTING;}
  if (!preflight_done_ && (state.hal_state == State::HAL_READY || state.hal_state == State::HAL_ACTIVE)) {state.hal_state = State::HAL_INITIALIZING;}
  if ((preflight_done_ && !preflight_result_) || measured_collision_ || monitor_.measurement_rejected() || monitor_.receiver_feedback_rejected() || monitor_.receiver_rejected() || !scene_.ready()) {state.hal_state = State::HAL_FAULT;}
  state_pub_->publish(state);
}
}  // namespace pac_hal
