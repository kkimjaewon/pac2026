// Copyright 2026 Jaewon Kim
#pragma once
#include <optional>
#include "pac_hal/hal_converter.hpp"
#include "pac_hal/scene_guard.hpp"

namespace pac_hal {
// A verified execution binds one model. A changed description requires a fresh
// launch/preflight, even if the original description is subsequently restored.
class ModelDescriptionGuard {
public:
  bool observe(const std::string & xml) {
    if (bound_ && xml != description_) {changed_ = true;}
    return !changed_;
  }
  void bind(const std::string & xml) {if (!bound_) {description_=xml;bound_=true;}}
  bool bound() const {return bound_;}
  bool changed() const {return changed_;}
private:
  std::string description_;
  bool bound_{false}, changed_{false};
};
// Unknown semantics are rejected, never converted to a position command.
Result check_task(const Config & config);
std::string describe_task_contract(const Config & config);
std::string describe_robot_contract(const Config & config);
std::string json_string(const std::string & value);
// Consume each own-session attempt before semantic checks. A rejected packet
// cannot be corrected and revived under the same identity; B must issue a new seq.
inline bool is_new_active_attempt(const Command & command,uint64_t active_session,
  const std::optional<uint64_t> & observed) {
  return command.session_id==active_session && active_session>0 && command.command_seq>0 &&
    (!observed || command.command_seq>*observed);
}
inline Result observe_command_sequence(const Command & command, uint64_t active_session,
  std::optional<uint64_t> & observed) {
  if(command.session_id==0 || command.session_id!=active_session) {
    return {Error::E_FORMAT,"B and C session_id must match"};
  }
  if(command.command_seq==0 || (observed && command.command_seq<=*observed)) {
    return {Error::E_FORMAT,"zero, duplicate or out-of-order command_seq"};
  }
  observed=command.command_seq;
  return {};
}
// HalState is a resumable progress watermark, not the identity of every rejected
// packet. The exact rejected identity remains in HalError/hal.csv.
inline void record_command_progress(const Command & command, uint64_t active_session,
  uint8_t outcome, uint64_t & sequence, uint8_t & result) {
  if(command.session_id==active_session && command.command_seq>=sequence) {
    sequence=command.command_seq;result=outcome;
  }
}
// Device-health errors describe the active session, never the most recent
// foreign/replayed packet. Command rejection events still keep that packet ID.
inline void record_active_command_context(const Command & command,uint64_t active_session,
  const std::string & frame,Command & context,bool & seen) {
  if(command.session_id==active_session && command.command_seq>0 &&
    (!seen || command.command_seq>context.command_seq)) {
    context=command;context.header.frame_id=frame;seen=true;
  }
}
Result preflight(const Config & config, HalConverter & converter, SceneGuard & scene,
  const Measurement & measured);

// One pending target, no historical queue, no new timestamps and no catch-up burst.
class LatestCommandRate {
public:
  explicit LatestCommandRate(int64_t period) : period_(period) {}
  struct Entry {Command command; TraceRecord trace;};
  std::optional<Entry> offer(const Command & command, const TraceRecord & trace) {
    auto replaced = std::move(pending_); pending_ = Entry{command, trace}; return replaced;
  }
  std::optional<Entry> take(int64_t now) {
    if (!pending_ || now < next_) {return std::nullopt;}
    auto entry = std::move(pending_); pending_.reset(); next_ = now + period_; return entry;
  }
  // Include computation/transport time in the minimum device send spacing.
  void completed(int64_t now) {next_ = now + period_;}
  std::optional<Entry> cancel() {auto entry = std::move(pending_);pending_.reset();return entry;}
private:
  int64_t period_, next_{0};
  std::optional<Entry> pending_;
};
}
