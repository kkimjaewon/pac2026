// Copyright 2026 Jaewon Kim
#pragma once
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include "intelion_interfaces/msg/hal_joint_command.hpp"

namespace pac_hal {
enum class ReceiverReject {None, Session, JointCount, Sequence, Clock, Ttl, Expired, Nonfinite, Range};
struct ReceiverCheck {ReceiverReject reason{ReceiverReject::None};int axis{-1};};
inline const char * receiver_reason(ReceiverReject reason) {
  switch(reason) {
    case ReceiverReject::None:return "none";
    case ReceiverReject::Session:return "session_mismatch";
    case ReceiverReject::JointCount:return "joint_count_mismatch";
    case ReceiverReject::Sequence:return "zero_sequence";
    case ReceiverReject::Clock:return "invalid_created_time";
    case ReceiverReject::Ttl:return "invalid_ttl";
    case ReceiverReject::Expired:return "expired_at_receiver";
    case ReceiverReject::Nonfinite:return "nonfinite_joint";
    case ReceiverReject::Range:return "joint_out_of_range";
  }
  return "unknown";
}
// Reads the receiver's own diagnostic JSON. Only an expired packet with a
// complete identity is reported; anything else is left to the status flag.
inline bool parse_receiver_expiry(const std::string & payload,uint64_t & session,uint64_t & seq) {
  if(payload.find("\"reason\":\"expired_at_receiver\"")==std::string::npos) {return false;}
  auto number=[&payload](const char * key,uint64_t & out) {
    const auto at=payload.find(key);
    if(at==std::string::npos) {return false;}
    const char * begin=payload.c_str()+at+std::char_traits<char>::length(key);
    if(*begin<'0' || *begin>'9') {return false;}
    char * end=nullptr;out=std::strtoull(begin,&end,10);
    return end!=begin && (*end==',' || *end=='}');
  };
  return number("\"packet_session_id\":",session) && number("\"command_seq\":",seq) && seq>0;
}
// Same validation drives both rejection and diagnostics; reporting cannot drift
// from the checks actually used by the receiver. Called outside the RT update.
inline ReceiverCheck check_receiver_packet(const intelion_interfaces::msg::HalJointCommand & msg,
  uint64_t session, size_t count, int64_t now, int64_t ttl,
  const std::vector<double> & lower, const std::vector<double> & upper) {
  if(msg.session_id!=session) {return {ReceiverReject::Session};}
  if(msg.joint_count!=count || count==0 || count>msg.positions.size() ||
    lower.size()!=count || upper.size()!=count) {return {ReceiverReject::JointCount};}
  if(msg.command_seq==0) {return {ReceiverReject::Sequence};}
  if(msg.created_mono_ns<0 || msg.created_mono_ns>now) {return {ReceiverReject::Clock};}
  if(msg.expires_mono_ns<=msg.created_mono_ns ||
    msg.expires_mono_ns-msg.created_mono_ns!=ttl) {return {ReceiverReject::Ttl};}
  if(now>=msg.expires_mono_ns) {return {ReceiverReject::Expired};}
  for(size_t i=0;i<count;++i) {
    if(!std::isfinite(msg.positions[i])) {return {ReceiverReject::Nonfinite,static_cast<int>(i)};}
    if(msg.positions[i]<lower[i] || msg.positions[i]>upper[i]) {
      return {ReceiverReject::Range,static_cast<int>(i)};
    }
  }
  return {};
}
}
