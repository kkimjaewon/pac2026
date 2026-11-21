// Copyright 2026 Jaewon Kim

#include "pac_hal/hal_latency_trace.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <stdexcept>

namespace pac_hal
{
namespace {
void bounded_detail(const std::string & detail,std::array<char,512> & out,bool & truncated) {
  size_t count=std::min(detail.find('\0'),std::min(detail.size(),out.size()-1));truncated=count<detail.size();
  // Do not truncate in the middle of a UTF-8 codepoint.
  while(count<detail.size() && count>0 && (static_cast<unsigned char>(detail[count])&0xC0)==0x80) {--count;}
  std::memcpy(out.data(),detail.data(),count);out[count]='\0';
}
void csv_text(std::ofstream & file,const char * text) {
  file<<'"';
  for(const char * p=text;*p;++p) {if(*p=='"') {file<<'"';}file<<*p;}
  file<<'"';
}
}
HalLatencyTrace::HalLatencyTrace(const Config & config) : session_(config.session)
{
  if (!config.logging) {return;}
  std::string directory = config.log_directory;
  if (directory.rfind("~/", 0) == 0) {
    const auto * home = std::getenv("HOME");
    if (!home) {throw std::runtime_error("HOME missing for log directory");}
    directory = std::string(home) + directory.substr(1);
  }
  const auto folder = std::filesystem::path(directory) / config.run_id;
  std::filesystem::create_directories(folder);
  const auto path = folder / "hal.csv";
  // O_EXCL로 실험 혼합을 막는다. 같은 run_id를 재사용하면 덮어쓰지 않고 시작을 실패시킨다.
  const auto fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
  if (fd < 0) {throw std::runtime_error("cannot create fresh trace: " + path.string());}
  ::close(fd);
  file_.open(path);
  if (!file_) {throw std::runtime_error("cannot open trace: " + path.string());}
  file_ << "session_id,command_seq,t1_mono_ns,t2_mono_ns,t3_mono_ns,t4_mono_ns,"
        << "t5_mono_ns,t6_mono_ns,result,error_code,ik_wall_ns,ik_cpu_ns,ik_budget_clock,"
        << "source_result_id,b_created_mono_ns,received_mono_ns,validated_mono_ns,"
        << "ik_end_mono_ns,transport_start_mono_ns,transport_end_mono_ns,"
        << "receiver_applied_mono_ns,ack_received_mono_ns\n";
  feedback_.open(folder / "feedback_gaps.csv");
  if (!feedback_) {throw std::runtime_error("cannot create feedback trace");}
  feedback_ << "source,callback_mono_ns,sample_ns,callback_gap_ns,sample_gap_ns\n";
  errors_.open(folder/"hal_errors.csv");diagnostics_.open(folder/"receiver_diagnostics.csv");
  if(!errors_ || !diagnostics_) {throw std::runtime_error("cannot create diagnostic trace");}
  errors_<<"session_id,command_seq,event_mono_ns,error_code,active,source,detail,detail_truncated\n";
  diagnostics_<<"c_received_mono_ns,payload,payload_truncated\n";
  stats_path_ = (folder / "trace_stats.yaml").string();
  worker_ = std::thread([this]() {drain();});
}

HalLatencyTrace::~HalLatencyTrace()
{
  stopping_.store(true, std::memory_order_release);
  if (worker_.joinable()) {worker_.join();}
  if (!stats_path_.empty()) {
    std::ofstream stats(stats_path_);
    stats << "enqueued: " << enqueued_ << "\ndropped: " << dropped_.load()
          << "\nsent_records: " << sent_ << "\napplied_records: " << applied_
          << "\nrejected_records: " << rejected_ << "\nsuperseded_records: " << superseded_
          << "\npending_evicted_without_ack: " << pending_evicted_
          << "\nolder_sent_without_individual_ack: " << older_without_ack_
          << "\nshutdown_pending_without_ack: " << pending_.size()
          << "\nunmatched_ack_events: " << unmatched_ack_
          << "\ninvalid_ack_time_events: " << invalid_ack_time_
          << "\nerror_transition_records: " << error_records_
          << "\nreceiver_diagnostic_records: " << diagnostic_records_
          << "\ntruncated_diagnostic_records: " << truncated_records_ << '\n';
  }
}
void HalLatencyTrace::enqueue(const Event & event)
{
  if (!worker_.joinable()) {return;}
  const auto head = head_.load(std::memory_order_relaxed);
  if (head - tail_.load(std::memory_order_acquire) >= kCapacity) {
    dropped_.fetch_add(1, std::memory_order_relaxed); return;
  }
  queue_[head % kCapacity] = event;
  ++enqueued_;
  head_.store(head + 1, std::memory_order_release);
}
void HalLatencyTrace::drain()
{
  auto last_flush=std::chrono::steady_clock::now();
  auto flush_due=[&]() {
    const auto now=std::chrono::steady_clock::now();
    if(now-last_flush>=std::chrono::seconds(1)) {
      file_.flush();feedback_.flush();errors_.flush();diagnostics_.flush();last_flush=now;
    }
  };
  for (;;) {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      if (stopping_.load(std::memory_order_acquire)) {break;}
      flush_due();
      std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue;
    }
    // Copy before releasing a slot; producer may immediately reuse it.
    const auto event = queue_[tail % kCapacity];
    tail_.store(tail + 1, std::memory_order_release);
    consume(event);
    flush_due();
  }
  file_.flush(); feedback_.flush();errors_.flush();diagnostics_.flush();
}
void HalLatencyTrace::feedback_gap(const char * source, int64_t callback, int64_t sample,
  int64_t callback_gap, int64_t sample_gap)
{
  if (callback_gap <= 20000000 && sample_gap <= 20000000) {return;}
  Event event;
  event.kind = std::strcmp(source, "joint_sim_time") == 0 ? Event::JointGap : Event::ReceiverGap;
  event.received = callback; event.sample = sample;
  event.callback_gap = callback_gap; event.sample_gap = sample_gap;
  enqueue(event);
}

void HalLatencyTrace::write(const TraceRecord & r, const char * result)
{
  if (!file_.is_open()) {return;}
  auto time = [this](int64_t ns) {if (ns > 0) {file_ << ns;} file_ << ',';};
  file_ << r.session << ',' << r.seq << ',';
  time(r.t1);
  time(r.t2);
  time(r.t3);
  time(r.t4);
  file_ << ',';  // t5: 측정을 특정 명령과 연결할 계약이 없으므로 비워 둔다.
  time(r.t6);
  file_ << result << ',' << r.error << ',';
  time(r.ik_wall_ns); time(r.ik_cpu_ns);
  if (r.ik_wall_ns) {file_ << ik_budget_name(r.ik_budget);}
  file_ << ',' << r.source_result_id << ',';
  time(r.created); time(r.t1); time(r.validated); time(r.ik_end);
  time(r.transport_start); time(r.transport_end); time(r.receiver_applied);
  if (r.t6 > 0) {file_ << r.t6;}
  file_ << '\n';
  // 매 명령 flush는 파일 I/O를 늘린다. OS buffer를 사용하고 종료 시 ofstream이 정리한다.
}

void HalLatencyTrace::command(const TraceRecord & record)
{
  Event event; event.record = record; enqueue(event);
}
void HalLatencyTrace::applied(const Receiver & receiver, int64_t received_ns)
{
  if (receiver.session_id != session_ || receiver.applied_mono_ns <= 0) {return;}
  Event event; event.kind = Event::Ack; event.session = receiver.session_id;
  event.seq = receiver.last_applied_command_seq; event.received = received_ns;
  event.sample = receiver.applied_mono_ns;
  enqueue(event);
}
void HalLatencyTrace::error(const Error & msg) {
  Event event;event.kind=Event::ErrorTransition;event.session=msg.session_id;
  event.seq=msg.command_seq;event.sample=msg.event_mono_ns;
  event.error_code=msg.code;event.active=msg.active;
  bounded_detail(msg.detail,event.detail,event.truncated);enqueue(event);
}
void HalLatencyTrace::receiver_diagnostic(const std::string & payload,int64_t received_ns) {
  Event event;event.kind=Event::ReceiverDiagnostic;event.received=received_ns;
  bounded_detail(payload,event.detail,event.truncated);enqueue(event);
}
void HalLatencyTrace::consume(const Event & event)
{
  if(event.kind==Event::ErrorTransition) {
    errors_<<event.session<<','<<event.seq<<','<<event.sample<<','<<event.error_code<<','
           <<event.active<<",hal_gateway,";csv_text(errors_,event.detail.data());
    errors_<<','<<event.truncated<<'\n';++error_records_;truncated_records_+=event.truncated;return;
  }
  if(event.kind==Event::ReceiverDiagnostic) {
    diagnostics_<<event.received<<',';csv_text(diagnostics_,event.detail.data());
    diagnostics_<<','<<event.truncated<<'\n';++diagnostic_records_;truncated_records_+=event.truncated;return;
  }
  if (event.kind == Event::CommandRecord) {
    const auto & record = event.record;
    write(record, record.superseded ? "superseded" : (record.sent ? "sent" : "rejected"));
    if(record.superseded) {++superseded_;}
    else if(!record.sent) {++rejected_;}
    if (record.sent) {
      ++sent_;
      pending_[record.seq] = record;
      while (pending_.size() > 256) {pending_.erase(pending_.begin());++pending_evicted_;}
    }
    return;
  }
  if (event.kind == Event::JointGap || event.kind == Event::ReceiverGap) {
    feedback_ << (event.kind == Event::JointGap ? "joint_sim_time" : "receiver_mono_time")
              << ',' << event.received << ',' << event.sample << ',' << event.callback_gap
              << ',' << event.sample_gap << '\n';
    return;
  }
  if (event.session != session_ || (ack_seen_ && event.seq <= last_ack_)) {return;}
  const auto found = pending_.find(event.seq);
  if (found == pending_.end()) {++unmatched_ack_;return;}
  // Target adoption can precede publish() returning, but cannot precede command
  // creation or transport start. Never admit negative/impossible latency rows.
  if(event.sample<found->second.created || event.sample<found->second.transport_start ||
    event.received<event.sample) {++invalid_ack_time_;return;}
  auto record = found->second; record.t6 = event.received;
  record.receiver_applied = event.sample;
  write(record, "applied"); last_ack_ = event.seq; ack_seen_ = true;
  ++applied_;
  // ACK is target adoption, not measured TCP arrival.
  older_without_ack_+=std::distance(pending_.begin(),found);
  pending_.erase(pending_.begin(), std::next(found));
}
}  // namespace pac_hal
