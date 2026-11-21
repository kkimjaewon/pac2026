// B 계층 ROS 2 노드
//
// 스레드 구성
//   - 통신 스레드 : rclcpp executor. /ai/control_command, /hal/state, /hal/error 콜백이 버퍼에 씀
//   - 제어 스레드 : rt_control_loop (1ms). 매 주기 A·C 버퍼를 읽고, RUN이면 HalCommand 버퍼에 씀
//   - 발행 스레드 : ros_bridge (자기만의 1ms 타이머). HalCommand 버퍼를 읽어 /hal/final_command 발행
//   - 메인 스레드 : 10ms마다 루프 이벤트(모드 변경·C 오류)를 바로 출력, 1초마다 요약 로그 (일반 스레드)
//   - 기록 스레드 : 수신 기록 CSV 저장 (AiCommandReceiver 안)
//
// ROS 파라미터 (실시간 옵션은 기본 모두 꺼짐)
//   fifo (bool, false)  SCHED_FIFO 켜기
//   prio (int, 80)      SCHED_FIFO 우선순위
//   cpu  (int, -1)      제어 스레드를 고정할 CPU (-1 = 고정 안 함, B 전용 코어는 6)
//   mlock (bool, false) mlockall 켜기
//
// 실행 기록 (계약 v0.3 인터페이스 5-2): ~/pac_logs/<run_id>/ 에 rt.csv, ai_rx.csv, meta.yaml
//   log_root     (string, "~/pac_logs")
//   run_id       (string, "")    비우면 run_id = 시작 시각 YYYYMMDD_HHMMSS (+ _run_tag).
//                                값이 있으면 <log_root>/<run_id>/ 를 그대로 씀 (폴더가 있어도 됨, rt.csv가 이미 있으면 오류로 멈춤)
//   run_tag      (string, "")    run_id가 비었을 때만 시각 뒤에 _태그로 붙음
//   trace_enable (bool, true)    false면 rt.csv를 쓰지 않음 (ai_rx.csv·meta.yaml은 씀)
//
// gate 파라미터 (기본값은 include/rt_control/gate.hpp의 GateConfig)
//   expected_layout_version (int, 1)
//   expected_command_type   (int, 0 = position)
//   ttl_ns                  (int, 200000000)
//   confidence_min          (double, 0.5)
//   planned_position        (double[3], [0.50, 0.00, 0.20])  map 기준 [m]
//   accept_radius           (double, 0.05)  [m]
//
// blend 파라미터 (ttl_ns는 gate와 같은 값, 시작 목표는 planned_position)
//   blend_mode              (string, "single")  지금은 single만 있음
//   target_offset           (double[3], [0, 0, 0])  [m] map 기준. gate 통과한 AI 위치에 더해 blend 목표로 씀
//                                     (철판 표면 -> TCP 작업 높이). gate·시작 위치·HOLD 목표에는 안 더함. NaN·무한대는 0
//   blend_deadband          (double, 0.0005)  [m] 반영하면 목표가 움직일 거리가 이보다 작으면 목표를 바꾸지 않음 (0 = 끔)
//
// motion_limiter 파라미터 (기본값은 include/rt_control/motion_limiter.hpp의 LimiterConfig)
//   v_max                   (double, 0.10)  [m/s]
//   a_max                   (double, 0.50)  [m/s²]
//   j_max                   (double, 1.0)   [m/s³] 저크 한계 (가속도가 1ms에 j_max×1ms 이하로만 바뀜, S자 출발·정지). 0 = 끔
//   (목표에 속도 0으로 도착하면 목표가 바뀔 때까지 출력 위치를 비트 단위로 그대로 유지)
//
// 도착 판정 (C와 합의, 기본값은 include/rt_control/arrival.hpp)
//   arrive_tol              (double, 0.002) [m] |blend 목표 - 측정 TCP|가 이 안에
//   arrive_hold_s           (double, 0.3)   [s] 이만큼 이상 머물면 "도착" (rt.csv arrived, tcp_error_m)
//
// 재개 기준 (DECEL_STOP·FAULT -> RUN으로 다시 움직일 때 limiter 시작 위치)
//   DECEL_STOP에서 재개: 측정 TCP가 있으면 limiter와 blend 목표를 둘 다 측정 TCP로 맞춤 (그 자리 유지 명령부터)
//   FAULT에서 재개 (또는 DECEL_STOP인데 측정 TCP 없음):
//   resume_from_last_cmd    (bool, true)    마지막 발행 HalCommand 위치와 측정 TCP 차이가 resume_match_tol 안이면
//                                           마지막 발행 위치에서 시작. 아니면(또는 false) 측정 TCP에서 시작 + 경고
//   resume_match_tol        (double, 0.002) [m]
//   rejected_goal_tol       (double, 0.01)  [m] C가 목표를 거부(오류 4·5)해 DECEL_STOP이 나면 마지막으로 반영한
//                                           AI 목표를 기억하고, 새 AI 목표가 이만큼 이상 달라질 때까지 반영 안 함
//
// 시작 (INIT -> RUN). INIT에서는 AI 명령을 쓰지 않고 safety 판단(STATE_TIMEOUT 등)도 하지 않는다
//   신선한 HalState를 받으면 그 측정 TCP 위치·자세를 "유지 명령"으로 1ms마다 발행 (include/rt_control/init_hold.hpp).
//   신선한 HalState 전에는 아무것도 발행하지 않고, 끊기면 발행을 멈추고 다시 기다림.
//   시작 준비(warmup)가 끝나면 마지막 유지 명령 위치에서 RUN 시작 (세션·command_seq 이어짐).
//   AI가 없을 때 blend 목표도 그 위치. warmup 조건은 include/rt_control/warmup.hpp
//   warmup_states           (int, 5)       새 HalState를 이만큼 받아야 함 (0 = 조건 끔)
//   warmup_stable_ms        (double, 100)  발행 스레드가 이만큼 overrun 없이 깨어나야 함
//   warmup_max_s            (double, 3.0)  이보다 오래 준비가 안 되면 경고 (시작은 계속 기다림)
//   warmup_hold_ms          (double, 100)  유지 명령을 이만큼 끊김·overrun 없이 발행해야 함 (0 = 조건 끔)
//   + /hal/final_command 구독자 1개 이상
//   session_id              (int, 0)  0 = 첫 신선한 HalState 때 B가 발급(그 순간 CLOCK_MONOTONIC ns), FAULT 해제 시 새 세션.
//                                     0이 아니면 C에 등록된 그 번호를 씀. FAULT 해제 시 세션·순번 유지,
//                                     C가 같은 세션의 마지막 순번을 보고하면 그 다음부터 발행 (B 재시작 대비)
//   require_registered_state (bool, true)  session_id가 0이 아닐 때만: C가 그 세션의 신선한 READY/ACTIVE 상태를
//                                     보고해야 시작
//   measured_orientation    (bool, true)  HalState에서 시작하면 그때 측정한 TCP 자세를 고정 자세로 씀
//   start_fallback_on_timeout (bool, false)  true면 start_timeout_s 안에 시작 못 할 때 start_position에서 시작
//                                     (예전 기본 동작). false면 INIT에서 계속 기다림 (임의 위치로 시작 안 함)
//   start_timeout_s         (double, 2.0)  [s]
//   start_position          (double[3], planned_position)  fallback 시작 위치
//
// B -> C 명령 (기본값은 include/rt_control/hal_command_snapshot.hpp의 HalCommandConfig)
//   hal_ttl_ns              (int, 5000000)  expires = created + hal_ttl_ns
//   tool_orientation        (double[4], [1, 0, 0, 0])  고정 자세 쿼터니언 (x, y, z, w)
//   bridge_cpu              (int, -1)  발행 스레드를 고정할 CPU (-1 = 고정 안 함)
//   bridge_prio             (int, 70)  fifo:=true일 때 발행 스레드의 SCHED_FIFO 우선순위 (prio보다 낮게)
//
// safety_mode 파라미터 (기본값은 include/rt_control/safety_mode.hpp의 SafetyConfig)
//   range_reject_n          (int, 5)      새 명령이 RANGE로 연속 이만큼 막히면 HOLD
//   resume_k                (int, 3)      HOLD에서 새 명령이 연속 이만큼 통과하면 RUN
//   ai_decel_after_s        (double, 0.0) HOLD가 이보다 오래 가면 DECEL_STOP (0 = 끔)
//   require_c_state         (bool, true)  false면 C 관련 규칙을 모두 끔 (C 없이 개발할 때)
//   c_state_timeout_ms      (double, 20)  C 상태 나이가 이보다 크면 DECEL_STOP
//   c_expired_n             (int, 10)     E_EXPIRED가 최근 1초에 이만큼 이상이면 HOLD
//   resume_delay_s          (double, 1.0) DECEL_STOP 원인이 풀린 뒤 이만큼 지나고 멈춰 있으면 RUN
// FAULT 해제: ros2 service call /rt/reset_fault std_srvs/srv/Trigger
//
// cpu를 지정하면 1ms 루프 말고 다른 B 스레드(DDS·executor·발행·기록)는 그 코어와 짝꿍 코어를 피한다
//
// 실행 예: ros2 run rt_control rt_control_node --ros-args -p fifo:=true -p cpu:=6 -p mlock:=true
// 가짜 A(base_target [0.5, 0.0, 0.71])로 시험할 때:
//   ros2 run rt_control rt_control_node --ros-args -p "planned_position:=[0.5, 0.0, 0.71]"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <sys/utsname.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rt_control/ai_command_receiver.hpp"
#include "rt_control/arrival.hpp"
#include "rt_control/blend_policy.hpp"
#include "rt_control/cycle_tracer.hpp"
#include "rt_control/env_check.hpp"
#include "rt_control/gate.hpp"
#include "rt_control/hal_command_snapshot.hpp"
#include "rt_control/hal_state_snapshot.hpp"
#include "rt_control/init_hold.hpp"
#include "rt_control/motion_limiter.hpp"
#include "rt_control/ros_bridge.hpp"
#include "rt_control/run_dir.hpp"
#include "rt_control/rt_control_loop.hpp"
#include "rt_control/safety_mode.hpp"
#include "rt_control/spsc_ring.hpp"
#include "rt_control/warmup.hpp"

namespace {

using rt_control::AiCommandSnapshot;
using rt_control::HalCommandSnapshot;
using rt_control::HalStateSnapshot;

// 1ms 루프의 운전 상태
enum class RunMode : uint8_t { kInit = 0, kRun = 1 };
// RUN을 어디서 시작했는지 (요약 로그 스레드가 한 번 알려 줌)
enum class StartSource : uint8_t { kNone = 0, kHalState = 1, kTimeout = 2 };

// 1ms 루프 -> 메인 스레드 이벤트 (루프는 링버퍼에 넣기만 하고, 메인이 꺼내 출력)
struct LoopEvent {
  enum Kind : uint8_t { kModeChange, kCError, kResetResult, kLimiterReset, kRejectedGoal, kInitHold };
  Kind kind;
  uint8_t from, to;      // kModeChange: SafetyMode
  uint8_t value;         // kResetResult: ResetResult, kLimiterReset: ResumeSource, kInitHold: InitHoldEvent
  uint16_t code;         // kCError
  bool active;           // kCError
  bool ignored;          // kCError: 다른 세션이라 무시함
  bool goal_reset;       // kLimiterReset: blend 목표도 같이 맞춤
  uint64_t session;      // kCError: 이벤트의 session_id
  uint32_t reasons;      // kModeChange: 들어온 이유
  uint64_t error_mask;   // kModeChange: 켜진 오류
  uint64_t number;       // kCError: command_seq, kResetResult·kInitHold: session_id
  double pos[3];         // kLimiterReset, kRejectedGoal, kInitHold
  double diff;           // kLimiterReset: |마지막 발행 위치 - 측정 TCP| [m] (비교 안 했으면 NaN)
};
constexpr size_t kEventRingSize = 256;

// 제어 루프 <-> 바깥(요약 로그) 사이에서 주고받는 값.
// 루프 콜백 안에서는 이 구조체만 만지고, printf·할당은 하지 않는다.
struct LoopContext {
  const rt_control::Seqlock<AiCommandSnapshot>* ai_buffer = nullptr;
  rt_control::GateConfig gate_config;  // 루프 시작 전에 정해지고 이후 바뀌지 않음
  rt_control::BlendConfig blend_config;  // RUN 시작 때 initial_goal만 측정 위치로 바꿔 blend를 다시 만듦
  rt_control::BlendPolicy blend;       // 루프 스레드 전용 (지금 목표를 들고 있음)
  rt_control::MotionLimiter limiter;   // 루프 스레드 전용 (출력 위치·속도를 들고 있음)
  rt_control::ArrivalDetector arrival;  // 루프 스레드 전용 (도착 판정 타이머)
  // 재개 기준: 마지막으로 버퍼에 쓴 HalCommand 위치. new_session이 hal_cmd를 지워도 남도록 따로 둠
  bool resume_from_last_cmd = true;
  double resume_match_tol = rt_control::kDefaultResumeMatchTolM;
  double last_sent_pos[3] = {0, 0, 0};
  bool has_last_sent = false;
  // limiter·속도 측정에 쓰는 간격 [s]. 실제 깨어난 간격(CLOCK_MONOTONIC). 첫 주기만 정해진 주기.
  // 고정 1ms로 계산하면 깨어나는 간격이 흔들릴 때 C가 보는 가속도가 튐 (C가 관절 가속도 초과로 거부)
  double dt = 0.001;
  int64_t prev_wakeup_ns = 0;

  // safety_mode (루프 스레드 전용)
  rt_control::SafetyStateMachine safety;
  rt_control::RosBridge* bridge = nullptr;  // /hal/error 이벤트 링버퍼의 소비자는 이 루프
  double last_speed = 0.0;                  // 지난 주기 limiter 속도
  uint64_t state_error_mask = 0;            // HalState.active_errors 비트마스크
  std::atomic<bool> reset_request{false};   // /rt/reset_fault 서비스가 켜고 루프가 꺼냄
  rt_control::SpscRing<LoopEvent, kEventRingSize> events;
  rt_control::CycleTracer* tracer = nullptr;  // 없으면 기록 안 함
  std::atomic<uint64_t> event_overflow{0};

  // B -> C 명령 (루프 스레드만 씀)
  rt_control::Seqlock<HalCommandSnapshot>* hal_buffer = nullptr;
  rt_control::HalCommandConfig hal_config;
  HalCommandSnapshot hal_cmd{};        // 지난 주기에 만든 명령 (command_seq를 이어 셈)
  uint64_t session_id = 0;             // 지금 세션 (0 = 아직 없음)
  uint64_t registered_session_id = 0;  // 파라미터 session_id. 0 = B가 발급, 아니면 C에 등록된 번호
  bool require_registered_state = true;
  bool measured_orientation = true;
  bool start_fallback_on_timeout = false;

  // C 상태 (루프 스레드 전용 복사본)
  const rt_control::Seqlock<HalStateSnapshot>* state_buffer = nullptr;
  HalStateSnapshot state{};
  bool has_state = false;

  // 거부된 목표 (루프 스레드 전용): 이번 DECEL_STOP에서 이미 기억했는지
  bool reject_recorded = false;

  // 시작 준비 (루프 스레드 전용, cmd_subscribers만 메인 스레드가 씀)
  rt_control::StartupWarmup warmup;
  rt_control::InitHold init_hold;      // INIT 유지 명령
  int64_t warm_last_state_ns = 0;      // 마지막으로 센 HalState의 recv_mono_ns
  std::atomic<uint64_t> cmd_subscribers{0};

  // INIT -> RUN (루프 스레드 전용)
  RunMode mode = RunMode::kInit;
  int64_t init_start_ns = 0;           // 첫 주기 시각. 여기서 start_timeout_ns가 지나면 start_position으로 시작
  int64_t start_timeout_ns = 2'000'000'000;
  double start_position[3] = {0, 0, 0};

  // 루프 스레드 전용: 출력 위치 차분으로 잰 속도·가속도 (최고값 기록용)
  double meas_prev_pos[3] = {0, 0, 0};
  double meas_prev_vel[3] = {0, 0, 0};

  // 루프 스레드 전용: 마지막으로 읽는 데 성공한 명령 (읽기 실패 시 이 값을 계속 씀)
  AiCommandSnapshot last{};
  bool has_cmd = false;

  // 루프 밖에서 읽는 값 (요약 로그용). 각각 따로 갱신되므로 서로 1주기 정도 어긋날 수 있음
  std::atomic<bool> out_has_cmd{false};
  std::atomic<uint64_t> out_read_ai_seq{0};  // 버퍼에서 마지막으로 읽은 seq
  std::atomic<uint32_t> out_used_boot_id{0};
  std::atomic<uint8_t> out_status{0};
  std::atomic<int64_t> out_msg_age_ns{0};  // A가 보낼 때 이미 지난 나이 (메시지의 age_ns)
  std::atomic<int64_t> out_age_ns{0};      // 지금 시각 기준 나이 = age_ns + (now - recv_mono_ns)
  // gate 결과 (이번 주기)
  std::atomic<bool> out_gate_pass{false};
  std::atomic<uint32_t> out_gate_reasons{0};
  // gate 누적 횟수 (루프 스레드만 씀)
  std::atomic<uint64_t> out_gate_pass_count{0};
  std::array<std::atomic<uint64_t>, rt_control::kGateReasonCount> out_gate_block_count{};
  // blend 결과. goal x,y,z는 따로 저장되므로 서로 1주기 어긋날 수 있음 (로그용으로는 충분)
  std::array<std::atomic<double>, 3> out_goal{};
  std::atomic<double> out_w{0.0};             // 마지막으로 반영했을 때의 w
  std::atomic<uint64_t> out_blend_used_seq{0};
  std::atomic<bool> out_blend_has_used{false};
  std::atomic<uint64_t> out_blend_apply_count{0};
  // limiter 결과
  std::array<std::atomic<double>, 3> out_pos{};
  std::array<std::atomic<double>, 3> out_vel{};
  std::atomic<double> out_speed{0.0};
  // 도착 판정
  std::atomic<bool> out_arrived{false};
  std::atomic<double> out_tcp_error{0.0};  // [m], 측정 없으면 NaN
  // 최고 속도·가속도 (출력 위치 차분으로 잼). 메인 스레드가 1초마다 exchange(0)로 비움.
  // 비우는 순간과 루프의 load→store가 겹치면 그 1주기 값이 다음 창으로 넘어갈 수 있음 (로그용이라 허용)
  std::atomic<double> out_peak_speed{0.0};
  std::atomic<double> out_peak_accel{0.0};
  std::atomic<uint64_t> out_read_failures{0};
  std::atomic<uint64_t> out_cycles{0};
  // 루프 상태
  std::atomic<uint8_t> out_mode{static_cast<uint8_t>(RunMode::kInit)};
  std::atomic<uint8_t> out_start_source{static_cast<uint8_t>(StartSource::kNone)};
  std::array<std::atomic<double>, 3> out_start_pos{};  // RUN을 시작한 위치 (out_start_source보다 먼저 씀)
  std::atomic<uint64_t> out_session_id{0};
  std::atomic<int64_t> out_peak_jitter_ns{0};  // 깨어난 시각 - 예정 시각의 최고값. 메인이 1초마다 비움
  // C 상태 (1ms 루프가 매 주기 저장만 함. 오류 대응은 나중에 safety_mode에서)
  std::atomic<bool> out_has_state{false};
  std::atomic<int64_t> out_state_age_ns{0};    // now - recv_mono_ns
  std::atomic<uint8_t> out_hal_state{0};
  std::atomic<uint64_t> out_state_seq{0};
  std::atomic<uint64_t> out_c_last_command_seq{0};
  std::atomic<uint8_t> out_c_last_command_result{0};
  std::atomic<uint64_t> out_c_session_id{0};
  std::atomic<uint64_t> out_state_error_mask{0};  // HalState.active_errors를 비트마스크로 (bit n = 코드 n)
  std::atomic<uint64_t> out_state_read_failures{0};
  // safety_mode
  std::atomic<uint8_t> out_safety_mode{0};
  std::atomic<uint32_t> out_safety_reasons{0};   // 지금 모드에 들어온 이유
  std::atomic<uint32_t> out_safety_active{0};    // 이번 주기에 켜진 조건
  std::atomic<int64_t> out_mode_duration_ns{0};
  std::atomic<uint64_t> out_c_error_mask{0};     // safety가 쓴 켜진 오류 (HalState + 이벤트)
  std::atomic<uint64_t> out_ignored_error_events{0};  // 다른 세션이라 무시한 /hal/error 이벤트 수
  // 거부된 목표 대기
  std::atomic<bool> out_rejected_wait{false};
  std::array<std::atomic<double>, 3> out_rejected_target{};
  std::atomic<uint64_t> out_rejected_skip_count{0};  // 거부된 목표와 가까워 반영 안 한 새 명령 수
  // 시작 준비
  std::atomic<uint32_t> out_warmup_met{0};
  std::atomic<int> out_warmup_states{0};
  std::atomic<int64_t> out_warmup_ns{0};  // INIT에 머문 시간 (RUN으로 가면 멈춤)
  std::atomic<bool> out_init_holding{false};
  std::atomic<uint64_t> out_init_hold_published{0};  // INIT에서 버퍼에 쓴 유지 명령 수
};

void push_event(LoopContext& c, const LoopEvent& e) {
  if (!c.events.push(e)) c.event_overflow.fetch_add(1, std::memory_order_relaxed);
}

// limiter를 pos에서 다시 시작 (속도 0). 위치 차분 측정값도 같이 맞춤
void reset_limiter_at(LoopContext& c, const double pos[3]) {
  c.limiter.reset(pos);
  c.last_speed = 0.0;
  for (int i = 0; i < 3; ++i) {
    c.meas_prev_pos[i] = pos[i];
    c.meas_prev_vel[i] = 0.0;
    c.out_pos[i].store(pos[i], std::memory_order_relaxed);
  }
}

// 세션 시작 (INIT 유지 명령 없이 RUN을 시작할 때, FAULT 해제 때 부름. 보통은 InitHold가 정함)
// - session_id 0: 새 번호 발급 (그 순간 CLOCK_MONOTONIC ns). command_seq는 새 세션에서 1부터
// - session_id 고정: 처음 한 번만 정하고 FAULT 해제에도 세션·순번 유지 (새 실행은 B와 C를 같이 재시작).
//   C가 같은 세션의 마지막 순번을 보고하면 그 다음부터 (B 재시작 시 C가 옛 순번으로 거부하지 않게)
void new_session(LoopContext& c, int64_t now) {
  if (c.registered_session_id == 0) {
    c.session_id = static_cast<uint64_t>(now);
    c.hal_cmd = HalCommandSnapshot{};
  } else if (c.session_id == 0) {
    c.session_id = c.registered_session_id;
    c.hal_cmd = HalCommandSnapshot{};
    if (c.has_state && c.state.active_session_id == c.registered_session_id) {
      c.hal_cmd.command_seq = c.state.last_command_seq;
    }
  }
  c.safety.set_session(c.session_id);  // /hal/error는 이 세션 것만 씀
  c.out_session_id.store(c.session_id, std::memory_order_relaxed);
}

// 지금 믿을 수 있는 실제 로봇 위치가 있으면 true (pos에 넣음)
bool robot_position(const LoopContext& c, int64_t now, double pos[3]) {
  const bool ok = c.has_state && c.state.pose_valid && !c.state.stale &&
                  now - c.state.recv_mono_ns <= c.safety.config().c_state_timeout_ns &&
                  std::isfinite(c.state.tcp_pos[0]) && std::isfinite(c.state.tcp_pos[1]) &&
                  std::isfinite(c.state.tcp_pos[2]);
  if (ok) {
    for (int i = 0; i < 3; ++i) pos[i] = c.state.tcp_pos[i];
  }
  return ok;
}

// RUN으로 바꾼다: limiter를 start에서 시작. 세션이 아직 없으면 발급 (루프 스레드에서만 호출)
//   quat: HalState에서 시작할 때의 측정 자세 (measured_orientation이면 고정 자세로 씀). nullptr = 안 바꿈
void enter_run(LoopContext& c, const double start[3], const double* quat, StartSource source, int64_t now) {
  reset_limiter_at(c, start);
  // AI가 아직 없으면 blend 목표 = 시작 위치 (planned_position으로 끌려가는 불필요한 이동을 없앰)
  for (int i = 0; i < 3; ++i) c.blend_config.initial_goal[i] = start[i];
  c.blend = rt_control::BlendPolicy(c.blend_config);
  c.arrival.reset();
  for (int i = 0; i < 3; ++i) c.out_goal[i].store(start[i], std::memory_order_relaxed);
  if (c.measured_orientation && quat) {
    const double n = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    if (std::isfinite(n) && n > 1e-6) {
      for (int i = 0; i < 4; ++i) c.hal_config.tool_orientation[i] = quat[i] / n;
    }
  }
  for (int i = 0; i < 3; ++i) c.out_start_pos[i].store(start[i], std::memory_order_relaxed);
  // INIT 유지 명령에서 이미 정한 세션·command_seq를 그대로 이어 씀
  if (c.session_id == 0) new_session(c, now);
  c.safety.reset(now);
  c.mode = RunMode::kRun;
  c.out_mode.store(static_cast<uint8_t>(RunMode::kRun), std::memory_order_relaxed);
  c.out_start_source.store(static_cast<uint8_t>(source), std::memory_order_release);
}

// C 상태 버퍼를 읽어 나이와 최신 상태를 원자 변수에 저장만 한다
void read_hal_state(LoopContext& c, int64_t now) {
  if (c.state_buffer->ever_written()) {
    if (c.state_buffer->try_read(c.state)) {
      c.has_state = true;
    } else {
      c.out_state_read_failures.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (!c.has_state) return;
  uint64_t mask = 0;
  for (int i = 0; i < c.state.active_error_count; ++i) {
    const uint16_t code = c.state.active_errors[i];
    mask |= (uint64_t{code <= 63} << (code & 63));  // 63보다 큰 코드는 무시 (분기 없음)
  }
  c.out_state_age_ns.store(now - c.state.recv_mono_ns, std::memory_order_relaxed);
  c.out_hal_state.store(c.state.hal_state, std::memory_order_relaxed);
  c.out_state_seq.store(c.state.state_seq, std::memory_order_relaxed);
  c.out_c_last_command_seq.store(c.state.last_command_seq, std::memory_order_relaxed);
  c.out_c_last_command_result.store(c.state.last_command_result, std::memory_order_relaxed);
  c.out_c_session_id.store(c.state.active_session_id, std::memory_order_relaxed);
  c.state_error_mask = mask;
  c.out_state_error_mask.store(mask, std::memory_order_relaxed);
  c.out_has_state.store(true, std::memory_order_relaxed);
}

// INIT에서 매 주기: 시작 준비 조건을 갱신 (새 HalState 수, 발행 안정, 구독자)
void update_warmup(LoopContext& c, int64_t now) {
  rt_control::WarmupInput w{};
  w.now_ns = now;
  w.new_state = c.has_state && c.state.recv_mono_ns != c.warm_last_state_ns;
  if (c.has_state) c.warm_last_state_ns = c.state.recv_mono_ns;
  w.publish_ticks = c.bridge->publish_tick_count();
  w.publish_overruns = c.bridge->publish_overrun_count();
  w.subscribers = c.cmd_subscribers.load(std::memory_order_relaxed);
  w.holding = c.init_hold.holding();
  w.published = c.bridge->published_count();
  c.out_warmup_met.store(c.warmup.update(w), std::memory_order_relaxed);
  c.out_warmup_states.store(c.warmup.states_seen(), std::memory_order_relaxed);
  c.out_warmup_ns.store(c.warmup.elapsed_ns(now), std::memory_order_relaxed);
}

// INIT에서 매 주기: 시작할 위치가 정해지면 RUN으로 바꾼다
void try_start(LoopContext& c, int64_t now) {
  if (c.init_start_ns == 0) c.init_start_ns = now;
  update_warmup(c, now);
  double pos[3], quat[4];
  const bool pose_ok = rt_control::fresh_pose(c.has_state, c.state, now, c.safety.config().c_state_timeout_ns, pos, quat);
  // session_id를 고정했으면 C가 그 세션의 신선한 READY/ACTIVE 상태를 보고해야 시작
  const bool registered_ok =
      c.registered_session_id == 0 || !c.require_registered_state ||
      (c.has_state && c.state.active_session_id == c.registered_session_id &&
       (c.state.hal_state == rt_control::HAL_READY || c.state.hal_state == rt_control::HAL_ACTIVE) &&
       now >= c.state.recv_mono_ns && now - c.state.recv_mono_ns <= c.safety.config().c_state_timeout_ns);
  // 지금도 신선하고, 지난 주기까지 유지 명령을 발행했으면 그 마지막 유지 명령 위치·자세에서 시작
  if (pose_ok && registered_ok && c.init_hold.holding() && c.warmup.ready()) {
    enter_run(c, c.init_hold.hold_pos(), c.init_hold.hold_quat(), StartSource::kHalState, now);
  } else if (c.start_fallback_on_timeout && !c.init_hold.holding() && now - c.init_start_ns >= c.start_timeout_ns) {
    enter_run(c, c.start_position, nullptr, StartSource::kTimeout, now);
  }
}

// INIT에서 매 주기 (RUN으로 못 갔을 때): 신선한 HalState면 측정 TCP "유지 명령"을 버퍼에 씀.
// AI 명령은 읽지도 쓰지도 않음. 돌려주는 값 = 이번 주기에 썼는지
bool publish_init_hold(LoopContext& c, int64_t now) {
  const rt_control::InitHoldResult h = c.init_hold.step(c.has_state, c.state, now, c.session_id, c.hal_cmd, c.hal_config);
  if (h.session_issued) {
    c.safety.set_session(c.session_id);  // /hal/error는 이 세션 것만 씀
    c.out_session_id.store(c.session_id, std::memory_order_relaxed);
  }
  if (h.publish) {
    c.hal_buffer->write(c.hal_cmd);
    for (int i = 0; i < 3; ++i) c.last_sent_pos[i] = c.hal_cmd.pos[i];
    c.has_last_sent = true;
  }
  if (h.event != rt_control::InitHoldEvent::kNone) {
    LoopEvent e{};
    e.kind = LoopEvent::kInitHold;
    e.value = static_cast<uint8_t>(h.event);
    e.number = c.session_id;
    for (int i = 0; i < 3; ++i) e.pos[i] = c.init_hold.hold_pos()[i];
    push_event(c, e);
  }
  c.out_init_holding.store(c.init_hold.holding(), std::memory_order_relaxed);
  c.out_init_hold_published.store(c.init_hold.published(), std::memory_order_relaxed);
  return h.publish;
}

bool on_cycle(const rt_control::CycleInfo& info, void* user) {
  LoopContext& c = *static_cast<LoopContext*>(user);

  const int64_t jitter = info.wakeup_ns - info.scheduled_ns;
  c.out_peak_jitter_ns.store(std::max(c.out_peak_jitter_ns.load(std::memory_order_relaxed), jitter),
                             std::memory_order_relaxed);
  c.out_cycles.fetch_add(1, std::memory_order_relaxed);

  // 이번 주기 기록 한 줄 (끝에서 링버퍼에 넣기만 함)
  rt_control::TraceRow row{};
  row.cycle = info.cycle;
  row.wake_mono_ns = info.wakeup_ns;
  row.overrun = info.overrun;
  row.jitter_ns = jitter;

  read_hal_state(c, info.wakeup_ns);

  // /hal/error 이벤트: safety에 알려 주고(켜진 오류·E_EXPIRED 횟수), 출력용으로 메인에 넘김
  rt_control::HalErrorEvent ev;
  while (c.bridge->pop_error_event(ev)) {
    // 다른 세션의 이벤트는 모드 판단에 쓰지 않음 (개수만 셈)
    const bool used = c.safety.on_error_event(ev.code, ev.active, info.wakeup_ns, ev.session_id);
    LoopEvent e{};
    e.kind = LoopEvent::kCError;
    e.code = ev.code;
    e.active = ev.active;
    e.ignored = !used;
    e.session = ev.session_id;
    e.number = ev.command_seq;
    push_event(c, e);
  }
  c.out_ignored_error_events.store(c.safety.ignored_error_events(), std::memory_order_relaxed);

  if (c.mode == RunMode::kInit) {
    try_start(c, info.wakeup_ns);
    // INIT에서는 A 명령을 쓰지 않는다 (A 버퍼는 최신값이라 RUN 이후 첫 주기에 그때 최신 명령을 읽으면 됨).
    // HalCommand는 신선한 HalState가 있을 때 측정 TCP 유지 명령만 내보낸다
    if (c.mode == RunMode::kInit) {
      const bool held = publish_init_hold(c, info.wakeup_ns);
      if (c.tracer) {
        row.mode = rt_control::TRACE_INIT;
        row.tcp_error_m = std::numeric_limits<double>::quiet_NaN();
        row.command_seq = held ? c.hal_cmd.command_seq : 0;
        for (int i = 0; i < 3; ++i) {
          row.goal[i] = held ? c.hal_cmd.pos[i] : c.blend.goal()[i];
          row.y[i] = held ? c.hal_cmd.pos[i] : c.limiter.state().pos[i];
        }
        row.compute_ns = rt_control::now_ns() - info.wakeup_ns;
        c.tracer->push(row);
      }
      return true;
    }
  }

  // 쓴 적 없는 버퍼(전부 0)는 명령으로 치지 않음
  if (c.ai_buffer->ever_written()) {
    if (c.ai_buffer->try_read(c.last)) {  // 최대 3회, 실패하면 c.last는 그대로(이전 값)
      c.has_cmd = true;
    } else {
      c.out_read_failures.fetch_add(1, std::memory_order_relaxed);
    }
  }

  const rt_control::GateResult g = rt_control::gate_check(c.last, c.has_cmd, info.wakeup_ns, c.gate_config);
  c.out_gate_pass.store(g.pass, std::memory_order_relaxed);
  c.out_gate_reasons.store(g.reasons, std::memory_order_relaxed);
  c.out_gate_pass_count.fetch_add(g.pass, std::memory_order_relaxed);
  for (int i = 0; i < rt_control::kGateReasonCount; ++i) {  // 이유별로 0 또는 1을 더함 (분기 없음)
    c.out_gate_block_count[i].fetch_add((g.reasons >> i) & 1u, std::memory_order_relaxed);
  }

  // safety_mode: gate 결과·새 명령 여부·C 상태로 모드를 정함
  rt_control::SafetyInput si{};
  si.now_ns = info.wakeup_ns;
  si.gate_pass = g.pass;
  si.gate_reasons = g.reasons;
  si.is_new_command = c.blend.is_new(c.last, g);
  si.c_has_state = c.has_state;
  si.c_state_age_ns = c.has_state ? info.wakeup_ns - c.state.recv_mono_ns : 0;
  si.hal_state = c.state.hal_state;
  si.c_error_mask = c.state_error_mask;
  si.reset_request = c.reset_request.exchange(false, std::memory_order_relaxed);
  si.limiter_speed = c.last_speed;
  const rt_control::SafetyOutput s = c.safety.update(si);

  if (s.new_session) new_session(c, info.wakeup_ns);  // FAULT 해제
  // C가 목표를 거부해(오류 4·5) DECEL_STOP: 이번 정지에서 한 번, 마지막으로 반영한 AI 목표를 기억
  if (s.mode == rt_control::SafetyMode::kDecelStop && !c.reject_recorded &&
      (s.reasons & rt_control::SAFE_C_ERROR_DECEL) && (s.c_error_mask & rt_control::kTargetRejectErrorMask)) {
    c.reject_recorded = true;
    if (c.blend.reject_last_applied()) {
      LoopEvent e{};
      e.kind = LoopEvent::kRejectedGoal;
      for (int i = 0; i < 3; ++i) e.pos[i] = c.blend.rejected_target()[i];
      e.error_mask = s.c_error_mask;
      push_event(c, e);
    }
  }
  if (s.reset_limiter) {
    c.reject_recorded = false;
    double robot[3];
    const bool robot_ok = robot_position(c, info.wakeup_ns, robot);
    LoopEvent e{};
    e.kind = LoopEvent::kLimiterReset;
    if (s.prev_mode == rt_control::SafetyMode::kDecelStop && robot_ok) {
      // DECEL_STOP에서 재개: limiter와 blend 목표를 측정 TCP로 맞춰 그 자리 유지 명령부터 보냄
      // (멈추기 전 목표를 다시 쫓아가 C가 같은 목표를 또 거부하지 않게)
      reset_limiter_at(c, robot);
      c.blend.reset_goal(robot);
      c.arrival.reset();
      e.value = static_cast<uint8_t>(rt_control::ResumeSource::kRobot);
      e.diff = std::numeric_limits<double>::quiet_NaN();
      e.goal_reset = true;
      for (int i = 0; i < 3; ++i) e.pos[i] = robot[i];
    } else {
      // FAULT에서 재개(또는 측정 TCP 없음): 마지막 발행 위치가 실제 로봇 위치와 가까우면 거기서 이어서,
      // 아니면 실제 로봇 위치에서 출발 (없으면 지금 출력 위치). 기준은 include/rt_control/arrival.hpp
      const rt_control::ResumeChoice rc = rt_control::choose_resume_position(
          c.resume_from_last_cmd, c.resume_match_tol, c.has_last_sent, c.last_sent_pos, robot_ok, robot,
          c.limiter.state().pos);
      reset_limiter_at(c, rc.pos);
      e.value = static_cast<uint8_t>(rc.source);
      e.diff = rc.diff_m;
      for (int i = 0; i < 3; ++i) e.pos[i] = rc.pos[i];
    }
    push_event(c, e);
  }
  if (s.reset_result != rt_control::ResetResult::kNone) {
    LoopEvent e{};
    e.kind = LoopEvent::kResetResult;
    e.value = static_cast<uint8_t>(s.reset_result);
    e.number = c.session_id;
    push_event(c, e);
  }
  if (s.changed) {
    LoopEvent e{};
    e.kind = LoopEvent::kModeChange;
    e.from = static_cast<uint8_t>(s.prev_mode);
    e.to = static_cast<uint8_t>(s.mode);
    e.reasons = s.reasons;
    e.error_mask = s.c_error_mask;
    push_event(c, e);
  }
  c.out_safety_mode.store(static_cast<uint8_t>(s.mode), std::memory_order_relaxed);
  c.out_safety_reasons.store(s.reasons, std::memory_order_relaxed);
  c.out_safety_active.store(s.active_reasons, std::memory_order_relaxed);
  c.out_mode_duration_ns.store(s.mode_duration_ns, std::memory_order_relaxed);
  c.out_c_error_mask.store(s.c_error_mask, std::memory_order_relaxed);

  // blend: RUN일 때만 새 AI 명령을 반영 (HOLD·DECEL_STOP·FAULT에서는 목표 유지)
  const rt_control::BlendResult b = c.blend.update(c.last, g, s.allow_blend);
  for (int i = 0; i < 3; ++i) c.out_goal[i].store(b.goal[i], std::memory_order_relaxed);
  if (b.applied) c.out_w.store(b.w, std::memory_order_relaxed);
  c.out_blend_used_seq.store(b.used_ai_seq, std::memory_order_relaxed);
  c.out_blend_has_used.store(b.has_used, std::memory_order_relaxed);
  c.out_blend_apply_count.fetch_add(b.applied, std::memory_order_relaxed);
  c.out_rejected_wait.store(b.rejected_wait, std::memory_order_relaxed);
  for (int i = 0; i < 3; ++i) c.out_rejected_target[i].store(c.blend.rejected_target()[i], std::memory_order_relaxed);
  c.out_rejected_skip_count.fetch_add(b.rejected_skipped, std::memory_order_relaxed);

  // 실제 깨어난 간격 (첫 주기는 정해진 주기 그대로)
  if (c.prev_wakeup_ns != 0 && info.wakeup_ns > c.prev_wakeup_ns) {
    c.dt = static_cast<double>(info.wakeup_ns - c.prev_wakeup_ns) * 1e-9;
  }
  c.prev_wakeup_ns = info.wakeup_ns;
  // limiter: DECEL_STOP·FAULT면 a_max로 감속해 멈춤. 멈춘 위치를 계속 HalCommand로 내보냄
  const rt_control::LimiterResult m = c.limiter.update(b.goal, c.dt, s.stop);
  c.last_speed = m.speed;
  double meas_v2 = 0.0, meas_a2 = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double v = (m.pos[i] - c.meas_prev_pos[i]) / c.dt;
    const double a = (v - c.meas_prev_vel[i]) / c.dt;
    meas_v2 += v * v;
    meas_a2 += a * a;
    c.meas_prev_pos[i] = m.pos[i];
    c.meas_prev_vel[i] = v;
    c.out_pos[i].store(m.pos[i], std::memory_order_relaxed);
    c.out_vel[i].store(m.vel[i], std::memory_order_relaxed);
  }
  c.out_speed.store(m.speed, std::memory_order_relaxed);
  c.out_peak_speed.store(std::fmax(c.out_peak_speed.load(std::memory_order_relaxed), std::sqrt(meas_v2)),
                         std::memory_order_relaxed);
  c.out_peak_accel.store(std::fmax(c.out_peak_accel.load(std::memory_order_relaxed), std::sqrt(meas_a2)),
                         std::memory_order_relaxed);

  // B -> C 명령: 이 주기의 limiter 출력을 버퍼에 씀 (발행은 ros_bridge 발행 스레드가 함)
  rt_control::fill_hal_command(c.hal_cmd, c.session_id, info.wakeup_ns, m.pos, b.has_used, b.used_ai_seq,
                               c.hal_config);
  c.hal_buffer->write(c.hal_cmd);
  for (int i = 0; i < 3; ++i) c.last_sent_pos[i] = c.hal_cmd.pos[i];
  c.has_last_sent = true;

  // 도착 판정: blend 목표와 측정 TCP 차이가 arrive_tol 안에 arrive_hold_s 이상
  double tcp[3];
  const bool tcp_ok = robot_position(c, info.wakeup_ns, tcp);
  const rt_control::ArrivalResult arr = c.arrival.update(b.goal, tcp, tcp_ok, info.wakeup_ns);
  c.out_arrived.store(arr.arrived, std::memory_order_relaxed);
  c.out_tcp_error.store(arr.error_m, std::memory_order_relaxed);

  if (c.tracer) {
    row.mode = static_cast<uint8_t>(s.mode);
    row.mode_reason = s.reasons;
    row.has_used = b.has_used;
    row.used_ai_seq = b.used_ai_seq;
    row.used_ai_boot_id = b.used_ai_boot_id;
    row.rejected_wait = b.rejected_wait;
    row.arrived = arr.arrived;
    row.tcp_error_m = arr.error_m;
    row.weight = b.applied ? b.w : 0.0;
    for (int i = 0; i < 3; ++i) {
      row.goal[i] = b.goal[i];
      row.y[i] = m.pos[i];
    }
    row.speed = m.speed;
    row.command_seq = c.hal_cmd.command_seq;
    row.ai_recv_mono_ns = c.has_cmd ? c.last.recv_mono_ns : 0;
    row.ai_age_ns = g.age_ns;
    row.ai_apply_mono_ns = b.applied ? info.wakeup_ns : 0;
    row.reject_reason = g.reasons;
    row.compute_ns = rt_control::now_ns() - info.wakeup_ns;  // 마지막에 잼
    c.tracer->push(row);
  }

  if (c.has_cmd) {
    c.out_read_ai_seq.store(c.last.seq, std::memory_order_relaxed);
    c.out_used_boot_id.store(c.last.writer_boot_id, std::memory_order_relaxed);
    c.out_status.store(c.last.status, std::memory_order_relaxed);
    c.out_msg_age_ns.store(c.last.age_ns, std::memory_order_relaxed);
    c.out_age_ns.store(g.age_ns, std::memory_order_relaxed);
    c.out_has_cmd.store(true, std::memory_order_relaxed);
  }
  return true;
}

double ms(int64_t ns) { return static_cast<double>(ns) / 1e6; }

// 최근 1초 gate 요약 문자열 (메인 스레드에서만 호출, 할당 OK)
// 예: "gate 통과 1000 | 차단 EXPIRED 1000"
std::string gate_summary(const LoopContext& c, uint64_t& prev_pass,
                         std::array<uint64_t, rt_control::kGateReasonCount>& prev_block) {
  const uint64_t pass = c.out_gate_pass_count.load(std::memory_order_relaxed);
  std::string s = "gate 통과 " + std::to_string(pass - prev_pass) + " | 차단";
  prev_pass = pass;
  bool any = false;
  for (int i = 0; i < rt_control::kGateReasonCount; ++i) {
    const uint64_t n = c.out_gate_block_count[i].load(std::memory_order_relaxed);
    if (n != prev_block[i]) {
      s += std::string(" ") + rt_control::kGateReasonNames[i] + " " + std::to_string(n - prev_block[i]);
      any = true;
    }
    prev_block[i] = n;
  }
  if (!any) s += " 없음";
  char bits[32];
  std::snprintf(bits, sizeof(bits), " | 지금 0x%02x", c.out_gate_reasons.load(std::memory_order_relaxed));
  return s + bits;
}

// 최근 1초 blend 요약 문자열 (메인 스레드에서만 호출)
// 예: "목표 [0.5000, 0.0000, 0.7100] | 마지막 w 0.952 | 반영 15 (seq 120)"
//     아직 한 번도 반영 안 했으면 "... | 반영 없음"
std::string blend_summary(const LoopContext& c, uint64_t& prev_apply) {
  const uint64_t n = c.out_blend_apply_count.load(std::memory_order_relaxed);
  // 거부된 목표 대기 중이면 끝에 붙임
  char rej[160] = "";
  if (c.out_rejected_wait.load(std::memory_order_relaxed)) {
    std::snprintf(rej, sizeof(rej), " | 거부된 목표 대기 중 [%.4f, %.4f, %.4f] (무시한 새 명령 누적 %lu)",
                  c.out_rejected_target[0].load(std::memory_order_relaxed),
                  c.out_rejected_target[1].load(std::memory_order_relaxed),
                  c.out_rejected_target[2].load(std::memory_order_relaxed),
                  c.out_rejected_skip_count.load(std::memory_order_relaxed));
  }
  char buf[160];
  if (!c.out_blend_has_used.load(std::memory_order_relaxed)) {
    std::snprintf(buf, sizeof(buf), "목표 [%.4f, %.4f, %.4f] | 반영 없음",
                  c.out_goal[0].load(std::memory_order_relaxed),
                  c.out_goal[1].load(std::memory_order_relaxed),
                  c.out_goal[2].load(std::memory_order_relaxed));
    prev_apply = n;
    return std::string(buf) + rej;
  }
  std::snprintf(buf, sizeof(buf), "목표 [%.4f, %.4f, %.4f] | 마지막 w %.3f | 반영 %lu (seq %lu)",
                c.out_goal[0].load(std::memory_order_relaxed),
                c.out_goal[1].load(std::memory_order_relaxed),
                c.out_goal[2].load(std::memory_order_relaxed),
                c.out_w.load(std::memory_order_relaxed), n - prev_apply,
                c.out_blend_used_seq.load(std::memory_order_relaxed));
  prev_apply = n;
  return std::string(buf) + rej;
}

// 1초 limiter 요약 (메인 스레드에서만 호출). 최고값은 읽으면서 0으로 비움
// 예: "출력 [0.5000, 0.0000, 0.7100] | 속도 0.0000 m/s | 1초 최고 속도 0.1000 m/s, 최고 가속 0.5000 m/s²
//      | 도착 예 (TCP 오차 0.31 mm)"
std::string limiter_summary(LoopContext& c) {
  char buf[256];
  const double err = c.out_tcp_error.load(std::memory_order_relaxed);
  char err_text[48];
  if (std::isfinite(err)) {
    std::snprintf(err_text, sizeof(err_text), "TCP 오차 %.2f mm", err * 1e3);
  } else {
    std::snprintf(err_text, sizeof(err_text), "TCP 오차 없음 (측정 없음)");
  }
  std::snprintf(buf, sizeof(buf),
                "출력 [%.4f, %.4f, %.4f] | 속도 %.4f m/s | 1초 최고 속도 %.4f m/s, 최고 가속 %.4f m/s² | 도착 %s (%s)",
                c.out_pos[0].load(std::memory_order_relaxed),
                c.out_pos[1].load(std::memory_order_relaxed),
                c.out_pos[2].load(std::memory_order_relaxed),
                c.out_speed.load(std::memory_order_relaxed),
                c.out_peak_speed.exchange(0.0, std::memory_order_relaxed),
                c.out_peak_accel.exchange(0.0, std::memory_order_relaxed),
                c.out_arrived.load(std::memory_order_relaxed) ? "예" : "아니오", err_text);
  return buf;
}

// 비트마스크 -> "[5, 9]" (메인 스레드 전용)
std::string error_list(uint64_t mask) {
  std::string s = "[";
  for (int code = 0; code < 64; ++code) {
    if (!((mask >> code) & 1u)) continue;
    if (s.size() > 1) s += ", ";
    s += std::to_string(code);
  }
  return s + "]";
}

// 이유 비트 -> "EXPIRED, RANGE". C 오류가 이유에 있으면 켜진 오류 코드도 붙임 (메인 스레드 전용)
std::string reason_list(uint32_t reasons, uint64_t error_mask) {
  std::string s;
  for (int i = 0; i < rt_control::kSafetyReasonCount; ++i) {
    if (!((reasons >> i) & 1u)) continue;
    if (!s.empty()) s += ", ";
    s += rt_control::kSafetyReasonNames[i];
    // 지금 켜진 오류 코드를 붙임 (이미 꺼졌으면 붙이지 않음)
    const uint32_t bit = 1u << i;
    const uint64_t codes = bit == rt_control::SAFE_C_ERROR_DECEL   ? error_mask & rt_control::kDecelErrorMask
                           : bit == rt_control::SAFE_C_ERROR_FAULT ? error_mask & rt_control::kFaultErrorMask
                                                                   : 0;
    if (codes) s += error_list(codes);
  }
  return s.empty() ? "없음" : s;
}

const char* reset_result_text(rt_control::ResetResult r) {
  switch (r) {
    case rt_control::ResetResult::kAccepted: return "수락";
    case rt_control::ResetResult::kNotFault: return "거절 (FAULT 아님)";
    case rt_control::ResetResult::kCauseActive: return "거절 (FAULT 원인이 아직 있음)";
    case rt_control::ResetResult::kMoving: return "거절 (아직 멈추지 않음)";
    default: return "?";
  }
}

const char* mode_name(uint8_t m) { return rt_control::safety_mode_name(static_cast<rt_control::SafetyMode>(m)); }

// 루프 이벤트를 모두 꺼내 바로 출력 (메인 스레드 전용)
void print_events(LoopContext& c, const rclcpp::Logger& log) {
  LoopEvent e;
  while (c.events.pop(e)) {
    switch (e.kind) {
      case LoopEvent::kModeChange:
        if (e.to == static_cast<uint8_t>(rt_control::SafetyMode::kRun)) {
          RCLCPP_INFO(log, "[모드] %s → %s", mode_name(e.from), mode_name(e.to));
        } else {
          RCLCPP_WARN(log, "[모드] %s → %s (이유: %s)", mode_name(e.from), mode_name(e.to),
                      reason_list(e.reasons, e.error_mask).c_str());
        }
        break;
      case LoopEvent::kCError:
        if (e.ignored) {
          RCLCPP_WARN(log, "C 오류 이벤트: code %u %s (command_seq %lu) -> 무시 (세션 %lu, B 세션 %lu와 다름)", e.code,
                      e.active ? "켜짐" : "꺼짐", e.number, e.session, c.out_session_id.load(std::memory_order_relaxed));
        } else {
          RCLCPP_WARN(log, "C 오류 이벤트: code %u %s (command_seq %lu)", e.code, e.active ? "켜짐" : "꺼짐", e.number);
        }
        break;
      case LoopEvent::kInitHold:
        if (e.value == static_cast<uint8_t>(rt_control::InitHoldEvent::kStarted)) {
          RCLCPP_INFO(log, "[시작 준비] 유지 명령 발행 시작: 측정 TCP [%.4f, %.4f, %.4f] (session %lu)", e.pos[0],
                      e.pos[1], e.pos[2], e.number);
        } else {
          RCLCPP_WARN(log, "[시작 준비] HalState가 끊겨 유지 명령 발행을 멈춤, 신선한 HalState를 다시 기다림");
        }
        break;
      case LoopEvent::kRejectedGoal:
        RCLCPP_WARN(log,
                    "[모드] C가 목표를 거부 (오류 %s): AI 목표 [%.4f, %.4f, %.4f]를 기억, "
                    "새 AI 목표가 %.1f mm 이상 달라질 때까지 반영 안 함",
                    error_list(e.error_mask & rt_control::kTargetRejectErrorMask).c_str(), e.pos[0], e.pos[1],
                    e.pos[2], c.blend_config.rejected_goal_tol_m * 1e3);
        break;
      case LoopEvent::kResetResult:
        if (e.value == static_cast<uint8_t>(rt_control::ResetResult::kAccepted)) {
          RCLCPP_INFO(log, "[모드] FAULT 해제 요청: 수락, %s %lu",
                      c.registered_session_id == 0 ? "새 세션" : "등록 세션·순번 유지", e.number);
        } else {
          RCLCPP_WARN(log, "[모드] FAULT 해제 요청: %s",
                      reset_result_text(static_cast<rt_control::ResetResult>(e.value)));
        }
        break;
      case LoopEvent::kLimiterReset:
        switch (static_cast<rt_control::ResumeSource>(e.value)) {
          case rt_control::ResumeSource::kLastCommand:
            RCLCPP_INFO(log, "[모드] limiter를 마지막 발행 위치 [%.4f, %.4f, %.4f]에서 다시 시작 (측정 TCP와 %.2f mm)",
                        e.pos[0], e.pos[1], e.pos[2], e.diff * 1e3);
            break;
          case rt_control::ResumeSource::kRobotMismatch:
            RCLCPP_WARN(log,
                        "[모드] 마지막 발행 위치와 측정 TCP 차이 %.2f mm > resume_match_tol %.2f mm: "
                        "limiter를 실제 로봇 위치 [%.4f, %.4f, %.4f]에서 다시 시작",
                        e.diff * 1e3, c.resume_match_tol * 1e3, e.pos[0], e.pos[1], e.pos[2]);
            break;
          case rt_control::ResumeSource::kRobot:
            RCLCPP_INFO(log, "[모드] limiter%s를 실제 로봇 위치 [%.4f, %.4f, %.4f]에서 다시 시작",
                        e.goal_reset ? "·blend 목표" : "", e.pos[0], e.pos[1], e.pos[2]);
            break;
          default:
            RCLCPP_INFO(log, "[모드] limiter를 지금 출력 위치(로봇 위치 없음) [%.4f, %.4f, %.4f]에서 다시 시작",
                        e.pos[0], e.pos[1], e.pos[2]);
            break;
        }
        break;
    }
  }
}

// "~/..." -> "$HOME/..."
std::string expand_home(const std::string& path) {
  if (path.empty() || path[0] != '~') return path;
  const char* home = std::getenv("HOME");
  return std::string(home ? home : "") + path.substr(1);
}

std::string local_time(const char* fmt) {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), fmt, &tm);
  return buf;
}

std::string yaml_list(const double* v, int n) {
  std::string s = "[";
  char buf[32];
  for (int i = 0; i < n; ++i) {
    std::snprintf(buf, sizeof(buf), "%s%.6g", i ? ", " : "", v[i]);
    s += buf;
  }
  return s + "]";
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rt_control_node");

  rt_control::LoopConfig config;
  config.rt.sched_fifo = node->declare_parameter<bool>("fifo", false);
  config.rt.fifo_priority = static_cast<int>(node->declare_parameter<int64_t>("prio", 80));
  config.rt.cpu = static_cast<int>(node->declare_parameter<int64_t>("cpu", -1));
  config.rt.lock_memory = node->declare_parameter<bool>("mlock", false);
  const std::string log_root = expand_home(node->declare_parameter<std::string>("log_root", "~/pac_logs"));
  const std::string run_id_param = node->declare_parameter<std::string>("run_id", "");
  const std::string run_tag = node->declare_parameter<std::string>("run_tag", "");
  const bool trace_enable = node->declare_parameter<bool>("trace_enable", true);

  // 환경 점검 (시작할 때 한 번): RT 커널, 전원 프로파일, 실시간 우선순위 한도
  const rt_control::EnvInfo env = rt_control::read_env_info(config.rt.sched_fifo);
  const std::vector<std::string> env_warn = rt_control::env_warnings(env);
  for (const std::string& w : env_warn) RCLCPP_WARN(node->get_logger(), "[환경] %s", w.c_str());
  if (env_warn.empty()) {
    RCLCPP_INFO(node->get_logger(), "[환경] 점검 통과: PREEMPT_RT %s, 전원 %s, ulimit -r %ld",
                env.preempt_rt ? "있음" : "?", env.power_profile.empty() ? "(못 읽음)" : env.power_profile.c_str(),
                env.rtprio_limit);
  }
  const std::string start_time = local_time("%Y-%m-%dT%H:%M:%S%z");
  const rt_control::RunDir run =
      rt_control::prepare_run_dir(log_root, run_id_param, run_tag, local_time("%Y%m%d_%H%M%S"));
  if (!run.ok) {
    RCLCPP_FATAL(node->get_logger(), "%s", run.error.c_str());
    rclcpp::shutdown();
    return 1;
  }
  if (!run_id_param.empty() && !run_tag.empty()) {
    RCLCPP_WARN(node->get_logger(), "run_id를 주었으므로 run_tag(%s)는 폴더 이름에 쓰지 않음", run_tag.c_str());
  }
  const std::string& run_id = run.run_id;
  const std::string& run_dir = run.path;

  // gate 설정: 기본값은 GateConfig 한 곳에서 가져옴
  rt_control::GateConfig gate_config;
  gate_config.expected_layout_version = static_cast<uint16_t>(node->declare_parameter<int64_t>(
      "expected_layout_version", gate_config.expected_layout_version));
  gate_config.expected_command_type = static_cast<uint8_t>(node->declare_parameter<int64_t>(
      "expected_command_type", gate_config.expected_command_type));
  gate_config.ttl_ns = node->declare_parameter<int64_t>("ttl_ns", gate_config.ttl_ns);
  gate_config.confidence_min =
      node->declare_parameter<double>("confidence_min", gate_config.confidence_min);
  const std::vector<double> planned = node->declare_parameter<std::vector<double>>(
      "planned_position", std::vector<double>(gate_config.planned_position,
                                              gate_config.planned_position + 3));
  if (planned.size() != 3) {
    RCLCPP_FATAL(node->get_logger(), "planned_position은 [x, y, z] 3개여야 함 (지금 %zu개)",
                 planned.size());
    rclcpp::shutdown();
    return 1;
  }
  for (size_t i = 0; i < 3; ++i) gate_config.planned_position[i] = planned[i];
  gate_config.accept_radius = node->declare_parameter<double>("accept_radius", gate_config.accept_radius);

  // blend 설정: ttl과 시작 목표는 gate 설정을 그대로 씀
  rt_control::BlendConfig blend_config;
  const std::string blend_mode = node->declare_parameter<std::string>("blend_mode", "single");
  if (!rt_control::parse_blend_mode(blend_mode.c_str(), &blend_config.mode)) {
    RCLCPP_FATAL(node->get_logger(), "모르는 blend_mode: %s (지금은 single만 있음)", blend_mode.c_str());
    rclcpp::shutdown();
    return 1;
  }
  blend_config.ttl_ns = gate_config.ttl_ns;
  const std::vector<double> offset = node->declare_parameter<std::vector<double>>("target_offset", {0.0, 0.0, 0.0});
  if (offset.size() != 3) {
    RCLCPP_FATAL(node->get_logger(), "target_offset은 [x, y, z] 3개여야 함 (지금 %zu개)", offset.size());
    rclcpp::shutdown();
    return 1;
  }
  for (int i = 0; i < 3; ++i) blend_config.target_offset[i] = offset[i];
  blend_config.deadband_m = node->declare_parameter<double>("blend_deadband", blend_config.deadband_m);
  if (!(blend_config.deadband_m >= 0.0)) {
    RCLCPP_FATAL(node->get_logger(), "blend_deadband는 0 이상이어야 함 (지금 %f)", blend_config.deadband_m);
    rclcpp::shutdown();
    return 1;
  }
  if (rt_control::sanitize_offset(blend_config.target_offset)) {
    RCLCPP_WARN(node->get_logger(), "target_offset에 NaN·무한대가 있어 그 성분을 0으로 씀: [%.4f, %.4f, %.4f]",
                blend_config.target_offset[0], blend_config.target_offset[1], blend_config.target_offset[2]);
  }

  // limiter 설정
  rt_control::LimiterConfig limiter_config;
  limiter_config.v_max = node->declare_parameter<double>("v_max", limiter_config.v_max);
  limiter_config.a_max = node->declare_parameter<double>("a_max", limiter_config.a_max);
  limiter_config.j_max = node->declare_parameter<double>("j_max", limiter_config.j_max);
  if (!(limiter_config.j_max >= 0.0) || !std::isfinite(limiter_config.j_max)) {
    RCLCPP_FATAL(node->get_logger(), "j_max는 0(끔) 이상이어야 함 (지금 %f)", limiter_config.j_max);
    rclcpp::shutdown();
    return 1;
  }
  if (!(limiter_config.v_max > 0.0) || !(limiter_config.a_max > 0.0)) {
    RCLCPP_FATAL(node->get_logger(), "v_max, a_max는 0보다 커야 함 (v_max=%f a_max=%f)",
                 limiter_config.v_max, limiter_config.a_max);
    rclcpp::shutdown();
    return 1;
  }
  // 도착 판정·재개 기준
  rt_control::ArrivalConfig arrival_config;
  arrival_config.tol_m = node->declare_parameter<double>("arrive_tol", arrival_config.tol_m);
  const double arrive_hold_s = node->declare_parameter<double>("arrive_hold_s", arrival_config.hold_ns * 1e-9);
  arrival_config.hold_ns = static_cast<int64_t>(std::llround(arrive_hold_s * 1e9));
  const bool resume_from_last_cmd = node->declare_parameter<bool>("resume_from_last_cmd", true);
  const double resume_match_tol =
      node->declare_parameter<double>("resume_match_tol", rt_control::kDefaultResumeMatchTolM);
  blend_config.rejected_goal_tol_m =
      node->declare_parameter<double>("rejected_goal_tol", blend_config.rejected_goal_tol_m);
  if (!(blend_config.rejected_goal_tol_m >= 0.0)) {
    RCLCPP_FATAL(node->get_logger(), "rejected_goal_tol은 0 이상이어야 함 (지금 %f)", blend_config.rejected_goal_tol_m);
    rclcpp::shutdown();
    return 1;
  }
  if (!(arrival_config.tol_m >= 0.0) || !(arrive_hold_s >= 0.0) || !(resume_match_tol >= 0.0)) {
    RCLCPP_FATAL(node->get_logger(), "arrive_tol, arrive_hold_s, resume_match_tol은 0 이상이어야 함 (%f, %f, %f)",
                 arrival_config.tol_m, arrive_hold_s, resume_match_tol);
    rclcpp::shutdown();
    return 1;
  }
  // HalState가 start_timeout_s 안에 안 오면 쓰는 시작 위치
  const std::vector<double> start = node->declare_parameter<std::vector<double>>(
      "start_position", std::vector<double>(gate_config.planned_position,
                                            gate_config.planned_position + 3));
  if (start.size() != 3) {
    RCLCPP_FATAL(node->get_logger(), "start_position은 [x, y, z] 3개여야 함 (지금 %zu개)",
                 start.size());
    rclcpp::shutdown();
    return 1;
  }
  for (int i = 0; i < 3; ++i) blend_config.initial_goal[i] = gate_config.planned_position[i];
  const int64_t session_param = node->declare_parameter<int64_t>("session_id", 0);
  if (session_param < 0) {
    RCLCPP_FATAL(node->get_logger(), "session_id는 0(B가 발급) 또는 C에 등록된 양수여야 함 (지금 %ld)", session_param);
    rclcpp::shutdown();
    return 1;
  }
  const bool require_registered_state = node->declare_parameter<bool>("require_registered_state", true);
  const bool measured_orientation = node->declare_parameter<bool>("measured_orientation", true);
  const bool start_fallback_on_timeout = node->declare_parameter<bool>("start_fallback_on_timeout", false);
  const double start_timeout_s = node->declare_parameter<double>("start_timeout_s", 2.0);
  if (!(start_timeout_s >= 0.0)) {
    RCLCPP_FATAL(node->get_logger(), "start_timeout_s는 0 이상이어야 함 (지금 %f)", start_timeout_s);
    rclcpp::shutdown();
    return 1;
  }
  // 시작 준비 (warmup)
  rt_control::WarmupConfig warmup_config;
  warmup_config.states = static_cast<int>(node->declare_parameter<int64_t>("warmup_states", warmup_config.states));
  const double warmup_stable_ms =
      node->declare_parameter<double>("warmup_stable_ms", warmup_config.stable_ns * 1e-6);
  const double warmup_max_s = node->declare_parameter<double>("warmup_max_s", warmup_config.max_ns * 1e-9);
  const double warmup_hold_ms = node->declare_parameter<double>("warmup_hold_ms", warmup_config.hold_ns * 1e-6);
  if (warmup_config.states < 0 || !(warmup_stable_ms >= 0.0) || !(warmup_max_s >= 0.0) || !(warmup_hold_ms >= 0.0)) {
    RCLCPP_FATAL(node->get_logger(),
                 "warmup_states, warmup_stable_ms, warmup_max_s, warmup_hold_ms는 0 이상이어야 함 (%d, %f, %f, %f)",
                 warmup_config.states, warmup_stable_ms, warmup_max_s, warmup_hold_ms);
    rclcpp::shutdown();
    return 1;
  }
  warmup_config.stable_ns = static_cast<int64_t>(std::llround(warmup_stable_ms * 1e6));
  warmup_config.max_ns = static_cast<int64_t>(std::llround(warmup_max_s * 1e9));
  warmup_config.hold_ns = static_cast<int64_t>(std::llround(warmup_hold_ms * 1e6));
  warmup_config.publish_period_ns = config.period_ns;

  // B -> C 명령 설정: 기본값은 HalCommandConfig 한 곳에서 가져옴
  rt_control::HalCommandConfig hal_config;
  hal_config.hal_ttl_ns = node->declare_parameter<int64_t>("hal_ttl_ns", hal_config.hal_ttl_ns);
  const std::vector<double> quat = node->declare_parameter<std::vector<double>>(
      "tool_orientation", std::vector<double>(hal_config.tool_orientation, hal_config.tool_orientation + 4));
  double quat_norm = 0.0;
  for (double q : quat) quat_norm += q * q;
  quat_norm = std::sqrt(quat_norm);
  if (quat.size() != 4 || !(quat_norm > 1e-9) || !(hal_config.hal_ttl_ns > 0)) {
    RCLCPP_FATAL(node->get_logger(),
                 "tool_orientation은 0이 아닌 [x, y, z, w] 4개, hal_ttl_ns는 0보다 커야 함 (지금 %zu개, ttl %ld)",
                 quat.size(), hal_config.hal_ttl_ns);
    rclcpp::shutdown();
    return 1;
  }
  for (int i = 0; i < 4; ++i) hal_config.tool_orientation[i] = quat[i] / quat_norm;  // 단위 쿼터니언으로
  rt_control::BridgeConfig bridge_config;
  bridge_config.period_ns = config.period_ns;
  bridge_config.cpu = static_cast<int>(node->declare_parameter<int64_t>("bridge_cpu", -1));
  bridge_config.loop_cpu = config.rt.cpu;
  const int bridge_prio = static_cast<int>(node->declare_parameter<int64_t>("bridge_prio", 70));
  bridge_config.fifo_priority = config.rt.sched_fifo ? bridge_prio : 0;

  // safety_mode 설정: 기본값은 SafetyConfig 한 곳에서 가져옴
  rt_control::SafetyConfig safety_config;
  safety_config.range_reject_n = static_cast<int>(
      node->declare_parameter<int64_t>("range_reject_n", safety_config.range_reject_n));
  safety_config.resume_k = static_cast<int>(node->declare_parameter<int64_t>("resume_k", safety_config.resume_k));
  safety_config.ai_decel_after_ns = static_cast<int64_t>(
      node->declare_parameter<double>("ai_decel_after_s", safety_config.ai_decel_after_ns * 1e-9) * 1e9);
  safety_config.require_c_state = node->declare_parameter<bool>("require_c_state", safety_config.require_c_state);
  safety_config.c_state_timeout_ns = static_cast<int64_t>(
      node->declare_parameter<double>("c_state_timeout_ms", safety_config.c_state_timeout_ns * 1e-6) * 1e6);
  safety_config.c_expired_n = static_cast<int>(
      node->declare_parameter<int64_t>("c_expired_n", safety_config.c_expired_n));
  safety_config.resume_delay_ns = static_cast<int64_t>(
      node->declare_parameter<double>("resume_delay_s", safety_config.resume_delay_ns * 1e-9) * 1e9);
  if (safety_config.range_reject_n < 1 || safety_config.resume_k < 1 || safety_config.ai_decel_after_ns < 0 ||
      safety_config.c_state_timeout_ns <= 0 || safety_config.c_expired_n < 1 ||
      safety_config.c_expired_n > rt_control::kMaxExpiredCount || safety_config.resume_delay_ns < 0) {
    RCLCPP_FATAL(node->get_logger(),
                 "safety 파라미터 범위 오류: range_reject_n·resume_k ≥ 1, c_expired_n 1~%d, "
                 "ai_decel_after_s·resume_delay_s ≥ 0, c_state_timeout_ms > 0",
                 rt_control::kMaxExpiredCount);
    rclcpp::shutdown();
    return 1;
  }
  if (config.rt.sched_fifo && bridge_prio >= config.rt.fifo_priority) {
    RCLCPP_WARN(node->get_logger(), "bridge_prio=%d 가 1ms 루프 prio=%d 이상. 루프보다 낮게 두는 것을 권장",
                bridge_prio, config.rt.fifo_priority);
  }

  // meta.yaml: 이 실행을 나중에 다시 볼 때 필요한 조건
  if (std::FILE* f = std::fopen((run_dir + "/meta.yaml").c_str(), "w")) {
    utsname un{};
    uname(&un);
    const char* rmw = std::getenv("RMW_IMPLEMENTATION");
    const char* domain = std::getenv("ROS_DOMAIN_ID");
    std::fprintf(f, "run_id: %s\nstart_time: \"%s\"\nrun_tag: \"%s\"\nlog_root: \"%s\"\nkernel: %s\nhost: %s\n",
                 run_id.c_str(), start_time.c_str(), run_tag.c_str(), log_root.c_str(), un.release, un.nodename);
    std::fprintf(f, "rmw: \"%s\"\nros_domain_id: \"%s\"\n", rmw ? rmw : "", domain ? domain : "");
    std::fprintf(f, "env:\n  preempt_rt: %s\n  power_profile: \"%s\"\n  rtprio_limit: %ld\n  warnings: %zu\n",
                 env.preempt_rt ? "true" : "false", env.power_profile.c_str(), env.rtprio_limit, env_warn.size());
    std::fprintf(f, "rt:\n  period_ns: %ld\n  fifo: %s\n  prio: %d\n  cpu: %d\n  mlock: %s\n"
                    "  bridge_cpu: %d\n  bridge_prio: %d\n  trace_enable: %s\n",
                 config.period_ns, config.rt.sched_fifo ? "true" : "false", config.rt.fifo_priority, config.rt.cpu,
                 config.rt.lock_memory ? "true" : "false", bridge_config.cpu, bridge_prio,
                 trace_enable ? "true" : "false");
    std::fprintf(f, "gate:\n  expected_layout_version: %u\n  expected_command_type: %u\n  ttl_ns: %ld\n"
                    "  confidence_min: %g\n  planned_position: %s\n  accept_radius: %g\n",
                 gate_config.expected_layout_version, gate_config.expected_command_type, gate_config.ttl_ns,
                 gate_config.confidence_min, yaml_list(gate_config.planned_position, 3).c_str(),
                 gate_config.accept_radius);
    std::fprintf(f, "blend:\n  blend_mode: %s\n  target_offset: %s\n  blend_deadband: %g\n  rejected_goal_tol: %g\n"
                    "limiter:\n  v_max: %g\n  a_max: %g\n  j_max: %g\n",
                 blend_mode.c_str(), yaml_list(blend_config.target_offset, 3).c_str(), blend_config.deadband_m,
                 blend_config.rejected_goal_tol_m, limiter_config.v_max, limiter_config.a_max, limiter_config.j_max);
    std::fprintf(f, "arrival:\n  arrive_tol: %g\n  arrive_hold_s: %g\n"
                    "resume:\n  resume_from_last_cmd: %s\n  resume_match_tol: %g\n",
                 arrival_config.tol_m, arrival_config.hold_ns * 1e-9, resume_from_last_cmd ? "true" : "false",
                 resume_match_tol);
    std::fprintf(f, "start:\n  session_id: %ld\n  require_registered_state: %s\n  measured_orientation: %s\n"
                    "  start_fallback_on_timeout: %s\n  start_timeout_s: %g\n  start_position: %s\n",
                 session_param, require_registered_state ? "true" : "false", measured_orientation ? "true" : "false",
                 start_fallback_on_timeout ? "true" : "false", start_timeout_s, yaml_list(start.data(), 3).c_str());
    std::fprintf(f, "warmup:\n  warmup_states: %d\n  warmup_stable_ms: %g\n  warmup_max_s: %g\n  warmup_hold_ms: %g\n",
                 warmup_config.states, warmup_stable_ms, warmup_max_s, warmup_hold_ms);
    std::fprintf(f, "hal_command:\n  hal_ttl_ns: %ld\n  tool_orientation: %s\n", hal_config.hal_ttl_ns,
                 yaml_list(hal_config.tool_orientation, 4).c_str());
    std::fprintf(f, "safety:\n  range_reject_n: %d\n  resume_k: %d\n  ai_decel_after_s: %g\n"
                    "  require_c_state: %s\n  c_state_timeout_ms: %g\n  c_expired_n: %d\n  resume_delay_s: %g\n",
                 safety_config.range_reject_n, safety_config.resume_k, safety_config.ai_decel_after_ns * 1e-9,
                 safety_config.require_c_state ? "true" : "false", safety_config.c_state_timeout_ns * 1e-6,
                 safety_config.c_expired_n, safety_config.resume_delay_ns * 1e-9);
    std::fclose(f);
  } else {
    RCLCPP_WARN(node->get_logger(), "meta.yaml을 쓸 수 없음 (기록 없이 계속)");
  }

  auto receiver = std::make_unique<rt_control::AiCommandReceiver>(*node, run_dir + "/ai_rx.csv");
  RCLCPP_INFO(node->get_logger(), "실행 기록 폴더: %s (run_id %s)", run_dir.c_str(), run_id.c_str());
  RCLCPP_INFO(node->get_logger(), "실시간 옵션: fifo=%d prio=%d cpu=%d mlock=%d",
              config.rt.sched_fifo, config.rt.fifo_priority, config.rt.cpu, config.rt.lock_memory);
  RCLCPP_INFO(node->get_logger(),
              "gate: layout_version=%u command_type=%u ttl=%.1f ms confidence_min=%.2f "
              "planned=[%.3f, %.3f, %.3f] accept_radius=%.3f m",
              gate_config.expected_layout_version, gate_config.expected_command_type,
              ms(gate_config.ttl_ns), gate_config.confidence_min, gate_config.planned_position[0],
              gate_config.planned_position[1], gate_config.planned_position[2],
              gate_config.accept_radius);
  RCLCPP_INFO(node->get_logger(),
              "blend: mode=%s ttl=%.1f ms 시작 목표=[%.3f, %.3f, %.3f] target_offset=[%.3f, %.3f, %.3f]",
              blend_mode.c_str(), ms(blend_config.ttl_ns), blend_config.initial_goal[0],
              blend_config.initial_goal[1], blend_config.initial_goal[2], blend_config.target_offset[0],
              blend_config.target_offset[1], blend_config.target_offset[2]);
  RCLCPP_INFO(node->get_logger(), "limiter: v_max=%.3f m/s a_max=%.3f m/s² j_max=%.3f m/s³%s | blend_deadband=%.2f mm",
              limiter_config.v_max, limiter_config.a_max, limiter_config.j_max,
              limiter_config.j_max > 0.0 ? "" : " (저크 제한 끔)", blend_config.deadband_m * 1e3);
  RCLCPP_INFO(node->get_logger(),
              "도착 판정: arrive_tol=%.2f mm arrive_hold=%.3f s | 재개: resume_from_last_cmd=%d resume_match_tol=%.2f mm",
              arrival_config.tol_m * 1e3, arrival_config.hold_ns * 1e-9, resume_from_last_cmd,
              resume_match_tol * 1e3);
  if (session_param == 0) {
    RCLCPP_INFO(node->get_logger(), "세션: 첫 신선한 HalState 때 B가 발급 (session_id 0, INIT 유지 명령부터 씀), FAULT 해제 시 새 세션");
  } else {
    RCLCPP_INFO(node->get_logger(), "세션: C 등록 세션 %ld 사용, FAULT 해제 시 세션·순번 유지%s", session_param,
                require_registered_state ? ", 그 세션의 신선한 READY/ACTIVE 상태까지 INIT 대기" : "");
  }
  RCLCPP_INFO(node->get_logger(),
              "시작 준비: HalState %d개 + 발행 %.0f ms 동안 overrun 없음 + /hal/final_command 구독자 1개 이상 "
              "+ 측정 TCP 유지 명령 %.0f ms 동안 끊김·overrun 없음 (최대 %.1f초, 넘으면 경고) | "
              "거부된 목표 재반영 거리 rejected_goal_tol=%.1f mm",
              warmup_config.states, warmup_stable_ms, warmup_hold_ms, warmup_max_s,
              blend_config.rejected_goal_tol_m * 1e3);
  if (!measured_orientation) {
    RCLCPP_WARN(node->get_logger(),
                "measured_orientation false: INIT 유지 명령은 측정 자세를, RUN은 tool_orientation을 씀 "
                "(RUN 시작 때 자세가 바뀔 수 있음)");
  }
  if (start_fallback_on_timeout) {
    RCLCPP_INFO(node->get_logger(),
                "시작: INIT에서 pose_valid HalState를 %.1f초 기다림, 없으면 start_position=[%.3f, %.3f, %.3f]",
                start_timeout_s, start[0], start[1], start[2]);
  } else {
    RCLCPP_INFO(node->get_logger(),
                "시작: INIT에서 pose_valid HalState를 계속 기다림 (start_fallback_on_timeout false, 임의 위치로 시작 안 함)");
  }
  RCLCPP_INFO(node->get_logger(),
              "HalCommand: hal_ttl=%.1f ms tool_orientation=[%.3f, %.3f, %.3f, %.3f] bridge_cpu=%d bridge_fifo=%d",
              ms(hal_config.hal_ttl_ns), hal_config.tool_orientation[0], hal_config.tool_orientation[1],
              hal_config.tool_orientation[2], hal_config.tool_orientation[3], bridge_config.cpu,
              bridge_config.fifo_priority);

  // 1ms 루프 코어(와 짝꿍)를 루프 전용으로: 지금 있는 스레드(메인·DDS)를 내보냄.
  // 이 뒤에 메인에서 만드는 스레드(통신·발행·기록)도 같은 설정을 물려받음
  if (config.rt.cpu >= 0) {
    const int moved = rt_control::keep_threads_off_cpu(config.rt.cpu);
    RCLCPP_INFO(node->get_logger(), "루프 CPU %d와 짝꿍 코어에서 다른 B 스레드 내보냄: %d개", config.rt.cpu, moved);
  }

  // 주기 기록(rt.csv). 기록 스레드는 위에서 루프 코어를 비운 뒤에 만들어야 그 코어를 피함
  std::unique_ptr<rt_control::CycleTracer> tracer;
  if (trace_enable) {
    tracer = std::make_unique<rt_control::CycleTracer>();
    if (!tracer->open(run_dir + "/rt.csv")) {
      RCLCPP_WARN(node->get_logger(), "rt.csv를 열 수 없음 (주기 기록 없이 계속)");
      tracer.reset();
    }
  }

  // B -> C 명령 버퍼 (1ms 루프가 쓰고 발행 스레드가 읽음)
  auto hal_buffer = std::make_unique<rt_control::Seqlock<HalCommandSnapshot>>();
  auto bridge = std::make_unique<rt_control::RosBridge>(*node, *hal_buffer, bridge_config);

  LoopContext ctx;
  ctx.ai_buffer = &receiver->buffer();
  ctx.gate_config = gate_config;
  ctx.blend_config = blend_config;
  ctx.blend = rt_control::BlendPolicy(blend_config);
  for (int i = 0; i < 3; ++i) ctx.out_goal[i].store(blend_config.initial_goal[i]);
  ctx.limiter = rt_control::MotionLimiter(limiter_config);
  ctx.arrival = rt_control::ArrivalDetector(arrival_config);
  ctx.resume_from_last_cmd = resume_from_last_cmd;
  ctx.resume_match_tol = resume_match_tol;
  ctx.dt = static_cast<double>(config.period_ns) * 1e-9;
  // limiter는 RUN으로 바뀔 때 다시 reset (HalState 위치 또는 start_position).
  // 그 전(INIT) 기록에 0이 찍히지 않게 일단 start_position에 둠
  ctx.limiter.reset(start.data());
  for (int i = 0; i < 3; ++i) {
    ctx.start_position[i] = start[i];
    ctx.out_pos[i].store(start[i]);
  }
  ctx.start_timeout_ns = static_cast<int64_t>(start_timeout_s * 1e9);
  ctx.registered_session_id = static_cast<uint64_t>(session_param);
  ctx.require_registered_state = require_registered_state;
  ctx.measured_orientation = measured_orientation;
  ctx.start_fallback_on_timeout = start_fallback_on_timeout;
  ctx.hal_buffer = hal_buffer.get();
  ctx.hal_config = hal_config;
  ctx.state_buffer = &bridge->state_buffer();
  ctx.bridge = bridge.get();
  ctx.tracer = tracer.get();
  ctx.safety = rt_control::SafetyStateMachine(safety_config);
  ctx.warmup = rt_control::StartupWarmup(warmup_config);
  rt_control::InitHoldConfig init_hold_config;
  init_hold_config.state_timeout_ns = safety_config.c_state_timeout_ns;
  init_hold_config.registered_session_id = static_cast<uint64_t>(session_param);
  ctx.init_hold = rt_control::InitHold(init_hold_config);
  RCLCPP_INFO(node->get_logger(),
              "safety: range_reject_n=%d resume_k=%d ai_decel_after=%.2f s require_c_state=%d "
              "c_state_timeout=%.1f ms c_expired_n=%d resume_delay=%.2f s",
              safety_config.range_reject_n, safety_config.resume_k, safety_config.ai_decel_after_ns * 1e-9,
              safety_config.require_c_state, ms(safety_config.c_state_timeout_ns), safety_config.c_expired_n,
              safety_config.resume_delay_ns * 1e-9);

  // FAULT 해제 서비스. 루프를 기다리지 않음: 해제 가능해 보이면 요청만 켜고, 결과는 [모드] 로그로 나옴
  auto reset_srv = node->create_service<std_srvs::srv::Trigger>(
      "/rt/reset_fault", [&ctx](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const uint8_t mode = ctx.out_safety_mode.load(std::memory_order_relaxed);
        const uint32_t active = ctx.out_safety_active.load(std::memory_order_relaxed);
        constexpr uint32_t kFaultBits = rt_control::SAFE_AI_VERSION | rt_control::SAFE_C_ERROR_FAULT;
        if (mode != static_cast<uint8_t>(rt_control::SafetyMode::kFault)) {
          res->success = false;
          res->message = std::string("FAULT가 아님 (지금 ") + mode_name(mode) + ")";
        } else if (active & kFaultBits) {
          res->success = false;
          res->message = "FAULT 원인이 아직 있음: " +
                         reason_list(active & kFaultBits, ctx.out_c_error_mask.load(std::memory_order_relaxed));
        } else {
          ctx.reset_request.store(true, std::memory_order_relaxed);
          res->success = true;
          res->message = "FAULT 해제 요청을 보냄 (결과는 B 로그의 [모드] 줄 확인)";
        }
      });

  // 통신 스레드
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread comm_thread([&executor] { executor.spin(); });

  // 제어 스레드 (실시간 설정은 이 스레드 자신에게 적용)
  rt_control::RtControlLoop loop(config);
  std::thread loop_thread([&loop, &ctx] {
    loop.apply_rt_settings();
    loop.run(on_cycle, &ctx);
  });

  // 발행 스레드 (INIT 동안은 버퍼가 비어 있어서 아무것도 발행하지 않음)
  bridge->start();

  // 메인 스레드: 10ms마다 루프 이벤트 출력, 1초마다 요약 (Ctrl+C면 rclcpp::ok()가 false가 됨)
  uint64_t prev_pass = 0;
  uint64_t prev_apply = 0;
  std::array<uint64_t, rt_control::kGateReasonCount> prev_block{};
  uint64_t prev_overrun = 0, prev_published = 0, prev_skipped = 0;
  bool start_reported = false;
  bool warmup_warned = false;
  bool profile_reported = false;
  auto next_summary = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // 구독자 수는 rmw 잠금이 있어 1ms 루프 대신 여기서 읽어 넘김 (INIT 시작 준비용)
    ctx.cmd_subscribers.store(bridge->command_subscriber_count(), std::memory_order_relaxed);
    print_events(ctx, node->get_logger());
    // 첫 발행 몇 번의 구간별 측정을 한 번 출력 (첫 발행 지연 원인 확인용)
    if (!profile_reported) {
      rt_control::PublishProfile prof[rt_control::kProfiledPublishes];
      if (bridge->publish_profile(prof)) {
        profile_reported = true;
        for (int i = 0; i < rt_control::kProfiledPublishes; ++i) {
          RCLCPP_INFO(node->get_logger(),
                      "[첫 발행 측정] #%d seq %lu | 깨어남 지연 %.1f us | now() %.1f us | publish() %.1f us | "
                      "page fault minor %ld major %ld | 컨텍스트 스위치 자발 %ld 비자발 %ld",
                      i + 1, prof[i].command_seq, prof[i].late_ns / 1e3, prof[i].now_ns / 1e3,
                      prof[i].publish_ns / 1e3, prof[i].minflt, prof[i].majflt, prof[i].nvcsw, prof[i].nivcsw);
        }
      }
    }
    // 시작 준비가 warmup_max_s 안에 안 끝나면 한 번 경고 (시작은 계속 기다림)
    if (!warmup_warned && ctx.out_mode.load(std::memory_order_relaxed) == static_cast<uint8_t>(RunMode::kInit) &&
        ctx.out_warmup_ns.load(std::memory_order_relaxed) > warmup_config.max_ns) {
      warmup_warned = true;
      const uint32_t met = ctx.out_warmup_met.load(std::memory_order_relaxed);
      std::string missing;
      if (!(met & rt_control::WARM_STATES)) {
        missing += " HalState " + std::to_string(ctx.out_warmup_states.load()) + "/" +
                   std::to_string(warmup_config.states) + "개";
      }
      if (!(met & rt_control::WARM_PUBLISH)) missing += " 발행 주기 불안정(overrun 계속)";
      if (!(met & rt_control::WARM_SUBSCRIBER)) missing += " /hal/final_command 구독자 없음";
      if (!(met & rt_control::WARM_HOLD)) {
        missing += ctx.out_init_holding.load() ? " 유지 명령 발행 불안정(overrun)" : " 유지 명령 발행 안 함(신선한 HalState 없음)";
      }
      RCLCPP_WARN(node->get_logger(), "[시작 준비] %.1f초가 지나도 준비 안 됨:%s (INIT에서 계속 기다림)", warmup_max_s,
                  missing.c_str());
    }
    if (!rclcpp::ok()) break;
    if (std::chrono::steady_clock::now() < next_summary) continue;
    next_summary += std::chrono::seconds(1);

    const auto status = static_cast<unsigned>(ctx.out_status.load(std::memory_order_relaxed));
    const std::string gate = gate_summary(ctx, prev_pass, prev_block);
    const std::string blend = blend_summary(ctx, prev_apply);
    const std::string motion = limiter_summary(ctx);
    if (ctx.out_has_cmd.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(node->get_logger(),
                  "받음 %lu | 버림 %lu | boot변경 %lu | 루프 seq %lu (boot %u) | 현재 나이 %.1f ms "
                  "(msg age %.1f ms) | status %u | seqlock 읽기실패 %lu | 주기 %lu | %s | %s",
                  receiver->received_count(), receiver->dropped_count(),
                  receiver->boot_change_count(), ctx.out_read_ai_seq.load(std::memory_order_relaxed),
                  ctx.out_used_boot_id.load(std::memory_order_relaxed),
                  ms(ctx.out_age_ns.load(std::memory_order_relaxed)),
                  ms(ctx.out_msg_age_ns.load(std::memory_order_relaxed)), status,
                  ctx.out_read_failures.load(std::memory_order_relaxed),
                  ctx.out_cycles.load(std::memory_order_relaxed), gate.c_str(), blend.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "받음 %lu | 버림 %lu | 아직 받은 명령 없음 | seqlock 읽기실패 %lu | 주기 %lu | %s | %s",
                  receiver->received_count(), receiver->dropped_count(),
                  ctx.out_read_failures.load(std::memory_order_relaxed),
                  ctx.out_cycles.load(std::memory_order_relaxed), gate.c_str(), blend.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "  %s", motion.c_str());

    // RUN 시작을 한 번 알림 (루프는 printf를 못 하므로 여기서)
    const auto source = static_cast<StartSource>(ctx.out_start_source.load(std::memory_order_acquire));
    if (!start_reported && source != StartSource::kNone) {
      start_reported = true;
      const double p[3] = {ctx.out_start_pos[0].load(), ctx.out_start_pos[1].load(), ctx.out_start_pos[2].load()};
      if (source == StartSource::kHalState) {
        RCLCPP_INFO(node->get_logger(),
                    "  마지막 유지 명령 위치에서 시작 [%.4f, %.4f, %.4f] (session %lu, 시작 준비 %.0f ms, 유지 명령 %lu개)",
                    p[0], p[1], p[2], ctx.out_session_id.load(), ms(ctx.out_warmup_ns.load()),
                    ctx.out_init_hold_published.load());
      } else {
        RCLCPP_WARN(node->get_logger(),
                    "  %.1f초 안에 pose_valid HalState가 없어 start_position에서 시작 [%.4f, %.4f, %.4f] (session %lu)",
                    start_timeout_s, p[0], p[1], p[2], ctx.out_session_id.load());
      }
    }

    const bool running = ctx.out_mode.load(std::memory_order_relaxed) == static_cast<uint8_t>(RunMode::kRun);
    const uint64_t overrun = loop.overrun_count();
    char warm[200] = "";
    if (!running) {
      const uint32_t met = ctx.out_warmup_met.load(std::memory_order_relaxed);
      std::snprintf(warm, sizeof(warm),
                    " | 시작 준비: HalState %d/%d, 발행 안정 %s, 구독자 %lu, 유지 명령 %s (누적 %lu, 안정 %s)",
                    ctx.out_warmup_states.load(), warmup_config.states, (met & rt_control::WARM_PUBLISH) ? "예" : "아니오",
                    ctx.cmd_subscribers.load(std::memory_order_relaxed),
                    ctx.out_init_holding.load(std::memory_order_relaxed) ? "발행 중" : "안 함",
                    ctx.out_init_hold_published.load(std::memory_order_relaxed),
                    (met & rt_control::WARM_HOLD) ? "예" : "아니오");
    }
    RCLCPP_INFO(node->get_logger(), "  루프: %s | overrun %lu | 최대 지터 %.1f us | C 상태 읽기실패 %lu%s",
                running ? "RUN" : "INIT", overrun - prev_overrun,
                static_cast<double>(ctx.out_peak_jitter_ns.exchange(0, std::memory_order_relaxed)) / 1e3,
                ctx.out_state_read_failures.load(std::memory_order_relaxed), warm);
    prev_overrun = overrun;

    const uint64_t published = bridge->published_count();
    const uint64_t skipped = bridge->skipped_seq_count();
    RCLCPP_INFO(node->get_logger(),
                "  bridge: 발행 %lu | 건너뛴 seq %lu (누적 %lu) | 읽기실패 %lu | 발행 overrun %lu | 마지막 발행 seq %lu"
                " | 최대 publish() %.1f us",
                published - prev_published, skipped - prev_skipped, skipped, bridge->read_failure_count(),
                bridge->publish_overrun_count(), bridge->last_published_seq(), bridge->max_publish_ns() / 1e3);
    prev_published = published;
    prev_skipped = skipped;

    if (ctx.out_has_state.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(node->get_logger(),
                  "  C 상태: 나이 %.1f ms | %s | state_seq %lu | C 처리 seq %lu (%s) | C 세션 %s | "
                  "켜진 오류 %s (상태 메시지 %s) | 다른 세션 오류 무시 누적 %lu",
                  ms(ctx.out_state_age_ns.load(std::memory_order_relaxed)),
                  rt_control::hal_state_name(ctx.out_hal_state.load(std::memory_order_relaxed)),
                  ctx.out_state_seq.load(std::memory_order_relaxed),
                  ctx.out_c_last_command_seq.load(std::memory_order_relaxed),
                  ctx.out_c_last_command_result.load(std::memory_order_relaxed) == 1 ? "REJECTED" : "ACCEPTED",
                  ctx.out_c_session_id.load(std::memory_order_relaxed) == ctx.out_session_id.load() ? "같음" : "다름",
                  error_list(bridge->active_error_mask()).c_str(),
                  error_list(ctx.out_state_error_mask.load(std::memory_order_relaxed)).c_str(),
                  ctx.out_ignored_error_events.load(std::memory_order_relaxed));
    } else {
      RCLCPP_INFO(node->get_logger(), "  C 상태: 아직 받은 HalState 없음 | 켜진 오류 %s",
                  error_list(bridge->active_error_mask()).c_str());
    }

    if (tracer) {
      RCLCPP_INFO(node->get_logger(), "  trace: 쓴 줄 %lu | 버린 줄 %lu", tracer->written(), tracer->dropped());
    }
    const bool in_run = ctx.out_mode.load(std::memory_order_relaxed) == static_cast<uint8_t>(RunMode::kRun);
    if (in_run) {
      RCLCPP_INFO(node->get_logger(), "  모드: %s (이유: %s) | %.1f초째 | 지금 켜진 조건: %s | 이벤트 넘침 %lu",
                  mode_name(ctx.out_safety_mode.load(std::memory_order_relaxed)),
                  reason_list(ctx.out_safety_reasons.load(std::memory_order_relaxed),
                              ctx.out_c_error_mask.load(std::memory_order_relaxed)).c_str(),
                  ctx.out_mode_duration_ns.load(std::memory_order_relaxed) * 1e-9,
                  reason_list(ctx.out_safety_active.load(std::memory_order_relaxed),
                              ctx.out_c_error_mask.load(std::memory_order_relaxed)).c_str(),
                  ctx.event_overflow.load(std::memory_order_relaxed));
    }
  }

  // 종료: 루프 -> 발행 스레드 -> executor -> 수신기(기록 스레드) 순서
  loop.request_stop();
  loop_thread.join();
  if (tracer) {
    tracer->close();  // 남은 줄까지 다 쓰고 닫음
    RCLCPP_INFO(node->get_logger(), "trace 끝: 쓴 줄 %lu | 버린 줄 %lu -> %s/rt.csv", tracer->written(),
                tracer->dropped(), run_dir.c_str());
  }
  bridge->stop();
  executor.cancel();
  comm_thread.join();
  if (loop.overrun_count() > 0) {
    RCLCPP_WARN(node->get_logger(), "건너뛴 주기(overrun): %lu", loop.overrun_count());
  }
  bridge.reset();
  receiver.reset();
  rclcpp::shutdown();
  return 0;
}
