// safety_mode: AI·C의 이상을 보고 B의 운전 모드를 정하는 상태 기계
//
// 모드 우선순위: FAULT > DECEL_STOP > HOLD > RUN  (INIT은 노드가 관리)
//   RUN        : 정상. blend가 새 AI 명령을 반영함
//   HOLD       : AI를 쓸 수 없음. blend 목표를 그대로 유지(새 AI 값만 반영 안 함). 로봇은 멈추지 않음
//                [팀 결정] 목표는 이미 신뢰도만큼 반영된 결과라 추가 보정 없음
//   DECEL_STOP : C 쪽 문제(로봇을 제어할 수 없음). limiter가 a_max로 감속해 그 자리에 멈춤
//   FAULT      : 사람이 확인해야 하는 문제. 멈추고, 원인이 풀린 뒤 해제 요청이 와야만 풀림
//
// 모드마다 "래치(걸쇠)"를 하나씩 두고, 조건이 생기면 래치를 걸고 풀림 규칙을 만족하면 푼다.
// 지금 모드 = 걸려 있는 래치 중 가장 높은 것. 그래서 여러 조건이 동시에 걸려도 가장 높은 모드가 된다.
//   HOLD 래치  : AI 조건(gate가 EXPIRED·NO_DATA·STATUS·LOW_CONF로 막음, RANGE 연속 range_reject_n번)
//                또는 C의 E_EXPIRED가 최근 1초에 c_expired_n번 이상.
//                풀림: HOLD 조건이 없는 상태에서 새 명령이 연속 resume_k번 gate 통과
//   DECEL 래치 : HOLD가 ai_decel_after_s보다 오래 감(0이면 끔), C 상태 없음·타임아웃,
//                hal_state가 READY·ACTIVE 아님, 오류 4·5·8·10.
//                풀림: 조건 없는 상태가 resume_delay 동안 계속 + limiter 속도 0 -> 그때 limiter reset 요청
//   FAULT 래치 : gate VERSION 불일치, 오류 1·2·6·9.
//                풀림: 조건 없음 + limiter 속도 0 + 해제 요청 -> 새 세션 + limiter reset 요청
//
// 시계 호출·로그·메모리 할당이 없다. 지금 시각은 호출하는 쪽이 넘겨준다.
#pragma once

#include <cstdint>

namespace rt_control {

enum class SafetyMode : uint8_t { kRun = 0, kHold = 1, kDecelStop = 2, kFault = 3 };

const char* safety_mode_name(SafetyMode m);

// 모드에 들어온 이유 비트 (여러 개가 동시에 켜질 수 있음)
enum SafetyReason : uint32_t {
  SAFE_AI_NO_DATA = 1u << 0,      // gate NO_DATA (A 명령을 받은 적 없음)
  SAFE_AI_EXPIRED = 1u << 1,      // gate EXPIRED (A가 끊김)
  SAFE_AI_STATUS = 1u << 2,       // gate STATUS (A가 실패 표시)
  SAFE_AI_LOW_CONF = 1u << 3,     // gate LOW_CONF
  SAFE_AI_RANGE = 1u << 4,        // 새 명령이 RANGE로 연속 range_reject_n번 막힘
  SAFE_HOLD_TIMEOUT = 1u << 5,    // HOLD가 ai_decel_after_s보다 오래 감
  SAFE_AI_VERSION = 1u << 6,      // gate VERSION 불일치
  SAFE_C_NO_STATE = 1u << 7,      // HalState를 받은 적 없음
  SAFE_C_STATE_TIMEOUT = 1u << 8, // 상태 나이 > c_state_timeout
  SAFE_C_NOT_READY = 1u << 9,     // hal_state가 READY·ACTIVE가 아님
  SAFE_C_ERROR_DECEL = 1u << 10,  // 오류 4·5·8·10이 켜져 있음
  SAFE_C_ERROR_FAULT = 1u << 11,  // 오류 1·2·6·9가 켜져 있음
  SAFE_C_EXPIRED = 1u << 12,      // E_EXPIRED가 최근 1초에 c_expired_n번 이상
};
constexpr int kSafetyReasonCount = 13;
constexpr const char* kSafetyReasonNames[kSafetyReasonCount] = {
    "NO_DATA",     "EXPIRED",       "STATUS",        "LOW_CONF", "RANGE",   "HOLD_TIMEOUT", "VERSION",
    "NO_C_STATE",  "STATE_TIMEOUT", "HAL_NOT_READY", "C_ERROR",  "C_FATAL", "C_EXPIRED"};

// HalError 코드 (intelion_interfaces/msg/HalError.msg와 같은 번호)
constexpr uint16_t kErrFormat = 1;
constexpr uint16_t kErrUnsupported = 2;
constexpr uint16_t kErrExpired = 3;
constexpr uint16_t kErrOutOfRange = 4;
constexpr uint16_t kErrIkFailed = 5;
constexpr uint16_t kErrTf = 6;
constexpr uint16_t kErrSendFailed = 7;
constexpr uint16_t kErrStateStale = 8;
constexpr uint16_t kErrDisconnected = 9;
constexpr uint16_t kErrDevice = 10;

constexpr uint64_t err_bit(uint16_t code) { return uint64_t{1} << code; }
// 오류 코드 -> 대응 (계약 v0.3 "오류별 B 대응" + 팀 결정). 바꿀 곳은 여기 한 곳
constexpr uint64_t kDecelErrorMask =
    err_bit(kErrOutOfRange) | err_bit(kErrIkFailed) | err_bit(kErrStateStale) | err_bit(kErrDevice);
constexpr uint64_t kFaultErrorMask =
    err_bit(kErrFormat) | err_bit(kErrUnsupported) | err_bit(kErrTf) | err_bit(kErrDisconnected);
// C가 B의 목표 자체를 거부한 오류 (DECEL_STOP 뒤 그 목표를 다시 보내지 않게 기억할 대상)
constexpr uint64_t kTargetRejectErrorMask = err_bit(kErrOutOfRange) | err_bit(kErrIkFailed);

// E_EXPIRED 시각을 기억해 둘 최대 개수 (c_expired_n의 상한)
constexpr int kMaxExpiredCount = 64;

struct SafetyConfig {
  int range_reject_n = 5;                     // [제안]
  int resume_k = 3;                           // [제안]
  int64_t ai_decel_after_ns = 0;              // [팀 결정] 0 = HOLD가 길어도 감속 정지로 올리지 않음
  bool require_c_state = true;                // false면 C 관련 규칙을 모두 끔 (C 없이 개발할 때)
  int64_t c_state_timeout_ns = 20'000'000;    // [제안] 20ms
  int c_expired_n = 10;                       // [제안]
  int64_t c_expired_window_ns = 1'000'000'000;  // 1초
  int64_t resume_delay_ns = 1'000'000'000;    // [제안] 1초
  double stopped_speed = 1e-9;                // limiter 속도가 이 값 이하면 "멈춤" [m/s]
};

// 매 주기 입력
struct SafetyInput {
  int64_t now_ns;
  // AI (gate·blend 결과)
  bool gate_pass;
  uint32_t gate_reasons;     // GateReason 비트
  bool is_new_command;       // blend가 판단한 "이번 주기에 새 명령이 들어왔는지"
  // C
  bool c_has_state;
  int64_t c_state_age_ns;
  uint8_t hal_state;         // HalStateCode
  uint64_t c_error_mask;     // HalState.active_errors를 비트마스크로 (bit n = 코드 n)
  // 기타
  bool reset_request;        // FAULT 해제 요청 (한 번만 true)
  double limiter_speed;      // 지난 주기 limiter 속도 [m/s]
};

// 해제 요청 처리 결과
enum class ResetResult : uint8_t {
  kNone = 0,        // 요청 없음
  kAccepted = 1,    // FAULT 해제됨
  kNotFault = 2,    // FAULT가 아니라 할 일 없음
  kCauseActive = 3, // FAULT 원인이 아직 남아 있음
  kMoving = 4,      // 아직 멈추지 않음
};

struct SafetyOutput {
  SafetyMode mode;
  SafetyMode prev_mode;      // changed일 때 바뀌기 전 모드
  bool changed;              // 이번 주기에 모드가 바뀜 (이벤트)
  bool allow_blend;          // blend가 새 AI 명령을 반영해도 되는지 (RUN일 때만)
  bool stop;                 // limiter 감속 정지 요청 (DECEL_STOP·FAULT)
  uint32_t reasons;          // 지금 모드에 들어온 이유 (래치가 걸려 있는 동안 생긴 이유를 모두 OR)
  uint32_t active_reasons;   // 이번 주기에 켜져 있는 모든 조건
  int64_t mode_duration_ns;  // 지금 모드가 이어진 시간
  bool reset_limiter;        // DECEL_STOP·FAULT에서 풀려 움직이기 시작: limiter를 실제 로봇 위치로 reset할 것
  bool new_session;          // FAULT 해제: 새 session_id를 발급할 것
  ResetResult reset_result;  // 이번 주기 해제 요청 처리 결과
  uint64_t c_error_mask;     // 판단에 쓴 켜진 오류 (HalState + /hal/error 이벤트)
};

class SafetyStateMachine {
 public:
  SafetyStateMachine() : SafetyStateMachine(SafetyConfig{}) {}
  explicit SafetyStateMachine(const SafetyConfig& config);

  // RUN에서 새로 시작 (노드가 INIT -> RUN으로 바꿀 때 부름). 래치·세는 값을 모두 비움
  void reset(int64_t now_ns);

  // B의 지금 세션 (노드가 세션을 발급·바꿀 때 부름). 세션이 바뀌면 이벤트로 켜진 오류·E_EXPIRED 기록을 비움
  // (옛 세션의 "꺼짐" 이벤트는 아래에서 무시되므로, 남겨 두면 풀리지 않음. 지금 오류는 HalState가 알려 줌)
  void set_session(uint64_t session_id);
  uint64_t session() const { return session_id_; }

  // /hal/error 이벤트 한 개 (update 전에, 받은 만큼 부름). 켜진 오류 목록과 E_EXPIRED 횟수에 씀.
  // session_id가 지금 세션과 다르면 무시하고 개수만 셈 (돌려주는 값 false)
  bool on_error_event(uint16_t code, bool active, int64_t now_ns, uint64_t session_id);
  uint64_t ignored_error_events() const { return ignored_error_events_; }

  SafetyOutput update(const SafetyInput& in);

  SafetyMode mode() const { return mode_; }
  const SafetyConfig& config() const { return config_; }

 private:
  uint32_t hold_conditions(const SafetyInput& in, uint64_t errors);
  uint32_t decel_conditions(const SafetyInput& in, uint64_t errors) const;
  uint32_t fault_conditions(const SafetyInput& in, uint64_t errors) const;

  SafetyConfig config_;
  SafetyMode mode_ = SafetyMode::kRun;
  int64_t mode_since_ns_ = 0;

  // HOLD
  bool hold_latched_ = false;
  uint32_t hold_reasons_ = 0;
  int64_t hold_since_ns_ = 0;
  int resume_count_ = 0;       // HOLD 래치 중 새 명령 연속 통과 수
  int range_reject_count_ = 0; // 새 명령 RANGE 연속 차단 수
  // DECEL_STOP
  bool decel_latched_ = false;
  uint32_t decel_reasons_ = 0;
  bool decel_clear_ = false;   // 조건이 없는 상태인지
  int64_t decel_clear_since_ns_ = 0;
  // FAULT
  bool fault_latched_ = false;
  uint32_t fault_reasons_ = 0;
  // C 오류
  uint64_t session_id_ = 0;                // B의 지금 세션 (HalError 세션 확인용)
  uint64_t ignored_error_events_ = 0;      // 다른 세션이라 무시한 /hal/error 이벤트 수
  uint64_t event_error_mask_ = 0;          // /hal/error 이벤트로 켜진 오류
  int64_t expired_times_[kMaxExpiredCount] = {};  // 최근 E_EXPIRED(켜짐) 받은 시각 (원형)
  int expired_head_ = 0;
  int expired_total_ = 0;      // 지금까지 받은 수 (kMaxExpiredCount에서 멈춤)
};

}  // namespace rt_control
