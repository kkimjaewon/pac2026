// C -> B 상태·오류를 B 안에서 들고 다니는 형식 (ROS 의존 없음)
//
// ros_bridge가 /hal/state(HalState)를 받아 HalStateSnapshot으로 바꿔 Seqlock 버퍼에 쓰고,
// 1ms 루프가 매 주기 읽는다. /hal/error(HalError)는 HalErrorEvent로 바꿔 링버퍼에 넣는다.
#pragma once

#include <cstdint>

namespace rt_control {

// HalState.active_errors를 담을 최대 개수 (넘치면 뒤쪽은 버리고 개수만 셈)
constexpr int kMaxActiveErrors = 16;

// HalState.hal_state 값 (intelion_interfaces/msg/HalState.msg와 같은 번호)
enum HalStateCode : uint8_t {
  HAL_INITIALIZING = 0,
  HAL_CONNECTING = 1,
  HAL_READY = 2,
  HAL_ACTIVE = 3,
  HAL_FAULT = 4,
  HAL_RECONNECTING = 5,
};

inline const char* hal_state_name(uint8_t s) {
  switch (s) {
    case HAL_INITIALIZING: return "INITIALIZING";
    case HAL_CONNECTING: return "CONNECTING";
    case HAL_READY: return "READY";
    case HAL_ACTIVE: return "ACTIVE";
    case HAL_FAULT: return "FAULT";
    case HAL_RECONNECTING: return "RECONNECTING";
    default: return "?";
  }
}

// Seqlock 버퍼에 들어가는 HalState 한 개 (필요한 필드 + B가 받은 시각)
struct HalStateSnapshot {
  int64_t recv_mono_ns;         // B가 받은 시각 (CLOCK_MONOTONIC). 상태 나이 = now - recv_mono_ns
  int64_t received_mono_ns;     // C가 로봇 상태를 받은 시각 (메시지 값)
  uint64_t state_seq;
  uint64_t active_session_id;   // C가 지금 따르는 B 세션
  uint64_t last_command_seq;    // C가 마지막으로 처리한 command_seq
  double tcp_pos[3];            // map 기준 [m]
  double tcp_quat[4];           // (x, y, z, w)
  uint16_t active_errors[kMaxActiveErrors];
  uint8_t active_error_count;   // active_errors에 담긴 개수 (kMaxActiveErrors 이하)
  uint8_t hal_state;            // HalStateCode
  uint8_t last_command_result;  // 0 ACCEPTED, 1 REJECTED, 2 SENT
  bool stale;
  bool pose_valid;
};

// /hal/error 이벤트 한 개
struct HalErrorEvent {
  int64_t event_mono_ns;
  int64_t recv_mono_ns;  // B가 받은 시각
  uint64_t session_id;
  uint64_t command_seq;
  uint16_t code;
  bool active;           // true = 켜짐, false = 꺼짐
};

}  // namespace rt_control
