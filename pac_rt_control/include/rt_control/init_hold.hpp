// INIT 중 "유지 명령" 발행 (첫 발행 비용을 warmup 안에서 치르기)
//
// 원인 (2026-10-08 측정, rmw_fastrtps_cpp): B가 켜진 지 약 100ms 만에 RUN으로 가면 첫 publish()가
// DDS 안에서 잠금에 막혀 230ms 걸렸다 (page fault 0, 자발적 컨텍스트 스위치 1). 그동안 발행이 멈춰
// overrun·건너뛴 seq가 수백 개 몰리고 첫 명령이 C에서 E_EXPIRED가 났다. 메모리 할당·로그 출력은 아님.
//
// 그래서 INIT에서 신선한 HalState를 받으면 그 측정 TCP 위치·자세를 그대로 "유지 명령"으로
// 정상 주기(1ms)마다 발행한다. 첫 전송 비용은 RUN 전에 치르고, warmup은 유지 명령이 overrun 없이
// 이어져야 끝난다 (warmup.hpp WARM_HOLD).
//   - 신선한 HalState = 받은 적 있음, pose_valid, stale 아님, 나이 < state_timeout, 위치 유한, 자세 정규화 가능
//   - 신선한 HalState 전에는 아무것도 발행하지 않음. INIT 중 HalState가 끊기면 발행을 멈추고 다시 기다림
//   - INIT 중 AI 목표는 쓰지 않음 (이 클래스는 AI 입력을 받지 않고, source_result_id는 0)
//   - 세션: 설정 session_id가 0이 아니면 그 값, 0이면 첫 신선한 HalState 시점(CLOCK_MONOTONIC ns)에 발급.
//     이 세션을 RUN에서도 그대로 씀 (command_seq도 이어짐)
//
// 시계 호출·로그·메모리 할당이 없다 (1ms 루프에서 부름).
#pragma once

#include <cstdint>

#include "rt_control/hal_command_snapshot.hpp"
#include "rt_control/hal_state_snapshot.hpp"

namespace rt_control {

// 세션 번호 고르기: 설정값(0이 아니면) > 이미 정한 값 > 지금 시각
inline uint64_t choose_session_id(uint64_t registered, uint64_t current, int64_t now_ns) {
  return registered != 0 ? registered : current != 0 ? current : static_cast<uint64_t>(now_ns);
}

// 신선한 HalState면 true (pos, quat에 측정 TCP 위치·정규화한 자세)
bool fresh_pose(bool has_state, const HalStateSnapshot& s, int64_t now_ns, int64_t state_timeout_ns, double pos[3],
                double quat[4]);

struct InitHoldConfig {
  int64_t state_timeout_ns = 20'000'000;  // safety의 c_state_timeout과 같은 값
  uint64_t registered_session_id = 0;     // 파라미터 session_id (0 = B가 발급)
};

enum class InitHoldEvent : uint8_t {
  kNone = 0,
  kStarted = 1,  // 유지 명령 발행 시작
  kLost = 2,     // HalState가 끊겨 발행 멈춤
};

struct InitHoldResult {
  bool publish;         // 이번 주기에 cmd를 채웠음 (버퍼에 쓸 것)
  bool session_issued;  // 이번 주기에 세션을 정함 (session_id가 0에서 바뀜)
  InitHoldEvent event;
};

class InitHold {
 public:
  InitHold() = default;
  explicit InitHold(const InitHoldConfig& config) : config_(config) {}

  // INIT 매 주기. 신선한 HalState면 session_id를 정하고(아직 0이면) cmd에 유지 명령을 채운다.
  //   session_id: 지금 세션 (0 = 아직 없음). 정하면 바꿔 줌
  //   cmd: 지난 주기 명령 (command_seq를 이어 셈). 세션을 새로 정하면 처음부터
  //   base: TTL 등 (자세는 측정 자세로 바꿔 씀)
  InitHoldResult step(bool has_state, const HalStateSnapshot& state, int64_t now_ns, uint64_t& session_id,
                      HalCommandSnapshot& cmd, const HalCommandConfig& base);

  bool holding() const { return holding_; }
  // 마지막으로 발행한 유지 명령의 위치·자세 (RUN 시작 위치)
  const double* hold_pos() const { return hold_pos_; }
  const double* hold_quat() const { return hold_quat_; }
  uint64_t published() const { return published_; }
  const InitHoldConfig& config() const { return config_; }

 private:
  InitHoldConfig config_;
  bool holding_ = false;
  double hold_pos_[3] = {0.0, 0.0, 0.0};
  double hold_quat_[4] = {0.0, 0.0, 0.0, 1.0};
  uint64_t published_ = 0;
};

}  // namespace rt_control
