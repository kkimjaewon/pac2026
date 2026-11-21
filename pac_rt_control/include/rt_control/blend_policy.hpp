// 시나리오 1 blend_policy: gate를 통과한 AI 명령을 목표 위치에 반영한다 (2·3단계)
//
// [확정, 계약 미확정 #4] 새 명령이 gate를 통과했을 때만 "한 번" 반영한다. 1차 필터 없음.
// 부드럽게 움직이는 건 그 뒤의 motion_limiter 몫이다.
//
//   신선도 = clamp(1 - 현재 나이 / ttl, 0, 1)
//   w      = clamp(confidence, 0, 1) × 신선도 × 새 명령(0/1) × gate 통과(0/1)
//   목표   = 목표 + w × (AI 위치 + target_offset - 목표)
//
// target_offset: A가 보내는 위치(철판 표면)를 TCP 작업 위치로 옮기는 값 (map 기준 [m]).
// gate는 오프셋을 더하기 전 원래 AI 위치로 검사하고, 오프셋은 여기(blend 입력)에서만 더한다.
// 시작 목표(initial_goal)·HOLD 중 유지 목표에는 더하지 않는다.
//
// deadband_m: 반영하면 목표가 움직이는 거리 |새 목표 - 지금 목표|가 이보다 작으면 목표를 바꾸지 않는다
//   (도착 뒤 같은 곳을 가리키는 AI 명령이 목표를 계속 미세하게 흔들지 않게. 0 = 끔).
//   이때는 "반영 안 함"(w = 0, applied false)으로 보고, 명령은 "봤다"고만 표시한다.
//
// 거부된 목표 대기 (rejected_goal_tol_m): C가 목표를 거부해(E_OUT_OF_RANGE·E_IK_FAILED) DECEL_STOP으로
//   멈추면 노드가 reject_last_applied()로 마지막으로 반영한 AI 목표(AI 위치 + target_offset)를 기억시킨다.
//   그 뒤 새 AI 목표가 기억한 목표에서 rejected_goal_tol_m 안이면 반영하지 않는다 ("봤다"고만 표시).
//   tol 이상 다른 목표가 반영 후보가 되면 기억을 지우고 평소처럼 반영한다.
//
// 시계 호출·로그·메모리 할당이 없다. 계산은 순수 함수 blend_single()에 있고,
// BlendPolicy 클래스는 상태(지금 목표, 마지막으로 본 boot_id·seq)만 들고 있다.
// 입력·출력(BlendResult)은 그대로 두고 blend_mode로 내부 방식만 바꿀 수 있게 해 두었다.
#pragma once

#include <cstdint>

#include "rt_control/ai_command_snapshot.hpp"
#include "rt_control/gate.hpp"

namespace rt_control {

// [제안] blend 목표 불감대 [m]. 바꿀 곳은 여기 한 곳 + ROS 파라미터 blend_deadband
constexpr double kDefaultBlendDeadbandM = 0.0005;  // 0.5mm
// [제안] 거부된 목표와 이만큼 이상 달라야 다시 반영 [m]. 바꿀 곳은 여기 + ROS 파라미터 rejected_goal_tol
constexpr double kDefaultRejectedGoalTolM = 0.01;  // 1cm

// 반영 방식. 지금은 single(방법 B)만 있음. 나중에 median3(최근 3개 중앙값) 추가 예정
enum class BlendMode : uint8_t {
  kSingle = 0,
};

// 이름 -> BlendMode. 모르는 이름이면 false (out은 그대로)
bool parse_blend_mode(const char* name, BlendMode* out);

struct BlendConfig {
  BlendMode mode = BlendMode::kSingle;
  int64_t ttl_ns = kDefaultTtlNs;  // gate와 같은 값을 쓸 것
  double initial_goal[3] = {kDefaultPlannedPosition[0], kDefaultPlannedPosition[1],
                            kDefaultPlannedPosition[2]};  // 시작 목표 (기본: planned_position)
  double target_offset[3] = {0.0, 0.0, 0.0};  // AI 위치에 더하는 값 [m]. NaN·무한대는 0으로 바꿔 씀
  double deadband_m = kDefaultBlendDeadbandM;  // 목표 변화가 이보다 작으면 무시 [m]. 0 = 끔
  double rejected_goal_tol_m = kDefaultRejectedGoalTolM;  // 거부된 목표와 이보다 가까우면 반영 안 함 [m]
};

// 오프셋에서 NaN·무한대인 성분을 0으로 바꾼다. 바꾼 성분이 있으면 true (설정할 때 경고용)
bool sanitize_offset(double offset[3]);

struct BlendResult {
  double goal[3];         // 새 목표 (map 기준 [m])
  double w;               // 이번 주기에 쓴 가중치 (반영 안 했으면 0)
  bool applied;           // 이번 주기에 반영했는지
  uint64_t used_ai_seq;   // 마지막으로 반영한 AI seq (아직 없으면 0)
  bool has_used;          // 한 번이라도 반영했는지. seq는 0부터라 used_ai_seq == 0만으로는 구분이 안 됨
  uint32_t used_ai_boot_id;  // 마지막으로 반영한 AI 명령의 writer_boot_id (has_used가 false면 0)
  bool deadband_skipped;  // 이번 주기에 반영할 명령이었지만 변화가 deadband보다 작아 무시했는지
  bool rejected_wait;     // 거부된 목표 대기 중 (이번 주기 처리 뒤 상태)
  bool rejected_skipped;  // 이번 주기에 새 명령이 거부된 목표와 가까워 반영하지 않았는지
};

// 한 번 반영하는 계산 (순수 함수). goal_inout을 고친다.
//   is_new_and_pass: 새 명령이면서 gate를 통과했는지
//   offset: AI 위치에 더할 값 (nullptr = 0). 유한한 값이어야 함 (sanitize_offset을 거친 값)
// 돌려주는 값 = 이번에 쓴 w (반영 안 했으면 0)
double blend_single(double goal_inout[3], const AiCommandSnapshot& cmd, bool is_new_and_pass,
                    int64_t age_ns, int64_t ttl_ns, const double* offset = nullptr);

class BlendPolicy {
 public:
  BlendPolicy() : BlendPolicy(BlendConfig{}) {}
  explicit BlendPolicy(const BlendConfig& config);

  // cmd가 처음 보는 명령(writer_boot_id·seq가 마지막으로 본 것과 다름)인지. 상태를 바꾸지 않는다.
  // safety_mode가 blend보다 먼저 "새 명령 여부"를 알아야 해서 따로 둔다.
  bool is_new(const AiCommandSnapshot& cmd, const GateResult& gate) const;

  // 매 주기 호출. gate.reasons에 NO_DATA가 있으면 아직 받은 명령이 없는 것으로 본다.
  //   allow: safety_mode가 반영을 허용하는지 (RUN일 때만 true). false면 목표를 그대로 두지만
  //          새 명령은 "봤다"고 표시한다 (HOLD 중에 막힌 명령이 나중에 다시 반영되지 않게)
  BlendResult update(const AiCommandSnapshot& cmd, const GateResult& gate, bool allow = true);

  // 지금 목표 (map [m])
  const double* goal() const { return goal_; }

  // 목표를 pos로 바꿈 (DECEL_STOP에서 재개할 때 측정 TCP로 맞춤). 본 명령·반영 기록은 그대로
  void reset_goal(const double pos[3]);

  // 마지막으로 반영한 AI 목표(AI 위치 + target_offset)를 "거부된 목표"로 기억. 반영한 적 없으면 false
  bool reject_last_applied();
  bool rejected_wait() const { return rejected_active_; }
  const double* rejected_target() const { return rejected_target_; }

 private:
  BlendConfig config_;
  double goal_[3];
  bool has_seen_ = false;      // 명령을 한 번이라도 봤는지
  uint32_t last_boot_id_ = 0;  // 마지막으로 본 명령 (gate 차단이어도 기록)
  uint64_t last_seq_ = 0;
  uint64_t used_ai_seq_ = 0;   // 마지막으로 반영한 명령
  uint32_t used_ai_boot_id_ = 0;
  bool has_used_ = false;
  double last_applied_target_[3] = {0.0, 0.0, 0.0};  // 마지막으로 반영한 AI 위치 + offset
  bool rejected_active_ = false;
  double rejected_target_[3] = {0.0, 0.0, 0.0};
};

}  // namespace rt_control
