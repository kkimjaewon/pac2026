// 도착 판정과 재개 기준 (C와 합의: 2mm · 0.3초)
//
// 도착 판정 (ArrivalDetector):
//   tcp_error = |목표(blend 출력) - 측정 TCP(HalState)|
//   tcp_error <= arrive_tol 인 상태가 arrive_hold 이상 이어지면 "도착".
//   측정 TCP가 없거나 믿을 수 없으면(tcp_valid false) 오차는 NaN, 타이머는 처음부터 다시.
//
// 재개 기준 (choose_resume_position):
//   DECEL_STOP·FAULT -> RUN으로 다시 움직일 때 limiter를 어디서 시작할지 정한다.
//   resume_from_last_cmd가 켜져 있고, 마지막으로 발행한 HalCommand 위치와 측정 TCP의 차이가
//   resume_match_tol 이하이면 마지막 발행 위치에서 시작 (C가 받은 명령이 이어지게).
//   아니면 예전처럼 측정 TCP에서 시작하고, 차이가 커서 그랬으면 경고할 수 있게 알려 준다.
//   측정 TCP가 없으면 지금 limiter 출력 위치에서 시작 (예전과 같음).
//
// 시계 호출·로그·메모리 할당이 없다 (1ms 루프에서 부름).
#pragma once

#include <cstdint>

namespace rt_control {

// [제안·C와 합의] 바꿀 곳은 여기 한 곳 + ROS 파라미터 arrive_tol, arrive_hold_s, resume_match_tol
constexpr double kDefaultArriveTolM = 0.002;           // 2mm
constexpr int64_t kDefaultArriveHoldNs = 300'000'000;  // 0.3초
constexpr double kDefaultResumeMatchTolM = 0.002;      // 2mm (C 답 대기)

struct ArrivalConfig {
  double tol_m = kDefaultArriveTolM;
  int64_t hold_ns = kDefaultArriveHoldNs;
};

struct ArrivalResult {
  bool arrived;
  double error_m;  // |목표 - 측정 TCP| [m]. 측정이 없으면 NaN
};

class ArrivalDetector {
 public:
  ArrivalDetector() = default;
  explicit ArrivalDetector(const ArrivalConfig& config) : config_(config) {}

  // 매 주기 호출. now_ns는 단조 증가하는 시각 (CLOCK_MONOTONIC)
  ArrivalResult update(const double goal[3], const double tcp[3], bool tcp_valid, int64_t now_ns);

  // 타이머를 지움 (다음 update부터 처음부터 셈)
  void reset() { inside_ = false; }

  const ArrivalConfig& config() const { return config_; }

 private:
  ArrivalConfig config_;
  bool inside_ = false;      // 지난 주기에 허용 오차 안이었는지
  int64_t since_ns_ = 0;     // 허용 오차 안에 들어온 시각
};

// 재개 시작 위치를 어디서 가져왔는지
enum class ResumeSource : uint8_t {
  kOutput = 0,          // 측정 TCP 없음 -> 지금 limiter 출력 위치
  kRobot = 1,           // 측정 TCP (resume_from_last_cmd 꺼짐 또는 발행한 명령 없음)
  kLastCommand = 2,     // 마지막 발행 HalCommand 위치 (측정 TCP와 차이가 허용 오차 이하)
  kRobotMismatch = 3,   // 측정 TCP (마지막 발행 위치와 차이가 허용 오차보다 커서. 경고 대상)
};

struct ResumeChoice {
  double pos[3];
  ResumeSource source;
  double diff_m;  // |마지막 발행 위치 - 측정 TCP| [m]. 비교하지 않았으면 NaN
};

// 순수 함수. robot_valid가 false면 robot은 읽지 않고, has_last_cmd가 false면 last_cmd는 읽지 않는다
ResumeChoice choose_resume_position(bool resume_from_last_cmd, double match_tol_m, bool has_last_cmd,
                                    const double last_cmd[3], bool robot_valid, const double robot[3],
                                    const double output[3]);

}  // namespace rt_control
