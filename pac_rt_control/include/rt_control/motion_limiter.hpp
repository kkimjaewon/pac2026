// motion_limiter: blend가 정한 목표(계단처럼 바뀜)를 1ms마다 속도·가속도 한계 안에서
// 부드럽게 따라가는 출력 위치로 바꾼다. 목표 근처에서는 미리 감속해서 지나치지 않는다.
//
// 축별이 아니라 벡터 하나로 제한한다 (대각선 이동이 곧게 가도록).
//   e      = 목표 - 지금 위치,  d = |e|
//   원하는 속도 = (e/d) × min(v_max, 감속 가능 속도(d), d/dt)  (stop이면 0)
//   v      = v_prev + clamp_norm(원하는 속도 - v_prev, a_max × dt)
//   위치  += v × dt
//
// 도착 처리: 원하는 속도의 상한 d/dt는 "이번 걸음으로 딱 목표까지"를 뜻한다.
//   그 속도를 가속도 한계 안에서 낼 수 있으면 이번 주기에 목표에 정확히 맞추고(지나침 없음),
//   다음 주기에 속도를 0으로 만든다. 이때 줄이는 속도는 항상 a_max × dt 이하라 가속도 한계를 지킨다.
//   (같은 주기에 위치 맞춤 + 속도 0을 하면, 위치 차분으로 잰 가속도가 한계를 넘는다)
//   빠르게 가는 중에 목표가 바로 앞(이번 걸음보다 가까이)으로 옮겨 오면 그 자리에서 멈출 수 없다.
//   이때는 가속도 한계를 우선해서, 한계 안에서 감속하며 지나쳤다가 돌아온다.
//
// 도착 후 고정: 목표에 도착하고(위치 = 목표) 속도가 0이 되면 hold에 들어간다.
//   hold 중에는 목표가 바뀌지 않는 한 계산을 건너뛰고 출력 위치를 비트 단위로 그대로 낸다
//   (매 주기 dt가 조금씩 달라도 위치가 미세하게 달라지지 않게). 목표가 바뀌면(== 비교) 바로 풀리고
//   지금 위치·속도 0에서 보통 계산을 다시 한다. 위치를 갑자기 맞추거나 속도를 강제로 0으로 만들지 않는다:
//   hold는 이미 속도 0으로 도착한 뒤에만 켜진다.
//
// 감속 가능 속도: 연속 시간 식 sqrt(2 × a_max × d)를 1ms 계단에 맞게 고친 값.
//   이번 주기에 그 속도로 움직이고 1ms마다 a_max×dt씩 줄여서 d 안에 멈출 수 있는 가장 큰 속도.
//   (정확한 식은 motion_limiter.cpp의 stoppable_speed 참고)
//   d가 크면 sqrt(2 × a_max × d) - a_max×dt/2 와 거의 같다.
//   sqrt(2ad)를 그대로 쓰면 마지막 몇 주기에 목표를 살짝 지나치고 가속도 한계가 깨진다.
//
// 시계 호출·로그·메모리 할당이 없다. 계산은 limiter_step() 순수 함수(상태를 인자로 받음),
// MotionLimiter 클래스는 상태만 들고 있다.
//
// 저크 제한 (j_max > 0, 기본 1.0 m/s³): 가속도가 한 주기에 j_max × dt 이하로만 바뀐다 (S자 출발·정지).
//   정지에서 출발할 때 가속도가 0 -> a_max로 한 번에 뛰지 않는다 (C가 관절 가속도 계단으로 거부, full_test_05).
//   목표 근처 감속도 저크·가속도 한계 안에서 미리 시작해 지나치지 않는다. DECEL_STOP 감속도 같은 한계.
//   계산은 motion_limiter.cpp의 step_jerk 설명 참고. j_max = 0이면 위의 예전 계산을 그대로 쓴다 (비트 단위로 같음).
#pragma once

namespace rt_control {

// [제안] 저크 한계 [m/s³]. 바꿀 곳은 여기 한 곳 + ROS 파라미터 j_max
constexpr double kDefaultJMax = 1.0;

struct LimiterConfig {
  double v_max = 0.10;  // [m/s]  [미정] Franka 안전 범위 안에서 조정
  double a_max = 0.50;  // [m/s²] [미정]
  double j_max = kDefaultJMax;  // [m/s³] 저크 한계. 0 = 끔 (예전 계산)
};

// limiter 상태 (매 주기 갱신)
struct LimiterState {
  double pos[3];   // 지금 출력 위치 (map 기준 [m])
  double vel[3];   // 지금 속도 [m/s]
  double acc[3];   // 지금 가속도 [m/s²] (저크 제한에 씀. j_max = 0이면 기록용)
  double goal[3];  // 마지막으로 받은 유한한 목표 (NaN·무한대 목표가 오면 이 값을 계속 씀)
  bool hold;       // 도착 후 고정 중 (목표가 바뀔 때까지 pos를 그대로 냄)
};

struct LimiterResult {
  double pos[3];        // 이번 주기 출력 위치
  double vel[3];        // 이번 주기 속도 벡터
  double acc[3];        // 이번 주기 가속도 벡터
  double speed;         // |vel|
  bool speed_limited;   // 원하는 속도가 v_max에 잘렸는지
  bool accel_limited;   // 속도 변화량이 a_max × dt에 잘렸는지
  bool arrived;         // 목표에 도착(위치 = 목표, 속도 0)
  bool hold;            // 도착 후 고정 중 (이번 주기 출력 = 지난 주기 출력, 비트 단위로 같음)
};

// 한 주기 계산 (순수 함수). state를 고친다.
//   goal: blend 출력 목표. 하나라도 NaN·무한대면 state.goal(이전 목표)을 쓴다
//   stop: true면 목표를 무시하고 a_max로 감속해서 그 자리에 멈춘다 (나중에 safety_mode가 씀)
LimiterResult limiter_step(LimiterState& state, const double goal[3], double dt, bool stop,
                           const LimiterConfig& config);

class MotionLimiter {
 public:
  MotionLimiter() = default;
  explicit MotionLimiter(const LimiterConfig& config) : config_(config) {}

  // 출력 위치를 start에 두고 속도 0, 목표도 start로 시작.
  // 지금은 ROS 파라미터 start_position(기본 planned_position)으로 부른다.
  // TODO(C 연결 후): HalState의 실제 로봇 TCP 위치로 reset 할 것 (로봇이 있는 곳에서 출발해야 튀지 않음)
  void reset(const double start[3]);

  LimiterResult update(const double goal[3], double dt, bool stop) {
    return limiter_step(state_, goal, dt, stop, config_);
  }

  const LimiterState& state() const { return state_; }

 private:
  LimiterConfig config_;
  LimiterState state_{};
};

}  // namespace rt_control
