#include "rt_control/motion_limiter.hpp"

#include <cmath>

namespace rt_control {

namespace {

constexpr double kEps = 1e-12;  // 0으로 나누기 방지용 아주 작은 거리·속도

inline double norm3(const double v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// 이번 주기에 v로 움직인 뒤 1ms마다 a×dt씩 줄여서 d 안에 멈출 수 있는 가장 큰 속도 (헤더 설명 참고)
//   단위: 속도 u = a×dt, 거리 D = d / (a×dt²). 속도 k×u로 시작하면 k, k-1, ..., (1 이하) 다음에 0.
//   m = ceil(k) - 1 이면 걸리는 거리 T(k) = (m+1)k - m(m+1)/2  (k에 대해 구간별 직선, 증가)
//   T(k) <= D 를 만족하는 가장 큰 k: m = floor((-1 + sqrt(1+8D))/2), k = (D + m(m+1)/2) / (m+1)
inline double stoppable_speed(double d, double a, double dt) {
  const double u = a * dt;
  const double D = d / (u * dt);
  const double m = std::floor(0.5 * (std::sqrt(1.0 + 8.0 * D) - 1.0));
  const double k = (D + 0.5 * m * (m + 1.0)) / (m + 1.0);
  return u * k;
}

// 저크 제한 없음 (j_max = 0): 예전 계산 그대로. s.goal은 이미 정해져 있음
LimiterResult step_accel(LimiterState& s, double dt, bool stop, const LimiterConfig& c) {
  // 남은 벡터와 거리
  double e[3];
  for (int i = 0; i < 3; ++i) e[i] = s.goal[i] - s.pos[i];
  const double d = norm3(e);

  // 원하는 속도: 방향은 e, 크기는 min(v_max, 감속 가능 속도, 이번 걸음으로 딱 닿는 속도). stop이면 0
  const double profile = stoppable_speed(d, c.a_max, dt);
  const double reach = d / dt;
  const double cap = std::fmin(c.v_max, profile);
  const double want_speed = std::fmin(cap, reach) * static_cast<double>(!stop);
  const double inv_d = 1.0 / std::fmax(d, kEps);
  double dv[3];
  for (int i = 0; i < 3; ++i) dv[i] = e[i] * inv_d * want_speed - s.vel[i];


  // 속도 변화량 벡터의 크기를 a_max × dt 이하로 자름 (방향이 바뀌어도 한계 유지)
  const double dv_max = c.a_max * dt;
  const double dv_norm = norm3(dv);
  const bool accel_limited = dv_norm > dv_max;
  const double dv_scale = std::fmin(1.0, dv_max / std::fmax(dv_norm, kEps));
  double v[3];
  for (int i = 0; i < 3; ++i) v[i] = s.vel[i] + dv[i] * dv_scale;

  // 이번 걸음으로 목표에 닿음 -> 위치를 목표에 정확히 맞춤 (속도는 그대로, 다음 주기에 0이 됨)
  //   1) 원하는 속도가 d/dt였고 가속도 한계에 안 걸렸을 때 (v × dt = e)
  //   2) 걸음 길이가 남은 거리와 사실상 같은데(반올림 오차 수준) 목표를 아주 조금 넘을 때
  // 빠르게 가는 중에 목표가 바로 앞으로 옮겨 온 경우(걸음 >> 남은 거리)에는 맞추지 않는다.
  // 거기서 멈추면 위치가 급정거해서 가속도 한계가 깨지므로, 한계 안에서 지나쳤다가 돌아온다.
  const double speed = norm3(v);
  const double step_toward = (v[0] * e[0] + v[1] * e[1] + v[2] * e[2]) * dt;
  const bool fp_overshoot = (step_toward >= d * d) & (speed * dt <= d * (1.0 + 1e-9) + 1e-15);
  const bool land = (((reach <= cap) & !accel_limited) | fp_overshoot) & !stop;

  for (int i = 0; i < 3; ++i) {
    const double moved = s.pos[i] + v[i] * dt;
    s.pos[i] = land ? s.goal[i] : moved;
    s.acc[i] = (v[i] - s.vel[i]) / dt;  // 기록용 (이 방식의 계산에는 쓰지 않음)
    s.vel[i] = v[i];
  }

  LimiterResult r;
  for (int i = 0; i < 3; ++i) {
    r.pos[i] = s.pos[i];
    r.vel[i] = s.vel[i];
    r.acc[i] = s.acc[i];
  }
  r.speed = speed;
  r.speed_limited = (profile > c.v_max) & (reach > c.v_max) & !stop;
  r.accel_limited = accel_limited;
  r.arrived = (d == 0.0) & (speed == 0.0);
  // 위치 = 목표, 속도 0으로 도착했으면 다음 주기부터 고정 (stop 중 멈춘 경우는 목표와 달라서 안 켜짐)
  s.hold = r.arrived & (s.pos[0] == s.goal[0]) & (s.pos[1] == s.goal[1]) & (s.pos[2] == s.goal[2]);
  r.hold = false;
  return r;
}

// ---------------- 저크 제한 (j_max > 0) ----------------
//
// 이산 시간 모델 (가속도 -> 속도 -> 위치 순서로 갱신. 출력 위치 차분으로 잰 가속도·저크와 정확히 같음):
//   a' = a + j × dt,  v' = v + a' × dt,  p' = p + v' × dt
// 목표 방향 u 한 축으로 본 1차원 계획: 매 주기 a'를 [a - j_max×dt, a + j_max×dt] ∩ [-a_max, a_max] 안에서
// 아래 두 조건을 만족하는 가장 큰 값으로 고른다 (조건이 a'에 대해 단조라 고정 횟수 이분 탐색, 일정 실행시간).
//   1) 멈출 수 있음: 다음 상태에서 저크·가속도 한계로 감속해도 목표를 넘지 않음 (brake_distance <= 남은 거리)
//   2) 속도 한계: 지금부터 가속도를 저크 한계로 0까지 줄여도 v_max를 넘지 않음 (v' + a'|a'|/(2 j_max) <= v_max)
// 둘 다 만족하는 값이 없으면(목표가 갑자기 바로 앞으로 옮겨 옴) 가장 세게 감속하고, 지나쳤다가 돌아온다.
// 목표 방향과 수직인 속도는 저크 한계 안에서 0으로 줄인다 (같은 방식, 목표 속도 0).
// 끝: 목표 근처에서는 3주기 데드비트(아래 step_jerk)로 위치 = 목표, 속도·가속도 0에 정확히 닿는다.

constexpr int kBisectIters = 40;
// 위 조건 계산(연속 시간 식)과 1ms 계단 계산의 차이를 메우는 여유 (주기 수 단위)
constexpr double kBrakeMarginSteps = 2.0;

inline double dot3(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// 가속도 0, 속도 v(>=0)에서 저크 J·가속도 A 한계로 멈출 때까지 가는 거리 (연속 시간)
inline double brake_from_rest_acc(double v, double A, double J) {
  const double vp = std::fmax(v, 0.0);
  const double tri = vp * std::sqrt(vp / J);              // v <= A²/J : 가속도가 삼각형
  const double trap = 0.5 * vp * (vp / A + A / J);         // v >  A²/J : 사다리꼴
  return vp <= A * A / J ? tri : trap;
}

// 속도 v, 가속도 a(목표 방향 +)에서 저크·가속도 한계로 멈출 때까지 가는 거리 (연속 시간)
double brake_distance(double v, double a, double A, double J) {
  if (a > 0.0) {
    // 먼저 가속도를 0으로 줄임 (그동안 더 감)
    const double t0 = a / J;
    const double d0 = v * t0 + 0.5 * a * t0 * t0 - J * t0 * t0 * t0 / 6.0;
    return d0 + brake_from_rest_acc(v + a * a / (2.0 * J), A, J);
  }
  if (v <= 0.0) return 0.0;  // 멀어지는 중 (또는 정지)이고 더 멀어지게 가속하지 않음
  const double am = -a;      // 지금 감속도 (>= 0)
  if (am * am > 2.0 * J * v) {
    // 이미 필요보다 세게 감속 중: 가속도를 바로 +J로 올려도 가속도가 0이 되기 전에 속도가 0이 됨.
    // 속도가 처음 0이 되는 시각 t*까지 간 거리 (정확한 값. 어림값을 쓰면 "못 멈춘다"로 잘못 보고 더 세게 감속함)
    const double t = (am - std::sqrt(am * am - 2.0 * J * v)) / J;
    return v * t - 0.5 * am * t * t + J * t * t * t / 6.0;
  }
  // 가속도 0, 속도 vs에서 감속을 시작해 지금 상태(v, a)에 온 것으로 보고, 그 앞부분 거리를 뺌
  const double ts = am / J;
  const double vs = v + am * am / (2.0 * J);
  const double ds = vs * ts - J * ts * ts * ts / 6.0;
  return std::fmax(brake_from_rest_acc(vs, A, J) - ds, 0.0);
}

// 1차원: a_next 후보가 조건을 만족하는지 (d: 남은 거리, v·a: 목표 방향 성분)
inline bool feasible(double a1, double d, double v, double dt, const LimiterConfig& c) {
  const double v1 = v + a1 * dt;
  const double d1 = d - v1 * dt;
  const double margin = kBrakeMarginSteps * std::fabs(v1) * dt;
  const bool can_stop = brake_distance(v1, a1, c.a_max, c.j_max) + margin <= d1;
  const bool under_vmax = v1 + a1 * std::fabs(a1) / (2.0 * c.j_max) <= c.v_max;
  return can_stop & under_vmax;
}

// 1차원 속도를 0으로 (감속 정지·수직 성분용): 가속도를 0까지 올리는 동안 속도가 0 아래로 내려가지 않는
// 가장 작은 a_next. s: 속도(>=0), a: 가속도 (같은 방향 +)
inline double zero_speed_acc(double sp, double a, double dt, const LimiterConfig& c) {
  const double lo = std::fmax(a - c.j_max * dt, -c.a_max);
  const double hi = std::fmin(a + c.j_max * dt, c.a_max);
  // ok(a1): v1 + a1|a1|/(2J) >= 0 (a1에 대해 증가). 가장 작은 ok 값을 찾음
  auto ok = [&](double a1) {
    const double v1 = sp + a1 * dt;
    return v1 + a1 * std::fabs(a1) / (2.0 * c.j_max) >= 0.0;
  };
  if (ok(lo)) return lo;
  if (!ok(hi)) return hi;
  double l = lo, h = hi;  // ok(l) false, ok(h) true
  for (int k = 0; k < kBisectIters; ++k) {
    const double m = 0.5 * (l + h);
    (ok(m) ? h : l) = m;
  }
  return h;
}

LimiterResult step_jerk(LimiterState& s, double dt, bool stop, const LimiterConfig& c) {
  const double J = c.j_max;
  const double dj = J * dt;  // 한 주기 가속도 변화 한계

  double e[3];
  for (int i = 0; i < 3; ++i) e[i] = s.goal[i] - s.pos[i];
  const double d = norm3(e);
  const double speed0 = norm3(s.vel);
  const double accn0 = norm3(s.acc);

  // 끝맺음 (3주기 데드비트): 오차 p = pos - goal, 속도·가속도를 정확히 3주기 뒤 0으로 만드는 저크
  //   j1 = -(p + h v + h² a)/h³,  j2 = (2p + h v)/h³,  j3 = -p/h³   (h = dt, 축마다)
  // 세 저크가 모두 j_max 안이면 j1을 씀 (다음 주기에 다시 풀면 j2, j3, 0이 나옴). 남은 값이 아주 작으면 정확히 맞춤.
  // 이렇게 해야 목표 근처 1e-9m 크기에서 맴돌지 않고 끝남 (한 주기 저크 걸음 j_max·dt³ ≈ 1e-9m보다 잘게는 못 움직임)
  const double h = dt, h2 = dt * dt, h3 = dt * dt * dt;
  double j1[3], j2[3], j3[3];
  for (int i = 0; i < 3; ++i) {
    const double p = -e[i];
    j1[i] = -(p + h * s.vel[i] + h2 * s.acc[i]) / h3;
    j2[i] = (2.0 * p + h * s.vel[i]) / h3;
    j3[i] = -p / h3;
  }
  const bool deadbeat = !stop & (norm3(j1) <= J) & (norm3(j2) <= J) & (norm3(j3) <= J);
  const double snap = 1e-3;  // 이 비율 아래로 남으면 정확히 맞춤 (그때 생기는 저크 < 0.01 j_max)
  const bool finish = deadbeat & (d <= snap * J * h3) & (speed0 <= snap * J * h2) & (accn0 <= snap * dj);
  if (deadbeat) {
    for (int i = 0; i < 3; ++i) {
      const double a1 = s.acc[i] + h * j1[i];
      const double v1 = s.vel[i] + h * a1;
      s.pos[i] = finish ? s.goal[i] : s.pos[i] + h * v1;
      s.vel[i] = finish ? 0.0 : v1;
      s.acc[i] = finish ? 0.0 : a1;
    }
    LimiterResult r;
    for (int i = 0; i < 3; ++i) {
      r.pos[i] = s.pos[i];
      r.vel[i] = s.vel[i];
      r.acc[i] = s.acc[i];
    }
    r.speed = norm3(s.vel);
    r.speed_limited = false;
    r.accel_limited = false;
    r.arrived = finish;
    s.hold = finish;
    r.hold = false;
    return r;
  }
  // 감속 정지 끝: 속도·가속도가 충분히 작으면 0으로 (위치는 그대로)
  const bool stop_finish = stop & (speed0 <= 0.5 * J * dt * dt) & (accn0 <= 0.5 * dj);

  // 기준 방향 u: 목표 쪽. 목표에 있으면 속도 방향, 그것도 없으면 가속도 방향
  double u[3] = {0, 0, 0};
  const double* dir = d > 0.0 ? e : (speed0 > 0.0 ? s.vel : s.acc);
  const double dn = d > 0.0 ? d : (speed0 > 0.0 ? speed0 : accn0);
  if (dn > 0.0) {
    for (int i = 0; i < 3; ++i) u[i] = dir[i] / dn;
  }
  // stop이면 기준 방향 = 지금 속도 방향 (목표 무시)
  if (stop & (speed0 > 0.0)) {
    for (int i = 0; i < 3; ++i) u[i] = s.vel[i] / speed0;
  }

  const double vp = dot3(s.vel, u);
  const double ap = dot3(s.acc, u);
  const double lo = std::fmax(ap - dj, -c.a_max);
  const double hi = std::fmin(ap + dj, c.a_max);

  // u 방향 다음 가속도
  double ap1;
  bool speed_limited = false;
  if (stop) {
    ap1 = zero_speed_acc(std::fmax(vp, 0.0), ap, dt, c);
  } else {
    const double dist_u = d > 0.0 ? d : 0.0;
    if (!feasible(lo, dist_u, vp, dt, c)) {
      ap1 = lo;  // 멈출 수 없음: 가장 세게 감속 (지나쳤다가 돌아옴)
    } else if (feasible(hi, dist_u, vp, dt, c)) {
      ap1 = hi;
    } else {
      double l = lo, h = hi;  // feasible(l) true, feasible(h) false
      for (int k = 0; k < kBisectIters; ++k) {
        const double m = 0.5 * (l + h);
        (feasible(m, dist_u, vp, dt, c) ? l : h) = m;
      }
      ap1 = l;
      // v_max 조건 때문에 막혔는지 (기록용)
      const double v1 = vp + h * dt;
      speed_limited = v1 + h * std::fabs(h) / (2.0 * J) > c.v_max;
    }
  }

  // u에 수직인 속도: 저크 한계 안에서 0으로
  double vq[3], aq[3];
  for (int i = 0; i < 3; ++i) {
    vq[i] = s.vel[i] - vp * u[i];
    aq[i] = s.acc[i] - ap * u[i];
  }
  const double sq = norm3(vq);
  double w[3] = {0, 0, 0};
  if (sq > 0.0) {
    for (int i = 0; i < 3; ++i) w[i] = vq[i] / sq;
  }
  const double aw = dot3(aq, w);
  const double aw1 = sq > 0.0 ? zero_speed_acc(sq, aw, dt, c) : 0.0;
  // u·w 둘 다에 수직인 가속도 성분은 0으로 (아래 벡터 저크 제한이 한계 안으로 자름)

  double a_des[3];
  for (int i = 0; i < 3; ++i) a_des[i] = ap1 * u[i] + aw1 * w[i];

  // 벡터 저크 제한·가속도 제한 (직선 이동이면 위 1차원 계산이 이미 한계 안이라 그대로 통과)
  double da[3];
  for (int i = 0; i < 3; ++i) da[i] = a_des[i] - s.acc[i];
  const double da_n = norm3(da);
  const double da_scale = std::fmin(1.0, dj / std::fmax(da_n, kEps));
  double a1[3];
  for (int i = 0; i < 3; ++i) a1[i] = s.acc[i] + da[i] * da_scale;
  const double a1_n = norm3(a1);
  const double a_scale = std::fmin(1.0, c.a_max / std::fmax(a1_n, kEps));
  for (int i = 0; i < 3; ++i) a1[i] *= a_scale;
  const bool accel_limited = a1_n >= c.a_max * (1.0 - 1e-12);

  double v1[3];
  for (int i = 0; i < 3; ++i) v1[i] = s.vel[i] + a1[i] * dt;

  for (int i = 0; i < 3; ++i) {
    const double moved = s.pos[i] + v1[i] * dt;
    s.pos[i] = moved;
    s.vel[i] = stop_finish ? 0.0 : v1[i];
    s.acc[i] = stop_finish ? 0.0 : a1[i];
  }
  double e1[3];
  for (int i = 0; i < 3; ++i) e1[i] = s.goal[i] - s.pos[i];

  LimiterResult r;
  for (int i = 0; i < 3; ++i) {
    r.pos[i] = s.pos[i];
    r.vel[i] = s.vel[i];
    r.acc[i] = s.acc[i];
  }
  r.speed = norm3(s.vel);
  r.speed_limited = speed_limited;
  r.accel_limited = accel_limited;
  r.arrived = (norm3(e1) == 0.0) & (r.speed == 0.0);
  s.hold = r.arrived & (norm3(s.acc) == 0.0);
  r.hold = false;
  return r;
}

}  // namespace

LimiterResult limiter_step(LimiterState& s, const double goal_in[3], double dt, bool stop,
                           const LimiterConfig& c) {
  // 목표가 NaN·무한대면 이전 목표를 계속 씀 (세 축을 한 묶음으로 선택)
  const bool goal_ok =
      std::isfinite(goal_in[0]) & std::isfinite(goal_in[1]) & std::isfinite(goal_in[2]);
  const bool goal_same = (!goal_ok) | ((goal_in[0] == s.goal[0]) & (goal_in[1] == s.goal[1]) &
                                     (goal_in[2] == s.goal[2]));
  for (int i = 0; i < 3; ++i) s.goal[i] = goal_ok ? goal_in[i] : s.goal[i];

  // 도착 후 고정: 목표가 그대로면 계산하지 않고 지난 출력을 그대로 냄 (속도는 이미 0)
  if (s.hold & goal_same) {
    LimiterResult r;
    for (int i = 0; i < 3; ++i) {
      r.pos[i] = s.pos[i];
      r.vel[i] = 0.0;
      r.acc[i] = 0.0;
    }
    r.speed = 0.0;
    r.speed_limited = false;
    r.accel_limited = false;
    r.arrived = true;
    r.hold = true;
    return r;
  }
  s.hold = false;

  return c.j_max > 0.0 ? step_jerk(s, dt, stop, c) : step_accel(s, dt, stop, c);
}

void MotionLimiter::reset(const double start[3]) {
  for (int i = 0; i < 3; ++i) {
    state_.pos[i] = start[i];
    state_.vel[i] = 0.0;
    state_.acc[i] = 0.0;
    state_.goal[i] = start[i];
  }
  state_.hold = false;
}

}  // namespace rt_control
