// motion_limiter 단위 테스트
//
// 속도·가속도는 limiter가 알려주는 값이 아니라 "출력 위치"의 차분으로 직접 잰다.
// (로봇이 실제로 받는 것은 위치이므로, 목표에 딱 맞추는 순간의 점프까지 포함해서 검사)
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "rt_control/motion_limiter.hpp"

namespace {

using rt_control::LimiterConfig;
using rt_control::LimiterResult;
using rt_control::MotionLimiter;

constexpr double kDt = 0.001;
constexpr double kVmax = 0.10;
constexpr double kAmax = 0.50;
constexpr double kAccTol = 1e-6;  // 부동소수점 오차 허용 [m/s²]

struct Vec3 {
  double v[3];
  double operator[](int i) const { return v[i]; }
};

double dist(const Vec3& a, const Vec3& b) {
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                   (a[2] - b[2]) * (a[2] - b[2]));
}

// 출력 위치 기록 + 위치 차분으로 잰 속도·가속도
struct Trace {
  std::vector<Vec3> pos;  // pos[0] = 시작 위치
  std::vector<LimiterResult> res;

  double speed_at(size_t k) const { return dist(pos[k + 1], pos[k]) / kDt; }  // k -> k+1 구간
  double accel_at(size_t k) const {  // k-1 -> k -> k+1 (시작 전에는 정지 상태였다고 봄)
    const Vec3& prev = k == 0 ? pos[0] : pos[k - 1];
    double s = 0;
    for (int i = 0; i < 3; ++i) {
      const double a = (pos[k + 1][i] - 2 * pos[k][i] + prev[i]) / (kDt * kDt);
      s += a * a;
    }
    return std::sqrt(s);
  }
  double max_speed() const {
    double m = 0;
    for (size_t k = 0; k + 1 < pos.size(); ++k) m = std::fmax(m, speed_at(k));
    return m;
  }
  double max_accel() const {
    double m = 0;
    for (size_t k = 0; k + 1 < pos.size(); ++k) m = std::fmax(m, accel_at(k));
    return m;
  }
  // 처음으로 arrived가 된 주기 수 (없으면 -1)
  int arrival_cycle() const {
    for (size_t k = 0; k < res.size(); ++k)
      if (res[k].arrived) return static_cast<int>(k) + 1;
    return -1;
  }
};

class Sim {
 public:
  // 예전 테스트는 저크 제한 없이 (j_max = 0) 예전 동작 그대로인지 확인한다
  explicit Sim(Vec3 start, double j_max = 0.0) : lim_(LimiterConfig{kVmax, kAmax, j_max}) {
    lim_.reset(start.v);
    trace.pos.push_back(start);
  }
  void run(Vec3 goal, int cycles, bool stop = false) {
    for (int k = 0; k < cycles; ++k) {
      const LimiterResult r = lim_.update(goal.v, kDt, stop);
      trace.res.push_back(r);
      trace.pos.push_back(Vec3{{r.pos[0], r.pos[1], r.pos[2]}});
    }
  }
  const Vec3& last() const { return trace.pos.back(); }
  Trace trace;

 private:
  MotionLimiter lim_;
};

// ---- 3cm 직선 이동 ----
TEST(Limiter, Straight3cm) {
  const Vec3 start{{0.5, -0.03, 0.71}};
  const Vec3 goal{{0.5, 0.0, 0.71}};
  Sim sim(start);
  sim.run(goal, 800);
  const Trace& t = sim.trace;

  // 가속 0.2s + 등속 0.1s + 감속 0.2s = 0.5s
  const int arrive = t.arrival_cycle();
  EXPECT_NEAR(arrive, 500, 10);
  EXPECT_LE(t.max_speed(), kVmax + 1e-9);
  EXPECT_GT(t.max_speed(), kVmax * 0.99);  // 등속 구간이 실제로 있음
  EXPECT_LE(t.max_accel(), kAmax + kAccTol);
  for (const Vec3& p : t.pos) EXPECT_LE(p[1], goal[1]);  // 지나치지 않음
  EXPECT_EQ(sim.last()[1], goal[1]);
  EXPECT_EQ(t.res.back().speed, 0.0);
  EXPECT_TRUE(t.res.back().arrived);
  // 도착 뒤에는 그대로
  for (size_t k = arrive; k < t.res.size(); ++k) EXPECT_EQ(t.pos[k + 1][1], goal[1]);
  // 한계 플래그: 처음엔 가속도 한계, 중간엔 속도 한계
  EXPECT_TRUE(t.res[10].accel_limited);
  EXPECT_TRUE(t.res[250].speed_limited);
}

// ---- 대각선 이동: 직선에서 벗어나지 않음 ----
TEST(Limiter, DiagonalStaysOnLine) {
  const Vec3 start{{0.0, 0.0, 0.0}};
  const Vec3 goal{{0.02, 0.02, -0.01}};
  Sim sim(start);
  sim.run(goal, 1000);
  const double L = dist(start, goal);
  double max_off = 0;
  for (const Vec3& p : sim.trace.pos) {
    // 시작->목표 직선까지의 거리 = |p × u|
    const double u[3] = {goal[0] / L, goal[1] / L, goal[2] / L};
    const double cx = p[1] * u[2] - p[2] * u[1];
    const double cy = p[2] * u[0] - p[0] * u[2];
    const double cz = p[0] * u[1] - p[1] * u[0];
    max_off = std::fmax(max_off, std::sqrt(cx * cx + cy * cy + cz * cz));
  }
  EXPECT_LT(max_off, 1e-12);
  EXPECT_LE(sim.trace.max_speed(), kVmax + 1e-9);
  EXPECT_LE(sim.trace.max_accel(), kAmax + kAccTol);
  EXPECT_TRUE(sim.trace.res.back().arrived);
}

// ---- 이동 중 목표 변경 (반대 방향, 옆 방향) ----
TEST(Limiter, GoalChangesMidMove) {
  Sim sim(Vec3{{0, 0, 0}});
  sim.run(Vec3{{0.03, 0, 0}}, 250);    // 최고 속도로 +x 이동 중
  sim.run(Vec3{{-0.02, 0, 0}}, 300);   // 반대 방향
  sim.run(Vec3{{-0.01, 0.02, 0}}, 1200);  // 옆으로
  EXPECT_LE(sim.trace.max_speed(), kVmax + 1e-9);
  EXPECT_LE(sim.trace.max_accel(), kAmax + kAccTol);
  EXPECT_TRUE(sim.trace.res.back().arrived);
  EXPECT_EQ(sim.last()[0], -0.01);
  EXPECT_EQ(sim.last()[1], 0.02);
}

// ---- 빠르게 가는 중 목표가 바로 앞(이번 걸음보다 가까이)으로 옮겨 옴 ----
// 그 자리에서 딱 멈추면 급정거(가속도 수십 m/s²)가 되므로, 한계 안에서 지나쳤다가 돌아와야 함
TEST(Limiter, GoalJumpsJustAheadWhileFast) {
  Sim sim(Vec3{{0, 0, 0}});
  sim.run(Vec3{{0.1, 0, 0}}, 300);  // 0.1 m/s로 이동 중 (한 걸음 = 0.1mm)
  const Vec3 near{{sim.last()[0] + 0.00005, 0, 0}};  // 0.05mm 앞
  sim.run(near, 1000);
  EXPECT_LE(sim.trace.max_accel(), kAmax + kAccTol);
  EXPECT_LE(sim.trace.max_speed(), kVmax + 1e-9);
  EXPECT_TRUE(sim.trace.res.back().arrived);
  EXPECT_EQ(sim.last()[0], near[0]);
}

// ---- 아주 작은 이동 0.1mm: 진동 없이 도착 ----
TEST(Limiter, TinyMoveNoOscillation) {
  Sim sim(Vec3{{0.5, 0.0, 0.2}});
  const Vec3 goal{{0.5001, 0.0, 0.2}};
  sim.run(goal, 200);
  const Trace& t = sim.trace;
  ASSERT_GT(t.arrival_cycle(), 0);
  for (size_t k = 0; k + 1 < t.pos.size(); ++k) {
    EXPECT_GE(t.pos[k + 1][0], t.pos[k][0]);  // 뒤로 가지 않음
    EXPECT_LE(t.pos[k + 1][0], goal[0]);      // 지나치지 않음
  }
  EXPECT_LE(t.max_accel(), kAmax + kAccTol);
  EXPECT_EQ(sim.last()[0], goal[0]);
}

// ---- 이동 중 stop ----
TEST(Limiter, StopDuringMove) {
  Sim sim(Vec3{{0, 0, 0}});
  sim.run(Vec3{{0.1, 0, 0}}, 300);  // 최고 속도 도달
  const double v0 = sim.trace.res.back().speed;
  ASSERT_NEAR(v0, kVmax, 1e-9);
  const size_t stop_at = sim.trace.res.size();
  sim.run(Vec3{{0.1, 0, 0}}, 400, /*stop=*/true);
  const Trace& t = sim.trace;

  // v/a_max = 0.2s 안에 멈춤 (+1주기 여유)
  size_t stopped = 0;
  for (size_t k = stop_at; k < t.res.size(); ++k) {
    if (t.res[k].speed == 0.0) {
      stopped = k;
      break;
    }
  }
  ASSERT_GT(stopped, 0u);
  EXPECT_LE(static_cast<double>(stopped - stop_at + 1) * kDt, v0 / kAmax + kDt + 1e-12);
  EXPECT_LE(t.max_accel(), kAmax + kAccTol);
  // 멈춘 뒤 위치 고정, 목표(0.1)에는 가지 않음
  const double held = t.pos[stopped + 1][0];
  for (size_t k = stopped + 1; k < t.pos.size(); ++k) EXPECT_EQ(t.pos[k][0], held);
  EXPECT_LT(held, 0.1);
  for (size_t k = stop_at; k < t.res.size(); ++k) EXPECT_FALSE(t.res[k].arrived);
}

// ---- NaN 목표 ----
TEST(Limiter, NanGoalKeepsPrevious) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  Sim sim(Vec3{{0, 0, 0}});
  sim.run(Vec3{{0.01, 0, 0}}, 50);
  sim.run(Vec3{{kNaN, 0.0, 0.0}}, 100);
  sim.run(Vec3{{0.0, std::numeric_limits<double>::infinity(), 0.0}}, 600);
  for (const Vec3& p : sim.trace.pos)
    for (int i = 0; i < 3; ++i) ASSERT_TRUE(std::isfinite(p[i]));
  EXPECT_LE(sim.trace.max_accel(), kAmax + kAccTol);
  // 이전 목표 0.01로 계속 가서 도착
  EXPECT_TRUE(sim.trace.res.back().arrived);
  EXPECT_EQ(sim.last()[0], 0.01);
}

// ---- 이미 목표에 있음 ----
TEST(Limiter, AlreadyAtGoal) {
  const Vec3 p{{0.5, 0.0, 0.2}};
  Sim sim(p);
  sim.run(p, 10);
  for (const LimiterResult& r : sim.trace.res) {
    EXPECT_TRUE(r.arrived);
    EXPECT_EQ(r.speed, 0.0);
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(r.pos[i], p[i]);
      EXPECT_EQ(r.vel[i], 0.0);
    }
  }
}

// ---- 도착 후 고정: 출력 위치가 여러 주기 동안 비트 단위로 같음 ----
// 목표 좌표를 일부러 2진수로 딱 떨어지지 않는 값으로 두고, dt도 매 주기 조금씩 흔듦 (노드는 실제 깨어난 간격을 씀)
TEST(LimiterHold, OutputBitIdenticalAfterArrival) {
  MotionLimiter lim(LimiterConfig{kVmax, kAmax});
  const double start[3] = {0.4123456789, -0.0312345, 0.7012345};
  const double goal[3] = {0.4198765432, -0.0287654, 0.7098765};
  lim.reset(start);
  int arrived_at = -1;
  for (int k = 0; k < 3000 && arrived_at < 0; ++k) {
    const LimiterResult r = lim.update(goal, kDt, false);
    if (r.arrived) arrived_at = k;
  }
  ASSERT_GE(arrived_at, 0);
  unsigned char ref[sizeof(double) * 3];
  std::memcpy(ref, lim.state().pos, sizeof(ref));
  for (int i = 0; i < 3; ++i) EXPECT_EQ(lim.state().pos[i], goal[i]);
  for (int k = 0; k < 5000; ++k) {
    const double dt = kDt * (1.0 + 0.05 * std::sin(0.37 * k));  // 0.95~1.05ms
    const LimiterResult r = lim.update(goal, dt, false);
    ASSERT_EQ(std::memcmp(r.pos, ref, sizeof(ref)), 0) << "cycle " << k;
    ASSERT_TRUE(r.arrived);
    ASSERT_TRUE(r.hold);
    ASSERT_EQ(r.speed, 0.0);
    for (int i = 0; i < 3; ++i) ASSERT_EQ(r.vel[i], 0.0);
  }
}

// 고정 중에 NaN 목표가 와도(이전 목표를 씀) 고정 유지, 목표가 바뀌면 풀리고 한계 안에서 다시 움직임
TEST(LimiterHold, ReleasesOnGoalChangeWithinLimits) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  Sim sim(Vec3{{0.5, 0.0, 0.7}});
  sim.run(Vec3{{0.5, 0.01, 0.7}}, 600);
  ASSERT_TRUE(sim.trace.res.back().hold);
  sim.run(Vec3{{kNaN, 0.0, 0.0}}, 50);
  EXPECT_TRUE(sim.trace.res.back().hold);
  EXPECT_EQ(sim.last()[1], 0.01);

  const size_t before = sim.trace.res.size();
  sim.run(Vec3{{0.5, 0.0105, 0.7}}, 600);  // 0.5mm 옮김
  EXPECT_FALSE(sim.trace.res[before].hold);
  EXPECT_GT(sim.trace.pos[before + 1][1], 0.01);  // 첫 주기부터 움직이기 시작 (위치를 갑자기 맞추지 않음)
  EXPECT_LT(sim.trace.pos[before + 1][1], 0.0105);
  EXPECT_LE(sim.trace.max_accel(), kAmax + kAccTol);
  EXPECT_TRUE(sim.trace.res.back().hold);
  EXPECT_EQ(sim.last()[1], 0.0105);
}

// stop(감속 정지)으로 멈춘 자리는 목표가 아니므로 고정하지 않고, stop이 풀리면 목표로 감
TEST(LimiterHold, StopDoesNotLatchHoldAwayFromGoal) {
  Sim sim(Vec3{{0.0, 0.0, 0.0}});
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 100);
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 300, true);
  EXPECT_EQ(sim.trace.res.back().speed, 0.0);
  EXPECT_FALSE(sim.trace.res.back().hold);
  EXPECT_FALSE(sim.trace.res.back().arrived);
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 1000);
  EXPECT_TRUE(sim.trace.res.back().hold);
  EXPECT_EQ(sim.last()[0], 0.03);
  EXPECT_LE(sim.trace.max_accel(), kAmax + kAccTol);
}

// ================= 저크 제한 (j_max > 0) =================

constexpr double kJmax = 1.0;      // [m/s³] 기본값
constexpr double kJerkTol = 1e-4;  // 위치 3차 차분의 반올림 오차 허용 [m/s³]

// 출력 위치만 보고 잰 속도·가속도·저크 (dt가 주기마다 달라도 됨). 시작 전에는 정지 상태
struct JerkTrace {
  std::vector<Vec3> pos;  // pos[0] = 시작
  std::vector<double> dt;
  std::vector<LimiterResult> res;
  std::vector<Vec3> vel, acc;  // 위치 차분 (k번째 = k번째 주기)

  void finish() {
    vel.clear();
    acc.clear();
    for (size_t k = 0; k < dt.size(); ++k) {
      Vec3 v, a;
      for (int i = 0; i < 3; ++i) v.v[i] = (pos[k + 1][i] - pos[k][i]) / dt[k];
      for (int i = 0; i < 3; ++i) a.v[i] = (v[i] - (k ? vel[k - 1][i] : 0.0)) / dt[k];
      vel.push_back(v);
      acc.push_back(a);
    }
  }
  static double n(const Vec3& x) { return std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]); }
  double jerk_at(size_t k) const {
    double s = 0;
    for (int i = 0; i < 3; ++i) {
      const double j = (acc[k][i] - (k ? acc[k - 1][i] : 0.0)) / dt[k];
      s += j * j;
    }
    return std::sqrt(s);
  }
  double max_speed() const { double m = 0; for (const Vec3& v : vel) m = std::fmax(m, n(v)); return m; }
  double max_accel() const { double m = 0; for (const Vec3& a : acc) m = std::fmax(m, n(a)); return m; }
  double max_jerk() const { double m = 0; for (size_t k = 0; k < acc.size(); ++k) m = std::fmax(m, jerk_at(k)); return m; }
  int arrival_cycle() const {
    for (size_t k = 0; k < res.size(); ++k)
      if (res[k].arrived) return static_cast<int>(k) + 1;
    return -1;
  }
};

class JerkSim {
 public:
  JerkSim(Vec3 start, LimiterConfig c) : lim_(c) {
    lim_.reset(start.v);
    t.pos.push_back(start);
  }
  // jitter: dt = 1ms × (1 + jitter × sin(0.37k)) (노드는 실제로 깨어난 간격을 씀)
  void run(Vec3 goal, int cycles, bool stop = false, double jitter = 0.0) {
    for (int k = 0; k < cycles; ++k) {
      const double dt = kDt * (1.0 + jitter * std::sin(0.37 * static_cast<double>(t.dt.size())));
      const LimiterResult r = lim_.update(goal.v, dt, stop);
      t.res.push_back(r);
      t.pos.push_back(Vec3{{r.pos[0], r.pos[1], r.pos[2]}});
      t.dt.push_back(dt);
    }
    t.finish();
  }
  const Vec3& last() const { return t.pos.back(); }
  JerkTrace t;

 private:
  MotionLimiter lim_;
};

LimiterConfig jerk_config(double j = kJmax) { return LimiterConfig{kVmax, kAmax, j}; }

TEST(LimiterJerk, DefaultJmaxIsOne) { EXPECT_DOUBLE_EQ(LimiterConfig{}.j_max, 1.0); }

// 정지에서 출발: 가속도가 매 주기 j_max × dt 이하로만 바뀜 (0 -> a_max로 한 번에 뛰지 않음)
TEST(LimiterJerk, StartFromRestAccelChangesAtMostJdtPerCycle) {
  JerkSim sim(Vec3{{0.5, -0.03, 0.71}}, jerk_config());
  sim.run(Vec3{{0.5, 0.0, 0.71}}, 1500);
  const JerkTrace& t = sim.t;
  // 첫 주기 가속도 = j_max × dt (예전에는 a_max)
  EXPECT_NEAR(JerkTrace::n(t.acc[0]), kJmax * kDt, 1e-9);
  // 처음 50주기는 가속도가 정확히 k × j_max × dt로 늘어남 (S자 출발)
  for (size_t k = 0; k < 50; ++k) EXPECT_NEAR(JerkTrace::n(t.acc[k]), (k + 1) * kJmax * kDt, 1e-6) << k;
  for (size_t k = 0; k < t.acc.size(); ++k) {
    const Vec3 prev = k ? t.acc[k - 1] : Vec3{{0, 0, 0}};
    double s = 0;
    for (int i = 0; i < 3; ++i) s += (t.acc[k][i] - prev[i]) * (t.acc[k][i] - prev[i]);
    ASSERT_LE(std::sqrt(s), kJmax * kDt + kJerkTol * kDt) << "cycle " << k;
  }
}

// 3cm 이동: 지나침 없이 정확히 목표에서 끝나고, 속도·가속도·저크 한계를 지킴. 끝난 뒤 출력 고정
TEST(LimiterJerk, Straight3cmNoOvershoot) {
  const Vec3 start{{0.5, -0.03, 0.71}};
  const Vec3 goal{{0.5, 0.0, 0.71}};
  JerkSim sim(start, jerk_config());
  sim.run(goal, 2000);
  const JerkTrace& t = sim.t;
  const int arrive = t.arrival_cycle();
  ASSERT_GT(arrive, 0);
  // 시간 최적 S자(삼각 가속도) 3cm: 최고 속도 (0.015)^(2/3) ≈ 0.0608 m/s, 걸리는 시간 ≈ 0.986 s
  EXPECT_NEAR(arrive, 1000, 60);
  for (const Vec3& p : t.pos) ASSERT_LE(p[1], goal[1]);  // 지나치지 않음
  EXPECT_EQ(sim.last()[1], goal[1]);
  EXPECT_EQ(sim.last()[0], goal[0]);
  EXPECT_EQ(sim.last()[2], goal[2]);
  EXPECT_LE(t.max_speed(), kVmax + 1e-9);
  EXPECT_LE(t.max_accel(), kAmax + 1e-9);
  EXPECT_LE(t.max_jerk(), kJmax + kJerkTol);
  // 도착 후 출력이 비트 단위로 같고 hold 유지
  for (size_t k = arrive; k < t.res.size(); ++k) {
    ASSERT_EQ(std::memcmp(t.res[k].pos, t.res[arrive - 1].pos, sizeof(double) * 3), 0) << k;
    ASSERT_TRUE(t.res[k].arrived);
  }
  EXPECT_TRUE(t.res.back().hold);
}

// 짧은 이동·여러 한계 조합: 모두 지나침 없이 정확히 도착
TEST(LimiterJerk, VariousDistancesAndLimits) {
  struct Case {
    double v, a, j, dist;
    int cycles;
  } cases[] = {{0.10, 0.50, 1.0, 0.0005, 1000},   // 0.5mm
               {0.10, 0.50, 1.0, 0.00001, 1000},  // 10um
               {0.10, 0.50, 10.0, 0.03, 2000},    // 가속도·속도 한계까지 닿음
               {0.10, 5.00, 0.5, 0.03, 3000},     // 저크가 지배
               {0.001, 0.002, 1.0, 0.005, 8000},  // fr3_integration.yaml 값, 5mm
               {0.001, 0.002, 0.01, 0.0001, 3000}};
  for (const Case& c : cases) {
    JerkSim sim(Vec3{{0.0, 0.0, 0.0}}, LimiterConfig{c.v, c.a, c.j});
    sim.run(Vec3{{c.dist, 0.0, 0.0}}, c.cycles);
    const JerkTrace& t = sim.t;
    SCOPED_TRACE(testing::Message() << "v " << c.v << " a " << c.a << " j " << c.j << " d " << c.dist);
    ASSERT_GT(t.arrival_cycle(), 0);
    for (const Vec3& p : t.pos) ASSERT_LE(p[0], c.dist);
    EXPECT_EQ(sim.last()[0], c.dist);
    EXPECT_LE(t.max_speed(), c.v * (1 + 1e-9));
    EXPECT_LE(t.max_accel(), c.a * (1 + 1e-9));
    EXPECT_LE(t.max_jerk(), c.j + kJerkTol);
  }
}

// dt가 주기마다 흔들려도(±5%) 저크 한계·지나침 없음·정확한 도착
TEST(LimiterJerk, VaryingDtKeepsLimits) {
  JerkSim sim(Vec3{{0.5, -0.03, 0.71}}, jerk_config());
  sim.run(Vec3{{0.5, 0.0, 0.71}}, 2500, false, 0.05);
  const JerkTrace& t = sim.t;
  ASSERT_GT(t.arrival_cycle(), 0);
  // 마지막 3주기(데드비트)는 다음 dt가 지금과 같다고 보고 풀기 때문에, dt가 바뀌면 반올림 오차 수준
  // (1e-25m 정도)로 목표를 스칠 수 있음. 1e-15m(펨토미터)까지 허용
  for (const Vec3& p : t.pos) ASSERT_LE(p[1], 1e-15);
  EXPECT_EQ(sim.last()[1], 0.0);
  EXPECT_LE(t.max_jerk(), kJmax + kJerkTol);
  EXPECT_LE(t.max_accel(), kAmax + 1e-9);
}

// 대각선 이동도 직선에서 벗어나지 않음
TEST(LimiterJerk, DiagonalStaysOnLine) {
  const Vec3 start{{0.0, 0.0, 0.0}};
  const Vec3 goal{{0.02, 0.02, -0.01}};
  JerkSim sim(start, jerk_config());
  sim.run(goal, 2500);
  const double L = dist(start, goal);
  double max_off = 0;
  const double u[3] = {goal[0] / L, goal[1] / L, goal[2] / L};
  for (const Vec3& p : sim.t.pos) {
    // 시작->목표 직선까지의 거리 = |p × u| (예전 테스트와 같은 방식)
    const double cx = p[1] * u[2] - p[2] * u[1];
    const double cy = p[2] * u[0] - p[0] * u[2];
    const double cz = p[0] * u[1] - p[1] * u[0];
    max_off = std::fmax(max_off, std::sqrt(cx * cx + cy * cy + cz * cz));
    ASSERT_LE(p[0] * u[0] + p[1] * u[1] + p[2] * u[2], L + 1e-12);
  }
  EXPECT_LT(max_off, 1e-12);
  ASSERT_GT(sim.t.arrival_cycle(), 0);
  EXPECT_LE(sim.t.max_jerk(), kJmax + kJerkTol);
}

// 감속 정지(stop): 저크 한계 안에서 속도가 정확히 0이 됨 (safety_mode가 "멈춤"으로 봄), 풀리면 목표로 감
TEST(LimiterJerk, StopDeceleratesWithinJerkToExactZero) {
  JerkSim sim(Vec3{{0.0, 0.0, 0.0}}, jerk_config());
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 400);
  ASSERT_GT(sim.t.res.back().speed, 0.01);
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 1000, true);
  EXPECT_EQ(sim.t.res.back().speed, 0.0);
  EXPECT_FALSE(sim.t.res.back().arrived);
  EXPECT_LE(sim.t.max_jerk(), kJmax + kJerkTol);
  EXPECT_LE(sim.last()[0], 0.03);
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 2000);
  ASSERT_TRUE(sim.t.res.back().arrived);
  EXPECT_EQ(sim.last()[0], 0.03);
  EXPECT_LE(sim.t.max_jerk(), kJmax + kJerkTol);
  for (const Vec3& p : sim.t.pos) ASSERT_LE(p[0], 0.03);
}

// 움직이는 중에 목표가 바뀜 (뒤로, 옆으로): 저크·가속도 한계는 항상 지키고 결국 정확히 도착
TEST(LimiterJerk, RetargetKeepsLimits) {
  JerkSim sim(Vec3{{0.0, 0.0, 0.0}}, jerk_config());
  sim.run(Vec3{{0.03, 0.0, 0.0}}, 500);
  sim.run(Vec3{{0.005, 0.01, 0.0}}, 300);  // 뒤·옆으로
  sim.run(Vec3{{0.02, -0.005, 0.003}}, 3000);
  EXPECT_LE(sim.t.max_jerk(), kJmax + kJerkTol);
  EXPECT_LE(sim.t.max_accel(), kAmax + 1e-9);
  EXPECT_LE(sim.t.max_speed(), kVmax + 1e-9);
  ASSERT_TRUE(sim.t.res.back().arrived);
  EXPECT_EQ(sim.last()[0], 0.02);
  EXPECT_EQ(sim.last()[1], -0.005);
  EXPECT_EQ(sim.last()[2], 0.003);
}

// j_max = 0이면 예전 limiter와 비트 단위로 같음.
// 기대값은 저크 제한을 넣기 전 커밋(c9a53ad)의 motion_limiter로 같은 시나리오를 돌려 얻은 값 (%a 표기)
TEST(LimiterJerk, ZeroJmaxMatchesPreviousLimiterBitForBit) {
  struct Ref {
    int cycle;
    double x, y, z;
  };
  const Ref kRef[] = {
      {99, 0x1p-1, -0x1.c2206d36a3f14p-6, 0x1.6b851eb851eb8p-1},
      {199, 0x1p-1, -0x1.46ba36f7c5f1ep-6, 0x1.6b851eb851eb8p-1},
      {299, 0x1.fede121c2a15fp-2, -0x1.86551c5f9f747p-7, 0x1.6c1615aa3ce0cp-1},
      {399, 0x1.fc24974d0604p-2, -0x1.112bccc8cf93bp-7, 0x1.6d72d311cee97p-1},
      {499, 0x1.f8cdf03ffedc2p-2, -0x1.3c622d9025051p-7, 0x1.6f1e2698527d9p-1},
      {599, 0x1.f637ebda61f64p-2, -0x1.ffae9dd0c7a43p-7, 0x1.706928cb20f02p-1},
      {699, 0x1.f561f1c30c161p-2, -0x1.4b1a6ea265174p-6, 0x1.70d425d6cbe06p-1},
      {799, 0x1.f54b444fcbd9fp-2, -0x1.531462cb2191dp-6, 0x1.70df7c906bfe9p-1},
      {899, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
      {999, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
      {1099, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
      {1199, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
      {1299, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
      {1399, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
      {1499, 0x1.f5c28f5c28f5cp-2, -0x1.47ae147ae147bp-6, 0x1.70a3d70a3d70ap-1},
  };
  LimiterConfig c{kVmax, kAmax, 0.0};
  MotionLimiter lim(c);
  const double s[3] = {0.5, -0.03, 0.71};
  lim.reset(s);
  const double g1[3] = {0.5, 0.0, 0.71}, g2[3] = {0.49, -0.02, 0.72};
  size_t ri = 0;
  for (int k = 0; k < 1500; ++k) {
    const double dt = 0.001 * (1.0 + 0.03 * std::sin(0.37 * k));
    const LimiterResult r = lim.update(k < 200 ? g1 : g2, dt, k >= 600 && k < 800);
    if (ri < sizeof(kRef) / sizeof(kRef[0]) && kRef[ri].cycle == k) {
      EXPECT_EQ(r.pos[0], kRef[ri].x) << k;
      EXPECT_EQ(r.pos[1], kRef[ri].y) << k;
      EXPECT_EQ(r.pos[2], kRef[ri].z) << k;
      ++ri;
    }
  }
  EXPECT_EQ(ri, sizeof(kRef) / sizeof(kRef[0]));
}

}  // namespace
