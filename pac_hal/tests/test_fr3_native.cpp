// Copyright 2026 Jaewon Kim

#include <cmath>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <random>
#include <iomanip>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include "gtest/gtest.h"
#include "pac_hal/hal_converter.hpp"
#include "pac_hal/hal_state_monitor.hpp"
#include "pac_hal/joint_trajectory.hpp"
#include "pac_hal/tcp_trajectory.hpp"
#include "pac_hal/scene_guard.hpp"
#include "pac_hal/hal_latency_trace.hpp"
#include "pac_hal/receiver_contract.hpp"
#include "pac_hal/execution_contract.hpp"

namespace
{
TEST(Scenario4, DiagnosticJsonEscapesControlCharactersAndPreservesUnicode)
{
  const std::string value=std::string("name\n\t\"\\")+std::string(1,'\0')+"재원";
  const auto encoded=pac_hal::json_string(value);
  EXPECT_EQ(encoded,"\"name\\n\\t\\\"\\\\\\u0000재원\"");
  EXPECT_EQ(YAML::Load(encoded).as<std::string>(),value);
  EXPECT_EQ(pac_hal::json_string("\b\f\r"),"\"\\b\\f\\r\"");
}
TEST(Scenario4, PublishedRobotContractPreservesFramesTransformsAndCapabilities)
{
  const auto c=pac_hal::load_robot_config(FR3_CONFIG);
  auto report=YAML::Load(pac_hal::describe_robot_contract(c));
  EXPECT_EQ(report["reference_frame"].as<std::string>(),c.reference);
  EXPECT_EQ(report["base_frame"].as<std::string>(),c.base);
  EXPECT_EQ(report["flange_frame"].as<std::string>(),c.flange);
  EXPECT_EQ(report["declared_tcp_frame"].as<std::string>(),c.tcp);
  EXPECT_DOUBLE_EQ(report["reference_to_base"]["translation_m"][2].as<double>(),c.reference_to_base.position.z);
  EXPECT_DOUBLE_EQ(report["flange_to_tcp"]["rotation_xyzw"][3].as<double>(),c.flange_to_tcp.orientation.w);
  EXPECT_EQ(report["capabilities"]["absolute_tcp_position"].as<bool>(),c.position_capability);
  EXPECT_EQ(report["capabilities"]["measured_joint_feedback"].as<bool>(),c.feedback_capability);
  EXPECT_EQ(report["capabilities"]["receiver_watchdog"].as<bool>(),c.watchdog_capability);
}
TEST(Scenario4, OnlyNewActiveAttemptsCanCancelPendingTargets)
{
  std::optional<uint64_t> observed=100;
  pac_hal::Command c;c.session_id=42;c.command_seq=101;
  EXPECT_TRUE(pac_hal::is_new_active_attempt(c,42,observed));
  c.session_id=43;EXPECT_FALSE(pac_hal::is_new_active_attempt(c,42,observed));
  c.session_id=0;EXPECT_FALSE(pac_hal::is_new_active_attempt(c,42,observed));
  c.session_id=42;
  for (const uint64_t seq : {0u,99u,100u}) {
    c.command_seq=seq;EXPECT_FALSE(pac_hal::is_new_active_attempt(c,42,observed));
  }
  observed.reset();c.command_seq=1;EXPECT_TRUE(pac_hal::is_new_active_attempt(c,42,observed));
  c.command_seq=0;EXPECT_FALSE(pac_hal::is_new_active_attempt(c,42,observed));
  c.session_id=0;c.command_seq=1;EXPECT_FALSE(pac_hal::is_new_active_attempt(c,0,observed));
}
TEST(ModelDescriptionGuard, BindsOnlyAfterInitializationAndLatchesChanges)
{
  pac_hal::ModelDescriptionGuard guard;
  EXPECT_TRUE(guard.observe("invalid first attempt"));
  EXPECT_FALSE(guard.bound());
  EXPECT_TRUE(guard.observe("verified model"));
  guard.bind("verified model");
  EXPECT_TRUE(guard.observe("verified model"));
  EXPECT_FALSE(guard.changed());
  EXPECT_FALSE(guard.observe("different tool or geometry"));
  EXPECT_TRUE(guard.changed());
  EXPECT_FALSE(guard.observe("verified model"));
  guard.bind("different tool or geometry");
  EXPECT_FALSE(guard.observe("different tool or geometry"));
}
TEST(ModelDescriptionGuard, PreparedSnapshotRejectsADifferentFirstDescription)
{
  pac_hal::ModelDescriptionGuard guard;guard.bind("receiver-prepared model");
  EXPECT_FALSE(guard.observe("different first ROS topic model"));
  EXPECT_TRUE(guard.changed());
  EXPECT_FALSE(guard.observe("receiver-prepared model"));
}
using pac_hal::Config;
using pac_hal::HalConverter;
using pac_hal::HalStateMonitor;
using pac_hal::Measurement;
using pac_hal::Receiver;
using pac_hal::State;

Config config()
{
  auto c = pac_hal::load_fr3_config(FR3_CONFIG);
  c.logging = false;
  return c;
}
std::string urdf()
{
  std::ifstream input(FR3_URDF);
  std::stringstream text;
  text << input.rdbuf();
  return text.str();
}
Measurement sample()
{
  Measurement m;
  const auto pi = std::acos(-1.);
  m.position = {0., -pi / 4, 0., -3 * pi / 4, 0., pi / 2, pi / 4};
  m.header.stamp.sec = 1;
  m.velocity_valid = true;
  return m;
}
Receiver receiver(const Config & c, int64_t now)
{
  Receiver r;
  r.session_id = c.session;
  r.joint_count = 7;
  r.active = true;
  r.sampled_mono_ns = now;
  return r;
}

TEST(Fr3Native, OfficialModelFkIkRoundTrip)
{
  auto c = config();
  c.ik_budget_ns = 100000000;  // 계산 정확도 시험은 OS 부하와 분리한다. production은 500us.
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  const auto m = sample();
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(m, pose, twist));
  EXPECT_NEAR(pose.position.x, .3068905666, 1e-8);
  EXPECT_NEAR(pose.position.z, .5902820523, 1e-8);
  pose.position.z += .00001;
  pac_hal::Joints solution;
  ASSERT_TRUE(converter.inverse(pose, m.position, solution));
  auto moved = m;
  moved.position = solution;
  geometry_msgs::msg::Pose measured;
  ASSERT_TRUE(converter.forward(moved, measured, twist));
  EXPECT_NEAR(measured.position.z, pose.position.z, 1e-6);
}

TEST(Fr3Native, InstallationAndTcpOffsetAreUsed)
{
  auto c = config();
  c.reference_to_base.position.x = .2;
  c.flange_to_tcp.position.z = .05;
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(sample(), pose, twist));
  EXPECT_NEAR(pose.position.x, .5068905666, 1e-8);
  // 초기 flange의 z축이 아래를 향하므로 tool +z는 map -z다.
  EXPECT_NEAR(pose.position.z, .5402820523, 1e-8);
}

TEST(Fr3Native, ToolJacobianMatchesFiniteDifference)
{
  auto c = config();
  c.flange_to_tcp.position.x = .03;
  c.flange_to_tcp.position.z = .05;
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  auto m = sample();
  m.velocity[0] = .1;
  geometry_msgs::msg::Pose before, after;
  geometry_msgs::msg::Twist twist, unused;
  ASSERT_TRUE(converter.forward(m, before, twist));
  m.position[0] += .1e-5;
  ASSERT_TRUE(converter.forward(m, after, unused));
  EXPECT_NEAR((after.position.x - before.position.x) / 1e-5, twist.linear.x, 1e-5);
  EXPECT_NEAR((after.position.y - before.position.y) / 1e-5, twist.linear.y, 1e-5);
}

TEST(Fr3Native, MissingPhysicalAndExcessiveModelsAreRejected)
{
  HalConverter converter(config());
  EXPECT_FALSE(converter.initialize("<robot name='physical'/ >"));
  EXPECT_FALSE(converter.initialize("<robot name='empty'><!-- gz_ros2_control --></robot>"));
  auto c = config();
  c.axes[0].velocity = 100.;
  HalConverter excessive(c);
  EXPECT_FALSE(excessive.initialize(urdf()));
}

TEST(Fr3Native, UnreachableAndDeadlineCommandsAreRejected)
{
  auto c = config();
  c.ik_budget_ns = 100000000;
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.;
  pose.position.x = 10.;
  pac_hal::Joints q;
  EXPECT_FALSE(converter.inverse(pose, sample().position, q));
  c.ik_budget_ns = 1; c.ik_budget = pac_hal::IkBudget::Wall;  // clock-budget mode
  HalConverter deadline(c);
  ASSERT_TRUE(deadline.initialize(urdf()));
  geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(deadline.forward(sample(), pose, twist));
  EXPECT_EQ(deadline.inverse(pose, sample().position, q).code, pac_hal::Error::E_IK_FAILED);
}

TEST(Fr3Native, HeartbeatAndDuplicateSamplesCannotRefreshPose)
{
  const auto c = config();
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  HalStateMonitor monitor(c);
  const int64_t now = 1000000000;
  EXPECT_FALSE(monitor.healthy(now));
  ASSERT_TRUE(monitor.observe(sample(), converter, now));
  EXPECT_FALSE(monitor.healthy(now));  // 실제 측정만 있어도 수신단 활성 확인 전 READY 금지.
  ASSERT_TRUE(monitor.observe_receiver(receiver(c, now), now));
  ASSERT_TRUE(monitor.healthy(now));
  const auto first = monitor.state(now, 1, State::CMD_SENT, {});
  EXPECT_EQ(first.hal_state, State::HAL_READY);
  EXPECT_FALSE(monitor.observe(sample(), converter, now + c.stale_ns));
  const auto old = monitor.state(now + c.stale_ns, 1, State::CMD_SENT, {});
  EXPECT_EQ(old.state_seq, first.state_seq);
  EXPECT_EQ(old.received_mono_ns, first.received_mono_ns);
  EXPECT_FALSE(old.pose_valid);
  EXPECT_TRUE(old.stale);
  EXPECT_EQ(old.hal_state, State::HAL_FAULT);
}

TEST(Fr3Native, WrongSessionAndOldReceiverDoNotArmHal)
{
  const auto c = config();
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  HalStateMonitor monitor(c);
  const int64_t now = 1000000000;
  ASSERT_TRUE(monitor.observe(sample(), converter, now));
  auto r = receiver(c, now);
  r.session_id++;
  EXPECT_FALSE(monitor.observe_receiver(r, now));
  r = receiver(c, now - c.lost_ns);
  EXPECT_FALSE(monitor.observe_receiver(r, now));
  r = receiver(c, now);
  ASSERT_TRUE(monitor.observe_receiver(r, now));
  EXPECT_FALSE(monitor.observe_receiver(r, now + 1));
  EXPECT_FALSE(monitor.healthy(now + c.lost_ns));
}

TEST(Fr3Native, SmoothVerticalTrajectoryDoesNotAcquireNumericalAccelerationSpikes)
{
  auto c = config();
  c.ik_budget_ns = 100000000;  // Numerical regression, not a real-time benchmark.
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  Measurement m;
  m.position = {-.0000005826, -.3571429835, .0000014681, -1.4191722009,
    -.0000038316, 1.0620303777, .7853970320};
  geometry_msgs::msg::Pose origin;
  geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(m, origin, twist));
  auto previous = m.position;
  pac_hal::Joints velocity{};
  double t = 0., previous_dt = .001;
  for (int step = 0; step < 20000; ++step) {
    const double dt = step % 2 == 0 ? .0007 : .0013;
    t += dt;
    auto target = origin;
    target.position.z += .025 * (1 - std::cos(.5 * t));
    pac_hal::Joints q;
    ASSERT_TRUE(converter.inverse(target, previous, q));
    const auto result = pac_hal::check_joint_trajectory(c, converter.lower(),
      converter.upper(), q, previous, velocity, dt, previous_dt);
    ASSERT_TRUE(result) << "step=" << step << " " << result.detail;
    for (size_t i = 0; i < 7; ++i) {velocity[i] = (q[i] - previous[i]) / dt;}
    previous = q;
    previous_dt = dt;
  }
}

TEST(Fr3Native, NonuniformIntervalsUseVelocityMidpointsAndKeepLimits)
{
  const auto c = config();
  pac_hal::Joints lower{}, upper{}, previous{}, q{}, velocity{};
  lower.fill(-1.); upper.fill(1.);
  // Constant acceleration 0.8, previous interval 3ms, current interval 1ms.
  // Previous average velocity=0.0012, next=0.0028: correct acceleration=0.8,
  // whereas dividing by only the current interval incorrectly gives 1.6.
  velocity[0] = .0012;
  q[0] = .0028 * .001;
  EXPECT_TRUE(pac_hal::check_joint_trajectory(c, lower, upper, q, previous,
      velocity, .001, .003));
  q[0] = .004 * .001;
  EXPECT_FALSE(pac_hal::check_joint_trajectory(c, lower, upper, q, previous,
      velocity, .001, .003));
  q[0] = .6 * .001;
  EXPECT_FALSE(pac_hal::check_joint_trajectory(c, lower, upper, q, previous,
      velocity, .001, .003));
  EXPECT_FALSE(pac_hal::check_joint_trajectory(c, lower, upper, q, previous,
      velocity, 0., .003));
}

TEST(Fr3Native, DelayedReceiverStatusCannotExtendItsFreshnessWindow)
{
  const auto c = config();
  HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  HalStateMonitor monitor(c);
  const int64_t now = 1000000000;
  ASSERT_TRUE(monitor.observe(sample(), converter, now));
  ASSERT_TRUE(monitor.observe_receiver(receiver(c, now - c.lost_ns + 1000000), now));
  EXPECT_TRUE(monitor.healthy(now));
  EXPECT_TRUE(monitor.receiver_lost(now + 1000000));
  EXPECT_FALSE(monitor.healthy(now + 1000000));
}

TEST(Fr3Native, SceneRejectsOverlappingHomeAndAllowsRaisedTaskPose)
{
  pac_hal::SceneGuard guard(config(), FR3_SCENE);
  ASSERT_TRUE(guard.initialize(urdf()));
  const auto collision = guard.check(sample().position);
  EXPECT_EQ(collision.code, pac_hal::Error::E_OUT_OF_RANGE);
  EXPECT_NE(collision.detail.find("environment collision"), std::string::npos);
  pac_hal::Joints q{-.0000005826, -.3571429835, .0000014681, -1.4191722009,
    -.0000038316, 1.0620303777, .7853970320};
  EXPECT_TRUE(guard.check(q));
  auto from = q, to = q;
  from[0] = -.1; to[0] = .1;
  EXPECT_TRUE(guard.check_path(from, to));
  EXPECT_EQ(guard.check_path(from, to, 1).code, pac_hal::Error::E_EXPIRED);
  from[0] = -.5; to[0] = .5;
  EXPECT_EQ(guard.check_path(from, to).code, pac_hal::Error::E_OUT_OF_RANGE);
}

TEST(Fr3Native, SceneSelfCollisionPairsAreChecked)
{
  auto yaml = YAML::LoadFile(FR3_SCENE);
  for (auto box : yaml["boxes"]) {box["center"] = std::vector<double>{100., 100., 100.};}
  const auto path = std::filesystem::temp_directory_path() /
    ("fr3_self_scene_" + std::to_string(getpid()) + ".yaml");
  {std::ofstream file(path); file << yaml;}
  pac_hal::SceneGuard guard(config(), path.string());
  std::filesystem::remove(path);
  ASSERT_TRUE(guard.initialize(urdf()));
  std::mt19937 random(42);
  const auto c = config();
  bool found = false;
  for (int k = 0; k < 200 && !found; ++k) {
    pac_hal::Joints q;
    for (size_t i = 0; i < 7; ++i) {
      q[i] = std::uniform_real_distribution<double>(c.axes[i].lower, c.axes[i].upper)(random);
    }
    const auto result = guard.check(q);
    found = result.detail.find("self collision") != std::string::npos;
  }
  EXPECT_TRUE(found);
}

TEST(Fr3Native, SceneFailsClosedWithoutModelAndForNonfinitePath)
{
  pac_hal::SceneGuard guard(config(), FR3_SCENE);
  EXPECT_FALSE(guard.ready());
  EXPECT_EQ(guard.check(sample().position).code, pac_hal::Error::E_TF);
  ASSERT_TRUE(guard.initialize(urdf()));
  auto q = sample().position;
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(guard.check_path(q, q).code, pac_hal::Error::E_FORMAT);
}
TEST(Fr3Native, AsyncTraceDrainsCommandsAndAssociatesRealAck)
{
  auto c = config(); c.logging = true;
  c.log_directory = std::filesystem::temp_directory_path().string();
  c.run_id = "fr3_async_trace_" + std::to_string(getpid()) + "_" + std::to_string(pac_hal::monotonic_ns());
  const auto folder = std::filesystem::path(c.log_directory) / c.run_id;
  {
    pac_hal::HalLatencyTrace trace(c);
    pac_hal::TraceRecord record; record.session = c.session; record.seq = 42;
    record.t1 = 100; record.t2 = 110; record.t3 = 120; record.t4 = 130; record.sent = true;
    record.source_result_id = 7; record.created = 90; record.validated = 112;
    record.ik_end = 115; record.transport_start = 130; record.transport_end = 140;
    trace.command(record);
    auto rejected = record;rejected.seq = 43;rejected.sent = false;rejected.error = 5;
    trace.command(rejected);
    auto superseded = record;superseded.seq = 44;superseded.sent = false;superseded.superseded = true;
    trace.command(superseded);
    auto ack = receiver(c, 500); ack.last_applied_command_seq = 42; ack.applied_mono_ns = 450;
    trace.applied(ack, 500);
    trace.applied(ack, 600);  // Same ACK must not create a second applied record.
    trace.feedback_gap("joint_sim_time", 100000000, 1000, 30000000, 1000000);
  }
  std::ifstream input(folder / "hal.csv"); std::stringstream csv; csv << input.rdbuf();
  EXPECT_NE(csv.str().find("sent,0"), std::string::npos);
  EXPECT_NE(csv.str().find("rejected,5"), std::string::npos);
  EXPECT_NE(csv.str().find("superseded,0"), std::string::npos);
  EXPECT_NE(csv.str().find("receiver_applied_mono_ns,ack_received_mono_ns"), std::string::npos);
  EXPECT_NE(csv.str().find(",7,90,100,112,115,130,140,450,500"), std::string::npos);
  EXPECT_NE(csv.str().find(",500,applied,0"), std::string::npos);
  EXPECT_EQ(csv.str().find(",600,applied,0"), std::string::npos);
  EXPECT_EQ(YAML::LoadFile((folder / "trace_stats.yaml").string())["dropped"].as<int>(), 0);
  std::ifstream gaps(folder / "feedback_gaps.csv"); std::stringstream gap; gap << gaps.rdbuf();
  EXPECT_NE(gap.str().find("joint_sim_time,100000000,1000,30000000,1000000"), std::string::npos);
  std::filesystem::remove_all(folder);
}
TEST(Scenario4, TraceReportsCoalescedMissingAndImpossibleAck)
{
  auto c=config();c.logging=true;c.log_directory=std::filesystem::temp_directory_path().string();
  c.run_id="hal_ack_audit_"+std::to_string(getpid())+"_"+std::to_string(pac_hal::monotonic_ns());
  const auto folder=std::filesystem::path(c.log_directory)/c.run_id;
  {
    pac_hal::HalLatencyTrace trace(c);pac_hal::TraceRecord record;
    record.session=c.session;record.sent=true;record.created=90;record.transport_start=130;
    for(uint64_t seq=1;seq<=300;++seq) {record.seq=seq;trace.command(record);}
    auto ack=receiver(c,500);ack.last_applied_command_seq=1;ack.applied_mono_ns=450;
    trace.applied(ack,500); // Evicted sent record: cannot invent a matching ACK.
    ack.last_applied_command_seq=300;ack.applied_mono_ns=50;trace.applied(ack,500);
    ack.applied_mono_ns=120;trace.applied(ack,500); // Before transport start.
    ack.applied_mono_ns=450;trace.applied(ack,400); // Arrival before adoption.
    trace.applied(ack,500); // Valid exact identity.
    record.seq=301;trace.command(record); // Deliberately unacknowledged at shutdown.
  }
  auto stats=YAML::LoadFile((folder/"trace_stats.yaml").string());
  EXPECT_EQ(stats["dropped"].as<int>(),0);
  EXPECT_EQ(stats["sent_records"].as<int>(),301);
  EXPECT_EQ(stats["applied_records"].as<int>(),1);
  EXPECT_EQ(stats["pending_evicted_without_ack"].as<int>(),44);
  EXPECT_EQ(stats["older_sent_without_individual_ack"].as<int>(),255);
  EXPECT_EQ(stats["shutdown_pending_without_ack"].as<int>(),1);
  EXPECT_EQ(stats["unmatched_ack_events"].as<int>(),1);
  EXPECT_EQ(stats["invalid_ack_time_events"].as<int>(),3);
  std::ifstream input(folder/"hal.csv");std::stringstream csv;csv<<input.rdbuf();
  EXPECT_EQ(csv.str().find(",50,500"),std::string::npos);
  EXPECT_EQ(csv.str().find(",120,500"),std::string::npos);
  EXPECT_NE(csv.str().find(",450,500"),std::string::npos);
  std::filesystem::remove_all(folder);
}

TEST(Scenario4, DiagnosticTraceEscapesCsvAndMarksTruncation)
{
  auto c=config();c.logging=true;c.log_directory=std::filesystem::temp_directory_path().string();
  c.run_id="hal_diagnostic_audit_"+std::to_string(getpid())+"_"+std::to_string(pac_hal::monotonic_ns());
  const auto folder=std::filesystem::path(c.log_directory)/c.run_id;
  {
    pac_hal::HalLatencyTrace trace(c);pac_hal::Error e;
    e.session_id=c.session;e.command_seq=7;e.event_mono_ns=100;e.code=10;e.active=true;
    e.detail="reason, \"quoted\"\nsecond line";trace.error(e);
    e.detail=std::string(510,'a')+"가";trace.error(e);
    trace.receiver_diagnostic("{\"reason\":\"expired_at_receiver\",\"command_seq\":7}",110);
    trace.receiver_diagnostic(std::string("part\0hidden",11),120);
  }
  const auto stats=YAML::LoadFile((folder/"trace_stats.yaml").string());
  EXPECT_EQ(stats["error_transition_records"].as<int>(),2);
  EXPECT_EQ(stats["receiver_diagnostic_records"].as<int>(),2);
  EXPECT_EQ(stats["truncated_diagnostic_records"].as<int>(),2);
  std::ifstream input(folder/"hal_errors.csv");std::stringstream errors;errors<<input.rdbuf();
  EXPECT_NE(errors.str().find("\"reason, \"\"quoted\"\"\nsecond line\",0"),std::string::npos);
  EXPECT_NE(errors.str().find(std::string(510,'a')+"\",1"),std::string::npos);
  std::ifstream data(folder/"receiver_diagnostics.csv");std::stringstream diagnostic;diagnostic<<data.rdbuf();
  EXPECT_NE(diagnostic.str().find("\"\"reason\"\":\"\"expired_at_receiver\"\""),std::string::npos);
  EXPECT_NE(diagnostic.str().find("120,\"part\",1"),std::string::npos);
  std::filesystem::remove_all(folder);
}

TEST(Fr3Native, OptionalCpuBudgetStillRejectsExpiredAndOverbudgetIK)
{
  auto c = config(); c.ik_budget = pac_hal::IkBudget::ThreadCpu;
  HalConverter converter(c); ASSERT_TRUE(converter.initialize(urdf()));
  geometry_msgs::msg::Pose pose; geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(sample(), pose, twist)); pac_hal::Joints output;
  EXPECT_EQ(converter.inverse(pose, sample().position, output, 1).code, pac_hal::Error::E_EXPIRED);
  EXPECT_TRUE(converter.inverse(pose, sample().position, output));
  c.ik_budget_ns = 1; HalConverter tight(c); ASSERT_TRUE(tight.initialize(urdf()));
  EXPECT_EQ(tight.inverse(pose, sample().position, output).code, pac_hal::Error::E_IK_FAILED);
}
TEST(Fr3Native, ReadyRequiresMeasuredRobotAndReceiverToBeStopped)
{
  const auto c = config(); HalConverter converter(c); ASSERT_TRUE(converter.initialize(urdf()));
  HalStateMonitor monitor(c); const int64_t now = 1000000000;
  auto measured = sample(); measured.velocity[0] = .001;
  ASSERT_TRUE(monitor.observe(measured, converter, now));
  ASSERT_TRUE(monitor.observe_receiver(receiver(c, now), now));
  EXPECT_EQ(monitor.state(now, 0, 0, {}).hal_state, State::HAL_ACTIVE);
  measured.velocity.fill(0.); measured.header.stamp.nanosec += 1000000;
  ASSERT_TRUE(monitor.observe(measured, converter, now + 1000000));
  EXPECT_EQ(monitor.state(now + 1000000, 0, 0, {}).hal_state, State::HAL_READY);
  measured.velocity_valid = false; measured.header.stamp.nanosec += 1000000;
  ASSERT_TRUE(monitor.observe(measured, converter, now + 2000000));
  EXPECT_EQ(monitor.state(now + 2000000, 0, 0, {}).hal_state, State::HAL_ACTIVE);
}
}  // namespace

TEST(Fr3Native, InvalidReceiverValuesCannotArmOrRefreshHal)
{
  const auto c = config(); HalConverter converter(c);
  ASSERT_TRUE(converter.initialize(urdf()));
  HalStateMonitor monitor(c); const int64_t now = 1000000000;
  ASSERT_TRUE(monitor.observe(sample(), converter, now));
  auto bad = receiver(c, now);
  bad.commanded_velocities[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(monitor.observe_receiver(bad, now));
  EXPECT_FALSE(monitor.healthy(now));
  auto valid = receiver(c, now);
  ASSERT_TRUE(monitor.observe_receiver(valid, now));
  bad = receiver(c, now + c.lost_ns);
  bad.commanded_positions[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(monitor.observe_receiver(bad, now + c.lost_ns));
  EXPECT_TRUE(monitor.receiver_lost(now + c.lost_ns));
}

TEST(Scenario4, CartesianMotionUsesVectorLimitsAndUnequalTimeIntervals)
{
  auto c=config();geometry_msgs::msg::Pose previous,candidate;
  std::array<double,3> velocity{};
  candidate.position.x=.00005;candidate.position.y=.00005;
  EXPECT_TRUE(pac_hal::check_tcp_trajectory(c,candidate,previous,{.05,.05,0},.001,.001,velocity));
  candidate.position.x=.00008;candidate.position.y=.00008;
  EXPECT_EQ(pac_hal::check_tcp_trajectory(c,candidate,previous,{.08,.08,0},.001,.001,velocity).code,pac_hal::Error::E_OUT_OF_RANGE);
  candidate.position.x=.0000021;candidate.position.y=0;
  EXPECT_EQ(pac_hal::check_tcp_trajectory(c,candidate,previous,{.0001,0,0},.003,.001,velocity).code,pac_hal::Error::E_OUT_OF_RANGE);
  candidate.position.x=.0000012;
  EXPECT_TRUE(pac_hal::check_tcp_trajectory(c,candidate,previous,{.0001,0,0},.003,.001,velocity));
  EXPECT_NEAR(velocity[0],.0004,1e-15);
  EXPECT_EQ(pac_hal::check_tcp_trajectory(c,candidate,previous,{0,0,0},0,.001,velocity).code,pac_hal::Error::E_FORMAT);
  candidate.position.x=std::numeric_limits<double>::infinity();
  EXPECT_EQ(pac_hal::check_tcp_trajectory(c,candidate,previous,{0,0,0},.001,.001,velocity).code,pac_hal::Error::E_OUT_OF_RANGE);
}

TEST(Scenario4, NativeProfileReadsAndRejectsInvalidCartesianLimits)
{
  auto yaml=YAML::LoadFile(FR3_CONFIG);
  auto path=std::filesystem::temp_directory_path()/("pac_tcp_limits_"+std::to_string(getpid())+".yaml");
  auto save=[&]() {std::ofstream file(path);file<<yaml;};
  yaml["workspace"]["max_tcp_speed_m_s"]=.001;
  yaml["workspace"]["max_tcp_acceleration_m_s2"]=.01;save();
  const auto c=pac_hal::load_robot_config(path.string());
  EXPECT_DOUBLE_EQ(c.max_tcp_speed,.001);EXPECT_DOUBLE_EQ(c.max_tcp_acceleration,.01);
  for(const double value : {0.,-1.,std::numeric_limits<double>::infinity()}) {
    yaml["workspace"]["max_tcp_speed_m_s"]=value;save();
    EXPECT_THROW(pac_hal::load_robot_config(path.string()),std::runtime_error);
  }
  yaml["workspace"]["max_tcp_speed_m_s"]=.001;
  yaml["workspace"]["max_tcp_acceleration_m_s2"]=0.;save();
  EXPECT_THROW(pac_hal::load_robot_config(path.string()),std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Scenario4, InvalidFirstFeedbackIsFaultAndMeasuredTcpSpeedMustRespectProfile)
{
  const auto c=config();HalConverter converter(c);ASSERT_TRUE(converter.initialize(urdf()));
  HalStateMonitor monitor(c);const int64_t now=1000000000;
  auto m=sample();m.velocity[1]=.4;
  geometry_msgs::msg::Pose pose;geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(m,pose,twist));
  ASSERT_GT(std::hypot(twist.linear.x,twist.linear.y,twist.linear.z),c.max_tcp_speed);
  EXPECT_FALSE(monitor.observe(m,converter,now));
  EXPECT_TRUE(monitor.measurement_rejected());
  EXPECT_EQ(monitor.state(now,0,0,{}).hal_state,State::HAL_FAULT);
  EXPECT_FALSE(monitor.state(now,0,0,{}).pose_valid);
  EXPECT_NE(monitor.measurement_failure_detail().find("measured TCP speed"),std::string::npos);
  ASSERT_TRUE(monitor.observe(sample(),converter,now+1000));
  auto bad=receiver(c,now+1000);bad.commanded_velocities[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(monitor.observe_receiver(bad,now+1000));
  EXPECT_EQ(monitor.state(now+1000,0,0,{}).hal_state,State::HAL_FAULT);
}

TEST(Scenario4, FreshInvalidReceiverImmediatelyDisarmsAndRequiresFreshOwnRecovery)
{
  const auto c=config();HalConverter converter(c);ASSERT_TRUE(converter.initialize(urdf()));
  const int64_t now=1000000000;
  for(int kind=0;kind<7;++kind) {
    HalStateMonitor monitor(c);ASSERT_TRUE(monitor.observe(sample(),converter,now));
    auto valid=receiver(c,now);valid.last_applied_command_seq=2;valid.applied_mono_ns=now-10;
    ASSERT_TRUE(monitor.observe_receiver(valid,now));ASSERT_TRUE(monitor.healthy(now));
    auto bad=valid;bad.sampled_mono_ns=now+1000;
    if(kind==0) {bad.joint_count=6;}
    if(kind==1) {bad.commanded_positions[0]=std::numeric_limits<double>::quiet_NaN();}
    if(kind==2) {bad.commanded_velocities[0]=std::numeric_limits<double>::infinity();}
    if(kind==3) {bad.applied_mono_ns=now+1001;}
    if(kind==4) {bad.applied_mono_ns=0;}
    if(kind==5) {bad.last_applied_command_seq=1;}
    if(kind==6) {bad.applied_mono_ns=now-11;}
    EXPECT_FALSE(monitor.observe_receiver(bad,now+1000));
    EXPECT_TRUE(monitor.receiver_feedback_rejected());EXPECT_FALSE(monitor.healthy(now+1000));
    EXPECT_EQ(monitor.state(now+1000,0,0,{}).hal_state,State::HAL_FAULT);
    EXPECT_NE(monitor.receiver_feedback_failure_detail().find("invalid receiver feedback"),std::string::npos);
    auto measured=sample();measured.header.stamp.nanosec=1;
    ASSERT_TRUE(monitor.observe(measured,converter,now+2000));
    EXPECT_TRUE(monitor.receiver_feedback_rejected());
    auto foreign=valid;foreign.session_id=43;foreign.sampled_mono_ns=now+3000;
    EXPECT_FALSE(monitor.observe_receiver(foreign,now+3000));
    EXPECT_TRUE(monitor.receiver_feedback_rejected());
    EXPECT_FALSE(monitor.observe_receiver(valid,now+3000)); // Old ACK cannot clear.
    valid.sampled_mono_ns=now+4000;ASSERT_TRUE(monitor.observe_receiver(valid,now+4000));
    EXPECT_FALSE(monitor.receiver_feedback_rejected());EXPECT_TRUE(monitor.healthy(now+4000));
  }
}

TEST(Fr3Native, InvalidReceiverAckTimeIsNotAccepted)
{
  const auto c = config(); HalStateMonitor monitor(c);
  const int64_t now = 1000000000;
  auto ack = receiver(c, now); ack.last_applied_command_seq = 3;
  ack.applied_mono_ns = now + 1;
  EXPECT_FALSE(monitor.observe_receiver(ack, now));
  ack.applied_mono_ns = 0;
  EXPECT_FALSE(monitor.observe_receiver(ack, now));
  ack.applied_mono_ns = now;
  EXPECT_TRUE(monitor.observe_receiver(ack, now));
}

TEST(Fr3Native, BudgetAndTtlStopIkBeforeConsumingIterations)
{
  auto c = config(); c.ik_budget_ns = 1; c.ik_iterations = 100; c.ik_budget = pac_hal::IkBudget::Wall;
  HalConverter converter(c); ASSERT_TRUE(converter.initialize(urdf()));
  geometry_msgs::msg::Pose pose; geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(sample(), pose, twist)); pose.position.x += .02;
  pac_hal::Joints output; output.fill(123.);
  EXPECT_EQ(converter.inverse(pose, sample().position, output).code, pac_hal::Error::E_IK_FAILED);
  EXPECT_EQ(converter.last_ik_steps(), 0U);
  EXPECT_EQ(output[0], 123.);  // No partial solution becomes a robot target.
  EXPECT_EQ(converter.inverse(pose, sample().position, output, 1).code, pac_hal::Error::E_EXPIRED);
  EXPECT_EQ(converter.last_ik_steps(), 0U);
}

namespace {
TEST(ExecutionContract, LatestOnlyNoCatchUpOrTimestampRefresh) {
  pac_hal::LatestCommandRate rate(2000000);
  pac_hal::Command a,b,c; a.command_seq=1;a.created_mono_ns=100;a.expires_mono_ns=5000100;
  pac_hal::TraceRecord trace;trace.t1=100;
  EXPECT_FALSE(rate.offer(a,trace));auto first=rate.take(100);ASSERT_TRUE(first);EXPECT_EQ(first->command.command_seq,1u);
  b=a;b.command_seq=2;c=a;c.command_seq=3;
  EXPECT_FALSE(rate.offer(b,trace));auto replaced=rate.offer(c,trace);ASSERT_TRUE(replaced);EXPECT_EQ(replaced->command.command_seq,2u);
  EXPECT_FALSE(rate.take(2000000));auto latest=rate.take(2000100);ASSERT_TRUE(latest);EXPECT_EQ(latest->command.command_seq,3u);
  EXPECT_EQ(latest->command.created_mono_ns,100);EXPECT_EQ(latest->command.expires_mono_ns,5000100);
  rate.completed(2100000);rate.offer(c,trace);
  EXPECT_FALSE(rate.take(4100000-1));ASSERT_TRUE(rate.take(4100000));
  EXPECT_FALSE(rate.take(9000000)); // no replay even after a scheduling stall
  rate.offer(c,trace);ASSERT_TRUE(rate.cancel());EXPECT_FALSE(rate.take(9000000));
}
TEST(ExecutionContract, GenericProfileAndCapacityAreChecked) {
  auto n=YAML::LoadFile(FR3_CONFIG);n["robot"]["model"]="third_test";n["robot"]["adapter"]="generic_gazebo";
  auto p=std::filesystem::temp_directory_path()/("pac_generic_"+std::to_string(getpid())+".yaml");
  auto save=[&](){std::ofstream f(p);f<<n;};save();
  auto c=pac_hal::load_robot_config(p.string());EXPECT_EQ(c.joint_count,7u);
  pac_hal::HalConverter k(c);ASSERT_TRUE(k.initialize(urdf()));
  n["robot"]["joint_count"]=8;save();EXPECT_THROW(pac_hal::load_robot_config(p.string()),std::runtime_error);
  n["robot"]["joint_count"]=7;n["execution"]["capabilities"]["state_feedback"]=false;save();EXPECT_THROW(pac_hal::load_robot_config(p.string()),std::runtime_error);
  std::filesystem::remove(p);
}
TEST(ExecutionContract, RepresentativeTasksMustPassIkAndCollision) {
  auto c=config();c.ik_budget_ns=20000000;
  pac_hal::HalConverter k(c);ASSERT_TRUE(k.initialize(urdf()));
  pac_hal::SceneGuard scene(c,FR3_SCENE);ASSERT_TRUE(scene.initialize(urdf()));
  auto m=sample();
  m.position={-0.1915941617730299, -0.3611638851715237, 0.12164291046410534,
    -1.4187463010169854, 0.04915708020230413, 1.060542641166919, -0.05370596034279201};
  ASSERT_TRUE(pac_hal::preflight(c,k,scene,m));
  geometry_msgs::msg::Pose p;p.position.x=100;p.orientation.w=1;c.task.samples={p};
  EXPECT_EQ(pac_hal::preflight(c,k,scene,m).code,pac_hal::Error::E_OUT_OF_RANGE);
  c.task.min_dof=8;EXPECT_EQ(pac_hal::check_task(c).code,pac_hal::Error::E_UNSUPPORTED);
}
}

TEST(ExecutionContract, ReceiverAckCannotMoveBackwards) {
  auto c=config();HalStateMonitor monitor(c);
  auto r=receiver(c,1000);r.last_applied_command_seq=2;r.applied_mono_ns=900;
  ASSERT_TRUE(monitor.observe_receiver(r,1000));
  r.sampled_mono_ns=1100;r.last_applied_command_seq=1;
  EXPECT_FALSE(monitor.observe_receiver(r,1100));
  r.last_applied_command_seq=2;r.applied_mono_ns=899;
  EXPECT_FALSE(monitor.observe_receiver(r,1100));
  r.applied_mono_ns=900;EXPECT_TRUE(monitor.observe_receiver(r,1100));
}

TEST(Scenario4, RejectedReplaysAndOtherSessionsCannotRegressProgress) {
  uint64_t seq=100;uint8_t result=pac_hal::State::CMD_SENT;pac_hal::Command c;
  c.session_id=42;c.command_seq=1;
  pac_hal::record_command_progress(c,42,pac_hal::State::CMD_REJECTED,seq,result);
  EXPECT_EQ(seq,100u);EXPECT_EQ(result,pac_hal::State::CMD_SENT);
  c.session_id=43;c.command_seq=1000000;
  pac_hal::record_command_progress(c,42,pac_hal::State::CMD_REJECTED,seq,result);
  EXPECT_EQ(seq,100u);EXPECT_EQ(result,pac_hal::State::CMD_SENT);
  c.session_id=42;c.command_seq=101;
  pac_hal::record_command_progress(c,42,pac_hal::State::CMD_REJECTED,seq,result);
  EXPECT_EQ(seq,101u);EXPECT_EQ(result,pac_hal::State::CMD_REJECTED);
  c.command_seq=102;
  pac_hal::record_command_progress(c,42,pac_hal::State::CMD_SENT,seq,result);
  EXPECT_EQ(seq,102u);EXPECT_EQ(result,pac_hal::State::CMD_SENT);
}

TEST(Scenario4, RejectedAttemptIdentityCannotBeRevived) {
  std::optional<uint64_t> observed;pac_hal::Command c;
  c.session_id=42;c.command_seq=0;
  EXPECT_EQ(pac_hal::observe_command_sequence(c,42,observed).code,pac_hal::Error::E_FORMAT);
  EXPECT_FALSE(observed);
  c.command_seq=10;ASSERT_TRUE(pac_hal::observe_command_sequence(c,42,observed));
  // Semantic rejection happens after this gate; changing the payload cannot
  // re-arm the same identity. Neither another session nor an old seq advances it.
  c.target_pose.position.y=.02;
  EXPECT_EQ(pac_hal::observe_command_sequence(c,42,observed).code,pac_hal::Error::E_FORMAT);
  c.target_pose.position.y=0.;
  EXPECT_EQ(pac_hal::observe_command_sequence(c,42,observed).code,pac_hal::Error::E_FORMAT);
  c.session_id=43;c.command_seq=1000000;
  EXPECT_EQ(pac_hal::observe_command_sequence(c,42,observed).code,pac_hal::Error::E_FORMAT);
  EXPECT_EQ(*observed,10u);
  c.session_id=42;c.command_seq=9;
  EXPECT_EQ(pac_hal::observe_command_sequence(c,42,observed).code,pac_hal::Error::E_FORMAT);
  c.command_seq=11;EXPECT_TRUE(pac_hal::observe_command_sequence(c,42,observed));
  EXPECT_EQ(*observed,11u);
}

TEST(Scenario4, ReceiverRejectReasonsMatchActualPacketGuards) {
  using R=pac_hal::ReceiverReject;
  intelion_interfaces::msg::HalJointCommand m;
  m.session_id=42;m.command_seq=1;m.joint_count=7;
  m.created_mono_ns=100;m.expires_mono_ns=5000100;
  const auto valid=m;std::vector<double> lower(7,-2.),upper(7,2.);
  auto check=[&]() {return pac_hal::check_receiver_packet(m,42,7,1000,5000000,lower,upper);};
  EXPECT_EQ(check().reason,R::None);
  m.session_id=43;EXPECT_EQ(check().reason,R::Session);
  m=valid;m.joint_count=6;EXPECT_EQ(check().reason,R::JointCount);
  m=valid;m.command_seq=0;EXPECT_EQ(check().reason,R::Sequence);
  m=valid;m.created_mono_ns=1001;EXPECT_EQ(check().reason,R::Clock);
  m=valid;m.expires_mono_ns=5000101;EXPECT_EQ(check().reason,R::Ttl);
  m=valid;m.created_mono_ns=0;m.expires_mono_ns=5000000;
  EXPECT_EQ(pac_hal::check_receiver_packet(m,42,7,5000000,5000000,lower,upper).reason,R::Expired);
  m=valid;m.positions[3]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(check().reason,R::Nonfinite);EXPECT_EQ(check().axis,3);
  m=valid;m.positions[5]=2.01;
  EXPECT_EQ(check().reason,R::Range);EXPECT_EQ(check().axis,5);
  EXPECT_STREQ(pac_hal::receiver_reason(check().reason),"joint_out_of_range");
  m=valid;m.joint_count=6;lower.resize(6);upper.resize(6);
  EXPECT_EQ(pac_hal::check_receiver_packet(m,42,6,1000,5000000,lower,upper).reason,R::None);
}

TEST(Scenario4, ForeignAndReplayedPacketsCannotChangeDeviceHealthContext) {
  pac_hal::Command context,c;bool seen=false;
  c.session_id=43;c.command_seq=1000000;c.header.frame_id="foreign_frame";
  pac_hal::record_active_command_context(c,42,"map",context,seen);
  EXPECT_FALSE(seen);
  c.session_id=42;c.command_seq=0;
  pac_hal::record_active_command_context(c,42,"map",context,seen);
  EXPECT_FALSE(seen);
  c.command_seq=10;
  pac_hal::record_active_command_context(c,42,"map",context,seen);
  ASSERT_TRUE(seen);EXPECT_EQ(context.session_id,42u);EXPECT_EQ(context.command_seq,10u);
  EXPECT_EQ(context.header.frame_id,"map");
  c.session_id=43;c.command_seq=1000000;
  pac_hal::record_active_command_context(c,42,"map",context,seen);
  c.session_id=42;c.command_seq=1;
  pac_hal::record_active_command_context(c,42,"map",context,seen);
  EXPECT_EQ(context.session_id,42u);EXPECT_EQ(context.command_seq,10u);
  c.command_seq=11;
  pac_hal::record_active_command_context(c,42,"map",context,seen);
  EXPECT_EQ(context.command_seq,11u);EXPECT_EQ(context.header.frame_id,"map");
}

TEST(Scenario4, InvalidMeasuredJointsDisarmImmediatelyAndRequireFreshRecovery) {
  const auto c=config();HalConverter converter(c);ASSERT_TRUE(converter.initialize(urdf()));
  for(int kind=0;kind<5;++kind) {
    HalStateMonitor monitor(c);const int64_t now=1000000000;
    ASSERT_TRUE(monitor.observe(sample(),converter,now));
    ASSERT_TRUE(monitor.observe_receiver(receiver(c,now),now));
    const auto first=monitor.state(now,0,0,{});
    auto bad=sample();bad.header.stamp.sec=2;
    if(kind==0) {bad.position[0]+=2*std::acos(-1.);}
    if(kind==1) {bad.position[0]=std::numeric_limits<double>::quiet_NaN();}
    if(kind==2) {bad.velocity[0]=c.axes[0].velocity+.01;}
    if(kind==3) {bad.velocity[0]=std::numeric_limits<double>::infinity();}
    if(kind==4) {bad.layout_valid=false;}
    EXPECT_FALSE(monitor.observe(bad,converter,now+1000));
    EXPECT_TRUE(monitor.measurement_rejected());EXPECT_FALSE(monitor.healthy(now+1000));
    auto invalid=monitor.state(now+1000,0,0,{});
    EXPECT_FALSE(invalid.pose_valid);EXPECT_FALSE(invalid.twist_valid);
    EXPECT_EQ(invalid.hal_state,State::HAL_FAULT);EXPECT_EQ(invalid.state_seq,first.state_seq);
    EXPECT_EQ(invalid.received_mono_ns,first.received_mono_ns);
    ASSERT_TRUE(monitor.observe_receiver(receiver(c,now+2000),now+2000));
    EXPECT_FALSE(monitor.healthy(now+2000)); // ACK cannot clear bad measurement.
    EXPECT_FALSE(monitor.observe(sample(),converter,now+3000)); // Replay cannot clear.
    auto restored=sample();restored.header.stamp.nanosec=1;
    ASSERT_TRUE(monitor.observe(restored,converter,now+4000));
    EXPECT_FALSE(monitor.measurement_rejected());EXPECT_TRUE(monitor.healthy(now+4000));
    EXPECT_TRUE(monitor.state(now+4000,0,0,{}).pose_valid);
  }
}

TEST(Policy1008, LongGapRestartsMotionBaselineFromRest) {
  const auto c=config();
  // Normal stream keeps the measured interval and the previous interval.
  auto b=pac_hal::motion_baseline(true,1002000000,1001000000,.001,c.period_ns,c.receiver_watchdog_ns);
  EXPECT_FALSE(b.restarted);EXPECT_DOUBLE_EQ(b.dt,.001);EXPECT_DOUBLE_EQ(b.previous_dt,.001);
  // First command still starts at the measured pose over one period.
  b=pac_hal::motion_baseline(false,5,0,0.,c.period_ns,c.receiver_watchdog_ns);
  EXPECT_FALSE(b.restarted);EXPECT_DOUBLE_EQ(b.dt,.001);
  // Non-increasing creation time remains a FORMAT rejection downstream.
  b=pac_hal::motion_baseline(true,1000,1000,.001,c.period_ns,c.receiver_watchdog_ns);
  EXPECT_FALSE(b.restarted);EXPECT_LE(b.dt,0.);
  geometry_msgs::msg::Pose previous,candidate;previous.orientation.w=candidate.orientation.w=1;
  std::array<double,3> velocity{};
  // 10s idle: the old rule evaluated a 0.5m jump over 10s and accepted it.
  candidate.position.x=.5;
  EXPECT_TRUE(pac_hal::check_tcp_trajectory(c,candidate,previous,{0,0,0},10.,.001,velocity));
  b=pac_hal::motion_baseline(true,10001000000,1000000,.001,c.period_ns,c.receiver_watchdog_ns);
  ASSERT_TRUE(b.restarted);EXPECT_DOUBLE_EQ(b.dt,c.receiver_watchdog_ns*1e-9);
  EXPECT_EQ(pac_hal::check_tcp_trajectory(c,candidate,previous,{0,0,0},b.dt,b.previous_dt,velocity).code,
    pac_hal::Error::E_OUT_OF_RANGE);
  // After a fast stream was stopped, a stationary hold must not wait ~2v/a seconds.
  candidate.position.x=0;
  b=pac_hal::motion_baseline(true,1000000+c.receiver_watchdog_ns,1000000,.001,c.period_ns,c.receiver_watchdog_ns);
  ASSERT_TRUE(b.restarted);
  EXPECT_EQ(pac_hal::check_tcp_trajectory(c,candidate,previous,{.1,0,0},b.dt,b.previous_dt,velocity).code,
    pac_hal::Error::E_OUT_OF_RANGE);  // old velocity kept: rejected
  EXPECT_TRUE(pac_hal::check_tcp_trajectory(c,candidate,previous,{0,0,0},b.dt,b.previous_dt,velocity));
  // A small braking shortfall (0.5mm) from rest is still accepted.
  candidate.position.x=.0005;
  EXPECT_TRUE(pac_hal::check_tcp_trajectory(c,candidate,previous,{0,0,0},b.dt,b.previous_dt,velocity));
}

TEST(Policy1008, ReceiverExpiryDiagnosticParsesExactIdentity) {
  uint64_t session=0,seq=0;
  EXPECT_TRUE(pac_hal::parse_receiver_expiry("{\"active_session_id\":42,\"packet_session_id\":42,\"command_seq\":441882,"
    "\"received_mono_ns\":9,\"created_mono_ns\":1,\"expires_mono_ns\":5,\"reason\":\"expired_at_receiver\",\"axis\":-1}",session,seq));
  EXPECT_EQ(session,42u);EXPECT_EQ(seq,441882u);
  EXPECT_FALSE(pac_hal::parse_receiver_expiry("{\"packet_session_id\":42,\"command_seq\":7,\"reason\":\"session_mismatch\"}",session,seq));
  EXPECT_FALSE(pac_hal::parse_receiver_expiry("{\"packet_session_id\":42,\"command_seq\":0,\"reason\":\"expired_at_receiver\"}",session,seq));
  EXPECT_FALSE(pac_hal::parse_receiver_expiry("{\"packet_session_id\":-1,\"command_seq\":7,\"reason\":\"expired_at_receiver\"}",session,seq));
  EXPECT_FALSE(pac_hal::parse_receiver_expiry("{\"command_seq\":7x,\"packet_session_id\":42,\"reason\":\"expired_at_receiver\"}",session,seq));
  EXPECT_FALSE(pac_hal::parse_receiver_expiry("garbage",session,seq));
}

TEST(Policy1008, ProfilesLoadReceiverWatchdogAndIkBudgetDefault) {
  const auto c=config();
  EXPECT_EQ(c.receiver_watchdog_ns,100000000);
  EXPECT_EQ(pac_hal::Config{}.ik_budget,pac_hal::IkBudget::Iterations);
}

namespace {
// Replays a B-like slow TCP approach through the production IK and the same
// joint finite-difference rule used by the gateway. Returns max |accel| per joint.
struct IkReplay {std::array<double,7> max_accel{};std::array<double,7> cruise_max_accel{};std::array<double,7> max_step_noise{};int over{0};int steps{0};double max_tcp_residual{0};};
IkReplay replay_fr3_approach(bool irregular_dt) {
  auto c=config();c.ik_budget_ns=100000000;
  HalConverter converter(c);
  if(!converter.initialize(urdf())) {ADD_FAILURE()<<"init";return {};}
  Measurement start=sample();
  start.position={0.5217159956713354,-0.887787653263612,-0.08295272477974769,
    -1.614392028894036,-0.09646110181032354,0.730606472727573,0.3973491336150268};
  geometry_msgs::msg::Pose pose;geometry_msgs::msg::Twist twist;
  if(!converter.forward(start,pose,twist)) {ADD_FAILURE()<<"fk";return {};}
  const std::array<double,3> a{pose.position.x,pose.position.y,pose.position.z};
  const std::array<double,3> b{.35,0.,.81};
  const double length=std::sqrt(std::pow(b[0]-a[0],2)+std::pow(b[1]-a[1],2)+std::pow(b[2]-a[2],2));
  const double v=.02,acc=.05;const double ta=v/acc;
  const double total=length/v+ta;
  auto s_of=[&](double t){
    if(t<=0) return 0.;
    if(t<ta) return .5*acc*t*t;
    if(t<total-ta) return .5*acc*ta*ta+v*(t-ta);
    if(t<total) {const double r=total-t;return length-.5*acc*r*r;}
    return length;};
  std::mt19937 rng(7);std::uniform_real_distribution<double> jitter(.0006,.0014);
  IkReplay out;
  pac_hal::Joints previous=start.position,velocity{};double previous_dt=.001;double t=0;bool first=true;
  while(t<total+.2) {
    const double dt=irregular_dt ? jitter(rng) : .001;t+=dt;
    const double s=s_of(t)/length;
    geometry_msgs::msg::Pose target=pose;
    target.position.x=a[0]+(b[0]-a[0])*s;target.position.y=a[1]+(b[1]-a[1])*s;target.position.z=a[2]+(b[2]-a[2])*s;
    pac_hal::Joints q{};
    if(!converter.inverse(target,previous,q)) {ADD_FAILURE()<<"ik at t="<<t;return out;}
    geometry_msgs::msg::Pose check;geometry_msgs::msg::Twist unused;Measurement m=start;m.position=q;
    converter.forward(m,check,unused);
    out.max_tcp_residual=std::max(out.max_tcp_residual,std::hypot(check.position.x-target.position.x,
      check.position.y-target.position.y,check.position.z-target.position.z));
    for(size_t i=0;i<7;++i) {
      const double vel=(q[i]-previous[i])/dt;
      const double accel=first?0.:2*(vel-velocity[i])/(dt+previous_dt);
      out.max_accel[i]=std::max(out.max_accel[i],std::abs(accel));
      if(t>ta+.1 && t<total-ta-.1) {out.cruise_max_accel[i]=std::max(out.cruise_max_accel[i],std::abs(accel));}
      if(std::abs(accel)>c.axes[i].acceleration+1e-5) {++out.over;}
      velocity[i]=vel;
    }
    first=false;previous=q;previous_dt=dt;++out.steps;
  }
  return out;
}
}

TEST(Policy1008, Fr3SlowApproachIkHasNoFalseJointAccelerationSpikes) {
  // Team-cell FR3 approach at v=.02m/s,a=.05m/s^2 (the 10/08 demo setting).
  // Before the fix, KDL treated orientation errors below 1e-6rad as zero; the
  // error drifted and was fixed in one step, so joint5 showed 0.57 (regular dt)
  // and 1.08 rad/s^2 (B-like 0.6..1.4ms dt) in the constant-speed segment.
  for(bool irregular : {false,true}) {
    const auto r=replay_fr3_approach(irregular);
    EXPECT_EQ(r.over,0)<<"irregular_dt="<<irregular;
    EXPECT_LT(r.max_tcp_residual,1e-9);
    for(size_t i=0;i<7;++i) {
      EXPECT_LT(r.cruise_max_accel[i],.05)<<"joint "<<i+1<<" irregular_dt="<<irregular;
    }
  }
}

TEST(Policy1008, IterationBudgetIgnoresClockStallsButKeepsTtlAndIterationCap) {
  auto c=config();c.ik_budget=pac_hal::IkBudget::Iterations;c.ik_budget_ns=1;  // a clock budget would fail
  HalConverter converter(c);ASSERT_TRUE(converter.initialize(urdf()));
  geometry_msgs::msg::Pose pose;geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(converter.forward(sample(),pose,twist));pac_hal::Joints output;
  pose.position.x+=.0001;
  EXPECT_TRUE(converter.inverse(pose,sample().position,output));
  // The absolute command expiry is still enforced.
  EXPECT_EQ(converter.inverse(pose,sample().position,output,1).code,pac_hal::Error::E_EXPIRED);
  // A target needing more Newton steps than the profile allows is still rejected.
  c.ik_iterations=1;HalConverter capped(c);ASSERT_TRUE(capped.initialize(urdf()));
  pose.position.x+=.05;
  const auto r=capped.inverse(pose,sample().position,output);
  EXPECT_EQ(r.code,pac_hal::Error::E_IK_FAILED);
  EXPECT_NE(r.detail.find("steps=1/1"),std::string::npos)<<r.detail;
}

TEST(Policy1008, ProcessCorridorTaskProfileIsRefusedBeforeStart) {
  auto profile=YAML::LoadFile(FR3_CONFIG);
  profile["execution"]["task"]["tcp_corridor"]["radius_m"]=.01;
  const auto path=std::filesystem::temp_directory_path()/"pac_hal_corridor_refusal.yaml";
  {std::ofstream out(path);out<<profile;}
  EXPECT_THROW(pac_hal::load_robot_config(path.string()),std::runtime_error);
  std::filesystem::remove(path);
  const auto c=config();
  EXPECT_EQ(YAML::Load(pac_hal::describe_task_contract(c))["input_semantics"].as<std::string>(),"absolute_tcp_pose");
}
