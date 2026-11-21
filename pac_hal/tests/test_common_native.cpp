#include <fstream>
#include <sstream>
#include <limits>
#include "gtest/gtest.h"
#include "pac_hal/hal_converter.hpp"
#include "pac_hal/hal_state_monitor.hpp"
#include "pac_hal/joint_trajectory.hpp"
#include "pac_hal/scene_guard.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
namespace {
auto config() {auto c=pac_hal::load_robot_config(UR_CONFIG); c.logging=false; return c;}
std::string model() {std::ifstream f(UR_URDF); std::stringstream s; s<<f.rdbuf(); return s.str();}
pac_hal::Measurement sample() {pac_hal::Measurement m; m.position={0.,-1.2,1.2,-1.4,-1.57,.1,0.};m.velocity_valid=true;m.header.stamp.sec=1;return m;}
TEST(CommonNative, UrModelAndCount) {
 auto c=config(); EXPECT_EQ(c.joint_count,6u); EXPECT_EQ(c.model,"ur5e");
 EXPECT_THROW(pac_hal::load_fr3_config(UR_CONFIG),std::runtime_error);
 pac_hal::HalConverter k(c); ASSERT_TRUE(k.initialize(model()));
 auto wrong=c;wrong.joint_count=7;pac_hal::HalConverter other(wrong);EXPECT_FALSE(other.initialize(model()));
}
TEST(CommonNative, UrIkFkAndInactiveAxis) {
 auto c=config(); c.ik_budget_ns=100000000;
 pac_hal::HalConverter k(c);ASSERT_TRUE(k.initialize(model()));
 auto m=sample(); m.position[6]=std::numeric_limits<double>::quiet_NaN();
 geometry_msgs::msg::Pose pose;geometry_msgs::msg::Twist twist;
 ASSERT_TRUE(k.forward(m,pose,twist));pose.position.x+=.0001;
 pac_hal::Joints q{};ASSERT_TRUE(k.inverse(pose,m.position,q));EXPECT_EQ(q[6],0.);
 m.position=q;geometry_msgs::msg::Pose actual;ASSERT_TRUE(k.forward(m,actual,twist));
 EXPECT_NEAR(actual.position.x,pose.position.x,1e-6);
 EXPECT_NEAR(actual.position.y,pose.position.y,1e-6);
 EXPECT_NEAR(actual.position.z,pose.position.z,1e-6);
 EXPECT_EQ(k.inverse(pose,q,q,pac_hal::monotonic_ns()-1).code,pac_hal::Error::E_EXPIRED);
}
TEST(CommonNative, UrInstallationAndRotatedTcp) {
 auto c=config();c.flange_to_tcp.orientation.x=0.;c.flange_to_tcp.orientation.y=0.;c.flange_to_tcp.orientation.z=0.;c.flange_to_tcp.orientation.w=1.;auto m=sample();pac_hal::HalConverter k(c);ASSERT_TRUE(k.initialize(model()));
 geometry_msgs::msg::Pose p;geometry_msgs::msg::Twist t;ASSERT_TRUE(k.forward(m,p,t));
 c.reference_to_base.position.x=.2;c.reference_to_base.position.z=.15;
 // 180 degree installation yaw; local tool x follows the measured flange rotation.
 c.reference_to_base.orientation.z=1.;c.reference_to_base.orientation.w=0.;
 c.flange_to_tcp.position.x=.04;
 c.ik_budget_ns=100000000;pac_hal::HalConverter installed(c);ASSERT_TRUE(installed.initialize(model()));
 geometry_msgs::msg::Pose target;ASSERT_TRUE(installed.forward(m,target,t));
 const auto & q=p.orientation;
 EXPECT_NEAR(target.position.x,.2-p.position.x-.04*(1-2*(q.y*q.y+q.z*q.z)),1e-8);
 EXPECT_NEAR(target.position.y,-p.position.y-.04*2*(q.x*q.y+q.w*q.z),1e-8);
 EXPECT_NEAR(target.position.z,.15+p.position.z+.04*2*(q.x*q.z-q.w*q.y),1e-8);
 target.position.z+=.0001;pac_hal::Joints result{};ASSERT_TRUE(installed.inverse(target,m.position,result));
}
TEST(CommonNative, TcpAlignmentMatchesOfficialTool0) {
 auto c=config();pac_hal::HalConverter k(c);ASSERT_TRUE(k.initialize(model()));
 KDL::Tree tree;ASSERT_TRUE(kdl_parser::treeFromString(model(),tree));KDL::Chain chain;
 ASSERT_TRUE(tree.getChain(c.base,"tool0",chain));KDL::ChainFkSolverPos_recursive fk(chain);
 KDL::JntArray q(6);auto m=sample();for(size_t i=0;i<6;++i)q(i)=m.position[i];KDL::Frame independent;
 ASSERT_EQ(fk.JntToCart(q,independent),0);
 geometry_msgs::msg::Pose actual;geometry_msgs::msg::Twist twist;ASSERT_TRUE(k.forward(m,actual,twist));
 KDL::Frame observed(KDL::Rotation::Quaternion(actual.orientation.x,actual.orientation.y,actual.orientation.z,actual.orientation.w),KDL::Vector(actual.position.x,actual.position.y,actual.position.z));
 EXPECT_LT(KDL::diff(independent,observed).vel.Norm(),1e-10);
 EXPECT_LT(KDL::diff(independent,observed).rot.Norm(),1e-10);
}
TEST(CommonNative, UrSceneUsesLocalMeshAndRobotSubtree) {
 auto c=pac_hal::load_robot_config(UR_WORK_CONFIG);c.session=43;
 pac_hal::SceneGuard guard(c,UR_SCENE);ASSERT_TRUE(guard.initialize(model()));
 pac_hal::Joints q{3.5323157813303343,-1.1736688280581857,-1.826774093867833,-1.7119460582538688,1.570796326590547,-1.1800731988493458,0.};
 EXPECT_TRUE(guard.check(q));auto candidate=q;candidate[0]+=.0001;EXPECT_TRUE(guard.check_path(q,candidate));
 EXPECT_EQ(guard.check_path(q,candidate,pac_hal::monotonic_ns()-1).code,pac_hal::Error::E_EXPIRED);
 candidate[2]=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(guard.check_path(q,candidate));
 // Putting this same UR posture at floor-level must not bypass environmental collision.
 c.reference_to_base.position.z=0.;pac_hal::SceneGuard low(c,UR_SCENE);ASSERT_TRUE(low.initialize(model()));EXPECT_FALSE(low.check(q));
}
TEST(CommonNative, ReceiverCountFreshnessAndRealStop) {
 auto c=config();pac_hal::HalConverter k(c);ASSERT_TRUE(k.initialize(model()));
 pac_hal::HalStateMonitor monitor(c);auto m=sample();m.velocity[6]=100.;
 auto now=pac_hal::monotonic_ns();ASSERT_TRUE(monitor.observe(m,k,now));
 pac_hal::Receiver r;r.session_id=c.session;r.joint_count=7;r.active=true;r.sampled_mono_ns=now;
 EXPECT_FALSE(monitor.observe_receiver(r,now));r.joint_count=6;
 // The seventh slot is inactive capacity for UR, not a physical joint.
 r.commanded_velocities[6]=std::numeric_limits<double>::quiet_NaN();
 r.commanded_positions[6]=std::numeric_limits<double>::infinity();
 ASSERT_TRUE(monitor.observe_receiver(r,now));EXPECT_EQ(monitor.state(now,0,0,{}).hal_state,pac_hal::State::HAL_READY);
 m.header.stamp.sec=2;m.velocity[0]=.01;ASSERT_TRUE(monitor.observe(m,k,now+1));
 EXPECT_EQ(monitor.state(now+1,0,0,{}).hal_state,pac_hal::State::HAL_ACTIVE);
 EXPECT_TRUE(monitor.state(now+1+c.stale_ns,0,0,{}).stale);
}
TEST(CommonNative, ActiveJointLimitsStillEnforced) {
 auto c=config();pac_hal::Joints lower{},upper{},prev{},v{},goal{};
 for(size_t i=0;i<c.joint_count;++i){lower[i]=c.axes[i].lower;upper[i]=c.axes[i].upper;}
 goal[6]=100.; EXPECT_TRUE(pac_hal::check_joint_trajectory(c,lower,upper,goal,prev,v,.01,.01));
 goal[2]=upper[2]+.1;EXPECT_FALSE(pac_hal::check_joint_trajectory(c,lower,upper,goal,prev,v,.01,.01));
}
}
