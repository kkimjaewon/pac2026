// Copyright 2026 Jaewon Kim
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "pac_hal/scene_guard.hpp"
#include "pac_hal/hal_converter.hpp"

pac_hal::Joints joints(std::string csv, size_t count)
{
  for (auto & ch : csv) {if (ch == ',' || ch == '[' || ch == ']') {ch = ' ';}}
  std::stringstream stream(csv);
  pac_hal::Joints q{};
  for (size_t i = 0; i < count; ++i) {if (!(stream >> q[i])) {throw std::runtime_error("configured joint count required");}}
  std::string extra;
  if (stream >> extra) {throw std::runtime_error("extra joint values");}
  return q;
}
int main(int argc, char ** argv)
{
  if (argc != 5 && argc != 6) {std::cerr << "config scene urdf initial_q [goal_q]\n"; return 2;}
  try {
    auto config = pac_hal::load_robot_config(argv[1]);
    pac_hal::SceneGuard guard(config, argv[2]);
    std::ifstream file(argv[3]); std::stringstream xml; xml << file.rdbuf();
    pac_hal::HalConverter converter(config);
    auto result = converter.initialize(xml.str());
    const auto initial = joints(argv[4], config.joint_count);
    if (result) {
      for (size_t i = 0; i < config.joint_count; ++i) {
        if (!std::isfinite(initial[i]) || initial[i] < converter.lower()[i] ||
          initial[i] > converter.upper()[i]) {
          result = {pac_hal::Error::E_OUT_OF_RANGE, "initial joint limit exceeded"}; break;
        }
      }
    }
    if (result) {result = guard.initialize(xml.str());}
    if (result) {result = guard.check(initial);}
    if (result && argc == 6) {result = guard.check_path(initial, joints(argv[5], config.joint_count));}
    geometry_msgs::msg::Pose pose; geometry_msgs::msg::Twist twist;
    pac_hal::Measurement sample; sample.position = joints(argv[4], config.joint_count);
    if (result) {result = converter.forward(sample, pose, twist);}
    std::cout << "{\"safe\":" << (result ? "true" : "false") << ",\"detail\":"
              << std::quoted(result.detail) << ",\"tcp\":[" << pose.position.x << ','
              << pose.position.y << ',' << pose.position.z << "],\"quaternion_xyzw\":["
              << pose.orientation.x << ',' << pose.orientation.y << ',' << pose.orientation.z
              << ',' << pose.orientation.w << "]}\n";
    return result ? 0 : 1;
  } catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 2;}
}
