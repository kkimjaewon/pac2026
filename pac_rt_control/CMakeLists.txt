cmake_minimum_required(VERSION 3.16)
project(rt_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(intelion_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# 1ms 실시간 루프 + gate + blend_policy + motion_limiter + safety_mode + cycle_tracer + arrival + warmup + init_hold 라이브러리 (ROS 의존 없음)
add_library(rt_control STATIC
  src/rt_control_loop.cpp src/gate.cpp src/blend_policy.cpp src/motion_limiter.cpp src/safety_mode.cpp
  src/cycle_tracer.cpp src/env_check.cpp src/run_dir.cpp src/arrival.cpp src/warmup.cpp src/init_hold.cpp)
target_include_directories(rt_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(rt_control PRIVATE -Wall -Wextra)
target_link_libraries(rt_control PUBLIC Threads::Threads)

# 지터 측정 데모
add_executable(rt_loop_demo src/rt_loop_demo.cpp)
target_compile_options(rt_loop_demo PRIVATE -Wall -Wextra)
target_link_libraries(rt_loop_demo PRIVATE rt_control)

# B 노드: A 명령 수신기 + 1ms 루프 + C와 잇는 ros_bridge
add_executable(rt_control_node src/rt_control_node.cpp src/ai_command_receiver.cpp src/ros_bridge.cpp)
target_compile_options(rt_control_node PRIVATE -Wall -Wextra)
target_link_libraries(rt_control_node PRIVATE
  rt_control
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  ${intelion_interfaces_TARGETS})

# ros2 run rt_control <실행파일> 로 실행할 수 있게 lib/rt_control 아래에 설치
install(TARGETS rt_loop_demo rt_control_node DESTINATION lib/${PROJECT_NAME})
# 가짜 C (rclpy): ros2 run rt_control fake_c_node.py
install(PROGRAMS scripts/fake_c_node.py DESTINATION lib/${PROJECT_NAME})
# launch 파일과 파라미터 yaml: ros2 launch rt_control rt_control.launch.py
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 단위 테스트: colcon test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gate tests/test_gate.cpp)
  target_link_libraries(test_gate rt_control)
  ament_add_gtest(test_blend_policy tests/test_blend_policy.cpp)
  target_link_libraries(test_blend_policy rt_control)
  ament_add_gtest(test_motion_limiter tests/test_motion_limiter.cpp)
  target_link_libraries(test_motion_limiter rt_control)
  ament_add_gtest(test_hal_command tests/test_hal_command.cpp)
  target_link_libraries(test_hal_command rt_control)
  ament_add_gtest(test_safety_mode tests/test_safety_mode.cpp)
  target_link_libraries(test_safety_mode rt_control)
  ament_add_gtest(test_cycle_tracer tests/test_cycle_tracer.cpp)
  target_link_libraries(test_cycle_tracer rt_control)
  ament_add_gtest(test_env_check tests/test_env_check.cpp)
  target_link_libraries(test_env_check rt_control)
  ament_add_gtest(test_run_dir tests/test_run_dir.cpp)
  target_link_libraries(test_run_dir rt_control)
  ament_add_gtest(test_arrival tests/test_arrival.cpp)
  target_link_libraries(test_arrival rt_control)
  ament_add_gtest(test_warmup tests/test_warmup.cpp)
  target_link_libraries(test_warmup rt_control)
  ament_add_gtest(test_init_hold tests/test_init_hold.cpp)
  target_link_libraries(test_init_hold rt_control)
endif()

ament_package()
