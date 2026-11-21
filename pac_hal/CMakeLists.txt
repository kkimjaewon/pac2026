cmake_minimum_required(VERSION 3.8)
project(pac_hal)
# Eigen IK in an unoptimized default build can exceed the 500us budget even
# with warm starts. Keep explicit Debug builds available; optimize by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "HAL build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(intelion_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fcl REQUIRED)
find_package(assimp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Common HAL core is independent of A/B and selects the model at startup.
add_library(pac_hal_core SHARED
  src/execution_contract.cpp src/hal_command_gateway.cpp src/robot_config.cpp src/hal_converter.cpp src/gazebo_adapter.cpp
  src/hal_state_monitor.cpp src/hal_latency_trace.cpp src/scene_guard.cpp)
target_compile_features(pac_hal_core PUBLIC cxx_std_17)
target_include_directories(pac_hal_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${orocos_kdl_INCLUDE_DIRS})
ament_target_dependencies(pac_hal_core rclcpp intelion_interfaces sensor_msgs std_msgs
  geometry_msgs kdl_parser urdf ament_index_cpp)
target_link_libraries(pac_hal_core yaml-cpp ${orocos_kdl_LIBRARIES} fcl assimp::assimp)
add_executable(hal_node src/hal_node.cpp)
target_link_libraries(hal_node pac_hal_core)
add_executable(hal_scene_check src/hal_scene_check.cpp)
target_link_libraries(hal_scene_check pac_hal_core)
# Preserve the previous CLI name for existing FR3 check scripts.
add_executable(fr3_scene_check src/hal_scene_check.cpp)
target_link_libraries(fr3_scene_check pac_hal_core)
install(TARGETS pac_hal_core LIBRARY DESTINATION lib)
install(TARGETS hal_node hal_scene_check fr3_scene_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
add_library(pac_hal_watchdog SHARED src/watchdog_controller.cpp)
target_compile_features(pac_hal_watchdog PUBLIC cxx_std_17)
ament_target_dependencies(pac_hal_watchdog rclcpp controller_interface hardware_interface
  pluginlib realtime_tools intelion_interfaces std_msgs)
target_include_directories(pac_hal_watchdog PRIVATE include)
pluginlib_export_plugin_description_file(controller_interface watchdog_plugins.xml)
install(TARGETS pac_hal_watchdog LIBRARY DESTINATION lib)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/hal_config_node scripts/hal_gazebo_node scripts/prepare_hal_controller scripts/prepare_workcell_service scripts/hal_demo_publisher scripts/hal_fault_probe scripts/fake_ai_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  execute_process(COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/generate_fr3_fixture.py
    ${CMAKE_CURRENT_BINARY_DIR}/fr3_test.urdf
    RESULT_VARIABLE fixture_result)
  if(NOT fixture_result EQUAL 0)
    message(FATAL_ERROR "Official FR3 test fixture generation failed")
  endif()
  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/config/robot_fr3.yaml test_profile)
  string(REPLACE "active_session_id: 0" "active_session_id: 42" test_profile "${test_profile}")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/fr3_test_config.yaml "${test_profile}")
  ament_add_gtest(test_fr3_native tests/test_fr3_native.cpp)
  target_link_libraries(test_fr3_native pac_hal_core)
  target_compile_definitions(test_fr3_native PRIVATE
    FR3_CONFIG="${CMAKE_CURRENT_BINARY_DIR}/fr3_test_config.yaml"
    FR3_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/config/fr3_workcell_scene.yaml"
    FR3_URDF="${CMAKE_CURRENT_BINARY_DIR}/fr3_test.urdf")
  # UR is an optional simulation target; FR3 checks stay enabled without it.
  option(PAC_HAL_BUILD_UR_TESTS "Enable UR tests when official UR packages are available" ON)
  set(pac_hal_test_ur 0)
  set(pac_hal_ur_available 1)
  if(PAC_HAL_BUILD_UR_TESTS)
    # Fixtures use Python's ament index, so availability must match that index.
    execute_process(COMMAND ${Python3_EXECUTABLE} -c
      "import sys; from ament_index_python.packages import get_packages_with_prefixes; p=get_packages_with_prefixes(); sys.exit(0 if all(n in p for n in ('ur_simulation_gz', 'ur_description')) else 1)"
      RESULT_VARIABLE pac_hal_ur_available)
  endif()
  if(PAC_HAL_BUILD_UR_TESTS AND pac_hal_ur_available EQUAL 0)
    set(pac_hal_test_ur 1)
    execute_process(COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/generate_ur_fixture.py
      ${CMAKE_CURRENT_BINARY_DIR}/ur5e_test.urdf RESULT_VARIABLE ur_fixture_result)
    if(NOT ur_fixture_result EQUAL 0)
      message(FATAL_ERROR "Official UR5e fixture generation failed")
    endif()
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/config/ur5e.yaml ur_test_profile)
    string(REPLACE "active_session_id: 0" "active_session_id: 43" ur_test_profile "${ur_test_profile}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ur5e_test_config.yaml "${ur_test_profile}")
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/config/ur5e_workcell.yaml ur_work_profile)
    string(REPLACE "active_session_id: 0" "active_session_id: 43" ur_work_profile "${ur_work_profile}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ur_work_config.yaml "${ur_work_profile}")
    ament_add_gtest(test_common_native tests/test_common_native.cpp)
    target_link_libraries(test_common_native pac_hal_core)
    target_compile_definitions(test_common_native PRIVATE
      UR_WORK_CONFIG="${CMAKE_CURRENT_BINARY_DIR}/ur_work_config.yaml"
      UR_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/config/ur5e_workcell_scene.yaml"
      UR_CONFIG="${CMAKE_CURRENT_BINARY_DIR}/ur5e_test_config.yaml"
      UR_URDF="${CMAKE_CURRENT_BINARY_DIR}/ur5e_test.urdf")
  else()
    message(STATUS "pac_hal: UR tests skipped (disabled or UR packages unavailable); FR3 tests remain enabled")
  endif()
  ament_add_pytest_test(test_log_contract tests/test_log_contract.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_execution tests/test_execution.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_contract tests/test_contract.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_workcell tests/test_workcell.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_kinematics tests/test_kinematics.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV PAC_HAL_TEST_UR=${pac_hal_test_ur})
endif()
ament_package()
