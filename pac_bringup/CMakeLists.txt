cmake_minimum_required(VERSION 3.8)
project(pac_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/plate_mover.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(shared_contract test/test_shared_contract.py)
endif()

ament_package()
