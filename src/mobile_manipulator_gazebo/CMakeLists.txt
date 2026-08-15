cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Phase 4: worlds/ and launch/; config/ holds the ros_gz_bridge topic map
install(DIRECTORY worlds launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/home_hold.py
  scripts/capture_screenshot.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
