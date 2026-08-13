cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Phase 6 will install: config/ launch/
# Phase 9 will add:    launch/warehouse_demo.launch.py
# Phase 10 will add:   test/
install(DIRECTORY DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
