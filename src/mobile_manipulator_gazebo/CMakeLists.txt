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

  # Phase 10.  These are launch_testing tests, not plain pytest: each one has
  # to stand a stack up before it can assert anything, so they are registered
  # with add_launch_test rather than ament_add_pytest_test.
  #
  # The timeouts are wall clock and the simulation is not.  The full stack runs
  # at a real-time factor near 0.5 on an 8-core box, so every budget here is
  # roughly double what the same work would take at real time.
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_tf_tree.py TIMEOUT 600)
  add_launch_test(test/test_controllers_active.py TIMEOUT 600)

  # The end-to-end cycle is opt-in: it needs a machine that can carry gz-sim,
  # Nav2, move_group and YOLOv8 at once, and a measured passing run is ~9
  # minutes wall clock.  Enable with -DMM_FULL_CYCLE_TEST=ON.
  if(MM_FULL_CYCLE_TEST)
    add_launch_test(test/test_full_cycle_launch.py TIMEOUT 1200)
  endif()
endif()

ament_package()
