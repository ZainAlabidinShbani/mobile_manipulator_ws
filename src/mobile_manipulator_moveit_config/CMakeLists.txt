cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# MoveIt Setup Assistant output: config/, launch/ and the .setup_assistant
# manifest that MoveItConfigsBuilder reads to locate the URDF and SRDF.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/regenerate_collision_matrix.sh
  scripts/phase5_plan_execute.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
