# All Mobile Manipulator Task Prompts & Commands (Master Reference)

---

## Preflight Environment Setup
```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-ur-description ros-humble-ur-msgs \
  ros-humble-robotiq-description \
  ros-humble-realsense2-description ros-humble-realsense2-camera \
  ros-humble-moveit ros-humble-moveit-setup-assistant \
  ros-humble-nav2-bringup ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools ros-humble-tf-transformations \
  python3-colcon-common-extensions python3-rosdep

pip install ultralytics opencv-python transforms3d mss
```

---

## Phase 1: Workspace Bootstrap
**Task Prompt**:
```text
In ~/mobile_manipulator_ws, create 6 valid ROS 2 Humble packages:
- mobile_manipulator_description (ament_cmake)
- mobile_manipulator_gazebo (ament_cmake)
- mobile_manipulator_navigation (ament_cmake)
- mobile_manipulator_moveit_config (ament_cmake, skeleton)
- mobile_manipulator_perception (ament_python)
- mobile_manipulator_orchestrator (ament_python)
Each package needs correct package.xml and CMakeLists.txt/setup.py.
Run `colcon build --symlink-install` and confirm all build with zero errors.
```
**Terminal Verification**:
```bash
cd ~/mobile_manipulator_ws && colcon build --symlink-install
source install/setup.bash && ros2 pkg list | grep mobile_manipulator
```

---

## Phase 2: Robot Description (URDF/Xacro)
**Task Prompt**:
```text
In ~/mobile_manipulator_ws/src/mobile_manipulator_description, build a
Xacro robot description that composes vendor_base_description (husky_description)
and ros-humble-ur-description: a mobile base with a UR5 arm mounted on top
via a mounting-plate xacro (top_plate_link -> ur5 base_link), a Robotiq 2F-85
gripper on tool0, and a RealSense D435i mounted at the wrist via a xacro macro,
publishing camera_link and camera_color_optical_frame per REP-103 conventions.
No "husky" anywhere in custom filenames or macro names. Provide a view_robot.launch.py.
Verify with `xacro urdf/mobile_manipulator.urdf.xacro | check_urdf /dev/stdin`.
```
**Terminal Verification**:
```bash
cd ~/mobile_manipulator_ws && source install/setup.bash
xacro src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro | check_urdf /dev/stdin
ros2 launch mobile_manipulator_description view_robot.launch.py
```

---

## Phase 3: ros2_control
**Task Prompt**:
```text
In mobile_manipulator_description, add a <ros2_control> tag using
mock_components/GenericSystem hardware interface covering wheels, UR5 joints,
and gripper. Write mobile_manipulator_controllers.yaml configuring:
joint_state_broadcaster, diff_drive_controller, joint_trajectory_controller,
and gripper_action_controller. Provide control_test.launch.py.
Verify with `ros2 control list_controllers` (all must show `active`).
```
**Terminal Verification**:
```bash
ros2 launch mobile_manipulator_description control_test.launch.py
ros2 control list_controllers
```

---

## Phase 4: Gazebo Warehouse World
**Task Prompt**:
```text
In mobile_manipulator_gazebo, create warehouse.world: a Gazebo world with a
warehouse floor plan (2+ storage-rack aisles, pallets, barrier obstacles, pick table,
drop table). Add gazebo_warehouse.launch.py that starts Gazebo and spawns the robot
using gazebo_ros2_control. Verify real-time factor > 0.7 and stable spawn.
```
**Terminal Verification**:
```bash
ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py
gz stats
```

---

## Phase 5: MoveIt 2 Configuration
**Task Prompt**:
```text
Generate mobile_manipulator_moveit_config with planning groups "arm" and "gripper",
self-collision matrix, and moveit_controllers.yaml. Verify with demo.launch.py
and execute trajectory against live Gazebo simulation.
```
**Terminal Verification**:
```bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py
```

---

## Phase 6: Nav2 Navigation
**Task Prompt**:
```text
In mobile_manipulator_navigation: configure AMCL + Nav2 stack against saved map.
Verify by publishing a PoseStamped goal at the pick-table location and confirming
robot navigates autonomously without collision.
```
**Terminal Verification**:
```bash
ros2 launch mobile_manipulator_navigation navigation.launch.py
```

---

## Phase 7: YOLOv8 Perception Node
**Task Prompt**:
```text
In mobile_manipulator_perception, write yolo_perception_node.py: subscribes to
color/depth camera feeds, runs YOLOv8n, displays cv2.imshow, and broadcasts
camera_color_optical_frame -> object_target_frame TF transform at 10 Hz.
```
**Terminal Verification**:
```bash
ros2 run mobile_manipulator_perception yolo_perception_node
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_target_frame
```

---

## Phase 8: Orchestrator State Machine
**Task Prompt**:
```text
In mobile_manipulator_orchestrator, write warehouse_orchestrator.py: state machine
(HOME -> NAV_TO_PICK -> PERCEIVE -> APPROACH_ARM -> GRASP -> NAV_TO_DROP -> PLACE_ARM -> RELEASE -> RETURN_HOME).
Include dry_run mode for logic verification.
```
**Terminal Verification**:
```bash
ros2 run mobile_manipulator_orchestrator warehouse_orchestrator --ros-args -p dry_run:=true
```

---

## Phase 9: Master Launch Integration
**Task Prompt**:
```text
Write warehouse_demo.launch.py bringing up Gazebo, controllers, Nav2, MoveIt2,
perception node, and orchestrator in event-driven dependency order. Verify full loop.
```
**Terminal Verification**:
```bash
ros2 launch mobile_manipulator_navigation warehouse_demo.launch.py
```

---

## Phase 10: Automated Tests
**Task Prompt**:
```text
Write pytest + launch_testing tests for TF tree, controller status, perception TF, and
end-to-end launch verification. Verify with `colcon test`.
```
**Terminal Verification**:
```bash
colcon test --packages-select mobile_manipulator_navigation mobile_manipulator_perception mobile_manipulator_orchestrator
colcon test-result --verbose
```

---

## Phase 11: System Hardening
**Task Prompt**:
```text
Harden warehouse_orchestrator.py to run 5 consecutive cycles unattended with retries
and error recovery. Verify 5 consecutive pick-and-place cycles.
```
**Terminal Verification**:
```bash
ros2 launch mobile_manipulator_navigation warehouse_demo.launch.py target_cycles:=5
```
