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

The mobile base is **not** available from apt on Humble — clone the plain-xacro community
port into `src/` (it keeps its upstream name because it is referenced, never redefined):

```bash
cd ~/mobile_manipulator_ws/src
git clone https://github.com/akrbot/husky_description_ros2.git husky_description
cd ~/mobile_manipulator_ws && rosdep install --from-paths src --ignore-src -r -y
```

**Per-terminal preamble for every phase below** (Phases 3+ break without it):
```bash
source /opt/ros/humble/setup.bash && source ~/mobile_manipulator_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
for p in $(ps aux | grep "[r]os2cli.daemon" | awk '{print $2}'); do kill -9 $p; done
```

---

## Phase 1: Workspace Bootstrap  ✅ complete
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
source install/setup.bash && ros2 pkg list | grep -E "mobile_manipulator|husky_description"
```

---

## Phase 2: Robot Description (URDF/Xacro)  ✅ complete
**Task Prompt**:
```text
In ~/mobile_manipulator_ws/src/mobile_manipulator_description, build a
Xacro robot description that composes the vendor packages husky_description
(cloned in src/), ur_description, robotiq_description and
realsense2_description: a mobile base with a UR5 arm mounted on top via a
mounting-plate xacro (top_plate_link -> arm mounting plate -> UR5 base_link),
a Robotiq 2F-85 gripper on tool0 via the ur_to_robotiq adapter, and a
RealSense D435i mounted at the wrist via a xacro macro, publishing
camera_link and camera_color_optical_frame per REP-103 conventions.
arm_prefix MUST be non-empty (arm_) or the UR5 base_link collides with the
mobile base's. Suppress the vendor ros2_control tags (Phase 3 adds ours).
No "husky" in anything we author. Provide a view_robot.launch.py.
Verify with `xacro urdf/mobile_manipulator.urdf.xacro | check_urdf /dev/stdin`.
```
**Terminal Verification**:
```bash
cd ~/mobile_manipulator_ws && source install/setup.bash
xacro src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro sensor_arch:=0 > /tmp/mm.urdf
check_urdf /tmp/mm.urdf          # 55 links / 54 joints, rooted at base_footprint
ros2 launch mobile_manipulator_description view_robot.launch.py
```

---

## Phase 3: ros2_control  ✅ complete
**Task Prompt**:
```text
In mobile_manipulator_description, add a <ros2_control> tag using
mock_components/GenericSystem hardware interface covering the 4 wheels, the 6
UR5 joints, and the gripper's actuated + 5 mimic joints. Write
mobile_manipulator_controllers.yaml configuring: joint_state_broadcaster,
diff_drive_controller, joint_trajectory_controller (arm_controller),
and gripper_action_controller. Provide control_test.launch.py.
Verify with `ros2 control list_controllers` (all must show `active`).
```
**Terminal Verification**:
```bash
setsid nohup ros2 launch mobile_manipulator_description control_test.launch.py \
  > /tmp/control_test.log 2>&1 < /dev/null &
ros2 control list_controllers
# cmd_vel needs BEST_EFFORT QoS and the _unstamped topic:
ros2 topic pub --rate 10 --qos-reliability best_effort \
  /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
# gripper action is gripper_cmd on Humble:
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 100}}"
```
See `03_phase3_ros2_control.md` for the full gotcha list (`position_feedback: false`,
`ParameterValue(value_type=str)`, pgrep truncation, …).

---

## Phase 4: Gazebo Warehouse World  ✅ complete
**Task Prompt**:
```text
In mobile_manipulator_gazebo, create warehouse.world: a Gazebo Classic world with a
warehouse floor plan (2+ storage-rack aisles, pallets, barrier obstacles, pick table,
drop table, colored target primitives, tuned lighting), composed from existing
model:// database models. Add gazebo_warehouse.launch.py that starts Gazebo and spawns
the robot at a named home pose using gazebo_ros2_control (use_gazebo:=true), with the
controller manager running inside gzserver. Verify real-time factor > 0.7 and stable spawn.
```
**Terminal Verification**:
```bash
setsid nohup ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py \
  > /tmp/gazebo_warehouse.log 2>&1 < /dev/null &
gz stats                       # RTF > 0.7
ros2 control list_controllers  # all 4 active (manager lives inside gzserver)
ros2 run mobile_manipulator_gazebo capture_screenshot.py \
  --topic /phase4_camera/image_raw --out /tmp/phase4_home_pose.png --settle 2
```
See `04_phase4_gazebo_world.md` for the full gotcha list (`GAZEBO_MODEL_PATH`,
`HUSKY_GAZEBO_PLUGINS=0`, URDF comment stripping, no depth-camera plugin, …).

---

## Phase 5: MoveIt 2 Configuration
**Task Prompt**:
```text
Generate mobile_manipulator_moveit_config with planning groups "ur5_arm" (the 6 UR5
joints) and "gripper" (Robotiq 2F-85), a self-collision matrix from the Setup
Assistant's default sampling, and moveit_controllers.yaml wired to arm_controller
and gripper_action_controller from Phase 3. Verify by planning to a reachable pose
above the pick table and executing it against the live Gazebo simulation.
```
**Terminal Verification**:
```bash
# self-contained bench (mock hardware)
ros2 launch mobile_manipulator_moveit_config demo.launch.py

# against Phase 4 Gazebo
ros2 launch mobile_manipulator_moveit_config move_group.launch.py use_sim_time:=true
ros2 launch mobile_manipulator_moveit_config moveit_rviz.launch.py use_sim_time:=true
ros2 run mobile_manipulator_moveit_config phase5_plan_execute.py \
  --frame base_footprint --use-sim-time \
  --goal-config 0.0 -1.1175 0.1054 -1.2083 -1.5708 0.0 \
  --workbench 1.307 0.0 1.000 1.50 0.80 0.03 --workbench-frame odom
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
