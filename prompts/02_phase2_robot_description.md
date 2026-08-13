# Phase 2 — Robot Description (Mobile Base + UR5 + Robotiq 2F-85 + D435i)

## Task Prompt for Agent
```text
Refer to PLAN.md and instructions.md — this task is Phase 2 only.
In ~/mobile_manipulator_ws/src/mobile_manipulator_description, build a
Xacro robot description that composes (not reimplements) two existing
packages already present in src/: vendor_base_description (a community
Humble mobile-base xacro port, already cloned — inspect its actual
xacro/macro filenames first) and ros-humble-ur-description (from apt): a
mobile base with a UR5 arm mounted on top via a mounting-plate xacro (base
top_plate_link -> ur5 base_link), a Robotiq 2F-85 gripper on the UR5 tool0,
and a RealSense D435i mounted at the wrist via a xacro macro, publishing
camera_link and camera_color_optical_frame per REP-103 conventions.
Reminder: no "husky" anywhere in filenames, macro names, or link/joint
names — use mobile_manipulator_* / generic base/arm naming throughout.
Provide a view_robot.launch.py that starts robot_state_publisher +
joint_state_publisher_gui + RViz2 (no Gazebo). Verify with
`xacro urdf/mobile_manipulator.urdf.xacro | check_urdf /dev/stdin` and
confirm zero errors, then take a screenshot artifact of the RViz view
showing the full assembled robot. Do not proceed to controllers or Gazebo
in this task.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_description husky_description
source install/setup.bash
```

### 2. Parse & Validate URDF
```bash
xacro src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro | check_urdf /dev/stdin
```

### 3. Launch Interactive RViz Viewer
```bash
ros2 launch mobile_manipulator_description view_robot.launch.py
```

### 4. Echo Frames & Check TF Tree (in another terminal tab)
```bash
ros2 run tf2_tools view_frames
```

**Pass Criteria**: `check_urdf` prints zero errors with 53 segments; RViz opens showing the mobile base, UR5 arm, Robotiq 2F-85 gripper, and D435i camera attached at the wrist.
