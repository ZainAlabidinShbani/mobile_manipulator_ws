# Phase 5 — MoveIt 2 Configuration

## Task Prompt for Agent
```text
Using the MoveIt Setup Assistant against the URDF/Xacro in
mobile_manipulator_description (from the completed, verified Phase 2), generate
mobile_manipulator_moveit_config with: a planning group "arm" covering the 6
UR5 joints, a planning group "gripper" for the Robotiq 2F-85, self-collision
matrix generated from default sampling, and moveit_controllers.yaml wired to the
joint_trajectory_controller and gripper_action_controller from Phase 3/4.
Verify by launching move_group.launch.py + demo.launch.py, planning a
motion in RViz's MotionPlanning panel to a reachable pose above the pick
table, confirming a collision-free green preview, and executing it against
the running Gazebo simulation from Phase 4 — report whether the physical
Gazebo arm moved to match the RViz plan.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Optional: Run MoveIt Setup Assistant GUI
```bash
source /opt/ros/humble/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### 2. Build MoveIt Config Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_moveit_config
source install/setup.bash
```

### 3. Launch MoveGroup with RViz MotionPlanning
```bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py
```

### 4. Execute MoveIt Execution with Gazebo (with Phase 4 running)
```bash
ros2 launch mobile_manipulator_moveit_config move_group.launch.py use_sim_time:=true
```

**Pass Criteria**: Motion planning succeeds in RViz with zero collision errors; executing plan moves arm in Gazebo simulation.
