# Phase 3 — ros2_control & Hardware Interface

## Task Prompt for Agent
```text
In mobile_manipulator_description, add a <ros2_control> tag using
mock_components/GenericSystem hardware interface (not Gazebo yet) covering:
diff-drive wheels, UR5's 6 arm joints, and the gripper's 1 actuated
joint. Write mobile_manipulator_controllers.yaml configuring:
joint_state_broadcaster, diff_drive_controller, joint_trajectory_controller
(UR5 arm), and gripper_action_controller (Robotiq). Provide control_test.launch.py
that starts ros2_control_node + spawns all 4 controllers using mock hardware.
Verify with `ros2 control list_controllers` (all must show `active`) and by
publishing one test JointTrajectory point to the arm controller and
confirming /joint_states reflects it. Report the exact output of
`ros2 control list_controllers` as evidence.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_description
source install/setup.bash
```

### 2. Launch Mock Hardware Controllers
```bash
ros2 launch mobile_manipulator_description control_test.launch.py
```

### 3. Check Controllers (in new terminal tab)
```bash
ros2 control list_controllers
```

### 4. Publish Test Trajectory Point
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint'],
  points: [{positions: [0.2, -1.0, 1.0, -0.5, 0.0, 0.0], time_from_start: {sec: 2}}]
}"
```

### 5. Echo Joint States
```bash
ros2 topic echo /joint_states --once
```

**Pass Criteria**: `ros2 control list_controllers` shows all 4 controllers `active`, and `/joint_states` reflects updated trajectory values.
