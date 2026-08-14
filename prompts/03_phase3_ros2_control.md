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

## Known Gotchas (learned the hard way — do NOT skip)

1. **`ROS_LOCALHOST_ONLY=1` is mandatory.** The machine has WiFi + Proton VPN
   interfaces; FastDDS multicast discovery over them is intermittent, so CLI
   tools randomly fail to see the launch tree. Export it before launch AND
   before every CLI call:
   ```bash
   export ROS_LOCALHOST_ONLY=1
   ```

2. **Stale ros2 daemons break every `ros2` CLI command** with
   `xmlrpc.client.Fault: ... !rclpy.ok()` (their rclpy context dies after the
   CLI that spawned them exits). Fix before any CLI call:
   ```bash
   for p in $(ps aux | grep "[r]os2cli.daemon" | awk '{print $2}'); do kill -9 $p; done
   ```

3. **The launch must run detached** — background jobs get reaped between
   terminal sessions. Use:
   ```bash
   setsid nohup ros2 launch mobile_manipulator_description control_test.launch.py \
     > /tmp/control_test.log 2>&1 < /dev/null &
   ```

4. **diff_drive_controller requires BEST_EFFORT cmd_vel QoS.** Humble uses
   `SystemDefaultsQoS()` for its subscription (resolves to BEST_EFFORT), so the
   default RELIABLE publisher silently drops messages. Always publish with:
   ```bash
   ros2 topic pub --qos-reliability best_effort /diff_drive_controller/cmd_vel_unstamped ...
   ```
   (`use_stamped_vel: false` → topic is `cmd_vel_unstamped`, NOT `/cmd_vel`.)

5. **`position_feedback: false` is required in the diff_drive config.** The
   Humble default is `true`, which integrates odometry from wheel *position*
   states — mock hardware never integrates velocity into position, so odom
   stays 0 forever. With `false`, odom integrates from velocity states (which
   the mock reports = commanded velocity).

6. **Gripper action name is `gripper_cmd` in Humble**, not `gripper_command`:
   ```bash
   ros2 action send_goal /gripper_action_controller/gripper_cmd \
     control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 100}}"
   ```

7. **The launch file MUST wrap the Command-substituted URDF** in
   `ParameterValue(..., value_type=str)` or launch dies with
   "Unable to parse the value of parameter robot_description as yaml".

8. **pgrep/pkill footguns**: process names are truncated to 15 chars
   (`ros2_control_no`), and `pkill -f spawner`/`pgrep -f control_test` match
   your own shell's command line. Use `pgrep -f "[c]ontrol_test.launch"` or
   match by PID.

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 0. Preflight (once per terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/mobile_manipulator_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
for p in $(ps aux | grep "[r]os2cli.daemon" | awk '{print $2}'); do kill -9 $p; done
```

### 1. Build Package
```bash
cd ~/mobile_manipulator_ws
colcon build --symlink-install --packages-select mobile_manipulator_description
source install/setup.bash
```

### 2. Launch Mock Hardware Controllers (detached)
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_LOCALHOST_ONLY=1
setsid nohup ros2 launch mobile_manipulator_description control_test.launch.py \
  > /tmp/control_test.log 2>&1 < /dev/null &
```

### 3. Check Controllers (in new terminal tab)
```bash
source /opt/ros/humble/setup.bash && source ~/mobile_manipulator_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 control list_controllers
```

### 4. Publish Test Trajectory Point (arm)
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint'],
  points: [{positions: [0.2, -1.0, 1.0, -0.5, 0.0, 0.0], time_from_start: {sec: 2}}]
}"
```

### 5. Echo Joint States (verify arm + wheels + gripper)
```bash
ros2 topic echo /joint_states --once
```

### 6. Optional: Diff Drive + Gripper (full controller sweep)
```bash
# diff drive — MUST use best_effort QoS on cmd_vel_unstamped
ros2 topic pub --rate 10 --qos-reliability best_effort \
  /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
# in another tab, confirm wheel velocity ≈ 3.03 rad/s and odom x increases
ros2 topic echo /joint_states --once
ros2 topic echo /diff_drive_controller/odom --once

# gripper — action is named gripper_cmd
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 100}}"
```

**Pass Criteria**:
- `ros2 control list_controllers` shows all 4 controllers `active`.
- `/joint_states` reflects the commanded arm trajectory values (all 6 joints
  match the published point after ~2 s).
- Optional: `/diff_drive_controller/odom` pose.x increases while cmd_vel is
  published; gripper knuckle reaches the commanded position.
