# Phase 9 — Master Launch & Full System Integration

## Task Prompt for Agent
```text
In mobile_manipulator_navigation (or bringup package), write warehouse_demo.launch.py
that brings up, in strict dependency order using launch event handlers (not naive
concurrent launch): Gazebo+warehouse world+robot spawn+ros2_control, then (after
controllers report active) Nav2, then move_group, then RViz2 with a
pre-saved config showing RobotModel/TF/MotionPlanning/camera image/Nav2
panels, then the yolo_perception_node, then finally the orchestrator with
dry_run:=false. Verify by running the full launch file once and confirming
the orchestrator completes one full HOME->pick->drop->HOME cycle with the
target object visibly transported between tables in Gazebo. If it fails,
report exactly which state the orchestrator was in when it failed and the
relevant node's log output — do not attempt to silently patch multiple
subsystems at once; identify the single root cause first.
```

---

## Terminal Commands

The launch file lives in **`mobile_manipulator_gazebo`**, not
`mobile_manipulator_navigation`: it is the package that already owns the
simulation bringup it has to sequence, and Phase 4's launch is included rather
than re-implemented.

### 0. Kill stale processes FIRST — this is not optional

A controller manager left over from an earlier session satisfies the launch's
readiness gates in about two seconds, and the whole stack then comes up against
a simulation whose robot has not spawned.  The gates are now written as
*sequences* (see `gate()` in the launch file) so this specific leftover cannot
fool them, but check anyway:

```bash
ps aux | grep -E "[i]gn gazebo|[m]ove_group|[r]os2_control_node|[n]av2"
```

Put any `pkill -f` patterns in a **script file**, never on the command line —
`pkill -f` matches the whole command line, so `pkill -f move_group` typed
directly kills the shell that typed it (exit code 144) before it kills anything
else, and the rest of the line never runs.

### 1. Build All Workspace Packages
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

### 2. Launch Full Warehouse Demo Integration
```bash
ros2 launch mobile_manipulator_gazebo warehouse_demo.launch.py
```

Useful arguments:

| Argument | Default | Why you would change it |
|---|---|---|
| `rviz` | `true` | `false` while debugging — RViz plus Gazebo plus torch is tight on 7 GB |
| `gui` | `false` | `true` to watch the sim, but it starves the wrist cameras (~6 Hz → <0.3 Hz) |
| `gui_render_engine` | `ogre2` | `ogre` if the GUI viewport renders sheared — see CLAUDE.md |
| `dry_run` | `false` | `true` re-runs the Phase 8 stub logic inside the full stack |
| `cycles` | `1` | Phase 11 wants 5 |

### 3. Watch the state machine (in a new terminal tab)
```bash
ros2 node list
ros2 topic list
ros2 topic echo /rosout | grep -A2 "\[STATE\]"
```

### 4. Confirm the object actually moved (the real gate)

Wheel odometry and the orchestrator's own logs are not evidence that the object
was transported — gz ground truth is:

```bash
ign topic -e -t /world/warehouse/pose/info -n 1 | grep -A6 target_ball
```

Pick bench targets start at `x = 3.92, y = {-0.26, 0.00, +0.26}, z = 0.633`.
A successful cycle leaves the carried one on the drop bench near
`x = 3.92, y = 3.2`, still at `z ≈ 0.633`, while the other two have not moved.

**Pass Criteria**: Full autonomous loop executes: robot navigates to pick table,
detects object with YOLOv8, picks object with MoveIt2 + Robotiq gripper,
navigates to drop table, places object, and returns home.
