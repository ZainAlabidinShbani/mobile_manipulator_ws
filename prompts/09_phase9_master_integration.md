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

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build All Workspace Packages
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Full Warehouse Demo Integration
```bash
ros2 launch mobile_manipulator_navigation warehouse_demo.launch.py
```

### 3. Monitor Active ROS Nodes & Topics (in new terminal tab)
```bash
ros2 node list
ros2 topic list
```

**Pass Criteria**: Full autonomous loop executes: robot navigates to pick table, detects object with YOLOv8, picks object with MoveIt2 + Robotiq gripper, navigates to drop table, places object, and returns home.
