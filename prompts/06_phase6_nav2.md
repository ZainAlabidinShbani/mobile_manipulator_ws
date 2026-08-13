# Phase 6 — Nav2 Navigation Stack

## Task Prompt for Agent
```text
In mobile_manipulator_navigation: (1) run slam_toolbox once against the warehouse.world
from Phase 4 to generate and save an occupancy map (warehouse.yaml +
warehouse.pgm). (2) Write nav2_params.yaml configured for AMCL localization
against that saved map, with costmaps (global + local) tuned with an
inflation radius appropriate for the mobile base footprint plus a safety
margin, and controller_server/planner_server using DWB/NavFn.
(3) Write nav2_bringup.launch.py starting AMCL + Nav2 stack against the saved map.
Verify by publishing a single hardcoded PoseStamped goal at the pick-table location
and confirming via `ros2 topic echo /plan` that a global plan is produced and the
robot drives there without colliding with any rack, pallet, or barrier model.
Report the final pose error.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Navigation Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_navigation
source install/setup.bash
```

### 2. Optional: Generate SLAM Map
```bash
ros2 launch slam_toolbox online_async_launch.py
# After driving around to map warehouse:
ros2 run nav2_map_server map_saver_cli -f ~/mobile_manipulator_ws/src/mobile_manipulator_navigation/maps/warehouse
```

### 3. Launch Nav2 Stack
```bash
ros2 launch mobile_manipulator_navigation navigation.launch.py
```

### 4. Send Navigation Goal (in new terminal tab)
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.5, y: 1.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

### 5. Echo Nav Goal Path
```bash
ros2 topic echo /plan --once
```

**Pass Criteria**: Nav2 computes a valid global and local path; robot navigates autonomously to the target pose within goal tolerance without collision.
