# Phase 4 — Gazebo Warehouse World

## Task Prompt for Agent
```text
In mobile_manipulator_gazebo, create warehouse.world: a Gazebo world with a
warehouse floor plan containing 2+ storage-rack aisles, wooden pallets, and barrier
obstacles, composed from existing Gazebo/Fuel models (do not hand-author
mesh geometry). Include a pick-up workbench with 2-3 spawned target objects
(cube, cylinder, box primitives with distinct colors for YOLO to
distinguish) and a separate drop-off table, plus a directional light and
ambient light tuned so the RGB camera feed is not blown out or too dark.
Add gazebo_warehouse.launch.py that starts Gazebo with this world, spawns
the mobile_manipulator robot (from Phase 2/3) at a named "home" pose using the
ros2_control Gazebo plugin (gazebo_ros2_control) instead of mock hardware.
Verify with `gz stats` showing real-time factor > 0.7 and confirm via a
screenshot that the robot is standing stably (not falling through the
floor or jittering) 30 seconds after spawn with zero commanded velocity.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Gazebo Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_gazebo
source install/setup.bash
```

### 2. Launch Gazebo Warehouse Simulation
```bash
ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py
```

### 3. Verify Gazebo Real-Time Factor (in new terminal tab)
```bash
gz stats
```

### 4. Verify Spawned Controllers
```bash
ros2 control list_controllers
```

**Pass Criteria**: Gazebo launches with real-time factor > 0.7; robot spawns upright at home pose without jittering or clipping through the floor; all controllers active in Gazebo.
