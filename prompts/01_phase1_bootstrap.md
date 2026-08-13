# Phase 1 — Workspace Bootstrap

## Task Prompt for Agent
```text
In ~/mobile_manipulator_ws, create 6 valid ROS 2 Humble packages:
- mobile_manipulator_description (ament_cmake)
- mobile_manipulator_gazebo (ament_cmake)
- mobile_manipulator_navigation (ament_cmake)
- mobile_manipulator_moveit_config (ament_cmake, skeleton)
- mobile_manipulator_perception (ament_python)
- mobile_manipulator_orchestrator (ament_python)

Each package needs correct package.xml, CMakeLists.txt / setup.py, and license/README.
Run `colcon build --symlink-install` and confirm all build with zero errors.
Do not write any robot-specific code yet — this phase is scaffolding only.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### Build Command
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Verification Command
```bash
source ~/mobile_manipulator_ws/install/setup.bash
ros2 pkg list | grep mobile_manipulator
```

**Pass Criteria**: All `mobile_manipulator_*` packages listed with zero build errors.
