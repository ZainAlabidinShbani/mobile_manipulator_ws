# Phase 11 — System Hardening & Resilience

## Task Prompt for Agent
```text
The full warehouse_demo pipeline currently completes one HOME->pick->drop
->HOME cycle successfully. Harden warehouse_orchestrator.py to run the
cycle 5 times consecutively without manual intervention: add a
configurable retry count on GRASP failure (re-attempt PERCEIVE+APPROACH_ARM
up to 3 times before declaring failure), add a Nav2 recovery trigger if path
planning fails, and ensure all ROS node handlers and action clients clean up
and re-initialize cleanly between cycles. Run 5 consecutive cycles unattended,
reporting the per-cycle completion time and overall success rate.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Hardened Orchestrator
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_orchestrator
source install/setup.bash
```

### 2. Launch Multi-Cycle Hardening Test
```bash
ros2 launch mobile_manipulator_navigation warehouse_demo.launch.py target_cycles:=5
```

### 3. Monitor Cycle Logs & Status
```bash
ros2 topic echo /orchestrator/status
```

**Pass Criteria**: Orchestrator completes 5 consecutive pick-and-place cycles unattended without locks, freezes, or crashes.
