# Phase 10 — Automated Integration Testing

## Task Prompt for Agent
```text
Write pytest + launch_testing tests covering:
(1) test_tf_tree.py - launches description+controllers, waits for TF,
asserts via tf2_ros.Buffer that the expected frame set exists with no
missing parent frames; (2) test_controllers_active.py - asserts every
controller in mobile_manipulator_controllers.yaml reports 'active' via the
controller_manager list_controllers service within a timeout;
(3) test_perception_tf.py - launches Gazebo+perception node, asserts
object_target_frame becomes available within 15 seconds;
(4) test_full_cycle_launch.py - a launch_testing test that runs
warehouse_demo.launch.py and asserts, via a subscription to the
orchestrator's state topic/log, that it reaches RETURN_HOME within a
generous timeout (e.g. 5 minutes). Run `colcon test` and
`colcon test-result --verbose`, and report the full pass/fail summary.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Tests
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. Run Automated Test Suite
```bash
colcon test --packages-select mobile_manipulator_navigation mobile_manipulator_perception mobile_manipulator_orchestrator
```

### 3. View Test Summary Output
```bash
colcon test-result --verbose
```

**Pass Criteria**: All integration test suites pass with zero failures (`colcon test-result` green).
