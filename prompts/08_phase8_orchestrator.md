# Phase 8 — Orchestrator State Machine

## Task Prompt for Agent
```text
In mobile_manipulator_orchestrator, write warehouse_orchestrator.py: an rclpy node
implementing an explicit state machine (Python enum) with states HOME,
NAV_TO_PICK, PERCEIVE, APPROACH_ARM, GRASP, NAV_TO_DROP, PLACE_ARM,
RELEASE, RETURN_HOME, RECOVERY/ABORT. NAV_TO_PICK/NAV_TO_DROP call Nav2's
NavigateToPose action client; PERCEIVE waits for a fresh
object_target_frame TF (with a timeout) from Phase 7's node;
APPROACH_ARM/PLACE_ARM call MoveIt2's MoveGroup action interface targeting
object_target_frame; GRASP/RELEASE call the gripper_action_controller.
Every state must have a bounded timeout and transition to RECOVERY/ABORT
on failure or timeout rather than blocking indefinitely. Add a `dry_run` ROS
param that stubs all four external calls (Nav2, MoveIt2, TF wait, gripper)
to return success after a short simulated delay, for logic testing without
a running simulation. Verify by running with dry_run:=true twice: once letting
all stubs succeed (confirm the full state sequence logs HOME through RETURN_HOME
back to HOME), and once forcing one stub to return failure (confirm it
transitions to RECOVERY/ABORT rather than hanging or crashing).
Report both log traces.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Orchestrator Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_orchestrator
source install/setup.bash
```

### 2. Test Dry-Run Success Sequence
```bash
ros2 run mobile_manipulator_orchestrator warehouse_orchestrator --ros-args -p dry_run:=true
```

### 3. Test Dry-Run Failure Recovery Sequence
```bash
ros2 run mobile_manipulator_orchestrator warehouse_orchestrator --ros-args -p dry_run:=true -p force_failure_state:=GRASP
```

**Pass Criteria**: Complete state sequence logs cleanly from HOME → NAV_TO_PICK → ... → RETURN_HOME in success test; failure test transitions to RECOVERY/ABORT without locking up.
