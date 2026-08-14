# Husky + UR5 Warehouse Mobile Manipulation — Phased Build Plan
### Optimized for Google Antigravity (Planning Mode, Implementation Plan artifacts, incremental review)

---

## 0. Why the original prompt needs to be split

Your original prompt asks for one agent run to produce: URDF/Xacro + ros2_control + MoveIt2 config, a hand-built Gazebo `.world` file, a YOLOv8 perception node, Nav2 config, a state-machine orchestrator, a master launch file, pytest/launch tests, *and* "automatically resolve all TF tree mismatches... until it runs seamlessly" — as a single task.

That fails in Antigravity for a structural reason, not a difficulty reason:

- **Antigravity's trust model is per-task Artifacts.** In Planning Mode it produces one Implementation Plan, you review it, then it executes. A task this size either skips depth in the plan (because the plan itself would be enormous) or the agent runs out of useful context tracking 7 interacting subsystems in one diff.
- **There is no independent verification gate between subsystems.** If the URDF has a bad joint origin, that error won't surface until step 4 of 6 in the orchestrator, at which point the agent is debugging TF + MoveIt2 + Nav2 + your state machine simultaneously — and "keep trying until it works" burns budget with no way to bisect the failure.
- **Simulation projects fail loudly and specifically** (missing frame, controller not spawned, planning group not found, `colcon build` error). Each of those has a *distinct, checkable* signal. The plan below turns each into its own phase with its own pass/fail gate, so when something breaks you know which of ~12 phases owns the bug.

**How to use this document:** Run each phase as its own Antigravity task, in **Planning Mode** (not Fast Mode — this project is not a typo fix). Do not start Phase *N+1* until Phase *N*'s verification command passes. Each phase below ends with a ready-to-paste "Antigravity task prompt" — copy that block in as the task description.

---

## 1. Phase Map

| # | Phase | Owns | Verifies |
|---|-------|------|----------|
| 1 | Workspace bootstrap | `~/mobile_manipulator_ws`, deps, empty package skeletons | `colcon build` succeeds on empty pkgs |
| 2 | Robot description | Husky+UR5+gripper+D435i Xacro/URDF | URDF parses, RViz shows correct model |
| 3 | ros2_control | Controller yaml, hardware interfaces | Controllers spawn, joints move via `ros2 control` |
| 4 | Gazebo world | Warehouse `.world`, spawn robot | Gazebo loads at real-time factor, robot spawns upright |
| 5 | MoveIt 2 config | `moveit_config` package (MoveIt Setup Assistant output) | Planning group plans a trajectory in RViz |
| 6 | Nav2 | Costmaps, AMCL/SLAM, params | Robot navigates a hardcoded goal pose in warehouse |
| 7 | YOLOv8 perception node | `yolo_perception_node.py`, TF broadcast | Bounding boxes render, `object_target_frame` TF published |
| 8 | Orchestrator state machine | `warehouse_orchestrator.py` | Each state transition logged, dry-run with mocked perception |
| 9 | Master launch + integration | `warehouse_demo.launch.py` | Full loop runs once, pick-and-place succeeds |
| 10 | Automated tests | `pytest` + `launch_testing` | `colcon test` green |
| 11 | Hardening pass | Retry/failure states, timeouts | Loop survives 5 consecutive cycles |

---

## 2. Environment Preflight (do this once, before Phase 1)

Run this yourself in the Zsh terminal — don't delegate it, it's just dependency installation and takes 2 minutes to eyeball:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-ur-description ros-humble-ur-msgs \
  ros-humble-robotiq-description \
  ros-humble-realsense2-description ros-humble-realsense2-camera \
  ros-humble-moveit ros-humble-moveit-setup-assistant \
  ros-humble-nav2-bringup ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools ros-humble-tf-transformations \
  python3-colcon-common-extensions python3-rosdep

pip install ultralytics opencv-python transforms3d

mkdir -p ~/mobile_manipulator_ws/src
cd ~/mobile_manipulator_ws && rosdep install --from-paths src --ignore-src -r -y
```

**Important — Husky is NOT available via `apt` on Humble.** Clearpath deprecated the standalone `husky_description`/`husky_control`/`husky_gazebo` debs for Humble+ in favor of a YAML-config-driven `clearpath_common` ecosystem, which is built around Clearpath's own auto-generated `ros2_control` config and standard sensor "attachments" — it does not cleanly support bolting on a full UR5 arm, and it would conflict with the hand-written controller setup the rest of this plan depends on (Phases 3, 5, 6).

Instead, clone a maintained plain-xacro Humble Husky package straight into `src/`, so it behaves like any normal ROS 2 package and composes cleanly with the UR5 xacro in Phase 2:

```bash
cd ~/mobile_manipulator_ws/src
git clone https://github.com/akrbot/husky_description_ros2.git husky_description
cd ~/mobile_manipulator_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select husky_description
```

**Check before proceeding:**
```bash
source install/setup.bash
ros2 pkg list | grep -E "husky|ur_description|moveit|nav2"
```
Should list `husky_description`, `ur_description`, `moveit`-related packages, and `nav2`-related packages. If `ur_description` also isn't available as a binary for Humble (some minimal installs omit it), tell me and Phase 2 gets the same source-clone treatment for the UR5 side (`ros-industrial/universal_robot` or `UniversalRobots/Universal_Robots_ROS2_Description`).

---

## 3. Phase 1 — Workspace Bootstrap

**Goal:** empty but buildable package skeletons, so every later phase has a build target from day one.

**Deliverables:**
```
~/mobile_manipulator_ws/src/
├── mobile_manipulator_description/   (ament_cmake — URDF/Xacro, Phase 2)
├── mobile_manipulator_gazebo/         (ament_cmake — Gazebo world + spawn, Phase 4)
├── mobile_manipulator_navigation/     (ament_cmake — Nav2 params + master launch, Phases 6/9/10)
├── mobile_manipulator_moveit_config/  (ament_cmake — MoveIt Setup Assistant output, Phase 5)
├── mobile_manipulator_perception/     (ament_python — perception_node.py, Phase 7)
└── mobile_manipulator_orchestrator/   (ament_python — orchestrator_node.py, Phase 8)
```

**Test / gate:**
```bash
cd ~/mobile_manipulator_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep husky_ur5
```
Pass = all 5 packages listed, zero build errors.

**Antigravity task prompt (Phase 1):**
```
In ~/mobile_manipulator_ws, create 5 empty-but-valid ROS 2 Humble packages: husky_ur5_description
(ament_cmake), husky_ur5_moveit_config (ament_cmake, leave empty — Setup Assistant
will populate it later), husky_ur5_bringup (ament_cmake), husky_ur5_perception
(ament_python), husky_ur5_orchestrator (ament_python). Each needs a correct
package.xml and CMakeLists.txt/setup.py with placeholder README. Run
`colcon build --symlink-install` and confirm all 5 build with zero errors.
Do not write any robot-specific code yet — this phase is scaffolding only.
```

---

## 4. Phase 2 — Robot Description (Husky + UR5 + Gripper + D435i)

**Goal:** one Xacro that composes existing vendor descriptions rather than hand-authoring a UR5 from scratch — this is the single biggest scope-reduction vs. the original prompt, and the biggest source of subtle bugs if skipped.

**Specific instruction to give the agent:** use `xacro:include` to pull in the `husky_description` package cloned into `src/` in the preflight step (community Humble port, plain xacro — NOT the official `clearpath_common` yaml-generator system) and the `ur_description` xacros, and compose them, rather than writing link/joint geometry by hand. Hand-authoring inertials/collision geometry for a UR5 is exactly the kind of task that produces a URDF that *parses* but has garbage inertial values, which then makes Gazebo physics unstable in Phase 4 in ways that are hard to trace back to "the mass matrix is wrong." Tell the agent explicitly: inspect `src/husky_description`'s actual xacro filenames/macro names first (community ports vary), don't assume they match the official Clearpath package's file layout.

**Deliverables:**
```
husky_ur5_description/
├── urdf/
│   ├── husky_ur5.urdf.xacro          # top-level: includes husky + ur5 + gripper + camera
│   ├── ur5_mount.xacro                 # mounting plate + transform from husky top_plate_link to ur5 base
│   └── d435i_wrist_mount.xacro          # camera mount xacro macro, attached to UR5 tool0 or a wrist link
├── config/
│   └── initial_positions.yaml
└── launch/
    └── view_robot.launch.py             # RViz-only sanity check, no Gazebo
```

**Test / gate (three separate checks — run all three):**
```bash
# 1. URDF parses
xacro urdf/husky_ur5.urdf.xacro > /tmp/husky_ur5.urdf && check_urdf /tmp/husky_ur5.urdf

# 2. No duplicate/dangling links
ros2 run tf2_tools view_frames   # (after step 3 is running)

# 3. Visual sanity check in RViz
ros2 launch husky_ur5_description view_robot.launch.py
```
Pass = `check_urdf` reports the correct link/joint count with no errors, RViz shows Husky base + UR5 arm mounted correctly + gripper + camera at the wrist, with no links floating in the wrong place.

**Antigravity task prompt (Phase 2):**
```
In ~/mobile_manipulator_ws/src/husky_ur5_description, build a Xacro robot description
that composes (not reimplements) two existing packages already present in
src/: the husky_description package (a community Humble xacro port, already
cloned — inspect its actual xacro/macro filenames first, don't assume they
match the official Clearpath layout) and the ros-humble-ur-description
package (from apt): a Husky base with a UR5 arm mounted on top via a
mounting-plate xacro (husky top_plate_link -> ur5 base_link), a Robotiq
2F-85 gripper on the UR5 tool0, and a RealSense D435i mounted at the wrist
via a xacro macro, publishing camera_link and camera_color_optical_frame
per REP-103 conventions.
Provide a view_robot.launch.py that starts robot_state_publisher +
joint_state_publisher_gui + RViz2 (no Gazebo). Verify with
`xacro urdf/husky_ur5.urdf.xacro | check_urdf /dev/stdin` and confirm zero
errors, then take a screenshot artifact of the RViz view showing the full
assembled robot. Do not proceed to controllers or Gazebo in this task.
```

---

## 5. Phase 3 — ros2_control

**Goal:** controllers that actually spawn and accept commands, tested *before* Gazebo enters the picture (use `fake` / `mock` hardware components first — this isolates "is my controller config right" from "is Gazebo plugin config right").

**Deliverables:**
```
husky_ur5_description/config/
├── husky_ur5_controllers.yaml     # joint_state_broadcaster, diff_drive/husky controller, joint_trajectory_controller for UR5, gripper_action_controller
└── ros2_control_tags.xacro          # <ros2_control> block, using mock_components/GenericSystem for this phase
```

**Test / gate:**
```bash
ros2 launch husky_ur5_description control_test.launch.py   # spawns controller_manager with mock hardware
ros2 control list_controllers      # all controllers should be "active"
ros2 topic pub /ur5_arm_controller/joint_trajectory ... # send one test point, confirm /joint_states updates
```
Pass = `ros2 control list_controllers` shows every controller `active`, and a test trajectory command visibly updates `/joint_states`.

**Antigravity task prompt (Phase 3):**
```
In husky_ur5_description, add a <ros2_control> tag using
mock_components/GenericSystem hardware interface (not Gazebo yet) covering:
Husky diff-drive wheels, UR5's 6 arm joints, and the gripper's 1 actuated
joint. Write husky_ur5_controllers.yaml configuring: joint_state_broadcaster,
diff_drive_controller (Husky), joint_trajectory_controller (UR5 arm),
gripper_action_controller (Robotiq). Provide control_test.launch.py that
starts ros2_control_node + spawns all 4 controllers using mock hardware.
Verify with `ros2 control list_controllers` (all must show `active`) and by
publishing one test JointTrajectory point to the arm controller and
confirming /joint_states reflects it. Report the exact output of
`ros2 control list_controllers` as evidence.
```

---

## 6. Phase 4 — Gazebo Warehouse World

**Goal:** de-scope "complex industrial warehouse" into something buildable and testable. A hand-authored `.world` with racks, pallets, barriers, a pick table, and a drop table is a lot of manual SDF; the pragmatic path is to compose from Gazebo's model database / `gazebo_models` fuel models rather than authoring geometry by hand.

**Deliverables:**
```
husky_ur5_bringup/worlds/
└── warehouse.world
husky_ur5_bringup/launch/
└── gazebo_warehouse.launch.py   # spawns world + spawns robot at a named "home" pose
```

**Specific instruction to give the agent:** use Fuel/Gazebo model:// includes for pallet, shelving, and barrier models rather than hand-writing box primitives with textures — hand-authored geometry is where most "looks empty/wrong" complaints come from, and it's unnecessary scope for a robotics-logic project.

**Test / gate:**
```bash
ros2 launch husky_ur5_bringup gazebo_warehouse.launch.py
# check real-time factor
gz stats
# confirm robot spawned upright, not falling through floor or exploding
```
Pass = Gazebo real-time factor stays above ~0.7, robot sits stably at the home pose for 30s with zero commanded velocity (this confirms the Phase 2 inertials were sane).

**Antigravity task prompt (Phase 4):**
```
In husky_ur5_bringup, create warehouse.world: a Gazebo Classic (or Ignition,
match whatever gazebo_ros_pkgs version is installed) world with a warehouse
floor plan containing 2+ storage-rack aisles, wooden pallets, and barrier
obstacles, composed from existing Gazebo/Fuel models (do not hand-author
mesh geometry). Include a pick-up workbench with 2-3 spawned target objects
(cube, cylinder, box primitives with distinct colors for YOLO to
distinguish) and a separate drop-off table, plus a directional light and
ambient light tuned so the RGB camera feed is not blown out or too dark.
Add gazebo_warehouse.launch.py that starts Gazebo with this world, spawns
the husky_ur5 robot (from Phase 2/3) at a named "home" pose using the
ros2_control Gazebo plugin (gazebo_ros2_control) instead of mock hardware.
Verify with `gz stats` showing real-time factor > 0.7 and confirm via a
screenshot that the robot is standing stably (not falling through the
floor or jittering) 30 seconds after spawn with zero commanded velocity.
```

---

## 7. Phase 5 — MoveIt 2 Configuration

**Goal:** don't hand-write MoveIt2 yaml — generate it with MoveIt Setup Assistant against the *already-verified* URDF from Phase 2, then just review the output. This avoids the single most common MoveIt integration bug: SRDF planning groups that don't match the actual joint names in your URDF.

**Deliverables:**
```
husky_ur5_moveit_config/   # MoveIt Setup Assistant output
├── config/
│   ├── husky_ur5.srdf           # planning group "ur5_arm", end-effector "gripper"
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   └── moveit_controllers.yaml  # wired to the joint_trajectory_controller from Phase 3
└── launch/
    └── move_group.launch.py
```

**Test / gate:**
```bash
ros2 launch husky_ur5_moveit_config move_group.launch.py
ros2 launch husky_ur5_moveit_config demo.launch.py   # RViz MotionPlanning panel
```
In RViz: drag the interactive marker to a reachable pose, click Plan, confirm a green trajectory preview with no red (collision) links. Click Execute, confirm the arm in Gazebo (Phase 4) moves to match.

Pass = plan succeeds without "no solution found," execution moves the *simulated Gazebo arm*, not just RViz's ghost.

**Antigravity task prompt (Phase 5):**
```
Using the MoveIt Setup Assistant against the URDF/Xacro in
husky_ur5_description (from the completed, verified Phase 2), generate
husky_ur5_moveit_config with: a planning group "ur5_arm" covering the 6
UR5 joints, a planning group "gripper" for the Robotiq 2F-85, self-collision
matrix generated from the assistant's default sampling, and
moveit_controllers.yaml wired to the joint_trajectory_controller and
gripper_action_controller from Phase 3/husky_ur5_controllers.yaml (match
controller names exactly — do not invent new ones).
Verify by launching move_group.launch.py + demo.launch.py, planning a
motion in RViz's MotionPlanning panel to a reachable pose above the pick
table, confirming a collision-free green preview, and executing it against
the running Gazebo simulation from Phase 4 — report whether the physical
Gazebo arm moved to match the RViz plan.
```

---

## 8. Phase 6 — Nav2

**Goal:** get one hardcoded nav goal working in the warehouse *before* wiring it into the orchestrator's dynamic goal-setting.

**Deliverables:**
```
husky_ur5_bringup/config/
├── nav2_params.yaml     # costmaps, AMCL or slam_toolbox, controller server, planner server
└── warehouse.yaml + warehouse.pgm   # occupancy map, generated by running SLAM once and saving the map
husky_ur5_bringup/launch/
└── nav2_bringup.launch.py
```

**Note on scope:** the original prompt says "AMCL / SLAM" as if interchangeable — they're not, pick one path explicitly. Recommended: run `slam_toolbox` once in the warehouse world to generate a saved map, then switch to **AMCL against that saved map** for the actual demo (localization against a known map is far more reliable for a scripted pick-and-place demo than live SLAM).

**Test / gate:**
```bash
ros2 launch husky_ur5_bringup nav2_bringup.launch.py
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ... # pick-table pose
```
Pass = Nav2 produces a global + local plan, robot drives through the warehouse aisle without colliding with rack/pallet models, and arrives within Nav2's default goal tolerance.

**Antigravity task prompt (Phase 6):**
```
In husky_ur5_bringup: (1) run slam_toolbox once against the warehouse.world
from Phase 4 to generate and save an occupancy map (warehouse.yaml +
warehouse.pgm). (2) Write nav2_params.yaml configured for AMCL localization
against that saved map, with costmaps (global + local) tuned with an
inflation radius appropriate for the Husky's footprint plus a safety
margin, and controller_server/planner_server using the default DWB/NavFn
plugins. (3) Write nav2_bringup.launch.py starting AMCL + Nav2 stack against
the saved map. Verify by publishing a single hardcoded PoseStamped goal at
the pick-table location and confirming via `ros2 topic echo
/plan` that a global plan is produced and the robot drives there without
colliding with any rack, pallet, or barrier model, arriving within Nav2's
default goal tolerance. Report the final pose error.
```

---

## 9. Phase 7 — YOLOv8 Perception Node

**Goal:** exactly as specified in your original prompt, but as an isolated, independently testable node — verify detection + TF broadcast with the robot stationary at the pick table, before it's ever called by the orchestrator.

**Deliverables:**
```
husky_ur5_perception/husky_ur5_perception/
└── yolo_perception_node.py
```

**Node responsibilities (unchanged from your spec, made explicit):**
1. Subscribe `/camera/color/image_raw` (sensor_msgs/Image) and `/camera/depth/image_raw` (sensor_msgs/Image or aligned depth).
2. Run Ultralytics YOLOv8 inference per frame (use a small model — `yolov8n.pt` — for real-time rate in sim).
3. Draw boxes + class + confidence, `cv2.imshow("Live YOLOv8 Target Detection", annotated_frame)`.
4. For the highest-confidence target-class detection: back-project pixel (u,v) + depth to a 3D point in the camera optical frame using the camera intrinsics (`/camera/color/camera_info`).
5. Broadcast a `tf2_ros.TransformBroadcaster` transform `camera_color_optical_frame -> object_target_frame` at that 3D point, at some throttled rate (e.g. 10 Hz, not every frame at full rate if inference is slower).

**Test / gate:**
```bash
ros2 run husky_ur5_perception yolo_perception_node
# with Gazebo (Phase 4) running and robot parked at the pick table
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_target_frame
```
Pass = the `cv2.imshow` window shows correctly drawn boxes on the sim camera feed, `tf2_echo` prints a stable, plausible (x,y,z) — sanity check it against the known simulated position of the target object in the world file, error should be small (a few cm, driven by depth noise if you added any).

**Antigravity task prompt (Phase 7):**
```
In husky_ur5_perception, write yolo_perception_node.py: a rclpy node
subscribing to /camera/color/image_raw and /camera/depth/image_raw
(message_filters ApproximateTimeSynchronizer to pair them), running
Ultralytics YOLOv8 (yolov8n.pt) inference per synced frame pair, drawing
2D bounding boxes + class label + confidence on the frame and displaying it
via cv2.imshow("Live YOLOv8 Target Detection", annotated_frame). For the
highest-confidence detection matching the target object classes, back-
project the box-center pixel using the depth value at that pixel and the
intrinsics from /camera/color/camera_info into a 3D point in the camera
optical frame, and broadcast it via tf2_ros.TransformBroadcaster as
camera_color_optical_frame -> object_target_frame at 10 Hz. Handle the
zero-detections case without crashing (skip the TF broadcast, keep the
window updating). Verify against the running Gazebo warehouse (Phase 4)
with the robot parked at the pick table: confirm the imshow window renders
correct boxes, and `ros2 run tf2_ros tf2_echo camera_color_optical_frame
object_target_frame` reports a stable transform whose position is within
a few cm of the target object's known spawn position in warehouse.world.
```

---

## 10. Phase 8 — Orchestrator State Machine (dry run first)

**Goal:** write and verify the state machine's *transition logic* against mocked/stubbed calls to Nav2 and MoveIt2 before wiring it to the real action servers — this isolates "is my state machine logic correct" from "did Nav2/MoveIt2 hang."

**Deliverables:**
```
husky_ur5_orchestrator/husky_ur5_orchestrator/
└── warehouse_orchestrator.py
```

**States (as specified in your prompt, made explicit as an enum):**
`HOME → NAV_TO_PICK → PERCEIVE → APPROACH_ARM → GRASP → NAV_TO_DROP → PLACE_ARM → RELEASE → RETURN_HOME → HOME`

Each state should be its own method with an explicit success/failure return, logged transition, and a timeout — do not let any state block forever waiting on an action server (a Nav2 goal that never completes should not hang the whole orchestrator silently; it should trip a `RECOVERY`/`ABORT` state after a bounded timeout).

**Test / gate (dry run, no real robot):**
```bash
ros2 run husky_ur5_orchestrator warehouse_orchestrator --ros-args -p dry_run:=true
```
With Nav2/MoveIt2/perception action calls stubbed to instantly return success, confirm the full state sequence logs correctly end to end, and confirm that forcing one stubbed call to return failure correctly trips the recovery/abort path instead of silently hanging or crashing.

**Antigravity task prompt (Phase 8):**
```
In husky_ur5_orchestrator, write warehouse_orchestrator.py: an rclpy node
implementing an explicit state machine (Python enum) with states HOME,
NAV_TO_PICK, PERCEIVE, APPROACH_ARM, GRASP, NAV_TO_DROP, PLACE_ARM,
RELEASE, RETURN_HOME, RECOVERY/ABORT. NAV_TO_PICK/NAV_TO_DROP call Nav2's
NavigateToPose action client; PERCEIVE waits for a fresh
object_target_frame TF (with a timeout) from Phase 7's node;
APPROACH_ARM/PLACE_ARM call MoveIt2's MoveGroup action interface (via
moveit_py or the move_group action client) targeting object_target_frame;
GRASP/RELEASE call the gripper_action_controller. Every state must have a
bounded timeout and transition to RECOVERY/ABORT on failure or timeout
rather than blocking indefinitely. Add a `dry_run` ROS param that stubs
all four external calls (Nav2, MoveIt2, TF wait, gripper) to return success
after a short simulated delay, for logic testing without a running
simulation. Verify by running with dry_run:=true twice: once letting all
stubs succeed (confirm the full state sequence logs HOME through
RETURN_HOME back to HOME), and once forcing one stub to return failure
(confirm it transitions to RECOVERY/ABORT rather than hanging or crashing).
Report both log traces.
```

---

## 11. Phase 9 — Master Launch File & Full Integration

**Goal:** only now — with every subsystem independently verified — wire it all together.

**Deliverables:**
```
husky_ur5_bringup/launch/
└── warehouse_demo.launch.py
```
Brings up, in dependency order (use `TimerAction`/event handlers, not bare concurrent launch, to avoid race conditions where the orchestrator starts before controllers are spawned):
1. Gazebo + warehouse world + robot spawn + ros2_control (Phase 3/4)
2. Nav2 (Phase 6)
3. move_group / MoveIt2 (Phase 5)
4. RViz2 with a saved `.rviz` config showing RobotModel, TF, MotionPlanning panel, camera image display, and the Nav2 display panels
5. YOLOv8 perception node (Phase 7) — the `cv2.imshow` window
6. Orchestrator (Phase 8), with `dry_run:=false`

**Test / gate:**
```bash
ros2 launch husky_ur5_bringup warehouse_demo.launch.py
```
Pass = one full cycle (HOME → pick → drop → HOME) completes with the object visibly transported in Gazebo. This is the first point where "does the whole thing work" is actually being tested — every prior phase de-risked one piece of it.

**Antigravity task prompt (Phase 9):**
```
In husky_ur5_bringup, write warehouse_demo.launch.py that brings up, in
strict dependency order using launch event handlers (not naive concurrent
launch): Gazebo+warehouse world+robot spawn+ros2_control, then (after
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

## 12. Phase 10 — Automated Tests

**Goal:** codify the manual verification from Phases 1–9 as `pytest` + `launch_testing`, so regressions are caught by `colcon test` instead of manual re-running.

**Deliverables:**
```
husky_ur5_bringup/test/
├── test_tf_tree.py            # asserts no missing/duplicate frames after launch
├── test_controllers_active.py  # asserts all ros2_control controllers report active
├── test_perception_tf.py        # asserts object_target_frame appears within N seconds
└── test_full_cycle_launch.py     # launch_testing: runs warehouse_demo, asserts orchestrator reaches RETURN_HOME within a timeout
```

**Test / gate:**
```bash
colcon test --packages-select husky_ur5_bringup husky_ur5_perception husky_ur5_orchestrator
colcon test-result --verbose
```
Pass = all green.

**Antigravity task prompt (Phase 10):**
```
Write pytest + launch_testing tests for husky_ur5_bringup covering:
(1) test_tf_tree.py - launches the description+controllers, waits for TF,
asserts via tf2_ros.Buffer that the expected frame set exists with no
missing parent frames; (2) test_controllers_active.py - asserts every
controller in husky_ur5_controllers.yaml reports 'active' via the
controller_manager list_controllers service within a timeout;
(3) test_perception_tf.py - launches Gazebo+perception node, asserts
object_target_frame becomes available within 15 seconds;
(4) test_full_cycle_launch.py - a launch_testing test that runs
warehouse_demo.launch.py and asserts, via a subscription to the
orchestrator's state topic/log, that it reaches RETURN_HOME within a
generous timeout (e.g. 5 minutes). Run `colcon test --packages-select
husky_ur5_bringup husky_ur5_perception husky_ur5_orchestrator` and
`colcon test-result --verbose`, and report the full pass/fail summary.
```

---

## 13. Phase 11 — Hardening (only after Phase 9 passes once)

This is the "make it actually robust" pass — don't ask for this until you've watched it succeed manually at least once, otherwise you're asking the agent to add retry/recovery logic for failure modes it hasn't observed yet.

**Antigravity task prompt (Phase 11):**
```
The full warehouse_demo pipeline currently completes one HOME->pick->drop
->HOME cycle successfully. Harden warehouse_orchestrator.py to run the
cycle 5 times consecutively without manual intervention: add a
configurable retry count on GRASP failure (re-attempt PERCEIVE+APPROACH_ARM
up to N times before aborting that cycle), reset gripper/arm to a known
safe pose at the start of each cycle, and log a per-cycle summary
(success/failure, duration, retry count). Verify by running
warehouse_demo.launch.py once and confirming via logs that 5 consecutive
cycles complete, reporting the per-cycle summary for all 5.
```

---

## 14. Quick-reference checklist

- [x] Preflight deps installed, confirmed with `ros2 pkg list`
- [x] Phase 1 — Package skeletons build cleanly
- [x] Phase 2 — URDF parses (55 links / 54 joints), RViz shows correct assembled robot
- [x] Phase 3 — ros2_control hardware interface & controllers active on mock hardware
- [x] Phase 4 — Gazebo RTF > 0.7, robot stable on spawn
- [x] Phase 5 — MoveIt2 plans + executes against live Gazebo arm
      (`mobile_manipulator_moveit_config`: groups `ur5_arm` / `gripper`, 162
      disabled pairs from the Setup Assistant's default sampling, execution via
      `arm_controller` + `gripper_action_controller`.  Open follow-ups for
      Phase 6/8, both rooted in Phase 3/4 rather than in MoveIt:
        (a) gazebo_ros2_control publishes the Robotiq mimic joints as
            `<joint>_mimic`, names absent from the URDF — move_group logs an
            error per /joint_states message and aborts outright if a client
            echoes those names back in a RobotState;
        (b) the base is not braked, so extending the arm rolls the robot
            ~0.16 m backwards (wheel odometry over-reports it as 0.26 m),
            which will move any pre-computed grasp pose out from under the
            gripper.)
- [ ] Phase 6 — Nav2 drives a hardcoded goal through the warehouse
- [ ] Phase 7 — YOLO detects + TF position within a few cm of ground truth
- [ ] Phase 8 — orchestrator dry-run: full sequence + failure path both correct
- [ ] Phase 9 — one full real cycle succeeds end-to-end
- [ ] Phase 10 — `colcon test` all green
- [ ] Phase 11 — 5 consecutive cycles succeed unattended
