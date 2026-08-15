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
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
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
# check real-time factor (Fortress; `gz stats` was Gazebo Classic)
ign topic -e -t /world/warehouse/stats
# confirm robot spawned upright, not falling through floor or exploding
```
Pass = Gazebo real-time factor stays above ~0.7, robot sits stably at the home pose for 30s with zero commanded velocity (this confirms the Phase 2 inertials were sane).

**Antigravity task prompt (Phase 4):**
```
In husky_ur5_bringup, create warehouse.world: a Gazebo Fortress (gz-sim 6,
via ros_gz) world with a warehouse
floor plan containing 2+ storage-rack aisles, wooden pallets, and barrier
obstacles, composed from existing Gazebo/Fuel models (do not hand-author
mesh geometry). Include a pick-up workbench with 2-3 spawned target objects
(cube, cylinder, box primitives with distinct colors for YOLO to
distinguish) and a separate drop-off table, plus a directional light and
ambient light tuned so the RGB camera feed is not blown out or too dark.
Add gazebo_warehouse.launch.py that starts Gazebo with this world via
ros_gz_sim's gz_sim.launch.py, spawns the husky_ur5 robot (from Phase 2/3)
at a named "home" pose with the `ros_gz_sim create` executable, using the
ros2_control Gazebo plugin (gz_ros2_control) instead of mock hardware.
Remember the world must load the Sensors system plugin explicitly or no
camera or lidar will ever publish, and that every sensor topic needs a
ros_gz_bridge entry -- including /clock, without which nothing steps.
Verify with real-time factor > 0.7 and confirm via a
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
mobile_manipulator_navigation/config/
├── nav2_params.yaml              # AMCL, costmaps, DWB controller, NavFn planner
└── slam_toolbox_mapping.yaml     # one-shot mapping session config
mobile_manipulator_navigation/maps/
└── warehouse.yaml + warehouse.pgm   # occupancy map, from running SLAM once
mobile_manipulator_navigation/launch/
├── nav2_bringup.launch.py        # map_server + AMCL + Nav2, lifecycle-managed
└── slam.launch.py                # mapping only, not part of the runtime stack
mobile_manipulator_navigation/scripts/
├── mapping_drive.py              # scripted mapping route
└── phase6_nav_goal.py            # gate: send one goal, report plan/error/clearance
```

**Prerequisite discovered here:** the robot had no laser scanner. Phase 6 adds
`mobile_manipulator_description/urdf/lidar_2d.xacro` (a front-bumper puck, scan
plane 0.30 m, 220° FOV, 25 m) plus the matching Gazebo ray sensor in
`gazebo.xacro`, publishing `sensor_msgs/LaserScan` on `/scan`.

**Note on scope:** the original prompt says "AMCL / SLAM" as if interchangeable — they're not, pick one path explicitly. Recommended: run `slam_toolbox` once in the warehouse world to generate a saved map, then switch to **AMCL against that saved map** for the actual demo (localization against a known map is far more reliable for a scripted pick-and-place demo than live SLAM).

**Test / gate:**
```bash
# Gazebo (Phase 4) must be up; kill home_hold first, it pins cmd_vel to zero
pkill -f "[h]ome_hold"
ros2 launch mobile_manipulator_navigation nav2_bringup.launch.py
ros2 run mobile_manipulator_navigation phase6_nav_goal.py --goal 2.9 0.0 0.0
ros2 topic echo /plan --once      # the gate's stated check, run separately
```
Pass = Nav2 produces a global + local plan, robot drives through the warehouse aisle without colliding with rack/pallet models, and arrives within Nav2's default goal tolerance.

**Antigravity task prompt (Phase 6):**
```
In mobile_manipulator_navigation: (1) run slam_toolbox once against the warehouse.world
from Phase 4 to generate and save an occupancy map (warehouse.yaml +
warehouse.pgm). (2) Write nav2_params.yaml configured for AMCL localization
against that saved map, with costmaps (global + local) tuned with an
inflation radius appropriate for the mobile base footprint plus a safety
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
mobile_manipulator_perception/mobile_manipulator_perception/
├── yolo_perception_node.py     # the node itself
├── phase7_look_pose.py         # gate helper: aim the wrist camera at the bench
└── phase7_target_check.py      # gate: sample the TF, score it against the world file
mobile_manipulator_perception/models/
└── yolov8n.pt                  # shipped with the package; no mid-demo download
```

**Node responsibilities (unchanged from your spec, made explicit):**
1. Subscribe `/camera/color/image_raw` (sensor_msgs/Image) and `/camera/depth/image_raw` (sensor_msgs/Image or aligned depth).
2. Run Ultralytics YOLOv8 inference per frame (use a small model — `yolov8n.pt` — for real-time rate in sim).
3. Draw boxes + class + confidence, `cv2.imshow("Live YOLOv8 Target Detection", annotated_frame)`.
4. For the highest-confidence target-class detection: back-project pixel (u,v) + depth to a 3D point in the camera optical frame using the camera intrinsics (`/camera/color/camera_info`).
5. Broadcast a `tf2_ros.TransformBroadcaster` transform `camera_color_optical_frame -> object_target_frame` at that 3D point, at some throttled rate (e.g. 10 Hz, not every frame at full rate if inference is slower).

**Four prerequisites discovered here.** None of them live in the node, and all
four are silent failures — the node runs perfectly while seeing nothing:

- **There was no depth stream.** `gazebo.xacro` only ever declared the colour
  camera. It now also declares a `<sensor type="depth_camera">` rendered by
  the gz-sim Sensors system and carried into ROS by ros_gz_bridge (under
  Gazebo Classic this was `<sensor type="depth">` served by
  `libgazebo_ros_camera.so`). It is mounted on
  the *colour* frame at the same FOV and resolution, so depth pixel (u,v) is
  colour pixel (u,v) and one CameraInfo back-projects both.
- **The wrist camera pointed 90° off its lens axis.** URDF→SDF fixed-joint
  lumping *discards* a `<pose>` authored on a `<sensor>` and substitutes the
  referenced frame's own transform, so the only way to aim a Gazebo camera is
  to reference a frame that is already X-forward. Both sensors now reference
  `camera_color_frame`, keeping `camera_color_optical_frame` for the
  published headers -- under Fortress that is set with
  `<ignition_frame_id>`, not Classic's `<frame_name>`.
- **The bench targets were undetectable by a COCO model.** The world's original
  red cube / blue cylinder / green box were chosen "so YOLO can tell them
  apart", but stock `yolov8n.pt` classifies by learned category, not colour,
  and COCO has no cube class — measured peak confidence 1–3 %, while the
  tabletop itself scored 0.82 as "bed". They are now three 75 mm spheres,
  which read as COCO **"sports ball"** at 0.67–0.89 from the same viewpoint and
  still fit the Robotiq 2F-85's 85 mm stroke.
- **The Gazebo GUI starves the sensor renderer.** Running it alongside
  the two 640x480 wrist cameras drops them from ~6 Hz to under 0.3 Hz on an
  8-core box. Under Fortress `gui:=false` maps to the gz_args `-s` flag
  (server only) rather than skipping a separate gzclient process.
  `gazebo_warehouse.launch.py` gained a `gui` argument; Phase 7 runs with
  `gui:=false`.

**Test / gate:**
```bash
# 1. Gazebo, headless (see above), robot spawned at the pick table
ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py \
    home_x:=3.1 gui:=false

# 2. The perception node, BEFORE the arm moves (loading YOLO takes ~15 s)
ros2 run mobile_manipulator_perception yolo_perception_node \
    --ros-args -p use_sim_time:=true

# 3. Aim the wrist camera at the bench and hold the base there
pkill -f "[h]ome_hold"                       # it pins cmd_vel to zero at 50 Hz
ros2 run mobile_manipulator_perception phase7_look_pose --hold 120 \
    --ros-args -p use_sim_time:=true

# 4. Score it
ros2 run mobile_manipulator_perception phase7_target_check \
    --duration 10 --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_target_frame
```
Pass = the `cv2.imshow` window shows correctly drawn boxes on the sim camera feed, `tf2_echo` prints a stable, plausible (x,y,z) — sanity check it against the known simulated position of the target object in the world file, error should be small (a few cm, driven by depth noise if you added any).

**Achieved:** 20/20 fresh samples over 10 s, transform peak-to-peak **0.7 mm**,
world-frame position error **2.9 mm** against `target_ball_green`'s spawn pose.
The annotated window shows `sports ball 0.89` / `sports ball 0.67` on two balls
(plus a harmless `bed 0.56` on the tabletop), with the locked target boxed in
red and its 3D point printed on the frame.

**Follow-up for Phase 8 — RESOLVED by the Gazebo Fortress migration.** Under
Gazebo Classic the parked base would not stay parked: whenever the arm held an
extended pose it rolled backwards at ~1 cm/s and then, after ~25 s, lurched
~0.4 m sideways and yawed ~0.65 rad. It was genuinely rolling — `/odom` saw it
— because the arm's `position` command interface has no PID, so
gazebo_ros2_control held the joints with a kinematic
`gazebo::physics::Joint::SetPosition` every cycle rather than a torque, and the
wheels were held the same kinematic way and could not absorb the reaction.

Re-measured on Fortress with the same look pose, the base is static: the
`camera_color_optical_frame -> object_target_frame` transform has peak-to-peak
**0.0000 m** on every axis over 20 samples / 10 s. gz_ros2_control's
GazeboSimSystem drives the joints through the physics engine rather than by
kinematic teleport, so the reaction is absorbed normally. `phase7_look_pose
--hold` still station-keeps on `/odom` but is no longer load-bearing, and
Phase 8 no longer needs to fix arm effort control before it can grasp.

**Antigravity task prompt (Phase 7):**
```
In mobile_manipulator_perception, write yolo_perception_node.py: a rclpy node
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
      (reworked 2026-08-14 before Phase 6: rack rows densified to 5 bays,
      barrels/cabinets/brick-blocks as asymmetric lidar landmarks, all
      clutter ≥ 1.0 m off the Phase 6 routes; skid-steer fix — wheel
      mu2 vendor patch + wheel_separation_multiplier — the base could not
      rotate in place before it. Both values were re-derived from ground
      truth in Phase 6 and now stand at mu2 0.15 / multiplier 1.37.
      RTF 0.96 headless.)
- [x] Phase 5 — MoveIt2 plans + executes against live Gazebo arm
      (`mobile_manipulator_moveit_config`: groups `ur5_arm` / `gripper`, 162
      disabled pairs from the Setup Assistant's default sampling, execution via
      `arm_controller` + `gripper_action_controller`.  Open follow-ups for
      Phase 6/8, both rooted in Phase 3/4 rather than in MoveIt:
        (a) gz_ros2_control (gazebo_ros2_control pre-migration) publishes
            the Robotiq mimic joints as
            `<joint>_mimic`, names absent from the URDF — move_group logs an
            error per /joint_states message and aborts outright if a client
            echoes those names back in a RobotState;
        (b) the base is not braked, so extending the arm rolls the robot
            ~0.16 m backwards (wheel odometry over-reports it as 0.26 m),
            which will move any pre-computed grasp pose out from under the
            gripper.)
- [x] Phase 6 — Nav2 drives a hardcoded goal through the warehouse
      (`mobile_manipulator_navigation`: slam_toolbox run once to build
      `maps/warehouse.{yaml,pgm}`, AMCL against that map at runtime, DWB +
      NavFn, inflation 0.75 m on a 0.35 m inscribed footprint.  Gate: 115-pose
      global plan on `/plan`, final pose error **0.245 m / 0.001 rad** inside
      Nav2's default 0.25 m / 0.25 rad, minimum clearance **0.111 m** to a
      jersey barrier while threading the 0.934 m gate, no collision;
      localization error 0.014 m.  Phase 6 also had to add the robot's first
      laser scanner (`urdf/lidar_2d.xacro`, scan plane 0.30 m) and re-derive
      the skid-steer odometry calibration — `wheel_separation_multiplier`
      1.875 → **1.37**, wheel `mu2` → **0.15** — because /odom under-reported
      yaw by 27 % and SLAM diverged within one turn.  Open follow-up for
      Phase 8: the base cannot park closer than x ≈ 3.24 m (front edge against
      the workbench legs), leaving the targets ~1.0 m from the arm base,
      beyond the UR5's 0.85 m reach.)
- [x] Phase 7 — YOLO detects + TF position within a few cm of ground truth
      (`mobile_manipulator_perception/yolo_perception_node.py`: message_filters
      ApproximateTimeSynchronizer over `/camera/color/image_raw` +
      `/camera/depth/image_raw`, Ultralytics YOLOv8n, annotated
      `cv2.imshow("Live YOLOv8 Target Detection")` mirrored on
      `~/annotated_image`, `camera_color_optical_frame -> object_target_frame`
      at 10 Hz.  Gate: **2.9 mm** position error against `target_ball_green`'s
      spawn pose in `warehouse.world`, transform peak-to-peak **0.7 mm** over
      20 samples / 10 s, 0 dropped — run `phase7_target_check`.
      Four things had to be fixed before any of that could work, none of them
      in the node:
        (a) there was no depth stream at all.  `gazebo.xacro` now declares a
            `<sensor type="depth_camera">` rendered by the gz-sim Sensors
            system and bridged by ros_gz_bridge (pre-migration this was
            `<sensor type="depth">` on `libgazebo_ros_camera.so`);
        (b) the wrist camera was pointing 90° off its own lens axis.  A sensor
            under `<gazebo reference="...">` has its authored `<pose>`
            *discarded* by URDF→SDF fixed-joint lumping, so the referenced
            frame's own orientation is the only thing that aims it — both
            camera sensors now hang off `camera_color_frame`, not
            `camera_color_optical_frame`;
        (c) the bench targets were undetectable.  Stock COCO yolov8n has no
            class for a coloured cube: the old cube/cylinder/box peaked at
            1–3 % confidence while the tabletop scored 0.82 as "bed".  They are
            now three 75 mm spheres — COCO "sports ball" at 0.67–0.89, still
            inside the 2F-85's 85 mm stroke;
        (d) the Gazebo GUI was starving the sensor renderer.  Launch with
            `gui:=false` (new argument; under Fortress it maps to the gz_args
            `-s` server-only flag) or the wrist cameras fall from ~6 Hz to
            under 0.3 Hz and the detector sees almost nothing.
      Follow-up for Phase 8 **resolved by the Fortress migration**: under
      Gazebo Classic the parked base rolled backwards ~1 cm/s and then lurched
      ~0.4 m / 0.65 rad after ~25 s whenever the arm held an extended pose,
      because gazebo_ros2_control held the un-PID'd position interface with
      kinematic `Joint::SetPosition`. Re-measured on Fortress the base is
      static: transform peak-to-peak **0.0000 m** over 20 samples / 10 s with
      the arm at the look pose. `phase7_look_pose --hold` is retained but is
      no longer load-bearing.)
- [x] **Gazebo Classic → Fortress migration** (Classic reached EOL Jan 2025).
      `gazebo_ros_pkgs`/`gazebo_ros2_control` → `ros_gz_sim`/`ros_gz_bridge`/
      `gz_ros2_control`, all from packages.ros.org (Fortress 6.18.0 is mirrored
      there, so no osrfoundation repo and no conflict with the existing
      ros-humble-* moveit/nav2/ur_description binaries).  Note the Fortress CLI
      is `ign gazebo`, **not** `gz sim` — `/usr/bin/gz` still belongs to Classic,
      which remains installed alongside.  What changed:
        * `gz_sim.launch.py` + `ros_gz_sim create` replace `gazebo.launch.py` +
          `spawn_entity.py`; `-entity` became `-name`; `GAZEBO_MODEL_PATH`
          became `IGN_GAZEBO_RESOURCE_PATH`.
        * The world must load Physics / UserCommands / SceneBroadcaster /
          **Sensors** explicitly.  Sensors is never implicit, and without it
          every camera and the lidar stay silent while the sim looks healthy.
        * Sensors lost their per-sensor ROS plugins; topic names come from
          `<topic>` and frame_ids from `<ignition_frame_id>`, mapped to frozen
          ROS names in `mobile_manipulator_gazebo/config/ros_gz_bridge.yaml`.
          `/clock` must be bridged or every use_sim_time node sits at t=0.
        * Ogre `<material><script>` is unsupported; the world's 10 blocks became
          PBR.  Models vendored from `~/.gazebo/models` still carry Ogre scripts
          and therefore render **black**, which darkens the scene.
        * Ground truth: `gz model -m <name> -p` no longer exists.  Both gate
          scripts now read `/world/warehouse/pose/info` (pose/info, not
          dynamic_pose/info — the latter omits a parked robot).
        * **Odometry had to be recalibrated.** `wheel_separation_multiplier`
          1.37 encoded ODE's 0.70 m effective track; DART's is 0.5573 m, so the
          value is now **1.0889**.  Under 1.37 /odom under-reported yaw by
          20.5 %, which rotated the SLAM map frame 17.7° off the world frame and
          broke the Phase 6 gate.  `maps/warehouse.{pgm,yaml}` regenerated on
          Fortress.  The wheel `mu2` 0.15 patch is still load-bearing — DART does
          honour anisotropic friction (with mu2 back at 1.0 the body reaches only
          0.31 of commanded yaw).
      Re-gated after migration: Phase 3 controllers all active; RTF 0.995;
      camera/depth/camera_info steady at 6.6–8.0 Hz; Phase 6 final pose error
      **0.104 m / 0.130 rad** (better than Classic's 0.245 m), collision-free;
      Phase 7 **2.2 mm** against `target_ball_green`, transform peak-to-peak
      0.0000 m.
      Two known issues carried forward:
        (a) YOLO confidence on the target spheres fell from 0.67–0.89 to **0.35**
            with only 1 of 3 balls detected, because the vendored `model://`
            furniture renders black.  Giving those models PBR materials (or
            raising scene light) should restore it; matters for Phase 8 PERCEIVE.
        (b) The **return** leg (pick table → origin) threads the 0.934 m barrier
            gate east-to-west very slowly — 16.6 m driven for a 2.71 m plan, 172
            replans — and exceeds `phase6_nav_goal.py`'s timeout.  It does
            converge (0.173 m) and is collision-free.  Phase 8 RETURN_HOME needs
            a longer allowance.
- [x] Phase 8 — orchestrator dry-run: full sequence + failure path both correct
      (`mobile_manipulator_orchestrator/warehouse_orchestrator.py`: `State` enum
      with HOME / NAV_TO_PICK / PERCEIVE / APPROACH_ARM / GRASP / NAV_TO_DROP /
      PLACE_ARM / RELEASE / RETURN_HOME plus RECOVERY, ABORT and DONE.  Each
      state is its own method returning `(ok, detail)` — failure is data, never
      an exception — and every external call is bounded by `_await_future()`,
      which polls against a `time.monotonic()` deadline.  A deadline built from
      `get_clock()` would expire the instant sim time arrives, so that choice is
      load-bearing, not stylistic.  RECOVERY owns the retry budget
      (`max_retries`, default 2) and falls through to ABORT.
      `dry_run:=true` stubs all four interfaces (Nav2 `/navigate_to_pose`, the
      `object_target_frame` TF wait, MoveIt `/move_action`, and
      `/gripper_action_controller/gripper_cmd`) so the transition logic can be
      gated with no simulator running; `dry_run_fail_state:=<STATE>` forces
      exactly one stub to fail.  Gate, both runs:
        success → HOME -> NAV_TO_PICK -> PERCEIVE -> APPROACH_ARM -> GRASP ->
                  NAV_TO_DROP -> PLACE_ARM -> RELEASE -> RETURN_HOME -> DONE,
                  exit 0;
        failure → ... -> GRASP -> RECOVERY -> GRASP -> RECOVERY -> GRASP ->
                  RECOVERY -> ABORT, exit 1, no hang and no crash.
      Live-path notes for Phase 9: `nav_timeout` defaults to 180 s because the
      return leg through the 0.934 m barrier gate is slow, and the MoveGroup
      request sends an empty `is_diff` RobotState so it can never echo the
      Gazebo mimic joint names back and crash move_group.)
- [~] Phase 9 — master launch built; **object transport demonstrated once
      (run 12) but NOT repeatable**, so the gate is NOT signed off.
      (`mobile_manipulator_gazebo/launch/warehouse_demo.launch.py` +
      `config/warehouse_demo.rviz`.  Bringup is ordered by *observed state*,
      not timers: each stage is separated by a gate process that polls for the
      thing the next stage needs and exits 0, with an OnProcessExit handler
      starting the next stage and a Shutdown on timeout.  The controllers gate
      is a *sequence* — four active, then home_hold appeared, then it left —
      because a stale controller_manager from an earlier session satisfied a
      plain state check in 2 s and brought the whole stack up against a
      simulation whose robot had not spawned.

      WORKS repeatably (5+ consecutive runs): bringup ordering; HOME;
      NAV_TO_PICK + dock (~29 mm along-body error); PERCEIVE (detection within
      ~20 mm of gz ground truth); APPROACH_ARM; GRASP (knuckle stalls at
      0.205-0.231 rad against a 0.30 command — the predicted band for a 66 mm
      sphere); lift + tuck; NAV_TO_DROP + dock; PLACE_ARM; RELEASE.
      Run 12 left `target_ball_green` on the drop bench at
      (3.844, 3.510, 0.633) — z exactly the resting height — other two targets
      untouched.  Runs 7 and 14 reached DONE with the object NOT delivered,
      which is exactly why the orchestrator's own trace is not evidence.

      NINE distinct defects found and fixed, each by measurement:
        1. REACH.  The vendored 1.0 m `model://table` put targets outside the
           UR5 envelope — /compute_ik returns NO_IK_SOLUTION beyond 0.45 m at
           z=1.0525 and the base cannot park closer than x=3.24.  Benches
           re-authored at 0.60 m (PBR, same leg footprint so the saved map
           stays valid); targets moved to x=3.92, 75 mm -> 66 mm for pad
           clearance.  Reachable band is now 0.45-0.85 m.
        2. `topdown_quat()` never assigned z/w and geometry_msgs/Quaternion
           defaults w=1, so every "top-down" goal was (1,0,0,1) — a 90 deg roll
           aiming the gripper sideways along -Y.  MoveIt warned and planned
           anyway; the descent swept the forearm into the chassis.
        3. `grasp_offset` was measured to the fingertip LINK ORIGIN (0.109 m
           below tool0) when the pad MESH reaches 0.160 m open / 0.169 closed.
           The pads were driven 9 mm into the bench (measured at the stall: pad
           face z=0.5989 against a 0.600 surface).  0.115 -> 0.145.
        4. SRDF gap: `camera_link` vs `gripper_robotiq_85_left_knuckle_link`
           interferes up to 3.88 mm only while the jaws close through
           0.1-0.55 rad — neither Default nor Always nor Never, so the sampler
           left it enabled and every plan after GRASP died on an invalid start
           state.  An audit of all camera/bracket-vs-gripper pairs found it the
           only interfering pair still enabled.
        5. `collisions_updater` DROPS the input SRDF's own <disable_collisions>,
           so regenerate_collision_matrix.sh now re-merges them — parsed as XML,
           after a regex matched the phrase inside a comment and spliced a
           second <robot> element into the output.
        6. The Cartesian carry produced a ~4 rad wrist_1 excursion: 7 of 8
           probed carry poses came back from KDL wound past 3pi/2, and
           `jump_threshold: 0.0` DISABLES the discontinuity check.  Carry is now
           a named joint configuration; the threshold is 1.5 rad.
        7. **The unifying timeout cause.**  MoveIt derives its execution bound
           from the trajectory's SIM-TIME duration and enforces it against a
           WALL clock.  The full stack saturates this 8-core box — measured RTF
           0.48, load average 7.9 — so trajectories need ~2.08x their duration
           in wall time against `allowed_execution_duration_scaling: 2.0`.
           That is why failures were non-deterministic, and why slowing the
           trajectories changed nothing: both sides scale together.  Now 6.0.
        8. Nav2 could not plan off the pick dock.  The live global costmap had
           the drop goal and the whole corridor at cost 0 — the route was never
           the problem, the START was: docking parks the footprint ~1 cm from
           the bench legs' lethal cells, DWB vetoes every trajectory, the BT
           escalates to Spin, and a Spin walks this skid-steer ~0.12 m sideways
           per radian unobservably.  Added an undock before every navigation
           goal; planner failures went from 4-6 per run to zero.
        9. AMCL diverged on any leg requiring a turn — measured against gz
           ground truth at 2 Hz: 0.18 m entering the turn, 0.44 m immediately
           after, growing past 0.77 m while AMCL insisted the robot drove north
           as it went south.  alpha4 (translation noise from rotation)
           0.2 -> 0.6, alpha1 0.3 -> 0.6.

      OPEN, and why the gate is not signed off:
        (a) PLACE/RELEASE is not repeatable.  Run 12 delivered; run 14 flung the
            object to (6.48, 9.41) on the floor while still logging DONE.  The
            machine cannot tell a real delivery from a failed one — it needs
            post-place verification (re-observe the object with the camera)
            rather than trusting the sequence.
        (b) The carry configuration self-collides (`arm_forearm_link` vs
            `arm_wrist_2_link`) in some runs; /compute_ik approved it but the
            executed configuration crosses the threshold.  Re-solve against
            /check_state_validity with margin.
        (c) RTF 0.48 is the environment, not the code.  Every timeout in this
            phase traces to it.  Phases 10/11 must budget for it or cut load.
        (d) The wrist camera image is rolled 45.8 deg at the LOOK pose (90 deg
            at stow).  The sensor pose and TF chain are provably CORRECT —
            camera_color_frame -> camera_color_optical_frame is exactly
            rpy(-pi/2, 0, -pi/2), both streams stamp camera_color_optical_frame,
            and no legacy gazebo_ros plugin remains — the roll comes from the
            arm's wrist configuration.  Back-projection stays accurate because a
            roll about the view axis barely moves a target near the image
            centre.  The fix belongs in the look_pose IK, never in the node.
        (e) The 0.60 m benches are drivable-under as far as a 0.30 m scan plane
            is concerned; the robot drove under one during a recovery.
        (f) The camera mount genuinely occupies the knuckle's sweep volume (the
            bracket penetrates 11.67 mm even fully open).  Re-siting it is
            description-level work that would invalidate Phase 7's calibration,
            so it is deferred to hardening.)
- [ ] Phase 10 — `colcon test` all green
- [ ] Phase 11 — 5 consecutive cycles succeed unattended
