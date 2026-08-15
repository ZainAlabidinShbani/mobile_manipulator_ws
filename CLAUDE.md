# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 **Humble** colcon workspace (Ubuntu 22.04, user shell is zsh) building a warehouse
pick-and-place mobile manipulator in **Gazebo Classic**: Clearpath Husky base + UR5 arm +
Robotiq 2F-85 gripper + RealSense D435i wrist camera, driven by ros2_control, MoveIt 2,
Nav2, a YOLOv8 perception node, and a state-machine orchestrator.

Work is executed as **11 sequential phases** defined in `PLAN.md`, each with a hard
verification gate. `prompts/` holds the per-phase task prompt plus the exact terminal
command sequence and known gotchas for that phase — **read `prompts/NN_phaseN_*.md` before
starting a phase**; it usually already documents the traps.

Status (checklist lives in `PLAN.md` §14, keep it updated): Phases 1–6 complete
(bootstrap, description, ros2_control, Gazebo world, MoveIt 2 config, Nav2). Phases
7–11 pending — `mobile_manipulator_perception` and `mobile_manipulator_orchestrator`
are deliberately empty skeletons awaiting their phase.

## Non-negotiable project rules (`.agents/rules/instructions.md`)

- **Never use the word "husky" in anything we author** — package names, directories, node
  names, launch files, target names, parameters. Everything we own is prefixed
  `mobile_manipulator_*`. The vendor package `husky_description` is *referenced* (include,
  depend, `$(find ...)`) but never renamed or re-implemented.
- **Do not start phase N+1 until phase N's gate passes with zero errors.** On failure,
  isolate the root cause inside the current phase rather than patching several subsystems.
- Workspace root is always `~/mobile_manipulator_ws`; run everything from there.
- When adding a dependency, update `package.xml` **and** `CMakeLists.txt`/`setup.py`.

## Commands

```bash
# Build (from workspace root)
colcon build --symlink-install
colcon build --symlink-install --packages-select mobile_manipulator_description

# Every shell needs this preamble
source /opt/ros/humble/setup.bash
source ~/mobile_manipulator_ws/install/setup.bash   # install/setup.zsh under zsh
export ROS_LOCALHOST_ONLY=1

# Kill stale ros2 daemons — they make every `ros2` CLI call fail with
# "xmlrpc.client.Fault: ... !rclpy.ok()"
for p in $(ps aux | grep "[r]os2cli.daemon" | awk '{print $2}'); do kill -9 $p; done

# Phase 2 gate — URDF sanity
xacro src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro sensor_arch:=0 > /tmp/mm.urdf
check_urdf /tmp/mm.urdf
ros2 launch mobile_manipulator_description view_robot.launch.py     # RViz only

# Phase 3 gate — mock-hardware controllers
ros2 launch mobile_manipulator_description control_test.launch.py
ros2 control list_controllers                                        # all 4 must be "active"

# Phase 4 gate — Gazebo warehouse
ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py
gz stats                                                             # RTF > 0.7
ros2 run mobile_manipulator_gazebo capture_screenshot.py \
  --topic /phase4_camera/image_raw --out /tmp/phase4_home_pose.png   # headless screenshot

# Phase 5 gate — MoveIt 2 (Gazebo from Phase 4 must already be running)
ros2 launch mobile_manipulator_moveit_config move_group.launch.py use_sim_time:=true
ros2 launch mobile_manipulator_moveit_config moveit_rviz.launch.py use_sim_time:=true
ros2 run mobile_manipulator_moveit_config phase5_plan_execute.py \
  --frame base_footprint --use-sim-time \
  --goal-config 0.0 -1.1175 0.1054 -1.2083 -1.5708 0.0 \
  --workbench 1.307 0.0 1.000 1.50 0.80 0.03 --workbench-frame odom
ros2 launch mobile_manipulator_moveit_config demo.launch.py          # mock bench, no Gazebo

# Phase 6 gate — Nav2 (Gazebo from Phase 4 running; kill home_hold first!)
pkill -f "[h]ome_hold"                    # it pins cmd_vel to zero at 50 Hz
ros2 launch mobile_manipulator_navigation nav2_bringup.launch.py
ros2 run mobile_manipulator_navigation phase6_nav_goal.py --goal 2.9 0.0 0.0

# Re-map the warehouse (only needed if the world changes)
ros2 launch mobile_manipulator_navigation slam.launch.py
ros2 run mobile_manipulator_navigation mapping_drive.py --ros-args -p use_sim_time:=true
ros2 run nav2_map_server map_saver_cli \
  -f src/mobile_manipulator_navigation/maps/warehouse --ros-args -p use_sim_time:=true

# Tests (Phase 10 will populate them)
colcon test --packages-select <pkg> && colcon test-result --verbose
```

**Launch long-running sims detached** — plain background jobs get reaped between agent
tool calls:

```bash
setsid nohup ros2 launch <pkg> <file>.launch.py > /tmp/<name>.log 2>&1 < /dev/null &
```

`pgrep`/`pkill` footgun: process names truncate to 15 chars (`ros2_control_no`), and
`-f` patterns match your own shell. Use `pgrep -f "[c]ontrol_test.launch"` or PIDs.

## Architecture

### Description composition (`mobile_manipulator_description`)

`urdf/mobile_manipulator.urdf.xacro` is the single top-level entry point. It **composes
vendor xacros, never re-authors geometry**:

```
base_footprint → base_link (husky_description) → top_plate_link
  → arm_mounting_plate (our arm_mount.xacro)
    → arm_base_link … arm_tool0 (ur_description ur_robot macro, tf_prefix "arm_")
      → gripper adapter + robotiq_85_* (robotiq_description, prefix "gripper_")
      → camera_link / camera_color_optical_frame (our d435i_wrist_mount.xacro)
```

Prefixes matter: `arm_prefix` **must** stay non-empty or the UR5's `base_link` collides
with the Husky's. Controller YAML, MoveIt SRDF, and any new code must use the prefixed
joint names (`arm_shoulder_pan_joint`, `gripper_robotiq_85_left_knuckle_joint`, …).

Key xacro args: `use_gazebo` (false → mock hardware; true → Gazebo backend + sensors),
`controllers_yaml` (absolute path, injected by the Gazebo launch — gzserver cannot resolve
`$(find ...)` inside plugin SDF), `sensor_arch:=0` (suppresses the Husky's decorative arch,
which the arm mount replaces).

### The `use_gazebo` switch

`urdf/ros2_control.xacro` declares three `<ros2_control>` systems (`base_system`,
`arm_system`, `gripper_system`) with identical joint layouts under both backends —
`mock_components/GenericSystem` (Phase 3 bench) or `gazebo_ros2_control/GazeboSystem`
(Phase 4+). `urdf/gazebo.xacro` is emitted only under `use_gazebo:=true` and adds the
`gazebo_ros2_control` model plugin plus the D435i RGB sensor. **Any new joint must be added
to both the URDF and the matching `<ros2_control>` block, and to
`config/mobile_manipulator_controllers.yaml`.**

Two different controller-manager topologies follow from this:
- **Mock (`control_test.launch.py`)** — a standalone `ros2_control_node` process.
- **Gazebo (`gazebo_warehouse.launch.py`)** — the controller manager runs *inside gzserver*
  via `libgazebo_ros2_control.so`; there is no `ros2_control_node`. Controller spawners are
  chained with `OnProcessExit` handlers (jsb → diff_drive → arm → gripper) to avoid
  parameter races.

### Controller configuration

`config/mobile_manipulator_controllers.yaml` is one file passed as `--params-file`,
carrying both the `controller_manager` type declarations and each controller's own
parameter namespace — `diff_drive_controller` reads `wheel_separation` during `on_init()`,
before a spawner's `-p` could inject it, so it has to live here.

Controllers: `joint_state_broadcaster`, `diff_drive_controller`, `arm_controller`
(JointTrajectoryController), `gripper_action_controller` (position_controllers/
GripperActionController). Later phases (MoveIt controllers, orchestrator) must reference
these names exactly rather than inventing new ones.

### Gazebo world (`mobile_manipulator_gazebo`)

`worlds/warehouse.world` is Gazebo Classic SDF: hand-authored slab/walls plus
`model://bookshelf`, `model://euro_pallet` etc. from `~/.gazebo/models`, target primitives
on a pick workbench, a drop-off table, and a world-fixed `phase4_camera` used for headless
screenshots. `scripts/home_hold.py` runs after spawn to publish zero `cmd_vel` (the wheels
free-roll until diff_drive sees its first command) and stow the arm.

### MoveIt config (`mobile_manipulator_moveit_config`)

Setup Assistant output, but assembled headlessly. `config/mobile_manipulator.urdf.xacro`
is a **wrapper** that includes the description package's xacro — never a copy, so the
planning model cannot drift from the Gazebo one. The SRDF is split: hand-authored
semantics in `config/mobile_manipulator.srdf.base`, and
`scripts/regenerate_collision_matrix.sh` runs the assistant's own headless sampler
(`/opt/ros/humble/lib/moveit_setup_assistant/collisions_updater`, defaults: 10000 trials /
0.95 fraction) to merge `<disable_collisions>` into `config/mobile_manipulator.srdf`.
Re-run it after any URDF or planning-group change; never hand-edit the merged file.

Groups: `ur5_arm` (chain `arm_base_link → arm_tool0`, exactly the 6 `arm_*` joints) and
`gripper` (only `gripper_robotiq_85_left_knuckle_joint` — the five URDF `<mimic>` followers
must stay out). `virtual_joint` is **planar**, `odom → base_footprint`, so MoveIt tracks
the mobile base from diff_drive's TF; that is why `demo.launch.py` spawns
`diff_drive_controller` even though MoveIt never commands the base. Two launch topologies:
`demo.launch.py` (own `ros2_control_node` + mock hardware, no Gazebo) and
`move_group.launch.py` (move_group only, for the Phase 4 Gazebo stack) — running both at
once gives you two controller managers.

### Navigation (`mobile_manipulator_navigation`)

`config/nav2_params.yaml` runs **AMCL against the saved map** (`maps/warehouse.yaml`),
not live SLAM — `launch/slam.launch.py` + `scripts/mapping_drive.py` exist only to
regenerate that map after a world change. AMCL's `set_initial_pose` is `(0,0,0)`,
which is valid only because slam_toolbox anchors the map frame where mapping started,
so **map ≡ world** provided mapping begins at the spawn pose. Costmaps use the real
footprint (chassis + the lidar puck that protrudes to x = +0.575), inscribed radius
0.35 m, `inflation_radius: 0.75`. DWB uses the **`ObstacleFootprint`** critic, not the
default `BaseObstacle` — the barrier gate at x = 1.6 is 0.934 m wide for a 0.70 m
robot, and `BaseObstacle` (which scores only the centre cell) vetoes every trajectory
through it. `cmd_vel` is remapped to `/diff_drive_controller/cmd_vel_unstamped`.

`scripts/phase6_nav_goal.py` is the phase gate: it publishes one `/goal_pose`, records
the `/plan`, tracks Gazebo ground truth, and checks the robot polygon against every
obstacle footprint parsed out of `warehouse.world`.

## Environment traps worth knowing before you debug

- `ROS_LOCALHOST_ONLY=1` is mandatory on this machine (WiFi + VPN interfaces make FastDDS
  multicast discovery intermittent). Export it for the launch *and* every CLI call.
- `diff_drive_controller` subscribes with `SystemDefaultsQoS()` → BEST_EFFORT, and
  `use_stamped_vel: false` means the topic is `/diff_drive_controller/cmd_vel_unstamped`.
  Publish with `--qos-reliability best_effort` or messages are silently dropped.
- `position_feedback: false` is required — mock hardware never integrates velocity into
  position, so the Humble default (`true`) leaves odom at zero forever.
- Gripper action is `/gripper_action_controller/gripper_cmd` on Humble (not
  `gripper_command`).
- **Under Gazebo, `/joint_states` contains joint names that are not in the URDF.**
  `libgazebo_hardware_plugins.so` publishes the Robotiq mimic joints as
  `gripper_robotiq_85_*_joint_mimic`. move_group logs `Joint '..._mimic' not found in
  model` at ~50 Hz, and **crashes** (`terminate called after throwing moveit::Exception`)
  if a client echoes those names back inside a `RobotState`. Always build `RobotState`
  as an arm-only diff (`is_diff = true`, six `arm_*` names). Root cause is in
  `ros2_control.xacro`'s Gazebo backend, not in MoveIt.
- **The base is not braked**: extending the arm rolls the robot ~0.16 m backwards, and
  wheel odometry over-reports it (~0.26 m) because the wheels also slip. Validate base and
  end-effector world positions with `gz model -m mobile_manipulator -p`, never with
  `/diff_drive_controller/odom`. This will invalidate any grasp pose computed before the
  arm moves — fix before Phase 8.
- **The base has no laser scanner until `urdf/lidar_2d.xacro` is included** (added in
  Phase 6): front puck, scan plane **0.30 m**, 220° FOV. The FOV cannot exceed ±110°
  at that mount point or the robot scans its own chassis corners (113.7°) and wheel
  tops (127°). Anything shorter than 0.30 m — euro pallets are 0.145 m — is invisible
  to SLAM, AMCL and both costmaps, so Nav2 will plan straight through a pallet.
- **Skid-steer odometry is only trustworthy after calibration.** `/odom` yaw was 27 %
  low with Clearpath's `wheel_separation_multiplier: 1.875`; the measured effective
  track on this slab is 0.70 m, hence **1.37** (with vendor wheel `mu2` patched to
  **0.15**). Get this wrong and slam_toolbox diverges inside a single turn while
  reporting no errors. Re-measure with `gz model -m mobile_manipulator -p` after any
  change to wheel friction, mass, or the arm's stowed pose.
- **Rotating in place is the expensive manoeuvre**: a hard stiction deadband below
  ~0.27 rad/s (the base simply does not move, so any tapering P-controller stalls),
  and it walks the base sideways ~0.12 m per radian. Rolling turns track within 2 %.
- **`home_hold.py` fights any navigation stack** — it publishes zero `cmd_vel` at
  50 Hz. `pkill -f "[h]ome_hold"` before sending goals.
- A stale `gzserver` holds port 11345 and the next launch dies with `Address already
  in use` deep in the log. Kill by PID (`pgrep -f gzserver`): `pkill -f` patterns also
  match the shell running them.
- URDF built via `Command([...xacro...])` in a launch file **must** be wrapped in
  `ParameterValue(..., value_type=str)`, else launch dies with "Unable to parse the value
  of parameter robot_description as yaml".
- `gazebo_warehouse.launch.py` runs `xacro` eagerly via `subprocess` and strips XML
  comments with a regex: gazebo_ros2_control 0.4.x re-injects `robot_description` as an rcl
  `--param` override whose YAML lexer chokes on comment characters. Keep that
  post-processing if you rewrite the launch file.
- The same launch sets `GAZEBO_MODEL_PATH` (including `src/`) and blanks
  `GAZEBO_MODEL_DATABASE_URI` — URDF→SDF rewrites `package://` mesh URIs to `model://`, and
  without these gzserver hangs trying the dead online model database.
- It also sets `HUSKY_GAZEBO_PLUGINS=0`, an `optenv` guard added to the vendor
  `husky.urdf.xacro`, disabling the classic `gazebo_ros` diff_drive / joint_state / imu /
  gps plugins that would fight ros2_control over the wheel joints.

## Vendor dependencies

`ur_description`, `robotiq_description`, `realsense2_description`, MoveIt, Nav2,
ros2_control, gazebo_ros_pkgs come from apt (see `PLAN.md` §2 / `prompts/00_environment_preflight.sh`).
**`husky_description` is not available on Humble via apt** — it is cloned from
`github.com/akrbot/husky_description_ros2` into `src/husky_description` as a *nested git
repo* (a bare gitlink, not a configured submodule), and it carries two local patches
committed inside its own history: the `HUSKY_GAZEBO_PLUGINS` guard in
`urdf/husky.urdf.xacro`, and wheel lateral friction `mu2` 1.0 → 0.15 in
`urdf/wheel.urdf.xacro` (without which the skid-steer base cannot rotate in place).
Do not blow away or
re-clone that directory without re-applying the patch, and commit changes to it inside its
own repo.
