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

Status (checklist lives in `PLAN.md` §14, keep it updated): Phases 1–4 complete
(bootstrap, description, ros2_control, Gazebo world). Phases 5–11 pending —
`mobile_manipulator_moveit_config`, `mobile_manipulator_navigation`,
`mobile_manipulator_perception`, and `mobile_manipulator_orchestrator` are deliberately
empty skeletons awaiting their phase.

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
repo* (a bare gitlink, not a configured submodule), and it carries a local uncommitted
patch (`urdf/husky.urdf.xacro`, the `HUSKY_GAZEBO_PLUGINS` guard). Do not blow away or
re-clone that directory without re-applying the patch, and commit changes to it inside its
own repo.
