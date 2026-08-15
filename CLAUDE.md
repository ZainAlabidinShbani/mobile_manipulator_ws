# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 **Humble** colcon workspace (Ubuntu 22.04, user shell is zsh) building a warehouse
pick-and-place mobile manipulator in **Gazebo Fortress** (gz-sim 6, via ros_gz): Clearpath Husky base + UR5 arm +
Robotiq 2F-85 gripper + RealSense D435i wrist camera, driven by ros2_control, MoveIt 2,
Nav2, a YOLOv8 perception node, and a state-machine orchestrator.

Work is executed as **11 sequential phases** defined in `PLAN.md`, each with a hard
verification gate. `prompts/` holds the per-phase task prompt plus the exact terminal
command sequence and known gotchas for that phase — **read `prompts/NN_phaseN_*.md` before
starting a phase**; it usually already documents the traps.

Status (checklist lives in `PLAN.md` §14, keep it updated): Phases 1–7 complete
(bootstrap, description, ros2_control, Gazebo world, MoveIt 2 config, Nav2, YOLOv8
perception), **and the whole stack has been migrated from Gazebo Classic (EOL
Jan 2025) to Gazebo Fortress + ros_gz** and re-gated. Phases 8–11 pending —
`mobile_manipulator_orchestrator` is a deliberately empty skeleton awaiting its
phase.

**The Fortress CLI is `ign gazebo`, not `gz sim`** (that spelling is Garden and
later). `/usr/bin/gz` still belongs to Gazebo Classic, which remains installed
alongside — so a stray `gz stats` or `gz model` will appear to "work" and then
silently tell you nothing about the running simulation.

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
ign topic -e -t /world/warehouse/stats                               # RTF > 0.7
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

# Phase 7 gate — YOLOv8 perception (order matters; see the base-creep trap below)
ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py home_x:=3.1 gui:=false
ros2 run mobile_manipulator_perception yolo_perception_node --ros-args -p use_sim_time:=true
pkill -f "[h]ome_hold"
ros2 run mobile_manipulator_perception phase7_look_pose --hold 120 --ros-args -p use_sim_time:=true
ros2 run mobile_manipulator_perception phase7_target_check --duration 10 --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_target_frame

# Ground truth (Classic's `gz model -m <name> -p` does not exist in Fortress).
# Use pose/info, NOT dynamic_pose/info — the latter omits a parked robot.
ign topic -e -t /world/warehouse/pose/info -n 1

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
`controllers_yaml` (absolute path, injected by the Gazebo launch — the gz-sim server
cannot resolve `$(find ...)` inside plugin SDF), `sensor_arch:=0` (suppresses the Husky's decorative arch,
which the arm mount replaces).

### The `use_gazebo` switch

`urdf/ros2_control.xacro` declares three `<ros2_control>` systems (`base_system`,
`arm_system`, `gripper_system`) with identical joint layouts under both backends —
`mock_components/GenericSystem` (Phase 3 bench) or `gz_ros2_control/GazeboSimSystem`
(Phase 4+). `urdf/gazebo.xacro` is emitted only under `use_gazebo:=true` and adds the
`libgz_ros2_control-system.so` plugin (addressed by its class name,
`gz_ros2_control::GazeboSimROS2ControlPlugin`) plus the D435i colour and depth
sensors and the 2D lidar. **Any new joint must be added
to both the URDF and the matching `<ros2_control>` block, and to
`config/mobile_manipulator_controllers.yaml`.**

Two different controller-manager topologies follow from this:
- **Mock (`control_test.launch.py`)** — a standalone `ros2_control_node` process.
- **Gazebo (`gazebo_warehouse.launch.py`)** — the controller manager runs *inside the
  gz-sim server* via `libgz_ros2_control-system.so`; there is no `ros2_control_node`.
  Controller spawners are chained with `OnProcessExit` handlers (jsb → diff_drive → arm
  → gripper) to avoid parameter races. The same launch also starts the
  `ros_gz_bridge` described below.

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

`worlds/warehouse.world` is SDF 1.7 loaded by gz-sim: hand-authored slab/walls plus
`model://bookshelf`, `model://euro_pallet` etc. from `~/.gazebo/models` (reached through
`IGN_GAZEBO_RESOURCE_PATH`, which replaced `GAZEBO_MODEL_PATH`), target primitives on a
pick workbench, a drop-off table, and a world-fixed `phase4_camera` used for headless
screenshots.

It declares four gz-sim systems explicitly — Physics, UserCommands (this is what serves
the spawn service `ros_gz_sim create` calls), SceneBroadcaster, and **Sensors**. Sensors
is never loaded implicitly, and without it every camera and the lidar stay silent while
the simulation otherwise looks perfectly healthy.

`config/ros_gz_bridge.yaml` maps gz transport topics to their frozen ROS names
(`/clock`, `/camera/color/image_raw`, `/camera/color/camera_info`,
`/camera/depth/image_raw`, `/scan`, `/phase4_camera/image_raw`). The gz-side names come
from `<topic>` in `gazebo.xacro` and the world — change one and you must change the
other. `/joint_states` is deliberately **not** bridged: gz_ros2_control runs
controller_manager as a native ROS 2 node, so joint_state_broadcaster already publishes
it, and a bridge entry would add a second competing publisher. `scripts/home_hold.py` runs after spawn to publish zero `cmd_vel` (the wheels
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

### Perception (`mobile_manipulator_perception`)

`yolo_perception_node.py` pairs `/camera/color/image_raw` and
`/camera/depth/image_raw` with a `message_filters` ApproximateTimeSynchronizer,
runs Ultralytics YOLOv8n per pair, annotates boxes/class/confidence into
`cv2.imshow("Live YOLOv8 Target Detection")` (mirrored on `~/annotated_image` so
it can be checked headlessly), back-projects the chosen box centre with the
`/camera/color/camera_info` intrinsics, and broadcasts
`camera_color_optical_frame -> object_target_frame` at 10 Hz. With no fresh
detection it simply stops broadcasting.

Weights ship in the package (`models/yolov8n.pt`). The three executables are
console-script entry points, which is why the package needs `setup.cfg` —
without the `[develop] script_dir` / `[install] install_scripts` stanza colcon
installs them to `install/<pkg>/bin` and `ros2 run` reports `No executable
found`.

Two behaviours to know before touching it: it **locks onto one target**
(`track_radius`, default 0.15 m), because three interchangeable balls make
"highest confidence" flip frame to frame and teleport the TF between objects;
and it adds `target_radius_m` (default 0.0375) along the view ray, because depth
measures the object's front surface rather than its centroid.

`phase7_look_pose.py` aims the wrist camera at the bench via a **two-waypoint**
trajectory — interpolating straight from the stowed pose drags the gripper
through the workbench slab, and Gazebo then launches the robot off the map while
the controllers report success the whole way. `--hold` station-keeps the base on
`/odom`. `phase7_target_check.py` is the gate: it composes the broadcast point
with Gazebo ground truth for `world -> base_footprint` and TF (pure FK) for
`base_footprint -> camera`, then scores it against the target poses parsed out of
`warehouse.world`.

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
  The Gazebo ros2_control backend publishes the Robotiq mimic joints as
  `gripper_robotiq_85_*_joint_mimic`. move_group logs `Joint '..._mimic' not found in
  model` at ~50 Hz, and **crashes** (`terminate called after throwing moveit::Exception`)
  if a client echoes those names back inside a `RobotState`. Always build `RobotState`
  as an arm-only diff (`is_diff = true`, six `arm_*` names). Root cause is in
  `ros2_control.xacro`'s Gazebo backend, not in MoveIt.
- **The base is not braked**: extending the arm can roll the robot, and wheel odometry
  over-reports the motion because the wheels also slip. Validate base and end-effector
  world positions against gz ground truth
  (`ign topic -e -t /world/warehouse/pose/info -n 1`), never with
  `/diff_drive_controller/odom`. Classic's `gz model -m <name> -p` no longer exists.
  *Measured on Fortress the parked base no longer creeps* (0.0000 m peak-to-peak over
  10 s with the arm at the Phase 7 look pose), because gz_ros2_control drives joints
  through the physics engine rather than by kinematic teleport.
- **The base has no laser scanner until `urdf/lidar_2d.xacro` is included** (added in
  Phase 6): front puck, scan plane **0.30 m**, 220° FOV. The FOV cannot exceed ±110°
  at that mount point or the robot scans its own chassis corners (113.7°) and wheel
  tops (127°). Anything shorter than 0.30 m — euro pallets are 0.145 m — is invisible
  to SLAM, AMCL and both costmaps, so Nav2 will plan straight through a pallet.
- **Skid-steer odometry is only trustworthy after calibration, and the calibration is
  physics-engine specific.** The measured effective track was 0.70 m under Classic's ODE
  (multiplier 1.37) but is **0.5573 m under Fortress's DART**, so the value is now
  **1.0889**. Running DART with the ODE figure made `/odom` under-report yaw by 20.5 %,
  which rotated the SLAM map frame 17.7° off the world frame and broke the Phase 6 gate.
  Get this wrong and slam_toolbox diverges inside a single turn while reporting no
  errors. The vendor wheel `mu2` **0.15** patch is still load-bearing — DART does honour
  anisotropic friction, and with `mu2` back at 1.0 the body reaches only 0.31 of
  commanded yaw. Re-measure against gz ground truth after any change to wheel friction,
  mass, the physics engine, or the arm's stowed pose. **Calibrate with in-place spins**:
  rolling turns from the home pose drive 1.8 m forward into the barrier gate at x = 1.6,
  and a pinned robot reads as zero body rotation (this produced a nonsense 2.57 m
  "effective track" on the first attempt).
- **Rotating in place is the expensive manoeuvre**: a hard stiction deadband below
  ~0.27 rad/s (the base simply does not move, so any tapering P-controller stalls),
  and it walks the base sideways ~0.12 m per radian. Rolling turns track within 2 %.
- **`home_hold.py` fights any navigation stack** — it publishes zero `cmd_vel` at
  50 Hz. `pkill -f "[h]ome_hold"` before sending goals.
- **Stale processes from a previous session are the highest-value thing to check
  first.** Three orphaned `robot_state_publisher` nodes once kept serving an *old*
  `robot_description`, so `ros_gz_sim create` spawned a stale robot out of a correctly
  migrated workspace and the logs blamed a plugin that was no longer referenced. Run
  `ros2 node list` and look for duplicate `robot_state_publisher` entries before
  believing any plugin-load error. Kill by PID: `pkill -f` / `pgrep -f` patterns also
  match the shell running them — and note the pattern matches your *whole* command
  line, so even an `echo "home_hold"` elsewhere in the same command makes
  `pgrep -f "[h]ome_hold"` match and kill your own shell.
- **A `<pose>` authored on a `<sensor>` under `<gazebo reference="...">` is
  discarded.** (Still true under Fortress — same sdformat lumping.) Every `camera_*` frame reaches its parent through a fixed joint,
  so URDF→SDF reduction lumps them into `arm_wrist_3_link` and gzsdf substitutes
  the referenced frame's own transform for whatever pose you wrote. The only
  thing that aims a Gazebo camera is the orientation of the frame it references,
  which must be X-forward/Y-left/Z-up — *not* a REP-103 optical frame. Both
  D435i sensors therefore reference `camera_color_frame` while still stamping
  images `camera_color_optical_frame` — under Fortress that stamp comes from
  `<ignition_frame_id>`, not Classic's `<frame_name>`. Verify with
  `ign sdf -p /tmp/mm.urdf | grep -A12 "sensor name='camera_color'"`.
- **Sensors carry no ROS plugin at all under Fortress.** The world-level Sensors
  system renders them onto gz transport and `ros_gz_bridge` carries them across. Classic
  type names changed: `depth` → **`depth_camera`**, `ray` → **`gpu_lidar`** (whose
  parameters live under `<lidar>`, not `<ray>`, and which needs an explicit 1-sample
  `<vertical>` scan). Topic names come from `<topic>`, not `<camera_name>`/`<remapping>`.
  A camera publishes its `camera_info` as a sibling of the image topic, so
  `<topic>camera/color/image_raw</topic>` yields both `/camera/color/image_raw` and
  `/camera/color/camera_info`.
- **Run Gazebo with `gui:=false` for anything that reads the wrist cameras.**
  (Under Fortress this maps to the gz_args `-s` server-only flag.) The GUI
  renders the whole warehouse and starves the server's own sensor
  rendering: the two 640x480 wrist cameras fall from ~6 Hz to under 0.3 Hz. It
  only starts when `DISPLAY` is set, so this first bites on a phase that needs a
  display for something else (Phase 7 wants `cv2.imshow`).
- **The Ogre2 GUI viewport renders corrupted on this machine; the workaround is
  `gui_render_engine:=ogre`, and it is GUI-only on purpose.** Symptom: the whole
  gz-sim window (3D scene *and* Qt's own toolbar icons) squeezes into three
  quarters of its width with the rest black, everything textured broken into
  vertical RGB stripes. That signature — a 3/4 width squeeze plus per-pixel
  channel rotation — is a 24bpp buffer being consumed as 32bpp, i.e. a surface
  stride mismatch, not a shading bug.
  It is **not** software rendering and **not** a driver fault, so do not go
  installing drivers. Measured, same window and same capture path each time:
  Ogre2 on the Intel iGPU (`GL_RENDERER = Mesa Intel(R) UHD Graphics (CML GT2)`,
  Mesa 23.2.1) corrupts; Ogre2 forced onto the discrete GTX 1650 with
  `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` (driver 595.84)
  corrupts **byte-for-byte identically**; `QSG_RENDER_LOOP=basic` changes
  nothing; **Ogre1 renders perfectly**. Two unrelated GPU/driver stacks failing
  the same way rules the driver out. The remaining suspect is Ogre2's surface
  handling under **XWayland** — this is a GNOME *Wayland* session
  (`XDG_SESSION_TYPE=wayland`) and Qt runs on xcb, which it announces as
  "Ignoring XDG_SESSION_TYPE=wayland on Gnome". The real fix is logging into a
  native X11 session, which keeps Ogre2 and its PBR materials; `ogre` is a
  stopgap that loses PBR.
  `gui_render_engine` maps to `--render-engine-gui`, never `--render-engine`:
  the *server* must stay on Ogre2 or the wrist camera stops seeing the world's
  PBR materials, and Phase 7's detector is calibrated against that image.
  To reproduce or re-check, capture the window rather than trusting an
  impression — `xwd -id $(xwininfo -root -tree | grep -m1 ign-gazebo-gui |
  awk '{print $1}')`, then decode with the 4-byte stride the header reports
  (`bytes_per_line / width`, *not* `bits_per_pixel / 8` — getting that wrong
  produces the very artefact you are trying to diagnose).
- **Headless sensor rendering silently falls back to llvmpipe.** Separate from
  the GUI issue above, and it does affect the wrist cameras.
  `~/.ignition/rendering/ogre2.log` shows Ogre2's offscreen PBuffer path trying
  `/dev/dri/card2` (the NVIDIA GPU), logging
  `eglInitialize failed for device EGL_EXT_device_drm`, and settling on
  `EGL Device: EGL_MESA_device_software` — that is the `libEGL warning: egl:
  failed to create dri2 screen` pair you see on every headless launch. When
  `DISPLAY` is set, ign-rendering opens a hidden XWayland window instead and
  lands on the Intel iGPU, which is why the cameras manage 6-8 Hz rather than
  something worse. That log file is the only place the truth is written down:
  `glxinfo` is not installed, and `grep GL_RENDERER` on it answers "which GPU
  am I actually on" in one command.
- **Base creep under an extended arm was a Gazebo Classic problem and is gone.**
  Classic rolled the parked base backwards at ~1 cm/s and then lurched it ~0.4 m /
  0.65 rad after ~25 s, because the arm's un-PID'd `position` interface was held with
  kinematic `Joint::SetPosition` and the wheels could not absorb the reaction. On
  Fortress the same look pose measures 0.0000 m peak-to-peak over 20 samples / 10 s.
  `phase7_look_pose --hold` still station-keeps on `/odom` but is no longer required.
- **A COCO-pretrained YOLO cannot see hand-authored primitives.** `yolov8n.pt`
  scored 1–3 % on the world's original cube/cylinder/box while calling the
  tabletop a `bed` at 0.82. The pick targets are now 75 mm spheres, which read as
  `sports ball` and still fit the 2F-85's 85 mm stroke.
- **Ogre `<material><script>` does not exist in Fortress.** The world's own materials
  were converted to PBR `<ambient>/<diffuse>/<specular>`, but the models vendored from
  `~/.gazebo/models` (bookshelf, table, pallets…) still carry Ogre scripts and therefore
  render **black**. That darkened the scene enough to drop sphere detection from
  0.67–0.89 under Classic to **0.35, with only 1 of 3 balls found** — the Phase 7 gate
  still passes at 2.2 mm, but Phase 8's PERCEIVE state will want this fixed by giving
  those models PBR materials or raising the scene lighting.
- **`cv_bridge` is unusable in this workspace.** Its Humble boost extension is
  built against numpy 1.x while Ultralytics/torch require numpy 2.x; importing it
  yields `AttributeError: _ARRAY_API not found`. Decode `sensor_msgs/Image` with
  numpy directly. Ultralytics also imports matplotlib, so the apt
  `python3-matplotlib` must be shadowed by `pip install --user -U matplotlib`.
- **Never build a timeout from `get_clock()` under `use_sim_time`.** It reads 0
  until the first `/clock` message and then jumps to the sim's uptime, so
  `deadline = now() + wait` expires instantly. Use `time.monotonic()`. For the
  same reason, judge TF freshness by the stamp *advancing* rather than by its
  absolute age — a node busy with inference lags `/clock` by a variable amount.
- **`ros2 topic hz` can report nothing on a healthy topic here.** It showed no
  messages on `/camera/color/image_raw` while a plain rclpy subscriber measured
  6.3 Hz. Count messages yourself before declaring a sensor dead.
- URDF built via `Command([...xacro...])` in a launch file **must** be wrapped in
  `ParameterValue(..., value_type=str)`, else launch dies with "Unable to parse the value
  of parameter robot_description as yaml".
- `gazebo_warehouse.launch.py` runs `xacro` eagerly via `subprocess` and strips XML
  comments with a regex: the description gets round-tripped through node parameter
  overrides whose YAML lexer chokes on comment characters (": ", box-drawing glyphs).
  Keep that post-processing if you rewrite the launch file.
- The same launch sets `IGN_GAZEBO_RESOURCE_PATH` (and `GZ_SIM_RESOURCE_PATH`,
  since `gz_sim.launch.py` forwards both), including `~/.gazebo/models` and `src/` —
  URDF→SDF rewrites `package://` mesh URIs to `model://`, so every directory holding a
  model must be on that path. `ros_gz_sim`'s launch file *appends* to whatever is
  already in the environment, so setting these is additive rather than a clobber.
- It also sets `HUSKY_GAZEBO_PLUGINS=0`, an `optenv` guard added to the vendor
  `husky.urdf.xacro`, disabling the classic `gazebo_ros` diff_drive / joint_state / imu /
  gps plugins that would fight ros2_control over the wheel joints.

## Vendor dependencies

`ur_description`, `robotiq_description`, `realsense2_description`, MoveIt, Nav2,
ros2_control, ros_gz + gz_ros2_control come from apt (see `PLAN.md` §2 / `prompts/00_environment_preflight.sh`).
**`husky_description` is not available on Humble via apt** — it is cloned from
`github.com/akrbot/husky_description_ros2` into `src/husky_description` as a *nested git
repo* (a bare gitlink, not a configured submodule), and it carries two local patches
committed inside its own history: the `HUSKY_GAZEBO_PLUGINS` guard in
`urdf/husky.urdf.xacro`, and wheel lateral friction `mu2` 1.0 → 0.15 in
`urdf/wheel.urdf.xacro` (without which the skid-steer base cannot rotate in place).
Do not blow away or
re-clone that directory without re-applying the patch, and commit changes to it inside its
own repo.
