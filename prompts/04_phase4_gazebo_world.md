# Phase 4 — Gazebo Warehouse World

**Status: ✅ complete** (see `PLAN.md` §14 checklist)

## Task Prompt for Agent
```text
In mobile_manipulator_gazebo, create warehouse.world: a Gazebo Classic world with a
warehouse floor plan containing 2+ storage-rack aisles, wooden pallets, and barrier
obstacles, composed from existing Gazebo model-database models (do not hand-author
mesh geometry). Include a pick-up workbench with 2-3 spawned target objects
(cube, cylinder, box primitives with distinct colors for YOLO to
distinguish) and a separate drop-off table, plus directional and point
lights tuned so the RGB camera feed is not blown out or too dark.
Add gazebo_warehouse.launch.py that starts Gazebo with this world, spawns
the mobile_manipulator robot (from Phase 2/3) at a named "home" pose using the
ros2_control Gazebo plugin (gazebo_ros2_control) instead of mock hardware.
Verify with `gz stats` showing real-time factor > 0.7 and confirm via a
screenshot that the robot is standing stably (not falling through the
floor or jittering) 30 seconds after spawn with zero commanded velocity.
```

---

## Known Gotchas (learned the hard way — do NOT skip)

1. **There is no `ros2_control_node` in this phase.** `gazebo_ros2_control` starts the
   controller manager *inside gzserver*. Launching a separate `ros2_control_node` as well
   gives you two controller managers fighting over the same joints. The spawners are
   chained with `OnProcessExit` (spawn_entity → jsb → diff_drive → arm → gripper) so they
   cannot race the plugin's parameter load.

2. **`GAZEBO_MODEL_PATH` must include the workspace `src/`.** The URDF→SDF converter
   rewrites `package://husky_description/meshes/...` into `model://husky_description/...`.
   If gzserver cannot resolve those, it falls back to the (long dead) online model
   database and **hangs the world-update loop**, which also stops `gazebo_ros2_control`
   from ever loading — the symptom looks like "controllers never spawn", not "mesh
   missing". The launch file also blanks `GAZEBO_MODEL_DATABASE_URI` so it fails fast
   instead of hanging.

3. **Strip XML comments out of the generated URDF.** `gazebo_ros2_control` 0.4.x re-injects
   `robot_description` into its own node as an rcl `--param name:=<xml>` override, and
   rcl's YAML lexer chokes on characters that appear in our comments (`: `, box-drawing
   glyphs). `gazebo_warehouse.launch.py` therefore runs `xacro` eagerly via `subprocess`
   and `re.sub(r'<!--.*?-->', '', ...)` before setting the parameter. Keep that step if you
   rewrite the launch file.

4. **`HUSKY_GAZEBO_PLUGINS=0` must be exported before the xacro runs.** The vendor
   `husky.urdf.xacro` carries a local `$(optenv HUSKY_GAZEBO_PLUGINS 1)` guard around its
   classic `gazebo_ros` plugins (diff_drive, joint_state_publisher, imu, gps). Left
   enabled, `libgazebo_ros_diff_drive.so` grabs the wheel joints and ros2_control gets
   nothing. That guard is an **uncommitted local patch** in `src/husky_description` — do
   not re-clone that repo without re-applying it.

5. **`libgazebo_ros_depth_camera.so` is not shipped** by the `ros-humble-gazebo-plugins`
   binary; declaring a depth sensor makes gzserver refuse to load the model. Only the RGB
   camera (`libgazebo_ros_camera.so`) is declared. Phase 7 needs a depth stream — plan on
   building that plugin from source or switching the sensor type there, not here.

6. **Camera topic names come from `<camera_name>`, not from `<ros><remapping>`.**
   `gazebo_ros_camera` derives `"<camera_name>/image_raw"` itself; a remapping keyed on the
   bare name `image_raw` never matches and is silently dropped. Hence the world-fixed
   screenshot camera really publishes `/phase4_camera/image_raw`.

7. **`<horizontal_fov>`, `<image>` and `<clip>` must sit inside `<camera>`.** As direct
   `<sensor>` children they are silently dropped and the sensor falls back to defaults.

8. **The wheels free-roll until diff_drive sees its first command**, so the spawn impulse
   makes the robot drift off the home pose. `home_hold.py` publishes a constant zero
   `Twist` at 50 Hz (locking the wheels) and sends the arm to its stowed pose; it is
   chained to run after the last spawner.

9. **Same CLI hygiene as Phase 3**: `export ROS_LOCALHOST_ONLY=1` for the launch *and*
   every CLI call, kill stale `ros2cli.daemon` processes first, and launch detached with
   `setsid nohup`.

10. **The optical-frame flip lives in the sensor pose.** `gazebo_ros` camera plugins publish
    the rendered image unflipped, so the D435i sensor carries `<pose>0 0 0 0 0 3.14159</pose>`
    to match the REP-103 ROS optical convention. Do not "fix" it in the consumer node.

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 0. Preflight (once per terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/mobile_manipulator_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
for p in $(ps aux | grep "[r]os2cli.daemon" | awk '{print $2}'); do kill -9 $p; done
```

### 1. Build Gazebo Package
```bash
cd ~/mobile_manipulator_ws
colcon build --symlink-install --packages-select mobile_manipulator_gazebo
source install/setup.bash
```

### 2. Launch Gazebo Warehouse Simulation (detached)
```bash
cd ~/mobile_manipulator_ws
setsid nohup ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py \
  > /tmp/gazebo_warehouse.log 2>&1 < /dev/null &
```
Home pose is overridable: `home_x:=`, `home_y:=`, `home_z:=`, `home_yaw:=` (default 0 0 0 0).

### 3. Verify Gazebo Real-Time Factor (in new terminal tab)
```bash
gz stats
```

### 4. Verify Spawned Controllers
```bash
ros2 control list_controllers
```

### 5. Headless Screenshot Evidence (30 s after spawn)
```bash
ros2 run mobile_manipulator_gazebo capture_screenshot.py \
  --topic /phase4_camera/image_raw --out /tmp/phase4_home_pose.png --settle 2
```
The world-fixed `phase4_camera` model means gzclient never has to run. Reposition it live
with `gz model -m phase4_camera -x <X> -y <Y> -z <Z> -R 0 -P <pitch> -Y <yaw>`.

### 6. Confirm the Robot Has Not Drifted
```bash
gz model -m mobile_manipulator -i | head -20      # pose should still be ~home
ros2 topic echo /camera/color/image_raw --field header.frame_id --once
```

**Pass Criteria**: `gz stats` real-time factor > 0.7; all 4 controllers `active` against the
in-gzserver controller manager; the robot sits upright at the home pose 30 s after spawn
(no jitter, no clipping through the slab) as shown by `/tmp/phase4_home_pose.png`; the
wrist camera publishes on `/camera/color/image_raw` with `frame_id
camera_color_optical_frame`.

---

## As Built

```
mobile_manipulator_gazebo/
├── worlds/warehouse.world
├── launch/gazebo_warehouse.launch.py
└── scripts/
    ├── home_hold.py            # zero cmd_vel @50 Hz + arm stow (--no-stow to skip)
    └── capture_screenshot.py   # one frame off a camera topic → PNG
mobile_manipulator_description/
└── urdf/gazebo.xacro           # gazebo_ros2_control plugin + D435i RGB sensor (use_gazebo:=true only)
```

`warehouse.world` — 36 × 28 m slab with perimeter walls, all furniture pulled from
`~/.gazebo/models` via `model://`:

- 12 `bookshelf` racks in two aisles (x ≈ −11/−8 and x ≈ 8/11)
- 8 `euro_pallet` models at aisle ends and side lanes
- 4 `jersey_barrier` obstacles flanking the central drive lane
- `pick_workbench` (`model://table`) at (4.5, 0) and `drop_off_table` at (4.5, 3.2)
- target objects on the workbench: `target_ball_red`, `target_ball_blue`,
  `target_ball_green` — three 75 mm spheres. **Changed in Phase 7**: these were
  originally a cube, a cylinder and a box "in distinct colors for YOLO", which
  does not work — stock COCO `yolov8n.pt` classifies by learned category, not
  colour, and peaked at 1–3 % confidence on them. A plain sphere reads as COCO
  `sports ball` at 0.67–0.89, and 75 mm still fits the 2F-85's 85 mm stroke.
- 2 directional lights (key + fill) + 2 point lights over the tables
- a visual-only home-pose marker at (0, 0) and the static `phase4_camera` screenshot camera

The robot URDF is generated with `use_gazebo:=true`, which swaps all three `<ros2_control>`
blocks to `gazebo_ros2_control/GazeboSystem` and emits `gazebo.xacro`. The controllers YAML
path is passed in as the `controllers_yaml` xacro arg as an **absolute path** — gzserver
does not resolve `$(find ...)` inside plugin SDF elements.

## Rework (2026-08-14) — denser world + skid-steer physics fix

The world was reworked mid-project (before Phase 6 mapping) to be busier and
harder while keeping every pose the later phases depend on — slab, walls,
barrier gate at x = 1.6, both tables, target objects, home marker, and
`phase4_camera` are all unchanged:

- rack rows densified to **5 bookshelves per row** (added y = ±3 bays)
- cargo pallets + static `cardboard_box` stock inside the rack aisles
- `construction_barrel` ×4, `cabinet` ×2, `brick_box_3x1x3` ×2,
  `construction_cone` ×3 as room-shaping clutter and **asymmetric AMCL
  landmarks** (north/south halves no longer look alike to a lidar)
- placement rule: everything keeps ≥ 1.0 m clearance from the Phase 6 demo
  route (home → gate → workbench) and the mapping loop (x = −3, y = 6.5,
  x = 6.2 lines). Anything shorter than the 0.30 m lidar scan plane
  (pallets 0.145 m, cardboard 0.30 m, cones) sits only where the robot
  never drives — the 2D lidar cannot see it, so it must never flank a route.
- keep clutter off the `phase4_camera` sight line ((−2.4,−2.4) → origin):
  the first rework attempt put a barrel at (−1.2,−1.9) and it filled a
  third of the screenshot frame.

Physics fix shipped with the rework (found while testing Phase 6 mapping —
commanded in-place spins produced ~0.005 rad/s, the base simply could not
rotate):

- `husky_description/urdf/wheel.urdf.xacro` (nested vendor repo, local
  patch): wheel `mu2` 1.0 → **0.15** — isotropic friction let the lateral
  force of four fixed wheels cancel the available yaw torque.
- `mobile_manipulator_controllers.yaml`: **`wheel_separation_multiplier:
  1.37`**, the *measured* effective track (0.70 m) of this base on this
  slab, not Clearpath's 1.875 (which is for the real Husky on loose ground).

  Both numbers were first set to mu2 = 0.5 / multiplier = 1.875 here in
  Phase 4, which fixed gross rotation but left odometry under-reporting yaw
  by 27 % — enough to make the Phase 6 SLAM scan matcher diverge inside one
  turn. Phase 6 re-derived them against Gazebo ground truth; see
  `prompts/06_phase6_nav2.md` for the calibration procedure and numbers.

Gate re-verification after the rework: robot pose bit-identical over 30 s
at spawn, RTF **0.96 headless / ≈ 0.7 with gzclient** (kill gzclient before
measuring: `pkill -9 -f "[g]zclient"`), screenshot via `phase4_camera`
shows the stowed robot on the home marker with the gate wall and workbench
targets in frame.
