# Phase 7 — YOLOv8 Perception Node

## Task Prompt for Agent
```text
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

## Read this first: four things break before the node ever gets a chance

Every one of them is silent. The node starts, the window opens, and it detects
nothing — or detects the wrong room. Check these before debugging any Python.

### 1. There was no depth stream at all

Phases 1–6 only ever declared the colour camera. `gazebo.xacro` now also
declares a `<sensor type="depth">`, publishing `/camera/depth/image_raw`
(32FC1, metres) and `/camera/depth/camera_info`.

- Use **`libgazebo_ros_camera.so`**, not `libgazebo_ros_depth_camera.so` — the
  latter is not in the Humble `ros-humble-gazebo-plugins` binary and gzserver
  refuses to load it. `gazebo_plugins::GazeboRosCamera` derives from
  `gazebo::DepthCameraPlugin` and handles depth sensors fine.
- `<camera_name>camera</camera_name>` is what yields `/camera/depth/image_raw`;
  the plugin appends `depth/image_raw` itself. A `<remapping>` keyed on the bare
  name is silently dropped (same trap as Phase 4's screenshot camera).
- The depth sensor is mounted on the **colour** frame with the same FOV and
  640x480 resolution, so depth pixel (u,v) *is* colour pixel (u,v) — the sim
  equivalent of the real D435i's `aligned_depth_to_color` stream. One CameraInfo
  back-projects both, and the recovered point is already in
  `camera_color_optical_frame`.

### 2. The wrist camera was aimed 90° off its own lens axis

A Gazebo camera looks down its **+X** with +Y left and +Z up. A REP-103 optical
frame is +X right, +Y down, +Z forward. Referencing a sensor to an optical frame
therefore photographs whatever is *beside* the robot.

The obvious fix — an authored `<pose>` on the `<sensor>` that rotates it back —
**does not work**. Every `camera_*` frame reaches its parent through a fixed
joint, so URDF→SDF reduction lumps them all into `arm_wrist_3_link` and gzsdf
then *replaces* the sensor's `<pose>` with the referenced frame's transform.
Any `<pose>` you write there is dead code. Confirm with:

```bash
xacro src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro \
  use_gazebo:=true sensor_arch:=0 controllers_yaml:=/tmp/x.yaml > /tmp/mm.urdf
gz sdf -p /tmp/mm.urdf | grep -B2 -A12 "sensor name='camera_color'"
```

The only thing that aims the camera is the referenced frame's own orientation,
so both sensors hang off **`camera_color_frame`** (the colour module's body
frame: same origin, already X-forward) while keeping
`<frame_name>camera_color_optical_frame</frame_name>` for the published headers.

### 3. A COCO model cannot see coloured cubes

The world's original targets were a red cube, a blue cylinder and a green box,
chosen "with distinct colors so YOLO can tell them apart". That premise is
wrong: stock `yolov8n.pt` classifies by learned COCO category, not colour, and
COCO has no cube class. Measured from the wrist camera at 0.5–1.0 m:

| object | best class | confidence |
|---|---|---|
| red 80 mm cube | `book` | 0.012 |
| blue cylinder | `cup` | 0.025 |
| green box | `cup` | 0.022 |
| *the tabletop itself* | `bed` | **0.818** |

The targets are now **three 75 mm spheres** (`target_ball_red/blue/green` in
`warehouse.world`), which read as COCO **`sports ball`** at 0.67–0.89 from the
same viewpoint. 75 mm also stays inside the Robotiq 2F-85's 85 mm stroke, so
Phase 8 can still grasp them. Vendor meshes were measured too and rejected:
`beer` is a solid 110 mm cylinder and `bowl` is 196 mm — both too wide to grasp.

### 4. gzclient starves the sensor renderer

Running the Gazebo GUI alongside the two 640x480 wrist cameras drops them from
~6 Hz to **under 0.3 Hz** on an 8-core box, and the detector then sees almost
nothing. `gazebo_warehouse.launch.py` gained a `gui` argument for this; Phase 7
runs headless. Note gzclient only actually starts when `DISPLAY` is set — which
it must be, because the phase requires `cv2.imshow`.

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 0. Preamble (every tab)
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
source install/setup.bash          # install/setup.zsh under zsh
export ROS_LOCALHOST_ONLY=1
export DISPLAY=:0                  # cv2.imshow needs it
```

### 1. Build Perception Package
```bash
colcon build --symlink-install --packages-select mobile_manipulator_perception
source install/setup.bash
```

### 2. Gazebo, headless, robot parked at the pick table
```bash
ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py \
    home_x:=3.1 gui:=false
```
`home_x:=3.1` is as close as the base gets: the lidar puck protrudes to
x = +0.575 and the workbench's near edge is at x = 3.75.

### 3. Run Perception Node — *before* the arm moves
```bash
ros2 run mobile_manipulator_perception yolo_perception_node \
    --ros-args -p use_sim_time:=true
```
Loading the weights takes ~15 s. Start it first so the gate is not racing the
base drift (see gotchas). Wait for the `intrinsics fx=381.347 ...` log line.

### 4. Aim the wrist camera at the bench and pin the base
```bash
pkill -f "[h]ome_hold"             # it pins cmd_vel to zero at 50 Hz and wins
ros2 run mobile_manipulator_perception phase7_look_pose --hold 120 \
    --ros-args -p use_sim_time:=true
```

### 5. Verify Target Object TF Transform (in new terminal tab)
```bash
ros2 run mobile_manipulator_perception phase7_target_check \
    --duration 10 --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_target_frame
```

### 6. Grab the annotated frame headlessly (optional evidence)
```bash
ros2 run mobile_manipulator_gazebo capture_screenshot.py \
    --topic /yolo_perception_node/annotated_image --out /tmp/phase7_annotated.png
```

**Pass Criteria**: OpenCV window renders live bounding box on color image feed;
`tf2_echo` prints stable 3D coordinates matching the physical target location on
the pick table.

**Achieved**: 20/20 fresh samples over 10 s, transform peak-to-peak **0.7 mm**,
world-frame error **2.9 mm** vs `target_ball_green`'s spawn pose. The window
shows `sports ball 0.89` and `sports ball 0.67` on two balls, plus a harmless
`bed 0.56` on the tabletop, with the locked target boxed in red.

---

## Gotchas

- **The parked base will not stay parked.** With the arm extended it rolls
  backwards at ~1 cm/s and then, after ~25 s, lurches ~0.4 m sideways and yaws
  ~0.65 rad — after which the bench is out of frame. It is genuinely *rolling*
  (`/odom` agrees with `gz model -p` to 1 mm), because the arm's `position`
  command interface has no PID, so gazebo_ros2_control holds the joints with a
  kinematic `gazebo::physics::Joint::SetPosition` each cycle instead of a
  torque, and the wheels — held the same kinematic way — cannot absorb the
  reaction. `phase7_look_pose --hold` closes a P loop on `/odom` to pin the base
  for the length of the gate. Phase 8 must fix the cause; a grasp cannot be
  planned against a base that walks away.
- **Do not raise `controller_manager.update_rate` to match physics.** It was
  tried (100 → 500 Hz to match the world's 500 Hz `max_step_size`) and it made
  things worse: the wrist cameras fell below 0.3 Hz and the base still crept.
- **`cv_bridge` is unusable here.** Its Humble boost extension is compiled
  against numpy 1.x while Ultralytics/torch pull numpy 2.x; importing it prints
  `AttributeError: _ARRAY_API not found`. The node decodes `sensor_msgs/Image`
  with numpy directly instead.
- **Ultralytics imports matplotlib**, and the apt `python3-matplotlib` is also
  numpy-1.x-only. `pip install --user --upgrade matplotlib` (3.10.x) fixes the
  `numpy.core.multiarray failed to import` traceback at `from ultralytics import
  YOLO`.
- **`use_sim_time:=true` is mandatory** for every node in this phase, or TF
  stamps are wall time while everything else is `/clock` and `tf2_echo` reports
  extrapolation errors. The node warns if you forget.
- **Never compute a timeout from `get_clock()` under sim time.** It reads 0
  until the first `/clock` message and then jumps to however long the sim has
  been up, so `deadline = now() + wait` expires instantly. `phase7_target_check`
  uses `time.monotonic()` for all wall-clock waiting.
- **Do not judge TF freshness by absolute age either.** The perception node runs
  inference on a single-threaded executor, so its `/clock` callback lags and its
  stamps trail real sim time by a variable amount. `phase7_target_check` treats
  a stamp that has not *advanced* since the previous sample as stale, which is
  clock-independent.
- **`ApproximateTimeSynchronizer` slop is 0.1 s, not the usual 0.05.** The two
  Gazebo sensors render in separate passes; measured colour/depth stamp offsets
  are 0 most of the time but reach a full frame period (0.14 s). 0.05 pairs 83 %
  of frames, 0.1 pairs 97 %.
- **The node locks onto one target.** Three interchangeable balls make
  "highest confidence" flip frame to frame, which teleports
  `object_target_frame` between objects several times a second and reads as
  ~0.5 m of jitter. `track_radius` (default 0.15 m) keeps the locked object
  while it stays in view; set it to 0.0 for pure per-frame highest-confidence.
- **Depth measures the front surface, not the centroid.** `target_radius_m`
  (default 0.0375 = the ball radius) is added along the view ray so the
  broadcast point is the object centre, which is what a grasp planner wants.
- **Judge stability in the world frame, not the camera frame.** The camera rides
  a base that creeps, so `tf2_echo`'s numbers legitimately drift; the ball's
  world position is the invariant, and that is what `phase7_target_check` gates
  on.
- **`ros2 topic hz` on the camera topics is unreliable here** — it reported
  nothing while a plain rclpy subscriber measured a healthy 6.3 Hz. Count
  messages yourself before concluding a sensor is dead.
- The weights ship in the package (`models/yolov8n.pt`, installed to
  `share/mobile_manipulator_perception/models/`) so `ros2 run` never depends on
  Ultralytics reaching the network mid-demo. `model_path` overrides it.
- **ament_python needs `setup.cfg`.** Without the `[develop] script_dir` /
  `[install] install_scripts` stanza, colcon installs console scripts to
  `install/<pkg>/bin` where `ros2 run` cannot see them — the symptom is a bare
  `No executable found`.
