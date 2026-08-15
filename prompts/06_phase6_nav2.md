# Phase 6 — Nav2 Navigation Stack

## Task Prompt for Agent
```text
In mobile_manipulator_navigation: (1) run slam_toolbox once against the warehouse.world
from Phase 4 to generate and save an occupancy map (warehouse.yaml +
warehouse.pgm). (2) Write nav2_params.yaml configured for AMCL localization
against that saved map, with costmaps (global + local) tuned with an
inflation radius appropriate for the mobile base footprint plus a safety
margin, and controller_server/planner_server using DWB/NavFn.
(3) Write nav2_bringup.launch.py starting AMCL + Nav2 stack against the saved map.
Verify by publishing a single hardcoded PoseStamped goal at the pick-table location
and confirming via `ros2 topic echo /plan` that a global plan is produced and the
robot drives there without colliding with any rack, pallet, or barrier model.
Report the final pose error.
```

---

## Read this first: the robot had no laser scanner

Phases 1–5 built a robot with a wrist RGB camera and nothing else. SLAM, AMCL and
the Nav2 costmaps all need `sensor_msgs/LaserScan` on `/scan`, so Phase 6 starts by
adding one:

- `mobile_manipulator_description/urdf/lidar_2d.xacro` — a front-bumper puck,
  `lidar_link`, fixed joint on `base_link` at `xyz="0.52 0 0.1677"`, i.e. a scan
  plane **0.30 m above the ground**.
- `gazebo.xacro` gains the matching `libgazebo_ros_ray_sensor.so` sensor (220° FOV,
  440 samples, 0.12–25 m, 10 Hz) — emitted only under `use_gazebo:=true`.

Two consequences that shape everything else in this phase:

- **±110° FOV is not arbitrary.** Pushed 0.52 m ahead of `base_link`, the chassis
  corners sit at 113.7° and the wheel tops at 127°. Widen the FOV past ±110° and the
  robot scans itself; move the puck back and you must narrow it.
- **Anything shorter than 0.30 m is invisible.** Euro pallets are 0.145 m tall, so
  they are absent from the map *and* from both costmaps — Nav2 will happily plan
  straight through one. Keep every route ≥ 1 m from a pallet, or raise them. Tables
  appear as four 0.02 m leg dots, not as a solid slab.

A URDF change means the MoveIt collision matrix is stale:
`src/mobile_manipulator_moveit_config/scripts/regenerate_collision_matrix.sh`
(11 new `lidar_link` pairs, 173 disabled total). That script needed two fixes before
it would run at all — see the gotchas below.

---

## Terminal Commands

### 1. Build
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  mobile_manipulator_description mobile_manipulator_navigation
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

### 2. Map the warehouse (once)
```bash
# Gazebo first; then free the base — home_hold pins cmd_vel to zero at 50 Hz
setsid nohup ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py \
  > /tmp/gazebo.log 2>&1 < /dev/null &
# wait for "Arm stow finished", then:
pkill -9 -f "[g]zclient"      # RTF 0.7 -> 0.96, and ~1 GB of RAM back
pkill -f "[h]ome_hold"

setsid nohup ros2 launch mobile_manipulator_navigation slam.launch.py \
  > /tmp/slam.log 2>&1 < /dev/null &
sleep 20
ros2 run mobile_manipulator_navigation mapping_drive.py --ros-args -p use_sim_time:=true

ros2 run nav2_map_server map_saver_cli \
  -f ~/mobile_manipulator_ws/src/mobile_manipulator_navigation/maps/warehouse \
  --ros-args -p use_sim_time:=true
```

### 3. Navigate (the gate)
```bash
# restart Gazebo so the robot is back at home: AMCL's set_initial_pose is (0,0,0)
# and the map frame is anchored where mapping started, i.e. map == world
pkill -f "[h]ome_hold"
ros2 launch mobile_manipulator_navigation nav2_bringup.launch.py
# wait for "Managed nodes are active", then:
ros2 run mobile_manipulator_navigation phase6_nav_goal.py --goal 2.9 0.0 0.0
ros2 topic echo /plan --once        # the gate's stated check, in another shell
```

---

## Gotchas

**The whole phase hinges on odometry calibration.** The first two mapping attempts
failed the same way: the robot drove confidently in the wrong direction, and
slam_toolbox's estimate ended up 1.7 m and 100° from ground truth. The cause was not
SLAM — it was `/odom`. Measured against `gz model -m mobile_manipulator -p`, wheel
odometry under-reported yaw by 27 %, which is far more than a scan matcher's ±20°
coarse search window can absorb once it accumulates over a turn.

Calibrate by teleporting to open floor, commanding a known twist, and comparing:

| mode | commanded | GT yaw | odom yaw | odom/GT |
|---|---|---|---|---|
| in-place | 0.40 rad/s | 0.393 | 0.287 | 0.73 |
| in-place | 0.60 rad/s | 0.664 | 0.472 | 0.71 |
| rolling 0.30 m/s | 0.30 rad/s | 0.262 | 0.199 | 0.76 |

A measured wheel differential of 0.275 m/s producing 0.393 rad/s of body yaw means
the **effective track is 0.70 m = 1.37 × geometric** — so
`wheel_separation_multiplier: 1.37`, not Clearpath's 1.875 (which describes a real
Husky on loose ground). After the change: odom/GT is 0.94–0.96 in place and 1.14
rolling, and SLAM closes the ~40 m mapping loop **5.8 cm / 0.03 rad** from truth.
Do this measurement before blaming SLAM or AMCL for anything.

**In-place rotation has a stiction deadband and it is expensive.** Below roughly
0.27 rad/s commanded the wheels cannot break lateral grip and the base does not move
at all — a P-controller that tapers its output simply stalls (this is exactly how the
first mapping run died: it stalled mid-turn at 0.216 rad/s while still believing it
was turning). Rotating also *walks the base sideways ~0.12 m per radian*, which
odometry cannot see. Rolling turns have neither problem (0.30/0.30 commanded →
0.285 m/s, 0.294 rad/s measured), so `mapping_drive.py` only turns in place when the
heading error exceeds 0.9 rad, applies a 0.45 rad/s floor when it does, and finishes
every turn rolling. The opening 360° spin was deleted outright: it drifted the base
0.83 m, and home is only 1.28 m from a barrier corner.

**`home_hold` fights Nav2.** It publishes a zero Twist at 50 Hz on
`/diff_drive_controller/cmd_vel_unstamped` to keep the base parked after spawn. Kill
it before sending any goal or the robot will not move and nothing will say why.

**A stale `gzserver` silently kills the next run.** It keeps port 11345, and the new
one dies with `Unable to start server[bind: Address already in use]` — buried in a
verbose log. Worse, `pkill -f "gzserver"` style patterns *match the tool call's own
shell*, so a cleanup line can kill the very command that contains it. Kill by PID:
`for pid in $(pgrep -f "gzserver|gzclient|async_slam_toolbox"); do kill -9 $pid; done`.

**`regenerate_collision_matrix.sh` was broken** (it had not been re-run since Phase 5
was authored): `set -u` came *before* `source /opt/ros/humble/setup.bash`, which reads
`AMENT_TRACE_SETUP_FILES` unguarded, and the script never sourced the workspace
overlay, so `collisions_updater` aborted with
`PackageNotFoundError: package 'mobile_manipulator_moveit_config' not found`
(SIGABRT, exit 134). Both are fixed in-place.

**DWB needs `ObstacleFootprint`, not the default `BaseObstacle`.** The barrier gate on
the way to the workbench is 0.934 m wide for a 0.70 m robot. `BaseObstacle` scores the
single cell under the robot's centre, which sits in inscribed-cost territory inside
the gate, and it vetoes every trajectory through it. `ObstacleFootprint` checks the
real polygon against lethal cells only, and the robot threads the gate with ~0.11 m
either side.

**`map_saver_cli` needs `-p use_sim_time:=true`**, otherwise it waits on a `/map` whose
timestamps it will never accept.

---

## As Built

```
mobile_manipulator_navigation/
├── config/
│   ├── nav2_params.yaml            # AMCL + costmaps + DWB + NavFn + behaviors + BT
│   └── slam_toolbox_mapping.yaml   # one-shot mapping session
├── launch/
│   ├── nav2_bringup.launch.py      # map_server, amcl, controller, planner,
│   │                               # behavior, bt_navigator + lifecycle manager
│   └── slam.launch.py              # mapping only
├── maps/
│   ├── warehouse.pgm               # 726 x 569 @ 0.05 m, origin (-18.1, -14.2)
│   └── warehouse.yaml
└── scripts/
    ├── mapping_drive.py            # scripted mapping route (map-frame waypoints)
    └── phase6_nav_goal.py          # the gate: goal -> plan -> pose error -> clearance
mobile_manipulator_description/
└── urdf/lidar_2d.xacro             # + ray sensor block in gazebo.xacro
```

**Localization: AMCL, not live SLAM** (as PLAN.md §8 recommends). `slam.launch.py` is
run once to produce the map and is not part of the runtime stack. AMCL uses
`set_initial_pose: (0,0,0)` — valid because slam_toolbox anchors the map frame at the
robot's pose when mapping starts, so *map ≡ world* as long as mapping begins at the
spawn pose. Restart Gazebo before a nav run and no `/initialpose` is ever needed.

**Costmap tuning.** Footprint `[[0.58,0.35],[0.58,-0.35],[-0.51,-0.35],[-0.51,0.35]]`
— chassis plus the lidar puck sticking out to x = +0.575. Inscribed radius 0.35 m,
circumscribed ≈ 0.68 m, so `inflation_radius: 0.75` is circumscribed + a small margin,
with `cost_scaling_factor: 3.0`. That combination deliberately leaves the gate
traversable: its centreline is 0.467 m from each barrier, i.e. 252·exp(−3.0·0.117) ≈
177 — expensive but well under the 253 that NavFn treats as an obstacle.

**Velocity limits** are set below what the base can actually deliver
(`max_vel_x: 0.4`, `max_vel_theta: 0.8`) because commanded yaw is only ~70 % realised
after the multiplier change; Nav2 is closed-loop and absorbs the shortfall.

---

## Gate Results

```
global plan         : 115 poses, 2.91 m long, frame "map" (9 replans)
start pose (gazebo) : x=-0.003 y=-0.000 yaw=+0.000
goal                : x=+2.900 y=+0.000 yaw=+0.000
final pose (gazebo) : x=+2.655 y=+0.004 yaw=+0.001
final pose (amcl)   : x=+2.664 y=-0.007 yaw=+0.018   [localization error 0.014 m]
FINAL POSE ERROR    : 0.245 m, 0.001 rad     (nav2 default tolerance 0.25 / 0.25: YES)
min clearance       : 0.111 m to "barrier_lane_north"      collision-free: YES
```

The 0.245 m is the goal checker doing its job, not drift: AMCL is within 1.4 cm of
ground truth, and `SimpleGoalChecker` (stateful) declares success the moment the
estimate enters the 0.25 m ball, so DWB stops there. Two consecutive runs reproduced
the result to within 3 mm.

`phase6_nav_goal.py` parses obstacle footprints straight out of `warehouse.world`
(54 of them) and checks the robot polygon against every one along the driven trace,
so the collision claim is measured rather than eyeballed.

**Open item for Phase 8:** the robot cannot get closer than x ≈ 3.24 (its front edge
against the workbench legs at x = 3.82), which leaves the nearest target object ~1.0 m
from the arm base — beyond the UR5's 0.85 m reach. The pick pose will need the base
approaching from a corner, or the targets moved toward the table edge.
