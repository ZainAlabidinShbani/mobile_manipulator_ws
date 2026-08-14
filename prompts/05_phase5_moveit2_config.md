# Phase 5 — MoveIt 2 Configuration

**Status: ✅ complete** (see `PLAN.md` §14 checklist)

## Task Prompt for Agent
```text
Using the MoveIt Setup Assistant against the URDF/Xacro in
mobile_manipulator_description (from the completed, verified Phase 2),
generate mobile_manipulator_moveit_config with: a planning group "ur5_arm"
covering the 6 UR5 joints, a planning group "gripper" for the Robotiq
2F-85, self-collision matrix from the assistant's default sampling, and
moveit_controllers.yaml wired to the joint_trajectory_controller and
gripper_action_controller from Phase 3/mobile_manipulator_controllers.yaml
(match controller names exactly). No "husky" anywhere in generated names.
Verify by launching move_group.launch.py + demo.launch.py, planning a
motion in RViz's MotionPlanning panel to a reachable pose above the pick
table, confirming a collision-free green preview, and executing it against
the running Gazebo simulation from Phase 4 — report whether the physical
Gazebo arm moved to match the RViz plan.
```

---

## Known Gotchas (learned the hard way — do NOT skip)

1. **The Setup Assistant GUI is not required, and not usable headlessly — but its sampler
   is.** `moveit_setup_assistant` is a Qt app. The self-collision step has a standalone
   headless entry point, `/opt/ros/humble/lib/moveit_setup_assistant/collisions_updater`,
   which runs the *same* algorithm. `scripts/regenerate_collision_matrix.sh` calls it at
   the assistant's own defaults (`--trials 10000 --min-collision-fraction 0.95 --default
   --always`). Everything else in the SRDF is hand-authored in
   `config/mobile_manipulator.srdf.base`; the updater merges `<disable_collisions>` into
   `config/mobile_manipulator.srdf`, so a regeneration can never clobber the semantics.

2. **`collisions_updater`'s "Total possible collisions: 1485" is a red herring.** That is
   55·54/2 over *all* links. Only 28 links carry collision geometry → 378 real candidate
   pairs, of which 162 end up disabled (28 Adjacent, 6 Default, 128 Never). The line that
   tells you the true pool size is `Thread complete 378`.

3. **The URDF in `config/` is a wrapper, not a copy.** `config/mobile_manipulator.urdf.xacro`
   just `xacro:include`s `mobile_manipulator_description`. Verify after any Phase 2 edit
   that both still produce identical XML (compare after stripping comments) — a copied
   URDF silently drifts and produces plans that are valid for a robot you no longer have.

4. **`use_gazebo:=false` is correct for move_group.** move_group only needs kinematics and
   collision geometry; which `<ros2_control>` hardware plugin the URDF names is irrelevant
   to it, and the false branch keeps Gazebo plugin tags out of the parsed model. Link and
   joint geometry are identical under both branches.

5. **The virtual joint must be `planar`, not `fixed`.** The base is mobile. A *fixed*
   `odom → base_footprint` joint pins the robot at the odom origin, so every plan is
   computed for a robot that is no longer where it thinks it is once the base drives. With
   `planar`, MoveIt reads the base pose from the `odom → base_footprint` TF that
   `diff_drive_controller` publishes. Consequence: **`diff_drive_controller` must be
   spawned even in the mock demo**, purely so that TF exists — `demo.launch.py` does this.

6. **Do not include the Robotiq mimic joints in the `gripper` group.** Only
   `gripper_robotiq_85_left_knuckle_joint` is actuated; the other five are URDF `<mimic>`
   followers and MoveIt derives them itself. Listing them makes them planning variables and
   the group immediately over-constrained.

7. **`action_ns: gripper_cmd`, not `gripper_command`.** The Humble
   `GripperActionController` serves `/gripper_action_controller/gripper_cmd`. The
   `gripper_command` spelling only exists on Rolling/Jazzy, and getting it wrong fails
   silently — move_group just never executes gripper goals.

8. **`trajectory_execution.allowed_start_tolerance: 0.0`** (i.e. check disabled). Gazebo's
   PID-tracked arm settles a few mrad off the last commanded point, which otherwise aborts
   roughly every second execution with *"Invalid Trajectory: start point deviates from
   current robot state"*.

9. **`gazebo_ros2_control` publishes the mimic joints under names that do not exist in the
   URDF.** `libgazebo_hardware_plugins.so` appends `_mimic`, so `/joint_states` carries
   `gripper_robotiq_85_left_knuckle_joint_mimic` and four siblings. Two consequences:
   move_group logs `Joint '..._mimic' not found in model` at the joint-state rate (~50 Hz),
   and — the dangerous one — **move_group aborts outright** with
   `terminate called after throwing an instance of 'moveit::Exception' / what(): Variable
   '..._mimic' is not known to model` the moment any client echoes `/joint_states` back
   inside a `RobotState` message. **Always build `RobotState` as an arm-only diff**
   (`is_diff = True`, only the six `arm_*` names) rather than copying `/joint_states`.
   This is a Phase 4 defect, not a MoveIt one; fixing it properly belongs in
   `ros2_control.xacro` / the Gazebo backend.

10. **KDL IK fails on poses you "know" are reachable, because the orientation is wrong, not
    the position.** At the edge of the workspace only a narrow band of tool orientations is
    feasible: over the pick table the tool axis has to be tilted ~37° off vertical, and the
    yaw is fixed near −90° by the arm's geometry. Guessing `rpy = (180, 0, 0)` returns
    `NO_IK_SOLUTION` at every reachable position. Get a real target by asking `/compute_fk`
    for the pose of a known configuration, then feed that pose back through `/compute_ik` —
    `phase5_plan_execute.py --goal-config` does exactly this.

11. **`avoid_collisions: true` on `/compute_ik` fails for *every* pose if a world collision
    object swallows the robot.** Modelling the workbench as one solid 1.5 × 0.8 × 1.03 box
    puts the stowed arm inside it, and then even the robot's current configuration reports
    `NO_IK_SOLUTION`. Model the tabletop slab only (`1.5 × 0.8 × 0.03` at z = 1.0); the
    space between the legs must stay free.

12. **Verify base pose with `gz model -m mobile_manipulator -p`, never with odometry.**
    Wheel odometry slips: after one arm extension it reported 0.26 m of backward travel
    while the ground truth was 0.16 m. If you validate "is the tool above the table?"
    against `odom` you will believe a pose that is 10 cm off.

13. **Same CLI hygiene as Phases 3/4**: `export ROS_LOCALHOST_ONLY=1` for the launch *and*
    every CLI call, kill stale `ros2cli.daemon` processes first, launch detached with
    `setsid nohup`.

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 0. Preflight (once per terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/mobile_manipulator_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
for p in $(ps aux | grep "[r]os2cli.daemon" | awk '{print $2}'); do kill -9 $p; done
```

### 1. Build the MoveIt Config Package
```bash
cd ~/mobile_manipulator_ws
colcon build --symlink-install --packages-select mobile_manipulator_moveit_config
source install/setup.bash
```

### 2. Regenerate the Self-Collision Matrix (only after a URDF or group change)
```bash
src/mobile_manipulator_moveit_config/scripts/regenerate_collision_matrix.sh
```

### 3a. Self-Contained Bench — mock hardware, no Gazebo
```bash
setsid nohup ros2 launch mobile_manipulator_moveit_config demo.launch.py \
  > /tmp/moveit_demo.log 2>&1 < /dev/null &
```
Starts robot_state_publisher + `ros2_control_node` (mock) + the four Phase 3 controllers +
move_group + RViz. `use_rviz:=false` for headless.

### 3b. Against the Phase 4 Gazebo World
```bash
# terminal 1 — Phase 4 (home_x puts the robot within arm's reach of the workbench)
setsid nohup ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py \
  home_x:=3.2 > /tmp/gazebo_warehouse.log 2>&1 < /dev/null &

# terminal 2 — move_group ONLY (Gazebo already provides rsp + controllers)
setsid nohup ros2 launch mobile_manipulator_moveit_config move_group.launch.py \
  use_sim_time:=true > /tmp/move_group.log 2>&1 < /dev/null &

# terminal 3 — RViz MotionPlanning panel
setsid nohup ros2 launch mobile_manipulator_moveit_config moveit_rviz.launch.py \
  use_sim_time:=true > /tmp/moveit_rviz.log 2>&1 < /dev/null &
```
Do **not** run `demo.launch.py` against Gazebo — it starts a second controller manager.

### 4. Headless Gate (equivalent to drag-marker → Plan → Execute)
```bash
ros2 run mobile_manipulator_moveit_config phase5_plan_execute.py \
  --frame base_footprint --use-sim-time \
  --goal-config 0.0 -1.1175 0.1054 -1.2083 -1.5708 0.0 \
  --workbench 1.307 0.0 1.000 1.50 0.80 0.03 --workbench-frame odom
```
Runs `/compute_fk` → `/compute_ik` → `/check_state_validity` → `/move_action`
(plan **and** execute) → per-waypoint collision check → `/joint_states` before/after.
`--plan-only` stops short of execution. `--x/--y/--z [--rpy R P Y]` takes a Cartesian goal
instead of `--goal-config`.

### 5. Ground-Truth Confirmation
```bash
gz model -m mobile_manipulator -p                    # world pose, x y z r p y
ros2 run tf2_ros tf2_echo odom arm_tool0
grep -E "Received new action goal|Goal reached" /tmp/gazebo_warehouse.log
```

**Pass Criteria**: `/compute_ik` returns `SUCCESS` for a pose above the pick workbench;
the goal state and **every** waypoint of the returned trajectory are collision free with
the tabletop in the planning scene; `move_group` returns `SUCCESS`; `arm_controller` in
gzserver logs `Goal reached, success!`; and `/joint_states` shows the Gazebo arm moved and
settled within a few mrad of the planned goal.

---

## As Built

```
mobile_manipulator_moveit_config/
├── .setup_assistant                       # MoveItConfigsBuilder manifest (urdf/srdf paths)
├── config/
│   ├── mobile_manipulator.urdf.xacro      # wrapper over mobile_manipulator_description
│   ├── mobile_manipulator.srdf.base       # groups / states / EE / virtual + passive joints
│   ├── mobile_manipulator.srdf            # .srdf.base + 162 <disable_collisions>
│   ├── kinematics.yaml                    # ur5_arm -> KDLKinematicsPlugin
│   ├── joint_limits.yaml                  # UR5 velocities + MoveIt-side accel limits
│   ├── moveit_controllers.yaml            # arm_controller + gripper_action_controller
│   ├── ompl_planning.yaml                 # RRTConnect default
│   ├── pilz_cartesian_limits.yaml
│   └── moveit.rviz                        # MotionPlanning panel, fixed frame odom
├── launch/
│   ├── move_group.launch.py               # move_group only (use_sim_time arg)
│   ├── demo.launch.py                     # mock-hardware bench, Phase 3 controllers
│   ├── moveit_rviz.launch.py
│   ├── rsp.launch.py
│   └── setup_assistant.launch.py          # reopen in the GUI
└── scripts/
    ├── regenerate_collision_matrix.sh     # collisions_updater at assistant defaults
    └── phase5_plan_execute.py             # headless plan+execute gate
```

**Planning groups**

| group | definition | solver |
|---|---|---|
| `ur5_arm` | chain `arm_base_link → arm_tool0` (exactly the 6 `arm_*` joints) | `kdl_kinematics_plugin/KDLKinematicsPlugin` |
| `gripper` | `gripper_robotiq_85_left_knuckle_joint` + 9 links | none (joint space only) |

End effector `robotiq_2f_85` (`parent_link arm_tool0`). Named states `home` / `upright` /
`pick_ready` for the arm, `open` / `closed` for the gripper. `virtual_joint` is **planar**,
`odom → base_footprint`; the four drive wheels are `passive_joint`.

**Execution wiring** — names copied verbatim from
`mobile_manipulator_description/config/mobile_manipulator_controllers.yaml`:

| MoveIt controller | type | action |
|---|---|---|
| `arm_controller` | `FollowJointTrajectory` | `/arm_controller/follow_joint_trajectory` |
| `gripper_action_controller` | `GripperCommand` | `/gripper_action_controller/gripper_cmd` |

**Measured gate result** (Gazebo at `home_x:=3.2`, ground-truth base x = 3.193):

```
IK (collision-aware):       SUCCESS
goal state collision free:  True
move_group error_code:      SUCCESS   (42 points, 4.0 s)
waypoints collision free:   42/42
|end-start| 1.5706 rad      |end-goal| 0.0008 rad
gzserver: [arm_controller] Received new action goal -> Goal reached, success!
gripper:  0.0 -> 0.6001 -> 0.0001 rad via /gripper_action_controller/gripper_cmd
```

---

## Open Items Handed to Later Phases

Neither is a MoveIt configuration problem; both are Phase 3/4 properties that Phase 5
exposed, and both will break Phase 8 pick-and-place if left alone.

1. **`_mimic` joint names** (gotcha 9). Fix belongs in `urdf/ros2_control.xacro` or a
   joint-state remap, so that `/joint_states` only ever carries names present in the URDF.

2. **The base is not braked.** Extending the arm rolled the robot **0.162 m backwards**
   (ground truth; odometry over-reported 0.261 m, so the wheels also slipped). A pose that
   was above the workbench at plan time (world x 3.793, table edge 3.75) ended up ~0.12 m
   short of the edge by the time execution finished. Any grasp pose computed before the
   arm moves will be wrong by that much.

3. **Reach margin over the pick workbench is thin.** With the base at a legal standoff
   (front bumper clear of the table edge at world x 3.75 and the leg at 3.82), the arm
   clears the 1.015 m tabletop by only a few centimetres and needs ~37° of tool tilt to get
   there — `arm_tool0` reaches world x 4.05 at z = 1.05 but only 3.97 at z = 1.15. A lower
   workbench or a taller arm mount would give Phase 8 real margin.
