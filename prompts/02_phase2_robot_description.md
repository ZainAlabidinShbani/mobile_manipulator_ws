# Phase 2 — Robot Description (Mobile Base + UR5 + Robotiq 2F-85 + D435i)

**Status: ✅ complete** (see `PLAN.md` §14 checklist)

## Task Prompt for Agent
```text
Refer to PLAN.md and instructions.md — this task is Phase 2 only.
In ~/mobile_manipulator_ws/src/mobile_manipulator_description, build a
Xacro robot description that composes (not reimplements) the vendor
packages: husky_description (the community Humble xacro port already
cloned into src/ — inspect its actual xacro/macro filenames first, they
do NOT match the official Clearpath layout), ur_description,
robotiq_description and realsense2_description (all from apt). Assemble a
mobile base with a UR5 arm mounted on top via a mounting-plate xacro
(top_plate_link -> arm mounting plate -> UR5 base_link), a Robotiq 2F-85
gripper on the UR5 tool0 via the ur_to_robotiq adapter, and a RealSense
D435i mounted at the wrist via a xacro macro, publishing camera_link and
camera_color_optical_frame per REP-103 conventions.
Reminder: no "husky" in anything we author — filenames, macro names,
link/joint names. The vendor package keeps its upstream name because it is
referenced, never redefined.
Provide a view_robot.launch.py that starts robot_state_publisher +
joint_state_publisher_gui + RViz2 (no Gazebo). Verify with
`xacro urdf/mobile_manipulator.urdf.xacro | check_urdf /dev/stdin` and
confirm zero errors, then take a screenshot artifact of the RViz view
showing the full assembled robot. Do not proceed to controllers or Gazebo
in this task.
```

---

## Known Gotchas (learned the hard way — do NOT skip)

1. **`arm_prefix` MUST be non-empty.** `ur_description`'s `ur_robot` macro names its root
   link `${tf_prefix}base_link`, which collides with the mobile base's own `base_link` and
   makes the URDF unparseable. The workspace standard is `arm_prefix:=arm_`, so the arm
   joints are `arm_shoulder_pan_joint … arm_wrist_3_joint` and the flange is `arm_tool0`.
   Everything downstream (controllers yaml, MoveIt SRDF, orchestrator) must use the
   prefixed names.

2. **Vendor ros2_control tags must be suppressed in this phase.** Pass
   `generate_ros2_control_tag:="false"` to `ur_robot` and `include_ros2_control:="false"`
   to `robotiq_gripper` — otherwise the vendor blocks fight the ones Phase 3 adds.

3. **The gripper is attached in two steps, not one.** `ur_to_robotiq` (adapter) is
   connected to `${arm_prefix}tool0` and produces `${gripper_prefix}gripper_mount_link`;
   the `robotiq_gripper` macro then parents to *that* link, not to `tool0`.

4. **`sensor_arch:=0` suppresses the base's decorative sensor arch**, which occupies the
   same top-plate real estate as the arm mounting plate. The top-level xacro already
   defaults it to `0`; launch files pass it explicitly so a changed vendor default cannot
   silently reintroduce the arch.

5. **A `Command([...xacro...])` substitution in a launch file must be wrapped in
   `ParameterValue(..., value_type=str)`**, or launch dies with "Unable to parse the value
   of parameter robot_description as yaml".

6. **`check_urdf` reads a file, not a pipe, reliably** — `xacro ... | check_urdf /dev/stdin`
   works, but writing to `/tmp/mm.urdf` first makes failures far easier to inspect.

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select husky_description mobile_manipulator_description
source install/setup.bash
```

### 2. Parse & Validate URDF
```bash
cd ~/mobile_manipulator_ws
xacro src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro sensor_arch:=0 > /tmp/mm.urdf
check_urdf /tmp/mm.urdf
```

### 3. Count links/joints (the numeric half of the gate)
```bash
python3 -c "
import xml.etree.ElementTree as ET
r = ET.parse('/tmp/mm.urdf').getroot()
print('links ', len(r.findall('link')))
print('joints', len(r.findall('joint')))
"
```

### 4. Launch Interactive RViz Viewer
```bash
ros2 launch mobile_manipulator_description view_robot.launch.py
```

### 5. Echo Frames & Check TF Tree (in another terminal tab)
```bash
ros2 run tf2_tools view_frames      # writes frames_<timestamp>.gv/.pdf into $PWD
```

**Pass Criteria**: `check_urdf` prints "Successfully Parsed XML" with the tree rooted at
`base_footprint` and zero errors — **55 links / 54 joints** (38 fixed, 8 continuous wheels
+ 8 revolute arm/gripper joints), no duplicate link names. RViz shows the mobile base, the
UR5 arm on its mounting plate, the Robotiq 2F-85 gripper on the flange, and the D435i at
the wrist, with nothing floating detached.

---

## As Built

```
mobile_manipulator_description/
├── urdf/
│   ├── mobile_manipulator.urdf.xacro   # top level — composes all four vendor packages
│   ├── arm_mount.xacro                 # 0.25 x 0.25 x 0.010 m plate, top_plate_link → arm_mounting_plate
│   └── d435i_wrist_mount.xacro         # bracket off arm_tool0 + realsense2 sensor_d435i macro
├── config/
│   ├── initial_positions.yaml          # UR5 stow pose (0, -90°, +90°, -90°, 0, 0)
│   └── view_robot.rviz
└── launch/
    └── view_robot.launch.py            # robot_state_publisher + joint_state_publisher_gui + RViz2
```

Attachment chain and prefixes (`arm_prefix:=arm_`, `gripper_prefix:=gripper_`,
`camera_name:=camera`):

```
base_footprint → base_link → top_plate_link
  → arm_mounting_plate → arm_base_link … arm_tool0
      → gripper_ur_to_robotiq_link → gripper_gripper_mount_link → gripper_robotiq_85_*
      → camera_bracket_link → camera_link / camera_color_optical_frame
```

The D435i bracket sits at `xyz="0.05 0 0.04"` off `arm_tool0` with `rpy="0 -0.5236 0"`
(−30° about Y) so the lens looks forward-and-down over a tabletop.
