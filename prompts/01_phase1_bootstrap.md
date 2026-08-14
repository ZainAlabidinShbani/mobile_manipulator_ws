# Phase 1 — Workspace Bootstrap

**Status: ✅ complete** (see `PLAN.md` §14 checklist)

## Task Prompt for Agent
```text
In ~/mobile_manipulator_ws, create 6 valid ROS 2 Humble packages:
- mobile_manipulator_description (ament_cmake)
- mobile_manipulator_gazebo (ament_cmake)
- mobile_manipulator_navigation (ament_cmake)
- mobile_manipulator_moveit_config (ament_cmake, skeleton)
- mobile_manipulator_perception (ament_python)
- mobile_manipulator_orchestrator (ament_python)

Each package needs correct package.xml, CMakeLists.txt / setup.py, and license/README.
Run `colcon build --symlink-install` and confirm all build with zero errors.
Do not write any robot-specific code yet — this phase is scaffolding only.
```

---

## Known Gotchas (learned the hard way — do NOT skip)

1. **The mobile base is not installable from apt on Humble.** Clearpath dropped the
   standalone `husky_description` deb in favour of the yaml-driven `clearpath_common`
   ecosystem, which does not compose cleanly with a hand-written UR5 + ros2_control setup.
   The plain-xacro community port is cloned straight into `src/` instead (see below), so it
   builds like any other workspace package. `src/husky_description` is therefore a
   **nested git repo** (a bare gitlink — there is no `.gitmodules`); commit changes to it
   from inside that directory.

2. **Keep the vendor package's upstream name.** The "no husky in any identifier" rule from
   `.agents/rules/instructions.md` applies to everything *we* author. The cloned vendor
   package stays `husky_description` — renaming it would break `$(find ...)`, its own
   internal includes, and the mesh `package://` URIs.

3. **`install/` `build/` `log/` are gitignored** — a fresh clone of this workspace must
   re-run the preflight + `colcon build` before any `ros2` command resolves.

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 0. Clone the vendor base description (once, before the first build)
```bash
cd ~/mobile_manipulator_ws/src
git clone https://github.com/akrbot/husky_description_ros2.git husky_description
cd ~/mobile_manipulator_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 1. Build Command
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. Verification Command
```bash
source ~/mobile_manipulator_ws/install/setup.bash
ros2 pkg list | grep -E "mobile_manipulator|husky_description"
```

**Pass Criteria**: all 6 `mobile_manipulator_*` packages **plus** the vendor
`husky_description` are listed, with zero build errors.

---

## As Built

```
src/
├── husky_description/                 (vendor clone, ament_cmake — nested git repo)
├── mobile_manipulator_description/    (ament_cmake — urdf/ config/ launch/, Phases 2–3)
├── mobile_manipulator_gazebo/         (ament_cmake — worlds/ launch/ scripts/, Phase 4)
├── mobile_manipulator_navigation/     (ament_cmake — empty skeleton, Phases 6/9/10)
├── mobile_manipulator_moveit_config/  (ament_cmake — empty skeleton, Phase 5)
├── mobile_manipulator_perception/     (ament_python — empty skeleton, Phase 7)
└── mobile_manipulator_orchestrator/   (ament_python — empty skeleton, Phase 8)
```

The four skeleton packages install an empty share directory on purpose
(`install(DIRECTORY DESTINATION share/${PROJECT_NAME})`) so they stay buildable until
their phase populates them.
