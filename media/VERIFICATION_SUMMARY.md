# Post-migration verification — Phases 2 to 9

Re-run of every phase gate from `PLAN.md` against the **Gazebo Fortress**
stack, in order, after the Classic → Fortress migration and the Phase 9 work.
Each gate was executed as written in `PLAN.md` / `CLAUDE.md`; media was captured
only after the gate condition was confirmed.

Phase 1 skipped (workspace scaffolding, nothing to verify at runtime).

| Phase | Gate | Result | Evidence |
|---|---|---|---|
| 2 — Robot description | `check_urdf` parses; RViz shows the assembled robot | **PASS** | `screenshots/Phase (2) - Robot Description.png` |
| 3 — ros2_control | all 4 controllers report `active` | **PASS** | `screenshots/Phase (3) - Controllers Active.png` |
| 4 — Gazebo world | robot spawns upright and stays; RTF > 0.7 | **PASS** (headless) | `videos/Phase (4) - Gazebo Stable Spawn.mp4`, `screenshots/Phase (4) - Real Time Factor.png` |
| 5 — MoveIt 2 | plans and executes a trajectory; arm follows in Gazebo | **PASS** | `videos/Phase (5) - MoveIt Plan Execution.mp4`, `videos/Phase (5) - Gazebo Arm Motion.mp4` |
| 6 — Nav2 | drives to a hardcoded goal, collision-free, inside tolerance | **PASS** | `videos/Phase (6) - Nav2 Navigation.mp4` |
| 7 — YOLOv8 perception | `object_target_frame` stable and within cm of ground truth | **PASS** | `videos/Phase (7) - YOLO Detection and TF.mp4` |
| 8 — Orchestrator dry run | full sequence, and forced failure → RECOVERY → ABORT | **PASS** | `screenshots/Phase (8) - Orchestrator Dry Run Success.png`, `screenshots/Phase (8) - Orchestrator Dry Run Failure Path.png` |
| 9 — Master launch + full cycle | one HOME→pick→drop→HOME with the object transported | **PASS, not yet repeatable** | `videos/Phase (9) - Full Pick and Place Cycle.mp4`, `screenshots/Phase (9) - Object On Drop Bench.png` |
| 10 — Automated tests | `colcon test` green | **PASS** — 53 tests, 0 errors, 0 failures, 3 skipped | see `PLAN.md` §14 |

## Measured numbers

| Phase | Metric | This run | Previously documented |
|---|---|---|---|
| 4 | Real-time factor (headless) | 0.72 – 1.03 | 0.995 |
| 4 | Base pose drift at home, 10 s | 0.0000 m | 0.0000 m |
| 5 | Final joint error vs goal | 0.0008 rad | — |
| 5 | Trajectory waypoints collision-free | 29 / 29 | — |
| 6 | Final pose error | **0.066 m / 0.197 rad** | 0.104 m / 0.130 rad |
| 6 | Min clearance to an obstacle | 0.059 m, collision-free | 0.111 m |
| 7 | `object_target_frame` vs ground truth | **1.2 mm** | 2.2 mm |
| 7 | Transform peak-to-peak over 10 s | 0.0000 m | 0.0000 m |
| 7 | Targets detected | **3 of 3** (0.39–0.76 conf.) | 1 of 3 (0.35 conf.) |
| 9 | Cycle wall-clock, zero recoveries | 418 s | — |

Phase 7's improvement is a direct consequence of the Phase 9 world change: the
pick/drop surfaces were re-authored as hand-written PBR benches, replacing the
vendored `model://table` whose unsupported Ogre material rendered **black** and
dragged the scene exposure down. That regression is now closed.

## Notes that matter when re-running these

* **Phase 4 must be measured headless.** With `gui:=true` the real-time factor
  is 0.36–0.47 and the gate *fails*; headless it is 0.72–1.03 and passes. The
  gz GUI costs ~127 % CPU on this 8-core box on top of the server's ~158 %.
  This is also why the Phase 9 video is recorded from a world-fixed camera
  (`demo_camera`, added to `warehouse.world`) rather than by screen-recording
  a GUI that must not be running.
* **Phase 7's look pose was re-solved.** `phase7_look_pose.py` still aimed at
  the old 1.0 m bench and framed empty air after the benches came down to
  0.60 m.
* **Phase 10's suite only became meaningful after two fixes.** colcon was
  running *no* tests in the two ament_python packages (it falls back to
  `setup.py test` without `tests_require`, and cheerfully reported "Ran 0
  tests ... OK"), and `test_tf_tree.py` failed for a real reason:
  `control_test.launch.py` starts the controllers but not
  `robot_state_publisher`, so there was no TF tree to test. Three tests are
  skipped by default because they need the whole simulator — enable with
  `-DMM_FULL_CYCLE_TEST=ON` and `MM_PERCEPTION_SIM_TEST=1`.
* **Phase 9 is a pass, not a guarantee.** Across eight end-to-end attempts in
  this session, three delivered the object onto the drop bench (confirmed
  against gz ground truth); one of those ran clean with zero recoveries in
  418 s. The rest failed in the manipulation states. The state machine now
  re-observes the object with its own camera after RELEASE and *fails* if it is
  not on the bench, so a failed delivery can no longer be reported as `DONE` —
  which is exactly what it caught on the failing runs. Remaining reliability
  work is listed under Phase 9 in `PLAN.md`.

## Tooling

No screen-capture tooling was installed on this machine (no ffmpeg,
wf-recorder, grim, gnome-screenshot or ImageMagick — only `xwd`). ffmpeg,
gnome-screenshot, ImageMagick and xterm were installed for this pass.

Capture is scripted in `media/capture.py`. Two constraints shaped it:

* the session is GNOME **Wayland**, so `ffmpeg -f x11grab` on the root returns
  black (measured mean pixel value 0.02) — only individual XWayland client
  windows can be dumped, with `xwd`;
* terminal gates are captured in `xterm`, an X client, because gnome-terminal
  is a native Wayland client and cannot be grabbed at all.

In-simulation views are recorded straight off ROS image topics, which avoids
needing a GUI window and costs nothing beyond the encode.
