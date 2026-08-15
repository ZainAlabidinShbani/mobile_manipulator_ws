# Mobile Manipulator Task Prompts & Commands Reference

This folder contains pre-formatted prompts and command sequences for each phase of the Mobile Manipulator ROS 2 Humble project.

## How to Use These Prompts

1. **Sequential Execution**: Run each phase in order. Do not advance to Phase *N+1* until Phase *N*'s verification gate passes with zero errors.
2. **In Antigravity / AI Assistant**: Copy the text inside the `Task Prompt` section of each phase document as your prompt when starting a new task phase.
3. **In GNOME Terminal (`gterminal`)**: Copy and execute the commands inside the `Terminal Commands` section to launch, verify, test, or troubleshoot each phase.

---

## Index of Phases

| File | Phase | Topic | Output / Verification Gate | Status |
|------|-------|-------|----------------------------|--------|
| [`00_environment_preflight.sh`](file:///home/zsh/mobile_manipulator_ws/prompts/00_environment_preflight.sh) | Preflight | Environment setup & apt packages | ROS 2 Humble deps installed | ✅ |
| [`01_phase1_bootstrap.md`](file:///home/zsh/mobile_manipulator_ws/prompts/01_phase1_bootstrap.md) | Phase 1 | Workspace Bootstrap | Package skeletons build cleanly | ✅ |
| [`02_phase2_robot_description.md`](file:///home/zsh/mobile_manipulator_ws/prompts/02_phase2_robot_description.md) | Phase 2 | Robot Description (Xacro/URDF) | `check_urdf` passes (55 links / 54 joints), RViz display green | ✅ |
| [`03_phase3_ros2_control.md`](file:///home/zsh/mobile_manipulator_ws/prompts/03_phase3_ros2_control.md) | Phase 3 | `ros2_control` & Controllers | Controllers `active` on mock hardware | ✅ |
| [`04_phase4_gazebo_world.md`](file:///home/zsh/mobile_manipulator_ws/prompts/04_phase4_gazebo_world.md) | Phase 4 | Gazebo Warehouse World | RTF > 0.7, robot stable on spawn | ✅ |
| [`05_phase5_moveit2_config.md`](file:///home/zsh/mobile_manipulator_ws/prompts/05_phase5_moveit2_config.md) | Phase 5 | MoveIt 2 Configuration | Motion planning + trajectory execution | ✅ |
| [`06_phase6_nav2.md`](file:///home/zsh/mobile_manipulator_ws/prompts/06_phase6_nav2.md) | Phase 6 | Nav2 Stack | Autonomous navigation to goal pose | ☐ next |
| [`07_phase7_yolo_perception.md`](file:///home/zsh/mobile_manipulator_ws/prompts/07_phase7_yolo_perception.md) | Phase 7 | YOLOv8 Perception Node | 3D bounding box & target TF published | ☐ |
| [`08_phase8_orchestrator.md`](file:///home/zsh/mobile_manipulator_ws/prompts/08_phase8_orchestrator.md) | Phase 8 | Orchestrator State Machine | State transitions & error handling dry-run | ☐ |
| [`09_phase9_master_integration.md`](file:///home/zsh/mobile_manipulator_ws/prompts/09_phase9_master_integration.md) | Phase 9 | Master Launch & Integration | End-to-end pick and place run | ☐ |
| [`10_phase10_automated_tests.md`](file:///home/zsh/mobile_manipulator_ws/prompts/10_phase10_automated_tests.md) | Phase 10 | Automated Integration Testing | `colcon test` all green | ☐ |
| [`11_phase11_hardening.md`](file:///home/zsh/mobile_manipulator_ws/prompts/11_phase11_hardening.md) | Phase 11 | System Hardening & Retries | 5 consecutive automated cycles succeed | ☐ |
| [`ALL_PHASE_PROMPTS.md`](file:///home/zsh/mobile_manipulator_ws/prompts/ALL_PHASE_PROMPTS.md) | Master | All Prompts Combined | Single reference document | — |

Completed phase docs (1–5) carry a **Known Gotchas** section and an **As Built** section
recording what actually shipped — read those before touching that subsystem again.
