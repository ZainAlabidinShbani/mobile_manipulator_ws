---
trigger: always_on
---

# Antigravity Agent Project Rules & Instructions

## 1. Workspace & Path Constraints
- The project workspace root is ALWAYS `~/mobile_manipulator_ws`.
- All commands, packages, builds, and launches MUST be executed inside `~/mobile_manipulator_ws`.
- Operating System & Environment: Ubuntu 22.04 LTS running ROS 2 Humble under Zsh shell (`/bin/zsh`).

## 2. Naming Conventions (STRICT)
- NEVER use the word "husky" in package names, folder names, Python nodes, C++ targets, or launch files.
- ALWAYS use the prefix `mobile_manipulator_*` for all custom packages:
  - `mobile_manipulator_description`
  - `mobile_manipulator_gazebo`
  - `mobile_manipulator_navigation`
  - `mobile_manipulator_moveit_config`
  - `mobile_manipulator_perception`
  - `mobile_manipulator_orchestrator`

## 3. Phased Execution & PLAN.md Adherence
- Follow the 11-phase workflow outlined in `@PLAN.md` strictly in sequential order.
- Do NOT jump to a new phase until the verification gate for the current phase is executed and passes with zero errors.
- Update the checklist (`- [x]`) in `@PLAN.md` upon successfully completing each phase.

## 4. Build, Verification & Dependency Rules
- Always update `package.xml` and `CMakeLists.txt` (or `setup.py`) whenever new ROS 2 dependencies or python libraries are added.
- Always verify generated/modified code using `colcon build --symlink-install` and source `install/setup.zsh`.
- If a build or execution error occurs, isolate the root cause within the current phase before modifying other subsystems.