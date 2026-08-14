#!/usr/bin/env bash
# regenerate_collision_matrix.sh
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — regenerate config/mobile_manipulator.srdf from
# config/mobile_manipulator.srdf.base using the MoveIt Setup Assistant's OWN
# self-collision sampler (collisions_updater, the headless entry point of the
# assistant's "Self-Collisions" step) at its default settings:
#
#   --trials 10000                 assistant's default sampling density
#   --min-collision-fraction 0.95  assistant's default "always colliding" cutoff
#   --default --always             disable default-state and always-colliding pairs
#
# Run it after any change to the URDF or to the planning groups:
#
#   src/mobile_manipulator_moveit_config/scripts/regenerate_collision_matrix.sh
#
# Everything except <disable_collisions> lives in the .srdf.base file, so the
# hand-authored semantics are never clobbered by a regeneration.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

/opt/ros/humble/lib/moveit_setup_assistant/collisions_updater \
  --urdf   "${PKG_DIR}/config/mobile_manipulator.urdf.xacro" \
  --srdf   "${PKG_DIR}/config/mobile_manipulator.srdf.base" \
  --output "${PKG_DIR}/config/mobile_manipulator.srdf" \
  --xacro-args "sensor_arch:=0" \
  --default --always --verbose \
  --trials 10000 \
  --min-collision-fraction 0.95

echo "Wrote ${PKG_DIR}/config/mobile_manipulator.srdf"
grep -c disable_collisions "${PKG_DIR}/config/mobile_manipulator.srdf" \
  | xargs echo "disabled pairs:"
