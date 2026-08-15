#!/usr/bin/env python3
# setup_assistant.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Re-open this package in the MoveIt Setup Assistant GUI (edit mode), e.g. to
# re-run the self-collision sampling or add a planning group.  For a headless
# regeneration of just the collision matrix, use
# scripts/regenerate_collision_matrix.sh instead.
# ─────────────────────────────────────────────────────────────────────────────
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'mobile_manipulator', package_name='mobile_manipulator_moveit_config'
    ).to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
