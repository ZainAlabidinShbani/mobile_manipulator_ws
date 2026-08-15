#!/usr/bin/env python3
# rsp.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — robot_state_publisher fed from the MoveIt config's URDF (which is a
# thin wrapper around the Phase 2 description).  Included by demo.launch.py;
# NOT needed against Gazebo, where gazebo_warehouse.launch.py already runs one.
# ─────────────────────────────────────────────────────────────────────────────
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            'mobile_manipulator', package_name='mobile_manipulator_moveit_config')
        .robot_description(mappings={'sensor_arch': '0'})
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument('publish_frequency', default_value='30.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                {'publish_frequency': LaunchConfiguration('publish_frequency'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
