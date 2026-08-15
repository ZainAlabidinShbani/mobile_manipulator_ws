#!/usr/bin/env python3
# slam.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 6 — one-shot slam_toolbox mapping session (online async) used to
# generate maps/warehouse.{yaml,pgm}. Run against the Phase 4 Gazebo stack,
# drive the mapping route (scripts/mapping_drive.py), then save:
#
#   ros2 run nav2_map_server map_saver_cli \
#     -f src/mobile_manipulator_navigation/maps/warehouse \
#     --ros-args -p use_sim_time:=true
#
# Not part of the runtime navigation stack — nav2_bringup.launch.py runs AMCL
# against the map this produces.
# ─────────────────────────────────────────────────────────────────────────────
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_manipulator_navigation')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='slam_toolbox parameter file')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([declare_params, slam])
