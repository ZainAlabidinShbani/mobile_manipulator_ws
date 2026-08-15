#!/usr/bin/env python3
# nav2_bringup.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 6 — AMCL localization + Nav2 stack against the saved warehouse map.
#
# Expects the Phase 4 Gazebo stack (gazebo_warehouse.launch.py) to be running:
# it provides /clock, /scan, odom → base_footprint TF and the
# diff_drive_controller that consumes the velocity commands.
#
# NOTE: kill the home_hold node before sending navigation goals — it holds a
# 50 Hz zero-velocity stream on /diff_drive_controller/cmd_vel_unstamped that
# fights the controller_server:
#   pkill -f "[h]ome_hold"
#
# Topology:
#   map_server ──> AMCL (map → odom TF)
#   planner_server (NavFn) / controller_server (DWB) / behavior_server /
#   bt_navigator, all lifecycle-managed with autostart.
#   cmd_vel is remapped to /diff_drive_controller/cmd_vel_unstamped
#   (use_stamped_vel: false), odometry from /diff_drive_controller/odom.
# ─────────────────────────────────────────────────────────────────────────────
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_manipulator_navigation')

    default_map = os.path.join(pkg_share, 'maps', 'warehouse.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the occupancy map yaml')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameter file')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo /clock')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically bring up the lifecycle nodes')

    sim_time = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    cmd_vel_remap = ('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
    odom_remap = ('odom', '/diff_drive_controller/odom')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, sim_time,
                    {'yaml_filename': map_yaml}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, sim_time],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, sim_time],
        remappings=[cmd_vel_remap, odom_remap],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, sim_time],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, sim_time],
        remappings=[cmd_vel_remap],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, sim_time],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[sim_time,
                    {'autostart': ParameterValue(autostart, value_type=bool)},
                    {'node_names': ['map_server',
                                    'amcl',
                                    'controller_server',
                                    'planner_server',
                                    'behavior_server',
                                    'bt_navigator']}],
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
