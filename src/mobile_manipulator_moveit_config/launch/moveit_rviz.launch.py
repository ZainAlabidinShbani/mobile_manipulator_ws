#!/usr/bin/env python3
# moveit_rviz.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — RViz2 with the MotionPlanning display, pointed at a move_group that
# is already running (started by move_group.launch.py or demo.launch.py).
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
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(moveit_config.package_path / 'config' / 'moveit.rviz'),
            description='RViz configuration file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from Gazebo. Set true when running against Phase 4.'),
        rviz_node,
    ])
