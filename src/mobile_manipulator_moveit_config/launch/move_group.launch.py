#!/usr/bin/env python3
# move_group.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — start move_group ONLY.
#
# Use this against an already-running robot stack that provides
# robot_state_publisher, /joint_states and the ros2_control controllers, i.e.
# the Phase 4 Gazebo world:
#
#   ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py
#   ros2 launch mobile_manipulator_moveit_config move_group.launch.py use_sim_time:=true
#   ros2 launch mobile_manipulator_moveit_config moveit_rviz.launch.py use_sim_time:=true
#
# For a self-contained bench with mock hardware and no Gazebo, use
# demo.launch.py instead.
#
# NOTE on use_sim_time: with Gazebo publishing /clock, move_group MUST run on
# sim time or the trajectory-execution monitor compares a wall-clock "now"
# against sim-time trajectory stamps and aborts every goal with
# "Controller ... failed with error TIMED_OUT".
# ─────────────────────────────────────────────────────────────────────────────
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            'mobile_manipulator', package_name='mobile_manipulator_moveit_config')
        .robot_description(mappings={'sensor_arch': '0'})
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    move_group_configuration = {
        'use_sim_time': use_sim_time,
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'capabilities': ParameterValue(LaunchConfiguration('capabilities'), value_type=str),
        'disable_capabilities': ParameterValue(
            LaunchConfiguration('disable_capabilities'), value_type=str),
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), move_group_configuration],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from Gazebo. Set true when running against Phase 4.'),
        DeclareLaunchArgument('capabilities', default_value=''),
        DeclareLaunchArgument('disable_capabilities', default_value=''),
        move_group_node,
    ])
