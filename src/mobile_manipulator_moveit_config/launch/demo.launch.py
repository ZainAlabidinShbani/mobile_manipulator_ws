#!/usr/bin/env python3
# demo.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — self-contained MoveIt demo: no Gazebo, mock hardware only.
#
#   robot_state_publisher
#   ros2_control_node (mock_components/GenericSystem, from the Phase 2 URDF)
#   joint_state_broadcaster → diff_drive_controller → arm_controller
#                           → gripper_action_controller   (chained spawners)
#   move_group
#   RViz2 + MotionPlanning panel
#
# The controller manager, the controller YAML and the controller NAMES are
# exactly the ones from Phase 3 (mobile_manipulator_description/config/
# mobile_manipulator_controllers.yaml) — nothing is re-declared here, so a plan
# that executes in this demo executes identically against Gazebo.
#
# diff_drive_controller is spawned even though MoveIt never commands the base:
# it is what publishes the odom → base_footprint TF that the SRDF's planar
# virtual joint resolves against.  Without it MoveIt cannot place base_footprint
# in its own planning frame.
#
# Against the Phase 4 Gazebo world use move_group.launch.py + moveit_rviz.launch.py
# instead — this file would start a second, competing controller manager.
# ─────────────────────────────────────────────────────────────────────────────
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory('mobile_manipulator_description')
    controllers_yaml = os.path.join(
        description_share, 'config', 'mobile_manipulator_controllers.yaml')

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
    moveit_share = str(moveit_config.package_path)

    use_rviz = LaunchConfiguration('use_rviz')

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'publish_frequency': 30.0}],
    )

    # ── ros2_control (mock hardware) — Phase 3 stack, verbatim ──────────────
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[moveit_config.robot_description, controllers_yaml],
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    spawn_jsb = spawner('joint_state_broadcaster')
    spawn_diff_drive = spawner('diff_drive_controller')
    spawn_arm = spawner('arm_controller')
    spawn_gripper = spawner('gripper_action_controller')

    chain_diff_drive = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_diff_drive]))
    chain_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawn_diff_drive, on_exit=[spawn_arm]))
    chain_gripper = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper]))

    # ── move_group ──────────────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'capabilities': ParameterValue('', value_type=str),
                'disable_capabilities': ParameterValue('', value_type=str),
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
        ],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
    )

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'moveit_rviz.launch.py')),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        control_node,
        spawn_jsb,
        chain_diff_drive,
        chain_arm,
        chain_gripper,
        move_group_node,
        rviz,
    ])
