import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF and YAML paths
    description_pkg = FindPackageShare('mobile_manipulator_description')
    xacro_file = PathJoinSubstitution([description_pkg, 'urdf', 'mobile_manipulator.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([description_pkg, 'config', 'mobile_manipulator_controllers.yaml'])

    # Generate robot_description cleanly (explicitly typed as a string parameter)
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                xacro_file,
                ' sensor_arch:=0'
            ]),
            value_type=str
        )
    }

    # Controller Manager Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
    )

    # Sequential Spawning using Events to prevent parameter race conditions
    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[arm_spawner],
        )
    )

    delay_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_spawner,
            on_exit=[gripper_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive,
        delay_arm,
        delay_gripper
    ])
