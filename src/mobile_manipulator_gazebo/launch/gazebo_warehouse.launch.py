#!/usr/bin/env python3
# gazebo_warehouse.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 4 — start Gazebo Classic with the warehouse world and spawn the
# mobile manipulator at its named "home" pose, driving the base + arm +
# gripper through the gazebo_ros2_control plugin (controller manager runs
# INSIDE gzserver via libgazebo_ros2_control.so — no separate
# ros2_control_node process).
#
# The robot URDF is generated with use_gazebo:=true, which:
#   * swaps every ros2_control hardware block to gazebo_ros2_control/GazeboSystem
#   * adds the gazebo_ros2_control model plugin + D435i RGB camera sensor
#     (image published on /camera/color/image_raw)
# The generated URDF is post-processed: XML comments are stripped because
# gazebo_ros2_control 0.4.x re-injects the URDF into its own node as an
# rcl "--param robot_description:=<xml>" override, and rcl's YAML lexer
# chokes on comment characters (": ", box-drawing glyphs) in the value.
# HUSKY_GAZEBO_PLUGINS=0 suppresses the classic gazebo_ros diff_drive /
# joint-state / imu / gps plugins that would conflict with ros2_control.
# ─────────────────────────────────────────────────────────────────────────────
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory('mobile_manipulator_description')
    gazebo_pkg = FindPackageShare('mobile_manipulator_gazebo')

    world_file = PathJoinSubstitution([gazebo_pkg, 'worlds', 'warehouse.world'])
    controllers_yaml = PathJoinSubstitution(
        [description_share, 'config', 'mobile_manipulator_controllers.yaml'])

    # ── Suppress the classic gazebo_ros plugins from husky_description ──────
    # (diff_drive / joint_state_publisher / imu / gps would conflict with the
    # ros2_control + gazebo_ros2_control stack used in this launch).
    os.environ['HUSKY_GAZEBO_PLUGINS'] = '0'

    # ── Model resolution ────────────────────────────────────────────────────
    # The URDF→SDF converter rewrites package:// mesh URIs to model:// URIs
    # (e.g. model://husky_description/meshes/base_link.dae), so the package
    # directories must be reachable through GAZEBO_MODEL_PATH.  Without this,
    # gzserver falls back to the (long dead) online model database and hangs
    # the world-update loop, which also blocks the gazebo_ros2_control plugin
    # from ever loading.
    ws_src = os.path.join(os.path.dirname(os.path.dirname(
        get_package_prefix('mobile_manipulator_gazebo'))), 'src')
    gazebo_model_path = os.pathsep.join([
        os.path.expanduser('~/.gazebo/models'),
        ws_src,
        '/opt/ros/humble/share',
        '/usr/share/gazebo-11/models',
    ])
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    # Never fall back to the online database (fail fast instead of hanging).
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    # ── Home pose arguments (named "home" spawn pose) ────────────────────────
    home_x = LaunchConfiguration('home_x', default='0.0')
    home_y = LaunchConfiguration('home_y', default='0.0')
    home_z = LaunchConfiguration('home_z', default='0.0')
    home_yaw = LaunchConfiguration('home_yaw', default='0.0')
    gui = LaunchConfiguration('gui', default='true')

    declare_home_x = DeclareLaunchArgument('home_x', default_value='0.0', description='Home pose X [m]')
    declare_home_y = DeclareLaunchArgument('home_y', default_value='0.0', description='Home pose Y [m]')
    declare_home_z = DeclareLaunchArgument('home_z', default_value='0.0', description='Home pose Z [m]')
    declare_home_yaw = DeclareLaunchArgument('home_yaw', default_value='0.0', description='Home pose yaw [rad]')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start gzclient too. false = headless gzserver, which leaves the cores for sensor rendering.')

    # ── Gazebo server (+ client unless gui:=false) with the warehouse world ──
    # gzclient is a full 3D render of the whole warehouse and it competes with
    # gzserver's own sensor rendering for the same cores and GPU.  On an 8-core
    # box, running it alongside the two 640x480 wrist cameras drops those
    # cameras from ~3 Hz to under 0.3 Hz, which starves everything downstream of
    # them (Phase 7's detector, most obviously).  Pass gui:=false for headless
    # runs.  Note gzclient only actually starts when DISPLAY is set, so this
    # first bites on the phase that needs a display for something else.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': gui,
        }.items(),
    )

    # ── Robot description (gazebo flavour) ───────────────────────────────────
    # Generated eagerly so the URDF string can be post-processed: gazebo_ros2_control
    # 0.4.x re-parses robot_description as an rcl "--param name:=value" override,
    # whose YAML lexer rejects characters found in XML comments.
    xacro_proc = subprocess.run(
        ['xacro',
         os.path.join(description_share, 'urdf', 'mobile_manipulator.urdf.xacro'),
         'sensor_arch:=0', 'use_gazebo:=true',
         f'controllers_yaml:={os.path.join(description_share, "config", "mobile_manipulator_controllers.yaml")}'],
        capture_output=True, text=True, check=True)
    urdf_xml = xacro_proc.stdout
    urdf_xml = re.sub(r'<!--.*?-->', '', urdf_xml, flags=re.S)
    robot_description = {'robot_description': urdf_xml}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, robot_description],
        output='screen',
    )

    # ── Spawn the robot at the named "home" pose ─────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_manipulator',
            '-x', home_x,
            '-y', home_y,
            '-z', home_z,
            '-Y', home_yaw,
            '-timeout', '60',
        ],
        output='screen',
    )

    # ── Controller spawners (chained on spawn exit; the controller manager
    #    itself lives inside gzserver via the gazebo_ros2_control plugin) ─────
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    chain_diff_drive = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_diff_drive]))
    chain_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawn_diff_drive, on_exit=[spawn_arm]))
    chain_gripper = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper]))
    chain_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb]))

    # ── Home-hold node: stows the arm via JTC and holds zero wheel velocity ──
    # The diff drive controller only writes wheel commands once it has seen a
    # cmd_vel message; before that the wheels roll freely and the robot can
    # wander off after the spawn impulse.  home_hold publishes a constant
    # zero Twist (locking the wheels) and sends the arm to its stowed pose.
    home_hold = Node(
        package='mobile_manipulator_gazebo',
        executable='home_hold.py',
        output='screen',
    )
    chain_hold = RegisterEventHandler(
        OnProcessExit(target_action=spawn_gripper, on_exit=[home_hold]))

    return LaunchDescription([
        declare_home_x,
        declare_home_y,
        declare_home_z,
        declare_home_yaw,
        declare_gui,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        chain_spawn,
        chain_diff_drive,
        chain_arm,
        chain_gripper,
        chain_hold,
    ])
