#!/usr/bin/env python3
# gazebo_warehouse.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 4 — start Gazebo Fortress (gz-sim 6) with the warehouse world and
# spawn the mobile manipulator at its named "home" pose, driving the base +
# arm + gripper through gz_ros2_control (the controller manager runs INSIDE
# the gz-sim server via libgz_ros2_control-system.so — there is no separate
# ros2_control_node process).
#
# The robot URDF is generated with use_gazebo:=true, which:
#   * swaps every ros2_control hardware block to gz_ros2_control/GazeboSimSystem
#   * adds the gz_ros2_control system plugin + the D435i camera/depth sensors
#     and the front 2D lidar
#
# MIGRATION NOTE (Gazebo Classic → Fortress, Jan 2025 EOL):
#   gazebo_ros/gazebo.launch.py  → ros_gz_sim/gz_sim.launch.py
#   gazebo_ros spawn_entity.py   → ros_gz_sim `create`  (-entity became -name)
#   GAZEBO_MODEL_PATH            → IGN_GAZEBO_RESOURCE_PATH
# On Fortress the binary is `ign gazebo`, not `gz sim` (that is Garden and
# later); gz_sim.launch.py picks the right one from gz_version, which must
# stay '6'.  /usr/bin/gz belongs to Gazebo Classic, which is still installed.
#
# Unlike Classic, sim time does NOT arrive by itself: gz-sim publishes it on
# the gz transport topic /clock, and it has to cross the ros_gz bridge before
# any use_sim_time node (controller_manager included) will step.  The clock
# bridge below is therefore load-bearing, not a convenience.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory('mobile_manipulator_description')
    gazebo_pkg = FindPackageShare('mobile_manipulator_gazebo')

    world_file = PathJoinSubstitution([gazebo_pkg, 'worlds', 'warehouse.world'])

    # ── Suppress the classic gazebo_ros plugins from husky_description ──────
    # Fortress ignores unknown Classic plugin filenames anyway, but the guard
    # keeps them out of the SDF entirely instead of emitting a load error per
    # plugin on every startup.
    os.environ['HUSKY_GAZEBO_PLUGINS'] = '0'

    # ── Model resolution ────────────────────────────────────────────────────
    # The URDF→SDF converter rewrites package:// mesh URIs to model:// URIs
    # (e.g. model://husky_description/meshes/base_link.dae), and the world's
    # <include><uri>model://bookshelf</uri> entries resolve the same way, so
    # every directory holding a model must be on the resource path.  Fortress
    # reads IGN_GAZEBO_RESOURCE_PATH; GZ_SIM_RESOURCE_PATH is set too because
    # gz_sim.launch.py forwards both and later gz versions only read the
    # latter.  Note ros_gz_sim's launch file APPENDS to whatever is already in
    # the environment, so setting these here is additive, not a clobber.
    ws_src = os.path.join(os.path.dirname(os.path.dirname(
        get_package_prefix('mobile_manipulator_gazebo'))), 'src')
    resource_path = os.pathsep.join([
        os.path.expanduser('~/.gazebo/models'),
        ws_src,
        '/opt/ros/humble/share',
    ])
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = resource_path
    os.environ['GZ_SIM_RESOURCE_PATH'] = resource_path

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
        description='Start the gz-sim GUI too. false = headless server, which leaves the cores for sensor rendering.')

    # ── Gazebo Sim server (+ GUI unless gui:=false) with the warehouse world ─
    # The GUI renders the whole warehouse and competes with the server's own
    # sensor rendering for the same cores and GPU.  On an 8-core box, running
    # it alongside the two 640x480 wrist cameras drops those cameras from
    # ~6 Hz to under 0.3 Hz, which starves everything downstream of them
    # (Phase 7's detector, most obviously).  Pass gui:=false for headless runs.
    #
    # gz_args flags:  -r  start the world unpaused (Classic ran on load)
    #                 -s  server only, no GUI process
    #                 -v4 info-level logging, the Classic verbose:=true analogue
    gz_args = [
        PythonExpression([
            '"-r -v 4 " if "', gui, '" in ("true", "True", "1") else "-s -r -v 4 "',
        ]),
        world_file,
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': gz_args,
            'gz_version': '6',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Robot description (gazebo flavour) ───────────────────────────────────
    # Generated eagerly so the URDF string can be post-processed: comments are
    # stripped because the description is round-tripped through node parameter
    # overrides whose YAML lexer rejects characters found in XML comments
    # (": ", box-drawing glyphs).
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

    # ── Clock bridge ─────────────────────────────────────────────────────────
    # Load-bearing: gz-sim owns simulation time and publishes it on the gz
    # transport topic /clock.  Without this bridge every use_sim_time node
    # (robot_state_publisher, the controller manager's own clock, MoveIt,
    # Nav2, the perception node) sits at t=0 forever.  Classic got this from
    # the gazebo_ros_init plugin, which has no Fortress equivalent.
    # Direction is gz -> ROS only ("[") so nothing can publish time back.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    # ── Spawn the robot at the named "home" pose ─────────────────────────────
    # ros_gz_sim `create` replaces gazebo_ros spawn_entity.py.  The flag for
    # the model name is -name, not -entity.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mobile_manipulator',
            '-x', home_x,
            '-y', home_y,
            '-z', home_z,
            '-Y', home_yaw,
        ],
        output='screen',
    )

    # ── Controller spawners (chained on spawn exit; the controller manager
    #    itself lives inside the gz-sim server via gz_ros2_control) ───────────
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
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        chain_spawn,
        chain_diff_drive,
        chain_arm,
        chain_gripper,
        chain_hold,
    ])
