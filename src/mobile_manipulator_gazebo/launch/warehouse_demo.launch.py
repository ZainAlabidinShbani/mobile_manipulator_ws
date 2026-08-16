#!/usr/bin/env python3
# Copyright 2026 Zain Alabidin Shbani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# warehouse_demo.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 9 — the master launch.  Brings the whole warehouse stack up in strict
# dependency order and finishes by handing control to the orchestrator:
#
#   1. Gazebo Fortress + warehouse world + robot spawn + ros2_control
#   2. Nav2            (after all four controllers report active)
#   3. move_group      (after Nav2's bt_navigator reaches the active state)
#   4. RViz2           (same stage as move_group — it renders both)
#   5. yolo_perception_node   (after move_group answers services)
#   6. warehouse_orchestrator, dry_run:=false (after the detector is up)
#
# ORDERING IS BY OBSERVED STATE, NOT BY TIMER.
#
# Every stage is separated by a "gate": a short process that polls the system
# for the thing the next stage actually depends on and exits 0 the moment it
# is true.  An OnProcessExit handler on that gate starts the next stage — and
# if a gate times out it exits non-zero and the launch shuts down with a named
# reason instead of cascading into a stack that half-works.  TimerActions were
# deliberately not used: on this box the controller chain alone varies between
# 12 s and 40 s depending on whether the Gazebo GUI is up, so any timer long
# enough to be safe is mostly dead waiting, and any timer short enough to feel
# responsive is a race.
#
# The interlock that matters most is the first one.  home_hold pins cmd_vel to
# zero at 50 Hz so the free-rolling wheels stay put between spawn and the
# first real command; that stream beats Nav2's controller_server outright, so
# Nav2 must not start until it stops.  Rather than race a `pkill` against the
# launch — whose -f pattern also matches the shell running it — the Gazebo
# launch is given hold_seconds, and the gate simply waits for the process to
# leave.
#
# Arguments:
#   gui:=false      run gz-sim headless (default).  The GUI renders the whole
#                   warehouse and starves the server's own sensor rendering:
#                   with it up, the two 640x480 wrist cameras fall from ~6 Hz
#                   to under 0.3 Hz and the detector sees almost nothing.
#   rviz:=true      start RViz2 with config/warehouse_demo.rviz
#   dry_run:=false  orchestrator stubs everything if true (Phase 8 logic test)
#   cycles:=1       how many pick-and-place cycles to run
#
# Verify with:
#   ros2 launch mobile_manipulator_gazebo warehouse_demo.launch.py
# ─────────────────────────────────────────────────────────────────────────────
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def gate(name, conditions, timeout=300, period=2):
    """
    Build a process that waits for each of `conditions` in turn, then exits 0.

    Each condition is a shell snippet evaluated for its exit status.  Passing
    more than one makes the gate a *sequence*, which is what turns a state
    check into a causality check: "home_hold is running" followed by
    "home_hold is gone" can only be satisfied by this launch's own bringup,
    whereas "home_hold is not running" is also true before it ever starts —
    and, worse, so is "four controllers are active" if a stale controller
    manager from an earlier session is still alive.  That exact leftover
    (a MoveIt demo bench) once satisfied every gate here within two seconds
    and started Nav2, move_group and the orchestrator against a simulation
    whose robot had not been spawned yet.

    On timeout the gate exits 1, which the caller turns into a launch shutdown
    with a named reason — a stage that never became ready must not be papered
    over by starting the stage that depends on it.
    """
    if isinstance(conditions, str):
        conditions = [conditions]
    steps = ''.join(
        f'echo "[gate:{name}] step {i + 1}/{len(conditions)}"; '
        f'while ! ( {c} ) >/dev/null 2>&1; do '
        f'  if [ $(date +%s) -ge $deadline ]; then '
        f'    echo "[gate:{name}] TIMED OUT after {timeout}s on step {i + 1}"; exit 1; fi; '
        f'  sleep {period}; '
        f'done; '
        for i, c in enumerate(conditions))
    script = (f'deadline=$(( $(date +%s) + {timeout} )); '
              f'echo "[gate:{name}] waiting"; '
              + steps
              + f'echo "[gate:{name}] satisfied"')
    return ExecuteProcess(
        cmd=['bash', '-c', script], name=f'gate_{name}', output='screen')


def when_ready(gate_action, name, actions):
    """Register an exit handler: run `actions` on success, shut down on failure."""
    def on_exit(event, context):
        if event.returncode == 0:
            return actions
        return [
            LogInfo(msg=f'[warehouse_demo] gate "{name}" failed — aborting bringup'),
            Shutdown(reason=f'{name} never became ready'),
        ]
    return RegisterEventHandler(OnProcessExit(target_action=gate_action, on_exit=on_exit))


def generate_launch_description():
    gazebo_share = get_package_share_directory('mobile_manipulator_gazebo')
    nav_share = get_package_share_directory('mobile_manipulator_navigation')

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    dry_run = LaunchConfiguration('dry_run')
    cycles = LaunchConfiguration('cycles')
    rviz_config = LaunchConfiguration('rviz_config')

    args = [
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start the gz-sim GUI. Leave false: it starves the wrist cameras.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 with the warehouse demo layout.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(gazebo_share, 'config', 'warehouse_demo.rviz'),
            description='RViz2 configuration file.'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='Orchestrator stubs Nav2/MoveIt/TF/gripper instead of calling them.'),
        DeclareLaunchArgument(
            'cycles', default_value='1',
            description='Pick-and-place cycles to run before exiting.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='25',
            description='Seconds home_hold holds the base before releasing it to Nav2.'),
        DeclareLaunchArgument(
            'gui_render_engine', default_value='ogre2',
            description='Rendering engine for the gz-sim GUI process only (see '
                        'gazebo_warehouse.launch.py). Pass ogre to work around the '
                        'corrupted Ogre2 viewport under XWayland; the server keeps '
                        'ogre2 either way, so the wrist camera still sees PBR.'),
    ]

    # ── stage 1 — Gazebo + world + robot + ros2_control ─────────────────────
    # Phase 4's launch already owns this whole subtree (world, bridge,
    # robot_state_publisher, spawn, and the four chained controller spawners),
    # so it is included rather than re-implemented: one definition of the
    # simulation, used by both the Phase 4 gate and the master launch.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo_warehouse.launch.py')),
        launch_arguments={
            'gui': gui,
            'hold_seconds': LaunchConfiguration('hold_seconds'),
            'gui_render_engine': LaunchConfiguration('gui_render_engine'),
        }.items(),
    )

    # ── stage 2 — Nav2 ───────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ── stage 3 — move_group + RViz2 ─────────────────────────────────────────
    # Built exactly like move_group.launch.py.  use_sim_time is not optional
    # here: with Gazebo publishing /clock, a wall-clock move_group compares
    # "now" against sim-time trajectory stamps and aborts every goal with
    # "Controller ... failed with error TIMED_OUT".
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

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {
            'use_sim_time': True,
            'publish_robot_description_semantic': True,
            'allow_trajectory_execution': True,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'monitor_dynamics': False,
        }],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='warehouse_demo_rviz',
        output='log',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    # ── stage 4 — YOLOv8 perception ──────────────────────────────────────────
    perception = Node(
        package='mobile_manipulator_perception',
        executable='yolo_perception_node',
        name='yolo_perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── stage 5 — the orchestrator ───────────────────────────────────────────
    orchestrator = Node(
        package='mobile_manipulator_orchestrator',
        executable='warehouse_orchestrator',
        name='warehouse_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'dry_run': ParameterValue(dry_run, value_type=bool),
            'cycles': ParameterValue(cycles, value_type=int),
        }],
    )

    # ── the gates ────────────────────────────────────────────────────────────
    # 1. All four controllers active, then the base released.  Written as a
    #    sequence for the reason in gate()'s docstring: home_hold is started by
    #    the Phase 4 launch only after the last controller spawner exits, so
    #    "it appeared" proves this launch's controllers came up, and "it left"
    #    proves the 50 Hz zero-velocity stream has stopped fighting Nav2.
    #    The [h]ome pattern keeps pgrep from matching this gate's own command
    #    line — pgrep -f matches the whole line, including the pattern itself.
    gate_controllers = gate(
        'controllers',
        ['ros2 control list_controllers 2>/dev/null | grep -q "gripper_action_controller.*active"',
         'pgrep -f "[h]ome_hold"',
         'test "$(ros2 control list_controllers 2>/dev/null | grep -c active)" -eq 4',
         '! pgrep -f "[h]ome_hold"'],
        timeout=300)
    # 2. Nav2 is not merely spawned, its behaviour tree is in the active state.
    gate_nav2 = gate(
        'nav2',
        'ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active',
        timeout=300)
    # 3. move_group answers.  /apply_planning_scene only exists once the
    #    capability plugins have finished loading, unlike the node name.
    gate_move_group = gate(
        'move_group',
        'ros2 service type /apply_planning_scene 2>/dev/null | grep -q ApplyPlanningScene',
        timeout=300)
    # 4. the detector is publishing its annotated stream.  Model load
    #    (ultralytics + torch) is what makes this the slow one.
    gate_perception = gate(
        'perception',
        'ros2 topic list 2>/dev/null | grep -q annotated_image',
        timeout=420)

    return LaunchDescription(args + [
        LogInfo(msg='[warehouse_demo] stage 1/5 — Gazebo, robot, ros2_control'),
        gazebo,
        gate_controllers,
        when_ready(gate_controllers, 'controllers', [
            LogInfo(msg='[warehouse_demo] stage 2/5 — Nav2'),
            nav2,
            gate_nav2,
        ]),
        when_ready(gate_nav2, 'nav2', [
            LogInfo(msg='[warehouse_demo] stage 3/5 — move_group + RViz2'),
            move_group,
            rviz,
            gate_move_group,
        ]),
        when_ready(gate_move_group, 'move_group', [
            LogInfo(msg='[warehouse_demo] stage 4/5 — YOLOv8 perception'),
            perception,
            gate_perception,
        ]),
        when_ready(gate_perception, 'perception', [
            LogInfo(msg='[warehouse_demo] stage 5/5 — orchestrator (live)'),
            orchestrator,
        ]),
    ])
