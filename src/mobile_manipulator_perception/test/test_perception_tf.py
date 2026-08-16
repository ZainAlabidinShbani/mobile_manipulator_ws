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
# test_perception_tf.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 10 — the detector must publish object_target_frame when the wrist
# camera is actually pointed at the bench.
#
# TWO THINGS THIS TEST HAS TO DO THAT THE ONE-LINE BRIEF DOES NOT SAY, because
# without them it would assert something that can never be true:
#
#   * spawn the base AT THE BENCH (home_x:=3.24).  The detector broadcasts a
#     transform only while it has a live detection, and from the world origin
#     the targets are four metres away and out of frame.
#   * point the arm.  With the arm stowed the wrist camera looks at the
#     warehouse floor, so phase7_look_pose runs first.  The 15 s budget in the
#     brief is measured from the camera being aimed, not from launch: Gazebo
#     bringup plus the arm move is a minute on its own, and timing that would
#     be testing the simulator's start-up, not the perception node.
#
# Sim time also has to be enabled on the node or every stamp it publishes is
# zero and the freshness check below can never advance.
# ─────────────────────────────────────────────────────────────────────────────
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
import launch_testing
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node as LaunchNode

import pytest

import rclpy
from rclpy.node import Node

import tf2_ros

TARGET_FRAME = 'object_target_frame'
CAMERA_FRAME = 'camera_color_optical_frame'
DETECTION_BUDGET = 15.0

# OPT-IN, like the Phase 9 full-cycle test, and for the same reason: this one
# stands up gz-sim, drives the arm and runs YOLOv8 before it can assert
# anything, which takes over two minutes and needs a machine that can carry the
# simulator.  On a runner that cannot, it fails for reasons that say nothing
# about the perception node.  Enable with MM_PERCEPTION_SIM_TEST=1.
RUN_SIM_TEST = os.environ.get('MM_PERCEPTION_SIM_TEST', '') not in ('', '0')


@pytest.mark.skipif(not RUN_SIM_TEST,
                    reason='needs the full simulator; set MM_PERCEPTION_SIM_TEST=1')
@pytest.mark.launch_test
def generate_test_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobile_manipulator_gazebo'),
            'launch', 'gazebo_warehouse.launch.py')),
        launch_arguments={'home_x': '3.24', 'gui': 'false',
                          'hold_seconds': '30'}.items())

    perception = TimerAction(period=45.0, actions=[LaunchNode(
        package='mobile_manipulator_perception',
        executable='yolo_perception_node',
        name='yolo_perception_node',
        parameters=[{'use_sim_time': True}],
        output='screen')])

    # Aim the camera at the bench once the controllers are up and home_hold
    # has released the arm.
    look = TimerAction(period=95.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'mobile_manipulator_perception', 'phase7_look_pose',
             '--hold', '240', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen')])

    return launch.LaunchDescription([
        gazebo, perception, look,
        TimerAction(period=130.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestPerceptionTf(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_perception_tf')
        cls.buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(cls.buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_object_target_frame_appears(self):
        """The detector publishes camera -> object_target_frame."""
        deadline = time.monotonic() + DETECTION_BUDGET
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.buffer.can_transform(CAMERA_FRAME, TARGET_FRAME,
                                         rclpy.time.Time()):
                return
        self.fail(f'{TARGET_FRAME} did not appear within '
                  f'{DETECTION_BUDGET:.0f}s of the camera being aimed')

    def test_transform_is_live_not_stale(self):
        """
        The stamp must ADVANCE, not merely exist.

        The node stops broadcasting when it loses the target, and a buffered
        transform lingers afterwards — so an existence check alone would pass
        against a detector that had already given up.
        """
        first = None
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            try:
                tf = self.buffer.lookup_transform(CAMERA_FRAME, TARGET_FRAME,
                                                  rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if first is None:
                first = stamp
            elif stamp > first:
                self.assertGreater(tf.transform.translation.z, 0.0,
                                   'target reported behind the camera')
                return
        self.fail('object_target_frame stamp never advanced — the detector is '
                  'not producing fresh detections')
