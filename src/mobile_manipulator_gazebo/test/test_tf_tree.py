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
# test_tf_tree.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 10 — the TF tree must contain every frame the rest of the stack looks
# up, with no gaps between them.
#
# The frames asserted here are the ones other packages actually name:
# base_footprint (Nav2, the MoveIt virtual joint), the arm chain MoveIt plans
# over, the gripper links, camera_color_optical_frame (Phase 7 stamps images
# with it) and lidar_link (the /scan frame).  A missing parent is caught by
# asking tf2 to connect each frame back to base_footprint rather than by
# checking a flat list, because a frame can be published and still be
# unreachable if its parent chain is broken.
#
# Runs on the mock bench: the tree comes from the URDF and the joint states,
# both identical under Gazebo, and skipping physics keeps it fast.
# ─────────────────────────────────────────────────────────────────────────────
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
import launch_testing
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node as LaunchNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import pytest

import rclpy
from rclpy.node import Node

import tf2_ros

ROOT = 'base_footprint'

EXPECTED_FRAMES = [
    'base_link',
    'top_plate_link',
    'arm_mounting_plate',
    'arm_base_link',
    'arm_shoulder_link',
    'arm_upper_arm_link',
    'arm_forearm_link',
    'arm_wrist_1_link',
    'arm_wrist_2_link',
    'arm_wrist_3_link',
    'arm_tool0',
    'gripper_robotiq_85_base_link',
    'gripper_robotiq_85_left_finger_tip_link',
    'gripper_robotiq_85_right_finger_tip_link',
    'camera_link',
    'camera_color_frame',
    'camera_color_optical_frame',
    'lidar_link',
]


@pytest.mark.launch_test
def generate_test_description():
    # control_test.launch.py brings up ros2_control and the four controllers,
    # but it does NOT start robot_state_publisher — it does not need to, since
    # nothing in the Phase 3 gate reads TF.  A TF test does, and without it the
    # tree is simply absent and every frame here fails, which is exactly what
    # the first run of this test reported.  So the description half is added
    # explicitly, built from the same xacro with the same arguments.
    description_pkg = FindPackageShare('mobile_manipulator_description')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                PathJoinSubstitution(
                    [description_pkg, 'urdf', 'mobile_manipulator.urdf.xacro']),
                ' sensor_arch:=0',
            ]),
            value_type=str),
    }
    state_publisher = LaunchNode(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen')

    bench = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobile_manipulator_description'),
            'launch', 'control_test.launch.py')))
    return launch.LaunchDescription([
        state_publisher, bench, launch_testing.actions.ReadyToTest()])


class TestTfTree(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_tf_tree')
        cls.buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(cls.buffer, cls.node)
        # Let robot_state_publisher and the broadcaster fill the buffer.
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.2)
            if cls.buffer.can_transform(ROOT, 'arm_tool0', rclpy.time.Time()):
                break

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _can(self, frame, timeout=5.0):
        """Report whether tf2 can connect `frame` to the root."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.buffer.can_transform(ROOT, frame, rclpy.time.Time()):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return False

    def test_every_expected_frame_connects_to_the_root(self):
        # Short per-frame budget on purpose.  setUpClass has already waited
        # for the tree to come up, so by here every frame is either present or
        # never coming; an optimistic per-frame timeout multiplied by eighteen
        # frames is what pushed this past the ctest limit on the first run.
        missing = [f for f in EXPECTED_FRAMES if not self._can(f, timeout=5.0)]
        self.assertFalse(
            missing,
            f'frames unreachable from {ROOT} (missing frame or broken parent '
            f'chain): {missing}')

    def test_no_duplicate_or_orphan_root(self):
        """base_footprint must be the single root of the tree."""
        self.assertTrue(self._can('base_link', timeout=5.0),
                        'base_link does not connect to base_footprint')
        yaml_tree = self.buffer.all_frames_as_yaml()
        self.assertIn('base_link', yaml_tree, 'tf tree is empty')
        # Any frame naming a parent that is itself never published is an orphan.
        self.assertNotIn('NO_PARENT', yaml_tree.upper())
