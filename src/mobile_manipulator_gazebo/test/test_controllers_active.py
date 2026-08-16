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
# test_controllers_active.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 10 — every controller declared in mobile_manipulator_controllers.yaml
# must reach the 'active' state.
#
# Runs against control_test.launch.py (mock_components/GenericSystem), not
# against Gazebo: the controller set is identical under both backends by
# construction — ros2_control.xacro declares the same three <ros2_control>
# systems either way — and the mock bench starts in seconds without a physics
# engine, which is what makes this worth running on every build.
#
# The expected names are READ FROM THE YAML rather than hard-coded, so adding a
# controller without activating it fails this test instead of silently passing.
# ─────────────────────────────────────────────────────────────────────────────
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers

import launch
import launch_testing
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import pytest

import rclpy
from rclpy.node import Node

import yaml


def _expected_controllers():
    cfg = os.path.join(
        get_package_share_directory('mobile_manipulator_description'),
        'config', 'mobile_manipulator_controllers.yaml')
    with open(cfg) as fh:
        data = yaml.safe_load(fh)
    params = data['controller_manager']['ros__parameters']
    return sorted(k for k, v in params.items()
                  if isinstance(v, dict) and 'type' in v)


@pytest.mark.launch_test
def generate_test_description():
    bench = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobile_manipulator_description'),
            'launch', 'control_test.launch.py')))
    return launch.LaunchDescription([bench, launch_testing.actions.ReadyToTest()])


class TestControllersActive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_controllers_active')
        cls.client = cls.node.create_client(
            ListControllers, '/controller_manager/list_controllers')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _list(self, timeout):
        """Poll list_controllers until it answers, bounded by wall clock."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.client.wait_for_service(timeout_sec=2.0):
                fut = self.client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
                if fut.done() and fut.result() is not None:
                    return fut.result().controller
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return None

    def test_every_declared_controller_is_active(self):
        expected = _expected_controllers()
        self.assertTrue(expected, 'no controllers declared in the yaml')

        # Spawners are chained, so the last one can be ~30 s behind the first.
        deadline = time.monotonic() + 120.0
        states = {}
        while time.monotonic() < deadline:
            found = self._list(20.0)
            if found:
                states = {c.name: c.state for c in found}
                if all(states.get(n) == 'active' for n in expected):
                    break
            time.sleep(2.0)

        missing = [n for n in expected if n not in states]
        inactive = {n: states[n] for n in expected
                    if n in states and states[n] != 'active'}
        self.assertFalse(missing, f'controllers never appeared: {missing}')
        self.assertFalse(inactive, f'controllers not active: {inactive}')
