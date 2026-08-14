#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint


STOWED_POSE = [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0]
ARM_JOINTS = [
    "arm_shoulder_pan_joint",
    "arm_shoulder_lift_joint",
    "arm_elbow_joint",
    "arm_wrist_1_joint",
    "arm_wrist_2_joint",
    "arm_wrist_3_joint",
]


class HomeHold(Node):
    def __init__(self):
        super().__init__("home_hold")
        self.declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped")
        self.declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory")
        topic = self.get_parameter("cmd_vel_topic").value
        action = self.get_parameter("arm_action").value
        self.pub = self.create_publisher(Twist, topic, 1)
        self.ac = ActionClient(self, FollowJointTrajectory, action)
        self.timer = self.create_timer(0.02, self._spin_cmd)
        self.get_logger().info(f"Publishing zero cmd_vel to {topic} at 50 Hz")

    def _spin_cmd(self):
        self.pub.publish(Twist())

    def stow(self, duration=3.0):
        if not self.ac.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Arm controller action server not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance = Duration(sec=0, nanosec=500_000_000)
        tol = JointTolerance()
        tol.name = "arm_shoulder_pan_joint"
        tol.position = 0.01
        tol.velocity = 0.01
        goal.goal_tolerance = [tol]
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = STOWED_POSE
        pt.time_from_start.sec = int(duration)
        goal.trajectory.points = [pt]
        self.get_logger().info("Sending arm stow goal ...")
        future = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or not future.result().accepted:
            self.get_logger().error("Arm stow goal not accepted")
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 20.0)
        if result_future.done():
            status = result_future.result().status
            self.get_logger().info(f"Arm stow finished with status {status}")
            return status == 4
        self.get_logger().error("Arm stow timed out")
        return False


def main():
    rclpy.init()
    node = HomeHold()
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-stow", action="store_true")
    args, _ = parser.parse_known_args()
    ok = True
    if not args.no_stow:
        ok = node.stow()
    if ok:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()