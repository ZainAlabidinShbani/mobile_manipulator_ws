#!/usr/bin/env python3
"""
Aim the wrist D435i at the pick workbench (Phase 7 gate helper).

phase7_look_pose.py
─────────────────────────────────────────────────────────────────────────────

The Phase 7 gate wants the robot "parked at the pick table" with the targets
in view.  Park the base by spawning there:

    ros2 launch mobile_manipulator_gazebo gazebo_warehouse.launch.py home_x:=3.24

3.24 m is as close as the base gets: the lidar puck protrudes to x = +0.575 and
the workbench's near legs stand at x = 3.82.  Then run this script to fold the arm
out over the bench.

WHY TWO WAYPOINTS, NOT ONE
    JointTrajectoryController interpolates in joint space.  Interpolating
    straight from the stowed pose to LOOK_POSE drags the wrist, tool0 and the
    gripper through the workbench slab between roughly 30 % and 65 % of the
    way — Gazebo resolves that interpenetration by launching the whole robot
    off the map (the base ends up hundreds of metres away and the joint
    controllers report "Goal reached, success!" the entire time).  LIFT_POSE
    raises the elbow first so the whole arm clears z = 1.05 before it reaches
    forward.  Both segments were swept at 200 samples against the slab and the
    chassis before being hard-coded here.

WHY --hold EXISTS
    The parked base does not stay parked once the arm is extended.  The arm
    joints are driven through a position command interface with no PID, so
    gazebo_ros2_control holds them with gazebo::physics::Joint::SetPosition —
    a kinematic teleport every control cycle rather than a torque.  That leaks
    momentum into the floating base, and the wheels cannot absorb it because
    they are held the same way (Joint::SetVelocity, also kinematic).  The
    parked robot therefore rolls backwards at about 1 cm/s — genuinely rolling,
    /odom sees it — and roughly 25 s later it lurches ~0.4 m sideways and
    yaws ~0.65 rad, after which the bench is no longer in frame.
    --hold closes the loop on /odom and steers the base back to where it stood
    when the arm arrived, which keeps the view usable for as long as the gate
    needs.  Kill home_hold first, it pins cmd_vel to zero at 50 Hz and wins.

Verify afterwards with:
    ros2 run tf2_ros tf2_echo base_footprint camera_color_optical_frame
which should read translation ~[0.601, -0.176, 1.240].
"""
import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'arm_shoulder_pan_joint',
    'arm_shoulder_lift_joint',
    'arm_elbow_joint',
    'arm_wrist_1_joint',
    'arm_wrist_2_joint',
    'arm_wrist_3_joint',
]

# Elbow up and back — clears the workbench before the arm reaches over it.
LIFT_POSE = [-0.8342, -2.2000, 0.9000, 0.1002, 0.9323, -0.6261]

# RE-SOLVED FOR THE 0.60 m BENCH (Phase 9).  The previous value aimed the
# camera at a bench top 1.015 m up; those benches were re-authored at 0.600 m
# because the UR5 cannot reach a target standing on a 1.0 m surface from the
# closest the base can park (/compute_ik returns NO_IK_SOLUTION beyond 0.45 m
# at that height).  Aiming the old pose at the new bench frames empty air.
#
# Camera at base_footprint (0.20, 0.15, 1.15) looking at (0.68, 0, 0.633):
# 0.72 m from the target cluster, tilted 45.8 deg down, and IMAGE LEVEL —
# the optical frame's image-right axis is horizontal to within 0.0 deg, which
# is the property that matters and is not the same thing as the camera's tilt.
# Solved with /compute_ik on camera_color_frame and checked by FK on the URDF.
LOOK_POSE = [-0.6104, -1.8772, 0.2168, 1.4008, 1.2730, -1.4930]


class LookPoseSender(Node):

    def __init__(self, action_name, cmd_vel_topic='/diff_drive_controller/cmd_vel_unstamped',
                 odom_topic='/diff_drive_controller/odom'):
        super().__init__('phase7_look_pose')
        self.client = ActionClient(self, FollowJointTrajectory, action_name)
        self.action_name = action_name
        # diff_drive_controller subscribes with SystemDefaultsQoS(), i.e.
        # BEST_EFFORT — a RELIABLE publisher is silently dropped.
        self.cmd_pub = self.create_publisher(
            Twist, cmd_vel_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.odom = None

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)

    def hold_station(self, seconds, tol_lin=0.01, tol_ang=0.02,
                     kp_lin=1.5, kp_ang=1.5, min_lin=0.05, min_ang=0.30):
        """
        Steer the base back to wherever it is now, for `seconds`.

        min_lin/min_ang exist because the skid-steer base has a hard stiction
        deadband — a tapering P command below roughly 0.27 rad/s simply does not
        move it — so corrections are floored rather than allowed to fade out.
        That trades a centimetre of limit-cycle chatter for not drifting away,
        which is the right trade when the camera has to stay pointed at a bench.
        """
        deadline = time.monotonic() + 10.0
        while self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            self.get_logger().error('no odometry — cannot hold station')
            return False

        target = self.odom
        self.get_logger().info(
            f'holding station at odom ({target[0]:+.3f}, {target[1]:+.3f}, '
            f'{target[2]:+.3f}) for {seconds:.0f}s')
        end = time.monotonic() + seconds
        worst = 0.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self.odom
            dx, dy = target[0] - x, target[1] - y
            # longitudinal error in the body frame; a diff drive cannot correct
            # lateral error directly, and yaw control takes care of heading.
            forward = math.cos(yaw) * dx + math.sin(yaw) * dy
            dyaw = math.atan2(math.sin(target[2] - yaw), math.cos(target[2] - yaw))
            worst = max(worst, math.hypot(dx, dy))

            cmd = Twist()
            if abs(forward) > tol_lin:
                cmd.linear.x = math.copysign(
                    min(max(kp_lin * abs(forward), min_lin), 0.20), forward)
            if abs(dyaw) > tol_ang:
                cmd.angular.z = math.copysign(
                    min(max(kp_ang * abs(dyaw), min_ang), 0.60), dyaw)
            self.cmd_pub.publish(cmd)

        self.cmd_pub.publish(Twist())
        self.get_logger().info(f'station hold done; worst excursion {worst * 100:.1f} cm')
        return True

    def send(self, waypoints, segment_time):
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(f'no action server on {self.action_name}')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance = Duration(sec=2)
        goal.trajectory.joint_names = ARM_JOINTS
        for i, positions in enumerate(waypoints, start=1):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            t = segment_time * i
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            goal.trajectory.points.append(point)

        self.get_logger().info(
            f'sending {len(waypoints)} waypoints, {segment_time:.1f}s per segment')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('trajectory goal rejected')
            return False

        result_future = handle.get_result_async()
        timeout = segment_time * len(waypoints) + 30.0
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().error('trajectory timed out')
            return False
        status = result_future.result().status
        ok = status == 4                                    # STATUS_SUCCEEDED
        self.get_logger().info(f'trajectory finished with status {status}'
                               f'{"" if ok else " (expected 4)"}')
        return ok


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--action', default='/arm_controller/follow_joint_trajectory')
    parser.add_argument('--segment-time', type=float, default=6.0,
                        help='seconds per waypoint; slower keeps the unbraked base still')
    parser.add_argument('--skip-lift', action='store_true',
                        help='go straight to the look pose — only safe if the arm is '
                             'already clear of the workbench')
    parser.add_argument('--hold', type=float, default=0.0,
                        help='after arriving, hold the base at its current odom pose for '
                             'this many seconds (0 = off). Kill home_hold first.')
    cli, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = LookPoseSender(cli.action)
    waypoints = [LOOK_POSE] if cli.skip_lift else [LIFT_POSE, LOOK_POSE]
    try:
        ok = node.send(waypoints, cli.segment_time)
        if ok and cli.hold > 0.0:
            ok = node.hold_station(cli.hold)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
