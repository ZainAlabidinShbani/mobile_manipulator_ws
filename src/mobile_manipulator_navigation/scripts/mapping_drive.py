#!/usr/bin/env python3
"""mapping_drive.py — Phase 6 scripted mapping route.

Drives the mobile base along a fixed loop through the warehouse while
slam_toolbox is mapping, then exits so the map can be saved.

Waypoints are tracked in the MAP frame via TF (map → base_footprint) so the
route self-corrects as slam_toolbox fixes odometry drift — the skid-steer
base slips badly during in-place spins and raw odom waypoints would walk
into the racks on the far side of the loop.  Falls back to odom-frame
tracking until the first map transform arrives.

Safety: aborts (exit code 2) if the forward ±30° scan sector drops below
the stop distance, so a bad route stops instead of grinding into a wall.

The route deliberately never enters the 0.93 m jersey-barrier gate at
x = 1.6 (too tight for open-loop driving) and never passes under the pick
workbench — it loops around the barrier island: west → north → east side
of the pick table → back the same way.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf2_ros

# (x, y) waypoints, or ("spin", radians) for an in-place rotation.
ROUTE = [
    (-3.0, 0.0),
    (-3.0, 6.5),
    (6.2, 6.5),
    (6.2, 0.3),
    ("spin", 2 * math.pi),
    (6.2, 6.5),
    (-3.0, 6.5),
    (-3.0, 0.0),
    (-1.2, 0.0),
]

CRUISE_V = 0.35        # m/s
SLOW_V = 0.15          # m/s inside approach radius
APPROACH_R = 0.6       # m
REACHED_R = 0.18       # m
SPIN_W = 0.6           # rad/s
# Skid-steer stiction: an in-place yaw command below ~0.25 rad/s never breaks
# the wheels' lateral grip and the base simply does not move (measured:
# with wheel_separation_multiplier 1.37 a command of 0.40 rad/s yields 0.285
# rad/s of real body yaw, and small commands yield nothing at all). Rolling
# turns are unaffected, so this floor applies only to the rotate-in-place
# branch.
SPIN_MIN = 0.45        # rad/s
HEAD_K = 1.4           # heading P gain
MAX_W = 0.6            # rad/s while driving
# Rotating in place is the expensive manoeuvre on this base: the wheels slide
# laterally and the body walks ~0.12 m per radian turned, which odometry cannot
# see and the scan matcher struggles to correct.  Rolling turns track almost
# perfectly (0.30/0.30 commanded -> 0.285 m/s, 0.294 rad/s measured), so only
# turn in place when the heading is badly wrong, then finish the turn rolling.
HEAD_ALIGN = 0.9       # rad — rotate in place only above this heading error
STOP_RANGE = 0.35      # m — abort if forward sector closer than this


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class MappingDrive(Node):
    def __init__(self):
        super().__init__("mapping_drive")
        self.pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
        best_effort = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._odom_cb, best_effort)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, best_effort)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_pose = None          # (x, y, yaw) odom frame
        self.front_min = float("inf")
        self.route_i = 0
        self.spin_accum = 0.0
        self.last_yaw = None
        self.used_map_tf = False
        self.done = False
        self.failed = False
        self.timer = self.create_timer(0.1, self._step)

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y,
                          yaw_from_quat(p.orientation))

    def _scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        # ±30° sector around straight ahead (angle 0)
        i0 = int((-0.524 - msg.angle_min) / msg.angle_increment)
        i1 = int((0.524 - msg.angle_min) / msg.angle_increment)
        sector = [r for r in msg.ranges[max(0, i0):min(n, i1)]
                  if msg.range_min < r < msg.range_max]
        self.front_min = min(sector) if sector else float("inf")

    def _pose(self):
        """Prefer map-frame pose from TF; fall back to raw odom."""
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            self.used_map_tf = True
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    yaw_from_quat(q))
        except Exception:
            return self.odom_pose

    def _step(self):
        if self.done or self.failed:
            self.pub.publish(Twist())
            return
        pose = self._pose()
        if pose is None:
            return
        x, y, yaw = pose

        step = ROUTE[self.route_i]
        cmd = Twist()

        if isinstance(step, tuple) and step[0] == "spin":
            if self.last_yaw is not None:
                self.spin_accum += abs(ang_norm(yaw - self.last_yaw))
            self.last_yaw = yaw
            if self.spin_accum >= step[1]:
                self._next_step(f"spin {math.degrees(step[1]):.0f}° complete")
            else:
                cmd.angular.z = SPIN_W
            self.pub.publish(cmd)
            return

        tx, ty = step
        dist = math.hypot(tx - x, ty - y)
        if dist < REACHED_R:
            self._next_step(f"waypoint ({tx:.1f}, {ty:.1f}) reached")
            self.pub.publish(Twist())
            return

        bearing = math.atan2(ty - y, tx - x)
        err = ang_norm(bearing - yaw)
        if abs(err) > HEAD_ALIGN:
            w = HEAD_K * err
            if abs(w) < SPIN_MIN:
                w = math.copysign(SPIN_MIN, w)
            cmd.angular.z = max(-SPIN_W, min(SPIN_W, w))
        else:
            if self.front_min < STOP_RANGE:
                self.get_logger().error(
                    f"ABORT: obstacle at {self.front_min:.2f} m in front sector "
                    f"(pose {x:.2f}, {y:.2f})")
                self.failed = True
                self.pub.publish(Twist())
                return
            v = SLOW_V if dist < APPROACH_R else CRUISE_V
            cmd.linear.x = v * max(0.3, math.cos(err))
            cmd.angular.z = max(-MAX_W, min(MAX_W, HEAD_K * err))
        self.pub.publish(cmd)

    def _next_step(self, msg):
        src = "map-TF" if self.used_map_tf else "odom"
        self.get_logger().info(f"[{self.route_i + 1}/{len(ROUTE)}] {msg} ({src})")
        self.route_i += 1
        self.spin_accum = 0.0
        self.last_yaw = None
        if self.route_i >= len(ROUTE):
            self.get_logger().info("Route complete — holding zero velocity")
            self.done = True


def main():
    rclpy.init()
    node = MappingDrive()
    try:
        while rclpy.ok() and not node.done and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.5)
        # hold still briefly so diff_drive latches zero
        end = node.get_clock().now() + rclpy.duration.Duration(seconds=1.0)
        while rclpy.ok() and node.get_clock().now() < end:
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    failed = node.failed
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(2 if failed else 0)


if __name__ == "__main__":
    main()
