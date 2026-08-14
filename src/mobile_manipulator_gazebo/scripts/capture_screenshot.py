#!/usr/bin/env python3
# capture_screenshot.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 4 — grab one frame from the world-fixed screenshot camera defined in
# warehouse.world and write it to a PNG.  This is what satisfies the phase
# gate "confirm via a screenshot that the robot is standing stably 30 seconds
# after spawn", and it works headlessly: gzclient never has to run.
#
#   ros2 run mobile_manipulator_gazebo capture_screenshot.py \
#       --topic /phase4_camera/image_raw --out /tmp/phase4_home_pose.png
#
# NOTE on the topic name: gazebo_ros_camera derives it from the plugin's
# <camera_name> element ("<camera_name>/image_raw").  A <ros><remapping>
# keyed on the bare name "image_raw" does NOT match and is silently dropped,
# so /phase4_camera/image_raw is the real topic regardless of any remapping.
#
# The publisher uses RELIABLE reliability, so this subscribes with the
# default (RELIABLE) QoS — a BEST_EFFORT sensor-data profile would still be
# compatible, but matching the publisher keeps the first frame from being
# dropped on a slow start-up.
# ─────────────────────────────────────────────────────────────────────────────
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ScreenshotGrabber(Node):
    def __init__(self, topic):
        super().__init__('phase4_screenshot_grabber')
        self.msg = None
        self.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg):
        self.msg = msg


def to_bgr(msg):
    """Convert a sensor_msgs/Image to an OpenCV BGR array."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'bgr8':
        return arr
    raise ValueError(f'unsupported encoding {msg.encoding!r} (expected rgb8/bgr8)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/phase4_camera/image_raw')
    parser.add_argument('--out', default='/tmp/phase4_home_pose.png')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='seconds to wait for the first frame')
    parser.add_argument('--settle', type=float, default=0.0,
                        help='seconds to keep receiving before saving, so the '
                             'saved frame is not the very first (possibly '
                             'half-rendered) one')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = ScreenshotGrabber(args.topic)

    deadline = time.time() + args.timeout
    while time.time() < deadline and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.msg is None:
        node.get_logger().error(
            f'no frame on {args.topic} within {args.timeout}s — check '
            f'"ros2 topic list | grep image_raw" for the real topic name')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    settle_until = time.time() + args.settle
    while time.time() < settle_until:
        rclpy.spin_once(node, timeout_sec=0.2)

    msg = node.msg
    cv2.imwrite(args.out, to_bgr(msg))
    node.get_logger().info(
        f'saved {msg.width}x{msg.height} {msg.encoding} frame '
        f'(frame_id={msg.header.frame_id!r}) to {args.out}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
