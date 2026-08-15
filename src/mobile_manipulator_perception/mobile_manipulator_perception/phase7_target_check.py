#!/usr/bin/env python3
"""
Phase 7 verification gate for object_target_frame.

phase7_target_check.py
─────────────────────────────────────────────────────────────────────────────

Answers the two questions the phase asks, with numbers instead of eyeballs:

  1. Is camera_color_optical_frame -> object_target_frame *stable*?
     Sampled over several seconds; reports the spread of the broadcast point.

  2. Is it *correct*?  Converts the broadcast point into world coordinates and
     compares it against the target's spawn pose parsed out of
     warehouse.world.

WHY THE WORLD CONVERSION IS BUILT THIS WAY
    world <- base_footprint comes from Gazebo ground truth (gz model -p), never
    from /odom: the skid-steer base slips and wheel odometry over-reports.  It
    is re-read on every sample because the base is not braked and creeps a
    centimetre at a time while the arm is extended.
    base_footprint <- camera_color_optical_frame comes from TF, which for this
    chain is pure forward kinematics off /joint_states, so it carries no
    odometry error.
    Composing those two with the perception node's camera-frame point isolates
    the perception error from base-localization error, which is the thing this
    phase is actually meant to verify.

Usage (Gazebo warehouse running, robot parked, perception node running):
    ros2 run mobile_manipulator_perception phase7_target_check \
        --ros-args -p use_sim_time:=true
"""
import argparse
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node

DEFAULT_MODEL = 'mobile_manipulator'


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def gazebo_model_pose(model=DEFAULT_MODEL, timeout=10.0):
    """Return the (translation, rotation) of a Gazebo model in the world frame."""
    try:
        out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                             capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    for line in reversed(out.stdout.strip().splitlines()):
        parts = line.split()
        if len(parts) == 6:
            try:
                v = [float(p) for p in parts]
            except ValueError:
                continue
            return np.array(v[:3]), rpy_to_matrix(*v[3:])
    return None


def world_targets(world_path, name_filter='target_'):
    """{model name: world xyz} for the pick targets declared in the world file."""
    targets = {}
    root = ET.parse(world_path).getroot()
    for model in root.iter('model'):
        name = model.get('name') or ''
        if not name.startswith(name_filter):
            continue
        pose = model.find('pose')
        if pose is None or not pose.text:
            continue
        vals = [float(v) for v in pose.text.split()]
        targets[name] = np.array(vals[:3])
    return targets


def default_world_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('mobile_manipulator_gazebo'),
                            'worlds', 'warehouse.world')
    except Exception:                                                   # noqa: BLE001
        return ''


class TargetChecker(Node):

    def __init__(self, camera_frame, target_frame, base_frame, max_age=0.3):
        super().__init__('phase7_target_check')
        self.camera_frame, self.target_frame, self.base_frame = (
            camera_frame, target_frame, base_frame)
        self.max_age = max_age
        self._last_stamp = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def _lookup(self, parent, child, require_fresh=False):
        """
        Return the latest parent->child transform, rejecting stale ones.

        tf2 keeps a 10 s buffer and lookup_transform(..., Time()) happily returns
        the newest entry no matter how old it is.  The perception node stops
        broadcasting the moment it loses the target, so without a freshness test
        a detection dropout silently freezes object_target_frame at its last
        value while the robot keeps moving — and composing that stale
        camera-frame point with a current base pose scatters the world estimate
        by a metre.

        Freshness is judged by the stamp ADVANCING between samples, not by
        comparing it against this node's clock.  The perception node runs YOLO
        inference on a single-threaded executor, so its /clock callback lags and
        its stamps trail real sim time by a variable amount; an absolute-age test
        rejects perfectly good transforms.  A stamp that has not moved since the
        previous sample means nothing new was broadcast, which is exactly the
        condition worth rejecting.  max_age stays as a loose backstop.
        """
        tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
        if require_fresh:
            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            if self._last_stamp is not None and stamp == self._last_stamp:
                raise tf2_ros.ExtrapolationException(
                    f'{parent} -> {child} has not been re-broadcast since the '
                    'previous sample')
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.max_age:
                raise tf2_ros.ExtrapolationException(
                    f'{parent} -> {child} is {age:.2f}s stale (> {self.max_age}s)')
            self._last_stamp = stamp
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(q.x, q.y, q.z, q.w)

    def sample(self):
        """One (point_in_camera, point_in_world) pair, or None."""
        p_cam, _ = self._lookup(self.camera_frame, self.target_frame, require_fresh=True)
        t_base_cam, r_base_cam = self._lookup(self.base_frame, self.camera_frame)
        gt = gazebo_model_pose()
        if gt is None:
            return p_cam, None
        t_world_base, r_world_base = gt
        p_base = r_base_cam @ p_cam + t_base_cam
        return p_cam, r_world_base @ p_base + t_world_base

    def spin(self, seconds):
        # Wall clock, not get_clock(): under use_sim_time the ROS clock reads 0
        # until the first /clock message and then jumps to however long the sim
        # has been up, so any deadline computed from it expires instantly.
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--target-frame', default='object_target_frame')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--world', default=default_world_path())
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--wait', type=float, default=60.0,
                        help='seconds to wait for the first transform')
    parser.add_argument('--max-error', type=float, default=0.05,
                        help='max world-frame distance to the nearest world target [m]')
    parser.add_argument('--max-jitter', type=float, default=0.05,
                        help='max peak-to-peak spread of the world-frame point [m]. '
                             '0.05 matches the phase gate\'s own "within a few cm" '
                             'tolerance; the residual is YOLO box-centre wander of a '
                             'few pixels, which at ~1.2 m is a few mm per pixel')
    parser.add_argument('--max-age', type=float, default=3.0,
                        help='backstop: reject object_target_frame older than this [s]')
    parser.add_argument('--min-samples', type=int, default=8,
                        help='minimum fresh samples required for a verdict')
    cli, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = TargetChecker(cli.camera_frame, cli.target_frame, cli.base_frame,
                         max_age=cli.max_age)
    log = node.get_logger()
    failures = []

    try:
        log.info(f'waiting up to {cli.wait:.0f}s for '
                 f'{cli.camera_frame} -> {cli.target_frame} ...')
        deadline = time.monotonic() + cli.wait           # wall clock — see spin()
        first = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                first = node.sample()
                break
            except tf2_ros.TransformException:
                continue
        if first is None:
            log.error(f'{cli.target_frame} never appeared — is the perception node '
                      'running, and does it see a target-class object?')
            return 1
        log.info('transform is live; sampling')

        cam_pts, world_pts, stale = [], [], 0
        n = max(1, int(cli.duration * cli.rate))
        for _ in range(n):
            node.spin(1.0 / cli.rate)
            try:
                p_cam, p_world = node.sample()
            except tf2_ros.TransformException as exc:
                stale += 1
                log.warn(f'sample dropped: {exc}')
                continue
            cam_pts.append(p_cam)
            if p_world is not None:
                world_pts.append(p_world)

        if stale:
            log.warn(f'{stale}/{n} samples dropped as stale or missing — the detector '
                     'is losing the target between frames')
        if len(cam_pts) < cli.min_samples:
            log.error(f'only {len(cam_pts)} fresh samples of {n} — the TF is not being '
                      'broadcast steadily')
            return 1

        cam = np.array(cam_pts)
        mean_cam, cam_spread = cam.mean(axis=0), cam.max(axis=0) - cam.min(axis=0)
        print()
        print('═══ Phase 7 — object_target_frame check ═══')
        print(f'samples                 : {len(cam_pts)} fresh of {n} over '
              f'{cli.duration:.0f}s ({len(world_pts)} with Gazebo ground truth, '
              f'{stale} dropped)')
        print(f'{cli.camera_frame} -> {cli.target_frame}   (what tf2_echo prints)')
        print(f'  mean position [m]     : '
              f'({mean_cam[0]:+.4f}, {mean_cam[1]:+.4f}, {mean_cam[2]:+.4f})')
        print(f'  peak-to-peak [m]      : '
              f'({cam_spread[0]:.4f}, {cam_spread[1]:.4f}, {cam_spread[2]:.4f})  '
              f'max {cam_spread.max():.4f}')
        print(f'  per-axis std dev [m]  : '
              f'({cam.std(axis=0)[0]:.4f}, {cam.std(axis=0)[1]:.4f}, {cam.std(axis=0)[2]:.4f})')
        print('  (this frame is attached to a camera on a base that creeps while the arm '
              'is extended,\n   so some drift here is the robot moving, not the detector — '
              'the world frame below\n   is the invariant one and is what PASS/FAIL uses.)')

        if not world_pts:
            failures.append('no Gazebo ground truth — could not check accuracy '
                            '(is gzserver running?)')
        else:
            world = np.array(world_pts)
            mean_world = world.mean(axis=0)
            world_spread = world.max(axis=0) - world.min(axis=0)
            print('\nback-projected point in world frame   (the target does not move)')
            print(f'  mean position [m]     : '
                  f'({mean_world[0]:+.4f}, {mean_world[1]:+.4f}, {mean_world[2]:+.4f})')
            print(f'  peak-to-peak [m]      : '
                  f'({world_spread[0]:.4f}, {world_spread[1]:.4f}, {world_spread[2]:.4f})  '
                  f'max {world_spread.max():.4f}')
            if world_spread.max() > cli.max_jitter:
                failures.append(f'world-frame jitter {world_spread.max():.4f} m '
                                f'> {cli.max_jitter} m')

            targets = world_targets(cli.world) if cli.world and os.path.exists(cli.world) else {}
            if not targets:
                failures.append(f'no target models parsed from {cli.world!r}')
            else:
                print(f'\ntargets declared in {os.path.basename(cli.world)}')
                ranked = sorted(((np.linalg.norm(mean_world - p), name, p)
                                 for name, p in targets.items()))
                for dist, name, p in ranked:
                    print(f'  {name:20s} ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})   '
                          f'error {dist:.4f} m')
                best_err, best_name, best_p = ranked[0]
                delta = mean_world - best_p
                print(f'\nnearest target          : {best_name}')
                print(f'  error [m]             : {best_err:.4f}  '
                      f'(dx {delta[0]:+.4f}, dy {delta[1]:+.4f}, dz {delta[2]:+.4f})')
                if best_err > cli.max_error:
                    failures.append(f'position error {best_err:.4f} m > {cli.max_error} m')

        print()
        if failures:
            print('RESULT: FAIL')
            for f in failures:
                print(f'  - {f}')
            return 1
        print('RESULT: PASS — transform is stable and matches the world target')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
