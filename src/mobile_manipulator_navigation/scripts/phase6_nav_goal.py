#!/usr/bin/env python3
"""phase6_nav_goal.py — Phase 6 gate: one hardcoded navigation goal.

Publishes a single PoseStamped on /goal_pose (the topic nav2_bt_navigator
subscribes to), then watches the run and reports the three things the phase
gate asks for:

  1. that a global plan was produced  (first /plan message: pose count + span)
  2. that the robot reached the goal  (final pose error, ground truth)
  3. that it hit nothing on the way   (minimum clearance between the robot
     footprint and every obstacle in warehouse.world, along the driven trace)

Pose truth comes from Gazebo (`gz model -m mobile_manipulator -p`), never from
wheel odometry: the skid-steer base slips and /odom is only good to ~10 % in
yaw.  The AMCL estimate (TF map -> base_footprint) is reported alongside it so
the localization error is visible too.

Obstacle footprints are parsed out of warehouse.world itself, so the clearance
check cannot drift out of sync with the world.

Usage (Gazebo + nav2_bringup.launch.py already running):
  ros2 run mobile_manipulator_navigation phase6_nav_goal.py \
      --goal 2.9 0.0 0.0 --timeout 180
"""
import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros

# ── robot footprint (matches nav2_params.yaml: chassis + front lidar puck) ────
FOOTPRINT = [(0.58, 0.35), (0.58, -0.35), (-0.51, -0.35), (-0.51, 0.35)]

# ── model -> planar collision footprint ──────────────────────────────────────
# rect: (size_x, size_y) in the model's own frame; circle: radius.
FOOTPRINTS = {
    'bookshelf': ('rect', 0.90, 0.45),
    'euro_pallet': ('rect', 1.20, 0.80),
    'jersey_barrier': ('rect', 4.07, 0.81),
    'table': ('rect', 1.50, 0.80),
    'cabinet': ('rect', 0.50, 0.50),
    'cardboard_box': ('rect', 0.50, 0.40),
    'brick_box_3x1x3': ('rect', 3.00, 1.00),
    'construction_barrel': ('circle', 0.36),
    'construction_cone': ('circle', 0.25),
}
# hand-authored <model> walls in warehouse.world (not <include>s)
WALLS = [
    ('north_wall', 0.0, -14.2, 0.0, 36.0, 0.3),
    ('south_wall', 0.0, 14.2, 0.0, 36.0, 0.3),
    ('west_wall', -18.15, 0.0, 0.0, 0.3, 29.0),
    ('east_wall', 18.15, 0.0, 0.0, 0.3, 29.0),
]

INCLUDE_RE = re.compile(
    r'<include>\s*<uri>model://([^<]+)</uri>\s*<name>([^<]+)</name>\s*'
    r'<pose>([^<]+)</pose>', re.S)


def load_obstacles(world_path):
    """Return [(name, kind, params...)] parsed from the world file."""
    obstacles = []
    for name, x, y, yaw, sx, sy in WALLS:
        obstacles.append((name, 'rect', x, y, yaw, sx, sy))
    with open(world_path) as fh:
        world = fh.read()
    for model, name, pose in INCLUDE_RE.findall(world):
        vals = [float(v) for v in pose.split()]
        x, y, yaw = vals[0], vals[1], vals[5]
        spec = FOOTPRINTS.get(model)
        if spec is None:
            continue
        if spec[0] == 'rect':
            obstacles.append((name, 'rect', x, y, yaw, spec[1], spec[2]))
        else:
            obstacles.append((name, 'circle', x, y, yaw, spec[1], 0.0))
    return obstacles


# ── geometry ─────────────────────────────────────────────────────────────────
def rect_corners(cx, cy, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for dx, dy in ((sx / 2, sy / 2), (sx / 2, -sy / 2),
                   (-sx / 2, -sy / 2), (-sx / 2, sy / 2)):
        out.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    return out


def robot_corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + dx * c - dy * s, y + dx * s + dy * c) for dx, dy in FOOTPRINT]


def seg_dist(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def polys_overlap(p, q):
    """Separating-axis test for two convex polygons."""
    for poly in (p, q):
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            nx, ny = -(by - ay), bx - ax
            pa = [nx * vx + ny * vy for vx, vy in p]
            qa = [nx * vx + ny * vy for vx, vy in q]
            if max(pa) < min(qa) or max(qa) < min(pa):
                return False
    return True


def poly_dist(p, q):
    if polys_overlap(p, q):
        return 0.0
    best = float('inf')
    for a, b in ((p, q), (q, p)):
        n = len(b)
        for v in a:
            for i in range(n):
                best = min(best, seg_dist(v, b[i], b[(i + 1) % n]))
    return best


def point_poly_dist(pt, poly):
    if polys_overlap([pt, pt, pt], poly):
        return 0.0
    n = len(poly)
    return min(seg_dist(pt, poly[i], poly[(i + 1) % n]) for i in range(n))


def clearance(pose, obstacles):
    """Smallest gap between the robot footprint at `pose` and any obstacle."""
    x, y, yaw = pose
    rc = robot_corners(x, y, yaw)
    worst = (float('inf'), None)
    for name, kind, ox, oy, oyaw, a, b in obstacles:
        if math.hypot(ox - x, oy - y) > 12.0:      # far away, skip the maths
            continue
        if kind == 'rect':
            d = poly_dist(rc, rect_corners(ox, oy, oyaw, a, b))
        else:
            d = max(0.0, point_poly_dist((ox, oy), rc) - a)
        if d < worst[0]:
            worst = (d, name)
    return worst


# ── ground truth ─────────────────────────────────────────────────────────────
def gz_pose(model='mobile_manipulator'):
    try:
        out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                             capture_output=True, text=True, timeout=15).stdout
        vals = [float(v) for v in out.strip().split('\n')[-1].split()]
        return vals[0], vals[1], vals[5]
    except Exception:
        return None


def ang_norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class GateRunner(Node):
    def __init__(self, goal, world):
        super().__init__('phase6_nav_goal')
        self.goal = goal
        self.obstacles = load_obstacles(world)
        self.get_logger().info(f'loaded {len(self.obstacles)} obstacle footprints '
                               f'from {os.path.basename(world)}')

        latching = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', latching)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.first_plan = None
        self.plan_count = 0
        self.trace = []
        self.min_clear = (float('inf'), None, None)
        self._stop = False
        self.sampler = threading.Thread(target=self._sample_loop, daemon=True)

    def _plan_cb(self, msg):
        self.plan_count += 1
        if self.first_plan is None and msg.poses:
            self.first_plan = msg

    def _sample_loop(self):
        while not self._stop:
            p = gz_pose()
            if p is not None:
                self.trace.append((time.time(), p))
                d, who = clearance(p, self.obstacles)
                if d < self.min_clear[0]:
                    self.min_clear = (d, who, p)
            time.sleep(0.15)

    def amcl_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                rclpy.time.Time())
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:
            return None

    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        msg.pose.orientation.z = math.sin(self.goal[2] / 2)
        msg.pose.orientation.w = math.cos(self.goal[2] / 2)
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'goal published: x={self.goal[0]:.2f} y={self.goal[1]:.2f} '
            f'yaw={self.goal[2]:.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', nargs=3, type=float, default=[2.9, 0.0, 0.0],
                    metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--world', default=None)
    ap.add_argument('--settle-window', type=float, default=5.0)
    args = ap.parse_args()

    world = args.world
    if world is None:
        from ament_index_python.packages import get_package_share_directory
        world = os.path.join(get_package_share_directory('mobile_manipulator_gazebo'),
                             'worlds', 'warehouse.world')

    rclpy.init()
    node = GateRunner(args.goal, world)
    node.sampler.start()

    # let TF / the plan subscription connect, and grab the start pose
    t_end = time.time() + 5.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    start_gt = gz_pose()
    node.send_goal()
    t0 = time.time()

    arrived = False
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        gt = node.trace[-1][1] if node.trace else None
        if gt is None:
            continue
        dist = math.hypot(gt[0] - args.goal[0], gt[1] - args.goal[1])
        recent = [p for t, p in node.trace if t > time.time() - args.settle_window]
        if len(recent) >= 4 and time.time() - t0 > 8.0:
            moved = max(math.hypot(p[0] - recent[0][0], p[1] - recent[0][1])
                        for p in recent)
            turned = max(abs(ang_norm(p[2] - recent[0][2])) for p in recent)
            if dist < 1.0 and moved < 0.02 and turned < 0.02:
                arrived = True
                break

    node._stop = True
    time.sleep(0.6)
    gt = node.trace[-1][1] if node.trace else (float('nan'),) * 3
    est = node.amcl_pose()
    dxy = math.hypot(gt[0] - args.goal[0], gt[1] - args.goal[1])
    dyaw = abs(ang_norm(gt[2] - args.goal[2]))
    driven = sum(math.hypot(node.trace[i][1][0] - node.trace[i - 1][1][0],
                            node.trace[i][1][1] - node.trace[i - 1][1][1])
                 for i in range(1, len(node.trace)))

    print('\n' + '=' * 68)
    print('PHASE 6 GATE — hardcoded nav goal')
    print('=' * 68)
    print(f'start pose (gazebo) : x={start_gt[0]:+.3f} y={start_gt[1]:+.3f} '
          f'yaw={start_gt[2]:+.3f}')
    print(f'goal                : x={args.goal[0]:+.3f} y={args.goal[1]:+.3f} '
          f'yaw={args.goal[2]:+.3f}')
    if node.first_plan is None:
        print('global plan         : NONE RECEIVED on /plan')
    else:
        p = node.first_plan.poses
        span = sum(math.hypot(p[i].pose.position.x - p[i - 1].pose.position.x,
                              p[i].pose.position.y - p[i - 1].pose.position.y)
                   for i in range(1, len(p)))
        print(f'global plan         : {len(p)} poses, {span:.2f} m long, '
              f'frame "{node.first_plan.header.frame_id}" '
              f'({node.plan_count} plan messages total)')
        print(f'  first pose        : ({p[0].pose.position.x:+.2f}, '
              f'{p[0].pose.position.y:+.2f})')
        print(f'  last pose         : ({p[-1].pose.position.x:+.2f}, '
              f'{p[-1].pose.position.y:+.2f})')
    print(f'final pose (gazebo) : x={gt[0]:+.3f} y={gt[1]:+.3f} yaw={gt[2]:+.3f}')
    if est:
        print(f'final pose (amcl)   : x={est[0]:+.3f} y={est[1]:+.3f} yaw={est[2]:+.3f}'
              f'   [localization error {math.hypot(est[0]-gt[0], est[1]-gt[1]):.3f} m]')
    print(f'FINAL POSE ERROR    : {dxy:.3f} m, {dyaw:.3f} rad '
          f'({math.degrees(dyaw):.1f} deg)')
    print(f'  within nav2 default tolerance (0.25 m / 0.25 rad): '
          f'{"YES" if dxy <= 0.25 and dyaw <= 0.25 else "NO"}')
    print(f'path driven         : {driven:.2f} m over {len(node.trace)} samples'
          f'{"" if arrived else "  (TIMED OUT, robot never settled)"}')
    d, who, where = node.min_clear
    print(f'min clearance       : {d:.3f} m to "{who}"'
          + (f' at ({where[0]:+.2f}, {where[1]:+.2f})' if where else ''))
    print(f'  collision-free    : {"YES" if d > 0.0 else "NO — FOOTPRINT OVERLAP"}')
    print('=' * 68)

    node.destroy_node()
    rclpy.shutdown()
    ok = (node.first_plan is not None and dxy <= 0.25 and dyaw <= 0.25 and d > 0.0)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
