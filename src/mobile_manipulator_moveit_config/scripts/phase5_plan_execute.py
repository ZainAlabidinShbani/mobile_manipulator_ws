#!/usr/bin/env python3
# phase5_plan_execute.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 verification gate, driven from the command line instead of by hand in
# the RViz MotionPlanning panel.  It performs exactly the sequence the panel
# performs when you drag the interactive marker and press Plan then Execute:
#
#   0. /apply_planning_scene   optionally insert the pick workbench as a box
#                             collision object, so the collision check below is
#                             about the world and not only about self-collision
#   1. /compute_ik            solve IK for a Cartesian pose of arm_tool0
#                             (RViz does this every time the marker moves)
#   2. /check_state_validity   confirm the goal state is collision free
#                             (this is what colours the goal state green/red)
#   3. /move_action            plan with OMPL and execute on the real controller
#   4. /check_state_validity   re-checked on every waypoint of the returned
#                             trajectory — the "collision-free green preview"
#   5. /joint_states           sampled before and after execution to prove the
#                             physical (simulated) arm actually moved
#
# Usage (robot spawned at world x = 3.2, so odom x = world x - 3.2):
#   ros2 run mobile_manipulator_moveit_config phase5_plan_execute.py \
#       --x 0.65 --y 0.11 --z 1.10 --rpy 180 20 0 --use-sim-time \
#       --workbench 1.30 0.0 0.515 1.50 0.80 1.03
# ─────────────────────────────────────────────────────────────────────────────
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningOptions,
    PlanningScene,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

GROUP = 'ur5_arm'
TIP_LINK = 'arm_tool0'
ARM_JOINTS = [
    'arm_shoulder_pan_joint',
    'arm_shoulder_lift_joint',
    'arm_elbow_joint',
    'arm_wrist_1_joint',
    'arm_wrist_2_joint',
    'arm_wrist_3_joint',
]

ERROR_NAMES = {
    getattr(MoveItErrorCodes, n): n
    for n in dir(MoveItErrorCodes) if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)
}


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Phase5Check(Node):

    def __init__(self, frame):
        super().__init__('phase5_plan_execute')
        self.frame = frame
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.fk_cli = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.sv_cli = self.create_client(GetStateValidity, '/check_state_validity')
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.move_cli = ActionClient(self, MoveGroup, '/move_action')

    def _on_js(self, msg):
        self.joint_state = msg

    # ── helpers ────────────────────────────────────────────────────────────
    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout=60.0):
        for cli, name in ((self.ik_cli, '/compute_ik'), (self.sv_cli, '/check_state_validity')):
            if not cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'service {name} unavailable')
                return False
        if not self.move_cli.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('action /move_action unavailable')
            return False
        end = time.time() + timeout
        while self.joint_state is None and time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            self.get_logger().error('no /joint_states received')
            return False
        return True

    def arm_positions(self):
        js = self.joint_state
        return [js.position[js.name.index(j)] for j in ARM_JOINTS]

    def call(self, client, req, timeout=15.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    # ── step 0: insert the workbench as a world collision object ───────────
    def add_workbench(self, cx, cy, cz, sx, sy, sz, frame=None):
        if not self.scene_cli.wait_for_service(timeout_sec=20.0):
            print('  /apply_planning_scene unavailable')
            return False
        obj = CollisionObject()
        obj.header.frame_id = frame or self.frame
        obj.id = 'pick_workbench'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cx, cy, cz
        pose.orientation.w = 1.0
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        req.scene = scene
        res = self.call(self.scene_cli, req, timeout=20.0)
        okay = bool(res and res.success)
        print(f'  workbench box added to planning scene: {okay} '
              f'(centre {cx},{cy},{cz}  size {sx}x{sy}x{sz} in {obj.header.frame_id})')
        self.spin_for(1.0)
        return okay

    # ── arm-only RobotState diff (never echo /joint_states back, see above) ─
    def arm_state(self, positions):
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = list(ARM_JOINTS)
        rs.joint_state.position = list(positions)
        rs.is_diff = True
        return rs

    # ── FK: where does a given configuration put arm_tool0? ────────────────
    def compute_fk(self, positions, frame):
        if not self.fk_cli.wait_for_service(timeout_sec=20.0):
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = frame
        req.fk_link_names = [TIP_LINK]
        req.robot_state = self.arm_state(positions)
        res = self.call(self.fk_cli, req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        return res.pose_stamped[0]

    # ── step 1: IK ─────────────────────────────────────────────────────────
    def solve_ik_pose(self, pose, seed=None, avoid_collisions=True, label=''):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = TIP_LINK
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout.sec = 2
        if seed is not None:
            req.ik_request.robot_state = self.arm_state(seed)
        res = self.call(self.ik_cli, req)
        if res is None:
            print(f'  IK {label}: service call failed')
            return None
        code = res.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            sol = res.solution.joint_state
            q_arm = [sol.position[sol.name.index(j)] for j in ARM_JOINTS]
            print(f'  IK {label}: SUCCESS -> {[round(v, 4) for v in q_arm]}')
            return q_arm
        print(f'  IK {label}: {ERROR_NAMES.get(code, code)}')
        return None

    def solve_ik(self, x, y, z, rpy_candidates):
        for rpy in rpy_candidates:
            pose = PoseStamped()
            pose.header.frame_id = self.frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = quat_from_rpy(*rpy)
            deg = tuple(round(math.degrees(a)) for a in rpy)
            q_arm = self.solve_ik_pose(pose, label=f'rpy(deg)={deg}')
            if q_arm is not None:
                return q_arm, pose
        return None, None

    # ── step 2/4: collision check ──────────────────────────────────────────
    def state_valid(self, positions):
        req = GetStateValidity.Request()
        req.group_name = GROUP
        # Send ONLY the arm joints, as a diff on move_group's current state.
        # Never echo /joint_states back verbatim: under gazebo_ros2_control the
        # Robotiq mimic joints are published as "<joint>_mimic", names that do
        # not exist in the URDF, and move_group aborts with an uncaught
        # moveit::Exception ("Variable ... is not known to model") when it is
        # asked to build a RobotState out of them.
        req.robot_state = self.arm_state(positions)
        res = self.call(self.sv_cli, req)
        if res is None:
            return None, ['<service call failed>']
        contacts = [f'{c.contact_body_1} <-> {c.contact_body_2}' for c in res.contacts]
        return res.valid, contacts

    # ── step 3: plan + execute ─────────────────────────────────────────────
    def plan_and_execute(self, goal_positions, plan_only, planning_time=10.0):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = planning_time
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = self.frame
        req.workspace_parameters.min_corner.x = -5.0
        req.workspace_parameters.min_corner.y = -5.0
        req.workspace_parameters.min_corner.z = -5.0
        req.workspace_parameters.max_corner.x = 5.0
        req.workspace_parameters.max_corner.y = 5.0
        req.workspace_parameters.max_corner.z = 5.0

        c = Constraints()
        for j, v in zip(ARM_JOINTS, goal_positions):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = v
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False

        send = self.move_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('  MoveGroup goal REJECTED')
            return None
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=180.0)
        wrapper = result_fut.result()
        if wrapper is None:
            print('  MoveGroup produced no result (timeout)')
            return None
        return wrapper.result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--z', type=float)
    ap.add_argument('--frame', default='odom')
    ap.add_argument('--rpy', nargs=3, type=float, metavar=('R', 'P', 'Y'),
                    help='goal orientation of arm_tool0 in degrees; if omitted a '
                         'set of downward-pointing candidates is tried in turn')
    ap.add_argument('--goal-config', nargs=6, type=float,
                    help='6 UR5 joint values that define the target instead of --x/--z. '
                         'The Cartesian pose is obtained from /compute_fk and then fed '
                         'back through /compute_ik, so the IK step is still exercised '
                         'but on a pose that is guaranteed to lie in the workspace.')
    ap.add_argument('--workbench', nargs=6, type=float,
                    metavar=('CX', 'CY', 'CZ', 'SX', 'SY', 'SZ'),
                    help='add a box collision object (centre + size)')
    ap.add_argument('--workbench-frame', default='odom',
                    help='frame the --workbench box is expressed in (default odom, '
                         'i.e. world-fixed while the base does not drive)')
    ap.add_argument('--use-sim-time', action='store_true')
    ap.add_argument('--plan-only', action='store_true')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=sys.argv[:1] + ros_args)
    node = Phase5Check(args.frame)
    if args.use_sim_time:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    ok = True
    try:
        print('== waiting for move_group, /joint_states ==')
        if not node.wait_ready():
            return 2
        node.spin_for(1.0)
        start = node.arm_positions()
        print(f'  start arm joints: {[round(v, 4) for v in start]}')

        if args.workbench:
            print('== step 0: add the pick workbench to the planning scene ==')
            node.add_workbench(*args.workbench, frame=args.workbench_frame)

        if args.goal_config:
            print(f'== step 1: FK of the target configuration, then IK back ==')
            pose = node.compute_fk(args.goal_config, args.frame)
            if pose is None:
                print('  /compute_fk failed')
                return 3
            p, o = pose.pose.position, pose.pose.orientation
            # tool0 z-axis is R[:,2]; its world-z component is 1 - 2(x^2 + y^2).
            # Angle away from "straight down" is acos(-that).
            tilt = math.degrees(math.acos(max(-1.0, min(1.0,
                2.0 * (o.x * o.x + o.y * o.y) - 1.0))))
            print(f'  arm_tool0 target pose in {pose.header.frame_id}: '
                  f'({p.x:.4f}, {p.y:.4f}, {p.z:.4f})  '
                  f'quat=({o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f})  '
                  f'tool axis {tilt:.1f} deg from straight down')
            goal_q = node.solve_ik_pose(pose, seed=args.goal_config,
                                        label='(collision-aware, from /compute_fk pose)')
            if goal_q is None:
                print('  RESULT: pose is NOT reachable -- no collision-free IK solution')
                return 3
            run_ik_sweep = False
        else:
            run_ik_sweep = True

        if run_ik_sweep:
            if args.x is None or args.z is None:
                print('  need --x/--z or --goal-config')
                return 2
            if args.rpy:
                cands = [tuple(math.radians(a) for a in args.rpy)]
            else:
                # Gripper pointing down, plus progressively tilted fallbacks: at
                # full extension over the table a perfectly vertical tool axis is
                # outside the UR5's dexterous workspace, so a tilt is needed.
                cands = [(math.pi, 0.0, 0.0),
                         (math.pi, math.radians(10), 0.0),
                         (math.pi, math.radians(20), 0.0),
                         (math.pi, math.radians(30), 0.0),
                         (math.pi, math.radians(40), 0.0)]
            print(f'== step 1: IK for {TIP_LINK} at '
                  f'({args.x}, {args.y}, {args.z}) in {args.frame} ==')
            goal_q, _ = node.solve_ik(args.x, args.y, args.z, cands)
            if goal_q is None:
                print('  RESULT: pose is NOT reachable -- no IK solution')
                return 3

        print('== step 2: goal-state collision check ==')
        valid, contacts = node.state_valid(goal_q)
        print(f'  goal state valid (collision free): {valid}')
        for c in contacts:
            print(f'    contact: {c}')
        if not valid:
            ok = False

        print('== step 3: plan (OMPL) + execute ==')
        result = node.plan_and_execute(goal_q, plan_only=args.plan_only)
        if result is None:
            return 4
        code = result.error_code.val
        print(f'  move_group error_code: {ERROR_NAMES.get(code, code)}')
        traj = result.planned_trajectory.joint_trajectory
        print(f'  planned trajectory: {len(traj.points)} points, '
              f'{traj.points[-1].time_from_start.sec}.'
              f'{traj.points[-1].time_from_start.nanosec // 10**8}s'
              if traj.points else '  planned trajectory: EMPTY')
        if code != MoveItErrorCodes.SUCCESS:
            ok = False

        print('== step 4: collision check on every trajectory waypoint ==')
        bad = 0
        idx = [traj.joint_names.index(j) for j in ARM_JOINTS]
        for i, pt in enumerate(traj.points):
            v, cs = node.state_valid([pt.positions[k] for k in idx])
            if not v:
                bad += 1
                print(f'    waypoint {i} IN COLLISION: {cs}')
        print(f'  {len(traj.points) - bad}/{len(traj.points)} waypoints collision free')
        if bad:
            ok = False

        print('== step 5: did the simulated arm actually move? ==')
        node.spin_for(2.0)
        end = node.arm_positions()
        delta = [abs(a - b) for a, b in zip(end, start)]
        err = [abs(a - b) for a, b in zip(end, goal_q)]
        print(f'  end   arm joints: {[round(v, 4) for v in end]}')
        print(f'  goal  arm joints: {[round(v, 4) for v in goal_q]}')
        print(f'  |end-start| max = {max(delta):.4f} rad')
        print(f'  |end-goal|  max = {max(err):.4f} rad')
        if args.plan_only:
            print('  (plan-only mode: no motion expected)')
        else:
            moved = max(delta) > 0.05
            tracked = max(err) < 0.05
            print(f'  ARM MOVED: {moved}    REACHED GOAL: {tracked}')
            ok = ok and moved and tracked

        print(f'\nPHASE 5 GATE: {"PASS" if ok else "FAIL"}')
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
