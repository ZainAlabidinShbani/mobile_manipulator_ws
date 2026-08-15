#!/usr/bin/env python3
# warehouse_orchestrator.py
# ─────────────────────────────────────────────────────────────────────────────
# Explicit state machine driving one warehouse pick-and-place cycle:
#
#   HOME → NAV_TO_PICK → PERCEIVE → APPROACH_ARM → GRASP
#        → NAV_TO_DROP → PLACE_ARM → RELEASE → RETURN_HOME → HOME
#
# Phase 8 wrote and gated the transition logic against stubs (dry_run:=true).
# Phase 9 wired it to the real stack; everything the stubs used to stand in
# for now lives here:
#
#   Nav2      /navigate_to_pose          coarse drive (NAV_TO_PICK/DROP, HOME)
#             + a short docking leg on the map frame, because Nav2's 0.25 m
#               goal tolerance is a third of the arm's whole usable band
#   MoveIt 2  /move_action               joint- and pose-goal motions
#             /compute_cartesian_path    straight-line approach / lift
#             /execute_trajectory        executing the Cartesian segments
#             /apply_planning_scene      the two benches, so the arm does not
#                                        plan straight through them
#   TF        object_target_frame        PERCEIVE (Phase 7's detector)
#   gripper   /gripper_action_controller/gripper_cmd   GRASP / RELEASE
#
# DESIGN RULES THIS FILE ENFORCES
#
# 1. No state may block forever.  Every external call goes through
#    _await_future(), which polls the rclpy future against a deadline built
#    from time.monotonic() and gives up.  A state that gives up returns
#    failure; it never leaves the machine wedged.
#
#    time.monotonic() and NOT get_clock().now(): under use_sim_time the ROS
#    clock reads 0 until the first /clock message and then jumps to the sim's
#    uptime, so `deadline = now() + timeout` expires the instant sim time
#    arrives.  That trap is documented in CLAUDE.md and has bitten this
#    workspace before.
#
# 2. Failure is data, not an exception.  Each state method returns
#    (ok: bool, detail: str).  The driver logs the transition and routes
#    every failure to RECOVERY, which retries a bounded number of times and
#    then falls through to ABORT.
#
# 3. dry_run stubs every external interface, so the transition logic can still
#    be gated with no simulator, no Nav2, no move_group and no controllers.
#    `dry_run_fail_state` forces exactly one state's stub to fail so the
#    recovery path can be exercised deliberately.
#
# GEOMETRY, AND WHY IT IS WHAT IT IS
#
#   The base parks at x = 3.24 m: its lidar puck protrudes to x = +0.575 and
#   the workbench's near legs stand at x = 3.82.  The arm's shoulder is at
#   base_footprint (0.0812, 0, 0.4844) and a top-down grasp needs tool0
#   0.115 m above the object, so the reachable band for an object standing on
#   the bench is 0.45 m to 0.85 m in front of base_footprint (measured with
#   /compute_ik over the ur5_arm group, everything outside returns
#   NO_IK_SOLUTION).  The targets sit at x = 3.92, i.e. 0.68 m out — dead
#   centre of that band, with room for Nav2 to be 0.2 m wrong either way.
#
# Usage:
#   ros2 run mobile_manipulator_orchestrator warehouse_orchestrator \
#       --ros-args -p dry_run:=true
#   ros2 run mobile_manipulator_orchestrator warehouse_orchestrator \
#       --ros-args -p dry_run:=true -p dry_run_fail_state:=GRASP
#   ros2 run mobile_manipulator_orchestrator warehouse_orchestrator \
#       --ros-args -p dry_run:=false -p use_sim_time:=true
# ─────────────────────────────────────────────────────────────────────────────
import math
import sys
import threading
import time
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'arm_shoulder_pan_joint',
    'arm_shoulder_lift_joint',
    'arm_elbow_joint',
    'arm_wrist_1_joint',
    'arm_wrist_2_joint',
    'arm_wrist_3_joint',
]

KNUCKLE_JOINT = 'gripper_robotiq_85_left_knuckle_joint'

# ── Gripper geometry, measured rather than guessed ───────────────────────────
# The fingertip LINK ORIGIN sits 0.1093 m along tool0's +z, but the pad's
# collision mesh (robotiq_description/meshes/collision/left_finger_tip.stl)
# spans local z from -0.006 to +0.051, so the pad's leading face reaches
# 0.1603 m below tool0 with the jaws open — and 0.1694 m once they close,
# because closing swings the tip origin a further 9 mm away from the wrist.
#
# Using the link origin instead of the mesh is what a top-down grasp offset of
# 0.115 encoded, and it drove the pads 9 mm into the bench: the arm stalled
# 8 mm short of its commanded pose, never settled inside the controller's
# 0.01 rad goal tolerance, and MoveIt cancelled the descent as "taking too long
# to execute". Measured at the stall: pad face z = 0.5989 against a bench
# surface at 0.600.
PAD_REACH_OPEN = 0.1603
PAD_REACH_CLOSED = 0.1694
#: radius of the warehouse target spheres (warehouse.world)
TARGET_RADIUS = 0.033
#: tool0 stands this far above the object's centre for a top-down grasp
GRASP_OFFSET = 0.145


def retime(trajectory, scale):
    """
    Stretch a JointTrajectory in time by `scale`, in place.

    /compute_cartesian_path has no max_velocity_scaling_factor field in Humble
    — the request carries only max_step, jump_threshold and avoid_collisions —
    so its solution comes back parameterized at 100 % of the joint limits,
    while every MoveGroup pose goal in this file asks for 25 %.  Under
    gz_ros2_control a position command is tracked by a proportional velocity
    command, so the arm lags in proportion to commanded speed: a full-speed
    trajectory is still catching up long after its nominal end.  Measured, the
    lift-and-carry segment needed 18.6 s to execute a 4.7 s trajectory, and
    MoveIt cancelled it at its 10.4 s bound ("Controller is taking too long to
    execute trajectory") — a bound which is itself derived from the too-short
    trajectory duration, so the error compounds.

    Scaling time by k and velocities by 1/k (accelerations by 1/k^2) is the
    same transformation max_velocity_scaling_factor would apply, and it widens
    MoveIt's own duration bound by the same factor.
    """
    for point in trajectory.points:
        total = (point.time_from_start.sec * 1_000_000_000
                 + point.time_from_start.nanosec) * scale
        point.time_from_start.sec = int(total // 1_000_000_000)
        point.time_from_start.nanosec = int(total % 1_000_000_000)
        point.velocities = [v / scale for v in point.velocities]
        point.accelerations = [a / (scale * scale) for a in point.accelerations]
    return trajectory


def pad_clearance(grasp_offset, target_radius=TARGET_RADIUS,
                  pad_reach=PAD_REACH_CLOSED):
    """
    Gap between the gripper pads and the surface the object stands on.

    The object's centre is `target_radius` above that surface and tool0 is
    `grasp_offset` above the centre, so the pad face clears the surface by
    grasp_offset - pad_reach + target_radius.  Negative means the gripper is
    being commanded through the bench.
    """
    return grasp_offset - pad_reach + target_radius


class State(Enum):
    """
    Every state the orchestrator can occupy.

    RECOVERY and ABORT are real states, not error codes: RECOVERY owns the
    retry budget, ABORT is terminal-failure, DONE is terminal-success.
    """

    HOME = 'HOME'
    NAV_TO_PICK = 'NAV_TO_PICK'
    PERCEIVE = 'PERCEIVE'
    APPROACH_ARM = 'APPROACH_ARM'
    GRASP = 'GRASP'
    NAV_TO_DROP = 'NAV_TO_DROP'
    PLACE_ARM = 'PLACE_ARM'
    RELEASE = 'RELEASE'
    RETURN_HOME = 'RETURN_HOME'
    RECOVERY = 'RECOVERY'
    ABORT = 'ABORT'
    DONE = 'DONE'


#: Nominal happy-path order.  RECOVERY consults this to know what to retry.
SEQUENCE = [
    State.HOME,
    State.NAV_TO_PICK,
    State.PERCEIVE,
    State.APPROACH_ARM,
    State.GRASP,
    State.NAV_TO_DROP,
    State.PLACE_ARM,
    State.RELEASE,
    State.RETURN_HOME,
]


def yaw_to_quat(yaw):
    """Quaternion for a yaw-only rotation."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def topdown_quat(yaw=0.0):
    """
    Orientation with the tool's +Z pointing at the floor.

    qz(yaw) * qx(pi) works out to (cos(yaw/2), sin(yaw/2), 0, 0) in
    (x, y, z, w).  At yaw = 0 the gripper's fingers straddle along the world
    x axis, which is what keeps them clear of the neighbouring targets — those
    are spaced 0.26 m apart along y.

    z AND w MUST BE ASSIGNED EXPLICITLY.  geometry_msgs/Quaternion defaults to
    w = 1, so leaving it out yields (1, 0, 0, 1) — which normalizes to a 90 deg
    rotation about X and aims the gripper sideways along -Y instead of at the
    floor.  MoveIt notices ("Orientation constraint for link 'arm_tool0' is
    probably incorrect ... Assuming identity instead") but plans anyway, so the
    damage only shows up much later as a Cartesian "descent" that sweeps the
    forearm through the chassis.  test_grasp_geometry.py pins this down.
    """
    q = Quaternion()
    q.x = math.cos(yaw / 2.0)
    q.y = math.sin(yaw / 2.0)
    q.z = 0.0
    q.w = 0.0
    return q


def wrap(angle):
    """Fold an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class WarehouseOrchestrator(Node):
    """Runs the pick-and-place state machine."""

    def __init__(self):
        super().__init__('warehouse_orchestrator')

        # ── behaviour ────────────────────────────────────────────────────────
        self.declare_parameter('dry_run', False)
        self.declare_parameter('dry_run_delay', 0.3)
        self.declare_parameter('dry_run_fail_state', '')
        self.declare_parameter('cycles', 1)
        self.declare_parameter('max_retries', 2)

        # ── per-state timeouts [s].  Nothing waits longer than these. ────────
        # nav_timeout is generous because the return leg threads the 0.934 m
        # jersey-barrier gate at x = 1.6 east-to-west, which takes far longer
        # than the outbound leg (measured 16.6 m driven for a 2.71 m plan).
        self.declare_parameter('nav_timeout', 240.0)
        self.declare_parameter('dock_timeout', 60.0)
        # Wall-clock, and the simulation runs at ~0.48 real time under the
        # full stack, so 40 s of wall clock is only ~19 s of simulated time —
        # a handful of YOLO frames once the detector is competing with
        # gz-sim and Nav2 for the same eight cores.
        self.declare_parameter('perceive_timeout', 90.0)
        # Wall-clock, against a simulation running at ~0.48 real time: a 40 s
        # trajectory needs ~83 s of wall clock before MoveIt has even decided
        # it is late.
        self.declare_parameter('arm_timeout', 150.0)
        self.declare_parameter('gripper_timeout', 20.0)
        self.declare_parameter('server_wait_timeout', 30.0)
        #: how long HOME waits for AMCL to publish map -> odom before giving up
        self.declare_parameter('localization_timeout', 90.0)

        # ── poses.  Nav2 goals are coarse; the dock poses are what the arm
        #    geometry actually depends on, and are reached by _dock_to(). ─────
        self.declare_parameter('pick_pose', [3.00, 0.0, 0.0])
        self.declare_parameter('drop_pose', [3.00, 3.2, 0.0])
        self.declare_parameter('home_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('pick_dock_pose', [3.24, 0.0, 0.0])
        self.declare_parameter('drop_dock_pose', [3.24, 3.2, 0.0])
        self.declare_parameter('nav_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic',
                               '/diff_drive_controller/cmd_vel_unstamped')

        # ── benches, as MoveIt collision objects: [cx, cy, top_z, sx, sy] in
        #    the map frame.  Without these the arm plans straight through the
        #    bench, and Gazebo answers an interpenetrating slab by launching
        #    the whole robot off the map. ──────────────────────────────────────
        self.declare_parameter('pick_bench', [4.5, 0.0, 0.60, 1.5, 0.8])
        self.declare_parameter('drop_bench', [4.5, 3.2, 0.60, 1.5, 0.8])
        #: where the object is set down, in the map frame (x, y, ball centre z)
        self.declare_parameter('place_point', [3.92, 3.2, 0.633])
        #: thickness of the slab used to model each bench top (see _bench_object)
        self.declare_parameter('bench_slab_thickness', 0.06)

        # ── arm ──────────────────────────────────────────────────────────────
        self.declare_parameter('planning_group', 'ur5_arm')
        self.declare_parameter('ee_link', 'arm_tool0')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('target_frame', 'object_target_frame')
        self.declare_parameter('stow_pose', [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
        # Wrist camera at base_footprint (0.20, 0.15, 1.15) looking at
        # (0.68, 0, 0.633) — 0.72 m out, the whole bench inside a 69 deg FOV.
        # Solved with /compute_ik on camera_color_frame, then picked for the
        # smallest joint excursions of the 82 configurations that solved.
        self.declare_parameter('look_pose',
                               [-0.6104, -1.8772, 0.2168, 1.4008, 1.2730, -1.4930])
        #: tool0 sits this far above the object's centre for a top-down grasp.
        #: See GRASP_OFFSET / pad_clearance() — this is measured to the pad's
        #: collision mesh, not to the fingertip link origin.
        self.declare_parameter('grasp_offset', GRASP_OFFSET)
        #: pre-grasp / pre-place stand-off above that
        self.declare_parameter('approach_height', 0.15)
        #: straight-up lift once the object is held
        self.declare_parameter('lift_height', 0.20)
        # Carry configuration, as JOINT ANGLES rather than a tool0 pose.
        #
        # It was a Cartesian waypoint, and that is what broke the lift: a
        # straight-line transit across the deck asks computeCartesianPath to
        # solve IK at every 10 mm step, and this workspace's KDL solver freely
        # returns solutions wound past +/-3pi/2 — of the eight carry poses
        # probed with /compute_ik, seven came back with a joint beyond 4.6 rad.
        # The interpolator stitched those branches together into a trajectory
        # containing a ~4 rad wrist_1 excursion, which the arm was still
        # unwinding 60 s later (measured: wrist_1 = +4.33 rad, tool0 overshot
        # to x = 0.243 against a 0.42 target).  Naming the configuration
        # removes the IK branch ambiguity entirely, and OMPL plans a route to
        # it that respects the 25 % velocity scaling.
        #
        # tool0 at base_footprint (0.40, 0.0, 0.85), jaws down, folded back
        # over the deck; the only one of the probed poses whose IK solution
        # keeps every joint inside 1.63 rad.
        self.declare_parameter(
            'carry_pose', [-0.3494, -1.5647, 1.5063, 1.6291, 1.5708, 1.2213])
        #: object centre is released this far above the drop surface
        self.declare_parameter('release_clearance', 0.015)
        # Time-stretch applied to Cartesian segments.  See retime().
        #
        # Was 4.0, matching the 25 % velocity scaling the pose goals use, when
        # the Cartesian timeouts were still thought to be a tracking-lag
        # problem.  They were not — they were MoveIt comparing a sim-time
        # trajectory duration against a wall clock at a real-time factor of
        # 0.48 (see allowed_execution_duration_scaling in
        # moveit_controllers.yaml).  With that fixed, a 4x stretch only makes
        # every approach and lift four times longer in an already
        # half-speed simulation, which then overruns this file's own per-state
        # timeouts instead.  1.5 keeps the motion gentle around the object
        # without spending the budget.
        self.declare_parameter('cartesian_time_scale', 1.5)
        # Maximum revolute joint step [rad] computeCartesianPath may take
        # between two 10 mm waypoints before it declares a discontinuity and
        # truncates the path.  ZERO DISABLES THE CHECK — that is the MoveIt
        # default and it is what allowed a 4 rad wrist flip to be returned as
        # a "100 % followed" solution.  A truncated path fails this file's
        # fraction test immediately, which is a far better outcome than an
        # inexecutable trajectory that times out a minute later.
        self.declare_parameter('revolute_jump_threshold', 1.5)

        # ── gripper ──────────────────────────────────────────────────────────
        # The 2F-85's pads are 85 mm apart at knuckle 0 and close 0.0505 m of
        # pad separation per 0.1355 m of fingertip-origin travel; a 66 mm ball
        # is first touched at 0.175 rad.  Commanding 0.30 leaves ~10 mm of
        # interference — enough to hold, not enough to fire the ball out of
        # the jaws.  A successful grasp therefore STALLS short of the command,
        # which is why the controller needs allow_stalling: true, and why
        # "did the knuckle stop inside [0.05, 0.28]" is the grasp check.
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_closed', 0.30)
        self.declare_parameter('gripper_effort', 60.0)
        self.declare_parameter('grasp_hold_min', 0.05)
        self.declare_parameter('grasp_hold_max', 0.28)

        g = self.get_parameter
        self.dry_run = g('dry_run').value
        self.dry_delay = float(g('dry_run_delay').value)
        self.fail_state = str(g('dry_run_fail_state').value).strip().upper()
        self.cycles = int(g('cycles').value)
        self.max_retries = int(g('max_retries').value)

        self.timeouts = {
            State.NAV_TO_PICK: float(g('nav_timeout').value),
            State.NAV_TO_DROP: float(g('nav_timeout').value),
            State.RETURN_HOME: float(g('nav_timeout').value),
            State.PERCEIVE: float(g('perceive_timeout').value),
            State.APPROACH_ARM: float(g('arm_timeout').value),
            State.PLACE_ARM: float(g('arm_timeout').value),
            State.GRASP: float(g('gripper_timeout').value),
            State.RELEASE: float(g('gripper_timeout').value),
            State.HOME: 60.0,
        }

        if self.fail_state and self.fail_state not in State.__members__:
            self.get_logger().warn(
                f"dry_run_fail_state '{self.fail_state}' is not a state name; ignoring")
            self.fail_state = ''

        # ── interfaces (never created in dry_run: the point is to need none) ─
        self.nav_cli = None
        self.move_cli = None
        self.exec_cli = None
        self.grip_cli = None
        self.cart_cli = None
        self.scene_cli = None
        self.cmd_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.joint_state = {}
        if not self.dry_run:
            self.nav_cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.move_cli = ActionClient(self, MoveGroup, '/move_action')
            self.exec_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
            self.grip_cli = ActionClient(
                self, GripperCommand, '/gripper_action_controller/gripper_cmd')
            # Straight to the controller, bypassing MoveIt — see _force_stow().
            self.traj_cli = ActionClient(
                self, FollowJointTrajectory,
                '/arm_controller/follow_joint_trajectory')
            self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
            self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
            # diff_drive_controller subscribes with SystemDefaultsQoS(), i.e.
            # BEST_EFFORT — a RELIABLE publisher is silently dropped.
            self.cmd_pub = self.create_publisher(
                Twist, g('cmd_vel_topic').value,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = State.HOME
        self.retries = 0
        self.failed_state = None
        self.grasp_point = None          # (x, y, z) in odom_frame
        self.history = []                # for the end-of-run trace

        mode = 'DRY RUN' if self.dry_run else 'LIVE'
        self.get_logger().info(f'orchestrator starting in {mode} mode, {self.cycles} cycle(s)')
        if self.dry_run and self.fail_state:
            self.get_logger().warn(f'dry run will FORCE FAILURE in state {self.fail_state}')

    # ══════════════════════════════════════════════════════════════════════
    # plumbing
    # ══════════════════════════════════════════════════════════════════════
    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_state[name] = pos

    def _enter(self, state):
        """Record entering `state`.  The single place history is appended."""
        self.history.append(state)
        self.state = state

    def _log_transition(self, frm, to, detail=''):
        arrow = f'{frm.value} -> {to.value}'
        self.get_logger().info(f'[STATE] {arrow}{"  (" + detail + ")" if detail else ""}')

    def _await_future(self, fut, timeout, what):
        """
        Block until `fut` resolves or `timeout` elapses.

        Returns (done, result).  Uses time.monotonic(): a deadline built from
        get_clock() would expire instantly the moment sim time arrives.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if fut.done():
                return True, fut.result()
            time.sleep(0.05)
        self.get_logger().error(f'timeout after {timeout:.0f}s waiting for {what}')
        return False, None

    def _stub(self, state):
        """Simulate an external call for dry runs."""
        time.sleep(self.dry_delay)
        if self.fail_state and state.value == self.fail_state:
            return False, f'stub forced failure in {state.value}'
        return True, 'stub ok'

    def _run_action(self, client, goal, timeout, what):
        """
        Send an action goal and wait for the result, bounded by `timeout`.

        Any of: server missing, goal rejected, no result, non-SUCCEEDED status
        returns failure rather than raising or hanging.
        """
        wait = float(self.get_parameter('server_wait_timeout').value)
        if not client.wait_for_server(timeout_sec=wait):
            return False, f'action server for {what} unavailable after {wait:.0f}s'

        send_fut = client.send_goal_async(goal)
        done, handle = self._await_future(send_fut, timeout, f'{what} goal ack')
        if not done or handle is None:
            return False, f'{what} goal was never acknowledged'
        if not handle.accepted:
            return False, f'{what} goal rejected by server'

        res_fut = handle.get_result_async()
        done, res = self._await_future(res_fut, timeout, f'{what} result')
        if not done or res is None:
            # Best-effort cancel so a late-finishing goal cannot drive the
            # robot after we have already moved on.
            try:
                handle.cancel_goal_async()
            except Exception:                                        # noqa: BLE001
                pass
            return False, f'{what} produced no result within {timeout:.0f}s'
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'{what} finished with status {res.status}'
        return True, f'{what} succeeded'

    def _call_service(self, client, request, timeout, what):
        """Call a service, bounded.  Returns (ok, response_or_detail)."""
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'service for {what} unavailable after {timeout:.0f}s'
        fut = client.call_async(request)
        done, res = self._await_future(fut, timeout, what)
        if not done or res is None:
            return False, f'{what} produced no response within {timeout:.0f}s'
        return True, res

    def _lookup(self, target, source, timeout=5.0):
        """TF lookup with a bounded monotonic wait.  Returns a Transform."""
        deadline = time.monotonic() + timeout
        last = 'no transform yet'
        while time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                last = str(exc)
            time.sleep(0.1)
        self.get_logger().error(f'TF {target} <- {source} unavailable: {last}')
        return None

    @staticmethod
    def _pose_of(tf):
        """(x, y, yaw) out of a TransformStamped."""
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _map_point_to(self, frame, point):
        """Express a map-frame (x, y, z) point in `frame` (planar transform)."""
        nav = self.get_parameter('nav_frame').value
        tf = self._lookup(frame, nav)
        if tf is None:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.x + c * point[0] - s * point[1],
                t.y + s * point[0] + c * point[1],
                t.z + point[2])

    # ══════════════════════════════════════════════════════════════════════
    # planning scene
    # ══════════════════════════════════════════════════════════════════════
    def _bench_object(self, name, spec):
        """Build a CollisionObject for one bench, expressed in the odom frame."""
        odom = self.get_parameter('odom_frame').value
        cx, cy, top_z, sx, sy = [float(v) for v in spec]
        # The TABLETOP SLAB ONLY, not a solid block from the floor up.
        #
        # A solid block was the first attempt, on the reasoning that the legs
        # are too thin to plan between.  It made the whole volume under the
        # bench forbidden, which put the obstacle's boundary exactly at the
        # height the arm works at: after lifting the object clear, FCL reported
        # 'pick_bench' against 'arm_upper_arm_link' and every subsequent plan
        # died with an invalid start state.  The arm cannot reach under the
        # bench anyway — what it can actually hit is the slab it reaches over,
        # which is also the surface phase7_look_pose warned about dragging the
        # gripper through.  Modelling only that removes the false positives
        # without giving up the protection.
        #
        # 60 mm thick against a real 30 mm top: the extra sits BELOW the
        # surface, where nothing operates, so it buys margin for free.
        thickness = float(self.get_parameter('bench_slab_thickness').value)
        centre = self._map_point_to(odom, (cx, cy, top_z - thickness / 2.0))
        if centre is None:
            return None
        obj = CollisionObject()
        obj.header.frame_id = odom
        obj.id = name
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, thickness]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = centre
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def _publish_scene(self):
        """Put both benches into move_group's world.  Bounded, never raises."""
        objects = []
        for name in ('pick_bench', 'drop_bench'):
            obj = self._bench_object(name, self.get_parameter(name).value)
            if obj is None:
                return False, f'cannot place {name}: no map->odom transform yet'
            objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects.extend(objects)
        ok, res = self._call_service(self.scene_cli, req, 20.0, 'apply_planning_scene')
        if not ok:
            return False, res
        if not res.success:
            return False, 'move_group rejected the planning scene diff'
        return True, f'{len(objects)} bench collision objects in the scene'

    # ══════════════════════════════════════════════════════════════════════
    # motion primitives
    # ══════════════════════════════════════════════════════════════════════
    def _move_goal(self):
        """A MoveGroup goal with the parts every request shares."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.get_parameter('planning_group').value
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.25
        req.max_acceleration_scaling_factor = 0.25
        # Empty arm-only diff, never a RobotState echoed back from
        # /joint_states: under Gazebo the ros2_control backend also publishes
        # `gripper_robotiq_85_*_joint_mimic` names that are absent from the
        # URDF, and move_group *crashes* if a client sends those names back
        # inside a RobotState.  See CLAUDE.md.
        req.start_state = RobotState()
        req.start_state.is_diff = True
        goal.planning_options.plan_only = False
        return goal

    def _move_joints(self, positions, timeout, what):
        """Plan and execute to an explicit arm configuration."""
        goal = self._move_goal()
        c = Constraints()
        for name, value in zip(ARM_JOINTS, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        return self._run_action(self.move_cli, goal, timeout, what)

    def _move_pose(self, point, timeout, what, frame=None, yaw=0.0):
        """Plan and execute a top-down tool0 pose goal."""
        frame = frame or self.get_parameter('base_frame').value
        goal = self._move_goal()

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = self.get_parameter('ee_link').value
        pc.weight = 1.0
        vol = SolidPrimitive()
        vol.type = SolidPrimitive.SPHERE
        vol.dimensions = [0.01]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in point]
        pose.orientation.w = 1.0
        pc.constraint_region.primitives.append(vol)
        pc.constraint_region.primitive_poses.append(pose)

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = pc.link_name
        oc.orientation = topdown_quat(yaw)
        oc.absolute_x_axis_tolerance = 0.10
        oc.absolute_y_axis_tolerance = 0.10
        oc.absolute_z_axis_tolerance = 0.20
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        goal.request.goal_constraints.append(c)
        return self._run_action(self.move_cli, goal, timeout, what)

    def _cartesian(self, points, timeout, what, yaw=0.0, avoid_collisions=True):
        """
        Move tool0 through `points` in a straight line, top-down throughout.

        The approach and the lift have to be straight: an OMPL joint-space
        detour on the last 15 cm sweeps the open jaws sideways through the
        object it is about to pick up.
        """
        frame = self.get_parameter('base_frame').value
        req = GetCartesianPath.Request()
        req.header.frame_id = frame
        req.start_state = RobotState()
        req.start_state.is_diff = True
        req.group_name = self.get_parameter('planning_group').value
        req.link_name = self.get_parameter('ee_link').value
        req.max_step = 0.01
        req.jump_threshold = 0.0          # legacy scalar; superseded below
        req.revolute_jump_threshold = float(
            self.get_parameter('revolute_jump_threshold').value)
        req.prismatic_jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        for p in points:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = [float(v) for v in p]
            pose.orientation = topdown_quat(yaw)
            req.waypoints.append(pose)

        ok, res = self._call_service(self.cart_cli, req, timeout, f'{what} (cartesian plan)')
        if not ok:
            return False, res
        if res.fraction < 0.95:
            return False, (f'{what}: cartesian plan covered only '
                           f'{res.fraction * 100:.0f}% of the path')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        retime(goal.trajectory.joint_trajectory,
               float(self.get_parameter('cartesian_time_scale').value))
        return self._run_action(self.exec_cli, goal, timeout, what)

    # ══════════════════════════════════════════════════════════════════════
    # external calls
    # ══════════════════════════════════════════════════════════════════════
    def _navigate(self, state, xyyaw, what):
        if self.dry_run:
            return self._stub(state)
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = self.get_parameter('nav_frame').value
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(xyyaw[0]), float(xyyaw[1])
        p.pose.orientation = yaw_to_quat(float(xyyaw[2]))
        goal.pose = p
        return self._run_action(self.nav_cli, goal, self.timeouts[state], what)

    def _dock_to(self, xyyaw, what, pos_tol=0.03, yaw_tol=0.05, max_travel=0.8):
        """
        Close the last few centimetres on the map frame, under our own control.

        Nav2 hands the base over anywhere inside a 0.25 m / 0.25 rad goal
        tolerance, and the arm's usable band in front of the base is only
        0.40 m wide, so a coarse arrival can put the target out of reach with
        Nav2 still reporting SUCCEEDED.  This nulls the along-body error and
        the heading; lateral error is deliberately left alone, because
        correcting it means spinning, and an in-place spin walks this
        skid-steer base ~0.12 m sideways per radian — it would cost more than
        it fixes, and perception measures wherever the object actually ends up.

        Speeds are floored, not tapered: the base has a hard stiction deadband
        below ~0.27 rad/s and ~0.05 m/s, so a P-controller that fades out
        simply stops moving while still reporting error.
        """
        base = self.get_parameter('base_frame').value
        nav = self.get_parameter('nav_frame').value
        tx, ty, tyaw = [float(v) for v in xyyaw]
        deadline = time.monotonic() + float(self.get_parameter('dock_timeout').value)

        start = self._lookup(nav, base, timeout=10.0)
        if start is None:
            return False, f'{what}: no {nav} -> {base} transform to dock against'
        sx, sy, _ = self._pose_of(start)

        last = None
        while time.monotonic() < deadline:
            tf = self._lookup(nav, base, timeout=2.0)
            if tf is None:
                break
            x, y, yaw = self._pose_of(tf)
            if math.hypot(x - sx, y - sy) > max_travel:
                self._stop_base()
                return False, (f'{what}: travelled more than {max_travel:.2f} m '
                               f'while docking — refusing to keep going')

            # error along the body x axis, and the heading error
            ex = math.cos(yaw) * (tx - x) + math.sin(yaw) * (ty - y)
            eyaw = wrap(tyaw - yaw)
            last = (x, y, yaw, ex, eyaw)

            cmd = Twist()
            if abs(eyaw) > yaw_tol:
                cmd.angular.z = math.copysign(max(0.30, min(0.6, abs(eyaw) * 1.2)), eyaw)
            elif abs(ex) > pos_tol:
                cmd.linear.x = math.copysign(max(0.06, min(0.25, abs(ex) * 0.8)), ex)
            else:
                self._stop_base()
                return True, (f'{what}: docked at ({x:+.3f}, {y:+.3f}, {yaw:+.3f}), '
                              f'along-body error {ex * 1000:+.0f} mm')
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

        self._stop_base()
        if last is None:
            return False, f'{what}: docking never saw a pose'
        x, y, yaw, ex, eyaw = last
        return False, (f'{what}: docking timed out at ({x:+.3f}, {y:+.3f}, {yaw:+.3f}), '
                       f'{ex * 1000:+.0f} mm / {eyaw:+.3f} rad still to go')

    def _force_stow(self, seconds=6.0):
        """
        Stow the arm through the controller, with no planner involved.

        RECOVERY cannot use MoveIt to get out of the situations that most need
        recovering.  Every failure that leaves the arm's start state in
        collision — a bench object, a self-collision — makes move_group reject
        the very next plan in about 0.2 s with "Motion planning start tree
        could not be initialized!", so the retry budget evaporates before
        anything has been attempted.  The controller has no such opinion: it
        interpolates to the commanded configuration regardless of what the
        planning scene thinks, which is exactly what is wanted for a retreat to
        a known-safe pose.

        The trajectory is a single point over `seconds`, deliberately slow.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.get_parameter('stow_pose').value]
        point.time_from_start.sec = int(seconds)
        goal.trajectory.points.append(point)
        # x3 plus a fixed margin: the trajectory duration is simulated time
        # and this wait is wall clock, against a ~0.48 real-time factor.
        return self._run_action(self.traj_cli, goal, seconds * 3.0 + 20.0,
                                'recovery stow (direct to controller)')

    def _undock(self, xyyaw, what):
        """
        Back away from a bench before handing control to Nav2.

        Docking deliberately parks the base 0.03 m from the bench legs so the
        arm can reach — which leaves the footprint's front edge (x = +0.58,
        the lidar puck) about a centimetre from cells the costmap calls
        lethal.  Nav2 cannot get out of that: DWB scores every candidate
        trajectory with the ObstacleFootprint critic, all of them start in
        collision, the behaviour tree escalates to Spin, and a Spin is the one
        manoeuvre this skid-steer base must not perform — it walks the body
        ~0.12 m sideways per radian, which odometry cannot see, so each
        recovery makes localization worse until the estimate is metres out and
        the robot drives under the bench it was trying to leave.

        Measured with the live global costmap while it was failing: the drop
        goal cell and the entire corridor to it were cost 0, i.e. the route was
        never the problem — only the start.

        Reversing to the same standoff Nav2 was asked to reach costs about a
        second and is done with _dock_to(), which already handles a negative
        along-body error.  Failure is logged, not propagated: if the undock
        does not complete, the navigation attempt afterwards is still worth
        making.
        """
        ok, detail = self._dock_to(xyyaw, f'undock from {what}')
        level = self.get_logger().info if ok else self.get_logger().warn
        level(f'[UNDOCK] {detail}')
        return ok

    def _stop_base(self):
        """Zero the wheels, several times — the topic is BEST_EFFORT."""
        for _ in range(10):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)

    def _wait_for_target(self, state):
        """
        Wait for a FRESH object_target_frame and latch it in the odom frame.

        Freshness is judged by the stamp *advancing*, not by its absolute age:
        the perception node runs inference and lags /clock by a variable
        amount, and it simply stops broadcasting when it loses the target —
        so a stale-but-recent stamp must not count as a live detection.

        The point is latched in odom rather than base_footprint so that any
        base motion between here and the grasp is compensated.
        """
        if self.dry_run:
            ok, detail = self._stub(state)
            if ok:
                self.grasp_point = (0.60, 0.0, 0.35)     # plausible stand-in
            return ok, detail

        cam = self.get_parameter('camera_frame').value
        tgt = self.get_parameter('target_frame').value
        odom = self.get_parameter('odom_frame').value
        timeout = self.timeouts[state]
        deadline = time.monotonic() + timeout
        first_stamp = None

        while time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(odom, tgt, rclpy.time.Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if first_stamp is None:
                    first_stamp = stamp
                elif stamp > first_stamp:
                    t = tf.transform.translation
                    self.grasp_point = (t.x, t.y, t.z)
                    return True, (f'{tgt} live in {odom} at '
                                  f'({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f})')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
            time.sleep(0.1)

        seen = 'never appeared' if first_stamp is None else 'stopped advancing'
        return False, f'{cam} -> {tgt} {seen} within {timeout:.0f}s'

    def _gripper(self, state, position, what):
        if self.dry_run:
            return self._stub(state)
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter('gripper_effort').value)
        return self._run_action(self.grip_cli, goal, self.timeouts[state], what)

    def _grasp_point_in_base(self):
        """The latched target, re-expressed in base_footprint right now."""
        if self.grasp_point is None:
            return None
        base = self.get_parameter('base_frame').value
        odom = self.get_parameter('odom_frame').value
        tf = self._lookup(base, odom)
        if tf is None:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        px, py, pz = self.grasp_point
        return (t.x + c * px - s * py, t.y + s * px + c * py, t.z + pz)

    # ══════════════════════════════════════════════════════════════════════
    # states — each returns (ok, detail); none of them raise
    # ══════════════════════════════════════════════════════════════════════
    def st_home(self):
        """Stow the arm, open the jaws, and hand move_group the benches."""
        if self.dry_run:
            return self._stub(State.HOME)
        self._stop_base()

        # Wait for localization before anything else.  Every map-frame pose in
        # this file — both bench collision objects, the drop point, both dock
        # targets — is resolved through map -> odom, and AMCL only starts
        # publishing that transform once it has fused its first laser scan.
        # The launch gates on bt_navigator reaching the active state, which
        # happens a few seconds earlier, so arriving here before the transform
        # exists is normal rather than exceptional and must not burn a retry.
        wait = float(self.get_parameter('localization_timeout').value)
        if self._lookup(self.get_parameter('odom_frame').value,
                        self.get_parameter('nav_frame').value, timeout=wait) is None:
            return False, f'AMCL published no map -> odom transform within {wait:.0f}s'

        ok, detail = self._publish_scene()
        if not ok:
            return False, detail
        ok, gdetail = self._gripper(State.HOME, self.get_parameter('gripper_open').value,
                                    'open gripper')
        if not ok:
            return False, gdetail
        ok, sdetail = self._move_joints(self.get_parameter('stow_pose').value,
                                        self.timeouts[State.HOME], 'stow arm')
        if not ok:
            return False, sdetail
        return True, f'at home, ready ({detail})'

    def st_nav_to_pick(self):
        ok, detail = self._navigate(State.NAV_TO_PICK,
                                    self.get_parameter('pick_pose').value,
                                    'nav to pick bench')
        if not ok or self.dry_run:
            return ok, detail
        return self._dock_to(self.get_parameter('pick_dock_pose').value, 'dock at pick bench')

    def st_perceive(self):
        """Aim the wrist camera at the bench, then wait for a live detection."""
        if not self.dry_run:
            ok, detail = self._move_joints(self.get_parameter('look_pose').value,
                                           self.timeouts[State.APPROACH_ARM],
                                           'arm to look pose')
            if not ok:
                return False, detail
        return self._wait_for_target(State.PERCEIVE)

    def st_approach_arm(self):
        """Pre-grasp above the object, then straight down onto it."""
        if self.dry_run:
            return self._stub(State.APPROACH_ARM)
        point = self._grasp_point_in_base()
        if point is None:
            return False, 'no target point available to plan to'

        offset = float(self.get_parameter('grasp_offset').value)
        approach = float(self.get_parameter('approach_height').value)
        grasp_z = point[2] + offset
        pre = (point[0], point[1], grasp_z + approach)

        ok, detail = self._move_pose(pre, self.timeouts[State.APPROACH_ARM],
                                     'pre-grasp above target')
        if not ok:
            return False, detail
        # avoid_collisions=False for the last 15 cm: the bench collision box
        # stands 20 mm below the fingertips at the grasp, which is real
        # clearance but reads as contact once MoveIt's default padding is
        # added, and the descent is a verified straight line.
        ok, detail = self._cartesian([(point[0], point[1], grasp_z)],
                                     self.timeouts[State.APPROACH_ARM],
                                     'descend onto target', avoid_collisions=False)
        if not ok:
            return False, detail
        return True, (f'jaws around target at ({point[0]:+.3f}, {point[1]:+.3f}, '
                      f'{point[2]:+.3f})')

    def st_grasp(self):
        """Close the jaws, confirm they stalled on something, then lift."""
        if self.dry_run:
            return self._stub(State.GRASP)
        ok, detail = self._gripper(State.GRASP,
                                   self.get_parameter('gripper_closed').value,
                                   'close gripper')
        if not ok:
            return False, detail

        knuckle = self.joint_state.get(KNUCKLE_JOINT)
        lo = float(self.get_parameter('grasp_hold_min').value)
        hi = float(self.get_parameter('grasp_hold_max').value)
        if knuckle is None:
            return False, f'no {KNUCKLE_JOINT} in /joint_states — cannot confirm the grasp'
        if not lo <= knuckle <= hi:
            return False, (f'gripper closed to {knuckle:.3f} rad, outside the '
                           f'holding band [{lo:.2f}, {hi:.2f}] — nothing in the jaws')

        point = self._grasp_point_in_base()
        if point is None:
            return False, 'holding the object but lost the odom transform to lift by'
        lift = float(self.get_parameter('lift_height').value)
        grasp_z = point[2] + float(self.get_parameter('grasp_offset').value)

        # Straight up first, and ONLY straight up.  This leg has to be
        # Cartesian: a joint-space plan out of the jaws-closed pose would swing
        # the object sideways into the bench before clearing it.  It is a pure
        # +z translation from a known-good configuration, so it asks nothing
        # awkward of the IK.
        ok, detail = self._cartesian([(point[0], point[1], grasp_z + lift)],
                                     self.timeouts[State.APPROACH_ARM],
                                     'lift clear of the bench', avoid_collisions=False)
        if not ok:
            return False, detail

        # Then fold back to the carry configuration in joint space, where the
        # planner is free to choose the route and no IK branch is implied.
        ok, detail = self._move_joints(self.get_parameter('carry_pose').value,
                                       self.timeouts[State.APPROACH_ARM],
                                       'tuck to carry pose')
        if not ok:
            return False, detail
        return True, f'holding at knuckle {knuckle:.3f} rad, lifted and tucked'

    def st_nav_to_drop(self):
        if not self.dry_run:
            self._undock(self.get_parameter('pick_pose').value, 'pick bench')
        ok, detail = self._navigate(State.NAV_TO_DROP,
                                    self.get_parameter('drop_pose').value,
                                    'nav to drop bench')
        if not ok or self.dry_run:
            return ok, detail
        return self._dock_to(self.get_parameter('drop_dock_pose').value, 'dock at drop bench')

    def st_place_arm(self):
        """Above the drop zone, then straight down to just above the surface."""
        if self.dry_run:
            return self._stub(State.PLACE_ARM)
        base = self.get_parameter('base_frame').value
        place = self._map_point_to(base, self.get_parameter('place_point').value)
        if place is None:
            return False, 'no map -> base_footprint transform for the drop point'

        offset = float(self.get_parameter('grasp_offset').value)
        approach = float(self.get_parameter('approach_height').value)
        clearance = float(self.get_parameter('release_clearance').value)
        release_z = place[2] + clearance + offset
        pre = (place[0], place[1], release_z + approach)

        ok, detail = self._move_pose(pre, self.timeouts[State.PLACE_ARM],
                                     'above the drop zone')
        if not ok:
            return False, detail
        ok, detail = self._cartesian([(place[0], place[1], release_z)],
                                     self.timeouts[State.PLACE_ARM],
                                     'lower onto the drop bench', avoid_collisions=False)
        if not ok:
            return False, detail
        return True, f'over the drop zone at ({place[0]:+.3f}, {place[1]:+.3f})'

    def st_release(self):
        """Open the jaws, retract, and stow for the drive home."""
        ok, detail = self._gripper(State.RELEASE,
                                   self.get_parameter('gripper_open').value,
                                   'open gripper')
        if not ok or self.dry_run:
            return ok, detail

        # The retract is BEST EFFORT, and deliberately so.  By the time it
        # runs the jaws are open and the object is already resting on the
        # bench — the cycle's actual work is done.  Straight-line retracts
        # cross an IK discontinuity often enough that the jump-threshold check
        # truncates them (measured: 77-79 % of the path), and failing the whole
        # state over a tidying motion aborted a run whose object had been
        # delivered correctly.  A short hop is enough to clear the object; the
        # joint-space stow below is what actually gets the arm out of the way,
        # and OMPL plans that around whatever the retract did not achieve.
        base = self.get_parameter('base_frame').value
        place = self._map_point_to(base, self.get_parameter('place_point').value)
        if place is not None:
            offset = float(self.get_parameter('grasp_offset').value)
            hop = float(self.get_parameter('lift_height').value) / 2.0
            up = (place[0], place[1], place[2] + offset + hop)
            ok, rdetail = self._cartesian([up], self.timeouts[State.PLACE_ARM],
                                          'retract from the drop zone',
                                          avoid_collisions=False)
            if not ok:
                self.get_logger().warn(
                    f'[RELEASE] retract skipped ({rdetail}); stowing from here')
        ok, sdetail = self._move_joints(self.get_parameter('stow_pose').value,
                                        self.timeouts[State.APPROACH_ARM],
                                        'stow arm for transit')
        if not ok:
            return False, sdetail
        return True, 'object released, arm stowed'

    def st_return_home(self):
        if not self.dry_run:
            self._undock(self.get_parameter('drop_pose').value, 'drop bench')
        return self._navigate(State.RETURN_HOME,
                              self.get_parameter('home_pose').value, 'nav home')

    HANDLERS = {
        State.HOME: st_home,
        State.NAV_TO_PICK: st_nav_to_pick,
        State.PERCEIVE: st_perceive,
        State.APPROACH_ARM: st_approach_arm,
        State.GRASP: st_grasp,
        State.NAV_TO_DROP: st_nav_to_drop,
        State.PLACE_ARM: st_place_arm,
        State.RELEASE: st_release,
        State.RETURN_HOME: st_return_home,
    }

    # ══════════════════════════════════════════════════════════════════════
    # driver
    # ══════════════════════════════════════════════════════════════════════
    def run_cycle(self, index):
        """Run one full cycle.  Returns True on success, False if it aborted."""
        self.get_logger().info(f'───── cycle {index + 1} of {self.cycles} ─────')
        self.retries = 0
        self.failed_state = None
        self._enter(State.HOME)
        self.get_logger().info(f'[STATE] entering {State.HOME.value}')

        while True:
            if self.state is State.ABORT:
                self.get_logger().error(
                    f'[STATE] ABORT — giving up after {self.retries} retries of '
                    f'{self.failed_state.value if self.failed_state else "?"}')
                return False
            if self.state is State.DONE:
                self.get_logger().info('[STATE] DONE — cycle complete, back at HOME')
                return True

            if self.state is State.RECOVERY:
                nxt = self.do_recovery()
                self._log_transition(State.RECOVERY, nxt)
                self._enter(nxt)
                continue

            handler = self.HANDLERS[self.state]
            ok, detail = handler(self)

            if ok:
                self.retries = 0
                step = SEQUENCE.index(self.state)
                nxt = SEQUENCE[step + 1] if step + 1 < len(SEQUENCE) else State.DONE
                self._log_transition(self.state, nxt, detail)
                self._enter(nxt)
            else:
                self.get_logger().error(f'[FAIL ] {self.state.value}: {detail}')
                self.failed_state = self.state
                self._log_transition(self.state, State.RECOVERY, detail)
                self._enter(State.RECOVERY)

    def do_recovery(self):
        """Bounded retry, then ABORT.  Never returns to a blocking wait."""
        if self.retries >= self.max_retries:
            self.get_logger().error(
                f'[RECOV] retry budget ({self.max_retries}) exhausted for '
                f'{self.failed_state.value} — aborting')
            return State.ABORT

        self.retries += 1
        self.get_logger().warn(
            f'[RECOV] retry {self.retries}/{self.max_retries} of '
            f'{self.failed_state.value}')

        # A forced dry-run failure is deterministic, so retrying it can only
        # burn the budget.  Say so plainly rather than pretending to recover.
        if self.dry_run and self.fail_state == self.failed_state.value:
            self.get_logger().warn(
                '[RECOV] forced dry-run failure is deterministic; '
                'this retry will fail again by construction')
            return self.failed_state

        # Live retries start from a known configuration: an arm left halfway
        # into a bench is not a valid start state for the state that failed.
        if not self.dry_run and self.failed_state in (
                State.APPROACH_ARM, State.GRASP, State.PLACE_ARM, State.RELEASE):
            ok, detail = self._force_stow()
            self.get_logger().info(f'[RECOV] {detail}')
            if self.failed_state in (State.APPROACH_ARM, State.GRASP):
                # Re-perceive: the object may have been nudged by the failure.
                return State.PERCEIVE
        return self.failed_state

    def summary(self, results):
        ok = sum(1 for r in results if r)
        self.get_logger().info('═════════ ORCHESTRATOR SUMMARY ═════════')
        self.get_logger().info(f'  cycles run     : {len(results)}')
        self.get_logger().info(f'  cycles ok      : {ok}')
        self.get_logger().info(f'  final state    : {self.state.value}')
        self.get_logger().info(
            '  trace          : ' + ' -> '.join(s.value for s in self.history))
        self.get_logger().info('════════════════════════════════════════')


def main(argv=None):
    rclpy.init(args=argv)
    node = WarehouseOrchestrator()

    # The state machine blocks on futures, so it cannot share a thread with
    # the executor that resolves them.  Executor spins in the background;
    # the machine runs here.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results = []
    try:
        for i in range(node.cycles):
            results.append(node.run_cycle(i))
            if not results[-1]:
                break
        node.summary(results)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted')
    finally:
        # Order matters.  The executor must be stopped and its thread joined
        # BEFORE the node is destroyed: tearing a node down while a spin
        # thread still holds it aborts the process with "terminate called
        # without an active exception", which reads as a crash even though
        # the cycle finished cleanly.
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if results and all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
