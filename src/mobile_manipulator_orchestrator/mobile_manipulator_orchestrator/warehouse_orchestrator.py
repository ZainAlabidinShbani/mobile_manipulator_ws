#!/usr/bin/env python3
# warehouse_orchestrator.py
# ─────────────────────────────────────────────────────────────────────────────
# Phase 8 — explicit state machine driving one warehouse pick-and-place cycle:
#
#   HOME → NAV_TO_PICK → PERCEIVE → APPROACH_ARM → GRASP
#        → NAV_TO_DROP → PLACE_ARM → RELEASE → RETURN_HOME → HOME
#
# Four external interfaces, one per kind of subsystem:
#   Nav2      /navigate_to_pose                  (NAV_TO_PICK/DROP, RETURN_HOME)
#   TF        camera_color_optical_frame
#               → object_target_frame            (PERCEIVE)
#   MoveIt 2  /move_action                       (APPROACH_ARM, PLACE_ARM)
#   gripper   /gripper_action_controller/gripper_cmd   (GRASP, RELEASE)
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
# 3. dry_run stubs all four interfaces.  This lets the transition logic be
#    tested with no simulator, no Nav2, no move_group and no controllers —
#    which is the whole point of gating Phase 8 before Phase 9 wires it to
#    real servers.  `dry_run_fail_state` forces exactly one state's stub to
#    fail so the recovery path can be exercised deliberately.
#
# Usage:
#   ros2 run mobile_manipulator_orchestrator warehouse_orchestrator \
#       --ros-args -p dry_run:=true
#   ros2 run mobile_manipulator_orchestrator warehouse_orchestrator \
#       --ros-args -p dry_run:=true -p dry_run_fail_state:=GRASP
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

import tf2_ros

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from nav2_msgs.action import NavigateToPose
from shape_msgs.msg import SolidPrimitive


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
        self.declare_parameter('nav_timeout', 180.0)
        self.declare_parameter('perceive_timeout', 30.0)
        self.declare_parameter('arm_timeout', 60.0)
        self.declare_parameter('gripper_timeout', 15.0)
        self.declare_parameter('server_wait_timeout', 20.0)

        # ── poses ────────────────────────────────────────────────────────────
        # x=3.1 matches the Phase 7 perception pose; the base cannot park
        # closer than x ~ 3.24 before its front edge meets the workbench legs.
        self.declare_parameter('pick_pose', [3.1, 0.0, 0.0])
        self.declare_parameter('drop_pose', [3.1, 3.2, 0.0])
        self.declare_parameter('home_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('nav_frame', 'map')
        self.declare_parameter('planning_group', 'ur5_arm')
        self.declare_parameter('ee_link', 'arm_tool0')
        self.declare_parameter('planning_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('target_frame', 'object_target_frame')
        # Stand-off along -Z of the target so the gripper approaches rather
        # than driving its fingers through the object.
        self.declare_parameter('pregrasp_offset', 0.12)
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_closed', 0.8)
        self.declare_parameter('gripper_effort', 40.0)

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
            State.HOME: 10.0,
        }

        if self.fail_state and self.fail_state not in State.__members__:
            self.get_logger().warn(
                f"dry_run_fail_state '{self.fail_state}' is not a state name; ignoring")
            self.fail_state = ''

        # ── interfaces (never created in dry_run: the point is to need none) ─
        self.nav_cli = None
        self.move_cli = None
        self.grip_cli = None
        self.tf_buffer = None
        self.tf_listener = None
        if not self.dry_run:
            self.nav_cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.move_cli = ActionClient(self, MoveGroup, '/move_action')
            self.grip_cli = ActionClient(
                self, GripperCommand, '/gripper_action_controller/gripper_cmd')
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = State.HOME
        self.retries = 0
        self.failed_state = None
        self.grasp_point = None          # (x, y, z) in planning_frame
        self.history = []                # for the end-of-run trace

        mode = 'DRY RUN' if self.dry_run else 'LIVE'
        self.get_logger().info(f'orchestrator starting in {mode} mode, {self.cycles} cycle(s)')
        if self.dry_run and self.fail_state:
            self.get_logger().warn(f'dry run will FORCE FAILURE in state {self.fail_state}')

    # ══════════════════════════════════════════════════════════════════════
    # plumbing
    # ══════════════════════════════════════════════════════════════════════
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

    def _wait_for_target(self, state):
        """
        Wait for a FRESH object_target_frame.

        Freshness is judged by the stamp *advancing*, not by its absolute age:
        the perception node runs inference and lags /clock by a variable
        amount, and it simply stops broadcasting when it loses the target —
        so a stale-but-recent stamp must not count as a live detection.
        """
        if self.dry_run:
            ok, detail = self._stub(state)
            if ok:
                self.grasp_point = (0.60, 0.0, 0.35)     # plausible stand-in
            return ok, detail

        cam = self.get_parameter('camera_frame').value
        tgt = self.get_parameter('target_frame').value
        base = self.get_parameter('planning_frame').value
        timeout = self.timeouts[state]
        deadline = time.monotonic() + timeout
        first_stamp = None

        while time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    base, tgt, rclpy.time.Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if first_stamp is None:
                    first_stamp = stamp
                elif stamp > first_stamp:
                    t = tf.transform.translation
                    self.grasp_point = (t.x, t.y, t.z)
                    return True, (f'{tgt} live in {base} at '
                                  f'({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f})')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
            time.sleep(0.1)

        seen = 'never appeared' if first_stamp is None else 'stopped advancing'
        return False, f'{cam} -> {tgt} {seen} within {timeout:.0f}s'

    def _move_arm(self, state, point, what):
        if self.dry_run:
            return self._stub(state)
        if point is None:
            return False, 'no target point available to plan to'

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.get_parameter('planning_group').value
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        # Empty arm-only diff, never a RobotState echoed back from
        # /joint_states: under Gazebo the ros2_control backend also publishes
        # `gripper_robotiq_85_*_joint_mimic` names that are absent from the
        # URDF, and move_group *crashes* if a client sends those names back
        # inside a RobotState.  See CLAUDE.md.
        req.start_state = RobotState()
        req.start_state.is_diff = True

        frame = self.get_parameter('planning_frame').value
        offset = float(self.get_parameter('pregrasp_offset').value)

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = self.get_parameter('ee_link').value
        pc.weight = 1.0
        vol = SolidPrimitive()
        vol.type = SolidPrimitive.SPHERE
        vol.dimensions = [0.01]
        pc.constraint_region.primitives.append(vol)
        pose = PoseStamped().pose
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.position.z = float(point[2]) + offset
        pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(pose)

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = pc.link_name
        oc.orientation = yaw_to_quat(0.0)
        oc.absolute_x_axis_tolerance = 0.6
        oc.absolute_y_axis_tolerance = 0.6
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 0.5

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal.planning_options.plan_only = False
        return self._run_action(self.move_cli, goal, self.timeouts[state], what)

    def _gripper(self, state, position, what):
        if self.dry_run:
            return self._stub(state)
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter('gripper_effort').value)
        return self._run_action(self.grip_cli, goal, self.timeouts[state], what)

    # ══════════════════════════════════════════════════════════════════════
    # states — each returns (ok, detail); none of them raise
    # ══════════════════════════════════════════════════════════════════════
    def st_home(self):
        if self.dry_run:
            return self._stub(State.HOME)
        return True, 'at home, ready'

    def st_nav_to_pick(self):
        return self._navigate(State.NAV_TO_PICK,
                              self.get_parameter('pick_pose').value, 'nav to pick table')

    def st_perceive(self):
        return self._wait_for_target(State.PERCEIVE)

    def st_approach_arm(self):
        return self._move_arm(State.APPROACH_ARM, self.grasp_point, 'approach target')

    def st_grasp(self):
        return self._gripper(State.GRASP,
                             self.get_parameter('gripper_closed').value, 'close gripper')

    def st_nav_to_drop(self):
        return self._navigate(State.NAV_TO_DROP,
                              self.get_parameter('drop_pose').value, 'nav to drop table')

    def st_place_arm(self):
        point = self.grasp_point or (0.60, 0.0, 0.35)
        return self._move_arm(State.PLACE_ARM, point, 'place over drop table')

    def st_release(self):
        return self._gripper(State.RELEASE,
                             self.get_parameter('gripper_open').value, 'open gripper')

    def st_return_home(self):
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
