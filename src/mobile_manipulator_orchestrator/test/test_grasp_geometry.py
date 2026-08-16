# test_grasp_geometry.py
# ─────────────────────────────────────────────────────────────────────────────
# Regression tests for the orchestrator's grasp geometry helpers.
#
# topdown_quat() earned a test the hard way.  geometry_msgs/Quaternion defaults
# to w = 1, and the original version assigned only x and y — so the "top-down"
# orientation every pose goal and every Cartesian waypoint asked for was
# (1, 0, 0, 1), which normalizes to a 90 deg rotation about X: the gripper
# pointed sideways along -Y instead of down at the floor.  MoveIt said so
# ("Orientation constraint for link 'arm_tool0' is probably incorrect:
# 1.000000, 0.000000, 0.000000, 1.000000. Assuming identity instead") and then
# planned anyway, so the failure surfaced 80 seconds later as a Cartesian
# descent that swept the forearm into the chassis and timed out.
#
# The assertion that catches this is on the resulting ROTATION, not on the
# quaternion's components: it is the tool's z axis in the world that has to
# point at the floor, and that is the thing a reader can check against the
# robot.
# ─────────────────────────────────────────────────────────────────────────────
import math

import pytest

from mobile_manipulator_orchestrator.warehouse_orchestrator import (
    topdown_quat,
    wrap,
    yaw_to_quat,
)


def rotation_matrix(q):
    """Rotation matrix of a geometry_msgs/Quaternion, normalized first."""
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    assert n > 0.0, 'zero quaternion'
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def tool_axis(q, column):
    return [rotation_matrix(q)[row][column] for row in range(3)]


@pytest.mark.parametrize('yaw', [0.0, 0.4, -0.4, math.pi / 2, -math.pi / 2])
def test_topdown_points_the_tool_at_the_floor(yaw):
    """The tool's +Z must point straight down for every yaw."""
    z_axis = tool_axis(topdown_quat(yaw), 2)
    assert z_axis[0] == pytest.approx(0.0, abs=1e-9)
    assert z_axis[1] == pytest.approx(0.0, abs=1e-9)
    assert z_axis[2] == pytest.approx(-1.0, abs=1e-9)


def test_topdown_quaternion_is_a_unit_quaternion():
    """An unnormalized constraint is silently reinterpreted by MoveIt."""
    q = topdown_quat(0.3)
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    assert norm == pytest.approx(1.0, abs=1e-9)


def test_topdown_yaw_spins_the_fingers_about_the_approach_axis():
    """Yaw must rotate the jaws in the horizontal plane, not tilt them."""
    straddle_at_zero = tool_axis(topdown_quat(0.0), 0)
    straddle_at_90 = tool_axis(topdown_quat(math.pi / 2), 0)
    # At yaw 0 the fingers straddle along world x; at 90 deg, along world y.
    assert straddle_at_zero[0] == pytest.approx(1.0, abs=1e-9)
    assert straddle_at_90[1] == pytest.approx(1.0, abs=1e-9)


def test_yaw_to_quat_matches_a_planar_rotation():
    """Nav2 goals use yaw_to_quat; a wrong w there aims the whole robot."""
    q = yaw_to_quat(math.pi / 2)
    x_axis = tool_axis(q, 0)
    assert x_axis[0] == pytest.approx(0.0, abs=1e-9)
    assert x_axis[1] == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize('angle,expected', [
    (3 * math.pi, math.pi),
    (-3 * math.pi, math.pi),
    (0.5, 0.5),
])
def test_wrap_folds_into_pi(angle, expected):
    assert abs(wrap(angle)) == pytest.approx(abs(expected), abs=1e-9)


# ── grasp offset vs the bench ────────────────────────────────────────────────
# The second Phase 9 defect: grasp_offset was measured to the fingertip link
# origin (0.1093 m below tool0) when the pad's collision mesh reaches 0.1603 m
# below tool0 open and 0.1694 m closed.  The descent therefore drove the pads
# through the bench top, the arm stalled against it, and MoveIt cancelled the
# trajectory.  These tests assert the clearance the geometry has to leave.

from mobile_manipulator_orchestrator.warehouse_orchestrator import (  # noqa: E402
    GRASP_OFFSET,
    PAD_REACH_CLOSED,
    PAD_REACH_OPEN,
    pad_clearance,
)


def test_pads_clear_the_bench_while_descending():
    """Jaws open, coming down around the target: must not touch the surface."""
    assert pad_clearance(GRASP_OFFSET, pad_reach=PAD_REACH_OPEN) > 0.005


def test_pads_clear_the_bench_while_closing():
    """Closing swings the pads a further 9 mm down; still must not touch."""
    assert pad_clearance(GRASP_OFFSET, pad_reach=PAD_REACH_CLOSED) > 0.005


def test_pads_straddle_the_object_equator():
    """
    The pad face must span the object's widest point or it cannot grip it.

    Pad face spans [tool0 - pad_reach, tool0 - pad_reach + 0.057] (the mesh is
    57 mm tall); the equator is at the object's centre.
    """
    tool0_above_centre = GRASP_OFFSET
    pad_bottom_below_centre = tool0_above_centre - PAD_REACH_CLOSED
    pad_top_below_centre = pad_bottom_below_centre + 0.057
    assert pad_bottom_below_centre < 0.0 < pad_top_below_centre


def test_the_old_offset_would_have_hit_the_bench():
    """Guards the regression itself: 0.115 must read as a collision."""
    assert pad_clearance(0.115, pad_reach=PAD_REACH_OPEN) < 0.0


# ── Cartesian retiming ───────────────────────────────────────────────────────
# The third Phase 9 defect: /compute_cartesian_path returns a trajectory
# parameterized at 100 % of the joint limits (its request has no scaling field
# in Humble), so the Cartesian segments ran four times faster than every pose
# goal.  Under gz_ros2_control the arm lags in proportion to commanded speed,
# so the controller was still catching up long after the nominal end and MoveIt
# cancelled it.  retime() applies the scaling the service will not.

from builtin_interfaces.msg import Duration  # noqa: E402
from trajectory_msgs.msg import (  # noqa: E402
    JointTrajectory,
    JointTrajectoryPoint,
)

from mobile_manipulator_orchestrator.warehouse_orchestrator import retime  # noqa: E402


def _trajectory():
    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS_FOR_TEST)
    for i, (sec, nsec) in enumerate(((0, 500_000_000), (1, 250_000_000), (2, 0))):
        p = JointTrajectoryPoint()
        p.time_from_start = Duration(sec=sec, nanosec=nsec)
        p.positions = [0.1 * i] * 6
        p.velocities = [0.8] * 6
        p.accelerations = [1.6] * 6
        traj.points.append(p)
    return traj


ARM_JOINTS_FOR_TEST = ['j%d' % i for i in range(6)]


def test_retime_stretches_time_and_slows_velocities():
    traj = retime(_trajectory(), 4.0)
    stamps = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
              for p in traj.points]
    assert stamps == pytest.approx([2.0, 5.0, 8.0])
    for p in traj.points:
        assert p.velocities == pytest.approx([0.2] * 6)
        assert p.accelerations == pytest.approx([0.1] * 6)


def test_retime_keeps_nanoseconds_in_range():
    """A nanosec field over 1e9 is silently wrong, not an error."""
    traj = retime(_trajectory(), 3.0)
    for p in traj.points:
        assert 0 <= p.time_from_start.nanosec < 1_000_000_000
        assert p.time_from_start.sec >= 0


def test_retime_is_monotonic():
    traj = retime(_trajectory(), 4.0)
    stamps = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
              for p in traj.points]
    assert stamps == sorted(stamps)


def test_retime_identity_at_scale_one():
    before = [(p.time_from_start.sec, p.time_from_start.nanosec)
              for p in _trajectory().points]
    after = [(p.time_from_start.sec, p.time_from_start.nanosec)
             for p in retime(_trajectory(), 1.0).points]
    assert before == after
