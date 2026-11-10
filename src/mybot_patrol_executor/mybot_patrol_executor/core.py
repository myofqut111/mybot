"""ROS-independent patrol ordering and pose validation helpers."""

import math
import random
from typing import List, Sequence


ONCE = 0
REPEAT = 1
RANDOM = 2
VALID_MODES = frozenset((ONCE, REPEAT, RANDOM))


def visit_order(point_count: int, mode: int, rng=None) -> List[int]:
    """Return one loop's waypoint order for the selected execution mode."""
    if point_count < 1:
        raise ValueError('route has no patrol points')
    if mode not in VALID_MODES:
        raise ValueError('unsupported patrol mode: %d' % mode)
    order = list(range(point_count))
    if mode == RANDOM:
        (rng or random).shuffle(order)
    return order


def loop_limit(mode: int, repeat_count: int) -> int:
    """Return total loops; zero means unlimited for repeat/random modes."""
    if mode not in VALID_MODES:
        raise ValueError('unsupported patrol mode: %d' % mode)
    if repeat_count < 0:
        raise ValueError('repeat_count must not be negative')
    if mode == ONCE:
        return 1
    return repeat_count


def yaw_from_quaternion(values: Sequence[float]) -> float:
    """Extract planar yaw from an XYZW quaternion."""
    if len(values) != 4:
        raise ValueError('quaternion must contain four values')
    x, y, z, w = (float(value) for value in values)
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError('quaternion contains a non-finite value')
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError('quaternion has zero length')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def angle_error(current: float, target: float) -> float:
    """Return the absolute shortest angular distance in radians."""
    return abs(math.atan2(
        math.sin(current - target), math.cos(current - target)))


def pose_residual(current_pose, target_pose):
    """Return planar position and yaw error for two Pose-like objects."""
    dx = current_pose.position.x - target_pose.position.x
    dy = current_pose.position.y - target_pose.position.y
    position_error = math.hypot(dx, dy)
    current_q = current_pose.orientation
    target_q = target_pose.orientation
    yaw_error = angle_error(
        yaw_from_quaternion((current_q.x, current_q.y, current_q.z, current_q.w)),
        yaw_from_quaternion((target_q.x, target_q.y, target_q.z, target_q.w)),
    )
    return position_error, yaw_error
