"""AMCL pose sampling and averaging without ROS-specific dependencies."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Iterable, Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseSample:
    """One received pose and the local monotonic receipt time."""

    receipt_time: float
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    position: Vector3
    orientation: Quaternion


def _normalized(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 1.0e-12:
        raise ValueError('pose contains a zero-length quaternion')
    return tuple(component / norm for component in quaternion)  # type: ignore


def _to_rpy(quaternion: Quaternion) -> Vector3:
    x, y, z, w = _normalized(quaternion)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(sin_yaw, cos_yaw)


def _from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return _normalized((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def average_samples(samples: Iterable[PoseSample]) -> PoseSample:
    """Average position and yaw, retaining the newest roll, pitch and stamp."""
    values = list(samples)
    if not values:
        raise ValueError('cannot average an empty pose sample set')

    newest = values[-1]
    count = float(len(values))
    position = tuple(
        sum(sample.position[index] for sample in values) / count
        for index in range(3)
    )
    yaws = [_to_rpy(sample.orientation)[2] for sample in values]
    yaw = math.atan2(
        sum(math.sin(value) for value in yaws),
        sum(math.cos(value) for value in yaws),
    )
    roll, pitch, _ = _to_rpy(newest.orientation)
    return PoseSample(
        receipt_time=newest.receipt_time,
        stamp_sec=newest.stamp_sec,
        stamp_nanosec=newest.stamp_nanosec,
        frame_id=newest.frame_id,
        position=position,  # type: ignore
        orientation=_from_rpy(roll, pitch, yaw),
    )


class PoseSampleBuffer:
    """Fixed-size pose window with freshness filtering."""

    def __init__(self, max_samples: int) -> None:
        if max_samples < 1:
            raise ValueError('max_samples must be at least one')
        self._samples: Deque[PoseSample] = deque(maxlen=max_samples)

    def add(self, sample: PoseSample) -> None:
        if self._samples and self._samples[-1].frame_id != sample.frame_id:
            self._samples.clear()
        self._samples.append(sample)

    def fresh_average(
        self, now: float, max_age: float
    ) -> Optional[PoseSample]:
        if max_age <= 0.0:
            raise ValueError('max_age must be greater than zero')
        if not self._samples:
            return None
        if now - self._samples[-1].receipt_time > max_age:
            return None
        fresh = [
            sample for sample in self._samples
            if 0.0 <= now - sample.receipt_time <= max_age
        ]
        if not fresh:
            return None
        return average_samples(fresh)
