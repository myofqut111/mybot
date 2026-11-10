#!/usr/bin/env python3
"""Exit successfully once AMCL provides a stable map-to-robot transform.

定位门控（由 Go2 工程原样移植，硬件无关）：
持续检查 /amcl_pose 的新鲜度、协方差(不确定性)，以及 map->base_link TF
在一个时间窗内的抖动；全部达标后进程正常退出(exit 0)。
navigation.launch.py 监听本进程的退出事件，退出瞬间才激活 Nav2 生命周期，
从而保证"定位没稳定前，导航不会工作、不会乱跑"。
"""

from collections import deque
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def _angle_error(first, second):
    return abs(math.atan2(
        math.sin(first - second), math.cos(first - second)))


def _yaw(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def _pose_jitter(samples):
    position_jitter = max(
        math.hypot(first[1] - second[1], first[2] - second[2])
        for first in samples
        for second in samples
    )
    yaw_jitter = max(
        _angle_error(first[3], second[3])
        for first in samples
        for second in samples
    )
    return position_jitter, yaw_jitter


class LocalizationGate(Node):
    def __init__(self):
        super().__init__('navigation_localization_gate')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('stable_duration', 2.0)
        self.declare_parameter('pose_max_age', 5.0)
        self.declare_parameter('max_xy_std', 0.15)
        self.declare_parameter('max_yaw_std', 0.15)
        self.declare_parameter('max_position_jitter', 0.05)
        self.declare_parameter('max_yaw_jitter', 0.05)
        self._global_frame = str(self.get_parameter('global_frame').value)
        self._robot_base_frame = str(
            self.get_parameter('robot_base_frame').value)
        self._stable_duration = max(
            0.0, float(self.get_parameter('stable_duration').value))
        self._pose_max_age = max(
            0.0, float(self.get_parameter('pose_max_age').value))
        self._max_xy_std = max(
            0.0, float(self.get_parameter('max_xy_std').value))
        self._max_yaw_std = max(
            0.0, float(self.get_parameter('max_yaw_std').value))
        self._max_position_jitter = max(
            0.0, float(self.get_parameter('max_position_jitter').value))
        self._max_yaw_jitter = max(
            0.0, float(self.get_parameter('max_yaw_jitter').value))
        self._latest_pose = None
        self._latest_pose_time = 0.0
        self._latest_xy_std = float('inf')
        self._latest_yaw_std = float('inf')
        self._tf_history = deque()
        self._position_jitter = float('inf')
        self._yaw_jitter = float('inf')
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self._buffer = Buffer()
        self._listener = TransformListener(
            self._buffer, self, spin_thread=False)

    def _on_pose(self, message):
        covariance = message.pose.covariance
        if len(covariance) < 36:
            return
        if not all(math.isfinite(value) for value in covariance):
            return
        if not all(math.isfinite(value) for value in (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w)):
            return
        self._latest_pose = message
        self._latest_pose_time = time.monotonic()
        xy_variance = max(float(covariance[0]), float(covariance[7]))
        yaw_variance = float(covariance[35])
        if xy_variance >= 0.0 and yaw_variance >= 0.0:
            self._latest_xy_std = math.sqrt(xy_variance)
            self._latest_yaw_std = math.sqrt(yaw_variance)

    def _pose_is_ready(self):
        if self._latest_pose is None:
            return False
        if time.monotonic() - self._latest_pose_time > self._pose_max_age:
            return False
        return (self._latest_xy_std <= self._max_xy_std and
                self._latest_yaw_std <= self._max_yaw_std)

    def _tf_is_stable(self, now):
        try:
            transform = self._buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time())
        except TransformException:
            self._tf_history.clear()
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sample = (now, translation.x, translation.y, _yaw(rotation))
        if not all(math.isfinite(value) for value in sample):
            self._tf_history.clear()
            return False
        self._tf_history.append(sample)

        cutoff = now - self._stable_duration
        # Keep one sample at or before the window boundary so its full duration
        # is measured instead of shortened by the 10 Hz polling interval.
        while (len(self._tf_history) > 1 and
               self._tf_history[1][0] <= cutoff):
            self._tf_history.popleft()

        if now - self._tf_history[0][0] < self._stable_duration:
            return False

        self._position_jitter, self._yaw_jitter = _pose_jitter(
            self._tf_history)
        return (self._position_jitter <= self._max_position_jitter and
                self._yaw_jitter <= self._max_yaw_jitter)

    def wait(self):
        next_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            pose_ready = self._pose_is_ready()
            tf_stable = self._tf_is_stable(now) if pose_ready else False
            if tf_stable:
                self.get_logger().info(
                    'Localization ready: xy_std={:.3f}m yaw_std={:.3f}rad, '
                    'TF jitter={:.3f}m/{:.3f}rad over {:.1f}s; starting Nav2.'.format(
                        self._latest_xy_std,
                        self._latest_yaw_std,
                        self._position_jitter,
                        self._yaw_jitter,
                        self._stable_duration,
                    ))
                return
            if not pose_ready:
                self._tf_history.clear()
            if now >= next_log:
                self.get_logger().info(
                    'Waiting for stable AMCL: xy_std={:.3f}/{:.3f}m, '
                    'yaw_std={:.3f}/{:.3f}rad, TF jitter={:.3f}/{:.3f}m '
                    'and {:.3f}/{:.3f}rad.'.format(
                        self._latest_xy_std, self._max_xy_std,
                        self._latest_yaw_std, self._max_yaw_std,
                        self._position_jitter, self._max_position_jitter,
                        self._yaw_jitter, self._max_yaw_jitter,
                    ))
                next_log = now + 5.0


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGate()
    try:
        node.wait()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
