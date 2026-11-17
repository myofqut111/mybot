#!/usr/bin/env python3
"""扫描门控：把激光原样转发给 slam，只在"打滑那几帧"掐断。

为什么需要它
------------
slam_toolbox 只有 `/pause_new_measurements` 一个开关，语义是"全局暂停"。
车打滑时用它，会连带把正常路段的建图一起停掉（现场表现：地图彻底不动）。

所以把"丢坏帧"和"建图"拆开：雷达 → 本节点 → `/scan_filtered` → slam。
打滑时本节点不转发，其余时候原样通过。**最多关 `max_closed_s` 秒**，
到点强制放行——任何情况下都不允许把建图掐死。

放在导航包而不是探索包：它属于建图链路，探索节点只是"告诉它什么时候
别转发"。这样即使不跑探索，`/scan_filtered` 也一直有数据。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('control_topic', '/explore/scan_gate')
        self.declare_parameter('max_closed_s', 5.0)
        self.declare_parameter('silence_warn_s', 2.0)

        self._in = str(self.get_parameter('input_topic').value)
        self._out = str(self.get_parameter('output_topic').value)
        self._ctl = str(self.get_parameter('control_topic').value)
        self._max_closed = float(self.get_parameter('max_closed_s').value)
        self._silence_warn = float(self.get_parameter('silence_warn_s').value)

        self._closed = False
        self._closed_at = None
        self._closed_reason = ''
        self._dropped = 0
        self._forced = 0
        self._forwarded = 0
        self._last_input = None
        self._warned_silent = False

        self._pub = self.create_publisher(
            LaserScan, self._out, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self._in, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, self._ctl, self._on_control, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            '扫描门控已启动：%s → %s（控制话题 %s，最长关闭 %.1f s）'
            % (self._in, self._out, self._ctl, self._max_closed))

    # ------------------------------------------------------------------
    # 控制
    # ------------------------------------------------------------------
    def _on_control(self, msg):
        if bool(msg.data):
            self.close('探索节点报告打滑')
        else:
            self.open('车已恢复移动')

    def close(self, reason=''):
        if self._closed:
            return
        self._closed = True
        self._closed_reason = reason
        self._closed_at = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            '扫描门控：关闭（%s）—— 打滑期间这几帧不进图' % (reason or '外部请求'))

    def open(self, reason='', forced=False):
        if not self._closed:
            return
        closed_for = 0.0
        if self._closed_at is not None:
            closed_for = (self.get_clock().now().nanoseconds / 1e9
                          - self._closed_at)
        self._closed = False
        self._closed_at = None
        if forced:
            self._forced += 1
            self.get_logger().warning(
                '扫描门控：打开（已关闭 %.1f s，到上限强制放行，%s）'
                % (closed_for, reason or '防止建图被掐死'))
        else:
            self.get_logger().info(
                '扫描门控：打开（已关闭 %.1f s，%s）' % (closed_for, reason or ''))

    # ------------------------------------------------------------------
    # 转发
    # ------------------------------------------------------------------
    def _on_scan(self, msg):
        self._last_input = self.get_clock().now().nanoseconds / 1e9
        self._warned_silent = False
        if self._closed:
            self._dropped += 1
            return
        self._forwarded += 1
        self._pub.publish(msg)

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._closed and self._closed_at is not None:
            if now - self._closed_at >= self._max_closed:
                self.open('超过 %.1f s' % self._max_closed, forced=True)
        if (self._last_input is None or now - self._last_input > self._silence_warn) \
                and not self._warned_silent:
            self._warned_silent = True
            self.get_logger().warning(
                '扫描门控：%s 已经 %.0f s 没有数据，slam 将收不到新帧'
                % (self._in, self._silence_warn))


def main(args=None):
    rclpy.init(args=args)
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:                            # noqa: N802
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
