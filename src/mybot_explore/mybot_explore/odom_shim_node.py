#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_shim：接管 odom -> base_link 的发布权，打滑时冻结、恢复时重锚。

为什么要它
----------
EKF 会把轮子打滑时的假位移一路积进 `odom→base_link`。slam 那段
`map→odom` 本可以抵消，但一旦建图被暂停 / 跟不上，复合位姿就漂，
恢复后同一面墙被拼成两处（"两张图拼在一起"）。

把 `odom→base_link` 换成我们发：平时透传，打滑时冻结，恢复时把
"冻结位姿 - 当前原始位姿"记成 offset 永久减掉，假位移就进不了位姿链。

用法
----
* 巡检（默认）：不用它。`ekf.yaml` 里 `publish_tf: true`。
* 探索：`ekf_publish_tf:=false odom_shim_enabled:=true`，
  由本节点发 `odom→base_link`；收到 `/explore/scan_gate` 的 true/false
  就冻结 / 重锚。

安全
----
输入断了就按"透传最近一次位姿"处理并告警；绝不静默停止发 TF，
否则整条定位链会断。
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from . import odom_anchor


class OdomShim(Node):

    def __init__(self):
        super().__init__('odom_shim')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('gate_topic', '/explore/scan_gate')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('warn_after_s', 2.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._gate_topic = str(self.get_parameter('gate_topic').value)
        self._parent = str(self.get_parameter('parent_frame').value)
        self._child = str(self.get_parameter('child_frame').value)
        self._warn_after = float(self.get_parameter('warn_after_s').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._anchor = odom_anchor.OdomAnchor()
        self._last_msg = None
        self._last_rx = None
        self._warned = False

        self._broadcaster = TransformBroadcaster(self)
        self._state_pub = self.create_publisher(String, '/odom_shim/state', 10)

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(Bool, self._gate_topic, self._on_gate, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self.get_logger().info(
            'odom_shim 已启动：%s → %s（源 %s，冻结信号 %s）'
            % (self._parent, self._child, self._odom_topic, self._gate_topic))

    # ---- 订阅 -----------------------------------------------------------
    def _on_odom(self, msg):
        pose = odom_anchor.pose_of(msg)
        if pose is None:
            return
        self._last_msg = msg
        self._last_rx = self.get_clock().now()
        self._warned = False
        self._anchor.update(pose)

    def _on_gate(self, msg):
        """true = 冻结（打滑），false = 重锚后恢复。"""
        if self._last_msg is None:
            self.get_logger().warning('收到冻结信号但还没有里程计，先忽略')
            return
        pose = odom_anchor.pose_of(self._last_msg)
        if msg.data:
            if not self._anchor.frozen:
                self._anchor.freeze(pose)
                self.get_logger().warning(
                    '里程计已冻结在 (%.2f, %.2f, %.2f rad)，打滑期间的位移不再计入位姿'
                    % self._anchor.frozen_pose)
        else:
            if self._anchor.frozen:
                before = self._anchor.snapshot()
                self._anchor.resume(pose)
                after = self._anchor.snapshot()
                self.get_logger().info(
                    '里程计已重锚：丢掉假位移 %.2f m（累计 %.2f m，第 %d 次），继续从当前位姿走'
                    % (after['discarded_m'] - before['discarded_m'],
                       after['discarded_m'], after['resumes']))

    # ---- 定时发布 -------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        if self._last_msg is None:
            if not self._warned:
                self.get_logger().warning(
                    '还没收到 %s，暂时没有 odom→%s（检查 EKF 是否在跑）'
                    % (self._odom_topic, self._child))
                self._warned = True
            return
        if (self._last_rx is not None
                and not self._anchor.frozen):
            age = (now - self._last_rx).nanoseconds / 1e9
            if age > self._warn_after:
                self.get_logger().warning(
                    '里程计断流 %.1f 秒，仍在按最后一次位姿发 TF' % age)

        pose = odom_anchor.pose_of(self._last_msg)
        shaped = self._anchor.apply(pose)
        if shaped is None:
            return
        x, y, yaw = shaped
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self._parent
        transform.child_frame_id = self._child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        quat = odom_anchor._quat_of(yaw)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self._broadcaster.sendTransform(transform)

        state = self._anchor.snapshot()
        state['age_s'] = round((now - self._last_rx).nanoseconds / 1e9, 2) if self._last_rx else None
        message = String()
        message.data = repr(state).replace("'", '"')
        self._state_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdomShim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:          # 被外部 SIGTERM 时 rclpy 会抛这个
        if 'context' not in str(error).lower():
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
