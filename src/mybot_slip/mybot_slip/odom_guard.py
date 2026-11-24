"""方案 §10：按打滑严重程度分三档处理编码器里程。

- **轻度**（疑似）：原样转发，只把协方差放大，降低融合权重；
- **持续/严重**（确认打滑、恢复动作执行中）：**冻结**——位姿保持最后一帧
  好值、速度清零，EKF 看到"没动"，融合位姿就不再跟着打滑漂，TF 仍然连续；
- **激光与 IMU 也不可信**：同上冻结，并由节点上报、停车待人工（保守模式）。

方案原文要求「SLAM 和 TF 不中断」，所以任何一档都照发消息，只是内容不同；
不像"停止发 odom"那样会把 EKF 饿死、把 TF 拉断。
"""
import copy

DEFAULTS = {
    'enabled': True,
    'in_topic': '/odom',
    'out_topic': '/odom_guarded',
    # 打滑时把位置方差放大多少倍（标准差约放大 sqrt(100)=10 倍）
    'position_factor': 100.0,
    'yaw_factor': 100.0,
    # 原来的协方差若是 0（很多底盘不发），放大 0 还是 0，给个下限
    'min_position_var': 1e-4,
    'min_yaw_var': 1e-4,
}


class OdomCovarianceGuard(object):
    def __init__(self, node, params=None):
        p = dict(DEFAULTS)
        if params:
            p.update({k: v for k, v in params.items() if v is not None})
        self.p = p
        self.node = node
        self._inflated = False
        self._frozen = False
        self._last_pose = None
        self._offset = None
        self._reanchor_pending = False
        self.published = 0
        self.frozen_frames = 0
        # ROS 消息类型延迟导入：单测（不 source ROS）也能验证协方差算术
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile
        self._pub = node.create_publisher(
            Odometry, p['out_topic'], QoSProfile(depth=20))
        node.create_subscription(Odometry, p['in_topic'], self._on_odom,
                                 QoSProfile(depth=20))

    @property
    def inflated(self):
        return self._inflated

    @property
    def frozen(self):
        return self._frozen

    def subscriber_count(self):
        """有几路在听 /odom_guarded。为 0 说明 EKF 还在听 /odom，这层防护是空转。"""
        try:
            return int(self._pub.get_subscription_count())
        except Exception:                            # noqa: BLE001
            return -1

    def set_inflated(self, want):
        """异常状态置 True；回到 NORMAL 置 False。"""
        want = bool(want)
        if want == self._inflated:
            return False
        self._inflated = want
        self.node.get_logger().warn(
            '编码器里程协方差：%s（位置 ×%.0f，偏航 ×%.0f）'
            % ('放大，降低融合权重' if want else '恢复正常',
               self.p['position_factor'], self.p['yaw_factor']))
        return True

    def set_frozen(self, want):
        """方案 §10 严重档：冻结里程对融合位姿的更新（位姿保持、速度清零）。"""
        want = bool(want)
        if want == self._frozen:
            return False
        self._frozen = want
        if want:
            self.node.get_logger().warn(
                '编码器里程：冻结（保持最后一帧位姿、速度清零，TF 仍然连续），'
                '位姿改由 IMU/激光维持')
        else:
            self.node.get_logger().warn('编码器里程：解冻，恢复正常转发')
            self._reanchor_pending = True
        return True

    def _on_odom(self, msg):
        out = copy.deepcopy(msg)
        if self._frozen and self._last_pose is not None:
            # 位姿保持：EKF 的 odom0 是靠相邻两帧的位姿差算速度的，
            # 位姿不动 → 速度 0 → 融合位姿不再跟着打滑往前走。
            out.pose.pose.position.x = self._last_pose.position.x
            out.pose.pose.position.y = self._last_pose.position.y
            out.pose.pose.position.z = self._last_pose.position.z
            out.pose.pose.orientation = copy.deepcopy(self._last_pose.orientation)
            out.twist.twist.linear.x = 0.0
            out.twist.twist.linear.y = 0.0
            out.twist.twist.linear.z = 0.0
            out.twist.twist.angular.x = 0.0
            out.twist.twist.angular.y = 0.0
            out.twist.twist.angular.z = 0.0
            self.frozen_frames += 1
        else:
            self._apply_offset(out)
            self._last_pose = copy.deepcopy(out.pose.pose)
        if self._inflated or self._frozen:
            self._inflate(out)
        self._pub.publish(out)
        self.published += 1

    def _apply_offset(self, out):
        """解冻后的第一个消息：把打滑期间攒下的位移从原点去掉。

        不这么做的话，EKF 会在解冻那一帧看到一个 1 m 级的位姿跳变，
        融合位姿会直接瞬移；重锚之后位姿保持连续，后续增量不受影响。
        """
        p = out.pose.pose.position
        if self._reanchor_pending and self._last_pose is not None:
            self._reanchor_pending = False
            self._offset = (p.x - self._last_pose.position.x,
                            p.y - self._last_pose.position.y)
            self.node.get_logger().warn(
                '编码器里程：解冻后重锚，去掉打滑期间累计的 %.2f m / %.2f m'
                % (self._offset[0], self._offset[1]))
        if self._offset is not None:
            p.x -= self._offset[0]
            p.y -= self._offset[1]

    def _inflate(self, msg):
        p = self.p
        cov = list(msg.pose.covariance)
        if len(cov) == 36:
            pvar = max(cov[0], p['min_position_var']) * p['position_factor']
            qvar = max(cov[7], p['min_position_var']) * p['position_factor']
            yvar = max(cov[35], p['min_yaw_var']) * p['yaw_factor']
            cov[0] = cov[7] = pvar
            cov[35] = yvar
            msg.pose.covariance = cov
        tcov = list(msg.twist.covariance)
        if len(tcov) == 36:
            vvar = max(tcov[0], p['min_position_var']) * p['position_factor']
            wvar = max(tcov[35], p['min_yaw_var']) * p['yaw_factor']
            tcov[0] = vvar
            tcov[35] = wvar
            msg.twist.covariance = tcov
