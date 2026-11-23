"""方案 §4：打滑期间把编码器里程的协方差放大，让融合少信它。

方案原文是「放大其协方差、降低融合权重」，同时「SLAM 和 TF 不中断」——
也就是**不改流程、只改可信度**。做法是原样转发 `/odom`，但在异常窗口里
把 pose/twist 的协方差乘一个系数，EKF 看到的就是"这一路现在很不可靠"。

为什么不在打滑时直接不发 odom：EKF 会因为没有输入而停止输出
`odometry/filtered`，TF 就断了，正是方案要避免的事。
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
        self.published = 0
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

    def _on_odom(self, msg):
        out = copy.deepcopy(msg)
        if self._inflated:
            self._inflate(out)
        self._pub.publish(out)
        self.published += 1

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
