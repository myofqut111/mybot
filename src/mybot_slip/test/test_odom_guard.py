"""odom_guard 的单测：不打滑时原样转发，打滑时协方差被放大。

不起 ROS：给一个只实现 create_publisher/create_subscription/get_logger
的假节点就够了，这里验证的是协方差算术，不是 rclpy。
"""
import unittest

from mybot_slip.odom_guard import DEFAULTS, OdomCovarianceGuard


class _Logger(object):
    def __init__(self):
        self.warns = []

    def warn(self, text):
        self.warns.append(text)

    def info(self, text):
        pass


class _Node(object):
    def __init__(self):
        self.logger = _Logger()
        self.pub = None
        self.sub = None

    def create_publisher(self, msg_type, topic, qos):
        self.pub = _Pub()
        return self.pub

    def create_subscription(self, msg_type, topic, cb, qos):
        self.sub = _Sub(cb)
        return self.sub

    def get_logger(self):
        return self.logger


class _Pub(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Sub(object):
    def __init__(self, cb):
        self.cb = cb


class _Cov(object):
    def __init__(self, diag):
        cov = [0.0] * 36
        for i, v in enumerate(diag):
            cov[i * 7] = v
        self.covariance = cov


class _Pose(object):
    def __init__(self, pos_var=0.01, yaw_var=0.01):
        self.covariance = _Cov([pos_var, pos_var, 0.0, 0.0, 0.0, yaw_var]).covariance


class _Twist(object):
    def __init__(self, lin_var=0.01, ang_var=0.01):
        self.covariance = _Cov([lin_var, lin_var, 0.0, 0.0, 0.0, ang_var]).covariance


class _Odom(object):
    def __init__(self, pos_var=0.01, yaw_var=0.01):
        self.pose = _Pose(pos_var, yaw_var)
        self.twist = _Twist(pos_var, yaw_var)


class OdomGuardTest(unittest.TestCase):
    def setUp(self):
        self.node = _Node()
        self.guard = OdomCovarianceGuard(self.node)

    def test_passthrough_when_normal(self):
        msg = _Odom()
        self.node.sub.cb(msg)
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.pose.covariance[0], 0.01)
        self.assertFalse(self.guard.inflated)

    def test_inflate_multiplies_position_variance(self):
        self.guard.set_inflated(True)
        self.node.sub.cb(_Odom())
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.pose.covariance[0],
                               0.01 * DEFAULTS['position_factor'])
        self.assertAlmostEqual(out.pose.covariance[35],
                               0.01 * DEFAULTS['yaw_factor'])

    def test_inflate_uses_floor_when_covariance_is_zero(self):
        # 很多底盘压根不发协方差（0），乘多少还是 0，必须用下限兜住
        self.guard.set_inflated(True)
        self.node.sub.cb(_Odom(pos_var=0.0, yaw_var=0.0))
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(
            out.pose.covariance[0],
            DEFAULTS['min_position_var'] * DEFAULTS['position_factor'])
        self.assertAlmostEqual(
            out.pose.covariance[35],
            DEFAULTS['min_yaw_var'] * DEFAULTS['yaw_factor'])

    def test_restore_returns_to_original(self):
        self.guard.set_inflated(True)
        self.guard.set_inflated(False)
        self.node.sub.cb(_Odom())
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.pose.covariance[0], 0.01)

    def test_set_inflated_reports_only_on_change(self):
        self.assertTrue(self.guard.set_inflated(True))
        self.assertFalse(self.guard.set_inflated(True))
        self.assertTrue(self.guard.set_inflated(False))

    def test_twist_covariance_also_inflated(self):
        self.guard.set_inflated(True)
        self.node.sub.cb(_Odom())
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.twist.covariance[0],
                               0.01 * DEFAULTS['position_factor'])

    def test_original_message_not_modified(self):
        msg = _Odom()
        self.guard.set_inflated(True)
        self.node.sub.cb(msg)
        self.assertAlmostEqual(msg.pose.covariance[0], 0.01)


if __name__ == '__main__':
    unittest.main()
