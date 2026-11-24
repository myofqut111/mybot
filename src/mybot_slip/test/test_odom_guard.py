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


class _Vec3(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quat(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _PoseMsg(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.position = _Vec3(x, y, 0.0)
        self.orientation = _Quat(0.0, 0.0, yaw, 1.0)


class _TwistMsg(object):
    def __init__(self, vx=0.0, wz=0.0):
        self.linear = _Vec3(vx, 0.0, 0.0)
        self.angular = _Vec3(0.0, 0.0, wz)


class _Odom(object):
    def __init__(self, pos_var=0.01, yaw_var=0.01, x=0.0, y=0.0, vx=0.15, wz=0.0):
        self.pose = _Pose(pos_var, yaw_var)
        self.pose.pose = _PoseMsg(x, y)
        self.twist = _Twist(pos_var, yaw_var)
        self.twist.twist = _TwistMsg(vx, wz)


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

    # ---------- 方案 §10 严重档：冻结里程对融合位姿的更新 ----------
    def test_freeze_holds_last_pose(self):
        self.node.sub.cb(_Odom(x=1.0, y=2.0))
        self.assertAlmostEqual(self.node.pub.sent[-1].pose.pose.position.x, 1.0)
        self.guard.set_frozen(True)
        self.node.sub.cb(_Odom(x=2.5, y=2.0))          # 打滑：odom 报又走了 1.5 m
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(out.pose.pose.position.y, 2.0)
        self.assertEqual(self.guard.frozen_frames, 1)

    def test_freeze_zeroes_twist(self):
        self.node.sub.cb(_Odom(x=0.0, vx=0.15, wz=0.4))
        self.guard.set_frozen(True)
        self.node.sub.cb(_Odom(x=0.2, vx=0.15, wz=0.4))
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.twist.twist.linear.x, 0.0)
        self.assertAlmostEqual(out.twist.twist.angular.z, 0.0)

    def test_freeze_still_publishes_and_inflates(self):
        self.node.sub.cb(_Odom(x=0.0))
        before = self.guard.published
        self.guard.set_frozen(True)
        self.node.sub.cb(_Odom(x=0.5))
        self.assertEqual(self.guard.published, before + 1)
        self.assertAlmostEqual(self.node.pub.sent[-1].pose.covariance[0],
                               0.01 * DEFAULTS['position_factor'])

    def test_unfreeze_resumes_forwarding(self):
        self.node.sub.cb(_Odom(x=1.0))
        self.guard.set_frozen(True)
        self.node.sub.cb(_Odom(x=5.0))
        before = self.guard.published
        self.guard.set_frozen(False)
        self.node.sub.cb(_Odom(x=5.2))
        self.assertEqual(self.guard.published, before + 1)
        self.assertAlmostEqual(self.guard.frozen_frames, 1)

    def test_unfreeze_reanchors_without_jump(self):
        """解冻那一帧正好接在冻结位姿上（1.0），不能跳到 5.2。"""
        self.node.sub.cb(_Odom(x=1.0))
        self.guard.set_frozen(True)
        self.node.sub.cb(_Odom(x=5.0))
        self.guard.set_frozen(False)
        self.node.sub.cb(_Odom(x=5.2))
        out = self.node.pub.sent[-1]
        self.assertAlmostEqual(out.pose.pose.position.x, 1.0)

    def test_reanchor_offset_persists(self):
        self.node.sub.cb(_Odom(x=1.0))
        self.guard.set_frozen(True)
        self.node.sub.cb(_Odom(x=5.0))
        self.guard.set_frozen(False)
        self.node.sub.cb(_Odom(x=5.2))     # 重锚：偏移 4.2
        self.node.sub.cb(_Odom(x=6.0))     # 增量照常：1.0 + 0.8
        self.assertAlmostEqual(self.node.pub.sent[-1].pose.pose.position.x, 1.8)

    def test_set_frozen_reports_only_on_change(self):
        self.assertTrue(self.guard.set_frozen(True))
        self.assertFalse(self.guard.set_frozen(True))
        self.assertTrue(self.guard.set_frozen(False))


if __name__ == '__main__':
    unittest.main()
