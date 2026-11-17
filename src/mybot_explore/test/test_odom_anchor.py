# -*- coding: utf-8 -*-
"""odom_anchor 与 odom_shim 的契约测试（纯逻辑 + 静态检查，不需要 ROS）。"""

import math
import os
import unittest

from mybot_explore import odom_anchor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


class WrapTest(unittest.TestCase):

    def test_wrap_into_pi(self):
        self.assertAlmostEqual(odom_anchor._wrap(3.5), 3.5 - 2 * math.pi, places=6)
        self.assertAlmostEqual(odom_anchor._wrap(-3.5), 2 * math.pi - 3.5, places=6)
        self.assertAlmostEqual(odom_anchor._wrap(0.3), 0.3, places=6)

    def test_quat_round_trip(self):
        for yaw in (0.0, 0.7, -1.2, 3.0):
            quat = odom_anchor._quat_of(yaw)
            self.assertAlmostEqual(odom_anchor._yaw_of(quat), yaw, places=6)


class AnchorBasicTest(unittest.TestCase):

    def test_passthrough_when_free(self):
        anchor = odom_anchor.OdomAnchor()
        self.assertEqual(anchor.update((1.0, 2.0, 0.3)), (1.0, 2.0, 0.3))
        self.assertFalse(anchor.frozen)

    def test_none_is_safe(self):
        anchor = odom_anchor.OdomAnchor()
        self.assertIsNone(anchor.update(None))
        self.assertIsNone(anchor.apply(None))


class FrozenTest(unittest.TestCase):

    def test_freeze_holds_pose_against_fake_motion(self):
        anchor = odom_anchor.OdomAnchor()
        anchor.update((0.4, 0.0, 0.0))
        anchor.freeze((0.4, 0.0, 0.0))
        snap = anchor.snapshot()
        self.assertTrue(snap['frozen'])
        self.assertEqual(snap['frozen_pose'][:2], [0.4, 0.0])
        # 打滑：轮子报车在走，对外位姿必须纹丝不动
        for step in (1.0, 2.5, 5.0):
            self.assertEqual(anchor.apply((step, 0.0, 0.0)), (0.4, 0.0, 0.0))

    def test_freeze_twice_keeps_first_anchor(self):
        anchor = odom_anchor.OdomAnchor()
        anchor.freeze((1.0, 1.0, 0.0))
        anchor.freeze((9.0, 9.0, 0.0))
        self.assertEqual(anchor.frozen_pose, (1.0, 1.0, 0.0))
        self.assertEqual(anchor.freezes, 1)


class ResumeTest(unittest.TestCase):

    def test_resume_reanchors_without_jump(self):
        anchor = odom_anchor.OdomAnchor()
        anchor.freeze((0.4, 0.0, 0.0))
        out = anchor.resume((5.0, 0.0, 0.0))
        # 恢复瞬间对外位姿必须还是冻结时那个点，不能跳
        self.assertEqual(out, (0.4, 0.0, 0.0))
        self.assertFalse(anchor.frozen)
        # 之后真实的 0.1 m 位移要体现出来
        self.assertAlmostEqual(anchor.update((5.1, 0.0, 0.0))[0], 0.5, places=6)

    def test_resume_records_discarded_distance(self):
        anchor = odom_anchor.OdomAnchor()
        anchor.freeze((0.0, 0.0, 0.0))
        anchor.resume((0.39, 0.0, 0.0))     # 日志里那 0.39 m
        self.assertAlmostEqual(anchor.snapshot()['discarded_m'], 0.39, places=6)
        self.assertEqual(anchor.snapshot()['resumes'], 1)

    def test_yaw_is_reanchored_too(self):
        anchor = odom_anchor.OdomAnchor()
        anchor.freeze((0.0, 0.0, 0.5))
        anchor.resume((0.0, 0.0, 1.4))
        self.assertAlmostEqual(anchor.apply((0.0, 0.0, 1.4))[2], 0.5, places=6)

    def test_resume_without_freeze_is_noop(self):
        anchor = odom_anchor.OdomAnchor()
        anchor.update((2.0, 0.0, 0.0))
        self.assertEqual(anchor.resume((2.0, 0.0, 0.0)), (2.0, 0.0, 0.0))
        self.assertEqual(anchor.snapshot()['resumes'], 0)


class PoseOfTest(unittest.TestCase):

    class _V:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    class _Pose:
        def __init__(self, x, y, yaw):
            self.position = type('Pos', (), {'x': x, 'y': y, 'z': 0.0})()
            quat = odom_anchor._quat_of(yaw)
            self.orientation = type('Quat', (), {
                'x': quat[0], 'y': quat[1], 'z': quat[2], 'w': quat[3]})()

    class _Odom:
        def __init__(self, pose):
            self.pose = pose

    def test_odometry_pose(self):
        msg = self._Odom(type('PS', (), {'pose': self._Pose(1.5, -2.0, 0.9)})())
        x, y, yaw = odom_anchor.pose_of(msg)
        self.assertAlmostEqual(x, 1.5)
        self.assertAlmostEqual(y, -2.0)
        self.assertAlmostEqual(yaw, 0.9, places=6)

    def test_garbage_returns_none(self):
        self.assertIsNone(odom_anchor.pose_of(object()))


class NodeContractTest(unittest.TestCase):
    """节点必须满足的硬性约定（改坏了会断整条定位链）。"""

    @classmethod
    def setUpClass(cls):
        path = os.path.join(ROOT, 'mybot_explore', 'odom_shim_node.py')
        with open(path, encoding='utf-8') as handle:
            cls.src = handle.read()

    def test_declares_tf_frames(self):
        self.assertIn("declare_parameter('parent_frame', 'odom')", self.src)
        self.assertIn("declare_parameter('child_frame', 'base_link')", self.src)

    def test_uses_broadcaster(self):
        self.assertIn('TransformBroadcaster', self.src)
        self.assertIn('sendTransform', self.src)

    def test_listens_to_gate(self):
        self.assertIn("declare_parameter('gate_topic', '/explore/scan_gate')", self.src)
        self.assertIn('def _on_gate', self.src)

    def test_never_silently_stops_publishing(self):
        # 断流时也要继续发（否则整条链断），只允许告警
        self.assertIn('仍在按最后一次位姿发 TF', self.src)
        self.assertNotIn('return  # 断流就停发', self.src)


if __name__ == '__main__':
    unittest.main()
