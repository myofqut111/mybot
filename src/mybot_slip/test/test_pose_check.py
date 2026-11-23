"""方案 §6（脱困后位姿校验）的判据测试。

只测"判不判跳变、判了要做什么"，不测 ROS 服务真的回没回——
那两个服务的调用是 fire-and-forget，回不回不影响这里的状态转移。
"""
import math
import unittest

from mybot_slip import slip_node as mod  # noqa: E402


class _Logger(object):
    def __init__(self):
        self.errors = []
        self.warns = []
        self.infos = []

    def error(self, text):
        self.errors.append(text)

    def warn(self, text):
        self.warns.append(text)

    def info(self, text):
        self.infos.append(text)


class _Stub(object):
    """只保留 _check_pose_jump 需要的属性。"""

    p = mod.SlipNode.p
    _wrap = staticmethod(mod.SlipNode._wrap)
    _check_pose_jump = mod.SlipNode._check_pose_jump

    def __init__(self, limits=(0.15, 0.20)):
        self._logger = _Logger()
        self._params = {'pose_jump_m': limits[0],
                        'pose_jump_rad': limits[1],
                        'relocate_on_jump': True}
        self._recovery_pose = None
        self._pose = None
        self.cleared = 0
        self.relocalized = 0

    # --- 被测方法依赖的三个口子 ---
    def get_parameter(self, name):
        return _Val(self._params[name])

    def get_logger(self):
        return self._logger

    def _clear_costmaps(self):
        self.cleared += 1

    def _relocalize(self):
        self.relocalized += 1


class _Val(object):
    def __init__(self, value):
        self.value = value


def _run(before, after, limits=(0.15, 0.20)):
    node = _Stub(limits)
    node._recovery_pose = before
    node._pose = after
    node._check_pose_jump(0.0)
    return node


class PoseJumpTest(unittest.TestCase):
    def test_small_change_passes(self):
        node = _run((0.0, 0.0, 0.0), (0.05, 0.02, 0.05))
        self.assertEqual(node.cleared, 0)
        self.assertEqual(node.relocalized, 0)
        self.assertEqual(node._logger.errors, [])
        self.assertTrue(any('校验通过' in t for t in node._logger.infos))

    def test_large_translation_triggers_both(self):
        node = _run((0.0, 0.0, 0.0), (0.40, 0.0, 0.0))
        self.assertEqual(node.cleared, 1)
        self.assertEqual(node.relocalized, 1)
        self.assertTrue(any('位姿跳变' in t for t in node._logger.errors))

    def test_large_rotation_triggers(self):
        node = _run((0.0, 0.0, 0.0), (0.0, 0.0, 0.35))
        self.assertEqual(node.cleared, 1)

    def test_yaw_wraparound_is_not_a_jump(self):
        # 从 +3.13 转到 -3.13 只差 0.02 rad，不能当成 6.26 rad 的跳变
        node = _run((0.0, 0.0, 3.13), (0.0, 0.0, -3.13))
        self.assertEqual(node.cleared, 0)

    def test_missing_pose_is_skipped(self):
        node = _run(None, (0.0, 0.0, 0.0))
        self.assertEqual(node.cleared, 0)
        node = _run((0.0, 0.0, 0.0), None)
        self.assertEqual(node.cleared, 0)

    def test_recovery_pose_cleared_after_check(self):
        node = _run((0.0, 0.0, 0.0), (0.01, 0.0, 0.0))
        self.assertIsNone(node._recovery_pose)

    def test_threshold_is_configurable(self):
        node = _run((0.0, 0.0, 0.0), (0.40, 0.0, 0.0), limits=(0.50, 0.20))
        self.assertEqual(node.cleared, 0)

    def test_boundary_equal_does_not_trigger(self):
        node = _run((0.0, 0.0, 0.0), (0.15, 0.0, 0.0))
        self.assertEqual(node.cleared, 0)


if __name__ == '__main__':
    unittest.main()
