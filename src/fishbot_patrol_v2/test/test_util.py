"""util.py 的单测：不依赖 ROS，直接 ``python3 test/test_util.py`` 就能跑。"""

import math
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_patrol_v2.util import (  # noqa: E402
    angle_diff, as_check, parse_detections, pose_error, route_index_of,
    stable_residual, yaw_of)


def yaw_quaternion(yaw):
    return SimpleNamespace(x=0.0, y=0.0,
                           z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def pose(x, y, yaw):
    return SimpleNamespace(position=SimpleNamespace(x=x, y=y),
                           orientation=yaw_quaternion(yaw))


class AsCheckTest(unittest.TestCase):
    """回调参数归一化：布尔值不能变成 `'bool' object is not callable`。"""

    def test_callable_passes_through(self):
        flag = {'value': False}
        check = as_check(lambda: flag['value'])
        self.assertFalse(check())
        flag['value'] = True
        self.assertTrue(check())

    def test_bool_is_wrapped(self):
        self.assertTrue(as_check(True)())
        self.assertFalse(as_check(False)())

    def test_none_is_false(self):
        self.assertFalse(as_check(None)())


class QuaternionHardeningTest(unittest.TestCase):
    """非单位四元数 / 退化四元数不能算出错误偏航。"""

    def test_non_unit_quaternion_is_normalised(self):
        scaled = yaw_quaternion(math.pi / 2.0)
        scaled = SimpleNamespace(x=scaled.x * 2.0, y=scaled.y * 2.0,
                                 z=scaled.z * 2.0, w=scaled.w * 2.0)
        self.assertAlmostEqual(yaw_of(SimpleNamespace(orientation=scaled)),
                               math.pi / 2.0, places=6)

    def test_zero_quaternion_falls_back_to_zero(self):
        zero = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
        self.assertEqual(yaw_of(SimpleNamespace(orientation=zero)), 0.0)


class StableResidualHardeningTest(unittest.TestCase):
    """NaN 残差必须当作「无法校验」，不能变成去假到点。"""

    def test_nan_sample_returns_none(self):
        self.assertIsNone(
            stable_residual(lambda: (float('nan'), 0.0), duration_s=0.0))
        self.assertIsNone(
            stable_residual(lambda: (0.0, float('inf')), duration_s=0.0))

    def test_finite_sample_still_works(self):
        self.assertEqual(
            stable_residual(lambda: (0.12, 0.03), duration_s=0.0),
            (0.12, 0.03))


class RouteIndexTest(unittest.TestCase):
    """作业下标读取要能容忍缺字段。"""

    def test_reads_route_index(self):
        self.assertEqual(route_index_of(SimpleNamespace(route_index=7)), 7)

    def test_missing_field_defaults_to_zero(self):
        self.assertEqual(route_index_of(SimpleNamespace()), 0)


class ParseDetectionsTest(unittest.TestCase):
    """任务结果里的 detections 只在格式对的时候才取。"""

    def test_extracts_list(self):
        payload = '{"detections": [{"label": "fire_extinguisher"}]}'
        self.assertEqual(parse_detections(payload),
                         [{'label': 'fire_extinguisher'}])

    def test_empty_message(self):
        self.assertEqual(parse_detections(''), [])

    def test_plain_text_is_ignored(self):
        self.assertEqual(parse_detections('found 2 extinguishers'), [])

    def test_broken_json_is_ignored(self):
        self.assertEqual(parse_detections('{"detections": [}'), [])

    def test_non_dict_json_is_ignored(self):
        self.assertEqual(parse_detections('[1, 2, 3]'), [])

    def test_missing_key_is_ignored(self):
        self.assertEqual(parse_detections('{"ok": true}'), [])

    def test_non_list_value_is_ignored(self):
        self.assertEqual(parse_detections('{"detections": 3}'), [])


class YawTest(unittest.TestCase):
    """四元数到偏航角。"""

    def test_zero(self):
        self.assertAlmostEqual(yaw_of(pose(0, 0, 0.0)), 0.0, places=9)

    def test_quarter_turn(self):
        self.assertAlmostEqual(yaw_of(pose(0, 0, math.pi / 2)), math.pi / 2, places=9)

    def test_half_turn(self):
        self.assertAlmostEqual(abs(yaw_of(pose(0, 0, math.pi))), math.pi, places=6)


class AngleDiffTest(unittest.TestCase):
    """角差要处理 ±π 处的回绕。"""

    def test_simple(self):
        self.assertAlmostEqual(angle_diff(0.5, 0.2), 0.3, places=9)

    def test_wraparound(self):
        self.assertAlmostEqual(angle_diff(math.pi - 0.1, -math.pi + 0.1), 0.2, places=9)

    def test_symmetric(self):
        self.assertAlmostEqual(angle_diff(0.2, 0.5), angle_diff(0.5, 0.2), places=12)


class PoseErrorTest(unittest.TestCase):
    """到点校验用的位置/偏航误差。"""

    def test_same_pose(self):
        error = pose_error(pose(1.0, 2.0, 0.3), pose(1.0, 2.0, 0.3))
        self.assertAlmostEqual(error[0], 0.0, places=9)
        self.assertAlmostEqual(error[1], 0.0, places=9)

    def test_translation_only(self):
        error = pose_error(pose(1.0, 1.0, 0.0), pose(4.0, 5.0, 0.0))
        self.assertAlmostEqual(error[0], 5.0, places=9)
        self.assertAlmostEqual(error[1], 0.0, places=9)

    def test_rotation_only(self):
        error = pose_error(pose(0.0, 0.0, 1.0), pose(0.0, 0.0, 0.0))
        self.assertAlmostEqual(error[0], 0.0, places=9)
        self.assertAlmostEqual(error[1], 1.0, places=9)


class StableResidualTest(unittest.TestCase):
    """到点复核取采样窗口内最坏残差，不是单帧。"""

    @staticmethod
    def cycler(values):
        state = {'index': 0}

        def sample():
            value = values[state['index'] % len(values)]
            state['index'] += 1
            return value
        return sample

    def test_takes_worst_sample(self):
        sample = self.cycler([(0.01, 0.02), (0.4, 0.2), (0.02, 0.01)])
        residual = stable_residual(sample, duration_s=0.15, interval_s=0.01)
        self.assertAlmostEqual(residual[0], 0.4, places=9)
        self.assertAlmostEqual(residual[1], 0.2, places=9)

    def test_one_untrusted_sample_poisons_the_window(self):
        sample = self.cycler([(0.01, 0.0), None, (0.02, 0.0)])
        self.assertIsNone(
            stable_residual(sample, duration_s=0.15, interval_s=0.01))

    def test_window_length_is_respected(self):
        calls = []

        def sample():
            calls.append(1)
            return (0.0, 0.0)

        stable_residual(sample, duration_s=0.12, interval_s=0.05)
        self.assertGreaterEqual(len(calls), 2)


if __name__ == '__main__':
    unittest.main()
