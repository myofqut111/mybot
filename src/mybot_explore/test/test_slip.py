# -*- coding: utf-8 -*-
"""``slip.py`` 的用例：判据、排除条件、状态机。"""

import unittest

from mybot_explore import slip as slip_mod


def make_scan(values, range_max=12.0):
    return {
        'ranges': list(values),
        'angle_min': -1.57,
        'angle_increment': 0.01,
        'range_max': range_max,
        'range_min': 0.05,
    }


def wall_scan(distance=1.0, count=20):
    """假想前方一堵墙：所有束都是同一个读数。"""
    return make_scan([distance] * count)


class ScanCompareCase(unittest.TestCase):
    def test_identical_scan_has_no_change(self):
        changed, valid, hits = slip_mod.scan_compare(wall_scan(1.0), wall_scan(1.0))
        self.assertEqual(changed, 0.0)
        self.assertEqual(valid, 1.0)
        self.assertEqual(hits, 20)

    def test_all_changed(self):
        changed, _valid, _hits = slip_mod.scan_compare(wall_scan(1.0), wall_scan(2.0))
        self.assertEqual(changed, 1.0)

    def test_change_below_diff_threshold_is_ignored(self):
        changed, _valid, _hits = slip_mod.scan_compare(wall_scan(1.00), wall_scan(1.02))
        self.assertEqual(changed, 0.0)

    def test_far_beams_are_ignored(self):
        changed, _valid, hits = slip_mod.scan_compare(
            wall_scan(5.0), wall_scan(5.5), near_m=3.0)
        self.assertEqual(hits, 0)
        self.assertEqual(changed, 0.0)

    def test_mixed_near_and_far(self):
        prev = make_scan([1.0] * 10 + [8.0] * 10)
        cur = make_scan([1.5] * 10 + [8.5] * 10)
        changed, valid, hits = slip_mod.scan_compare(prev, cur, near_m=3.0)
        self.assertEqual(hits, 10)
        self.assertAlmostEqual(changed, 1.0)
        self.assertAlmostEqual(valid, 0.5)

    def test_invalid_readings_are_skipped(self):
        prev = make_scan([1.0, float('inf'), 1.0, float('nan')])
        cur = make_scan([1.0, float('inf'), 1.2, 1.0])
        changed, _valid, hits = slip_mod.scan_compare(prev, cur)
        self.assertEqual(hits, 2)
        self.assertAlmostEqual(changed, 0.5)

    def test_max_range_readings_are_skipped(self):
        prev = make_scan([12.0, 1.0], range_max=12.0)
        cur = make_scan([12.0, 1.0], range_max=12.0)
        _changed, _valid, hits = slip_mod.scan_compare(prev, cur, range_max=12.0)
        self.assertEqual(hits, 1)

    def test_empty_scan(self):
        changed, valid, hits = slip_mod.scan_compare(None, None)
        self.assertEqual((changed, valid, hits), (0.0, 0.0, 0))


class SlipDetectorCase(unittest.TestCase):
    def setUp(self):
        self.detector = slip_mod.SlipDetector({'slip_min_hits': 5})

    def drive(self, ticks, odom_step=0.2, drift_step=0.1,
              scan_fn=None, start=0.0, linear=0.15, angular=0.0):
        """按 0.5 s 一拍喂数据，返回最后一次的 result。"""
        result = None
        odom = [0.0, 0.0]
        drift = [0.0, 0.0]
        scan_fn = scan_fn or (lambda i: wall_scan(1.0))
        now = start
        result = self.detector.update(now, odom_pose=tuple(odom),
                                      map_odom=tuple(drift),
                                      scan=scan_fn(0), linear=linear, angular=angular)
        for index in range(1, ticks):
            now = start + index * 0.5
            odom[0] += odom_step
            drift[0] += drift_step
            result = self.detector.update(now, odom_pose=tuple(odom),
                                          map_odom=tuple(drift),
                                          scan=scan_fn(index),
                                          linear=linear, angular=angular)
        return result

    def test_disabled_does_nothing(self):
        detector = slip_mod.SlipDetector({'slip_enabled': False})
        result = detector.update(0.0, (0, 0), (0, 0), wall_scan(1.0), 0.2, 0.0)
        self.assertIsNone(result['action'])
        self.assertEqual(result['reason'], 'disabled')

    def test_rate_limit_keeps_state(self):
        detector = slip_mod.SlipDetector({'slip_min_hits': 5})
        detector.update(0.0, (0, 0), (0, 0), wall_scan(1.0), 0.2, 0.0)
        detector.update(0.2, (0.2, 0), (0.1, 0), wall_scan(1.0), 0.2, 0.0)
        self.assertEqual(detector.state, slip_mod.IDLE)

    def test_turning_is_excluded(self):
        result = self.drive(6, angular=0.8)
        self.assertIsNone(result['action'])
        self.assertEqual(result['reason'], 'turning')

    def test_not_moving_is_excluded(self):
        result = self.drive(6, linear=0.0)
        self.assertIsNone(result['action'])
        self.assertEqual(result['reason'], 'not_moving')

    def test_too_few_hits_is_excluded(self):
        # 整帧都是无效读数：放宽 near 也救不回来
        result = self.drive(6, scan_fn=lambda i: make_scan([float('nan')] * 20))
        self.assertIsNone(result['action'])
        self.assertEqual(result['reason'], 'too_few_hits')

    def test_map_odom_only_is_not_enough(self):
        """判据 A 成立但扫描在变 → 不是打滑。"""
        result = self.drive(6, scan_fn=lambda i: wall_scan(1.0 + 0.1 * i))
        self.assertIsNone(result['action'])
        self.assertEqual(result['reason'], 'not_slipping')

    def test_scan_static_but_no_drift_is_not_enough(self):
        """判据 B 成立但 map→odom 没长 → 不是打滑（比如原地等）。"""
        result = self.drive(6, drift_step=0.0)
        self.assertIsNone(result['action'])

    def test_confirms_after_confirm_window(self):
        result = self.drive(5)
        self.assertEqual(result['action'], 'confirmed')
        self.assertEqual(result['state'], slip_mod.HOLD)

    def test_suspect_before_confirmed(self):
        self.drive(3)
        self.assertEqual(self.detector.state, slip_mod.SUSPECT)
        detector = slip_mod.SlipDetector()
        detector.update(0.0, (0, 0), (0, 0), wall_scan(1.0), 0.2, 0.0)
        detector.update(0.5, (0.2, 0), (0.1, 0), wall_scan(1.0), 0.2, 0.0)
        self.assertEqual(detector.state, slip_mod.SUSPECT)

    def test_resume_when_scan_changes(self):
        self.drive(5)
        result = self.detector.update(
            2.5, (1.0, 0), (0.5, 0), wall_scan(2.0), 0.2, 0.0)
        self.assertEqual(result['action'], 'resume')
        self.assertEqual(result['state'], slip_mod.IDLE)

    def test_give_up_after_hold_timeout(self):
        self.drive(5)
        result = None
        for index in range(40):
            now = 2.5 + index * 0.5
            result = self.detector.update(
                now, (1.0 + 0.01 * index, 0), (0.5, 0), wall_scan(1.0), 0.2, 0.0)
            if result['action'] == 'give_up':
                break
        self.assertEqual(result['action'], 'give_up')
        self.assertEqual(result['state'], slip_mod.IDLE)

    def test_snapshot_fields(self):
        result = self.drive(5)
        for key in ('state', 'action', 'reason', 'ratio', 'valid_ratio',
                    'odom_moved', 'map_odom_moved', 'hold_for'):
            self.assertIn(key, result)

    def test_window_prunes_old_samples(self):
        """窗口只保留 slip_window_s 内的样本。"""
        detector = slip_mod.SlipDetector({'slip_window_s': 1.0, 'slip_min_hits': 5})
        detector.update(0.0, (0, 0), (0.0, 0.0), wall_scan(1.0), 0.2, 0.0)
        detector.update(0.5, (0.2, 0), (0.3, 0.0), wall_scan(1.0), 0.2, 0.0)
        detector.update(5.0, (0.4, 0), (5.0, 0.0), wall_scan(1.0), 0.2, 0.0)
        # 5.0 那一拍把 0.0 和 0.5 都挤出窗口，只剩自己 → 位移为 0
        self.assertEqual(detector.map_odom_moved, 0.0)

    def test_reset_clears_state(self):
        self.drive(5)
        self.detector.reset()
        self.assertEqual(self.detector.state, slip_mod.IDLE)
        self.assertEqual(self.detector.map_odom_moved, 0.0)


if __name__ == '__main__':
    unittest.main()
