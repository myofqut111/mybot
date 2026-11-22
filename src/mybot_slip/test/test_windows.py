# -*- coding: utf-8 -*-
"""窗口聚合的契约：不重叠、积分、覆盖门槛、warmup、扫描配对。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from mybot_slip import windows


def feed(agg, t0, t1, step, vx, vyaw, cmd=(0.2, 0.0), gyro=0.0):
    t = t0
    while t <= t1 + 1e-9:
        agg.add_odom(t, vx, vyaw)
        agg.add_wheel(t, vx * t, vx * t)
        agg.add_imu(t, gyro)
        agg.add_cmd(t, *cmd)
        t += step


class WindowTest(unittest.TestCase):
    def test_emits_non_overlapping_windows(self):
        agg = windows.WindowAggregator({'warmup_s': 0.0})
        feed(agg, 0.0, 1.2, 0.05, 0.18, 0.0)
        wins = agg.poll(1.2)
        self.assertGreaterEqual(len(wins), 3)
        for a, b in zip(wins, wins[1:]):
            self.assertAlmostEqual(a['t1'], b['t0'], places=6)

    def test_denc_integral(self):
        agg = windows.WindowAggregator({'warmup_s': 0.0})
        feed(agg, 0.0, 0.9, 0.05, 0.20, 0.0)
        wins = agg.poll(0.9)
        w = wins[1]
        self.assertAlmostEqual(w['denc'], 0.20 * w['dt'], places=3)
        self.assertTrue(w['odom_ok'])

    def test_wheel_delta_from_cumulative(self):
        agg = windows.WindowAggregator({'warmup_s': 0.0})
        feed(agg, 0.0, 0.9, 0.05, 0.2, 0.0)
        wins = agg.poll(0.9)
        w = wins[1]
        self.assertAlmostEqual(w['d_left'], 0.2 * w['dt'], places=3)
        self.assertAlmostEqual(w['d_right'], 0.2 * w['dt'], places=3)

    def test_low_cmd_coverage_invalid(self):
        agg = windows.WindowAggregator({'warmup_s': 0.0})
        feed(agg, 0.0, 0.6, 0.05, 0.2, 0.0)
        t = 0.65
        while t <= 1.2 + 1e-9:
            agg.add_odom(t, 0.2, 0.0)
            t += 0.05
        wins = agg.poll(1.2)
        self.assertTrue(any((not w['valid']) and '指令覆盖' in w['reason'] for w in wins))

    def test_odom_gap_invalidates(self):
        agg = windows.WindowAggregator({'warmup_s': 0.0})
        for t in (0.0, 0.05):
            agg.add_odom(t, 0.2, 0.0)
        for t in (0.0, 0.3, 0.6):
            agg.add_cmd(t, 0.2, 0.0)
        wins = agg.poll(0.6)
        self.assertTrue(any(not w['valid'] for w in wins))

    def test_warmup_suppresses_window(self):
        agg = windows.WindowAggregator({'warmup_s': 0.4})
        agg.add_cmd(0.0, 0.0, 0.0)
        feed(agg, 0.05, 0.6, 0.05, 0.2, 0.0)
        wins = agg.poll(0.6)
        self.assertTrue(wins[0]['warmup'])

    def test_scan_pair_selected(self):
        agg = windows.WindowAggregator({'warmup_s': 0.0})
        scan = {'ranges': [1.0] * 10, 'angle_min': 0.0,
                'angle_increment': 0.1, 'range_max': 8.0}
        for t in (0.02, 0.12, 0.22, 0.32):
            agg.add_scan(t, scan)
        feed(agg, 0.0, 0.6, 0.05, 0.2, 0.0)
        wins = agg.poll(0.6)
        self.assertTrue(wins[0]['scan_ok'])
        self.assertGreater(wins[0]['scan_dt'], 0)


if __name__ == '__main__':
    unittest.main()
