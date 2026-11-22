# -*- coding: utf-8 -*-
"""激光平移估计的判据：能估出真位移、能顶住噪声、退化时老实说不ok。"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from mybot_slip import ranges

# 一个 4m x 4m 的房间，原点在中心；用来做“真值”仿真
ROOM = 2.0


def raycast(x, y, n=360, rmax=8.0, half=True):
    lo, hi = (-math.pi, math.pi) if not half else (-math.pi / 2, math.pi / 2)
    inc = (hi - lo) / n
    out = []
    for i in range(n):
        th = lo + i * inc
        c, s = math.cos(th), math.sin(th)
        best = rmax
        # 与四面墙求交
        for plane, sign in ((ROOM, 1), (-ROOM, -1)):
            if abs(c) > 1e-9:
                t = (plane - x) / c
                yh = y + t * s
                if t > 0 and -ROOM <= yh <= ROOM:
                    best = min(best, t)
            if abs(s) > 1e-9:
                t = (plane - y) / s
                xh = x + t * c
                if t > 0 and -ROOM <= xh <= ROOM:
                    best = min(best, t)
        out.append(best)
    return {'ranges': out, 'angle_min': lo, 'angle_increment': inc, 'range_max': rmax}


class FitTest(unittest.TestCase):
    def test_forward_shift_recovered(self):
        prev = raycast(0.0, 0.0)
        cur = raycast(0.10, 0.0)
        r = ranges.fit_translation(prev, cur)
        self.assertTrue(r['ok'], r['reason'])
        self.assertAlmostEqual(r['dx'], 0.10, delta=0.02)
        self.assertAlmostEqual(r['dy'], 0.0, delta=0.02)

    def test_lateral_shift_recovered(self):
        prev = raycast(0.0, 0.0)
        cur = raycast(0.0, 0.08)
        r = ranges.fit_translation(prev, cur)
        self.assertTrue(r['ok'], r['reason'])
        self.assertAlmostEqual(r['dy'], 0.08, delta=0.03)

    def test_zero_motion(self):
        prev = raycast(0.0, 0.0)
        r = ranges.fit_translation(prev, raycast(0.0, 0.0))
        self.assertTrue(r['ok'])
        self.assertAlmostEqual(r['dx'], 0.0, delta=0.01)
        self.assertAlmostEqual(r['dy'], 0.0, delta=0.01)

    def test_noise_tolerated(self):
        import random
        random.seed(7)
        prev = raycast(0.0, 0.0)
        cur = raycast(0.12, 0.0)
        cur['ranges'] = [v + random.gauss(0, 0.01) for v in cur['ranges']]
        r = ranges.fit_translation(prev, cur)
        self.assertTrue(r['ok'], r['reason'])
        self.assertAlmostEqual(r['dx'], 0.12, delta=0.03)

    def test_single_dynamic_object_does_not_derail(self):
        prev = raycast(0.0, 0.0)
        cur = raycast(0.10, 0.0)
        # 前方 30 束被一个人挡住，测距突然变成 0.8 m
        for i in range(165, 195):
            cur['ranges'][i] = 0.8
        r = ranges.fit_translation(prev, cur)
        self.assertTrue(r['ok'], r['reason'])
        self.assertAlmostEqual(r['dx'], 0.10, delta=0.05)

    def test_too_few_beams_fails(self):
        scan = {'ranges': [float('nan')] * 360, 'angle_min': -1.5,
                'angle_increment': 0.01, 'range_max': 8.0}
        r = ranges.fit_translation(scan, scan)
        self.assertFalse(r['ok'])
        self.assertIn('不足', r['reason'])

    def test_all_blocked_fails(self):
        scan = {'ranges': [0.0] * 360, 'angle_min': -1.5,
                'angle_increment': 0.01, 'range_max': 8.0}
        r = ranges.fit_translation(scan, scan)
        self.assertFalse(r['ok'])


if __name__ == '__main__':
    unittest.main()
