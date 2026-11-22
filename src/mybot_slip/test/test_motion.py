# -*- coding: utf-8 -*-
"""动作达成判断：给一串位姿，看退够/转够没有。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from mybot_slip.motion import TravelMonitor


class BackTest(unittest.TestCase):
    def test_not_done_when_not_moved(self):
        m = TravelMonitor('back', 0.10)
        m.update(0.0, 0.0, 0.0)
        for i in range(5):
            m.update(0.0, 0.0, 0.0)
        self.assertFalse(m.done())

    def test_done_after_distance(self):
        m = TravelMonitor('back', 0.10)
        m.update(1.0, 2.0, 0.3)
        m.update(0.96, 2.0, 0.3)      # 退了 4 cm
        self.assertFalse(m.done())
        m.update(0.90, 2.0, 0.3)      # 累计 10 cm
        self.assertTrue(m.done())

    def test_distance_accumulates_on_curve(self):
        """沿曲线退也算走过的路：不是直线位移，是路径长度。"""
        m = TravelMonitor('back', 0.30)
        m.update(0.0, 0.0, 0.0)
        m.update(0.0, 0.15, 0.0)
        m.update(0.15, 0.15, 0.0)
        self.assertTrue(m.done())

    def test_tolerance(self):
        m = TravelMonitor('back', 0.10, tolerance=0.015)
        m.update(0.0, 0.0, 0.0)
        m.update(0.086, 0.0, 0.0)     # 差 1.4 cm，在容差内
        self.assertTrue(m.done())

    def test_reached_at_least(self):
        m = TravelMonitor('back', 0.20)
        m.update(0.0, 0.0, 0.0)
        m.update(0.12, 0.0, 0.0)
        self.assertTrue(m.reached_at_least(0.6))
        self.assertFalse(m.done())


class EscapeTest(unittest.TestCase):
    def test_net_rotation_not_sum_of_abs(self):
        """来回摆不算进度：原地抖两下凑不出「转够了」。"""
        m = TravelMonitor('escape', 1.0)
        m.update(0.0, 0.0, 0.0)
        m.update(0.0, 0.0, 0.6)
        self.assertAlmostEqual(m.progress(), 0.6)
        m.update(0.0, 0.0, 0.4)       # 回摆 0.2
        self.assertAlmostEqual(m.progress(), 0.4)

    def test_wrap_around(self):
        """从 +3.1 rad 转到 -3.1 rad 是转了 0.08，不是 6.2。"""
        m = TravelMonitor('escape', 0.10)
        m.update(0.0, 0.0, 3.10)
        m.update(0.0, 0.0, -3.10)
        self.assertAlmostEqual(m.progress(), 0.0832, places=3)
        self.assertFalse(m.done())

    def test_done_on_target(self):
        m = TravelMonitor('escape', 0.9)
        m.update(0.0, 0.0, 0.0)
        m.update(0.0, 0.0, 0.95)
        self.assertTrue(m.done())


if __name__ == '__main__':
    unittest.main()
