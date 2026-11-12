"""battery.py 的单测：中值滤波、迟滞转移、数据过期。

这几个用例都是「现场真会踩」的：

- 加速瞬间电压掉到 11.4 V → 不能立刻报 crit（那是压降，不是没电）
- 停在 11.75 V 抖动 → 不能 ok/warn 反复横跳
- 发布者挂了 → 必须回到 unknown，不能拿三分钟前的电压继续决策
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_patrol_v2.battery import (  # noqa: E402
    LEVEL_CRIT, LEVEL_OK, LEVEL_UNKNOWN, LEVEL_WARN, BatteryMonitor)


class FakeClock:
    """可控时钟，省得测试里 sleep。"""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def make_monitor(warn_v=11.8, crit_v=11.5, hyst_v=0.4, window=5,
                 stale_sec=30.0):
    clock = FakeClock()
    monitor = BatteryMonitor(warn_v=warn_v, crit_v=crit_v, hyst_v=hyst_v,
                            window=window, stale_sec=stale_sec, clock=clock)
    return monitor, clock


class MedianFilterTest(unittest.TestCase):

    def test_empty_returns_none(self):
        monitor, _ = make_monitor()
        self.assertIsNone(monitor.voltage())
        self.assertEqual(monitor.level(), LEVEL_UNKNOWN)

    def test_single_spike_is_ignored(self):
        monitor, _ = make_monitor(window=5)
        for value in (12.6, 12.6, 11.2, 12.6, 12.6):
            monitor.update(value)
        # 五个样本里只有一个是 11.2，中值仍是 12.6
        self.assertAlmostEqual(monitor.voltage(), 12.6)
        self.assertEqual(monitor.level(), LEVEL_OK)

    def test_window_keeps_latest_only(self):
        monitor, _ = make_monitor(window=3)
        for value in (12.6, 12.6, 12.6, 11.0, 11.0, 11.0):
            monitor.update(value)
        self.assertAlmostEqual(monitor.voltage(), 11.0)

    def test_even_window_averages_middle_pair(self):
        monitor, _ = make_monitor(window=4)
        for value in (11.0, 11.2, 11.4, 11.6):
            monitor.update(value)
        self.assertAlmostEqual(monitor.voltage(), 11.3)

    def test_bad_values_rejected(self):
        monitor, _ = make_monitor()
        for value in (None, 'abc', float('nan'), float('inf'), -1.0, 0.0):
            self.assertFalse(monitor.update(value))
        self.assertIsNone(monitor.voltage())
        self.assertEqual(monitor.level(), LEVEL_UNKNOWN)


class HysteresisTest(unittest.TestCase):

    def test_ok_to_warn_at_threshold(self):
        monitor, _ = make_monitor()
        for _ in range(5):
            monitor.update(11.8)
        self.assertEqual(monitor.level(), LEVEL_WARN)

    def test_warn_does_not_clear_on_small_bounce(self):
        monitor, _ = make_monitor()
        for _ in range(5):
            monitor.update(11.7)
        self.assertEqual(monitor.level(), LEVEL_WARN)
        # 回到 11.9（高于 11.8 但没到 11.8+0.4）仍算 warn
        for _ in range(5):
            monitor.update(11.9)
        self.assertEqual(monitor.level(), LEVEL_WARN)
        # 越过回差才恢复
        for _ in range(5):
            monitor.update(12.2)
        self.assertEqual(monitor.level(), LEVEL_OK)

    def test_crit_then_back_to_warn_not_ok(self):
        monitor, _ = make_monitor()
        for _ in range(5):
            monitor.update(11.4)
        self.assertEqual(monitor.level(), LEVEL_CRIT)
        # crit 回升：先经过 crit+hyst=11.9，此时只到 warn
        for _ in range(5):
            monitor.update(11.95)
        self.assertEqual(monitor.level(), LEVEL_WARN)
        for _ in range(5):
            monitor.update(12.4)
        self.assertEqual(monitor.level(), LEVEL_OK)

    def test_crit_while_warn(self):
        monitor, _ = make_monitor()
        for _ in range(5):
            monitor.update(11.7)
        self.assertEqual(monitor.level(), LEVEL_WARN)
        for _ in range(5):
            monitor.update(11.4)
        self.assertEqual(monitor.level(), LEVEL_CRIT)


class StaleTest(unittest.TestCase):

    def test_stale_becomes_unknown(self):
        monitor, clock = make_monitor(stale_sec=30.0)
        monitor.update(12.6)
        self.assertEqual(monitor.level(), LEVEL_OK)
        clock.advance(31.0)
        # 值还在，但已经不能拿它做决策
        self.assertAlmostEqual(monitor.voltage(), 12.6)
        self.assertEqual(monitor.level(), LEVEL_UNKNOWN)

    def test_fresh_again_after_update(self):
        monitor, clock = make_monitor(stale_sec=30.0)
        monitor.update(12.6)
        clock.advance(40.0)
        self.assertEqual(monitor.level(), LEVEL_UNKNOWN)
        monitor.update(12.6)
        self.assertEqual(monitor.level(), LEVEL_OK)

    def test_reset_clears_everything(self):
        monitor, _ = make_monitor()
        monitor.update(12.6)
        monitor.reset()
        self.assertIsNone(monitor.voltage())
        self.assertEqual(monitor.level(), LEVEL_UNKNOWN)
        self.assertFalse(monitor.fresh())


class DescribeTest(unittest.TestCase):

    def test_describe_without_data(self):
        monitor, _ = make_monitor()
        self.assertIn('unknown', monitor.describe())

    def test_describe_with_data(self):
        monitor, _ = make_monitor()
        for _ in range(3):
            monitor.update(12.34)
        text = monitor.describe()
        self.assertIn('12.34V', text)
        self.assertIn(LEVEL_OK, text)


if __name__ == '__main__':
    unittest.main()
