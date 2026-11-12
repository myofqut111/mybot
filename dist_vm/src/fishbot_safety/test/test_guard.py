# -*- coding: utf-8 -*-
"""guard.py 的单测：不需要 ROS，在本机直接跑。

这些用例覆盖的正是"推车试错很难覆盖"的边界：刚启动、里程计从没来过、
刹车后自己发的零速不能当成心跳、发布者恢复后要能解除。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_safety.guard import OK, TRIPPED, Guard  # noqa: E402


def make(**kwargs):
    # 关掉启动宽限，让用例里的时间点可以随便挑
    kwargs.setdefault('brake_settle', 0.0)
    return Guard(**kwargs)


class TestGuard(unittest.TestCase):
    def test_starts_clean(self):
        # 给 0.5s 宽限：刚起来的头几百毫秒还什么都没收到，不该立刻刹
        guard = make(brake_settle=0.5)
        tripped, reason = guard.evaluate(0.0)
        self.assertFalse(tripped)
        self.assertEqual(reason, '')
        self.assertEqual(guard.snapshot()['trips'], 0)
        # 宽限过后仍然一帧里程计都没有 → 该刹
        tripped, reason = guard.evaluate(0.6)
        self.assertTrue(tripped)
        self.assertIn('从未收到里程计', reason)

    def test_trips_when_odom_never_arrives(self):
        guard = make()
        tripped, reason = guard.evaluate(1.0)
        self.assertTrue(tripped)
        self.assertIn('从未收到里程计', reason)

    def test_settle_window_does_not_trip(self):
        guard = make(brake_settle=0.2)
        tripped, _ = guard.evaluate(0.1)
        self.assertFalse(tripped)
        tripped, _ = guard.evaluate(0.35)      # 明确越过 0.2s 边界
        self.assertTrue(tripped)

    def test_stationary_without_commands_is_fine(self):
        guard = make()
        guard.observe_odom(0.0, 0.0, 0.0)
        guard.observe_odom(1.0, 0.0, 0.0)
        tripped, _ = guard.evaluate(1.0)
        self.assertFalse(tripped, '停着的车不该被刹车')

    def test_moving_without_commands_trips(self):
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_cmd(0.0, 0.3, 0.0)
        guard.observe_odom(1.0, 0.3, 0.0)      # 车还在动
        # 0.4s 前还有指令 → 不算
        guard.observe_cmd(0.6, 0.3, 0.0)
        tripped, _ = guard.evaluate(0.9)
        self.assertFalse(tripped)
        # 1.0s 已超过 cmd_timeout 0.5
        tripped, reason = guard.evaluate(1.2)
        self.assertTrue(tripped)
        self.assertIn('没有速度指令', reason)

    def test_moving_with_commands_does_not_trip(self):
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_cmd(0.0, 0.3, 0.0)
        for step in range(1, 11):
            now = step * 0.1
            guard.observe_cmd(now, 0.3, 0.0)
            guard.observe_odom(now, 0.3, 0.0)
            tripped, _ = guard.evaluate(now)
            self.assertFalse(tripped)

    def test_rotation_also_counts_as_moving(self):
        guard = make()
        guard.observe_odom(0.0, 0.0, 0.6)
        guard.observe_odom(1.0, 0.0, 0.6)
        tripped, _ = guard.evaluate(1.0)
        self.assertTrue(tripped, '原地转也是在动')

    def test_zero_commands_are_not_heartbeat(self):
        """关键：我们自己刹车时发的是零速，不能把它当成"还活着"。"""
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_cmd(0.0, 0.3, 0.0)
        tripped, _ = guard.evaluate(1.0)
        self.assertTrue(tripped)
        for step in range(1, 11):
            now = 1.0 + step * 0.05
            guard.observe_cmd(now, 0.0, 0.0)     # 看门狗自己发的零速
            guard.observe_odom(now, 0.3, 0.0)    # 车还在滑
            tripped, _ = guard.evaluate(now)
            self.assertTrue(tripped, '零速指令不能解除刹车')

    def test_real_command_releases(self):
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_odom(1.0, 0.3, 0.0)
        tripped, _ = guard.evaluate(1.0)
        self.assertTrue(tripped)
        guard.observe_cmd(1.05, 0.2, 0.0)        # 真人/导航回来了
        guard.observe_odom(1.1, 0.2, 0.0)
        tripped, _ = guard.evaluate(1.1)
        self.assertFalse(tripped)
        self.assertEqual(guard.snapshot()['reason'], '')

    def test_trip_counted_once_per_episode(self):
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_odom(1.0, 0.3, 0.0)
        guard.evaluate(1.0)
        guard.observe_odom(1.05, 0.3, 0.0)
        guard.evaluate(1.05)
        guard.evaluate(1.10)
        self.assertEqual(guard.snapshot()['trips'], 1)

    def test_reason_reported_only_on_change(self):
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_odom(1.0, 0.3, 0.0)
        tripped, reason = guard.evaluate(1.0)
        self.assertTrue(tripped)
        self.assertTrue(reason)
        guard.observe_odom(1.05, 0.3, 0.0)
        tripped, reason = guard.evaluate(1.05)
        self.assertTrue(tripped)
        self.assertEqual(reason, '', '没变化就不要重复刷日志')

    def test_odom_stale_after_running(self):
        guard = make()
        guard.observe_odom(0.0, 0.3, 0.0)
        guard.observe_cmd(0.0, 0.3, 0.0)
        tripped, _ = guard.evaluate(0.1)
        self.assertFalse(tripped)
        tripped, reason = guard.evaluate(2.0)     # 里程计超过 1.0s 没来
        self.assertTrue(tripped)
        self.assertIn('odom 断开', reason)

    def test_speed_below_epsilon_is_stationary(self):
        guard = make(move_epsilon=0.05)
        guard.observe_odom(0.0, 0.01, 0.01)
        guard.observe_odom(2.0, 0.01, 0.01)
        tripped, _ = guard.evaluate(2.0)
        self.assertFalse(tripped)

    def test_snapshot_shape(self):
        guard = make()
        guard.observe_odom(0.0, 0.2, 0.1)
        snap = guard.snapshot()
        self.assertEqual(snap['tripped'], False)
        self.assertEqual(snap['moving'], True)
        self.assertAlmostEqual(snap['speed']['vx'], 0.2)
        self.assertIn('trips', snap)

    def test_constants(self):
        self.assertEqual(OK, 'OK')
        self.assertEqual(TRIPPED, 'TRIPPED')


if __name__ == '__main__':
    unittest.main()
