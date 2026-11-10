#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航层中断判定的回归测试。

背景：``rclpy`` 的 ``ServerGoalHandle.is_cancel_requested`` 是**属性**而不是
方法。曾经把它直接当 ``cancel_check`` 回调传进 ``navigate()``，导航层每
20 ms 调一次 ``cancel_check()``，于是每个点位一开始导航就抛
``'bool' object is not callable``，整轮巡检在第一个点就崩。

这里锁住两件事：
  1. 传布尔值不再崩，且语义正确；
  2. 传回调仍然照常工作。

本机没有 rclpy 时自动跳过（在虚拟机/板子上才会真正执行）。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from mybot_patrol_v2.navigation import (
        NAV_CANCELED, NAV_PAUSED, NAV_PREEMPTED, Navigator,
    )
except Exception:                      # noqa: BLE001 - 缺 rclpy / nav2_msgs 就跳过
    Navigator = None
    NAV_CANCELED = NAV_PAUSED = NAV_PREEMPTED = None


@unittest.skipIf(Navigator is None, 'rclpy 或 nav2_msgs 不可用，跳过')
class InterruptReasonTest(unittest.TestCase):
    """``_interrupt_reason`` 对「回调」和「布尔值」都要能处理。"""

    def make(self):
        """不跑 __init__，只借方法（它不依赖实例状态）。"""
        return Navigator.__new__(Navigator)

    def test_bool_true_is_treated_as_cancel(self):
        nav = self.make()
        self.assertEqual(
            nav._interrupt_reason(True, lambda: None), NAV_CANCELED)

    def test_bool_false_falls_through_to_interrupt(self):
        nav = self.make()
        self.assertIsNone(nav._interrupt_reason(False, lambda: None))
        self.assertEqual(
            nav._interrupt_reason(False, lambda: NAV_PAUSED), NAV_PAUSED)

    def test_callable_is_still_supported(self):
        nav = self.make()
        self.assertEqual(
            nav._interrupt_reason(lambda: True, lambda: None), NAV_CANCELED)
        self.assertEqual(
            nav._interrupt_reason(lambda: False, lambda: NAV_PREEMPTED),
            NAV_PREEMPTED)

    def test_cancel_has_priority_over_pause(self):
        nav = self.make()
        self.assertEqual(
            nav._interrupt_reason(True, lambda: NAV_PAUSED), NAV_CANCELED)

    def test_shutdown_stop_check_cancels(self):
        """节点正在退出时，即使没收到 cancel 也必须立刻放弃当前导航。"""
        nav = self.make()
        nav._stop_check = lambda: True
        self.assertEqual(
            nav._interrupt_reason(False, lambda: None), NAV_CANCELED)
        self.assertEqual(
            nav._interrupt_reason(False, lambda: NAV_PAUSED), NAV_CANCELED)

    def test_shutdown_stop_check_absent_is_safe(self):
        """老测试/老调用方没有 _stop_check 属性时不能崩。"""
        nav = self.make()
        self.assertIsNone(nav._interrupt_reason(False, lambda: None))

    def test_abort_without_active_goal_is_noop(self):
        nav = self.make()
        nav._active_goal = None
        nav._cancel_timeout = 1.0
        nav.abort()          # 不应抛异常


if __name__ == '__main__':
    unittest.main(verbosity=2)
