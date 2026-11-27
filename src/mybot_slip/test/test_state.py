# -*- coding: utf-8 -*-
"""状态机：确认、分流、后退校验、放大膨胀、放弃。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from mybot_slip import state as st


def verdict(t=None, denc=0.05):
    return {'slip': t is not None, 'type': t, 'ratios': {'primary': 1.0}, '_denc': denc}


class NormalTest(unittest.TestCase):
    def test_clean_stays_normal(self):
        m = st.SlipStateMachine()
        for i in range(5):
            out = m.step(i * 0.3, verdict())
            self.assertIsNone(out['action'])
        self.assertEqual(m.state, st.NORMAL)

    def test_single_window_does_not_trigger(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        out = m.step(0.0, verdict('LINEAR_SLIP'))
        self.assertIsNone(out['action'])
        self.assertEqual(m.state, st.SLIP)


class ConfirmTest(unittest.TestCase):
    def test_two_windows_confirm_back(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP', denc=0.06))
        out = m.step(0.3, verdict('LINEAR_SLIP', denc=0.06))
        self.assertEqual(out['action'], 'back')
        # 表 9-1 / §9.3：后退固定 0.12 m、速度 0.15 m/s、超时 1.5 s
        self.assertAlmostEqual(out['distance'], 0.12)
        self.assertAlmostEqual(out['speed'], 0.15)
        self.assertAlmostEqual(out['timeout_s'], 1.5)
        self.assertAlmostEqual(out['rear_check_m'], 0.30)

    def test_confirmation_needs_time(self):
        m = st.SlipStateMachine({'confirm_windows': 2, 'confirm_window_s': 1.0})
        m.step(0.0, verdict('LINEAR_SLIP'))
        out = m.step(0.3, verdict('LINEAR_SLIP'))     # 只有 0.3 s
        self.assertIsNone(out['action'])
        out = m.step(1.2, verdict('LINEAR_SLIP'))
        self.assertEqual(out['action'], 'back')

    def test_second_back_is_longer(self):
        """§9.5：第二次恢复后退 0.20 m（不是按比例算，也不是重复 0.12）。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP', denc=2.0))
        first = m.step(0.3, verdict('LINEAR_SLIP', denc=2.0))
        self.assertAlmostEqual(first['distance'], 0.12)
        m.on_action_done(0.6, ok=False)            # 第一次失败 → 第二次
        second = m.pending_action
        self.assertEqual(second['action'], 'back')
        self.assertAlmostEqual(second['distance'], 0.20)

    def test_drop_back_to_normal(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP'))
        m.step(0.3, verdict())
        self.assertEqual(m.state, st.NORMAL)


class StallTest(unittest.TestCase):
    def test_stall_escapes_without_backing(self):
        """方案 §3：堵转不后退、不加力，停车上报等人工。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('STALL'))
        # 方案 §12 Tstall：必须持续 0.6 s
        out = m.step(0.8, verdict('STALL'))
        self.assertEqual(out['action'], 'stop')
        self.assertTrue(out.get('terminal'))
        self.assertNotIn('distance', out)
        self.assertEqual(m.stats['backs'], 0)
        # 停车做完就收工等人工，不再重试
        done = m.on_action_done(2.0, ok=True, moved=0.0)
        self.assertEqual(done['action'], 'abort')
        self.assertEqual(m.state, st.FAILED)


class RecoveryTest(unittest.TestCase):
    def _to_recovery(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP', denc=0.06))
        out = m.step(0.3, verdict('LINEAR_SLIP', denc=0.06))
        self.assertEqual(out['action'], 'back')
        return m

    def test_action_done_enters_check(self):
        m = self._to_recovery()
        out = m.on_action_done(0.6, ok=True, moved=0.10)
        # 方案 §3：后退完成后接一次低速重试，而不是直接宣布恢复
        self.assertEqual(out['action'], 'go')
        self.assertEqual(m.state, st.RECOVERY)

    def test_short_back_escalates(self):
        m = self._to_recovery()
        out = m.on_action_done(0.6, ok=True, moved=0.01)   # 只退到期望的 20%
        # 方案 §7：最多两次，第二次仍是同类型动作（不发明新姿势）
        self.assertEqual(out['action'], 'back')
        self.assertIn('第 2 次', out['reason'])
        self.assertEqual(m.state, st.RECOVERY)

    def test_repeat_failures_request_inflate(self):
        m = self._to_recovery()
        actions = []
        t = 0.6
        for _ in range(4):
            out = m.on_action_done(t, ok=False, moved=0.0)
            actions.append(out['action'])
            t += 0.6
        # 方案 §7：两次恢复都失败 → FAILED 上报
        self.assertIn('abort', actions)
        self.assertEqual(m.state, st.FAILED)

    def test_clean_windows_recover(self):
        m = self._to_recovery()
        m.on_action_done(0.6, ok=True, moved=0.12)      # 后退完成 → 低速重试
        m.on_action_done(1.2, ok=True, moved=0.10)      # 重试完成 → CHECK
        # §9.5：静止 0.3 s 后先采两个干净窗（这里第一次只算第 1 个）
        self.assertIsNone(m.step(2.0, verdict())['action'])
        probe = m.step(2.4, verdict())
        self.assertEqual(probe['action'], 'probe')
        self.assertAlmostEqual(probe['distance'], 0.05)
        # 探针走完 → 复检一次（1 个干净窗）就恢复
        m.on_action_done(2.8, ok=True, moved=0.05)
        out = m.step(3.2, verdict())
        self.assertEqual(out['action'], 'recovered')
        self.assertEqual(m.state, st.NORMAL)
        self.assertEqual(m.recoveries, 0)

    def test_slip_during_check_goes_back_to_recovery(self):
        m = self._to_recovery()
        m.on_action_done(0.6, ok=True, moved=0.12)
        m.on_action_done(1.2, ok=True, moved=0.10)
        m.step(2.0, verdict())          # 第 1 个干净窗
        m.step(2.4, verdict())          # 第 2 个干净窗 → 发出 probe
        m.on_action_done(2.8, ok=True, moved=0.05)
        m.step(3.5, verdict('LINEAR_SLIP'))
        self.assertEqual(m.state, st.SLIP)

    def test_max_recovery_aborts(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'max_recovery': 2})
        m.step(0.0, verdict('LINEAR_SLIP'))
        m.step(0.3, verdict('LINEAR_SLIP'))
        actions = []
        for i in range(3):
            out = m.on_action_done(1.0 + i, ok=False)
            actions.append(out['action'])
        self.assertIn('abort', actions)
        self.assertEqual(m.state, st.FAILED)

    def test_snapshot_fields(self):
        m = st.SlipStateMachine()
        snap = m.snapshot()
        for k in ('state', 'type', 'recoveries', 'stats'):
            self.assertIn(k, snap)


class NoExecutorTest(unittest.TestCase):
    """执行器不在时的行为：不卡死、别装死。"""

    def _confirm(self, m, t0=0.0, n=3):
        for i in range(n):
            m.step(t0 + i * 0.6, verdict('LINEAR_SLIP'))

    def test_recovery_disabled_stays_in_slip(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.allow_recovery = False
        self._confirm(m)
        self.assertEqual(m.state, st.SLIP)
        self.assertIsNone(m.pending_action)

    def test_action_timeout_does_not_hang(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'action_timeout_s': 2.0})
        self._confirm(m)
        self.assertEqual(m.state, st.RECOVERY)
        out = m.step(10.0, verdict('LINEAR_SLIP'))
        self.assertEqual(out['action'], 'none')
        self.assertEqual(m.state, st.CHECK)
        self.assertEqual(m.stats['timeouts'], 1)

    def test_action_before_timeout_waits(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'action_timeout_s': 20.0})
        self._confirm(m)
        out = m.step(1.4, verdict('LINEAR_SLIP'))
        self.assertIsNone(out['action'])
        self.assertEqual(m.state, st.RECOVERY)

    def test_failed_back_switches_to_escape(self):
        """方案 §7：第一次失败后用同类型动作再试一次，第二次失败就收工。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP'))
        out = m.step(0.3, verdict('LINEAR_SLIP'))
        self.assertEqual(out['action'], 'back')
        out = m.on_action_done(1.0, ok=False, moved=0.0)
        self.assertEqual(out['action'], 'back')
        self.assertIn('第 2 次', out['reason'])
        out = m.on_action_done(2.0, ok=False, moved=0.0)
        self.assertEqual(out['action'], 'abort')
        self.assertEqual(m.state, st.FAILED)

    def test_failed_escape_goes_back_to_back(self):
        """旋转卡死：第二次仍是反向微转，不换成后退。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('ANGULAR_SLIP'))
        m.step(0.3, verdict('ANGULAR_SLIP'))
        self.assertEqual(m.pending_action['action'], 'escape')
        out = m.on_action_done(1.0, ok=False, moved=0.0)
        self.assertEqual(out['action'], 'escape')
        self.assertLess(out['yaw'], 0)      # 反向微转

    def test_stall_keeps_escaping(self):
        """堵转：任何情况下都不发后退，只停车上报。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('STALL'))
        # 方案 §12 Tstall：堵转要持续 0.6 s，0.4 s 时还不该动
        out = m.step(0.4, verdict('STALL'))
        self.assertIsNone(out['action'])
        out = m.step(0.8, verdict('STALL'))
        self.assertEqual(out['action'], 'stop')
        self.assertEqual(m.stats['backs'], 0)
        self.assertEqual(m.stats['escapes'], 0)

    def test_failed_state_resets_after_cooldown(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'max_recovery': 2,
                                 'recovery_cooldown_s': 5.0,
                                 'auto_reset_failed': True})
        self._confirm(m)
        for i in range(3):
            m.on_action_done(1.0 + i, ok=False)
        self.assertEqual(m.state, st.FAILED)
        out = m.step(30.0, verdict())
        self.assertEqual(out['action'], 'reset')
        self.assertEqual(m.state, st.NORMAL)

    def test_failed_needs_upper_reset_by_default(self):
        """方案 §9.6：默认不自动复位，要上层 reset（静止 reset_hold_s 后生效）。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'max_recovery': 2,
                                 'recovery_cooldown_s': 5.0})
        self._confirm(m)
        for i in range(3):
            m.on_action_done(1.0 + i, ok=False)
        self.assertEqual(m.state, st.FAILED)
        out = m.step(30.0, verdict())
        self.assertEqual(out.get('action'), None)
        self.assertEqual(m.state, st.FAILED)
        self.assertTrue(m.request_reset(30.0))
        out = m.step(31.0, verdict())          # 只过了 1 s，不够静止 2 s
        self.assertEqual(m.state, st.FAILED)
        out = m.step(32.1, verdict())
        self.assertEqual(out['action'], 'reset')
        self.assertEqual(m.state, st.NORMAL)


if __name__ == '__main__':
    unittest.main()
