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
        # 0.5*0.06=0.03，但受 back_min_m=0.10 托底
        self.assertAlmostEqual(out['distance'], 0.10)
        self.assertGreaterEqual(out['distance'], m.p['back_min_m'])

    def test_confirmation_needs_time(self):
        m = st.SlipStateMachine({'confirm_windows': 2, 'confirm_window_s': 1.0})
        m.step(0.0, verdict('LINEAR_SLIP'))
        out = m.step(0.3, verdict('LINEAR_SLIP'))     # 只有 0.3 s
        self.assertIsNone(out['action'])
        out = m.step(1.2, verdict('LINEAR_SLIP'))
        self.assertEqual(out['action'], 'back')

    def test_back_distance_clamped(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP', denc=2.0))
        out = m.step(0.3, verdict('LINEAR_SLIP', denc=2.0))
        self.assertLessEqual(out['distance'], m.p['back_max_m'])

    def test_drop_back_to_normal(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('LINEAR_SLIP'))
        m.step(0.3, verdict())
        self.assertEqual(m.state, st.NORMAL)


class StallTest(unittest.TestCase):
    def test_stall_escapes_without_backing(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0})
        m.step(0.0, verdict('STALL'))
        out = m.step(0.3, verdict('STALL'))
        self.assertEqual(out['action'], 'escape')
        self.assertNotIn('distance', out)
        self.assertEqual(m.stats['backs'], 0)


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
        self.assertEqual(m.state, st.CHECK)
        self.assertIsNone(out['action'])

    def test_short_back_escalates(self):
        m = self._to_recovery()
        out = m.on_action_done(0.6, ok=True, moved=0.01)   # 只退到期望的 20%
        # 退回去了但没退够：不再重复同一种方式，改原地转向
        self.assertEqual(out['action'], 'escape')
        self.assertEqual(m.state, st.RECOVERY)

    def test_repeat_failures_request_inflate(self):
        m = self._to_recovery()
        actions = []
        t = 0.6
        for _ in range(4):
            out = m.on_action_done(t, ok=False, moved=0.0)
            actions.append(out['action'])
            t += 0.6
        self.assertIn('inflate', actions)
        self.assertIn('abort', actions)

    def test_clean_windows_recover(self):
        m = self._to_recovery()
        m.on_action_done(0.6, ok=True, moved=0.10)
        actions = []
        for i in range(3):
            out = m.step(2.0 + i * 0.3, verdict())
            actions.append(out['action'])
        self.assertIn('recovered', actions)
        self.assertEqual(m.state, st.NORMAL)
        self.assertEqual(m.recoveries, 0)

    def test_slip_during_check_goes_back_to_recovery(self):
        m = self._to_recovery()
        m.on_action_done(0.6, ok=True, moved=0.10)
        m.step(2.0, verdict())
        out = m.step(2.3, verdict('LINEAR_SLIP'))
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
        """后退顶不动时应该换姿势（转向），而不是反复用同一种方式撞同一个东西。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'inflate_try': 9})
        m.step(0.0, verdict('LINEAR_SLIP'))
        out = m.step(0.3, verdict('LINEAR_SLIP'))
        self.assertEqual(out['action'], 'back')
        out = m.on_action_done(1.0, ok=False, moved=0.0)
        self.assertEqual(out['action'], 'escape')
        self.assertIn('转向', out['reason'])

    def test_failed_escape_goes_back_to_back(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'inflate_try': 9})
        m.step(0.0, verdict('LINEAR_SLIP'))
        m.step(0.3, verdict('LINEAR_SLIP'))
        m.on_action_done(1.0, ok=False, moved=0.0)      # -> escape
        self.assertEqual(m.pending_action['action'], 'escape')
        out = m.on_action_done(2.0, ok=False, moved=0.0)
        self.assertEqual(out['action'], 'back')

    def test_stall_keeps_escaping(self):
        """堵转本来就不该后退：失败后仍然转向。"""
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'inflate_try': 9})
        m.step(0.0, verdict('STALL'))
        out = m.step(0.3, verdict('STALL'))
        self.assertEqual(out['action'], 'escape')
        out = m.on_action_done(1.0, ok=False, moved=0.0)
        self.assertEqual(out['action'], 'escape')

    def test_failed_state_resets_after_cooldown(self):
        m = st.SlipStateMachine({'confirm_window_s': 0.0, 'max_recovery': 2,
                                 'recovery_cooldown_s': 5.0})
        self._confirm(m)
        for i in range(3):
            m.on_action_done(1.0 + i, ok=False)
        self.assertEqual(m.state, st.FAILED)
        out = m.step(30.0, verdict())
        self.assertEqual(out['action'], 'reset')
        self.assertEqual(m.state, st.NORMAL)


if __name__ == '__main__':
    unittest.main()
