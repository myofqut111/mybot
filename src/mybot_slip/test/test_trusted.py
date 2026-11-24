# -*- coding: utf-8 -*-
"""《补充说明》§3/§4/§5.2：可信运动状态与恢复准入。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from mybot_slip import trusted as T


def v(slip=True, vtype='LINEAR_SLIP'):
    return {'slip': slip, 'type': vtype if slip else None, 'ratios': {},
            'reasons': [], 'detail': {}}


class TrustedTest(unittest.TestCase):
    def test_normal_is_s0(self):
        t = T.TrustedState()
        st, _ = t.update(v(False), machine_state='normal')
        self.assertEqual(st, T.S0)
        self.assertTrue(t.map_allowed)
        self.assertTrue(t.nav_allowed)
        self.assertIsNone(t.speed_cap)

    def test_suspect_is_s1_and_limits_speed(self):
        t = T.TrustedState()
        st, _ = t.update(v(), machine_state='slip')
        self.assertEqual(st, T.S1)
        self.assertFalse(t.map_allowed)   # 暂停持久地图写入
        self.assertTrue(t.nav_allowed)    # 但还没禁止导航
        self.assertEqual(t.speed_cap, (0.10, 0.50))

    def test_s1_needs_two_clean_windows_to_exit(self):
        t = T.TrustedState()
        t.update(v(), machine_state='slip')
        st, _ = t.update(v(False), machine_state='normal')
        self.assertEqual(st, T.S1)             # 一个干净窗不够（滞回）
        st, _ = t.update(v(False), machine_state='normal')
        self.assertEqual(st, T.S0)

    def test_confirmed_is_s2(self):
        t = T.TrustedState()
        st, _ = t.update(v(), machine_state='recovery', confirmed=True)
        self.assertEqual(st, T.S2)
        # S2 位姿已冻结：写进去的就是脏图，所以建图要停
        self.assertFalse(t.map_allowed)
        self.assertFalse(t.nav_allowed)

    def test_sensor_conflict_escalates_s3_then_s4(self):
        t = T.TrustedState()
        t.update(v(vtype='SENSOR_CONFLICT'), machine_state='slip')
        st, reason = t.update(v(vtype='SENSOR_CONFLICT'), machine_state='slip')
        self.assertEqual(st, T.S3)
        self.assertIn('冲突', reason)
        self.assertFalse(t.nav_allowed)
        t.update(v(vtype='SENSOR_CONFLICT'), machine_state='slip')
        st, _ = t.update(v(vtype='SENSOR_CONFLICT'), machine_state='slip')
        self.assertEqual(st, T.S4)
        self.assertFalse(t.nav_allowed)
        self.assertFalse(t.map_allowed)

    def test_no_independent_observation_is_s4(self):
        t = T.TrustedState()
        st, reason = t.update(v(), machine_state='slip', scan_ok=False)
        self.assertEqual(st, T.S4)
        self.assertIn('独立位移观测', reason)

    def test_pose_jump_enters_s3_until_relocalized(self):
        t = T.TrustedState()
        t.note_pose_jump(0.20)
        st, reason = t.update(v(False), machine_state='normal')
        self.assertEqual(st, T.S3)
        self.assertIn('位姿跳变', reason)
        t.note_relocalized()
        for _ in range(2):
            st, _ = t.update(v(False), machine_state='normal')
        self.assertEqual(st, T.S0)


class AdmissionTest(unittest.TestCase):
    def base(self, **kw):
        args = dict(state=T.S2, scan_ok=True, imu_ok=True, tilt_deg=0.0,
                    consecutive_fail=0, max_fail=2, max_tilt_deg=10.0)
        args.update(kw)
        return T.check_admission(v(), **args)

    def test_pass_when_everything_ok(self):
        ok, why = self.base()
        self.assertTrue(ok, why)

    def test_reject_without_type(self):
        ok, why = T.check_admission(v(False), state=T.S2, scan_ok=True, imu_ok=True)
        self.assertFalse(ok)
        self.assertIn('类型不可信', why)

    def test_reject_in_s3_s4(self):
        self.assertFalse(self.base(state=T.S3)[0])
        self.assertFalse(self.base(state=T.S4)[0])

    def test_reject_without_scan(self):
        ok, why = self.base(scan_ok=False)
        self.assertFalse(ok)
        self.assertIn('独立位移观测', why)

    def test_reject_when_imu_down(self):
        self.assertFalse(self.base(imu_ok=False)[0])

    def test_reject_when_tilted(self):
        ok, why = self.base(tilt_deg=12.0)
        self.assertFalse(ok)
        self.assertIn('倾斜', why)

    def test_reject_after_two_failures(self):
        ok, why = self.base(consecutive_fail=2)
        self.assertFalse(ok)
        self.assertIn('连续失败', why)


if __name__ == '__main__':
    unittest.main()
