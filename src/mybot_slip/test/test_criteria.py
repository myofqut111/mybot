# -*- coding: utf-8 -*-
"""判据层：每类滑移都能被点名，正常情况不误报。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from mybot_slip import criteria


def fit(dx=0.0, dy=0.0, ok=True, quality=1.0, coverage=1.0, dtheta=0.0):
    return {'dx': dx, 'dy': dy, 'dtheta': dtheta, 'ok': ok, 'quality': quality,
            'coverage': coverage, 'beams': int(300 * coverage), 'residual': 0.01,
            'reason': '' if ok else 'low'}


def win(denc=0.05, dtheta_enc=0.0, dtheta_imu=0.0, d_left=None, d_right=None,
        cmd=(0.18, 0.0), dt=0.3, valid=True, warmup=False, odom_ok=True,
        wheel_ok=True, imu_ok=True, scan_ok=True):
    return {
        't0': 0.0, 't1': dt, 'dt': dt, 'valid': valid, 'warmup': warmup,
        'reason': '' if valid else '指令覆盖 10%',
        'cmd_linear': cmd[0], 'cmd_angular': cmd[1], 'cmd_coverage': 1.0,
        'denc': denc, 'dtheta_enc': dtheta_enc, 'odom_ok': odom_ok,
        'd_left': denc if d_left is None else d_left,
        'd_right': denc if d_right is None else d_right, 'wheel_ok': wheel_ok,
        'dtheta_imu': dtheta_imu, 'imu_ok': imu_ok,
        'scan_ok': scan_ok, 'scan_prev': None, 'scan_cur': None, 'scan_dt': 0.1,
    }


class CriteriaTest(unittest.TestCase):
    def test_normal_straight_is_clean(self):
        r = criteria.evaluate(win(denc=0.054, dtheta_imu=0.0), fit=fit(dx=0.052))
        self.assertFalse(r['slip'])
        self.assertIsNone(r['type'])

    def test_linear_slip_detected(self):
        r = criteria.evaluate(win(denc=0.060, dtheta_imu=0.0), fit=fit(dx=0.000))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'LINEAR_SLIP')
        self.assertGreaterEqual(r['ratios']['linear'], 0.9)

    def test_low_speed_slip_not_missed(self):
        # 0.1 m/s × 0.3 s = 0.03 m，固定 0.05 m 门槛会漏；自适应后应判出
        r = criteria.evaluate(win(denc=0.030, cmd=(0.10, 0.0), dtheta_imu=0.0),
                              fit=fit(dx=0.000))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'LINEAR_SLIP')

    def test_angular_slip_detected(self):
        # 原地转：左右轮反向、与编码器转角 0.15rad 自洽，但陀螺只有 0.03
        b = 0.175
        dl = -0.5 * b * 0.15
        dr = 0.5 * b * 0.15
        r = criteria.evaluate(win(denc=0.0, dtheta_enc=0.15, dtheta_imu=0.03,
                                  cmd=(0.0, 0.5), d_left=dl, d_right=dr),
                              fit=fit(dx=0.0))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'ANGULAR_SLIP')

    def test_angular_sign_mismatch_is_not_slip(self):
        # 真机上见过：编码器说 +0.69 rad/s，陀螺说 -0.30。方向相反不是滑移，
        # 是左右轮接线/陀螺符号的问题，报成 ANGULAR_SLIP 会把人带偏。
        b = 0.175
        dl = -0.5 * b * 0.15
        dr = 0.5 * b * 0.15
        r = criteria.evaluate(win(denc=0.0, dtheta_enc=0.15, dtheta_imu=-0.06,
                                  cmd=(0.0, 0.5), d_left=dl, d_right=dr),
                              fit=fit(dx=0.0))
        self.assertFalse(r['slip'])
        self.assertFalse(r['detail'].get('angular_same_sign', True))
        self.assertIn('angular_sign_mismatch', r['detail'])

    def test_angular_sign_check_can_be_disabled(self):
        b = 0.175
        dl = -0.5 * b * 0.15
        dr = 0.5 * b * 0.15
        r = criteria.evaluate(win(denc=0.0, dtheta_enc=0.15, dtheta_imu=-0.06,
                                  cmd=(0.0, 0.5), d_left=dl, d_right=dr),
                              params={'angular_require_same_sign': False},
                              fit=fit(dx=0.0))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'ANGULAR_SLIP')

    def test_stall_detected(self):
        r = criteria.evaluate(win(denc=0.0005, dtheta_enc=0.0, cmd=(0.18, 0.0),
                                  d_left=0.0, d_right=0.0), fit=fit(dx=0.0))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'STALL')

    def test_no_command_no_stall(self):
        r = criteria.evaluate(win(denc=0.0, cmd=(0.0, 0.0)), fit=fit(dx=0.0))
        self.assertFalse(r['slip'])

    def test_wheel_left_slip(self):
        r = criteria.evaluate(win(denc=0.09, d_left=0.02, d_right=0.09, dtheta_imu=0.0),
                              fit=fit(dx=0.09))
        self.assertTrue(r['slip'])
        self.assertIn(r['type'], ('WHEEL_L', 'LINEAR_SLIP'))

    def test_no_wheel_ratio_when_body_not_moving(self):
        """车体没动时单轮期望位移无定义，不能吐出 -1.0 的假滑移率。"""
        r = criteria.evaluate(win(denc=0.037, d_left=0.037, d_right=0.037,
                                  dtheta_imu=0.009), fit=fit(dx=0.0))
        self.assertNotIn('wheel_l', r['ratios'])
        self.assertNotIn('wheel_r', r['ratios'])
        self.assertEqual(r['type'], 'LINEAR_SLIP')

    def test_lateral_slip(self):
        r = criteria.evaluate(win(denc=0.05, cmd=(0.18, 0.0)),
                              fit=fit(dx=0.05, dy=0.06))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'LATERAL_SLIP')

    def test_overrun(self):
        r = criteria.evaluate(win(denc=0.02, dtheta_imu=0.0), fit=fit(dx=0.10))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'OVERRUN')

    def test_untrusted_scan_cannot_claim_linear_slip(self):
        r = criteria.evaluate(win(denc=0.06, dtheta_imu=0.0),
                              fit=fit(dx=0.0, ok=False, quality=0.1, coverage=0.2))
        self.assertFalse(r['slip'])

    def test_warmup_window_skipped(self):
        r = criteria.evaluate(win(warmup=True), fit=fit())
        self.assertFalse(r['usable'])
        self.assertFalse(r['slip'])

    def test_invalid_window_skipped(self):
        r = criteria.evaluate(win(valid=False), fit=fit())
        self.assertFalse(r['usable'])

    def test_release_hysteresis(self):
        # 滑移率 0.5：在 r_high(0.6) 以下、但在 r_low(0.36) 以上 -> 不解除
        w = win(denc=0.06, dtheta_imu=0.0)
        released, _ = criteria.is_released(w, fit=fit(dx=0.03))
        self.assertFalse(released)

    def test_release_when_clean(self):
        w = win(denc=0.06, dtheta_imu=0.0)
        released, _ = criteria.is_released(w, fit=fit(dx=0.058))
        self.assertTrue(released)

    def test_sensor_conflict_needs_turn_command(self):
        # 直行时（cmd_w≈0）陀螺有零偏、编码器本来就不转，不该算矛盾。
        r = criteria.evaluate(win(denc=0.054, dtheta_enc=0.0, dtheta_imu=0.20,
                                  cmd=(0.18, 0.0)), fit=fit(dx=0.052))
        self.assertNotEqual(r['type'], 'SENSOR_CONFLICT')

    def test_sensor_conflict_fires_when_asked_to_turn(self):
        # 让车转 0.9 rad/s，编码器说没转、陀螺说转了 → 真矛盾
        # denc 取 0.02：既高于堵转门槛（否则先判 STALL），又不足以构成直行滑移
        r = criteria.evaluate(win(denc=0.02, dtheta_enc=0.0, dtheta_imu=0.20,
                                  cmd=(0.0, 0.9)), fit=fit(dx=0.0))
        self.assertEqual(r['type'], 'SENSOR_CONFLICT')

    # ---------- 方案 §12：IMU 超时 / 倾斜 / 转向放大 ----------
    def test_imu_stale_window_is_not_judged(self):
        w = win(denc=0.060, dtheta_imu=0.0)
        w['imu_stale'] = True
        r = criteria.evaluate(w, fit=fit(dx=0.0))
        self.assertFalse(r['usable'])
        self.assertFalse(r['slip'])

    def test_tilt_over_ten_degrees_is_not_judged(self):
        w = win(denc=0.060, dtheta_imu=0.0)
        w['tilt_deg'] = 12.0
        r = criteria.evaluate(w, fit=fit(dx=0.0))
        self.assertFalse(r['usable'])
        self.assertFalse(r['slip'])

    def test_tilt_within_limit_still_judges(self):
        w = win(denc=0.060, dtheta_imu=0.0)
        w['tilt_deg'] = 6.0
        r = criteria.evaluate(w, fit=fit(dx=0.0))
        self.assertTrue(r['slip'])
        self.assertEqual(r['type'], 'LINEAR_SLIP')

    def test_turning_scales_threshold(self):
        # 0.6 rad/s 转向：阈值 ×1.5，短差 0.045 m 不再算滑移
        w = win(denc=0.060, dtheta_enc=0.24, dtheta_imu=0.24, cmd=(0.18, 0.6))
        r = criteria.evaluate(w, fit=fit(dx=0.015))
        self.assertEqual(r['detail'].get('threshold_scale'), 1.5)
        self.assertNotEqual(r['type'], 'LINEAR_SLIP')
        # 同样的差，在直行时就算滑移
        r2 = criteria.evaluate(win(denc=0.060, dtheta_imu=0.0), fit=fit(dx=0.0))
        self.assertTrue(r2['slip'])


if __name__ == '__main__':
    unittest.main()
