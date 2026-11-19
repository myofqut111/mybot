# -*- coding: utf-8 -*-
"""推台阶 / 顶矮墙判据（只靠 B：扫描没变但 odom 在涨）。纯逻辑，不需要 ROS。"""

import unittest

from mybot_explore import slip

DEFAULTS = {
    'slip_enabled': True,
    'slip_check_period_s': 0.0,        # 测试里不节流
    'slip_window_s': 3.0,
    'slip_odom_min_m': 0.05,
    'slip_map_odom_min_m': 0.08,
    'slip_scan_change_ratio': 0.15,
    'slip_scan_diff_m': 0.03,
    'slip_scan_near_m': 3.0,
    'slip_max_omega': 0.30,
    'slip_min_valid_ratio': 0.40,
    'slip_min_hits': 8,
    'slip_min_speed': 0.05,
    'slip_confirm_s': 1.5,
    'slip_hold_timeout_s': 10.0,
    'blocked_enabled': True,
    'block_window_s': 3.0,
    'block_odom_min_m': 0.30,
    'block_scan_change_ratio': 0.10,
    'block_scan_near_m': 1.0,
    'block_scan_diff_m': 0.03,
    'block_min_hits': 8,
    'block_min_speed': 0.05,
    'block_max_omega': 0.20,
    'block_confirm_s': 1.0,
    'block_cooldown_s': 5.0,
}


def make_scan(distance, count=90):
    """一圈等距扫描（1° 一束）。"""
    return {'ranges': [distance] * count,
            'angle_min': -1.57,
            'angle_increment': 3.14159 * 2 / count,
            'range_min': 0.05,
            'range_max': 8.0}


class BlockedTest(unittest.TestCase):

    def setUp(self):
        self.det = slip.SlipDetector(dict(DEFAULTS))

    def feed(self, t, x, scan, linear=0.15, angular=0.0):
        return self.det.update(t, odom_pose=(x, 0.0, 0.0), map_odom=(0.0, 0.0, 0.0),
                               scan=scan, linear=linear, angular=angular)

    def test_stuck_on_step_is_detected(self):
        """odom 一路涨、扫描一模一样 → 判"被顶住"。"""
        scan = make_scan(0.8)
        actions = []
        for i in range(12):                     # 0..5.5 s，0.5 s 一拍
            t = i * 0.5
            actions.append(self.feed(t, 0.05 * i, scan).get('action'))
        self.assertIn('blocked', actions, '顶住台阶没被判出来: %s' % actions)

    def test_real_motion_is_not_blocked(self):
        """真在走：扫描随距离变化 → 不能误判。"""
        actions = []
        for i in range(12):
            t = i * 0.5
            actions.append(self.feed(t, 0.05 * i, make_scan(0.8 + 0.05 * i)).get('action'))
        self.assertNotIn('blocked', actions)

    def test_not_moving_is_not_blocked(self):
        """没让车走（指令为 0）→ 不算顶住，否则停着也报。"""
        scan = make_scan(0.8)
        actions = [self.feed(i * 0.5, 0.0, scan, linear=0.0).get('action')
                   for i in range(12)]
        self.assertNotIn('blocked', actions)

    def test_turning_is_not_blocked(self):
        """原地转时扫描会变，这里用恒定扫描 + 大角速度模拟，必须被排除。"""
        scan = make_scan(0.8)
        actions = [self.feed(i * 0.5, 0.05 * i, scan, angular=0.8).get('action')
                   for i in range(12)]
        self.assertNotIn('blocked', actions)

    def test_open_space_is_not_blocked(self):
        """1 m 内没有命中（空旷）→ 不判。"""
        scan = make_scan(6.0)
        actions = [self.feed(i * 0.5, 0.05 * i, scan).get('action') for i in range(12)]
        self.assertNotIn('blocked', actions)

    def test_needs_minimum_odom_travel(self):
        """odom 只挪了一点点（0.2 m < 0.30）→ 不判。"""
        scan = make_scan(0.8)
        actions = [self.feed(i * 0.5, 0.02 * i, scan).get('action') for i in range(12)]
        self.assertNotIn('blocked', actions)

    def test_cooldown_prevents_spam(self):
        """判出一次之后要有冷却，不能每拍都报。"""
        scan = make_scan(0.8)
        count = 0
        for i in range(30):
            if self.feed(i * 0.5, 0.05 * i, scan).get('action') == 'blocked':
                count += 1
        self.assertGreaterEqual(count, 1)
        self.assertLessEqual(count, 2, '冷却没起作用，报了 %d 次' % count)

    def test_disabled_switch(self):
        self.det.params['blocked_enabled'] = False
        scan = make_scan(0.8)
        actions = [self.feed(i * 0.5, 0.05 * i, scan).get('action') for i in range(12)]
        self.assertNotIn('blocked', actions)


class BlockedTurnCase(unittest.TestCase):
    """原地顶住（转不动）：|w| 大、odom 的 yaw 变了、扫描没变。

    旧判据里 |angular| > block_max_omega 直接排除，所以这种最常见的卡死形态
    从来不会触发（用户现场：台阶顶住后轮子一直转、车不停、点云跟着换方向）。
    """

    def _params(self, **kw):
        params = dict(slip.DEFAULTS)
        params['block_confirm_s'] = 0.0
        params.update(kw)
        return params

    def _scan(self, base=3.0):
        return {'ranges': [base] * 360, 'angle_min': -3.14159,
                'angle_increment': 6.28318 / 360.0,
                'range_min': 0.05, 'range_max': 6.0}

    def test_turning_in_place_counts_as_blocked(self):
        det = slip.SlipDetector(self._params())
        scan = self._scan()
        # 一直在转，但扫描一点没变，odom 的 yaw 从 0 涨到 0.6
        det.update(0.0, odom_pose=(0.0, 0.0, 0.0), map_odom=(0.0, 0.0),
                   scan=scan, linear=0.0, angular=0.5)
        det.update(2.0, odom_pose=(0.0, 0.0, 0.0), map_odom=(0.0, 0.0),
                   scan=scan, linear=0.0, angular=0.5)
        det.update(3.2, odom_pose=(0.01, 0.0, 0.4), map_odom=(0.0, 0.0),
                   scan=scan, linear=0.0, angular=0.5)
        last = det.update(4.4, odom_pose=(0.01, 0.0, 0.6), map_odom=(0.0, 0.0),
                          scan=scan, linear=0.0, angular=0.5)
        self.assertEqual(last.get('action'), 'blocked')

    def test_real_rotation_does_not_fire(self):
        det = slip.SlipDetector(self._params())
        turn = self._scan(3.0)
        turned = {'ranges': [3.0 + 0.5 * (i % 3) for i in range(360)],
                  'angle_min': -3.14159, 'angle_increment': 6.28318 / 360.0,
                  'range_min': 0.05, 'range_max': 6.0}
        det.update(0.0, odom_pose=(0.0, 0.0, 0.0), map_odom=(0.0, 0.0),
                   scan=turn, linear=0.0, angular=0.5)
        last = det.update(3.2, odom_pose=(0.01, 0.0, 0.6), map_odom=(0.0, 0.0),
                          scan=turned, linear=0.0, angular=0.5)
        self.assertNotEqual(last.get('action'), 'blocked')

    def test_low_step_with_few_near_hits_still_fires(self):
        """低台阶：激光看不见，1 m 内没几束——近场半径要自动放大。"""
        det = slip.SlipDetector(self._params(block_min_hits=50))
        scan = self._scan(3.0)          # 全在 3 m，1 m 内 0 束
        det.update(0.0, odom_pose=(0.0, 0.0, 0.0), map_odom=(0.0, 0.0),
                   scan=scan, linear=0.15, angular=0.0)
        det.update(3.2, odom_pose=(0.30, 0.0, 0.0), map_odom=(0.0, 0.0),
                   scan=scan, linear=0.15, angular=0.0)
        last = det.update(4.4, odom_pose=(0.45, 0.0, 0.0), map_odom=(0.0, 0.0),
                          scan=scan, linear=0.15, angular=0.0)
        self.assertEqual(last.get('action'), 'blocked')


if __name__ == '__main__':
    unittest.main()
