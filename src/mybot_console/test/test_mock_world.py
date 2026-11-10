# -*- coding: utf-8 -*-
"""mock_world：假地图/射线/运动学。

这些数字一旦错，表现出来是"点云穿墙""车往墙里走"，从 UI 上很难一眼看出是
几何错还是渲染错。所以每条都钉一个具体数字，而不是"大致大于 0"。
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_console import mock_world  # noqa: E402


class TestMap(unittest.TestCase):
    def setUp(self):
        self.data, self.w, self.h, self.meta = mock_world.build_map(
            resolution=0.05)

    def test_outer_walls_are_walls(self):
        self.assertEqual(self.w, self.h)
        self.assertEqual(mock_world.cell_at(
            self.data, self.w, self.h, 0.05, 0.0, 0.0, 0.05, 5.0),
            mock_world.WALL, '左墙没标')
        self.assertEqual(mock_world.cell_at(
            self.data, self.w, self.h, 0.05, 0.0, 0.0, 9.95, 5.0),
            mock_world.WALL, '右墙没标')

    def test_center_is_free(self):
        self.assertEqual(mock_world.cell_at(
            self.data, self.w, self.h, 0.05, 0.0, 0.0, 2.0, 2.0),
            mock_world.FREE)

    def test_out_of_bounds_counts_as_wall(self):
        # 出界按墙算，否则射线会从地图边缘漏出去，画面上就是一束穿墙的线
        self.assertEqual(mock_world.cell_at(
            self.data, self.w, self.h, 0.05, 0.0, 0.0, -0.5, 5.0),
            mock_world.WALL)

    def test_divider_has_door_gap(self):
        # 隔断在 x≈5.0，y∈[4.0,5.2) 是门洞：洞里应可通行，洞外是墙
        inside_gap = mock_world.cell_at(
            self.data, self.w, self.h, 0.05, 0.0, 0.0, 5.0, 4.6)
        outside_gap = mock_world.cell_at(
            self.data, self.w, self.h, 0.05, 0.0, 0.0, 5.0, 2.0)
        self.assertEqual(inside_gap, mock_world.FREE)
        self.assertEqual(outside_gap, mock_world.WALL)


class TestRaycast(unittest.TestCase):
    def setUp(self):
        self.data, self.w, self.h, self.meta = mock_world.build_map(
            resolution=0.05)

    def _ray(self, x, y, yaw, max_range=8.0):
        return mock_world.raycast(
            self.data, self.w, self.h, 0.05, 0.0, 0.0,
            x, y, yaw, [0.0], max_range=max_range)[0]

    def test_ray_hits_left_wall(self):
        # 从 (2,5) 朝 -x 打：左墙内沿在 x=0.10，距离 1.90
        hit = self._ray(2.0, 5.0, math.pi)
        self.assertAlmostEqual(hit, 1.90, delta=0.10)

    def test_ray_hits_block(self):
        # 从 (2,2.8) 朝 +x：障碍块左沿在 x=2.6，距离 0.60
        hit = self._ray(2.0, 2.8, 0.0)
        self.assertAlmostEqual(hit, 0.60, delta=0.10)

    def test_ray_into_gap_passes(self):
        # 从 (4.0,4.6) 朝 +x 穿过门洞，应打到右墙（x=9.90）而不是 1 m 就停
        hit = self._ray(4.0, 4.6, 0.0)
        self.assertGreater(hit, 5.0)

    def test_no_hit_returns_inf(self):
        # 很短量程、朝门洞：打不到任何东西
        hit = self._ray(4.0, 4.6, 0.0, max_range=1.0)
        self.assertEqual(hit, float('inf'))

    def test_all_beams_finite_or_inf(self):
        angles = [(-math.pi + 2.0 * math.pi * i / 240) for i in range(240)]
        ranges = mock_world.raycast(
            self.data, self.w, self.h, 0.05, 0.0, 0.0,
            5.0, 5.0, 0.0, angles)
        self.assertEqual(len(ranges), 240)
        for value in ranges:
            self.assertTrue(math.isfinite(value) or value == float('inf'))
            if math.isfinite(value):
                self.assertGreaterEqual(value, 0.0)
                self.assertLessEqual(value, 8.0 + 0.05)


class TestScanFields(unittest.TestCase):
    def test_inf_becomes_over_range(self):
        fields = mock_world.scan_msg_fields(
            [0.0, 0.1], [1.5, float('inf')], range_max=8.0)
        self.assertAlmostEqual(fields['ranges'][0], 1.5)
        self.assertGreater(fields['ranges'][1], 8.0)

    def test_clamped_to_range(self):
        fields = mock_world.scan_msg_fields([0.0], [0.0], range_min=0.1)
        self.assertGreaterEqual(fields['ranges'][0], 0.1)

    def test_increment(self):
        fields = mock_world.scan_msg_fields([0.0, 0.5, 1.0], [1, 1, 1])
        self.assertAlmostEqual(fields['angle_increment'], 0.5)


class TestMotion(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(mock_world.wrap_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(mock_world.wrap_angle(-3 * math.pi), math.pi)
        self.assertAlmostEqual(mock_world.wrap_angle(0.5), 0.5)

    def test_step_forward(self):
        x, y, yaw = mock_world.step((0.0, 0.0, 0.0), 1.0, 0.0, 1.0)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_step_in_place_rotation_keeps_position(self):
        x, y, yaw = mock_world.step((1.0, 2.0, 0.0), 0.0, 1.0, 1.0)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(y, 2.0, places=6)
        self.assertAlmostEqual(yaw, 1.0, places=6)

    def test_step_arc_is_not_a_straight_line(self):
        # 边转边走时若用起点航向积分，会画出一条直折线；用中间航向才是圆弧
        x, y, _ = mock_world.step((0.0, 0.0, 0.0), 1.0, math.pi / 2, 1.0)
        self.assertGreater(y, 0.3)

    def test_step_ignores_negative_dt(self):
        pose = (1.0, 1.0, 0.0)
        self.assertEqual(mock_world.step(pose, 1.0, 0.0, -1.0), pose)


class TestGeometry(unittest.TestCase):
    def test_quaternion_round_trip(self):
        for yaw in (-2.5, -0.3, 0.0, 0.7, 3.0):
            quat = mock_world.quaternion_of_yaw(yaw)

            class _Q(object):
                x = quat['x']
                y = quat['y']
                z = quat['z']
                w = quat['w']

            self.assertAlmostEqual(
                mock_world.yaw_of_quaternion(_Q()), yaw, places=6)

    def test_approach_turns_first_when_backwards(self):
        # 目标在正后方：先原地转，不该一边倒车一边撞过去
        vx, wz = mock_world.approach((0.0, 0.0, 0.0), (-1.0, 0.0))
        self.assertEqual(vx, 0.0)
        self.assertGreater(abs(wz), 0.1)

    def test_approach_stops_when_close(self):
        vx, wz = mock_world.approach((0.0, 0.0, 0.0), (0.05, 0.0))
        self.assertEqual((vx, wz), (0.0, 0.0))

    def test_approach_drives_towards_target(self):
        vx, wz = mock_world.approach((0.0, 0.0, 0.0), (2.0, 0.0))
        self.assertGreater(vx, 0.0)
        self.assertAlmostEqual(wz, 0.0, places=6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
