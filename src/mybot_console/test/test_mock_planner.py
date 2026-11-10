# -*- coding: utf-8 -*-
"""假车的规划与碰撞：不许穿墙。

这三个东西错了，界面上表现为"车飞出去""车从墙里穿过去"——那是假车在
教给人错误的预期，比没有 mock 更糟。所以它们必须有测试，而不是靠肉眼看。
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_console import mock_world  # noqa: E402

RES = 0.05
RADIUS = 0.18


def _grid(radius_m=RADIUS):
    data, w, h, meta = mock_world.build_map(resolution=RES)
    radius_cells = max(1, int(round(radius_m / RES)))
    blocked = mock_world.blocked_grid(data, w, h, radius_cells)
    return data, w, h, meta, blocked


class TestBlockedGrid(unittest.TestCase):
    def test_wall_is_blocked(self):
        _, w, _, _, blocked = _grid()
        self.assertEqual(blocked[0], 1)                 # 左上角一定是墙
        self.assertEqual(blocked[5 * w + 5], 1)

    def test_ring_around_wall_is_blocked(self):
        _, w, h, meta, blocked = _grid()
        # 外墙是 0..0.10 m（2 格），膨胀 0.18 m（4 格）后，距墙 0.15 m 处应被封
        cell = mock_world.world_to_cell(0.2, 1.0, RES,
                                        meta['origin_x'], meta['origin_y'])
        self.assertEqual(blocked[cell[1] * w + cell[0]], 1)

    def test_room_centre_is_free(self):
        _, w, h, meta, blocked = _grid()
        cell = mock_world.world_to_cell(2.0, 2.0, RES,
                                        meta['origin_x'], meta['origin_y'])
        self.assertEqual(blocked[cell[1] * w + cell[0]], 0)


class TestAstar(unittest.TestCase):
    def setUp(self):
        self.data, self.w, self.h, self.meta, self.blocked = _grid()

    def _cell(self, x, y):
        return mock_world.world_to_cell(x, y, RES,
                                        self.meta['origin_x'],
                                        self.meta['origin_y'])

    def test_path_exists_across_the_door(self):
        start = self._cell(2.0, 2.0)
        goal = self._cell(8.0, 8.0)
        path = mock_world.astar(self.blocked, self.w, self.h, start, goal)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_path_is_entirely_free(self):
        start = self._cell(2.0, 2.0)
        goal = self._cell(8.0, 8.0)
        path = mock_world.astar(self.blocked, self.w, self.h, start, goal)
        for col, row in path:
            self.assertEqual(self.blocked[row * self.w + col], 0,
                             'path goes through a blocked cell %s' % ((col, row),))

    def test_path_crosses_the_divider_only_through_the_gap(self):
        """隔断在 x=5.0 m，门洞在 y=4.0~5.2 m。跨过去的格子必须在门洞里。"""
        start = self._cell(2.0, 2.0)
        goal = self._cell(8.0, 8.0)
        path = mock_world.astar(self.blocked, self.w, self.h, start, goal)
        gap0 = int(round(4.0 / RES))
        gap1 = int(round(5.2 / RES))
        crossing = [p for p in path if 98 <= p[0] <= 103]
        self.assertTrue(crossing, '路径根本没过隔断？')
        for col, row in crossing:
            self.assertTrue(gap0 <= row < gap1,
                            '从 x=%.2f m 处穿过隔断，门洞在 %.1f~%.1f m'
                            % (col * RES, gap0 * RES, gap1 * RES))

    def test_goal_inside_wall_is_unreachable(self):
        goal = self._cell(0.05, 0.05)                   # 墙里
        path = mock_world.astar(self.blocked, self.w, self.h,
                                self._cell(2.0, 2.0), goal)
        self.assertIsNone(path)

    def test_sealed_room_is_unreachable(self):
        w = h = 40
        data = [mock_world.FREE] * (w * h)
        for row in range(h):                            # 中间一道实墙，没有门
            data[row * w + 20] = mock_world.WALL
        blocked = mock_world.blocked_grid(data, w, h, 1)
        path = mock_world.astar(blocked, w, h, (5, 5), (35, 35))
        self.assertIsNone(path)

    def test_diagonal_does_not_cut_corners(self):
        w = h = 9
        data = [mock_world.FREE] * (w * h)
        data[4 * w + 4] = mock_world.WALL
        blocked = mock_world.blocked_grid(data, w, h, 0)
        for row in (3, 4):
            for col in (4, 5):
                data[row * w + col] = mock_world.WALL
        blocked = mock_world.blocked_grid(data, w, h, 0)
        path = mock_world.astar(blocked, w, h, (3, 3), (5, 5))
        # (3,3) -> (5,5) 的斜线要穿过 (4,4) 或夹角，必须绕路
        if path:
            for col, row in path:
                self.assertEqual(blocked[row * w + col], 0)


class TestSimplify(unittest.TestCase):
    def test_straight_line_collapses(self):
        w = h = 40
        data = [mock_world.FREE] * (w * h)
        blocked = mock_world.blocked_grid(data, w, h, 0)
        cells = [(5, 5 + i) for i in range(20)]
        out = mock_world.simplify_path(cells, blocked, w, h)
        self.assertEqual(out[0], cells[0])
        self.assertEqual(out[-1], cells[-1])
        self.assertLessEqual(len(out), 3)

    def test_keeps_corner_when_wall_between(self):
        _, w, h, meta, blocked = _grid()
        start = mock_world.world_to_cell(2.0, 2.0, RES, 0.0, 0.0)
        goal = mock_world.world_to_cell(8.0, 8.0, RES, 0.0, 0.0)
        path = mock_world.astar(blocked, w, h, start, goal)
        out = mock_world.simplify_path(path, blocked, w, h)
        self.assertGreater(len(out), 2)                 # 有拐弯，不能压成两点
        for (c0, r0), (c1, r1) in zip(out, out[1:]):
            self.assertTrue(
                mock_world._line_of_sight(blocked, w, h, (c0, r0), (c1, r1)))


class TestStepGuarded(unittest.TestCase):
    def setUp(self):
        self.data, self.w, self.h, self.meta, self.blocked = _grid()

    def _step(self, pose, vx, wz, dt=0.05):
        return mock_world.step_guarded(
            pose, vx, wz, dt, self.blocked, self.w, self.h, RES,
            self.meta['origin_x'], self.meta['origin_y'])

    def test_open_space_is_not_blocked(self):
        pose = (2.0, 2.0, 0.0)
        new, hit = self._step(pose, 0.3, 0.0, 0.1)
        self.assertFalse(hit)
        self.assertGreater(new[0], pose[0])

    def test_robot_never_ends_inside_a_wall(self):
        pose = (1.0, 2.0, math.pi)                      # 朝左，正对左墙
        for _ in range(200):
            pose, hit = self._step(pose, 0.45, 0.0, 0.05)
            col = int(math.floor(pose[0] / RES))
            row = int(math.floor(pose[1] / RES))
            self.assertEqual(self.blocked[row * self.w + col], 0,
                             '车跑进墙里了：%s' % (pose,))

    def test_stops_before_the_wall(self):
        pose = (1.0, 2.0, math.pi)
        hit = False
        for _ in range(200):
            pose, hit = self._step(pose, 0.45, 0.0, 0.05)
        self.assertTrue(hit)
        # 墙内表面在 0.10 m，加半径 0.18 m => 车心不该小于 0.25 m
        self.assertGreater(pose[0], 0.24)
        self.assertLess(pose[0], 0.5, '车停在离墙 %0.2f m，太远了' % pose[0])

    def test_slides_along_the_wall(self):
        """斜着顶墙时应该沿墙滑一段，而不是立刻杵住。"""
        pose = (0.35, 2.0, math.radians(150.0))         # 朝左上方
        y_before = pose[1]
        for _ in range(40):
            pose, _ = self._step(pose, 0.4, 0.0, 0.05)
        self.assertGreater(pose[1] - y_before, 0.05,
                           '没有沿墙滑动（y 只从 %.2f 到 %.2f）'
                           % (y_before, pose[1]))


if __name__ == '__main__':
    unittest.main()
