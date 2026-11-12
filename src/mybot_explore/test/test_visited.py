# -*- coding: utf-8 -*-
"""覆盖层的单元测试。"""
import math
import unittest

from mybot_explore.visited import VisitedLayer, free_mask_of
from mybot_explore.frontier import Grid


def make_grid(width=40, height=40, res=0.05, origin=(0.0, 0.0), default=-1):
    return Grid(width, height, res, origin[0], origin[1], [default] * (width * height))


class BuildTest(unittest.TestCase):
    def test_from_grid_matches_size(self):
        g = make_grid(20, 10, 0.1)
        v = VisitedLayer.from_grid(g)
        self.assertEqual((v.width, v.height), (20, 10))
        self.assertEqual(v.resolution, 0.1)
        self.assertEqual(v.count(), 0)

    def test_contains(self):
        v = VisitedLayer(10, 10)
        self.assertTrue(v.contains(0, 0))
        self.assertTrue(v.contains(9, 9))
        self.assertFalse(v.contains(-1, 0))
        self.assertFalse(v.contains(10, 0))


class MarkTest(unittest.TestCase):
    def test_mark_is_idempotent(self):
        v = VisitedLayer(10, 10)
        self.assertTrue(v.mark(3, 3))
        self.assertFalse(v.mark(3, 3))
        self.assertEqual(v.count(), 1)

    def test_mark_out_of_bounds_ignored(self):
        v = VisitedLayer(10, 10)
        self.assertFalse(v.mark(-1, 5))
        self.assertFalse(v.mark(5, 99))
        self.assertEqual(v.count(), 0)

    def test_mark_circle_covers_radius(self):
        g = make_grid(40, 40, 0.1)
        v = VisitedLayer.from_grid(g)
        newly = v.mark_circle(2.0, 2.0, 0.3, g)
        self.assertGreater(newly, 0)
        # 圆心自己一定被标
        gx, gy = g.world_to_cell(2.0, 2.0)
        self.assertTrue(v.is_visited(gx, gy))
        # 半径外一点点不该被标（0.3 m / 0.1 m = 3 格）
        far = g.world_to_cell(2.0 + 0.45, 2.0)
        self.assertFalse(v.is_visited(*far))

    def test_mark_circle_near_edge_does_not_crash(self):
        g = make_grid(20, 20, 0.1)
        v = VisitedLayer.from_grid(g)
        v.mark_circle(0.0, 0.0, 0.5, g)          # 角落外
        v.mark_circle(1.95, 1.95, 0.5, g)        # 另一个角
        self.assertGreater(v.count(), 0)
        self.assertLessEqual(v.count(), 20 * 20)

    def test_mark_circle_without_grid(self):
        v = VisitedLayer(40, 40, 0.1, 0.0, 0.0)
        v.mark_circle(1.0, 1.0, 0.2)
        self.assertTrue(v.is_visited(10, 10))

    def test_mark_path_interpolates(self):
        g = make_grid(60, 60, 0.1)
        v = VisitedLayer.from_grid(g)
        v.mark_path([(0.5, 0.5), (5.0, 0.5)], 0.1, g)
        # 直线中间任意一点都该覆盖到
        for x in (1.0, 2.5, 4.0, 4.9):
            gx, gy = g.world_to_cell(x, 0.5)
            self.assertTrue(v.is_visited(gx, gy), 'x=%.1f 没被覆盖' % x)


class StatsTest(unittest.TestCase):
    def test_coverage_ratio(self):
        v = VisitedLayer(4, 1)
        v.mark(0, 0)
        v.mark(1, 0)
        mask = [True, True, True, True]
        self.assertAlmostEqual(v.coverage(mask), 0.5)
        self.assertEqual(v.coverage([False] * 4), 0.0)

    def test_free_mask_of(self):
        g = Grid(3, 1, 0.05, 0, 0, [0, 100, -1])
        self.assertEqual(free_mask_of(g), [True, False, False])

    def test_reset(self):
        v = VisitedLayer(5, 5)
        v.mark(1, 1)
        v.reset()
        self.assertEqual(v.count(), 0)


if __name__ == '__main__':
    unittest.main()
