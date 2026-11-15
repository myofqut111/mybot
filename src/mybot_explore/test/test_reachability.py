# -*- coding: utf-8 -*-
"""可达性 / 封缝的单元测试：这几条决定"车会不会往凳子腿中间钻"。"""
import unittest

from mybot_explore import frontier as fr


def grid_from(rows, res=0.1):
    """'.' 空闲、'#' 障碍、'?' 未知；列数取最宽一行。"""
    height = len(rows)
    width = len(rows[0])
    data = []
    for row in rows:
        for ch in row.ljust(width, '?'):
            if ch == '.':
                data.append(0)
            elif ch == '#':
                data.append(100)
            else:
                data.append(-1)
    return fr.Grid(width, height, res, 0.0, 0.0, data)


class DistanceTest(unittest.TestCase):
    def test_obstacle_distance(self):
        g = grid_from([".....",
                       "..#..",
                       "....."])
        dist = fr.obstacle_distance(g)
        idx = g.index(2, 1)
        self.assertEqual(dist[idx], 0.0)
        # 相邻格的距离就是一个格子分辨率
        self.assertAlmostEqual(dist[g.index(1, 1)], 0.1, places=6)
        self.assertAlmostEqual(dist[g.index(0, 1)], 0.2, places=6)


class TraversableTest(unittest.TestCase):
    def test_clearance_matters(self):
        g = grid_from(["#####",
                       ".....",
                       "#####"])
        # 走廊只有一格高（0.1 m），要求 0.15 m 余量时整条都不可通行
        tight = fr.traversable_mask(g, 0.15)
        self.assertEqual(sum(tight), 0)
        loose = fr.traversable_mask(g, 0.05)
        self.assertEqual(sum(loose), 5)

    def test_unknown_is_not_traversable(self):
        g = grid_from([".??"])
        mask = fr.traversable_mask(g, 0.0)
        self.assertEqual(sum(mask), 1)


class ReachableTest(unittest.TestCase):
    def test_wall_blocks(self):
        g = grid_from(["....",
                       "####",
                       "...."])
        trav = fr.traversable_mask(g, 0.0)
        reach = fr.reachable_mask(g, trav, (0.05, 0.05))
        self.assertEqual(reach[g.index(0, 0)], 1)
        self.assertEqual(reach[g.index(3, 0)], 1)
        self.assertEqual(reach[g.index(0, 2)], 0)   # 墙那边

    def test_thin_gap_is_blocked_by_clearance(self):
        # 两堵墙中间一格宽的缝，车半径大就过不去
        g = grid_from(["..#..",
                       "...?.",
                       "..#.."])
        trav = fr.traversable_mask(g, 0.15)
        reach = fr.reachable_mask(g, trav, (0.05, 0.05))
        self.assertEqual(reach[g.index(4, 1)], 0)


class SealGapsTest(unittest.TestCase):
    def test_thin_slit_is_closed(self):
        # 两根"腿"中间隔两格（0.2 m），封缝半径 0.2 m 后中间应变成障碍
        g = grid_from([".....",
                       "..#..",
                       ".....",
                       "..#..",
                       "....."])
        sealed = fr.seal_gaps(g, 0.2)
        self.assertGreaterEqual(sealed.at(2, 2), 50)

    def test_wide_gap_survives(self):
        # 中间隔 5 格（0.5 m），封缝半径 0.15 m（每边 2 格）封不住它
        g = grid_from([".........",
                       "..#.....#",
                       "........."])
        sealed = fr.seal_gaps(g, 0.15)
        self.assertLess(sealed.at(4, 1), 50)


class FindFrontiersReachTest(unittest.TestCase):
    def test_frontier_behind_wall_is_dropped(self):
        rows = ["......?",
                "######.",
                "......?"]
        g = grid_from(rows)
        base = fr.find_frontiers(g, robot_xy=(0.05, 0.05), min_size=1,
                                 min_separation_m=0.0)
        self.assertTrue(base)
        filtered = fr.find_frontiers(g, robot_xy=(0.05, 0.05), min_size=1,
                                     min_separation_m=0.0,
                                     reach_clearance_m=0.05,
                                     reach_search_m=0.3)
        # 墙那头的候选必须没了
        for c in filtered:
            self.assertLess(c['y'], 0.15)


if __name__ == '__main__':
    unittest.main()
