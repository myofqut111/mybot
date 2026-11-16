# -*- coding: utf-8 -*-
"""B 批：存图 / 去毛刺 / 孤立障碍率"""
import os
import shutil
import tempfile
import unittest

from mybot_explore import frontier, pgm


def build(width=40, height=20, resolution=0.1):
    data = [0] * (width * height)

    def at(x, y):
        return y * width + x
    # 一圈墙
    for x in range(width):
        data[at(x, 0)] = 100
        data[at(x, height - 1)] = 100
    for y in range(height):
        data[at(0, y)] = 100
        data[at(width - 1, y)] = 100
    # 一块实心障碍（3x3）
    for y in range(5, 8):
        for x in range(5, 8):
            data[at(x, y)] = 100
    # 三个孤立噪点
    data[at(20, 5)] = 100
    data[at(25, 10)] = 100
    data[at(30, 14)] = 100
    return frontier.Grid(width, height, resolution, -2.0, -1.0, data)


class PgmWriteCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='mybot_map_')

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_round_trip_keeps_size_and_values(self):
        grid = build()
        base = os.path.join(self.tmp, 'out.pgm')
        pgm_path, yaml_path = pgm.write_map(base, grid)
        self.assertTrue(os.path.exists(pgm_path))
        self.assertTrue(os.path.exists(yaml_path))
        again = pgm.load_map(yaml_path)
        self.assertEqual(again.width, grid.width)
        self.assertEqual(again.height, grid.height)
        self.assertEqual(list(again.data), list(grid.data))

    def test_yaml_keeps_resolution_and_origin(self):
        grid = build(resolution=0.05)
        _, yaml_path = pgm.write_map(os.path.join(self.tmp, 'a'), grid)
        again = pgm.load_map(yaml_path)
        self.assertAlmostEqual(again.resolution, 0.05, places=6)
        self.assertAlmostEqual(again.origin_x, grid.origin_x, places=6)
        self.assertAlmostEqual(again.origin_y, grid.origin_y, places=6)


class DespikeCase(unittest.TestCase):
    def test_isolated_rate_counts_single_points(self):
        grid = build()
        rate = frontier.isolated_rate(grid)
        # 3 个孤立点 / 障碍总数
        obstacles = sum(1 for v in grid.data if v >= 50)
        self.assertAlmostEqual(rate, 3.0 / obstacles, places=6)

    def test_median_despike_removes_isolated_keeps_wall(self):
        grid = build()
        clean = frontier.median_despike(grid)
        # 孤立点没了
        self.assertLess(clean.data[5 * grid.width + 20], 50)
        self.assertLess(clean.data[10 * grid.width + 25], 50)
        # 实心 3x3 还在
        self.assertGreaterEqual(clean.data[6 * grid.width + 6], 50)
        # 外墙还在
        self.assertGreaterEqual(clean.data[0], 50)
        self.assertLess(frontier.isolated_rate(clean), frontier.isolated_rate(grid))


class CountUnknownCase(unittest.TestCase):
    def test_counts_only_unknown(self):
        grid = build()
        self.assertEqual(frontier.count_unknown(grid), 0)
        for i in range(5):
            grid.data[i] = -1
        self.assertEqual(frontier.count_unknown(grid), 5)
        self.assertEqual(frontier.count_unknown(None), 0)


if __name__ == '__main__':
    unittest.main()
