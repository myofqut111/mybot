"""擦图（用激光把未知变空闲）的纯逻辑测试。"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_explore import carve, frontier  # noqa: E402

FREE = 0
OCC = 100
UNK = -1


def make_grid(rows, resolution=0.1):
    flat = []
    for row in rows:
        flat.extend(row)
    return frontier.Grid(len(rows[0]), len(rows), resolution, 0.0, 0.0, flat)


def scan_of(ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 2,
            range_min=0.05, range_max=10.0):
    return {'ranges': ranges, 'angle_min': angle_min,
            'angle_increment': angle_increment,
            'range_min': range_min, 'range_max': range_max}


class CarveCase(unittest.TestCase):
    def test_single_beam_turns_unknown_to_free(self):
        # 一条 1x8 的走廊，右半边未知；车在最左边，朝正前方打一束 3 m
        g = make_grid([[FREE] * 4 + [UNK] * 4], resolution=0.1)
        pose = (0.05, 0.05, 0.0)          # 朝 +x
        used, hit = carve.carve_scan(
            g, scan_of([3.0], angle_min=0.0, angle_increment=0.0), pose,
            robot_radius_m=0.0)
        self.assertGreater(used, 0)
        self.assertEqual(hit, 0)
        # 3 m 以内的格子应该都变空闲了
        free_cols = [x for x in range(4, 8) if g.data[g.index(x, 0)] == FREE]
        self.assertGreaterEqual(len(free_cols), 3)

    def test_hit_marks_obstacle(self):
        # 全未知的地图里，0.35 m 处打到东西：终点那格应该变成障碍
        g = make_grid([[UNK] * 8], resolution=0.1)
        pose = (0.05, 0.05, 0.0)
        used, hit = carve.carve_scan(
            g, scan_of([0.35], angle_min=0.0, angle_increment=0.0), pose,
            robot_radius_m=0.0)
        self.assertEqual(hit, 1)
        gx, gy = g.world_to_cell(0.05 + 0.35, 0.05)
        self.assertEqual(g.data[g.index(gx, gy)], OCC)

    def test_infinite_range_is_ignored(self):
        g = make_grid([[FREE] * 4 + [UNK] * 4], resolution=0.1)
        pose = (0.05, 0.05, 0.0)
        used, hit = carve.carve_scan(
            g, scan_of([float('inf')], angle_min=0.0, angle_increment=0.0),
            pose, robot_radius_m=0.0)
        self.assertEqual((used, hit), (0, 0))

    def test_max_range_truncates(self):
        g = make_grid([[FREE] + [UNK] * 7], resolution=0.1)
        pose = (0.05, 0.05, 0.0)
        carve.carve_scan(g, scan_of([5.0], angle_min=0.0, angle_increment=0.0),
                         pose, max_range_m=0.3, robot_radius_m=0.0)
        # 0.3 m 之外还是未知
        self.assertEqual(g.data[g.index(6, 0)], UNK)

    def test_robot_footprint_is_cleared(self):
        g = make_grid([[UNK] * 9 for _ in range(9)], resolution=0.1)
        carve.carve_scan(g, scan_of([], angle_min=0.0, angle_increment=1.0),
                         (0.45, 0.45, 0.0), robot_radius_m=0.15)
        self.assertEqual(g.data[g.index(4, 4)], FREE)

    def test_wall_stops_the_beam(self):
        # 中间一堵墙，墙后面不该被擦成空闲
        rows = [[FREE, FREE, OCC, UNK, UNK, UNK]]
        g = make_grid(rows, resolution=0.1)
        carve.carve_scan(g, scan_of([0.15], angle_min=0.0, angle_increment=0.0),
                         (0.05, 0.05, 0.0), robot_radius_m=0.0)
        self.assertEqual(g.data[g.index(5, 0)], UNK)


if __name__ == '__main__':
    unittest.main()
