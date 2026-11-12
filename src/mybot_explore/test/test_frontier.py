"""前沿检测的纯逻辑测试：不启动 ROS，构造小地图直接算。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_explore import frontier  # noqa: E402

FREE = 0
OCC = 100
UNK = -1


def make_grid(rows, resolution=1.0, origin_x=0.0, origin_y=0.0):
    height = len(rows)
    width = len(rows[0])
    flat = []
    for row in rows:
        assert len(row) == width
        flat.extend(row)
    return frontier.Grid(width, height, resolution, origin_x, origin_y, flat)


class FrontierCellsCase(unittest.TestCase):
    def test_all_unknown_has_no_frontier(self):
        g = make_grid([[UNK] * 5 for _ in range(5)])
        self.assertEqual(frontier.frontier_cells(g), [])

    def test_all_free_has_no_frontier(self):
        g = make_grid([[FREE] * 5 for _ in range(5)])
        self.assertEqual(frontier.frontier_cells(g), [])

    def test_boundary_column_is_frontier(self):
        rows = []
        for _ in range(4):
            rows.append([FREE, FREE, FREE, FREE, UNK, UNK])
        g = make_grid(rows)
        cells = frontier.frontier_cells(g)
        self.assertEqual(len(cells), 4)
        self.assertTrue(all(gx == 3 for gx, gy in cells))

    def test_map_border_is_not_treated_as_unknown(self):
        # 地图之外不算未知，否则四边永远是前沿，噪声压不住
        g = make_grid([[FREE] * 4 for _ in range(4)])
        self.assertEqual(frontier.frontier_cells(g), [])


class ClusterCase(unittest.TestCase):
    def test_two_groups_are_separated(self):
        cells = [(0, 0), (1, 0), (10, 10), (11, 10)]
        groups = frontier.cluster(cells)
        self.assertEqual(len(groups), 2)
        self.assertEqual([len(x) for x in groups], [2, 2])

    def test_diagonal_counts_as_connected(self):
        cells = [(0, 0), (1, 1)]
        groups = frontier.cluster(cells)
        self.assertEqual(len(groups), 1)

    def test_min_size_filters_small(self):
        cells = [(0, 0), (10, 10), (11, 10), (12, 10)]
        groups = frontier.cluster(cells, min_size=2)
        self.assertEqual(len(groups), 1)
        self.assertEqual(len(groups[0]), 3)


class FindFrontiersCase(unittest.TestCase):
    def _split_grid(self):
        # 左半空闲、右半未知，10×10，分辨率 1 m，原点 (0,0)
        rows = []
        for _ in range(10):
            rows.append([FREE] * 5 + [UNK] * 5)
        return make_grid(rows)

    def test_single_candidate_in_world_coordinates(self):
        g = self._split_grid()
        cands = frontier.find_frontiers(g, robot_xy=(0.0, 0.0), min_size=2)
        self.assertEqual(len(cands), 1)
        cand = cands[0]
        self.assertAlmostEqual(cand['x'], 4.5, places=2)   # 第 5 列格子中心
        self.assertAlmostEqual(cand['y'], 4.5, places=2)
        self.assertGreater(cand['unknown'], 0)

    def test_resolution_and_origin_are_respected(self):
        rows = [[FREE, FREE, FREE, UNK, UNK]]
        g = make_grid(rows, resolution=0.5, origin_x=-1.0, origin_y=2.0)
        cands = frontier.find_frontiers(g, robot_xy=(-1.0, 2.0), min_size=1)
        self.assertEqual(len(cands), 1)
        # 前沿在 gx=2 → x = -1 + 2.5*0.5 = 0.25；gy=0 → y = 2 + 0.5*0.5 = 2.25
        self.assertAlmostEqual(cands[0]['x'], 0.25, places=3)
        self.assertAlmostEqual(cands[0]['y'], 2.25, places=3)

    def test_fully_explored_map_returns_nothing(self):
        g = make_grid([[OCC, OCC, OCC], [OCC, OCC, OCC], [OCC, OCC, OCC]])
        self.assertEqual(frontier.find_frontiers(g, robot_xy=(0, 0)), [])

    def test_centroid_on_obstacle_is_moved_to_free(self):
        # 前沿格子在左边，但质心那一格被人为改成障碍
        rows = [
            [UNK, FREE, OCC, FREE, UNK],
            [UNK, FREE, FREE, FREE, UNK],
        ]
        g = make_grid(rows)
        g.data[1 * 5 + 2] = OCC          # (2,1) 障碍，质心可能落这里
        cands = frontier.find_frontiers(g, robot_xy=(2.0, 1.0), min_size=1)
        for cand in cands:
            self.assertTrue(g.is_free(cand['gx'], cand['gy']))

    def test_distance_weighting_prefers_near(self):
        # 两个孤立的未知块，各自只有一个相邻空闲格 → 正好两个候选
        rows = [
            [FREE, UNK, OCC, OCC, FREE, UNK],
            [OCC, UNK, OCC, OCC, OCC, UNK],
            [OCC, OCC, OCC, OCC, OCC, UNK],
        ]
        g = make_grid(rows)
        cands = frontier.find_frontiers(g, robot_xy=(0.5, 0.5), min_size=1,
                                        w_distance=1.0, w_gain=0.0)
        self.assertEqual(len(cands), 2)
        self.assertLess(cands[0]['distance'], cands[1]['distance'])
        self.assertAlmostEqual(cands[0]['x'], 0.5, places=3)

    def test_gain_weighting_prefers_more_unknown(self):
        # 右边那片未知比左边大，按增益排它应该在前
        rows = [
            [FREE, UNK, OCC, OCC, FREE, UNK, UNK],
            [OCC, UNK, OCC, OCC, FREE, UNK, UNK],
            [OCC, OCC, OCC, OCC, FREE, UNK, UNK],
        ]
        g = make_grid(rows)
        cands = frontier.find_frontiers(g, robot_xy=(0.5, 0.5), min_size=1,
                                        unknown_radius_m=2.0,
                                        w_distance=0.0, w_gain=1.0)
        self.assertEqual(len(cands), 2)
        self.assertGreater(cands[0]['unknown'], cands[1]['unknown'])
        self.assertGreater(cands[0]['x'], 3.0)          # 赢的是右边那块

    def test_max_candidates_is_capped(self):
        rows = [[FREE, UNK] * 10]
        g = make_grid(rows)
        cands = frontier.find_frontiers(g, robot_xy=(0.5, 0.5), min_size=1,
                                        max_candidates=3)
        self.assertLessEqual(len(cands), 3)

    def test_no_robot_pose_still_works(self):
        g = self._split_grid()
        cands = frontier.find_frontiers(g, robot_xy=None, min_size=2)
        self.assertEqual(len(cands), 1)
        self.assertEqual(cands[0]['distance'], 0.0)

    def test_unknown_radius_only_counts_within_radius(self):
        rows = [[FREE, UNK, UNK, UNK, FREE]]
        g = make_grid(rows)
        small = frontier.find_frontiers(g, robot_xy=(0.5, 0.5), min_size=1,
                                        unknown_radius_m=0.6)
        big = frontier.find_frontiers(g, robot_xy=(0.5, 0.5), min_size=1,
                                      unknown_radius_m=3.0)
        self.assertLess(small[0]['unknown'], big[0]['unknown'])


class GoalClearanceCase(unittest.TestCase):
    """前沿点天生贴着边界，直接当目标会被 nav2 拒（"目标在障碍里"）。"""

    def _corridor(self, width=9, height=5):
        # 四周是墙，中间一条横向走廊；未知只留在最上面那行的右端，
        # 这样前沿质心天然贴着上墙，正好检验"往外挪"。
        rows = [[OCC] * width for _ in range(height)]
        for y in range(1, height - 1):
            rows[y] = [OCC] + [FREE] * (width - 1)
        rows[1][width - 1] = UNK
        rows[2][width - 1] = OCC      # 只留一个前沿格，避免聚类被顺手带偏
        return make_grid(rows, resolution=0.05)

    def test_goal_is_pushed_away_from_walls(self):
        g = self._corridor()
        plain = frontier.find_frontiers(g, robot_xy=(0.05, 0.1), min_size=1)
        safe = frontier.find_frontiers(g, robot_xy=(0.05, 0.1), min_size=1,
                                       goal_clearance_m=0.05)
        self.assertEqual(len(plain), 1)
        self.assertEqual(len(safe), 1)
        # 走廊高 3 格，膨胀 1 格后只有正中间那行合格
        tight = frontier.tight_cells(g, 0.05)
        self.assertFalse(bool(tight[g.index(safe[0]['gx'], safe[0]['gy'])]))
        self.assertNotEqual(plain[0]['gy'], safe[0]['gy'])

    def test_candidate_dropped_when_nowhere_to_stand(self):
        # 单格宽的死胡同：前沿永远贴着墙，留不出安全落脚点
        rows = [[OCC, OCC, OCC],
                [OCC, FREE, UNK],
                [OCC, OCC, OCC]]
        g = make_grid(rows, resolution=0.05)
        self.assertEqual(len(frontier.find_frontiers(g, min_size=1)), 1)
        self.assertEqual(
            len(frontier.find_frontiers(g, min_size=1, goal_clearance_m=0.05)), 0)


class GridGeometryCase(unittest.TestCase):
    def test_world_cell_roundtrip(self):
        g = frontier.Grid.from_msg  # 只是确认 API 存在
        grid = make_grid([[FREE] * 4 for _ in range(4)],
                         resolution=0.2, origin_x=-1.0, origin_y=0.5)
        gx, gy = grid.world_to_cell(-0.9, 0.6)
        self.assertEqual((gx, gy), (0, 0))
        x, y = grid.cell_to_world(gx, gy)
        self.assertAlmostEqual(x, -0.9, places=6)
        self.assertAlmostEqual(y, 0.6, places=6)
        self.assertTrue(callable(g))


if __name__ == '__main__':
    unittest.main()
