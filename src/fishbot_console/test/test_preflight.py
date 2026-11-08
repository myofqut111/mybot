# -*- coding: utf-8 -*-
"""发巡检前的"点位在地图里吗"检查（纯函数，不需要 ROS）。"""

import unittest

from fishbot_console import data


META = {'width': 242, 'height': 288, 'resolution': 0.05,
        'origin_x': -1.5, 'origin_y': -1.62}


class MapBoundsTest(unittest.TestCase):

    def test_bounds_from_meta(self):
        x0, y0, x1, y1 = data.map_bounds(META)
        self.assertAlmostEqual(x0, -1.5)
        self.assertAlmostEqual(y0, -1.62)
        self.assertAlmostEqual(x1, -1.5 + 242 * 0.05)      # 10.6
        self.assertAlmostEqual(y1, -1.62 + 288 * 0.05)     # 12.78

    def test_empty_meta_gives_none(self):
        self.assertIsNone(data.map_bounds({}))
        self.assertIsNone(data.map_bounds(None))
        self.assertIsNone(data.map_bounds({'width': 0, 'height': 10,
                                           'resolution': 0.05}))

    def test_broken_meta_gives_none(self):
        self.assertIsNone(data.map_bounds({'width': 'x', 'height': 1,
                                           'resolution': 0.05}))


class PointsOutsideTest(unittest.TestCase):

    bounds = data.map_bounds(META)

    def test_inside_route_is_clean(self):
        points = [{'point_id': 'p1', 'x': 8.19, 'y': 3.17},
                  {'point_id': 'p2', 'x': 7.5, 'y': 5.0}]
        self.assertEqual(data.points_outside(points, self.bounds), [])

    def test_far_outside_route_is_flagged(self):
        points = [{'point_id': 'p1', 'x': -1.0, 'y': -9.4},
                  {'point_id': 'p2', 'x': 8.19, 'y': 3.17}]
        outside = data.points_outside(points, self.bounds)
        self.assertEqual([p['point_id'] for p in outside], ['p1'])
        self.assertEqual(outside[0]['x'], -1.0)

    def test_margin_counts_as_outside(self):
        # 贴着上边界（y1=12.78）：12.7 落在 0.2 的余量里，跑不进去
        points = [{'point_id': 'edge', 'x': 5.0, 'y': 12.7}]
        self.assertEqual(len(data.points_outside(points, self.bounds)), 1)

    def test_no_bounds_means_no_check(self):
        points = [{'point_id': 'p1', 'x': 999.0, 'y': 999.0}]
        self.assertEqual(data.points_outside(points, None), [])

    def test_bad_point_is_skipped(self):
        points = [{'point_id': 'p1'}, {'point_id': 'p2', 'x': 1.0, 'y': 1.0}]
        self.assertEqual(data.points_outside(points, self.bounds), [])


class DescribeOutsideTest(unittest.TestCase):

    def test_message_names_route_and_points(self):
        text = data.describe_outside('my_route3', [
            {'point_id': 'p1', 'x': -1.0, 'y': -9.4},
            {'point_id': 'p2', 'x': -2.0, 'y': -10.0}])
        self.assertIn('my_route3', text)
        self.assertIn('2 个点', text)
        self.assertIn('p1', text)
        self.assertIn('-1.00', text)

    def test_empty_outside_gives_empty_text(self):
        self.assertEqual(data.describe_outside('main', []), '')

    def test_unnamed_route_still_readable(self):
        text = data.describe_outside('', [{'point_id': 'p1', 'x': 0, 'y': 0}])
        self.assertIn('未命名', text)


if __name__ == '__main__':
    unittest.main()
