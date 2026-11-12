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


class _Position(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Pose(object):
    def __init__(self, x, y):
        self.position = _Position(x, y)


class _PoseStamped(object):
    """geometry_msgs/PoseStamped：真正的位置在这里面一层。"""

    def __init__(self, x, y):
        self.pose = _Pose(x, y)


class _PatrolPoint(object):
    def __init__(self, point_id, x, y, stamped=True):
        self.point_id = point_id
        self.pose = _PoseStamped(x, y) if stamped else _Pose(x, y)


class PointXyTest(unittest.TestCase):
    """0.7.9 的真 bug：按 pose.position 取，每个点都读成 (0, 0)。"""

    def test_posestamped(self):
        self.assertEqual(data.point_xy(_PatrolPoint('p1', 8.19, 3.17)),
                         (8.19, 3.17))

    def test_bare_pose(self):
        self.assertEqual(data.point_xy(_PatrolPoint('p1', -1.2, -9.4,
                                                    stamped=False)),
                         (-1.2, -9.4))

    def test_dict_shape(self):
        self.assertEqual(data.point_xy({'pose': {'position': {'x': 1.5,
                                                              'y': 2.5}}}),
                         (1.5, 2.5))

    def test_missing_position_is_zero(self):
        self.assertEqual(data.point_xy({'point_id': 'p1'}), (0.0, 0.0))

    def test_points_outside_uses_real_coords(self):
        """回归：main2 的点在 map_v2 里，不能被判成图外。"""
        bounds = data.map_bounds({'width': 242, 'height': 288,
                                  'resolution': 0.05,
                                  'origin_x': -1.5, 'origin_y': -1.62})
        points = [{'point_id': p.point_id, 'x': x, 'y': y}
                  for p, x, y in ((_PatrolPoint('p1', 8.19, 3.17), 8.19, 3.17),
                                  (_PatrolPoint('p2', 7.54, 4.62), 7.54, 4.62),
                                  (_PatrolPoint('p3', 6.42, 5.87), 6.42, 5.87))]
        self.assertEqual(data.points_outside(points, bounds), [])


if __name__ == '__main__':
    unittest.main()
