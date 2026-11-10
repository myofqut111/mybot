# -*- coding: utf-8 -*-
"""mock_world 的 inflate/crop 与「代价地图是否真的包住墙」的测验。

这几个函数一错，表现出来是「膨胀带画到墙里面去了」或者「局部窗口一圈全黑」，
在 UI 上很难分辨是渲染错还是数据错，所以在这里钉死。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_console import mock_world  # noqa: E402


class TestInflate(unittest.TestCase):
    def test_wall_keeps_lethal(self):
        data, w, h, _ = mock_world.build_map()
        cost = mock_world.inflate(data, w, h, 4)
        for index, value in enumerate(data):
            if value == mock_world.WALL:
                self.assertEqual(cost[index], mock_world.WALL)

    def test_free_cell_next_to_wall_gets_cost(self):
        data = [mock_world.FREE] * 25
        data[12] = mock_world.WALL            # 5x5 的中间
        cost = mock_world.inflate(data, 5, 5, 2)
        self.assertGreater(cost[11], 0)       # 左边一格
        self.assertGreater(cost[7], 0)        # 上面一格
        self.assertEqual(cost[0], 0)          # 角落够远，保持 0

    def test_cost_decays_with_distance(self):
        data = [mock_world.FREE] * 49
        data[24] = mock_world.WALL            # 7x7 的中间
        cost = mock_world.inflate(data, 7, 7, 3)
        near, far = cost[23], cost[21]
        self.assertGreater(near, far)
        self.assertGreater(far, 0)

    def test_radius_zero_is_noop(self):
        data, w, h, _ = mock_world.build_map()
        self.assertEqual(mock_world.inflate(data, w, h, 0), list(data))

    def test_does_not_mutate_input(self):
        data = [mock_world.FREE] * 9
        data[4] = mock_world.WALL
        before = list(data)
        mock_world.inflate(data, 3, 3, 1)
        self.assertEqual(data, before)

    def test_cap_at_max_cost(self):
        data = [mock_world.FREE] * 9
        data[4] = mock_world.WALL
        cost = mock_world.inflate(data, 3, 3, 1, max_cost=90)
        self.assertEqual(cost[0], 0)          # 对角超出半径 1（切比雪夫距离 1）
        self.assertLessEqual(cost[1], 90)


class TestCrop(unittest.TestCase):
    def test_crop_shape(self):
        data = list(range(100))
        out = mock_world.crop(data, 10, 10, 2, 3, 4, 5)
        self.assertEqual(len(out), 20)

    def test_crop_values(self):
        data = list(range(100))
        out = mock_world.crop(data, 10, 10, 2, 3, 2, 2)
        # (col=2,row=3) -> 32, (col=3,row=3) -> 33
        self.assertEqual(out[:2], [32, 33])
        self.assertEqual(out[2:], [42, 43])

    def test_out_of_range_filled(self):
        data = [0] * 4
        out = mock_world.crop(data, 2, 2, -1, -1, 2, 2, fill=7)
        self.assertEqual(out, [7, 7, 7, 0])


if __name__ == '__main__':
    unittest.main()
