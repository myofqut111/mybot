# -*- coding: utf-8 -*-
"""``blocked.py`` 的用例：短墙几何、写进网格、禁试区存取。"""

import json
import math
import os
import shutil
import tempfile
import unittest

from mybot_explore import blocked as blocked_mod
from mybot_explore import frontier


def make_grid(width=40, height=40, resolution=0.05, origin=(-1.0, -1.0)):
    data = [0] * (width * height)
    return frontier.Grid(width, height, resolution, origin[0], origin[1], data)


class WallSegmentCase(unittest.TestCase):
    def test_length_and_center(self):
        segment = blocked_mod.wall_segment(0.0, 0.0, 0.0, offset_m=0.22, length_m=0.5)
        length = math.hypot(segment['x2'] - segment['x1'], segment['y2'] - segment['y1'])
        self.assertAlmostEqual(length, 0.5, places=6)
        self.assertAlmostEqual(segment['cx'], 0.22, places=6)
        self.assertAlmostEqual(segment['cy'], 0.0, places=6)

    def test_segment_is_perpendicular_to_heading(self):
        segment = blocked_mod.wall_segment(1.0, 2.0, math.pi / 2)
        # 朝 +y，墙应沿 x 方向
        self.assertAlmostEqual(segment['y1'], segment['y2'], places=6)
        self.assertNotAlmostEqual(segment['x1'], segment['x2'], places=6)

    def test_rotated_heading_keeps_center_on_heading(self):
        yaw = math.radians(45.0)
        segment = blocked_mod.wall_segment(0.0, 0.0, yaw, offset_m=0.4)
        self.assertAlmostEqual(segment['cx'], 0.4 * math.cos(yaw), places=6)
        self.assertAlmostEqual(segment['cy'], 0.4 * math.sin(yaw), places=6)


class StampWallCase(unittest.TestCase):
    def test_stamps_a_line_of_cells(self):
        grid = make_grid()
        segment = blocked_mod.wall_segment(0.0, 0.0, 0.0, offset_m=0.22, length_m=0.5)
        written = blocked_mod.stamp_wall(grid, segment)
        self.assertGreaterEqual(written, 9)          # 0.5 m / 0.05 m = 10 格
        occupied = sum(1 for value in grid.data if value >= 50)
        self.assertEqual(occupied, written)

    def test_outside_cells_are_skipped(self):
        grid = make_grid(width=4, height=4, origin=(100.0, 100.0))
        segment = blocked_mod.wall_segment(0.0, 0.0, 0.0)
        self.assertEqual(blocked_mod.stamp_wall(grid, segment), 0)

    def test_thickness_widens_the_wall(self):
        thin = make_grid()
        thick = make_grid()
        segment = blocked_mod.wall_segment(0.0, 0.0, 0.0)
        blocked_mod.stamp_wall(thin, segment)
        blocked_mod.stamp_wall(thick, segment, thickness_cells=1)
        self.assertGreater(sum(1 for v in thick.data if v >= 50),
                           sum(1 for v in thin.data if v >= 50))


class BlockedStoreCase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, 'blocked.json')
        self.store = blocked_mod.BlockedStore(self.path)

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_add_and_hits_accumulate(self):
        self.store.add(1.0, 2.0, 0.5, now='t1')
        spot = self.store.add(1.1, 2.0, 0.6, now='t2')
        self.assertEqual(len(self.store.spots), 1)
        self.assertEqual(spot['hits'], 2)
        self.assertEqual(spot['first_seen'], 't1')
        self.assertEqual(spot['last_seen'], 't2')

    def test_far_spot_creates_new_entry(self):
        self.store.add(0.0, 0.0, 0.0, now='t1')
        self.store.add(5.0, 5.0, 0.0, now='t2')
        self.assertEqual(len(self.store.spots), 2)

    def test_save_and_load_roundtrip(self):
        self.store.add(1.0, 2.0, 1.2, now='t1', slip_seconds=3.2)
        self.store.save()
        other = blocked_mod.BlockedStore(self.path)
        spots = other.load()
        self.assertEqual(len(spots), 1)
        self.assertAlmostEqual(spots[0]['slip_seconds'], 3.2)
        with open(self.path, 'r') as handle:
            payload = json.load(handle)
        self.assertEqual(payload['version'], blocked_mod.VERSION)

    def test_load_missing_file_is_empty(self):
        store = blocked_mod.BlockedStore(os.path.join(self.dir, 'nope.json'))
        self.assertEqual(store.load(), [])

    def test_load_broken_file_is_empty(self):
        with open(self.path, 'w') as handle:
            handle.write('{not json')
        self.assertEqual(self.store.load(), [])

    def test_keepout(self):
        self.store.add(0.0, 0.0, 0.0, now='t1')
        self.assertTrue(self.store.in_keepout(0.5, 0.0, 1.0))
        self.assertFalse(self.store.in_keepout(2.0, 0.0, 1.0))

    def test_to_keepouts(self):
        self.store.add(1.0, 1.0, 0.0, now='t1')
        keepouts = self.store.to_keepouts(radius_m=1.5)
        self.assertEqual(keepouts[0]['radius'], 1.5)
        self.assertEqual(keepouts[0]['x'], 1.0)


if __name__ == '__main__':
    unittest.main()
