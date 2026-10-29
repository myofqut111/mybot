#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_render.py：PNG 真的能被解出来，地图上下不能颠倒。"""
import os
import struct
import sys
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import map_render  # noqa: E402


def read_png(payload):
    """最小 PNG 解码器（只认 filter=0 的 8bit RGB），用来验证编码正确。"""
    assert payload[:8] == b'\x89PNG\r\n\x1a\n'
    pos = 8
    width = height = None
    idat = b''
    while pos < len(payload):
        length = struct.unpack('>I', payload[pos:pos + 4])[0]
        tag = payload[pos + 4:pos + 8]
        body = payload[pos + 8:pos + 8 + length]
        crc = struct.unpack('>I', payload[pos + 8 + length:pos + 12 + length])[0]
        assert crc == (zlib.crc32(tag + body) & 0xFFFFFFFF), 'bad crc'
        if tag == b'IHDR':
            width, height = struct.unpack('>II', body[:8])
        elif tag == b'IDAT':
            idat += body
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = width * 3 + 1
    rows = []
    for row in range(height):
        line = raw[row * stride:(row + 1) * stride]
        assert line[0] == 0, 'unexpected filter'
        rows.append(line[1:])
    return width, height, rows


class TestWritePng(unittest.TestCase):
    def test_roundtrip_small(self):
        pixels = bytes([255, 0, 0] * 2 + [0, 255, 0] * 2)
        png = map_render.write_png(2, 2, pixels)
        width, height, rows = read_png(png)
        self.assertEqual((width, height), (2, 2))
        self.assertEqual(rows[0][:3], b'\xff\x00\x00')
        self.assertEqual(rows[1][3:6], b'\x00\xff\x00')

    def test_bad_size(self):
        with self.assertRaises(ValueError):
            map_render.write_png(0, 4, b'')
        with self.assertRaises(ValueError):
            map_render.write_png(2, 2, b'\x00' * 11)


class TestColors(unittest.TestCase):
    def test_extremes(self):
        self.assertEqual(map_render.occupancy_colors(-1),
                         map_render.COLOR_UNKNOWN)
        self.assertEqual(map_render.occupancy_colors(None),
                         map_render.COLOR_UNKNOWN)
        self.assertEqual(map_render.occupancy_colors(0),
                         map_render.COLOR_FREE)
        self.assertEqual(map_render.occupancy_colors(100),
                         map_render.COLOR_OCCUPIED)

    def test_midpoint_is_darker_than_free(self):
        mid = map_render.occupancy_colors(50)
        free = map_render.COLOR_FREE
        occ = map_render.COLOR_OCCUPIED
        self.assertTrue(free[0] >= mid[0] >= occ[0])
        self.assertTrue(free[1] >= mid[1] >= occ[1])


class TestGridToPng(unittest.TestCase):
    def test_vertical_flip(self):
        # data 是自下而上的：row0=占据, row1=自由
        grid = [100, 100, 0, 0]
        png = map_render.grid_to_png(grid, 2, 2)
        _w, _h, rows = read_png(png)
        # PNG 第一行应该是"地图上边" = data 的 row1 = 自由
        self.assertEqual(tuple(rows[0][:3]), map_render.COLOR_FREE)
        self.assertEqual(tuple(rows[1][:3]), map_render.COLOR_OCCUPIED)

    def test_unknown_value(self):
        png = map_render.grid_to_png([-1], 1, 1)
        _w, _h, rows = read_png(png)
        self.assertEqual(tuple(rows[0][:3]), map_render.COLOR_UNKNOWN)

    def test_short_data_raises(self):
        with self.assertRaises(ValueError):
            map_render.grid_to_png([0, 0, 0], 2, 2)


class TestPixelMath(unittest.TestCase):
    def test_roundtrip(self):
        # 像素代表一个格子，center 约定下往返误差不超过半个格子
        origin_x, origin_y, res, height = -2.0, 1.0, 0.05, 40
        for wx, wy in ((0.0, 1.0), (-1.9, 1.9), (0.5, 1.5)):
            px, py = map_render.world_to_pixel(wx, wy, origin_x, origin_y,
                                               res, height)
            back_x, back_y = map_render.pixel_to_world(
                px, py, origin_x, origin_y, res, height)
            self.assertAlmostEqual(back_x, wx, delta=res)
            self.assertAlmostEqual(back_y, wy, delta=res)

    def test_flip_relation(self):
        # 世界 y 越大 -> 像素 y 越小（图片上方）
        px_top, py_top = map_render.world_to_pixel(0, 2.0, 0, 0, 0.05, 40)
        px_bot, py_bot = map_render.world_to_pixel(0, 0.0, 0, 0, 0.05, 40)
        self.assertLess(py_top, py_bot)

    def test_bad_resolution(self):
        with self.assertRaises(ValueError):
            map_render.world_to_pixel(0, 0, 0, 0, 0, 10)


class TestMapMeta(unittest.TestCase):
    class _Pose(object):
        x = 1.5
        y = -2.5

    class _Info(object):
        resolution = 0.1
        width = 20
        height = 30
        origin = None

    def test_defaults(self):
        meta = map_render.map_meta(self._Info())
        self.assertEqual(meta['width'], 20)
        self.assertEqual(meta['resolution'], 0.1)
        self.assertEqual(meta['origin_x'], 0.0)

    def test_origin(self):
        info = self._Info()
        info.origin = self._Pose()
        meta = map_render.map_meta(info)
        self.assertAlmostEqual(meta['origin_x'], 1.5)
        self.assertAlmostEqual(meta['origin_y'], -2.5)

    def test_missing_all(self):
        meta = map_render.map_meta(object())
        self.assertEqual(meta['width'], 0)
        self.assertGreater(meta['resolution'], 0)


if __name__ == '__main__':
    unittest.main()
