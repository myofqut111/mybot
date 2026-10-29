#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 nav_msgs/OccupancyGrid 变成浏览器能显示的 PNG，以及世界坐标 <-> 像素换算。

为什么自己写 PNG 编码：机器人上不一定装了 PIL，而 cv2 虽然大概率有，
但把"渲染地图"绑死在图像库上就没法在本机跑单测了。PNG 的 L/RGB 编码
其实只有 20 行（zlib 压缩 + 每行一个 filter 字节）。
"""
import struct
import zlib

# 地图配色（值 -> 颜色）
COLOR_UNKNOWN = (0x2b, 0x33, 0x44)
COLOR_FREE = (0xf2, 0xf5, 0xf8)
COLOR_OCCUPIED = (0x1a, 0x1f, 0x28)
# 中间灰阶：50% 占据 = 这两色的中点
COLOR_MID = (0x8c, 0x93, 0xa3)


def write_png(width, height, pixels, color_type=2):
    """pixels 是逐行 RGB/RGBA 字节；返回完整 PNG 字节串。

    color_type=2 表示 RGB。这里只实现无隔行、filter=0 的最简形式，
    浏览器的解码器完全认。
    """
    if width <= 0 or height <= 0:
        raise ValueError('bad size %dx%d' % (width, height))
    channels = 4 if color_type == 6 else 3
    expected = width * height * channels
    if len(pixels) != expected:
        raise ValueError('pixels length %d != %d' % (len(pixels), expected))

    stride = width * channels
    raw = bytearray()
    for row in range(height):
        raw.append(0)                       # filter type: None
        raw += pixels[row * stride:(row + 1) * stride]

    def chunk(tag, payload):
        out = struct.pack('>I', len(payload)) + tag + payload
        return out + struct.pack('>I', zlib.crc32(tag + payload) & 0xFFFFFFFF)

    header = struct.pack('>IIBBBBB', width, height, 8, color_type, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', header)
            + chunk(b'IDAT', zlib.compress(bytes(raw), 6))
            + chunk(b'IEND', b''))


def occupancy_colors(value):
    """单个占据值 -> RGB。未知(-1) 深灰、自由(0) 近白、占据(100) 近黑。"""
    if value is None or value < 0:
        return COLOR_UNKNOWN
    if value == 0:
        return COLOR_FREE
    ratio = min(100.0, float(value)) / 100.0
    return (int(COLOR_MID[0] + (COLOR_OCCUPIED[0] - COLOR_MID[0]) * ratio),
            int(COLOR_MID[1] + (COLOR_OCCUPIED[1] - COLOR_MID[1]) * ratio),
            int(COLOR_MID[2] + (COLOR_OCCUPIED[2] - COLOR_MID[2]) * ratio))


def grid_to_png(data, width, height):
    """OccupancyGrid.data -> PNG。

    ROS 的 data 是**自下而上**逐行存的（第 0 行对应 origin.y），
    而 PNG 是自上而下，所以这里要竖向翻转，否则地图上下颠倒。
    """
    if width <= 0 or height <= 0:
        raise ValueError('bad size')
    if len(data) < width * height:
        raise ValueError('data %d < %d' % (len(data), width * height))
    cache = {}
    rows = []
    for row in range(height - 1, -1, -1):
        base = row * width
        line = bytearray()
        for col in range(width):
            value = data[base + col]
            rgb = cache.get(value)
            if rgb is None:
                rgb = occupancy_colors(value)
                cache[value] = rgb
            line += bytes(rgb)
        rows.append(bytes(line))
    return write_png(width, height, b''.join(rows), color_type=2)


def world_to_pixel(wx, wy, origin_x, origin_y, resolution, height):
    """世界坐标 -> 像素坐标（左上角原点，含竖向翻转）。"""
    if resolution <= 0:
        raise ValueError('resolution must be > 0')
    px = (wx - origin_x) / resolution
    py = (height - 1) - (wy - origin_y) / resolution
    return px, py


def pixel_to_world(px, py, origin_x, origin_y, resolution, height):
    """像素坐标 -> 世界坐标（取像素中心）。"""
    if resolution <= 0:
        raise ValueError('resolution must be > 0')
    wx = origin_x + (px + 0.5) * resolution
    wy = origin_y + ((height - 1) - py + 0.5) * resolution
    return wx, wy


def map_meta(grid_info):
    """把消息里的 map.info 转成前端需要的尺寸信息（缺字段给默认值）。"""
    resolution = float(getattr(grid_info, 'resolution', 0.05) or 0.05)
    width = int(getattr(grid_info, 'width', 0) or 0)
    height = int(getattr(grid_info, 'height', 0) or 0)
    origin = getattr(grid_info, 'origin', None)
    origin_x = float(getattr(origin, 'x', 0.0)) if origin is not None else 0.0
    origin_y = float(getattr(origin, 'y', 0.0)) if origin is not None else 0.0
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
    }
