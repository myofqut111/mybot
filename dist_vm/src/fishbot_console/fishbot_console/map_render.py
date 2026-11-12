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


# ---------------------------------------------------------------- 代价地图叠加层
# 配色照着 RViz 的 costmap 来：致命障碍纯红、膨胀层橙、未知透明。
# 用 RGBA 让浏览器自己去和底图混合，服务端不做预乘，省一半计算。
LAYER_LETHAL = (0xef, 0x2f, 0x2f, 0xC0)
LAYER_INFLATED = (0xf2, 0x92, 0x2d, 0x66)
LAYER_TRANSPARENT = (0x00, 0x00, 0x00, 0x00)


def layer_rgba(value):
    """代价地图单值 -> RGBA。0 自由不要画，-1 未知也不要画。"""
    if value is None or value <= 0:
        return LAYER_TRANSPARENT
    if value >= 99:
        return LAYER_LETHAL
    # 1~98 是膨胀代价：越接近致命越不透明
    alpha = int(0x30 + (LAYER_INFLATED[3] - 0x30) * (value / 98.0))
    return (LAYER_INFLATED[0], LAYER_INFLATED[1], LAYER_INFLATED[2], alpha)


def costmap_rows(data, width, height):
    """OccupancyGrid.data -> RGBA 行（自上而下，已竖向翻转）。"""
    cache = {}
    rows = []
    for row in range(height - 1, -1, -1):
        base = row * width
        line = bytearray()
        for col in range(width):
            value = data[base + col]
            rgba = cache.get(value)
            if rgba is None:
                rgba = layer_rgba(value)
                cache[value] = rgba
            line += bytes(rgba)
        rows.append(bytes(line))
    return rows


def composite_layers(base_w, base_h, base_meta, layers):
    """把若干张开在地图坐标系上的代价地图叠成一张 RGBA（尺寸同底图）。

    layers: [(data, meta), ...]，按顺序覆盖；越靠后越上面。
    只用整数最近邻搬家：两边分辨率通常都是 0.05，不做插值反而更准。

    性能：代价地图绝大多数格子是 0（自由），所以先按行判"这一行有没有代价"，
    整行为空的直接跳过——实测 384x384 的图能省掉 90% 以上的循环。
    """
    canvas = bytearray(base_w * base_h * 4)     # 全透明
    base_res = float(base_meta.get('resolution') or 0.0)
    if base_res <= 0:
        raise ValueError('bad base resolution')
    base_ox = float(base_meta.get('origin_x') or 0.0)
    base_oy = float(base_meta.get('origin_y') or 0.0)

    for data, meta in layers:
        width = int(meta.get('width') or 0)
        height = int(meta.get('height') or 0)
        res = float(meta.get('resolution') or base_res)
        if width <= 0 or height <= 0 or res <= 0:
            continue
        if len(data) < width * height:
            continue
        ox = float(meta.get('origin_x') or 0.0)
        oy = float(meta.get('origin_y') or 0.0)
        cache = {}
        for row in range(height):
            base_index = row * width
            line_data = data[base_index:base_index + width]
            # array.array / list 都有 count()，整行没代价就不用逐格算
            if line_data.count(0) == width:
                continue
            bottom_row = row                 # data 的 row 本身就是自下而上的格号
            wy = oy + (bottom_row + 0.5) * res
            canvas_row = base_h - 1 - int((wy - base_oy) / base_res)
            if canvas_row < 0 or canvas_row >= base_h:
                continue
            target = canvas_row * base_w * 4
            for col in range(width):
                value = line_data[col]
                rgba = cache.get(value)
                if rgba is None:
                    rgba = layer_rgba(value)
                    cache[value] = rgba
                if not rgba[3]:
                    continue
                wx = ox + (col + 0.5) * res
                zcol = int((wx - base_ox) / base_res)
                if zcol < 0 or zcol >= base_w:
                    continue
                dst = target + zcol * 4
                canvas[dst:dst + 4] = rgba
    return write_png(base_w, base_h, bytes(canvas), color_type=6)


