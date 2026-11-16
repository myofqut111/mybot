"""直接读 map_server 的 pgm + yaml，不经过 ROS。

为什么要自己读
--------------
nav2 的 map_server 用的是
    occ = (255 - value) / 255 ; occ > occupied_thresh -> 100 ; occ < free_thresh -> 0 ; 否则 -1
本仓库的 `map_v2.yaml` 写的是 `free_thresh: 0.25`，而灰 205 算出来 0.196，
比 0.25 小 → **在 nav2 眼里这 60% 的灰是"空闲"**（这也是为什么导航能直接
穿过没探过的地方）。前沿检测要的是"真没看过"，所以按 pgm 的 trinary 约定
自己还原：254=空闲，0=障碍，中间=未知。
"""
from __future__ import print_function

import os

from . import frontier as frontier_mod

FREE_VALUE = 254
OCCUPIED_VALUE = 0


def read_pgm(path):
    """读二进制 PGM（P5），返回 (width, height, bytearray)。支持注释和 16 位。"""
    with open(path, 'rb') as f:
        raw = f.read()
    pos = 0
    fields = []
    while len(fields) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            while pos < len(raw) and raw[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        fields.append(raw[start:pos])
    magic, w, h, maxval = fields
    if magic != b'P5':
        raise ValueError('只支持二进制 PGM (P5)，实际是 %r' % magic)
    width, height, maxval = int(w), int(h), int(maxval)
    pos += 1
    pixels = raw[pos:pos + width * height]
    if maxval != 255:
        pixels = bytearray(int(round(v * 255.0 / maxval)) for v in bytearray(pixels))
    return width, height, bytearray(pixels)


def read_yaml(path):
    """只解析需要的字段，不依赖 pyyaml。"""
    out = {}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            out[key.strip()] = value.strip()
    origin = out.get('origin', '0, 0, 0').strip('[]').split(',')
    return {
        'image': out.get('image', '').strip('"\''),
        'resolution': float(out.get('resolution', 0.05)),
        'origin_x': float(origin[0]),
        'origin_y': float(origin[1]),
    }


def pgm_to_grid(width, height, pixels, resolution, origin_x, origin_y,
                free_value=FREE_VALUE, occupied_value=OCCUPIED_VALUE):
    """trinary 约定：接近白=空闲，接近黑=障碍，其余=未知。"""
    data = []
    for value in pixels:
        if value >= free_value - 5:
            data.append(0)
        elif value <= occupied_value + 10:
            data.append(100)
        else:
            data.append(-1)
    return frontier_mod.Grid(width, height, resolution, origin_x, origin_y, data)


def write_map(base_path, grid, occupied_thresh=50, origin=None):
    """把 Grid 写回 pgm + yaml（base_path 不带扩展名）。返回 (pgm, yaml)。"""
    if not base_path.endswith('.pgm'):
        base_path = base_path + '.pgm'
    lines = ['P5', '%d %d' % (grid.width, grid.height), '255']
    header = ('\n'.join(lines) + '\n').encode('ascii')
    body = bytearray(grid.width * grid.height)
    for i, value in enumerate(grid.data):
        if value < 0:
            body[i] = 205
        elif value >= occupied_thresh:
            body[i] = OCCUPIED_VALUE
        else:
            body[i] = FREE_VALUE
    with open(base_path, 'wb') as handle:
        handle.write(header)
        handle.write(bytes(body))

    ox = grid.origin_x if origin is None else origin[0]
    oy = grid.origin_y if origin is None else origin[1]
    yaml_path = base_path[:-4] + '.yaml'
    text = ('image: %s\nmode: trinary\nresolution: %s\norigin: [%s, %s, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
            % (os.path.basename(base_path), repr(float(grid.resolution)),
               repr(float(ox)), repr(float(oy))))
    with open(yaml_path, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(text)
    return base_path, yaml_path


def load_map(yaml_path, free_value=FREE_VALUE):
    """给一个 .yaml 路径，返回 frontier.Grid（未知已按 pgm 原始约定还原）。"""
    meta = read_yaml(yaml_path)
    image = meta['image'] or os.path.splitext(os.path.basename(yaml_path))[0] + '.pgm'
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    width, height, pixels = read_pgm(image)
    return pgm_to_grid(width, height, pixels, meta['resolution'],
                       meta['origin_x'], meta['origin_y'], free_value=free_value)
