# -*- coding: utf-8 -*-
"""造一张"只探了一半"的地图，给 mock 环境演示自主探索用。

mock_robot 发出来的 /map 整张都是已知的（不是空闲就是墙），前沿检测自然
一个点都找不到。这个脚本拿 mock_world 的同一张图，把右半边改成"未知"，
存成 map_server 的 pgm + yaml。探索节点用 map_yaml 直接读它，就能看到
一条真实的前沿线（x≈5 m 那道隔断的门口）。

用法：
    python3 make_partial_map.py /tmp/mock_partial         # 生成 .pgm + .yaml
    # 然后在虚拟机里：
    ros2 run fishbot_console mock_robot &
    ros2 run fishbot_explore explore_node --ros-args \
        -p map_yaml:=/tmp/mock_partial.yaml -p drive:=true -p autostart:=true
"""
from __future__ import print_function

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mybot_console import mock_world                       # noqa: E402


def build(width=200, height=200, resolution=0.05, split_x_m=5.0):
    """左半边保留 mock_world 的原始内容，右半边的**空闲**改成未知。

    墙要留着：不然前沿会直接贴着不存在的墙，造出来的图和 mock 的世界对不上
    （踩过：目标点落在 mock 的隔断墙里，被 "目标不可达" 拒掉）。
    """
    data, w, h, meta = mock_world.build_map(resolution=resolution,
                                            width=width, height=height)
    split = int(round(split_x_m / resolution))
    out = []
    for row in range(h):
        for col in range(w):
            value = data[row * w + col]
            if col >= split and value < 50:      # 只把空闲改成未知
                out.append(-1)
            else:
                out.append(value)
    return out, w, h, meta


def write_pgm(path, data, width, height):
    """0(墙) -> 0，100(障碍) -> 0，-1(未知) -> 205，其余 -> 254。"""
    pixels = bytearray()
    for value in data:
        if value < 0:
            pixels.append(205)
        elif value >= 50:
            pixels.append(0)
        else:
            pixels.append(254)
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (width, height))
        f.write(bytes(pixels))


def write_yaml(path, image_name, resolution, origin_x, origin_y):
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('image: %s\n' % image_name)
        f.write('mode: trinary\n')
        f.write('resolution: %.4f\n' % resolution)
        f.write('origin: [%.3f, %.3f, 0.0]\n' % (origin_x, origin_y))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')


def main(argv):
    base = argv[1] if len(argv) > 1 else '/tmp/mock_partial'
    data, width, height, meta = build()
    root = os.path.splitext(base)[0]
    pgm = root + '.pgm'
    yaml = root + '.yaml'
    write_pgm(pgm, data, width, height)
    write_yaml(yaml, os.path.basename(pgm), meta['resolution'],
               meta['origin_x'], meta['origin_y'])

    free = sum(1 for v in data if 0 <= v < 50)
    unknown = sum(1 for v in data if v < 0)
    wall = sum(1 for v in data if v >= 50)
    total = float(len(data))
    print('%s  %dx%d  %.2f m/cell' % (yaml, width, height, meta['resolution']))
    print('空闲 %.1f%%  未知 %.1f%%  墙 %.1f%%' %
          (100 * free / total, 100 * unknown / total, 100 * wall / total))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
