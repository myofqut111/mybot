# -*- coding: utf-8 -*-
"""离线跑一遍前沿检测，把结果画成 PNG——不启 ROS、不动车。

用法：
    python3 explore_offline.py <地图.yaml> [--pose X Y] [--radius 1.2] [--out out.png]

做什么：
    1. 读 map_server 格式的 pgm + yaml
    2. 转成 frontier.Grid（0 空闲 / 100 障碍 / -1 未知）
    3. 用 Explorer 的同一套参数找候选点
    4. 画图：灰=未知、白=空闲、黑=障碍、蓝点=候选（编号）、绿圈=已走过
"""
from __future__ import print_function

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mybot_explore import frontier as frontier_mod          # noqa: E402
from mybot_explore import pgm                              # noqa: E402
from mybot_explore import visited as visited_mod           # noqa: E402


def read_pgm(path):
    return pgm.read_pgm(path)


def read_yaml(path):
    return pgm.read_yaml(path)


def pgm_to_grid(width, height, pixels, meta):
    return pgm.pgm_to_grid(width, height, pixels, meta['resolution'],
                           meta['origin_x'], meta['origin_y'])


def pick_pose(grid, grid_pose):
    """没给位姿就挑一个靠中心、且空闲的格子。"""
    if grid_pose is not None:
        return grid_pose
    cx, cy = grid.width // 2, grid.height // 2
    for r in range(0, max(grid.width, grid.height)):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                gx, gy = cx + dx, cy + dy
                if grid.is_free(gx, gy):
                    return grid.cell_to_world(gx, gy)
    return (meta_origin_fallback(grid))


def meta_origin_fallback(grid):
    return grid.cell_to_world(0, 0)


def render(grid, candidates, out_path, robot_xy, visited=None):
    from PIL import Image, ImageDraw
    scale = 3
    img = Image.new('RGB', (grid.width * scale, grid.height * scale), (128, 128, 128))
    px = img.load()
    for gy in range(grid.height):
        for gx in range(grid.width):
            value = grid.at(gx, gy)
            if value == -1:
                color = (128, 128, 128)
            elif value == 0:
                color = (255, 255, 255)
            else:
                color = (0, 0, 0)
            for yy in range(scale):
                for xx in range(scale):
                    px[gx * scale + xx, (grid.height - 1 - gy) * scale + yy] = color

    draw = ImageDraw.Draw(img)

    if visited is not None:
        for gy in range(grid.height):
            for gx in range(grid.width):
                if visited.is_visited(gx, gy):
                    x0 = gx * scale
                    y0 = (grid.height - 1 - gy) * scale
                    draw.point([(x0 + 1, y0 + 1)], fill=(120, 190, 255))

    # 候选点
    for i, c in enumerate(candidates):
        cx = c['x']
        cy = c['y']
        gx, gy = grid.world_to_cell(cx, cy)
        x0 = gx * scale
        y0 = (grid.height - 1 - gy) * scale
        r = 4
        draw.ellipse([x0 - r, y0 - r, x0 + r, y0 + r], outline=(220, 40, 40), width=2)
        draw.text((x0 + 5, y0 - 6), str(i + 1), fill=(180, 0, 0))

    # 机器人
    if robot_xy:
        gx, gy = grid.world_to_cell(robot_xy[0], robot_xy[1])
        x0 = gx * scale
        y0 = (grid.height - 1 - gy) * scale
        draw.ellipse([x0 - 6, y0 - 6, x0 + 6, y0 + 6], outline=(0, 150, 0), width=3)
        draw.line([x0, y0 - 12, x0, y0 - 4], fill=(0, 150, 0), width=3)

    img.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml')
    ap.add_argument('--pose', nargs=2, type=float, default=None)
    ap.add_argument('--radius', type=float, default=1.2)
    ap.add_argument('--min-size', type=int, default=4)
    ap.add_argument('--max-candidates', type=int, default=20)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    meta = read_yaml(args.yaml)
    pgm = os.path.join(os.path.dirname(os.path.abspath(args.yaml)), meta['image'])
    width, height, pixels = read_pgm(pgm)
    grid = pgm_to_grid(width, height, pixels, meta)

    unknown = sum(1 for v in grid.data if v == -1)
    free = sum(1 for v in grid.data if v == 0)
    occ = sum(1 for v in grid.data if v == 100)
    total = float(len(grid.data))

    robot_xy = pick_pose(grid, tuple(args.pose) if args.pose else None)
    visited = visited_mod.VisitedLayer.from_grid(grid)
    visited.mark_circle(robot_xy[0], robot_xy[1], args.radius, grid)

    candidates = frontier_mod.find_frontiers(
        grid, robot_xy=robot_xy, min_size=args.min_size,
        unknown_radius_m=1.0, max_candidates=args.max_candidates,
        visited=visited)

    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(args.yaml)),
        os.path.splitext(os.path.basename(args.yaml))[0] + '_frontiers.png')
    render(grid, candidates, out, robot_xy, visited)

    print('地图 %dx%d  %.2f m/cell' % (grid.width, grid.height, grid.resolution))
    print('未知 %.1f%%  空闲 %.1f%%  障碍 %.1f%%' %
          (100 * unknown / total, 100 * free / total, 100 * occ / total))
    print('机器人位姿 (%.2f, %.2f)' % robot_xy)
    print('候选 %d 个' % len(candidates))
    for i, c in enumerate(candidates[:10]):
        print('  #%-2d (%.2f, %.2f)  距离 %.2f m  前沿格 %d  增益 %d  分数 %.3f' %
              (i + 1, c['x'], c['y'], c['distance'], c['cells'],
               c['unknown'], c['score']))
    print('图已保存: %s' % out)


if __name__ == '__main__':
    main()
