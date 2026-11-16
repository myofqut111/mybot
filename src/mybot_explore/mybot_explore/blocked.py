# -*- coding: utf-8 -*-
"""挡住点：短墙 + 禁试区（纯逻辑，不依赖 ROS）。

设计依据《自主探索_打滑与低矮障碍_方案》第 5 节：

* 车只知道"我在 P 点朝 H 方向，车头撞到了东西"；
* 所以只画一条**垂直于车头、长度等于车宽**的短墙（有物证）；
* 其余保持未知（没看过就是没看过）；
* 另存一份**禁试区**（只影响选点，不写进地图），防止反复往里试。
"""

import json
import math
import os
import tempfile

VERSION = 1
DEFAULT_PATH = '~/.ros/mybot_blocked.json'


def wall_segment(x, y, yaw, offset_m=0.22, length_m=0.50):
    """车头前沿、垂直于朝向的一段墙。

    ``(x, y, yaw)`` 是车身中心位姿；``offset_m`` 是车身中心到车头前沿的距离；
    ``length_m`` 是墙长（默认 0.5 m＝车宽 0.3 + 两侧各 0.1 余量）。
    """
    center_x = float(x) + float(offset_m) * math.cos(float(yaw))
    center_y = float(y) + float(offset_m) * math.sin(float(yaw))
    half = float(length_m) / 2.0
    # 朝向的垂直方向
    nx = -math.sin(float(yaw))
    ny = math.cos(float(yaw))
    return {
        'x1': center_x - nx * half,
        'y1': center_y - ny * half,
        'x2': center_x + nx * half,
        'y2': center_y + ny * half,
        'cx': center_x,
        'cy': center_y,
        'yaw': float(yaw),
        'length_m': float(length_m),
    }


def _cells_along(grid, x1, y1, x2, y2, step_m):
    """沿线段按 ``step_m`` 采样，返回格坐标列表（去重、只保留界内）。"""
    length = math.hypot(x2 - x1, y2 - y1)
    steps = max(1, int(math.ceil(length / max(step_m, 1e-6))))
    cells = []
    seen = set()
    for index in range(steps + 1):
        ratio = float(index) / steps
        wx = x1 + (x2 - x1) * ratio
        wy = y1 + (y2 - y1) * ratio
        cell = grid.world_to_cell(wx, wy)
        if cell is None:
            continue
        if not grid.contains(cell[0], cell[1]):
            continue
        if cell in seen:
            continue
        seen.add(cell)
        cells.append(cell)
    return cells


def stamp_wall(grid, segment, occupied_value=100, thickness_cells=0):
    """把短墙写进栅格，返回写入的格数。

    ``thickness_cells > 0`` 时向两侧各扩若干格（默认不扩——只标有物证的那条线）。
    """
    step_m = min(grid.resolution, 0.05)
    cells = _cells_along(grid, segment['x1'], segment['y1'],
                         segment['x2'], segment['y2'], step_m)
    written = 0
    for gx, gy in cells:
        for dx in range(-thickness_cells, thickness_cells + 1):
            for dy in range(-thickness_cells, thickness_cells + 1):
                nx, ny = gx + dx, gy + dy
                if not grid.contains(nx, ny):
                    continue
                index = ny * grid.width + nx
                if grid.data[index] != occupied_value:
                    grid.data[index] = occupied_value
                    written += 1
    return written


class BlockedStore(object):
    """跨轮次的挡住点记录（JSON，原子写）。"""

    def __init__(self, path=DEFAULT_PATH):
        self.path = os.path.expanduser(path)
        self.spots = []

    # ---------- 存取 ----------

    def load(self):
        try:
            with open(self.path, 'r') as handle:
                payload = json.load(handle)
        except (IOError, OSError, ValueError):
            self.spots = []
            return self.spots
        spots = payload.get('spots') if isinstance(payload, dict) else None
        self.spots = [s for s in (spots or []) if isinstance(s, dict)]
        return self.spots

    def save(self):
        directory = os.path.dirname(self.path) or '.'
        try:
            os.makedirs(directory)
        except OSError:
            pass
        payload = {'version': VERSION, 'spots': self.spots}
        handle = tempfile.NamedTemporaryFile('w', delete=False,
                                             dir=directory, suffix='.tmp')
        try:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
            handle.write('\n')
            handle.close()
            os.replace(handle.name, self.path)
        except Exception:
            try:
                handle.close()
            except Exception:
                pass
            raise
        return self.path

    # ---------- 增改 ----------

    def near(self, x, y, radius_m):
        """返回 ``radius_m`` 内已有的记录（最近的一条）。"""
        best = None
        best_distance = None
        for spot in self.spots:
            distance = math.hypot(float(spot.get('x', 0.0)) - x,
                                  float(spot.get('y', 0.0)) - y)
            if distance <= radius_m and (best_distance is None or distance < best_distance):
                best = spot
                best_distance = distance
        return best

    def add(self, x, y, heading, now=None, radius_m=1.0, slip_seconds=0.0):
        """记一个挡住点；同一片已有时只累加 ``hits`` 并更新朝向。"""
        spot = self.near(x, y, radius_m)
        if spot is None:
            spot = {
                'x': float(x), 'y': float(y), 'heading': float(heading),
                'hits': 0, 'slip_seconds': 0.0,
                'first_seen': now, 'last_seen': now,
            }
            self.spots.append(spot)
        spot['hits'] = int(spot.get('hits', 0)) + 1
        spot['heading'] = float(heading)
        spot['slip_seconds'] = round(float(spot.get('slip_seconds', 0.0)) + slip_seconds, 2)
        spot['last_seen'] = now
        return spot

    def in_keepout(self, x, y, radius_m):
        return self.near(x, y, radius_m) is not None

    def to_keepouts(self, radius_m=1.0):
        """给选点用的圆列表 ``[{'x','y','radius'}]``。"""
        return [{'x': float(s.get('x', 0.0)), 'y': float(s.get('y', 0.0)),
                 'radius': radius_m} for s in self.spots]
