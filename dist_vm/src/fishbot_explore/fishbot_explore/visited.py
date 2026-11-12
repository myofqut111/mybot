"""覆盖层：记录"车走过哪儿"，弥补静态地图永远不会更新这件事。

为什么需要它
------------
用 ``map_server`` 加载的静态地图是**只读**的：车开进未知区，地图文件里的
灰色格子不会变白。如果只按地图找前沿，同一个点会被反复选中，车会来回横跳。

所以自己记一本账：车每走一步，就把周围一圈标记成"已走过"。找前沿时把
已走过的格子减掉，前沿自然就"消耗"掉了。

设计原则
--------
* 纯数组操作，不依赖 ROS、不依赖 numpy，方便单测
* 坐标一律用**格子索引**（和 frontier.Grid 一致），世界坐标换算交给 Grid
* 标记是**幂等**的：同一片重复标记不会出错，只统计新标记的数量
"""
from __future__ import print_function

import math


class VisitedLayer(object):
    """与 Grid 同尺寸的布尔图层。"""

    def __init__(self, width, height, resolution=0.05, origin_x=0.0, origin_y=0.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution) or 0.05
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.bits = bytearray(self.width * self.height)

    # ---------- 构造 ----------

    @classmethod
    def from_grid(cls, grid):
        """按 Grid 的尺寸建一张空白图层。"""
        return cls(grid.width, grid.height, grid.resolution,
                   grid.origin_x, grid.origin_y)

    # ---------- 基本操作 ----------

    def index(self, gx, gy):
        return gy * self.width + gx

    def contains(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_visited(self, gx, gy):
        if not self.contains(gx, gy):
            return False
        return bool(self.bits[self.index(gx, gy)])

    def mark(self, gx, gy):
        """标记一个格子，返回它是否是新标记的。"""
        if not self.contains(gx, gy):
            return False
        i = self.index(gx, gy)
        if self.bits[i]:
            return False
        self.bits[i] = 1
        return True

    def mark_circle(self, x, y, radius_m, grid=None):
        """把世界坐标 (x, y) 周围 radius_m 米内的格子标记为已走过。

        ``grid`` 只用来做世界→格子的换算；不传就按本图层自己的
        resolution/origin 换算（两者一致，传不传都一样）。

        返回新标记的格子数。
        """
        h = grid if grid is not None else self
        res = h.resolution
        cx = int(math.floor((x - h.origin_x) / res))
        cy = int(math.floor((y - h.origin_y) / res))
        r = max(0, int(round(float(radius_m) / res)))
        newly = 0
        r2 = r * r
        for dy in range(-r, r + 1):
            gy = cy + dy
            if gy < 0 or gy >= self.height:
                continue
            row = gy * self.width
            dy2 = dy * dy
            for dx in range(-r, r + 1):
                if dx * dx + dy2 > r2:
                    continue
                gx = cx + dx
                if gx < 0 or gx >= self.width:
                    continue
                if not self.bits[row + gx]:
                    self.bits[row + gx] = 1
                    newly += 1
        return newly

    def mark_path(self, points, radius_m, grid=None):
        """沿一条路径标记（点与点之间按 resolution 插值，避免跳格漏标）。"""
        if not points:
            return 0
        step = (grid.resolution if grid is not None else self.resolution) * 0.5
        newly = 0
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            dist = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(dist / max(step, 1e-6))))
            for k in range(n + 1):
                t = k / float(n)
                newly += self.mark_circle(x0 + (x1 - x0) * t,
                                          y0 + (y1 - y0) * t,
                                          radius_m, grid)
        return newly

    # ---------- 统计 ----------

    def count(self):
        return sum(self.bits)

    def coverage(self, free_mask):
        """已知空闲区里，已走过的比例。

        ``free_mask`` 是一个与图层等长的序列，True 表示该格是空闲。
        返回 0~1；空闲格为 0 时返回 0。
        """
        free = 0
        seen = 0
        for i, is_free in enumerate(free_mask):
            if is_free:
                free += 1
                if self.bits[i]:
                    seen += 1
        if free == 0:
            return 0.0
        return seen / float(free)

    def reset(self):
        self.bits = bytearray(self.width * self.height)


def free_mask_of(grid, occupied_thresh=50):
    """把 Grid 转成"是否空闲"的布尔列表，给 coverage() 用。"""
    out = []
    for value in grid.data:
        out.append(value != -1 and 0 <= value < occupied_thresh)
    return out
