"""前沿检测：把一张占据栅格地图变成「值得去的地方」列表。

这一步**不控制车、不发指令、不依赖 ROS**，纯函数，方便单测和调参。
只有 node.py 碰 ROS。

术语：
    未知 unknown   data < 0        —— 没看过的地方
    空闲 free      0 <=data< occ    —— 走过、确认能过
    障碍 occupied  data >= occ     —— 撞过或看着过不去

    前沿 frontier = 空闲格子中，与未知格子相邻的那些
    （"我在这看过，可再往外没看过" 的那条边）
"""
from __future__ import print_function

import math

UNKNOWN = -1


class Grid(object):
    """占据栅格的轻量包装，只做坐标换算和取值。"""

    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution) or 0.05
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.data = list(data)

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        origin = info.origin
        return cls(info.width, info.height, info.resolution,
                   origin.position.x, origin.position.y, msg.data)

    def index(self, gx, gy):
        return gy * self.width + gx

    def at(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return UNKNOWN
        return self.data[self.index(gx, gy)]

    def world_to_cell(self, x, y):
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def cell_to_world(self, gx, gy):
        """格子中心的世界坐标。"""
        return (self.origin_x + (gx + 0.5) * self.resolution,
                self.origin_y + (gy + 0.5) * self.resolution)

    def contains(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx, gy, occupied_thresh=50):
        value = self.at(gx, gy)
        return value != UNKNOWN and 0 <= value < occupied_thresh

    def is_unknown(self, gx, gy):
        """地图之外的格子**不算**未知——否则地图四边永远全是前沿，噪声太大。"""
        return self.contains(gx, gy) and self.data[self.index(gx, gy)] == UNKNOWN


NEIGHBORS4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
NEIGHBORS8 = ((1, 0), (-1, 0), (0, 1), (0, -1),
              (1, 1), (1, -1), (-1, 1), (-1, -1))


def frontier_cells(grid, occupied_thresh=50, visited=None):
    """返回所有"空闲且四邻里至少一个未知"的格子 [(gx, gy), ...]。

    给了 ``visited``（visited.VisitedLayer）就把已经走过的格子减掉——
    静态地图不会自己更新，靠这一步消耗掉"去过但地图仍是灰"的前沿。
    """
    out = []
    for gy in range(grid.height):
        for gx in range(grid.width):
            if not grid.is_free(gx, gy, occupied_thresh):
                continue
            if visited is not None and visited.is_visited(gx, gy):
                continue
            for dx, dy in NEIGHBORS4:
                if grid.is_unknown(gx + dx, gy + dy):
                    out.append((gx, gy))
                    break
    return out


def cluster(cells, min_size=1):
    """把相邻的前沿格子聚成块，用 8 邻接的 BFS。

    返回 [[(gx, gy), ...], ...]，按块内格子数从多到少。
    """
    remaining = set(cells)
    clusters = []
    while remaining:
        seed = remaining.pop()
        group = [seed]
        queue = [seed]
        while queue:
            gx, gy = queue.pop()
            for dx, dy in NEIGHBORS8:
                nxt = (gx + dx, gy + dy)
                if nxt in remaining:
                    remaining.remove(nxt)
                    group.append(nxt)
                    queue.append(nxt)
        if len(group) >= min_size:
            clusters.append(group)
    clusters.sort(key=len, reverse=True)
    return clusters


def nearest_free(grid, gx, gy, radius=6, occupied_thresh=50):
    """质心可能落在墙上或未知里，挪到最近的空闲格子；找不到返回 None。"""
    if grid.is_free(gx, gy, occupied_thresh):
        return gx, gy
    best = None
    best_dist = None
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = gx + dx, gy + dy
            if not grid.is_free(nx, ny, occupied_thresh):
                continue
            dist = dx * dx + dy * dy
            if best_dist is None or dist < best_dist:
                best, best_dist = (nx, ny), dist
    return best


def unknown_around(grid, gx, gy, radius_cells):
    """质心周围一圈里有多少未知格子——代表"去了能新看到多少"。"""
    count = 0
    r = int(radius_cells)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            if grid.is_unknown(gx + dx, gy + dy):
                count += 1
    return count


def _is_blacklisted(x, y, blacklist, radius_m):
    for bx, by in blacklist:
        if math.hypot(x - bx, y - by) <= radius_m:
            return True
    return False


def find_frontiers(grid, robot_xy=None, min_size=4, occupied_thresh=50,
                   unknown_radius_m=1.0, max_candidates=20,
                   w_distance=0.6, w_gain=0.4, visited=None,
                   blacklist=None, blacklist_radius_m=0.6,
                   min_goal_distance_m=0.0, max_goal_distance_m=None,
                   min_separation_m=0.0):
    """主入口：返回按分数从高到低排好的候选点。

    每个候选是 dict：
        {'x','y','cells','unknown','distance','score','gx','gy'}
    ``robot_xy`` 给了就算距离，没给就只按信息增益排。

    可选过滤：
        visited            已走过图层（见 visited.VisitedLayer）
        blacklist          [(x, y), ...] 失败过的点，附近的候选直接丢
        blacklist_radius_m 拉黑半径
        min_goal_distance_m 太近的点不去（防止原地打转）
        max_goal_distance_m 太远的点不去
    """
    cells = frontier_cells(grid, occupied_thresh, visited=visited)
    if not cells:
        return []
    groups = cluster(cells, min_size=min_size)
    radius_cells = max(1, int(round(unknown_radius_m / grid.resolution)))

    candidates = []
    for group in groups:
        sum_x = sum(c[0] for c in group)
        sum_y = sum(c[1] for c in group)
        cx = int(round(sum_x / float(len(group))))
        cy = int(round(sum_y / float(len(group))))
        free = nearest_free(grid, cx, cy, occupied_thresh=occupied_thresh)
        if free is None:
            continue
        cx, cy = free
        gain = unknown_around(grid, cx, cy, radius_cells)
        wx, wy = grid.cell_to_world(cx, cy)
        if robot_xy is None:
            distance = 0.0
        else:
            distance = math.hypot(wx - robot_xy[0], wy - robot_xy[1])

        # 过滤：太近、太远、被拉黑
        if min_goal_distance_m and distance < min_goal_distance_m:
            continue
        if max_goal_distance_m is not None and distance > max_goal_distance_m:
            continue
        if blacklist and _is_blacklisted(wx, wy, blacklist, blacklist_radius_m):
            continue

        candidates.append({'x': wx, 'y': wy, 'cells': len(group),
                           'unknown': gain, 'distance': distance,
                           'gx': cx, 'gy': cy})

    if not candidates:
        return []

    max_gain = max(c['unknown'] for c in candidates) or 1
    max_dist = max(c['distance'] for c in candidates) or 1.0
    for c in candidates:
        gain_score = c['unknown'] / float(max_gain)
        if robot_xy is None or max_dist <= 0:
            dist_score = 1.0
        else:
            dist_score = 1.0 - (c['distance'] / float(max_dist))
        c['score'] = w_distance * dist_score + w_gain * gain_score

    candidates.sort(key=lambda c: c['score'], reverse=True)
    if min_separation_m and min_separation_m > 0:
        candidates = _thin_out(candidates, min_separation_m)
    return candidates[:max_candidates]


def _thin_out(candidates, min_separation_m):
    """非极大抑制：分数高的先留，离已留的点太近的一律丢。

    不然一条走廊上的碎前沿会占满候选表（前 20 个全是同一个口子）。
    """
    kept = []
    for c in candidates:
        too_close = False
        for k in kept:
            if math.hypot(c['x'] - k['x'], c['y'] - k['y']) < min_separation_m:
                too_close = True
                break
        if not too_close:
            kept.append(c)
    return kept
