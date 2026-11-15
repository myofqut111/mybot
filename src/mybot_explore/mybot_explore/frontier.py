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

import collections
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
        # 1e-9 是用来兜浮点的：0.05+0.35 会算出 0.39999999999999997，
        # 除 0.1 之后被 floor 到前一格，光束终点就会差一格。
        gx = int(math.floor((x - self.origin_x) / self.resolution + 1e-9))
        gy = int(math.floor((y - self.origin_y) / self.resolution + 1e-9))
        return gx, gy

    def cell_to_world(self, gx, gy):
        """格子中心的世界坐标。"""
        return (self.origin_x + (gx + 0.5) * self.resolution,
                self.origin_y + (gy + 0.5) * self.resolution)

    def contains(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def contains_world(self, x, y):
        """世界坐标是否落在地图范围内（擦图时用，省得先转格子再判）。"""
        gx, gy = self.world_to_cell(x, y)
        return self.contains(gx, gy)

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


def tight_cells(grid, radius_m, occupied_thresh=50):
    """标出"贴着障碍"的格子（以障碍为种子做多源 BFS 膨胀）。

    为什么要它：前沿点天生就贴着边界，很容易落进墙角的膨胀区里。
    nav2 的代价地图会直接拒绝那种目标（"目标在障碍里"），
    所以真正下发之前要把目标往外挪到"至少离墙这么远"的空闲格。
    """
    tight = bytearray(grid.width * grid.height)
    radius = max(0, int(round(radius_m / grid.resolution)))
    if radius <= 0:
        return tight

    queue = collections.deque()
    for idx, value in enumerate(grid.data):
        if value >= occupied_thresh:
            tight[idx] = 1
            queue.append((idx % grid.width, idx // grid.width))

    for _ in range(radius):
        if not queue:
            break
        for _ in range(len(queue)):
            cx, cy = queue.popleft()
            for dx, dy in NEIGHBORS4:
                nx, ny = cx + dx, cy + dy
                if not grid.contains(nx, ny):
                    continue
                idx = grid.index(nx, ny)
                if tight[idx]:
                    continue
                tight[idx] = 1
                queue.append((nx, ny))
    return tight


def safe_goal(grid, gx, gy, tight, search_m, occupied_thresh=50):
    """从 (gx, gy) 往外找最近的、不贴障碍的空闲格；找不到返回 None。"""
    limit = max(1, int(round(search_m / grid.resolution)))
    best = None
    best_dist = None
    for dy in range(-limit, limit + 1):
        for dx in range(-limit, limit + 1):
            dist = dx * dx + dy * dy
            if dist > limit * limit:
                continue
            nx, ny = gx + dx, gy + dy
            if not grid.contains(nx, ny):
                continue
            if not grid.is_free(nx, ny, occupied_thresh):
                continue
            if tight[grid.index(nx, ny)]:
                continue
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




def despeckle(grid, occupied_thresh=50, min_neighbors=2):
    """去掉孤立的障碍点（激光噪点），保留真正的墙与家具。

    为什么需要：激光偶尔会在空地中间留下一个两个黑点，代价地图里无所谓
    （会被膨胀抹平），但"离障碍多远"是按**每个障碍格**算的——这些噪点
    会把可通行区打成筛子，连通性 BFS 走两步就断，前沿全被判成"到不了"。

    规则：8 邻域里的障碍格少于 min_neighbors 个，就当噪点清掉。
    一格厚的直墙，中间每格左右各有邻居（>=2），不会被误删；墙角更多。
    """
    total = grid.width * grid.height
    data = list(grid.data)
    removed = 0
    for gy in range(grid.height):
        for gx in range(grid.width):
            idx = grid.index(gx, gy)
            if grid.data[idx] < occupied_thresh:
                continue
            count = 0
            for dx, dy in NEIGHBORS8:
                nx, ny = gx + dx, gy + dy
                if not grid.contains(nx, ny):
                    continue
                if grid.data[grid.index(nx, ny)] >= occupied_thresh:
                    count += 1
            if count < min_neighbors:
                data[idx] = 0
                removed += 1
    if not removed:
        return grid
    return Grid(grid.width, grid.height, grid.resolution,
                grid.origin_x, grid.origin_y, data)


def obstacle_distance(grid, occupied_thresh=50):
    """每个格子到最近障碍的距离（米）；障碍本身是 0。

    以障碍为种子做多源 BFS（四邻接，一步 = 一个格子）。
    未知格子不参与播种：它们是"没看过"，不是"挡路"。
    到不了的格子给一个很大的数，调用方用不到。
    """
    total = grid.width * grid.height
    far = float(grid.width + grid.height) * grid.resolution
    dist = [far] * total
    queue = collections.deque()
    for idx, value in enumerate(grid.data):
        if value >= occupied_thresh:
            dist[idx] = 0.0
            queue.append(idx)
    step = grid.resolution
    while queue:
        idx = queue.popleft()
        base = dist[idx] + step
        gx = idx % grid.width
        gy = idx // grid.width
        for dx, dy in NEIGHBORS4:
            nx, ny = gx + dx, gy + dy
            if not grid.contains(nx, ny):
                continue
            nidx = grid.index(nx, ny)
            if dist[nidx] > base:
                dist[nidx] = base
                queue.append(nidx)
    return dist


def traversable_mask(grid, clearance_m, occupied_thresh=50, distance=None):
    """"机器人身体能过去"的格子：空闲 + 离障碍 >= clearance_m。

    未知格子一律不可通行——没看过就是不敢走。
    """
    if distance is None:
        distance = obstacle_distance(grid, occupied_thresh)
    mask = bytearray(grid.width * grid.height)
    clearance = max(0.0, float(clearance_m))
    for idx, value in enumerate(grid.data):
        if value != UNKNOWN and 0 <= value < occupied_thresh and \
                distance[idx] >= clearance:
            mask[idx] = 1
    return mask


def reachable_mask(grid, traversable, start_xy):
    """从 start_xy 出发、只在可通行格子上 BFS，返回可到达的格子掩码。"""
    mask = bytearray(grid.width * grid.height)
    if start_xy is None:
        return mask
    sx, sy = grid.world_to_cell(float(start_xy[0]), float(start_xy[1]))
    if not grid.contains(sx, sy):
        return mask
    # 起点本身可能在墙边（刚停下时很常见），先在附近找一格能站的
    seed = None
    for radius in range(0, 8):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = sx + dx, sy + dy
                if not grid.contains(nx, ny):
                    continue
                if traversable[grid.index(nx, ny)]:
                    seed = (nx, ny)
                    break
            if seed:
                break
        if seed:
            break
    if seed is None:
        return mask
    queue = collections.deque([seed])
    mask[grid.index(seed[0], seed[1])] = 1
    while queue:
        cx, cy = queue.popleft()
        for dx, dy in NEIGHBORS4:
            nx, ny = cx + dx, cy + dy
            if not grid.contains(nx, ny):
                continue
            idx = grid.index(nx, ny)
            if mask[idx] or not traversable[idx]:
                continue
            mask[idx] = 1
            queue.append((nx, ny))
    return mask


def _nearest_reachable(grid, gx, gy, reach, limit_cells):
    """在 (gx, gy) 周围找最近的可到达格子；找不到返回 None。"""
    if reach[grid.index(gx, gy)]:
        return gx, gy
    best = None
    best_dist = None
    for dy in range(-limit_cells, limit_cells + 1):
        for dx in range(-limit_cells, limit_cells + 1):
            d2 = dx * dx + dy * dy
            if d2 > limit_cells * limit_cells:
                continue
            nx, ny = gx + dx, gy + dy
            if not grid.contains(nx, ny):
                continue
            if not reach[grid.index(nx, ny)]:
                continue
            if best_dist is None or d2 < best_dist:
                best, best_dist = (nx, ny), d2
    return best


def seal_gaps(grid, radius_m, occupied_thresh=50):
    """形态学闭运算：把"腿和腿之间"这种细缝算成障碍。

    为什么需要：2D 激光能从凳子腿、桌腿之间穿过去，于是地图上腿之间是
    **空闲**，前沿就长在那片"看得见但过不去"的地方；更糟的是椅子面在
    激光平面之上，图里根本不存在，车会照着"空"开过去。

    做法：先把障碍膨胀 radius_m（这样缝被填上），再腐蚀回来（障碍恢复
    原尺寸，但被填上的缝保留）。只返回新地图，不改原对象。
    """
    radius = int(round(float(radius_m) / grid.resolution))
    if radius <= 0:
        return grid
    total = grid.width * grid.height
    dil = bytearray(total)
    queue = collections.deque()
    for idx, value in enumerate(grid.data):
        if value >= occupied_thresh:
            dil[idx] = 1
            queue.append(idx)
    for _ in range(radius):
        if not queue:
            break
        for _ in range(len(queue)):
            idx = queue.popleft()
            gx = idx % grid.width
            gy = idx // grid.width
            for dx, dy in NEIGHBORS4:
                nx, ny = gx + dx, gy + dy
                if not grid.contains(nx, ny):
                    continue
                nidx = grid.index(nx, ny)
                if dil[nidx]:
                    continue
                dil[nidx] = 1
                queue.append(nidx)
    # 腐蚀：从"没被膨胀到"的格子出发做 BFS，离它 >= radius 的膨胀格才是闭运算结果
    depth = [0] * total
    queue = collections.deque()
    for idx in range(total):
        if not dil[idx]:
            depth[idx] = 0
            queue.append(idx)
        else:
            depth[idx] = -1
    while queue:
        idx = queue.popleft()
        gx = idx % grid.width
        gy = idx // grid.width
        nd = depth[idx] + 1
        for dx, dy in NEIGHBORS4:
            nx, ny = gx + dx, gy + dy
            if not grid.contains(nx, ny):
                continue
            nidx = grid.index(nx, ny)
            if depth[nidx] != -1:
                continue
            depth[nidx] = nd
            queue.append(nidx)
    data = list(grid.data)
    added = 0
    for idx in range(total):
        if not dil[idx]:
            continue
        if depth[idx] == -1 or depth[idx] < radius:
            continue
        if data[idx] >= occupied_thresh:
            continue
        data[idx] = 100
        added += 1
    if not added:
        return grid
    return Grid(grid.width, grid.height, grid.resolution,
                grid.origin_x, grid.origin_y, data)


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
                   min_separation_m=0.0, goal_clearance_m=0.0,
                   reach_clearance_m=0.0, reach_search_m=0.6):
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
        reach_clearance_m  机器人半径（含余量）。给了就只保留"车真能开过去"的点：
                           离障碍 < 该值的格子不可通行，从车位做连通性 BFS，
                           走不到的前沿直接丢弃（激光能穿过凳子腿不代表车能过）
        reach_search_m     候选点附近多远以内有可达格子就算数
    """
    cells = frontier_cells(grid, occupied_thresh, visited=visited)
    if not cells:
        return []
    groups = cluster(cells, min_size=min_size)
    radius_cells = max(1, int(round(unknown_radius_m / grid.resolution)))
    tight = None
    if goal_clearance_m and goal_clearance_m > 0:
        tight = tight_cells(grid, goal_clearance_m, occupied_thresh)

    # 可达性：只在"车能站、且从车那里能连过来"的格子上选目标
    reach = None
    reach_limit = 0
    if reach_clearance_m and reach_clearance_m > 0 and robot_xy is not None:
        # 先去掉孤立噪点再算距离：不然可通行区会被打成筛子，
        # 连通性 BFS 走两步就断，所有前沿都被判成"到不了"。
        clean = despeckle(grid, occupied_thresh)
        distance_map = obstacle_distance(clean, occupied_thresh)
        traversable = traversable_mask(grid, reach_clearance_m,
                                       occupied_thresh, distance_map)
        reach = reachable_mask(grid, traversable, robot_xy)
        reach_limit = max(1, int(round(float(reach_search_m) / grid.resolution)))

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
        if tight is not None:
            safe = safe_goal(grid, cx, cy, tight,
                             goal_clearance_m * 2.5, occupied_thresh)
            if safe is None:
                continue              # 这条前沿贴着墙，去了也停不进去
            cx, cy = safe
        if reach is not None:
            moved = _nearest_reachable(grid, cx, cy, reach, reach_limit)
            if moved is None:
                continue          # 走不到这片：激光穿得过去，车过不去
            cx, cy = moved
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
