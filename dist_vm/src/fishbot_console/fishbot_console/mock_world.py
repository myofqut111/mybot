# -*- coding: utf-8 -*-
"""纯几何部分：假地图 + 射线追踪 + 单轮车积分。

为什么单独一个文件：这三个东西是整个"不用开小车也能验证"的地基，一旦算错，
表现出来是"点云穿墙""车往墙里走"，而那种现象很难从 UI 上判断是渲染错还是
数据错。放在这里不 import rclpy，本机就能跑单测。
"""
import math

FREE = 0
WALL = 100
UNKNOWN = -1


# ---------------------------------------------------------------- 地图
def build_map(resolution=0.05, width=200, height=200):
    """10 m x 10 m 的房间：外墙 + 一道隔断 + 两个方块障碍。

    返回 (data, width, height, meta)，data 是 row-major 的 int 列表，
    原点在 (0, 0)，第 y 行对应 y ∈ [y*res, (y+1)*res)。
    """
    data = [FREE] * (width * height)

    def fill(x0, y0, x1, y1, value=WALL):
        for row in range(max(0, y0), min(height, y1)):
            base = row * width
            for col in range(max(0, x0), min(width, x1)):
                data[base + col] = value

    edge = max(1, int(round(0.10 / resolution)))          # 10 cm 厚的墙
    fill(0, 0, width, edge)                                # 下
    fill(0, height - edge, width, height)                  # 上
    fill(0, 0, edge, height)                               # 左
    fill(width - edge, 0, width, height)                   # 右

    # 中间一道带缺口的隔断（留 1.2 m 门洞，车能过去）
    gap0 = int(round(4.0 / resolution))
    gap1 = int(round(5.2 / resolution))
    mid = width // 2
    fill(mid, edge, mid + edge, gap0)
    fill(mid, gap1, mid + edge, height - edge)

    # 两个障碍块
    fill(int(round(2.6 / resolution)), int(round(2.4 / resolution)),
         int(round(3.4 / resolution)), int(round(3.2 / resolution)))
    fill(int(round(7.0 / resolution)), int(round(6.0 / resolution)),
         int(round(7.8 / resolution)), int(round(7.6 / resolution)))

    meta = {
        'resolution': resolution,
        'origin_x': 0.0,
        'origin_y': 0.0,
        'width': width,
        'height': height,
    }
    return data, width, height, meta


def cell_at(data, width, height, resolution, origin_x, origin_y, x, y):
    """世界坐标 -> 格子值；出界按墙算（否则射线会从地图边上漏出去）。"""
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor((y - origin_y) / resolution))
    if col < 0 or row < 0 or col >= width or row >= height:
        return WALL
    return data[row * width + col]


# ---------------------------------------------------------------- 射线
def raycast(data, width, height, resolution, origin_x, origin_y,
            x, y, yaw, angles, max_range=8.0, step_ratio=0.5):
    """从 (x, y, yaw) 打一圈射线，返回每个角度的距离（没打到就是 inf）。

    angles 是相对 yaw 的偏角序列。步长取 0.5 个格子：再细只是多花时间，
    再粗会从薄墙上跨过去（外墙正好 2 个格子厚，0.5 格步长不会漏）。
    """
    step = resolution * step_ratio
    if step <= 0:
        raise ValueError('step must be positive')
    steps = int(math.ceil(max_range / step))
    out = []
    cos_a = [math.cos(a) for a in angles]
    sin_a = [math.sin(a) for a in angles]
    c0, s0 = math.cos(yaw), math.sin(yaw)
    for k in range(len(angles)):
        # 直接合成绝对方向，少一次三角函数
        c = c0 * cos_a[k] - s0 * sin_a[k]
        s = s0 * cos_a[k] + c0 * sin_a[k]
        hit = float('inf')
        for i in range(1, steps + 1):
            d = i * step
            if cell_at(data, width, height, resolution, origin_x, origin_y,
                       x + c * d, y + s * d) == WALL:
                hit = d
                break
        out.append(hit)
    return out


def scan_msg_fields(angles, ranges, range_min=0.05, range_max=8.0):
    """把射线结果整理成 LaserScan 需要的字段（inf 换成 range_max+1）。"""
    cleaned = []
    for value in ranges:
        if value is None or not math.isfinite(value):
            cleaned.append(range_max + 1.0)     # ROS 约定：超出量程用 >range_max
        else:
            cleaned.append(max(range_min, min(range_max, value)))
    if not angles:
        return {'angle_min': 0.0, 'angle_increment': 0.0,
                'range_min': range_min, 'range_max': range_max,
                'ranges': cleaned}
    increment = angles[1] - angles[0] if len(angles) > 1 else 0.0
    return {'angle_min': angles[0], 'angle_increment': increment,
            'range_min': range_min, 'range_max': range_max,
            'ranges': cleaned}


# ---------------------------------------------------------------- 代价地图
def inflate(data, width, height, radius_cells, max_cost=90):
    """把墙周围的格子标成代价（模拟 nav2 的膨胀层）。

    墙本身保持 100（致命），越远代价越低，至少 1；空地不动。
    用“在每个墙格周围盖盒子”而不是对每个空格做窗口最小值，因为墙格很少（百分之几）。

    掩膜是**圆形**（欧氏距离），不是菱形也不是方形：
    - 菱形（曼哈顿）会让对角方向只留 0.71 倍余量，车角会切进墙角；
    - 方形又会把四个角多封死，车在拐角处无谓地绕远。
    圆形才和“车半径 = 0.18 m”这个语义一致。
    """
    out = [int(v) for v in data]
    r = int(radius_cells)
    if r <= 0:
        return out
    r2 = r * r
    span = 2.0 * r + 1.0
    for index, value in enumerate(data):
        if value != WALL:
            continue
        cx, cy = index % width, index // width
        for dy in range(-r, r + 1):
            y = cy + dy
            if y < 0 or y >= height:
                continue
            base = y * width
            dy2 = dy * dy
            limit = int(math.sqrt(max(0, r2 - dy2)))     # 这一行的半宽
            for dx in range(-limit, limit + 1):
                x = cx + dx
                if x < 0 or x >= width:
                    continue
                dist = math.sqrt(dx * dx + dy2)
                cost = int(round(max_cost * (r - dist + 1.0) / span))
                slot = base + x
                if cost > out[slot]:
                    out[slot] = cost
    return out


def crop(data, width, height, x0, y0, out_w, out_h, fill=WALL):
    """从地图里切一块（越界用 fill 补），返回 (data, w, h)。"""
    out = []
    for row in range(out_h):
        sy = y0 + row
        for col in range(out_w):
            sx = x0 + col
            if sx < 0 or sy < 0 or sx >= width or sy >= height:
                out.append(fill)
            else:
                out.append(data[sy * width + sx])
    return out


# ---------------------------------------------------------------- 运动
def blocked_grid(data, width, height, radius_cells):
    """不可通行栅格：墙 + 离墙 radius 以内的格子。

    假车必须和真车一样有“半径”。只判中心点的话，车头能压进墙里半个身位，
    看起来就是“穿墙”——这正是之前的问题。
    """
    inflated = inflate(data, width, height, radius_cells)
    return [1 if value > 0 else 0 for value in inflated]


def _free(blocked, width, height, resolution, origin_x, origin_y,
          x, y, radius_m=0.0):
    """(x, y) 是否可以站车：中心点 + 圆周采样，任一被占就是不行。"""
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor((y - origin_y) / resolution))
    if col < 0 or row < 0 or col >= width or row >= height:
        return False
    if blocked[row * width + col]:
        return False
    if radius_m <= 0.0:
        return True
    for k in range(8):                       # 8 个方向采一圈，够用且便宜
        angle = k * math.pi / 4.0
        px = x + radius_m * math.cos(angle)
        py = y + radius_m * math.sin(angle)
        c = int(math.floor((px - origin_x) / resolution))
        r = int(math.floor((py - origin_y) / resolution))
        if c < 0 or r < 0 or c >= width or r >= height:
            return False
        if blocked[r * width + c]:
            return False
    return True


def step_guarded(pose, vx, wz, dt, blocked, width, height,
                 resolution, origin_x, origin_y, radius_m=0.0):
    """带碰撞的积分一步，返回 (new_pose, hit)。

    locked 必须是**已经按车半径膨胀过**的栅格（locked_grid(...) 的输出），
    所以 radius_m 默认 0：再膨胀一次等于把车半径算了两遍，车会在离墙 0.4 m 的
    地方就停下，看着像“车卡在原地不动”。

    撞上就退化：先试“只沿 x 走”、再试“只沿 y 走”（这就是贴着墙滑），
    最后试原地转；都不行就原地不动。
    不做规划会“直着撞墙”，但至少要**真的停下**——否则假车教给人一个错误的
    预期：以为车会自己避开。
    """
    dt = max(0.0, float(dt))

    def free(x, y):
        return _free(blocked, width, height, resolution, origin_x, origin_y,
                     x, y, radius_m)

    candidate = step(pose, vx, wz, dt)
    if free(candidate[0], candidate[1]):
        return candidate, False

    x, y, yaw = pose
    dx = vx * math.cos(yaw) * dt
    dy = vx * math.sin(yaw) * dt
    for slide in ((x + dx, y, yaw), (x, y + dy, yaw),
                  (x, y, wrap_angle(yaw + wz * dt))):
        if free(slide[0], slide[1]):
            return slide, True
    return pose, True


def _line_of_sight(blocked, width, height, a, b):
    """Bresenham：a、b 两个格子之间是否全程可通行。"""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if x0 < 0 or y0 < 0 or x0 >= width or y0 >= height:
            return False
        if blocked[y0 * width + x0]:
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def astar(blocked, width, height, start, goal):
    """8 邻域 A*，返回格子列表（含首尾），无路返回 None。

    假车要走“走廊—门洞—房间”这种路线，直着走必然穿墙。给 mock 一个真正的
    最短路，演示时才有可信度。
    """
    import heapq

    if not (0 <= start[0] < width and 0 <= start[1] < height):
        return None
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        return None
    if blocked[goal[1] * width + goal[0]]:
        return None

    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2.0) * min(dx, dy)

    open_heap = [(0.0, start)]
    came = {}
    cost = {start: 0.0}
    seen = set()
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in seen:
            continue
        seen.add(current)
        if current == goal:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            path.reverse()
            return path
        cx, cy = current
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                if blocked[ny * width + nx]:
                    continue
                # 斜着穿两个墙的夹角是不允许的：真车过不去，规划也不该过
                if dx and dy:
                    if (blocked[cy * width + nx]
                            or blocked[ny * width + cx]):
                        continue
                step_cost = math.sqrt(2.0) if (dx and dy) else 1.0
                new_cost = cost[current] + step_cost
                node = (nx, ny)
                if new_cost < cost.get(node, float('inf')):
                    cost[node] = new_cost
                    came[node] = current
                    heapq.heappush(open_heap,
                                   (new_cost + heuristic(node, goal), node))
    return None


def simplify_path(path, blocked, width, height):
    """贪婪拉直：能直达就跳过中间点，省得车走成锯齿。"""
    if not path:
        return []
    out = [path[0]]
    index = 0
    while index < len(path) - 1:
        far = len(path) - 1
        while far > index + 1 and not _line_of_sight(
                blocked, width, height, path[index], path[far]):
            far -= 1
        out.append(path[far])
        index = far
    return out


def cells_to_points(cells, resolution, origin_x, origin_y):
    """格子中心 -> 世界坐标。"""
    return [(origin_x + (c + 0.5) * resolution,
             origin_y + (r + 0.5) * resolution) for c, r in cells]


def world_to_cell(x, y, resolution, origin_x, origin_y):
    return (int(math.floor((x - origin_x) / resolution)),
            int(math.floor((y - origin_y) / resolution)))


def wrap_angle(angle):
    """归一到 (-pi, pi]，避免 yaw 越滚越大导致画出来的车头方向跳。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def step(pose, vx, wz, dt):
    """单轮车积分一步。pose=(x, y, yaw)，返回新 pose。"""
    x, y, yaw = pose
    dt = max(0.0, float(dt))
    new_yaw = wrap_angle(yaw + wz * dt)
    # 用中间航向积分，原地转 + 前进同时发生时不会画出一条折线
    mid = wrap_angle(yaw + 0.5 * wz * dt)
    new_x = x + vx * math.cos(mid) * dt
    new_y = y + vx * math.sin(mid) * dt
    return new_x, new_y, new_yaw


def yaw_of_quaternion(q):
    """四元数取平面朝向。假导航目标用得上。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_of_yaw(yaw):
    """平面朝向转四元数（z, w 只有这两个分量）。"""
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}


def approach(pose, target, max_v=0.45, max_w=1.2, stop_dist=0.15):
    """朝目标走一步的控制量，返回 (vx, wz)。

    先用原地转把车头对准（差得多时），再直着走——和真人开小车的习惯一致，
    也让"点地图发目标"看起来像回事。
    """
    dx = target[0] - pose[0]
    dy = target[1] - pose[1]
    dist = math.hypot(dx, dy)
    if dist <= stop_dist:
        return 0.0, 0.0
    desired = math.atan2(dy, dx)
    error = wrap_angle(desired - pose[2])
    if abs(error) > 0.7:
        # 差得多就先原地转：一边前进一边大幅转向会画弧、切到墙角。
        return 0.0, max(-max_w, min(max_w, 2.0 * error))
    vx = max_v * max(0.25, math.cos(error))
    return min(max_v, vx), max(-max_w, min(max_w, 2.0 * error))
