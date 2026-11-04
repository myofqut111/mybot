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
    """
    out = [int(v) for v in data]
    r = int(radius_cells)
    if r <= 0:
        return out
    span = 2 * r + 1
    for index, value in enumerate(data):
        if value != WALL:
            continue
        cx, cy = index % width, index // width
        for dy in range(-r, r + 1):
            y = cy + dy
            if y < 0 or y >= height:
                continue
            base = y * width
            remain = r - abs(dy)
            for dx in range(-remain, remain + 1):
                x = cx + dx
                if x < 0 or x >= width:
                    continue
                dist = max(abs(dx), abs(dy))
                cost = int(round(max_cost * (r - dist + 1) / float(span)))
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
    if abs(error) > 1.2:
        return 0.0, max(-max_w, min(max_w, 2.0 * error))
    vx = max_v * max(0.25, math.cos(error))
    return min(max_v, vx), max(-max_w, min(max_w, 2.0 * error))
