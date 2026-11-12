# -*- coding: utf-8 -*-
"""用激光把"未知"擦成"空闲"——让探索能一层层往里推。

为什么需要它：
    静态地图 + 前沿检测有个死结——地图不会更新，所以未知边界永远原地不动，
    车走到边上就"到头了"，进不去。
    但车身上有激光：光束扫过的地方，其实已经知道那里是空的。
    把光束经过的未知格改成空闲、打到墙的那格标成障碍，
    未知边界就会跟着车往后缩，下一轮前沿自然出现在更里面。

这层只在内存里改（不动 pgm 文件），重启即还原。
将来换成 slam_toolbox 时，这一层直接删掉，别的地方一行不用改。
"""
from __future__ import print_function

import math

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def _beam_hit(scan, index):
    """第 index 束的测距值；无效（NaN/inf/低于 range_min）返回 None。"""
    value = scan['ranges'][index]
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    if value < scan['range_min']:
        return None
    return min(value, scan['range_max'])


def carve_scan(grid, scan, pose, max_range_m=0.0, robot_radius_m=0.25,
               occupied_thresh=50, step_m=0.0):
    """把一帧激光"擦"进地图，返回 (空闲格数, 障碍格数)。

    scan 是一个字典：{'ranges': [...], 'angle_min': a0, 'angle_increment': da,
                      'range_min': r0, 'range_max': r1}
    pose 是 map 系下的 (x, y, yaw)。
    max_range_m > 0 时只信这么远以内的光束（激光远处噪声大）。
    """
    hit_range = float(max_range_m) if max_range_m and max_range_m > 0 \
        else float(scan['range_max'])
    step = float(step_m) if step_m and step_m > 0 else grid.resolution * 0.5
    reserved = max(1, int(round(robot_radius_m / grid.resolution)))

    x0, y0, yaw = pose
    free_count = 0
    occ_count = 0

    # 车底盘底下一定是空的：先把车身那么大一圈擦掉
    cx, cy = grid.world_to_cell(x0, y0)
    for dy in range(-reserved, reserved + 1):
        for dx in range(-reserved, reserved + 1):
            if dx * dx + dy * dy > reserved * reserved:
                continue
            nx, ny = cx + dx, cy + dy
            if not grid.contains(nx, ny):
                continue
            idx = grid.index(nx, ny)
            if grid.data[idx] < 0:
                grid.data[idx] = FREE
                free_count += 1

    ranges = scan['ranges']
    for i in range(len(ranges)):
        distance = _beam_hit(scan, i)
        if distance is None:
            continue
        angle = yaw + scan['angle_min'] + i * scan['angle_increment']
        limit = min(distance, hit_range)
        # 沿光束走一遍：路过的未知格 -> 空闲
        r = grid.resolution
        steps = max(1, int(round(limit / step)))
        for k in range(1, steps + 1):
            dist = min(k * step, limit)
            if dist >= limit - step * 0.5:
                break                  # 终点那格留给下面标障碍
            wx = x0 + math.cos(angle) * dist
            wy = y0 + math.sin(angle) * dist
            if not grid.contains_world(wx, wy):
                break
            gx, gy = grid.world_to_cell(wx, wy)
            idx = grid.index(gx, gy)
            if grid.data[idx] < 0:
                grid.data[idx] = FREE
                free_count += 1
            elif grid.data[idx] >= occupied_thresh:
                break                      # 前面是墙，后面的不用擦了
        # 打到东西了：终点那格标成障碍
        # （不要求它本来是未知——光束撞上的那个面确实在那儿；
        #   一帧噪声最多留一个障碍格，下一帧擦过去就没了）
        if distance < scan['range_max'] - 1e-6:
            wx = x0 + math.cos(angle) * distance
            wy = y0 + math.sin(angle) * distance
            if grid.contains_world(wx, wy):
                gx, gy = grid.world_to_cell(wx, wy)
                idx = grid.index(gx, gy)
                if grid.data[idx] < occupied_thresh:
                    grid.data[idx] = OCCUPIED
                    occ_count += 1
    return free_count, occ_count
