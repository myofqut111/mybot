# -*- coding: utf-8 -*-
"""打滑/低矮障碍时把假位移挡在位姿链外的"里程计锚点"。

背景
----
位姿链是三段复合的：

    map --(slam)--> odom --(EKF)--> base_link

轮子打滑时车没动，但编码器还在累加 → EKF 那段 `odom→base_link` 继续涨；
slam 那段 `map→odom` 反过来要把它抵消掉（日志里"map→odom 漂了 0.39 m"）。
一旦 slam 那段跟不上（或者我们主动暂停建图），复合位姿就跟着漂 ——
恢复建图后 scan matching 吃不下这个差，把同一面墙当成新地方拼上去，
这就是"两张图拼在一起"。

做法
----
把 `odom→base_link` 的发布权从 EKF 手里接管：

* 平时：`发布 = 原始 + offset`，offset 恒为 0（或上次重锚留下的值）
* 打滑确认：**冻结**（一直发同一个位姿，车确实没动）
* 恢复：**重锚** `offset = 冻结位姿 - 当前原始位姿`，把这一段假位移永久丢掉

这样 EKF 内部怎么涨都不影响对外位姿；slam 看到的 odom 始终是"老实"的。

本模块只做纯计算，不碰 ROS；节点在 `odom_shim_node.py`。
"""

from __future__ import annotations

import math

DEFAULTS = {
    'odom_shim_enabled': False,
    'odom_shim_topic': '/odometry/filtered',
    'odom_shim_parent': 'odom',
    'odom_shim_child': 'base_link',
    'odom_shim_timeout_s': 2.0,
}


def _wrap(angle):
    """把角度折到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_of(q):
    """四元数 (x, y, z, w) → 偏航角。"""
    x, y, z, w = q
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _quat_of(yaw):
    """偏航角 → 只有 z/w 的四元数。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def pose_of(msg):
    """从 Odometry / PoseStamped 之类取 (x, y, yaw)，取不到就返回 None。"""
    pose = getattr(msg, 'pose', None)
    if pose is None:
        return None
    if hasattr(pose, 'pose'):          # PoseStamped / Odometry
        pose = pose.pose
    position = getattr(pose, 'position', None)
    orientation = getattr(pose, 'orientation', None)
    if position is None or orientation is None:
        return None
    return (float(position.x), float(position.y),
            _yaw_of((orientation.x, orientation.y, orientation.z, orientation.w)))


class OdomAnchor:
    """把原始里程计位姿"整形"成对外位姿；支持冻结与重锚。"""

    def __init__(self, offset=(0.0, 0.0, 0.0)):
        self.offset = (float(offset[0]), float(offset[1]), float(offset[2]))
        self.frozen = False
        self.frozen_pose = None       # 冻结期间对外发布的那个位姿
        self.resumes = 0
        self.freezes = 0
        self.discarded_m = 0.0        # 累计丢掉的假位移（含重锚时的那一段）
        self._last_raw = None

    # ---- 位姿整形 -------------------------------------------------------
    def apply(self, raw):
        """原始位姿 → 对外位姿。冻结时忽略输入，直接返回冻结位姿。"""
        if raw is None:
            return None
        if self.frozen:
            return self.frozen_pose
        x, y, yaw = raw
        ox, oy, oyaw = self.offset
        return (x + ox, y + oy, _wrap(yaw + oyaw))

    # ---- 状态切换 -------------------------------------------------------
    def freeze(self, raw=None):
        """进入冻结。第一次冻结时记住当前对外位姿，之后一律发它。"""
        if raw is not None:
            self._last_raw = raw
        if not self.frozen:
            pose = self.apply(raw) if raw is not None else self.frozen_pose
            self.frozen = True
            self.frozen_pose = pose
            self.freezes += 1
        return self.frozen_pose

    def resume(self, raw=None):
        """退出冻结并重锚：让 发布 == 冻结位姿（此刻它等于"车真实位置"）。"""
        if raw is not None:
            self._last_raw = raw
        if not self.frozen:
            return self.apply(self._last_raw)
        if raw is None:
            raw = self._last_raw
        if raw is None or self.frozen_pose is None:
            self.frozen = False
            return None
        target = self.frozen_pose
        fx, fy, fyaw = target
        rx, ry, ryaw = raw
        self.discarded_m += math.hypot(rx - fx, ry - fy)
        self.offset = (fx - rx, fy - ry, _wrap(fyaw - ryaw))
        self.frozen = False
        self.resumes += 1
        self.frozen_pose = None
        return target

    def update(self, raw):
        """喂一帧原始位姿，返回对外位姿（同时更新内部最近原始值）。"""
        if raw is None:
            return None
        self._last_raw = raw
        return self.apply(raw)

    def snapshot(self):
        return {
            'frozen': bool(self.frozen),
            'offset': [round(v, 4) for v in self.offset],
            'freezes': self.freezes,
            'resumes': self.resumes,
            'discarded_m': round(self.discarded_m, 3),
            'frozen_pose': [round(v, 4) for v in self.frozen_pose] if self.frozen_pose else None,
        }
