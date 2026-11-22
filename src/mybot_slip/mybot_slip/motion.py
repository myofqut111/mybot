# -*- coding: utf-8 -*-
"""动作达成的纯逻辑：给一串位姿，算出「退够了没有 / 转够了没有」。

抽出来单独放，是因为这部分最容易写错、也最好测：不需要 ROS、
不需要真车，喂一串 (x, y, yaw) 就能验证。
"""
import math


def _norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class TravelMonitor(object):
    """跟踪一次动作的进度。

    kind='back'   累计走过的平面距离（不管航向，后退时航向几乎不变）
    kind='escape' 累计转过的角度（带符号，按目标方向累计）

    目标达成即 `done()` 为 True；调用方负责超时。
    """

    def __init__(self, kind, target, tolerance=0.015):
        self.kind = kind
        self.target = abs(float(target or 0.0))
        self.tolerance = float(tolerance)
        self.started = None
        self.origin = None
        self.last = None
        self.yaw_acc = 0.0
        self.travel = 0.0

    def update(self, x, y, yaw):
        if self.origin is None:
            self.origin = (x, y, yaw)
            self.last = (x, y, yaw)
            return 0.0
        lx, ly, lyaw = self.last
        dyaw = _norm_angle(yaw - lyaw)
        self.yaw_acc += dyaw
        self.travel += math.hypot(x - lx, y - ly)
        self.last = (x, y, yaw)
        return self.progress()

    def progress(self):
        if self.kind == 'escape':
            return abs(self.yaw_acc)
        return self.travel

    def done(self):
        return self.progress() >= max(self.target - self.tolerance, 0.0)

    def reached_at_least(self, ratio):
        return self.progress() >= self.target * ratio
