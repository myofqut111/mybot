# -*- coding: utf-8 -*-
"""打滑 / 低矮障碍判定（纯逻辑，不依赖 ROS）。

判据来自《自主探索_打滑与低矮障碍_方案》第 3 节：

  判据 A：``map→odom`` 的平移在长 —— SLAM 认为车没动，odom 认为车在走，
          两者的差就是"odom 累积了多少假位移"。
  判据 B：扫描没变、odom 在涨 —— 只看近场（默认 3 m）命中束，
          变化束占比低于阈值就算"扫描认为车没动"。

``slip_odom_min_m`` 是"一拍（默认 0.5 s）里 odom 至少走这么多才算在动"，
所以它同时决定了速度下限：0.05 m / 0.5 s ≈ 0.1 m/s。把它写太大（比如 0.1）
会让 0.15 m/s 的车永远判不出来——真机踩过。

A 与 B 必须同时成立，并排除三种正常情况：原地转、空旷区、本来就没让走。

**例外：推台阶 / 顶矮墙。** 这时车在世界里真的没动、扫描也真的没变，slam 没有
任何依据去修正 ``map→odom``，判据 A 恒为 0 —— 只按 A+B 永远判不出来（真机踩过：
车顶住台阶一路打滑还不肯停）。所以另开一条**只靠 B** 的判据：跨 3 秒比两帧扫描
（变化 <10%）且 odom 涨了 ≥0.30 m，就判"被顶住"（``action='blocked'``）。

状态机：

    IDLE ──(疑)──> SUSPECT ──(持续 1.5 s)──> CONFIRMED ──(由外部处理)──> HOLD
                       │                                                   │
                       └──(不再满足，退回 IDLE)                             ├──(真移动)──> IDLE(resume)
                                                                           └──(超时)────> IDLE(give_up)
"""

import math

DEFAULTS = {
    'slip_enabled': True,
    'slip_check_period_s': 0.5,
    'slip_window_s': 3.0,
    'slip_odom_min_m': 0.05,
    'slip_map_odom_min_m': 0.08,
    'slip_scan_change_ratio': 0.15,
    'slip_scan_diff_m': 0.03,
    'slip_scan_near_m': 3.0,
    'slip_max_omega': 0.30,
    'slip_min_valid_ratio': 0.40,
    'slip_min_hits': 20,
    'slip_min_speed': 0.05,
    'slip_confirm_s': 1.5,
    # ---- 推台阶 / 顶墙：只有 B 单独成立（map→odom 不会漂，因为世界里车真没动）----
    'blocked_enabled': True,
    'block_window_s': 3.0,
    'block_odom_min_m': 0.30,
    'block_scan_change_ratio': 0.10,
    'block_scan_near_m': 1.0,
    'block_scan_diff_m': 0.03,
    'block_min_hits': 8,
    'block_min_speed': 0.05,
    'block_max_omega': 0.20,
    'block_confirm_s': 1.0,
    'block_cooldown_s': 5.0,
    'slip_hold_timeout_s': 10.0,
}

IDLE = 'idle'
SUSPECT = 'suspect'
CONFIRMED = 'confirmed'
HOLD = 'hold'


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def scan_compare(prev, cur, near_m=3.0, diff_m=0.03, range_min=0.05, range_max=None):
    """比较两帧扫描，只看近场命中束。

    返回 ``(changed_ratio, valid_ratio, hits)``：

    * ``changed_ratio``：命中束里读数变化超过 ``diff_m`` 的占比；
    * ``valid_ratio``：整帧里"两帧都命中近场"的束占比（太少说明空旷，不可判）；
    * ``hits``：参与统计的束数。
    """
    if not prev or not cur:
        return 0.0, 0.0, 0
    ranges_a = prev.get('ranges') or []
    ranges_b = cur.get('ranges') or []
    total = min(len(ranges_a), len(ranges_b))
    if total <= 0:
        return 0.0, 0.0, 0
    limit = range_max or cur.get('range_max') or 12.0
    changed = 0
    hits = 0
    for index in range(total):
        a = ranges_a[index]
        b = ranges_b[index]
        if not (_finite(a) and _finite(b)):
            continue
        a = float(a)
        b = float(b)
        if a < range_min or b < range_min:
            continue
        if a >= limit or b >= limit:
            continue
        if min(a, b) > near_m:
            continue
        hits += 1
        if abs(a - b) > diff_m:
            changed += 1
    changed_ratio = (float(changed) / hits) if hits else 0.0
    valid_ratio = float(hits) / total
    return changed_ratio, valid_ratio, hits


class SlipDetector(object):
    """按 tick 喂数据，返回状态与动作。"""

    def __init__(self, params=None):
        self.params = dict(DEFAULTS)
        if params:
            self.params.update(params)
        self.state = IDLE
        self.reason = ''
        self.ratio = 0.0
        self.valid_ratio = 0.0
        self.hits = 0
        self.near_m = 0.0
        self.odom_moved = 0.0
        self.map_odom_moved = 0.0
        self.drift_confirmed = False
        self.hold_for = 0.0
        self.slip_at = None
        self.suspect_at = None
        self.hold_at = None
        self._prev_scan = None
        self._prev_odom = None
        self._prev_map_odom = None
        self._window = []
        self._hist = []
        self._block_suspect_at = None
        self._block_until = 0.0          # [(t, x, y)] 只存 map→odom
        self._last_tick = None
        # 推台阶/顶墙：跨 block_window_s 的两帧对比
        self._hist = []
        self._block_suspect_at = None
        self._block_until = 0.0
        self.block_ratio = 0.0
        self.block_odom_moved = 0.0
        self.block_hits = 0

    # ---------- 对外 ----------

    def reset(self):
        self.state = IDLE
        self.reason = ''
        self.slip_at = None
        self.suspect_at = None
        self.hold_at = None
        self.hold_for = 0.0
        self.odom_moved = 0.0
        self.map_odom_moved = 0.0
        self._prev_odom = None
        self._prev_map_odom = None
        self._window = []

    def update(self, now, odom_pose=None, map_odom=None, scan=None,
               linear=0.0, angular=0.0):
        """喂一拍数据，返回 ``{'state', 'action', 'reason', ...}``。

        ``action`` 取值：``None``（什么都不做）、``'suspect'``、
        ``'confirmed'``（调用方应取消目标并暂停建图）、``'resume'``、
        ``'give_up'``。
        """
        params = self.params
        if not params.get('slip_enabled', True):
            return self.snapshot(action=None, reason='disabled')

        if self._last_tick is not None and now - self._last_tick < params['slip_check_period_s']:
            return self.snapshot(action=None)
        self._last_tick = now

        # 1) 记录 odom 走了多少
        self.odom_moved = 0.0
        if odom_pose is not None and self._prev_odom is not None:
            self.odom_moved = math.hypot(odom_pose[0] - self._prev_odom[0],
                                         odom_pose[1] - self._prev_odom[1])
        if odom_pose is not None:
            self._prev_odom = (float(odom_pose[0]), float(odom_pose[1]))

        # 2) 记录 map→odom 的滑动窗口
        if map_odom is not None:
            self._window.append((float(now), float(map_odom[0]), float(map_odom[1])))
        horizon = now - params['slip_window_s']
        while len(self._window) > 1 and self._window[0][0] < horizon:
            self._window.pop(0)
        self.map_odom_moved = 0.0
        if len(self._window) >= 2:
            first = self._window[0]
            last = self._window[-1]
            self.map_odom_moved = math.hypot(last[1] - first[1], last[2] - first[2])

        # 3) 比两帧扫描
        self.ratio = 0.0
        self.valid_ratio = 0.0
        self.hits = 0
        if scan is not None and self._prev_scan is not None:
            # 近场优先（最近的点最有信息量）；命中太少就逐步放宽，
            # 否则在开阔场地里 3 m 内根本没有几束，永远判不出来。
            near = params['slip_scan_near_m']
            self.ratio, self.valid_ratio, self.hits = scan_compare(
                self._prev_scan, scan, near_m=near,
                diff_m=params['slip_scan_diff_m'],
                range_max=params.get('slip_scan_range_max'))
            tries = 0
            while self.hits < params['slip_min_hits'] and tries < 3:
                near *= 2.0
                tries += 1
                self.ratio, self.valid_ratio, self.hits = scan_compare(
                    self._prev_scan, scan, near_m=near,
                    diff_m=params['slip_scan_diff_m'],
                    range_max=params.get('slip_scan_range_max'))
            self.near_m = near
        if scan is not None:
            self._prev_scan = scan

        # 4) HOLD 期间只看"有没有真动起来"
        if self.state == HOLD:
            self.hold_for = now - (self.hold_at or now)
            moved = (self.ratio >= params['slip_scan_change_ratio']
                     or self.odom_moved >= params['slip_odom_min_m'] * 1.5
                     and self.ratio >= params['slip_scan_change_ratio'] * 0.5)
            if moved:
                self.state = IDLE
                self.suspect_at = None
                self.reason = 'moved_again'
                return self.snapshot(action='resume')
            if self.hold_for >= params['slip_hold_timeout_s']:
                self.state = IDLE
                self.suspect_at = None
                self.reason = 'hold_timeout'
                return self.snapshot(action='give_up')
            return self.snapshot(action=None)

        # 4.5) 推台阶 / 顶墙：只靠 B 的独立判据（跨窗口比两帧扫描）
        blocked_action = self._check_blocked(now, scan, odom_pose, linear, angular)
        if blocked_action == 'suspect':
            return self.snapshot(action='suspect')
        if blocked_action == 'blocked':
            return self.snapshot(action='blocked')

        # 5) 排除三种正常情况
        blocked = self._excluded(linear, angular)
        if blocked:
            if self.state == SUSPECT:
                self.state = IDLE
                self.suspect_at = None
            self.reason = blocked
            return self.snapshot(action=None)

        # 6) 判据：以 B 为主（扫描没变 + odom 在涨），A 只记录不参与判定。
        #    slam 正常时会把打滑立刻补偿回来，map→odom 漂移很小；若还要求 A，
        #    就会漏掉真打滑（2026-11-19 实测三段 0.8 m 的假位移全漏）。
        cond_a = self.map_odom_moved >= params['slip_map_odom_min_m']
        cond_b = (self.ratio < params['slip_scan_change_ratio']
                  and self.odom_moved >= params['slip_odom_min_m'])
        self.drift_confirmed = bool(cond_a)
        if not cond_b:
            if self.state == SUSPECT:
                self.state = IDLE
                self.suspect_at = None
            self.reason = 'not_slipping'
            return self.snapshot(action=None)

        if self.state == IDLE:
            self.state = SUSPECT
            self.suspect_at = now
            self.reason = 'suspect'
            return self.snapshot(action='suspect')

        if self.state == SUSPECT:
            if now - (self.suspect_at or now) >= params['slip_confirm_s']:
                self.state = HOLD
                self.slip_at = now
                self.hold_at = now
                self.hold_for = 0.0
                self.reason = 'confirmed'
                return self.snapshot(action='confirmed')
            return self.snapshot(action=None)

        return self.snapshot(action=None)

    def _check_blocked(self, now, scan, odom_pose, linear, angular):
        """推台阶 / 顶矮墙：只靠"扫描没变但 odom 在涨"。

        返回 ``'suspect'`` / ``'blocked'`` / ``None``。
        """
        params = self.params
        if not params.get('blocked_enabled', True):
            return None
        if now < self._block_until:
            return None
        if scan is None or odom_pose is None:
            return None

        window = float(params['block_window_s'])
        self._hist.append((float(now), scan, (float(odom_pose[0]), float(odom_pose[1]))))
        keep_from = now - window * 2.0
        while len(self._hist) > 2 and self._hist[0][0] < keep_from:
            self._hist.pop(0)

        target = now - window
        best = None
        for item in self._hist:
            if best is None or abs(item[0] - target) < abs(best[0] - target):
                best = item
        if best is None or abs(best[0] - target) > window * 0.5:
            return None

        ratio, _valid, hits = scan_compare(
            best[1], scan, near_m=params['block_scan_near_m'],
            diff_m=params['block_scan_diff_m'],
            range_max=params.get('slip_scan_range_max'))
        moved = math.hypot(odom_pose[0] - best[2][0], odom_pose[1] - best[2][1])
        self.block_ratio = ratio
        self.block_odom_moved = moved
        self.block_hits = hits

        # 只在"确实在往前开"的时候判：原地转、没让走、近处没东西都要放过
        if (abs(angular) > params['block_max_omega']
                or abs(linear) < params['block_min_speed']
                or hits < params['block_min_hits']):
            self._block_suspect_at = None
            return None

        if (ratio < params['block_scan_change_ratio']
                and moved >= params['block_odom_min_m']):
            if self._block_suspect_at is None:
                self._block_suspect_at = now
                return 'suspect'
            if now - self._block_suspect_at >= params['block_confirm_s']:
                self._block_suspect_at = None
                self._block_until = now + float(params['block_cooldown_s'])
                return 'blocked'
            return None

        self._block_suspect_at = None
        return None

    def _excluded(self, linear, angular):
        params = self.params
        if abs(angular) > params['slip_max_omega']:
            return 'turning'
        if self.hits < params['slip_min_hits']:
            return 'too_few_hits'
        if abs(linear) < params['slip_min_speed']:
            return 'not_moving'
        return ''

    def snapshot(self, action=None, reason=None):
        if reason is not None:
            self.reason = reason
        return {
            'state': self.state,
            'action': action,
            'reason': self.reason,
            'ratio': round(self.ratio, 4),
            'valid_ratio': round(self.valid_ratio, 4),
            'hits': self.hits,
            'near_m': round(self.near_m, 2),
            'odom_moved': round(self.odom_moved, 4),
            'map_odom_moved': round(self.map_odom_moved, 4),
            'hold_for': round(self.hold_for, 2),
            'block_ratio': round(self.block_ratio, 4),
            'block_odom_moved': round(self.block_odom_moved, 4),
            'block_hits': self.block_hits,
        }
