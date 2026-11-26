"""非重叠时间窗口：把 /odom /wheel_joints /imu /scan /cmd_vel 折成一段一段的「运动增量」。

设计依据（方案 v1.1 第 4 章）：
- 窗口默认 0.30 s、**不重叠**，相邻判定样本互相独立；
- 高频量（轮子位置、陀螺角速度）在窗口内积分/差分；
- 激光只有 10~20 Hz，窗口边界和激光帧不重合，必须**按时间比例插值**，不能直接拿「最近两帧」；
- 指令要求窗口内有效覆盖 >= 70%，否则本窗不参与判定；
- 起步/制动/指令切换后的第一个窗口标记 warmup，不判定。

这里只做纯数据聚合，不做任何判定；判定在 criteria.py。
"""
import math


DEFAULTS = {
    'window_s': 0.40,          # 方案 §12：Tw 0.25~0.40 s，取上限（8 Hz 的 odom 一窗 3~4 条）
    'max_window_s': 1.50,      # 里程计太稀疏时窗口最多拉长到这里
    'min_odom_samples': 2,     # 窗口里至少这么多条 odom 才算够用（8 Hz 时 0.5 s 有 4 条）
    'warmup_s': 0.30,          # 指令切换后多久内的窗口不算
    'min_cmd_coverage': 0.70,  # 指令覆盖低于此值判窗无效
    'cmd_active_eps': 0.02,    # 生效指令小于这个值算"没让它动"，不判
    'min_samples': 2,          # odom/wheel/imu 每个窗口至少几个样本
    'max_gap_s': 0.60,         # 相邻样本间隔超过此值认为断流，本窗作废（WiFi 抖一下不该废窗）
    'imu_yaw_cov': 0.02,       # 陀螺角速度方差，用作降级判断
}


class _Series(object):
    """按时间排序的 (t, value) 序列，value 是标量或元组。"""

    def __init__(self):
        self.items = []

    def add(self, t, value):
        # 迟到的样本直接丢弃，避免把序列搞乱
        if self.items and t < self.items[-1][0]:
            return
        self.items.append((t, value))

    def between(self, t0, t1):
        return [(t, v) for t, v in self.items if t0 - 1e-9 <= t <= t1 + 1e-9]

    def last_before(self, t):
        prev = None
        for ts, v in self.items:
            if ts <= t:
                prev = (ts, v)
            else:
                break
        return prev

    def first_after(self, t):
        for ts, v in self.items:
            if ts >= t:
                return (ts, v)
        return None

    def clip(self, t0):
        self.items = [(t, v) for t, v in self.items if t >= t0 - 1e-9]


def _trapz(pts):
    """对 [(t, v), ...] 做梯形积分，v 必须是数值。"""
    total = 0.0
    for (t_a, v_a), (t_b, v_b) in zip(pts, pts[1:]):
        total += 0.5 * (v_a + v_b) * (t_b - t_a)
    return total


def _interp(series, t, fallback='nearest'):
    """线性插值取 series 在时刻 t 的值（支持标量或等长元组）。"""
    if not series:
        return None
    before = after = None
    for ts, v in series:
        if ts <= t:
            before = (ts, v)
        else:
            after = (ts, v)
            break
    if before and after:
        span = after[0] - before[0]
        if span <= 1e-9:
            return before[1]
        w = (t - before[0]) / span
        if isinstance(before[1], tuple):
            return tuple(a + (b - a) * w for a, b in zip(before[1], after[1]))
        return before[1] + (after[1] - before[1]) * w
    if before and fallback in ('nearest', 'previous'):
        return before[1]
    if after and fallback in ('nearest', 'next'):
        return after[1]
    return None


class WindowAggregator(object):
    """喂数据、吐窗口。

    用法::

        agg = WindowAggregator()
        agg.add_odom(t, vx, vyaw)
        agg.add_wheel(t, left_m, right_m)
        agg.add_imu(t, gyro_z)
        agg.add_scan(t, scan_dict)
        agg.add_cmd(t, linear, angular)
        for win in agg.poll(now):
            ...
    """

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.odom = _Series()
        self.wheel = _Series()
        self.imu = _Series()
        self.scan = _Series()
        self.cmd = _Series()
        self.t0 = None
        self._warm_until = None
        self._last_cmd = (0.0, 0.0)
        # 累计量：用于统计「连续多少个窗无效」
        self.stats = {'windows': 0, 'skipped': 0}

    # ---------- 输入 ----------

    def _touch(self, t):
        """第一帧数据到达时定下窗口起点，窗口从此严格按 Tw 切。"""
        if self.t0 is None:
            self.t0 = t

    def add_odom(self, t, vx, vyaw):
        self._touch(t)
        self.odom.add(t, (float(vx), float(vyaw)))

    def add_wheel(self, t, left_m, right_m):
        self._touch(t)
        self.wheel.add(t, (float(left_m), float(right_m)))

    def add_imu(self, t, gyro_z):
        self._touch(t)
        self.imu.add(t, float(gyro_z))

    def add_scan(self, t, scan):
        self._touch(t)
        # scan: {'ranges': [...], 'angle_min': ., 'angle_increment': ., 'range_max': .}
        self.scan.add(t, scan)

    def add_cmd(self, t, linear, angular):
        self._touch(t)
        linear, angular = float(linear), float(angular)
        # 指令从「停」变成「动」或反向，视为一次切换 -> 之后 warmup_s 不判
        was_idle = abs(self._last_cmd[0]) < 1e-3 and abs(self._last_cmd[1]) < 1e-3
        now_idle = abs(linear) < 1e-3 and abs(angular) < 1e-3
        sign_flip = (self._last_cmd[0] * linear < 0) or (self._last_cmd[1] * angular < 0)
        if (was_idle and not now_idle) or sign_flip:
            self._warm_until = t + self.p['warmup_s']
        self._last_cmd = (linear, angular)
        self.cmd.add(t, (linear, angular))

    # ---------- 输出 ----------

    def poll(self, now):
        """把已经攒满的窗口吐出来（可能一次吐多个）。

        里程计只有 8~9 Hz 时，0.3 s 里只落得下 2 条样本（还常常只剩 1 条），
        固定按 0.3 s 切就会一路「odom 样本不足」。所以窗口是**能长能短**的：
        到了 0.3 s 但样本不够就继续攒，最多攒到 `max_window_s`。
        窗口拉长对判据其实更好——同样 50% 滑移，位移更大、激光看得更清。
        """
        if self.t0 is None:
            return []
        out = []
        tw = self.p['window_s']
        tw_max = max(tw, float(self.p.get('max_window_s', tw)))
        need = int(self.p.get('min_odom_samples', self.p['min_samples']))
        while now - self.t0 >= tw - 1e-9:
            t1 = self.t0 + tw
            if len(self.odom.between(self.t0, t1)) < need:
                # 0.3 s 里 odom 不够：把窗口往长里拉，能等到几条算几条
                t1 = min(now, self.t0 + tw_max)
                if len(self.odom.between(self.t0, t1)) < need and t1 < self.t0 + tw_max - 1e-9:
                    break          # 还没到上限，接着攒
            win = self._build(self.t0, t1)
            if win is not None:
                out.append(win)
            self.t0 = t1
        return out

    def _build(self, t0, t1):
        self.stats['windows'] += 1
        win = {
            't0': t0, 't1': t1, 'dt': t1 - t0,
            'valid': True, 'warmup': False, 'reason': '',
        }

        # --- 指令 ---
        # 遥控器/键盘只在"按键变化"时发一条，之后车按最后一条指令继续走
        # （这个包里 /cmd_vel 只有 0.6 Hz，机器人却一路在动）。所以不能用
        # "窗口内指令的跨度"算覆盖率——那样遥控录的包会被整片判无效。
        # 按"保持"语义：窗口中间生效的指令 = 窗口内插值，否则取窗口前最近一条；
        # 只要它非零就算覆盖，为零（没人让它动）就不判。
        mid = 0.5 * (t0 + t1)
        cmd_pts = self.cmd.between(t0, t1)
        lin = _interp(cmd_pts, mid)
        if lin is None:
            prev_cmd = self.cmd.last_before(t0) or self.cmd.last_before(mid)
            lin = prev_cmd[1] if prev_cmd else self._last_cmd
        active = (abs(lin[0]) >= self.p['cmd_active_eps']
                  or abs(lin[1]) >= self.p['cmd_active_eps'])
        cov = 1.0 if active else 0.0
        win['cmd_coverage'] = cov
        win['cmd_linear'] = lin[0]
        win['cmd_angular'] = lin[1]
        win['warmup'] = bool(self._warm_until and t0 < self._warm_until)

        # --- 里程计（编码器） ---
        od = self.odom.between(t0, t1)
        win['odom_n'] = len(od)
        if len(od) >= int(self.p.get('min_odom_samples', self.p['min_samples'])):
            gaps = [b[0] - a[0] for a, b in zip(od, od[1:])]
            if gaps and max(gaps) > self.p['max_gap_s']:
                win['valid'] = False
                win['reason'] = 'odom 断流 %.2fs' % max(gaps)
            win['denc'] = _trapz([(t, v[0]) for t, v in od])
            win['dtheta_enc'] = _trapz([(t, v[1]) for t, v in od])
            win['odom_ok'] = True
        else:
            win['denc'] = None
            win['dtheta_enc'] = None
            win['odom_ok'] = False
            if win['valid']:
                win['valid'] = False
                win['reason'] = 'odom 样本不足（本窗 %d 条）' % len(od)

        # --- 左右轮（累计米，差分） ---
        wh = self.wheel.between(t0, t1)
        if len(wh) >= self.p['min_samples']:
            win['d_left'] = wh[-1][1][0] - wh[0][1][0]
            win['d_right'] = wh[-1][1][1] - wh[0][1][1]
            win['wheel_ok'] = True
        else:
            win['d_left'] = win['d_right'] = None
            win['wheel_ok'] = False

        # --- IMU（陀螺积分） ---
        im = self.imu.between(t0, t1)
        if len(im) >= self.p['min_samples']:
            win['dtheta_imu'] = _trapz([(t, v) for t, v in im])
            win['imu_hz'] = (len(im) - 1) / (im[-1][0] - im[0][0]) if im[-1][0] > im[0][0] else 0.0
            win['imu_ok'] = True
        else:
            win['dtheta_imu'] = None
            win['imu_ok'] = False

        # --- 激光：取覆盖 t0、t1 的两帧，按时间插值 ---
        s0 = self.scan.last_before(t0)
        s1 = self.scan.last_before(t1)
        if s0 and s1 and s1[0] > s0[0]:
            win['scan_prev'] = s0[1]
            win['scan_cur'] = s1[1]
            win['scan_dt'] = s1[0] - s0[0]
            win['scan_ok'] = True
        else:
            win['scan_prev'] = win['scan_cur'] = None
            win['scan_dt'] = None
            win['scan_ok'] = False

        # 指令覆盖不够 -> 本窗不判（但保留数据便于调试）
        if cov < self.p['min_cmd_coverage']:
            win['valid'] = False
            win['reason'] = win['reason'] or '指令覆盖 %.0f%%' % (100 * cov)

        if not win['valid']:
            self.stats['skipped'] += 1

        # 丢掉老数据，防止内存一直涨
        cut = t1 - 2.0
        for s in (self.odom, self.wheel, self.imu, self.scan, self.cmd):
            s.clip(cut)
        return win


def window_travel_ok(win, min_speed=0.02):
    """窗口内到底有没有「在动」——给判据层做使能。"""
    if not win.get('odom_ok'):
        return False
    return abs(win['denc']) >= min_speed * win['dt']
