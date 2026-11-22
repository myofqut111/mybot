"""判据层：一个 0.3 s 窗口 -> 一个「有没有打滑 / 哪一类」的结论。

对应方案 v1.1 的 P1~P5。这里的实现刻意保持「纯函数」：
输入是 windows.py 产出的窗口字典，输出是结论字典，不碰 ROS、不碰时间。

为什么不再用「扫描变化百分比」那种粗糙代理：
它会同时被转向、动态障碍、回波噪声触发；而下面每条判据都要求
「某个量按运动学应该怎么变，实际没怎么变」——有方向、有符号、有量纲。
"""
import math

from . import ranges as ranges_mod


DEFAULTS = {
    # ---- 窗口与使能 ----
    'v_floor': 0.10,          # 归一化滑移率的分母下限 m/s
    'omega_min': 0.30,        # 旋转判据的使能死区 rad/s
    'd_floor_m': 0.005,       # 绝对位移死区
    # ---- 阈值 ----
    'r_high': 0.60,           # 确认用滑移率
    'r_low': 0.36,            # 解除用（= 0.6 * r_high）
    'r_omega': 0.60,
    'r_wheel': 0.50,
    'thresh_abs_m': 0.05,     # 绝对下限上限（会按运动量缩放）
    'thresh_scale': 0.5,      # 自适应系数：thresh_eff = min(abs, scale*denc)
    'thresh_lat_m': 0.03,     # 横向滑移
    'thresh_over_m': 0.03,    # 过位移（下坡滑行）
    'min_shortfall_m': 0.03,  # P1/P2 的绝对差下限（ICP 噪声 ~1.5 cm，取 2σ）
    'stall_eps_m': 0.005,     # 堵转：几乎没有位移
    'stall_eps_omega': 0.05,  # 堵转：几乎没有转角 rad/s
    'stall_cmd_linear': 0.05,
    'stall_cmd_angular': 0.15,
    'wheel_base_m': 0.175,    # 轮距，用于左右轮反解
    'scan_valid_ratio': 0.60, # 激光置信度门槛（内点比例）
    'turn_limit_rad': 0.30,   # 窗内转角超过它时 P1 只作参考
}

TYPES = (
    'STALL', 'WHEEL_L', 'WHEEL_R', 'ANGULAR_SLIP',
    'LINEAR_SLIP', 'LATERAL_SLIP', 'OVERRUN', 'SENSOR_CONFLICT',
)

# 出现多个候选时的优先次序：越靠前越具体、越紧急
PRIORITY = {t: i for i, t in enumerate(TYPES)}


def effective_threshold(denc, p):
    """随窗口运动量缩放的绝对下限。

    文档 5.2 的要点：低速时 denc 只有几厘米，固定 0.05 m 门槛会让
    「完全卡死」在数学上永远判不出来。这里按 denc 的一半缩放。
    """
    denc = abs(denc or 0.0)
    return min(p['thresh_abs_m'], p['thresh_scale'] * denc)


def _slip_ratio(expected, measured, floor):
    """归一化滑移率：1 = 完全没动，0 = 完全一致，负数 = 动得比预期多。"""
    denom = max(abs(expected), floor)
    if denom <= 1e-9:
        return 0.0
    return (expected - measured) / denom


def evaluate(win, params=None, fit=None):
    """返回 {'type', 'slip', 'ratios', 'reasons', 'detail'}。

    - `slip=True` 表示**本窗口**有异常候选，连续确认由状态机负责；
    - `type=None` 表示正常；
    - `fit` 可传入已算好的平移估计，省一次计算。
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)

    out = {
        'slip': False, 'type': None, 'ratios': {}, 'reasons': [],
        'detail': {}, 'usable': True,
    }

    if not win.get('valid'):
        out['usable'] = False
        out['reasons'].append(win.get('reason') or '窗口无效')
        return out
    if win.get('warmup'):
        out['usable'] = False
        out['reasons'].append('warmup')
        return out

    denc = win.get('denc') or 0.0
    dtheta_enc = win.get('dtheta_enc') or 0.0
    dtheta_imu = win.get('dtheta_imu')
    dt = win.get('dt') or p.get('window_s', 0.3)
    cmd_v = win.get('cmd_linear') or 0.0
    cmd_w = win.get('cmd_angular') or 0.0
    v_floor = max(p['v_floor'] * dt, p['d_floor_m'] * 4)

    # ---------- 激光平移估计 ----------
    if fit is None:
        if win.get('scan_ok'):
            fit = ranges_mod.fit_translation(win['scan_prev'], win['scan_cur'])
        else:
            fit = {'ok': False, 'reason': '无扫描', 'quality': 0.0, 'dx': None, 'dy': None,
                   'beams': 0, 'coverage': 0.0, 'residual': None}
    out['detail']['fit'] = fit
    scan_ok = bool(fit.get('ok')) and fit.get('coverage', 0.0) >= 0.0
    scan_trusted = scan_ok and (fit.get('quality', 0.0) >= 0.55) and \
        (fit.get('coverage', 1.0) >= p['scan_valid_ratio'])

    turning = abs(dtheta_enc) > p['turn_limit_rad']

    ratios = out['ratios']
    cands = []

    # ---------- P5 堵转：有指令、但轮子和车都没动 ----------
    cmd_active = abs(cmd_v) >= p['stall_cmd_linear'] or abs(cmd_w) >= p['stall_cmd_angular']
    if cmd_active and win.get('odom_ok'):
        no_travel = abs(denc) < p['stall_eps_m']
        no_turn = abs(dtheta_enc) * 2.0 / dt < p['stall_eps_omega']
        if no_travel and no_turn:
            ratios['stall'] = 1.0
            cands.append(('STALL', 1.0, '有指令（v=%.2f w=%.2f）但 odom 位移 %.3fm、转角 %.3frad'
                          % (cmd_v, cmd_w, denc, dtheta_enc)))

    # ---------- P4 旋转滑移：编码器说转了，陀螺说不转 ----------
    if dtheta_imu is not None and abs(dtheta_enc) >= p['omega_min'] * dt:
        s_w = _slip_ratio(dtheta_enc, dtheta_imu, floor=p['omega_min'] * dt)
        ratios['omega'] = s_w
        if s_w >= p['r_omega']:
            cands.append(('ANGULAR_SLIP', s_w,
                          '编码器转速 %.2frad/s，陀螺只有 %.2frad/s（滑移率 %.2f）'
                          % (dtheta_enc / dt, dtheta_imu / dt, s_w)))

    # ---------- P3 左右轮独立滑移 ----------
    # 前置条件：车体确实往前走了，「单轮期望位移」才有意义。
    # dx≈0 时 exp_l = dx - half ≈ 0，而 d_left > 0，会算出 -1.0 这种假滑移率；
    # 那其实是「整体没动」，归 P1 管，这里不该参与（否则真机上满屏 -1.0）。
    dx = fit.get('dx') or 0.0
    p3_allowed = dx >= p['min_shortfall_m']
    if win.get('wheel_ok') and dtheta_imu is not None and scan_trusted \
            and not turning and p3_allowed:
        half = 0.5 * p['wheel_base_m'] * dtheta_imu
        exp_l = dx - half
        exp_r = dx + half
        s_l = _slip_ratio(exp_l, win['d_left'], floor=v_floor)
        s_r = _slip_ratio(exp_r, win['d_right'], floor=v_floor)
        ratios['wheel_l'] = s_l
        ratios['wheel_r'] = s_r
        if s_l >= p['r_wheel'] and s_l > s_r + 0.1:
            cands.append(('WHEEL_L', s_l, '左轮走了 %.3fm，车体只走了 %.3fm' % (win['d_left'], exp_l)))
        elif s_r >= p['r_wheel'] and s_r > s_l + 0.1:
            cands.append(('WHEEL_R', s_r, '右轮走了 %.3fm，车体只走了 %.3fm' % (win['d_right'], exp_r)))

    # ---------- P1 直行滑移：轮子说走了，激光说没走 ----------
    if scan_trusted and not turning and abs(denc) > effective_threshold(denc, p):
        s = _slip_ratio(denc, fit['dx'], floor=v_floor)
        ratios['linear'] = s
        shortfall = denc - (fit['dx'] or 0.0)
        # 低速度下 ICP 的绝对误差和位移同量级，差得不够多就不判
        if s >= p['r_high'] and shortfall >= p['min_shortfall_m']:
            cands.append(('LINEAR_SLIP', s,
                          '编码器 %.3fm，激光只有 %.3fm（滑移率 %.2f，激光可信度 %.2f）'
                          % (denc, fit['dx'], s, fit['quality'])))
        # ---------- P2 过位移 / 侧滑 ----------
        if s <= -p['r_high'] and -shortfall >= max(p['min_shortfall_m'], p['thresh_over_m']):
            cands.append(('OVERRUN', -s, '车体走了 %.3fm，轮子只记了 %.3fm（下坡/被推）'
                          % (fit['dx'], denc)))
        if abs(cmd_w) < 0.10 and abs(fit.get('dy') or 0.0) > p['thresh_lat_m']:
            cands.append(('LATERAL_SLIP', abs(fit['dy']), '横向漂了 %.3fm' % fit['dy']))

    # ---------- 传感器互相矛盾 ----------
    if dtheta_imu is not None and scan_trusted and abs(dtheta_enc) < 0.02 and \
            abs(dtheta_imu) > max(p['omega_min'] * dt, 0.05):
        cands.append(('SENSOR_CONFLICT', 1.0, '编码器说没转，陀螺说转了 %.3frad' % dtheta_imu))

    if not cands:
        return out

    cands.sort(key=lambda c: PRIORITY.get(c[0], 99))
    out['slip'] = True
    out['type'] = cands[0][0]
    out['ratios']['primary'] = cands[0][1]
    for t, r, why in cands:
        out['reasons'].append('%s: %s' % (t, why))
    return out


def is_released(win, params=None, fit=None):
    """滞回：滑移率回到 r_low 以下才算「真的好了」。

    注意这里不能只看 evaluate 的 slip 布尔——r_high 与 r_low 之间有一段
    「还在滑、但不够确认」的缓冲区，状态机在这段里应当继续保持恢复状态，
    否则会出现「滑移率 0.5 反复跨线 -> 状态抖动」。
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    res = evaluate(win, p, fit)
    if res['type'] in ('STALL', 'SENSOR_CONFLICT'):
        return False, res
    r = res['ratios']
    worst = max(
        r.get('omega', 0.0),
        r.get('linear', 0.0),
        r.get('wheel_l', 0.0),
        r.get('wheel_r', 0.0),
    )
    if worst >= p['r_low']:
        return False, res
    return True, res
