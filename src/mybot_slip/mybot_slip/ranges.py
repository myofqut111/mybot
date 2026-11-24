"""用两帧激光估「车体真实平移/转角」——方案里 P1/P2/P3 的实测来源。

为什么不是「按角度逐束比测距」：
    对着一面斜墙时，同一束激光打在墙上的点会沿墙面滑动，
    r'(θ) = r − (t·n)/(n·û)，系数取决于墙面法向，不是简单的 cosθ。
    直接套 Δr = −(dx·cosθ + dy·sinθ) 会系统性地低估位移。

这里做的是**点对点 ICP 的一步**（够用且便宜）：
1. 两帧都转成机器人坐标系下的点；
2. 用 0.1 m 的网格哈希找最近点，限制在 0.5 m 内；
3. 线性化小角度旋转，最小二乘解 (dx, dy, dθ)；
4. 用残差中位数做一次鲁棒剔除，再解一次（对付行人/动态遮挡）；
5. 迭代 3 次重找对应点。

输出里 `ok=False` 表示「这一窗测不出真实运动」，判据层必须把它当
「没有证据」，绝不能当「车没动」。
"""
import math


DEFAULTS = {
    'min_points': 40,          # 有效点数下限
    'max_corr_dist': 0.50,     # 最近点搜索半径（要大于窗内位移）
    'cell_m': 0.10,            # 网格哈希边长
    'iterations': 3,
    'outlier_sigma': 2.5,      # 鲁棒剔除阈值（残差中位数倍数）
    'min_inlier_ratio': 0.55,  # 内点比例下限
    'residual_scale_m': 0.03,  # 残差 -> 质量
    'max_shift_m': 0.60,
    'max_rot_rad': 0.35,
}


def _field(scan, key, default=0.0):
    """dict 和 LaserScan 消息都能取字段（回放时传的是消息）。"""
    if isinstance(scan, dict):
        val = scan.get(key)
    else:
        val = getattr(scan, key, None)
    return default if val is None else val


def _points(scan, min_range=0.05, max_ratio=0.98):
    rs = _field(scan, 'ranges', []) or []
    a0 = float(_field(scan, 'angle_min', 0.0))
    inc = float(_field(scan, 'angle_increment', 0.0))
    rmax = float(_field(scan, 'range_max', 0.0) or 0.0)
    limit = rmax * max_ratio if rmax > 0 else float('inf')
    pts = []
    for i, r in enumerate(rs):
        if r is None:
            continue
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if not (r == r) or math.isinf(r):      # NaN / inf
            continue
        if r < min_range or r > limit:
            continue
        th = a0 + i * inc
        pts.append((r * math.cos(th), r * math.sin(th)))
    return pts


def _build_grid(pts, cell):
    g = {}
    inv = 1.0 / cell
    for x, y in pts:
        k = (int(math.floor(x * inv)), int(math.floor(y * inv)))
        g.setdefault(k, []).append((x, y))
    return g, inv


def _nearest(grid, inv, cell, px, py, max_d):
    cx, cy = int(math.floor(px * inv)), int(math.floor(py * inv))
    span = int(math.ceil(max_d / cell))
    best = None
    best_d2 = max_d * max_d
    for gx in range(cx - span, cx + span + 1):
        for gy in range(cy - span, cy + span + 1):
            for qx, qy in grid.get((gx, gy), ()):
                d2 = (qx - px) ** 2 + (qy - py) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best = (qx, qy)
    return best, math.sqrt(best_d2) if best else None


def scan_motion(prev, cur, params=None):
    """两帧激光之间「世界到底动没动」——不做配准，直接量最近点距离中位数。

    这是给 ICP 打的补丁。ICP 在长走廊/重复结构里会陷进局部极小：
    车明明走了 6.6 cm，它却算出 `dx≈0`，看着跟打滑一模一样。两者的区别是：

    * 真打滑：车没动，两帧扫描来自同一视角 → `scan_motion` ≈ 0
    * ICP 失败：车真动了 6.6 cm → `scan_motion` ≈ 6.6 cm

    所以判据层再加一条：`dx≈0` 但 `scan_motion` 明显不为 0 时，不许判滑移。
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    src = _points(cur)
    dst = _points(prev)
    if len(src) < p['min_points'] or len(dst) < p['min_points']:
        return None
    grid, inv = _build_grid(dst, p['cell_m'])
    ds = []
    for px, py in src:
        _q, d = _nearest(grid, inv, p['cell_m'], px, py, p['max_corr_dist'])
        ds.append(d if d is not None else p['max_corr_dist'])
    if not ds:
        return None
    ds.sort()
    return ds[len(ds) // 2]


def fit_translation(prev, cur, params=None, init=None):
    """估计 prev -> cur 的刚体变换（cur 相对 prev，单位米/弧度）。

    返回 dict(dx, dy, dtheta, beams, coverage, residual, quality, ok, reason)
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)

    src = _points(cur)
    dst = _points(prev)
    if len(src) < p['min_points'] or len(dst) < p['min_points']:
        return _fail('点数不足 %d/%d' % (len(src), len(dst)))

    grid, inv = _build_grid(dst, p["cell_m"])
    # §4：扫描匹配以里程计预测为初值，否则长走廊/重复结构容易陷局部极小
    if init is not None:
        dx, dy, dth = (float(init[0]), float(init[1]),
                       float(init[2]) if len(init) > 2 else 0.0)
    else:
        dx = dy = dth = 0.0
    inliers = 0
    residual = None

    for _ in range(p['iterations']):
        # 用当前估计把 src 变换过去，再找最近点
        c, s = math.cos(dth), math.sin(dth)
        pairs = []
        for px, py in src:
            rx = c * px - s * py + dx
            ry = s * px + c * py + dy
            q, _d = _nearest(grid, inv, p['cell_m'], rx, ry, p['max_corr_dist'])
            if q is not None:
                pairs.append((px, py, q[0], q[1]))
        if len(pairs) < p['min_points']:
            return _fail('对应点不足 %d' % len(pairs))

        sol = _solve(pairs, p)
        if sol is None:
            return _fail('法方程奇异')
        dx, dy, dth = sol['dx'], sol['dy'], sol['dtheta']
        residual = sol['residual']
        # 鲁棒剔除：残差大于 k×中位数的点不要
        keep = _inliers(pairs, dx, dy, dth, p['outlier_sigma'])
        if len(keep) >= p['min_points'] and len(keep) < len(pairs):
            sol2 = _solve(keep, p)
            if sol2 is not None:
                dx, dy, dth = sol2['dx'], sol2['dy'], sol2['dtheta']
                residual = sol2['residual']
                inliers = len(keep)
        else:
            inliers = len(pairs)

    coverage = inliers / float(len(src))
    shift = math.hypot(dx, dy)
    quality = coverage / (1.0 + (residual / p['residual_scale_m']) ** 2)
    ok = (quality >= 0.55 and coverage >= p['min_inlier_ratio']
          and shift <= p['max_shift_m'] and abs(dth) <= p['max_rot_rad'])
    reason = ''
    if not ok:
        if shift > p['max_shift_m']:
            reason = '位移 %.2fm 超范围' % shift
        elif abs(dth) > p['max_rot_rad']:
            reason = '转角 %.2frad 超范围' % dth
        elif coverage < p['min_inlier_ratio']:
            reason = '内点只有 %.0f%%' % (100 * coverage)
        else:
            reason = '可信度 %.2f 低（残差 %.3fm）' % (quality, residual or 0.0)

    return {
        'dx': dx, 'dy': dy, 'dtheta': dth, 'beams': inliers,
        'coverage': coverage, 'residual': residual, 'quality': quality,
        'ok': ok, 'reason': reason, 'motion': scan_motion(prev, cur, p),
    }


def _solve(pairs, p):
    """最小二乘解 (dx, dy, dθ)。

    残差： p + (t + dθ·J p) - q，J = [[0,-1],[1,0]]
    """
    a11 = a12 = a13 = a22 = a23 = a33 = 0.0
    b1 = b2 = b3 = 0.0
    for px, py, qx, qy in pairs:
        ex = qx - px
        ey = qy - py
        # 行1: dx - dθ*py = ex ; 行2: dy + dθ*px = ey
        a11 += 1.0
        a12 += 0.0
        a13 += -py
        a22 += 1.0
        a23 += px
        a33 += px * px + py * py
        b1 += ex
        b2 += ey
        b3 += ex * (-py) + ey * px
    n = float(len(pairs))
    if n < 1:
        return None
    # 3x3 对称阵求逆（行列式法）
    det = (a11 * (a22 * a33 - a23 * a23)
           - a12 * (a12 * a33 - a23 * a13)
           + a13 * (a12 * a23 - a22 * a13))
    if abs(det) < 1e-9:
        return None
    dx = (b1 * (a22 * a33 - a23 * a23)
          - a12 * (b2 * a33 - a23 * b3)
          + a13 * (b2 * a23 - a22 * b3)) / det
    dy = (a11 * (b2 * a33 - a23 * b3)
          - b1 * (a12 * a33 - a23 * a13)
          + a13 * (a12 * b3 - b2 * a13)) / det
    dth = (a11 * (a22 * b3 - b2 * a23)
           - a12 * (a12 * b3 - b2 * a13)
           + b1 * (a12 * a23 - a22 * a13)) / det
    # 残差
    c, s = math.cos(dth), math.sin(dth)
    tot = 0.0
    for px, py, qx, qy in pairs:
        rx = c * px - s * py + dx - qx
        ry = s * px + c * py + dy - qy
        tot += rx * rx + ry * ry
    return {'dx': dx, 'dy': dy, 'dtheta': dth,
            'residual': math.sqrt(tot / len(pairs))}


def _inliers(pairs, dx, dy, dth, sigma):
    c, s = math.cos(dth), math.sin(dth)
    res = []
    for px, py, qx, qy in pairs:
        rx = c * px - s * py + dx - qx
        ry = s * px + c * py + dy - qy
        res.append(math.hypot(rx, ry))
    ordered = sorted(res)
    med = ordered[len(ordered) // 2]
    thr = max(med * sigma, 0.02)
    return [pr for pr, r in zip(pairs, res) if r <= thr]


def _fail(reason):
    return {'dx': None, 'dy': None, 'dtheta': None, 'beams': 0,
            'coverage': 0.0, 'residual': None, 'quality': 0.0,
            'ok': False, 'reason': reason}
