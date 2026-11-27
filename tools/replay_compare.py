"""离线对比：轮式 odom / 激光 ICP / 协方差加权融合 三条位移轨迹（方案 §7.1）。

不启 ROS 节点，直接读 bag，用与 scan_match_odom 相同的口径算激光速度，
再用同一套协方差模型做"模拟融合"，回答一个问题：

    打滑段里，把激光速度按这样的协方差权重融进去，位移前冲能减少多少？

用法：
    python3 replay_compare.py ~/bags/manual_1127_1451 --start 238 --end 258
"""
import argparse
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

sys.path.insert(0, '/home/my/fishbot_full_ws/install/fishbot_slip/lib/python3.10/site-packages')
from fishbot_slip import ranges as ranges_mod  # noqa: E402

# 与 scan_match_odom 默认参数保持一致
SIGMA_ODOM_VX = 0.14      # /odom twist 协方差 0.02 开方
SIGMA_SCAN_BASE = 0.020
SIGMA_SCAN_GAIN = 0.15
QUALITY_MIN = 0.30


def read_bag(uri, start, end):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    t0 = None
    scans, odoms = [], []
    while r.has_next():
        topic, data, ts = r.read_next()
        t = ts * 1e-9
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel < start or rel > end:
            continue
        if topic == '/scan':
            scans.append((rel, deserialize_message(data, LaserScan)))
        elif topic == '/odom':
            m = deserialize_message(data, Odometry)
            odoms.append((rel, m.twist.twist.linear.x, m.twist.twist.angular.z))
    return scans, odoms


def scan_speeds(scans):
    out = []
    prev = None
    for t, msg in scans:
        if prev is not None:
            t0, m0 = prev
            dt = t - t0
            if 0.05 <= dt <= 0.60:
                # fit_translation 自己会调 _points()，要传扫描对象
                fit = ranges_mod.fit_translation(m0, msg)
                if fit.get('ok') and (fit.get('quality') or 0) >= QUALITY_MIN:
                    q = fit['quality']
                    out.append((t, fit['dx'] / dt, fit['dtheta'] / dt, q))
        prev = (t, msg)
    return out


def integrate(series, step=0.05):
    """series: [(t, vx)] → (t, x) 累计位移。"""
    out = [(0.0, 0.0)]
    x = 0.0
    t_prev = None
    for t, vx in series:
        if t_prev is not None and t > t_prev:
            x += vx * min(t - t_prev, 0.5)
        t_prev = t
        out.append((t, x))
    return out


def at(series, t):
    """线性插值取值。"""
    if not series:
        return 0.0
    if t <= series[0][0]:
        return series[0][1]
    for i in range(1, len(series)):
        if series[i][0] >= t:
            t0, v0 = series[i - 1]
            t1, v1 = series[i]
            if t1 <= t0:
                return v1
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return series[-1][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--end', type=float, default=1e9)
    ap.add_argument('--focus', default='',
                    help='重点区间，逗号分隔，如 244:252')
    args = ap.parse_args()

    scans, odoms = read_bag(args.bag, args.start, args.end)
    sp = scan_speeds(scans)
    print('区间 %.1f~%.1f s：扫描 %d 帧，/odom %d 条，有效激光速度 %d 个'
          % (args.start, args.end, len(scans), len(odoms), len(sp)))

    odom_ser = [(t, vx) for t, vx, _ in odoms]
    scan_ser = [(t, vx) for t, vx, _, _ in sp]

    # 融合：按 1/σ² 加权（σ 与 scan_match_odom 同模型）
    fused = []
    for t, vx_s, _, q in sp:
        # 找最近的 odom 速度
        best = None
        for to, vxo, _ in odoms:
            if best is None or abs(to - t) < abs(best[0] - t):
                best = (to, vxo)
        v_o = best[1] if best else 0.0
        s_s = SIGMA_SCAN_BASE + (1.0 - q) * SIGMA_SCAN_GAIN
        w_o = 1.0 / (SIGMA_ODOM_VX ** 2)
        w_s = 1.0 / (s_s ** 2)
        fused.append((t, (w_o * v_o + w_s * vx_s) / (w_o + w_s)))

    x_odom = integrate(odom_ser)
    x_scan = integrate(scan_ser)
    x_fuse = integrate(fused)

    windows = []
    if args.focus:
        for part in args.focus.split(','):
            a, b = part.split(':')
            windows.append((float(a), float(b)))
    else:
        windows = [(args.start, min(args.end, args.start + 10.0))]

    print('%8s %10s %10s %10s' % ('区间(s)', 'odom(m)', '激光(m)', '融合(m)'))
    for a, b in windows:
        d_o = at(x_odom, b) - at(x_odom, a)
        d_s = at(x_scan, b) - at(x_scan, a)
        d_f = at(x_fuse, b) - at(x_fuse, a)
        print('%4.1f-%4.1f %10.3f %10.3f %10.3f' % (a, b, d_o, d_s, d_f))

    if sp:
        qs = sorted(q for _, _, _, q in sp)
        print('激光质量：中位 %.2f，10%% 分位 %.2f，最小 %.2f'
              % (qs[len(qs) // 2], qs[max(0, len(qs) // 10)], qs[0]))


if __name__ == '__main__':
    main()
