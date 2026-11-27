#!/usr/bin/env python3
"""离线评估：轮式 odom / 激光 ICP / 1/σ² 融合 三条位移轨迹。

和线上用同一套 `scan_match_odom.decide_cov()`（质量分档 + 按轴放大），
所以这里算出的权重就是 EKF 会用的权重。注意这是**离线模拟**，不是跑真 EKF。

用法：
    source /opt/ros/humble/setup.bash
    python3 tools/fusion_eval.py ~/bags/base_1127_1717 --window 0:129,129:137
"""
import argparse
import bisect
import math
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mybot_slip import ranges as R
from mybot_slip.scan_match_odom import decide_cov

CFG = {'r_good': 0.60, 'r_mid': 0.25, 'mid_scale': 3.0, 'degen_scale': 6.0,
       'degen_cond': 0.20, 'along_cos': 0.80, 'publish_mid': True}
SIGMA_VX = 0.020        # 激光速度基础 σ（方案 §5 基础值）
SIGMA_WHEEL = 0.14      # 轮式里程 σ（m/s）：正常约 0.02，按打滑场景取保守值


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'),
           ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    t0 = None
    odoms = []
    scans = []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if t0 is None:
            t0 = stamp
        rel = (stamp - t0) / 1e9
        if topic == '/odom':
            m = deserialize_message(data, get_message(types[topic]))
            odoms.append((rel, m.pose.pose.position.x, m.pose.pose.position.y,
                          yaw_of(m.pose.pose.orientation),
                          m.twist.twist.linear.x))
        elif topic == '/scan':
            scans.append((rel, deserialize_message(data, get_message(types[topic]))))
    return odoms, scans


def odom_at(odoms, times, t):
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return odoms[0]
    if i >= len(odoms):
        return odoms[-1]
    a, b = odoms[i - 1], odoms[i]
    if abs(a[0] - t) <= abs(b[0] - t):
        return a
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--window', default='0:1e9')
    ap.add_argument('--min-dt', type=float, default=0.05)
    ap.add_argument('--max-dt', type=float, default=0.6)
    args = ap.parse_args()

    odoms, scans = load(args.bag)
    if len(scans) < 3:
        print('扫描太少')
        return 1
    times = [o[0] for o in odoms]
    steps = []
    for i in range(1, len(scans)):
        t_a, scan_a = scans[i - 1]
        t_b, scan_b = scans[i]
        dt = t_b - t_a
        if not (args.min_dt <= dt <= args.max_dt):
            continue
        oa = odom_at(odoms, times, t_a)
        ob = odom_at(odoms, times, t_b)
        # 用机体前向速度积分：和激光的 dx 是同一个量，才能公平比较
        d_odom = 0.5 * ((oa[4] or 0.0) + (ob[4] or 0.0)) * dt
        yaw_d = ob[3] - oa[3]
        while yaw_d > math.pi:
            yaw_d -= 2 * math.pi
        while yaw_d < -math.pi:
            yaw_d += 2 * math.pi
        fit = R.fit_translation(scan_a, scan_b,
                                params={'min_quality': 0.25, 'residual_scale_m': 0.05},
                                init=(d_odom, 0.0, yaw_d))
        res = decide_cov(fit.get('quality') or 0.0, fit.get('cond'),
                         fit.get('weak_angle'), CFG)
        if 'drop' in res or not fit.get('ok'):
            why = res.get('drop') or fit.get('reason') or '-'
            steps.append((t_b, dt, d_odom, None, None, why[:28], '-'))
            continue
        d_laser = fit.get('dx') or 0.0
        sig = SIGMA_VX * res['sx'] * dt
        steps.append((t_b, dt, d_odom, d_laser, sig, res['tier'], res['degen']))

    wins = []
    for part in args.window.split(','):
        a, b = part.split(':')
        wins.append((float(a), float(b)))

    print('步数 %d（有效激光 %d）' % (len(steps), sum(1 for s in steps if s[3] is not None)))
    for a, b in wins:
        sel = [s for s in steps if a <= s[0] < b]
        if not sel:
            continue
        d_odom = sum(s[2] for s in sel)
        d_las = sum(s[3] for s in sel if s[3] is not None)
        # 1/σ² 融合：逐拍把两路位移加权平均，再把各拍加起来
        d_fuse = 0.0
        for s in sel:
            w_o = 1.0 / (SIGMA_WHEEL * s[1]) ** 2
            num = w_o * s[2]
            den = w_o
            if s[3] is not None:
                w_l = 1.0 / s[4] ** 2
                num += w_l * s[3]
                den += w_l
            d_fuse += num / den
        tiers = {}
        degen = {}
        drops = {}
        for s in sel:
            tiers[s[5]] = tiers.get(s[5], 0) + 1
            degen[s[6]] = degen.get(s[6], 0) + 1
            if s[3] is None:
                key = s[5].split('（')[0]
                drops[key] = drops.get(key, 0) + 1
        print('=' * 62)
        print('区间 %.1f ~ %.1f s：%d 步' % (a, b, len(sel)))
        print('  轮式 odom   %.3f m' % d_odom)
        print('  激光 ICP    %.3f m（%d 步有效）'
              % (d_las, sum(1 for s in sel if s[3] is not None)))
        print('  1/σ² 融合   %.3f m   → 前冲压到轮式的 %.0f%%'
              % (d_fuse, 100.0 * d_fuse / d_odom if d_odom else 0.0))
        print('  质量档 %s   退化 %s' % (tiers, degen))
        if drops:
            print('  丢弃原因 %s' % drops)
    return 0


if __name__ == '__main__':
    sys.exit(main())
