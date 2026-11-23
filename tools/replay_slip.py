#!/usr/bin/env python3
"""把一段 bag 喂给打滑判据，看会不会误报。

用法（虚拟机里）：
    python3 tools/replay_slip.py ~/bags/2026
    python3 tools/replay_slip.py ~/bags/2026 --verbose

它不启动 ROS 节点，直接按 bag 里的时间戳把消息喂进
WindowAggregator，再逐窗口跑 criteria.evaluate()，
最后打印：窗口数、无效窗口、各类型命中次数、以及每个命中窗口的详情。

**这是打滑判据的发布门槛：正常跑一段，命中必须是 0。**
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'mybot_slip'))

try:                                    # 源码树
    from mybot_slip import criteria, ranges, state, windows
except ImportError:                     # 安装后（VM 上是 fishbot_slip）
    from fishbot_slip import criteria, ranges, state, windows

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANTED = {
    '/odom': 'odom',
    '/wheel_joints': 'wheel',
    '/imu': 'imu',
    '/scan': 'scan',
    '/scan_filtered': 'scan',
    '/cmd_vel': 'cmd',
}


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage, conv)
    return reader


def stamp_of(msg, fallback):
    st = getattr(getattr(msg, 'header', None), 'stamp', None)
    if st is None:
        return fallback
    v = st.sec + st.nanosec * 1e-9
    return v if (st.sec or st.nanosec) else fallback


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--verbose', action='store_true', help='打印每个命中窗口')
    ap.add_argument('--max-windows', type=int, default=0)
    ap.add_argument('--window', type=float, default=0.0,
                    help='覆盖窗口长度（秒），默认用 windows.DEFAULTS')
    ap.add_argument('--sim-slip', default='',
                    help='人为制造滑移 start:end:factor（秒，相对 bag 开头），'
                         '例如 60:75:0.2 表示这段编码器位移只按 20% 算')
    args = ap.parse_args()

    reader = open_reader(args.bag)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    types = {}
    agg_params = {}
    if args.window > 0:
        agg_params['window_s'] = args.window
    agg = windows.WindowAggregator(agg_params or None)
    machine = state.SlipStateMachine()
    confirmed = []
    slip_a = slip_b = 0.0
    slip_f = None
    if args.sim_slip:
        a, b, f = args.sim_slip.split(':')
        slip_a, slip_b, slip_f = float(a), float(b), float(f)
    t_start = None
    hits = {}
    n_win = 0
    n_invalid = 0
    inv_why = {}
    n_slip = 0
    fed = {}
    last_t = 0.0
    first_t = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        kind = WANTED.get(topic)
        if kind is None:
            continue
        fallback = t_ns * 1e-9
        if topic not in types:
            if topic not in topic_types:
                raise KeyError('bag 里没有类型信息：%s' % topic)
            types[topic] = get_message(topic_types[topic])
        msg = deserialize_message(data, types[topic])
        t = stamp_of(msg, fallback)
        fed[kind] = fed.get(kind, 0) + 1

        if kind == 'odom':
            vx = msg.twist.twist.linear.x
            if slip_f and t_start is not None and slip_a <= (t - t_start) <= slip_b:
                vx *= slip_f                    # 人为制造"轮子转、车不走"
            agg.add_odom(t, vx, msg.twist.twist.angular.z)
        elif kind == 'wheel':
            pos = list(msg.position)
            if len(pos) >= 2:
                agg.add_wheel(t, pos[0], pos[1])
        elif kind == 'imu':
            agg.add_imu(t, msg.angular_velocity.z)
        elif kind == 'scan':
            if msg.ranges:
                agg.add_scan(t, msg)
        elif kind == 'cmd':
            agg.add_cmd(t, msg.linear.x, msg.angular.z)

        if first_t is None:
            first_t = t
            t_start = t
        # 每 0.1 s 推进一次（窗口 0.3 s，足够跟上）
        if t - last_t >= 0.1:
            last_t = t
            for win in agg.poll(t):
                n_win += 1
                if not win.get('valid'):
                    n_invalid += 1
                    why = win.get('reason') or '?'
                    why = why.split(' ')[0]
                    inv_why[why] = inv_why.get(why, 0) + 1
                    continue
                fit = None
                if win.get('scan_ok'):
                    fit = ranges.fit_translation(win['scan_prev'], win['scan_cur'])
                v = criteria.evaluate(win, None, fit)
                if os.environ.get('SHOW_MOTION') and v.get('slip'):
                    print('  t=%.1f motion=%s dx=%s denc=%s q=%s' % (
                        win['t1'], (fit or {}).get('motion'), (fit or {}).get('dx'),
                        win.get('denc'), (fit or {}).get('quality')))
                act = machine.step(win['t1'], v)
                if act.get('action'):
                    confirmed.append((win['t1'], v['type'], act['action']))
                if v.get('slip'):
                    n_slip += 1
                    hits[v['type']] = hits.get(v['type'], 0) + 1
                    if args.verbose:
                        print('t=%.1f %s %s' % (win['t1'] - first_t, v['type'],
                                                '; '.join(v['reasons'])[:150]))
            if args.max_windows and n_win >= args.max_windows:
                break

    dur = (last_t - first_t) if first_t is not None else 0.0
    print('=' * 60)
    print('bag            : %s' % args.bag)
    print('时长           : %.1f s' % dur)
    print('喂进去的消息   : %s' % ', '.join('%s=%d' % (k, v) for k, v in sorted(fed.items())))
    print('窗口           : %d（无效 %d）' % (n_win, n_invalid))
    if inv_why:
        print('   无效原因    : %s' % ', '.join(
            '%s=%d' % (k, v) for k, v in sorted(inv_why.items(), key=lambda kv: -kv[1])[:6]))
    print('候选窗口       : %d' % n_slip)
    print('确认（会动作） : %d' % len(confirmed))
    for t, typ, act in confirmed:
        print('   t=%.1f %s -> %s' % (t - (first_t or 0.0), typ, act))
    for k, v in sorted(hits.items(), key=lambda kv: -kv[1]):
        print('   %-16s %d' % (k, v))
    print('=' * 60)
    print('门槛：正常跑一段，"命中"应当是 0。')
    return 0 if not confirmed else 2


def _type_of(reader, topic):
    for t in reader.get_all_topics_and_types():
        if t.name == topic:
            return t.type
    raise KeyError(topic)


if __name__ == '__main__':
    sys.exit(main())
