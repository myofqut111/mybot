#!/usr/bin/env python3
"""A 阶段基线报告：从一份 rosbag 里量出定位/建图/门控的关键数字。

用法：
    source /opt/ros/humble/setup.bash
    python3 tools/map_report.py ~/bags/base_1127_1717 [--markers]

输出分七块：
  1. 概况（时长、里程、话题条数）
  2. 位姿（filtered 逐帧位移的跳变次数与幅度、odom 与 filtered 的分歧）
  3. 建图（末帧地图的占用/空闲/孤立障碍）
  4. 门控（/scan → /scan_safe 的丢弃率、map_allowed=False 的时长占比）
  5. 激光速度（发布率、协方差分档、与轮速的冲突率）
  6. 恢复动作（次数、成功/失败、位移分布）
  7. 分段（按 /bag_marker；没有标记就按里程是否在走来切）
"""
import argparse
import json
import math
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

JUMP_M = 0.15          # 一帧位移超过这个值算跳变
ISO_NEIGH = 2          # 8 邻域里占用格少于这个数算孤立


def open_bag(uri):
    r = SequentialReader()
    r.open(StorageOptions(uri=uri, storage_id='sqlite3'), ConverterOptions('', ''))
    return r, {t.name: t.type for t in r.get_all_topics_and_types()}


def isolated_ratio(grid, w, h, data):
    occ = [i for i, v in enumerate(data) if v >= 65]
    if not occ:
        return 0, 0.0
    iso = 0
    for idx in occ:
        y, x = divmod(idx, w)
        n = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                yy, xx = y + dy, x + dx
                if 0 <= yy < h and 0 <= xx < w and data[yy * w + xx] >= 65:
                    n += 1
        if n < ISO_NEIGH:
            iso += 1
    return len(occ), iso / float(len(occ))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--markers', action='store_true', help='连标记时间点一起打印')
    args = ap.parse_args()

    r, types = open_bag(args.bag)
    cnt = {}
    t0 = None
    last_odom = None
    last_filt = None
    odom_dist = 0.0
    jumps = []
    div = []                      # |filtered - odom| 的逐步分歧
    filt_x = filt_y = odom_x = odom_y = 0.0
    grid = None
    occ = idle = unk = 0
    iso = 0.0
    scan_n = safe_n = 0
    map_true = map_false = 0
    map_false_since = None
    map_false_time = 0.0
    icp_sig = []
    reports = []
    markers = []
    rate_hist = []

    while r.has_next():
        topic, data, stamp = r.read_next()
        cnt[topic] = cnt.get(topic, 0) + 1
        if t0 is None:
            t0 = stamp
        rel = (stamp - t0) / 1e9

        if topic == '/odom':
            m = deserialize_message(data, get_message(types[topic]))
            p = m.pose.pose.position
            odom_x, odom_y = p.x, p.y
            if last_odom is not None:
                odom_dist += math.hypot(p.x - last_odom[0], p.y - last_odom[1])
            last_odom = (p.x, p.y)

        elif topic == '/odometry/filtered':
            m = deserialize_message(data, get_message(types[topic]))
            p = m.pose.pose.position
            if last_filt is not None:
                d = math.hypot(p.x - last_filt[0], p.y - last_filt[1])
                if d > JUMP_M:
                    jumps.append((rel, d))
                div.append((rel, math.hypot(p.x - odom_x, p.y - odom_y)))
            else:
                filt_x, filt_y = p.x, p.y
            last_filt = (p.x, p.y)

        elif topic == '/map':
            m = deserialize_message(data, get_message(types[topic]))
            w, h, d = m.info.width, m.info.height, m.data
            occ = sum(1 for v in d if v >= 65)
            unk = sum(1 for v in d if v < 0)
            idle = len(d) - occ - unk
            grid = (w, h, d)

        elif topic == '/scan':
            scan_n += 1

        elif topic in ('/scan_safe',):
            safe_n += 1

        elif topic == '/slip/state':
            try:
                st = json.loads(deserialize_message(
                    data, get_message(types[topic])).data)
            except Exception:                      # noqa: BLE001
                continue
            allowed = st.get('map_allowed')
            if allowed:
                map_true += 1
                if map_false_since is not None:
                    map_false_time += rel - map_false_since
                    map_false_since = None
            else:
                map_false += 1
                if map_false_since is None:
                    map_false_since = rel

        elif topic == '/scan_match/log_only':
            m = deserialize_message(data, get_message(types[topic]))
            cov = list(m.twist.covariance)
            if cov and cov[0] > 0:
                icp_sig.append(math.sqrt(cov[0]))

        elif topic == '/slip/report':
            try:
                reports.append(json.loads(deserialize_message(
                    data, get_message(types[topic])).data))
            except Exception:                      # noqa: BLE001
                pass

        elif topic == '/bag_marker':
            m = deserialize_message(data, get_message(types[topic]))
            markers.append((rel, m.data))

    duration = 0.0
    if last_filt is not None and t0 is not None:
        duration = (stamp - t0) / 1e9 if stamp else 0.0

    # ---------- 1 概况 ----------
    print('=' * 62)
    print('1 概况')
    print('  时长 %.1f s，话题 %d 个，总里程 %.2f m' % (
        duration, len(cnt), odom_dist))
    for k in sorted(cnt):
        print('    %-26s %6d' % (k, cnt[k]))

    # ---------- 2 位姿 ----------
    print('=' * 62)
    print('2 位姿')
    print('  跳变（filtered 单帧 > %.2f m）：%d 次' % (JUMP_M, len(jumps)))
    for rel, d in jumps[:10]:
        print('    %7.1f s  %.3f m' % (rel, d))
    if div:
        ds = sorted(x[1] for x in div)
        print('  odom 与 filtered 的逐步分歧：中位 %.3f m，95%% %.3f m，最大 %.3f m'
              % (ds[len(ds)//2], ds[int(len(ds)*0.95)], ds[-1]))
        bad = [x for x in div if x[1] > 0.10]
        print('    分歧 > 0.10 m 的帧：%d（%.1f%%）' % (len(bad), 100.0*len(bad)/len(div)))

    # ---------- 3 建图 ----------
    print('=' * 62)
    print('3 建图（末帧）')
    if grid:
        w, h, d = grid
        occ_n, iso_r = isolated_ratio(grid, w, h, d)
        known = occ + idle
        print('  尺寸 %dx%d，占用 %d，空闲 %d，未知 %d' % (w, h, occ, idle, unk))
        print('  已知率 %.1f%%，孤立障碍 %.1f%%' % (
            100.0 * known / max(1, len(d)), 100.0 * iso_r))
    else:
        print('  包里没有 /map')

    # ---------- 4 门控 ----------
    print('=' * 62)
    print('4 建图门控')
    if scan_n:
        print('  /scan %d 帧 → /scan_safe %d 帧，丢弃 %.1f%%' % (
            scan_n, safe_n, 100.0 * (scan_n - safe_n) / scan_n))
    tot = map_true + map_false
    if tot:
        print('  map_allowed：true %d / false %d（%.1f%%）' % (
            map_true, map_false, 100.0 * map_false / tot))

    # ---------- 5 激光速度 ----------
    print('=' * 62)
    print('5 激光速度（/scan_match/log_only）')
    if icp_sig:
        good = sum(1 for s in icp_sig if s <= 0.03)
        mid = sum(1 for s in icp_sig if 0.03 < s <= 0.10)
        bad = len(icp_sig) - good - mid
        print('  观测 %d 条，σ 中位 %.3f m' % (len(icp_sig),
                                            sorted(icp_sig)[len(icp_sig)//2]))
        print('  好(≤0.03) %.1f%% / 一般 %.1f%% / 差(>0.10) %.1f%%' % (
            100.0*good/len(icp_sig), 100.0*mid/len(icp_sig), 100.0*bad/len(icp_sig)))
    else:
        print('  没有观测（没起 scan_match 或全被丢）')

    # ---------- 6 恢复 ----------
    print('=' * 62)
    print('6 恢复动作（/slip/report）')
    if reports:
        ok = sum(1 for x in reports if x.get('ok'))
        print('  共 %d 个动作，成功 %d，失败 %d' % (len(reports), ok, len(reports)-ok))
        for x in reports:
            print('    id=%s %-6s ok=%-5s moved=%.3f why=%s' % (
                x.get('id'), x.get('kind'), x.get('ok'),
                float(x.get('moved') or 0.0), x.get('why')))
    else:
        print('  没有动作')

    # ---------- 7 分段 ----------
    print('=' * 62)
    print('7 分段（/bag_marker）')
    if markers:
        for rel, name in markers:
            print('    %7.1f s  %s' % (rel, name))
    else:
        print('  没有标记（录的时候没跑 mark_segment.sh）')


if __name__ == '__main__':
    sys.exit(main())
