#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把多次巡检的 jsonl 报告汇总成一张表——用来回答「跑得稳不稳」。

每次巡检执行器都会在 ``~/patrol_reports`` 下落一份
``patrol_<时间>_<exec>.jsonl``，每个点位一行，含 nav_seconds / task_seconds /
status / detections。这个脚本只读这些文件，不依赖 ROS，在哪都能跑。

用法：
    python3 summarize_runs.py                     # 默认读 ~/patrol_reports
    python3 summarize_runs.py --dir /path/to/dir
    python3 summarize_runs.py --last 4            # 只看最近 4 次
    python3 summarize_runs.py --csv runs.csv      # 另存一份 CSV

输出两段：
  1) 每次运行一行（点位/成功/失败/成功率/总时长/单点耗时 P50、P95/检出总数）
  2) 全部运行合起来的总体指标
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import datetime
from typing import Dict, List, Optional

STATUS_ZH = {
    'ok': '成功',
    'skipped': '跳过',
    'failed': '失败',
    'canceled': '取消',
}


def percentile(values: List[float], ratio: float) -> float:
    """线性插值分位数；空列表返回 0。"""
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * ratio
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def load_run(path: str) -> Optional[dict]:
    """读一份 jsonl，返回这次运行的摘要；读不动就返回 None。"""
    route = ''
    points: List[dict] = []
    try:
        with open(path, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except ValueError:
                    continue
                kind = record.get('type')
                if kind == 'event' and not route:
                    route = str(record.get('route_name', ''))
                elif kind == 'point':
                    points.append(record)
    except OSError as error:
        print('读不了 %s：%s' % (path, error), file=sys.stderr)
        return None

    if not points:
        return None

    times = [p.get('time', '') for p in points if p.get('time')]
    latencies = [float(p.get('nav_seconds', 0.0)) +
                 float(p.get('task_seconds', 0.0)) for p in points]
    counts: Dict[str, int] = {}
    detections = 0
    for point in points:
        status = str(point.get('status', 'unknown'))
        counts[status] = counts.get(status, 0) + 1
        detections += len(point.get('detections') or [])

    return {
        'file': os.path.basename(path),
        'route': route,
        'points': len(points),
        'counts': counts,
        'ok': counts.get('ok', 0),
        'latencies': latencies,
        'detections': detections,
        'start': min(times) if times else '',
        'end': max(times) if times else '',
    }


def run_seconds(run: dict) -> float:
    """总时长：用记录里的 ISO 时间首尾相减；解析失败返回 0。"""
    start, end = run['start'], run['end']
    if not start or not end:
        return 0.0
    try:
        fmt = '%Y-%m-%dT%H:%M:%S'
        begin = datetime.strptime(start[:19], fmt)
        finish = datetime.strptime(end[:19], fmt)
        return max(0.0, (finish - begin).total_seconds())
    except ValueError:
        return 0.0


def print_table(runs: List[dict]) -> None:
    header = ('#', '开始时间', '路线', '点位', '成功', '失败', '跳过',
              '成功率', '总时长', '单点P50', '单点P95', '检出')
    rows = [header]
    for index, run in enumerate(runs, 1):
        total = run['points']
        ok = run['ok']
        rows.append((
            str(index),
            (run['start'][5:19].replace('T', ' ') or '-'),
            run['route'] or '-',
            str(total),
            str(ok),
            str(run['counts'].get('failed', 0)),
            str(run['counts'].get('skipped', 0)),
            '%.0f%%' % (100.0 * ok / total) if total else '-',
            '%.0fs' % run_seconds(run),
            '%.1fs' % percentile(run['latencies'], 0.5),
            '%.1fs' % percentile(run['latencies'], 0.95),
            str(run['detections']),
        ))

    widths = [max(display_width(row[i]) for row in rows)
              for i in range(len(header))]
    for position, row in enumerate(rows):
        line = '  '.join(pad(cell, widths[i]) for i, cell in enumerate(row))
        print(line.rstrip())
        if position == 0:
            print('  '.join('─' * width for width in widths))


def display_width(text: str) -> int:
    return sum(2 if ord(char) > 0x2E80 else 1 for char in text)


def pad(text: str, width: int) -> str:
    return text + ' ' * max(0, width - display_width(text))


def main() -> int:
    parser = argparse.ArgumentParser(
        description='汇总多次巡检的 jsonl 报告')
    parser.add_argument('--dir', default='~/patrol_reports',
                        help='报告目录，默认 ~/patrol_reports')
    parser.add_argument('--last', type=int, default=0,
                        help='只看最近 N 次（0 = 全部）')
    parser.add_argument('--csv', default='', help='另存一份 CSV')
    args = parser.parse_args()

    directory = os.path.expanduser(args.dir)
    if not os.path.isdir(directory):
        print('目录不存在：%s' % directory, file=sys.stderr)
        return 2

    files = sorted(name for name in os.listdir(directory)
                   if name.endswith('.jsonl'))
    if not files:
        print('目录里没有 .jsonl：%s' % directory, file=sys.stderr)
        return 2
    if args.last > 0:
        files = files[-args.last:]

    runs = [run for run in (load_run(os.path.join(directory, name))
                            for name in files) if run]
    if not runs:
        print('没有可解析的运行记录', file=sys.stderr)
        return 2

    print('报告目录：%s（%d 次运行）\n' % (directory, len(runs)))
    print_table(runs)

    total_points = sum(run['points'] for run in runs)
    total_ok = sum(run['ok'] for run in runs)
    all_latencies = [value for run in runs for value in run['latencies']]
    total_det = sum(run['detections'] for run in runs)
    perfect = sum(1 for run in runs if run['ok'] == run['points'])

    print('\n总体：%d 次运行 / %d 个点位访问，成功 %d（%.1f%%），'
          '整轮零失败的运行 %d/%d'
          % (len(runs), total_points, total_ok,
             100.0 * total_ok / total_points if total_points else 0.0,
             perfect, len(runs)))
    print('     单点耗时 P50 %.1fs ｜ P95 %.1fs ｜ 平均 %.1fs'
          % (percentile(all_latencies, 0.5),
             percentile(all_latencies, 0.95),
             sum(all_latencies) / len(all_latencies) if all_latencies else 0.0))
    print('     检出目标 %d 次（平均每点位 %.2f 个）'
          % (total_det, total_det / total_points if total_points else 0.0))

    skipped = {}
    for run in runs:
        for status, count in run['counts'].items():
            if status != 'ok':
                skipped[status] = skipped.get(status, 0) + count
    if skipped:
        print('     非成功：' + '，'.join(
            '%s %d' % (STATUS_ZH.get(key, key), value)
            for key, value in sorted(skipped.items())))

    if args.csv:
        with open(args.csv, 'w', encoding='utf-8', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['file', 'route', 'start', 'points', 'ok',
                             'failed', 'skipped', 'success_rate',
                             'seconds', 'p50_s', 'p95_s', 'detections'])
            for run in runs:
                total = run['points'] or 1
                writer.writerow([
                    run['file'], run['route'], run['start'], run['points'],
                    run['ok'], run['counts'].get('failed', 0),
                    run['counts'].get('skipped', 0),
                    round(100.0 * run['ok'] / total, 1),
                    round(run_seconds(run), 1),
                    round(percentile(run['latencies'], 0.5), 2),
                    round(percentile(run['latencies'], 0.95), 2),
                    run['detections'],
                ])
        print('\nCSV：%s' % args.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
