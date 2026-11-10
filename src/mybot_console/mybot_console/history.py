# -*- coding: utf-8 -*-
"""点位历史：把一堆已经落盘的 jsonl 报告，按"路线 / 点位"折叠成时间线。

只读报告目录，不碰 ROS、不建数据库——历史本来就写在 jsonl 里，再存一份
SQLite 反而多一个会不同步的真源。文件都很小（一轮 15 点约 20 KB），
按 mtime 从新到旧读到够用为止即可。

输出给页面用：
    {
      'ok': True, 'days': 30, 'runs': 12, 'routes': ['main2'],
      'points': [
        {'route': 'main2', 'point': 'p1', 'runs': 12,
         'missing': 1, 'false_alarm': 0, 'unknown': 0,
         'last_time': '2026-11-06T16:49:00',
         'samples': [{'time': ..., 'run': '013b39eb', 'verdict': 'tp',
                      'detected': True, 'status': 'SUCCEEDED', 'task': 'inspect'}]}
      ]
    }
"""
import datetime
import json
import os

MAX_FILES = 300          # 最多扫多少个 jsonl（按 mtime 从新到旧）
PER_POINT_SAMPLES = 20   # 每个点位保留最近多少次
VERDICTS = ('tp', 'tn', 'fp', 'fn')


def _now():
    return datetime.datetime.now()


def _parse_time(text, fallback):
    """jsonl 里的 time 是本地 ISO 字符串；解析不了就退回文件 mtime。"""
    raw = str(text or '').strip()
    if raw:
        try:
            stamp = datetime.datetime.fromisoformat(raw.replace('Z', ''))
            return stamp.replace(tzinfo=None) if stamp.tzinfo else stamp
        except ValueError:
            pass
    try:
        return datetime.datetime.fromtimestamp(float(fallback))
    except (TypeError, ValueError, OSError):
        return None


def _read_events(path):
    """逐行读 jsonl，坏行跳过（报告写到一半被 kill 时最后一行常常是残的）。"""
    events = []
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    item = json.loads(line)
                except ValueError:
                    continue
                if isinstance(item, dict):
                    events.append(item)
    except OSError:
        return []
    return events


def run_files(reports_dir, limit=MAX_FILES):
    """报告目录里的 jsonl，按 mtime 从新到旧。进行中的那一份也在这儿。"""
    try:
        names = os.listdir(reports_dir)
    except OSError:
        return []
    rows = []
    for name in names:
        if not name.endswith('.jsonl'):
            continue
        path = os.path.join(reports_dir, name)
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        rows.append((mtime, path))
    rows.sort(reverse=True)
    return rows[:max(0, int(limit))]


def _detected(item):
    dets = item.get('detections')
    return bool(dets) if isinstance(dets, (list, tuple)) else False


def point_history(reports_dir, days=30, route='', per_point=PER_POINT_SAMPLES,
                  limit=MAX_FILES, now=None):
    """按点位折叠历史。

    ``days`` 只看最近几天（按报告里写的运行时间算，解析不了用文件 mtime）；
    ``route`` 非空则只留这条路线。同一个点位在**同一轮**里可能出现多次
    （重试），只取第一次出结果的那次，避免一次失败把曲线砸出好几个坑。
    """
    try:
        days = max(0, int(days))
    except (TypeError, ValueError):
        days = 30
    route = str(route or '').strip()
    cutoff = None
    if days:
        cutoff = (now or _now()) - datetime.timedelta(days=days)

    buckets = {}
    run_count = 0
    for mtime, path in run_files(reports_dir, limit=limit):
        events = _read_events(path)
        if not events:
            continue
        header = {}
        for item in events:
            if item.get('type') == 'header':
                header = item
                break
        stamp = _parse_time(header.get('time'), mtime)
        if cutoff is not None and stamp is not None and stamp < cutoff:
            continue
        name = str(header.get('execution_id') or os.path.basename(path)[:-len('.jsonl')])
        this_route = str(header.get('route_name') or '')
        if route and this_route != route:
            continue
        run_count += 1
        seen = set()
        for item in events:
            if item.get('type') != 'point':
                continue
            point_id = str(item.get('point_id') or '')
            if not point_id or point_id in seen:
                continue
            seen.add(point_id)
            expect = str(item.get('expect') or '')
            verdict = str(item.get('verdict') or '') if expect else ''
            if verdict and verdict not in VERDICTS:
                verdict = ''
            sample = {
                'time': stamp.strftime('%Y-%m-%dT%H:%M:%S') if stamp else '',
                'run': name[:8],
                'verdict': verdict,
                'detected': _detected(item),
                'status': str(item.get('status') or ''),
                'task': str(item.get('task_id') or ''),
            }
            key = (this_route, point_id)
            buckets.setdefault(key, []).append(sample)

    points = []
    routes = set()
    for (this_route, point_id), samples in buckets.items():
        samples.sort(key=lambda row: (row['time'], row['run']), reverse=True)
        routes.add(this_route)
        missing = sum(1 for row in samples if row['verdict'] == 'fn')
        false_alarm = sum(1 for row in samples if row['verdict'] == 'fp')
        unknown = sum(1 for row in samples if not row['verdict'])
        points.append({
            'route': this_route,
            'point': point_id,
            'runs': len(samples),
            'missing': missing,
            'false_alarm': false_alarm,
            'unknown': unknown,
            'last_time': samples[0]['time'] if samples else '',
            'samples': samples[:max(1, int(per_point))],
        })
    # 有缺失的排前面，其次按最近时间——报告页第一眼想看的正是"哪个点老出问题"
    points.sort(key=lambda row: (-row['missing'], -row['false_alarm'],
                                 row['route'], row['point']))
    return {
        'ok': True,
        'days': days,
        'runs': run_count,
        'routes': sorted(routes),
        'points': points,
    }
