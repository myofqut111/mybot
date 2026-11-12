#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""网页控制台的纯逻辑部分：不 import rclpy，可在本机直接跑单测。

这里只做"把 ROS 数据整理成前端好用的形状"，不做任何 IO。
所有函数对缺字段 / 脏数据都取"尽力而为"策略：给 None 或跳过，绝不抛异常
——一个坏掉的报告文件不应该让整个看板 500。
"""
import json
import os
import time

# 执行器会出现的状态（见 node.py 的 _publish_status 调用点）
KNOWN_STATES = (
    'IDLE', 'LOADING_ROUTE', 'RUNNING', 'NAVIGATING', 'WAITING_TASK',
    'PAUSED', 'PREEMPTING', 'SUCCEEDED', 'FAILED', 'CANCELED',
)

TERMINAL_STATES = ('SUCCEEDED', 'FAILED', 'CANCELED')

BATTERY_UNKNOWN = 'unknown'


def battery_band(volts, warn_v=11.8, crit_v=11.5):
    """电压 -> 'ok' / 'warn' / 'crit' / 'unknown'。

    与执行器 battery.py 的分档保持一致：<=crit 才算 crit，<=warn 才算 warn。
    None、非数字、负数一律当 unknown（不拿坏数据做决策，也不刷红）。
    """
    try:
        value = float(volts)
    except (TypeError, ValueError):
        return BATTERY_UNKNOWN
    if value != value or value <= 0.0:      # NaN 或 0/负值
        return BATTERY_UNKNOWN
    if value <= float(crit_v):
        return 'crit'
    if value <= float(warn_v):
        return 'warn'
    return 'ok'


def _as_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if out != out:
        return None
    return out


def normalize_status(payload, now=None, warn_v=11.8, crit_v=11.5):
    """执行器状态 JSON -> 前端形状。

    输入可以是 dict 或 JSON 字符串；解析失败给一个空壳（state=None），
    让页面显示"未连接"而不是崩掉。
    """
    now = time.time() if now is None else float(now)
    data = payload
    if isinstance(data, (str, bytes)):
        try:
            data = json.loads(data)
        except (ValueError, TypeError):
            data = None
    if not isinstance(data, dict):
        data = {}

    stamp = _as_float(data.get('time'))
    volts = _as_float(data.get('battery_v'))
    if volts is None:
        volts = _as_float(data.get('battery_voltage'))
    state = data.get('state')
    return {
        'connected': bool(data),
        'state': str(state) if state else None,
        'known_state': state in KNOWN_STATES,
        'terminal': state in TERMINAL_STATES,
        'message': str(data.get('message') or ''),
        'execution_id': str(data.get('execution_id') or ''),
        'route_name': str(data.get('route_name') or ''),
        'point_id': str(data.get('point_id') or ''),
        'current_index': _as_int(data.get('current_index')),
        'completed_points': _as_int(data.get('completed_points')),
        'current_loop': _as_int(data.get('current_loop')),
        'battery_v': volts,
        'battery_band': battery_band(volts, warn_v, crit_v),
        'age_s': (None if stamp is None else max(0.0, now - stamp)),
    }


def normalize_result(payload):
    """PatrolTaskResult 摘要 -> 前端一行。字段缺失就填空，不抛异常。"""
    if not isinstance(payload, dict):
        return {'point_id': '', 'status': None, 'message': ''}
    status = payload.get('status')
    return {
        'execution_id': str(payload.get('execution_id') or ''),
        'point_id': str(payload.get('point_id') or ''),
        'point_index': _as_int(payload.get('point_index')),
        'task_id': str(payload.get('task_id') or ''),
        'status': _as_int(status, None) if status is not None else None,
        'message': str(payload.get('message') or ''),
        'received_at': _as_float(payload.get('received_at')),
    }


def summarize_run(summary, fallback_name=''):
    """单轮 summary.json -> 历史表格一行。

    兼容"字段缺失"和"老版本没有 verdicts"两种情况：
    比率类字段保持 None（前端显示 —），不要填 0。
    """
    data = summary if isinstance(summary, dict) else {}
    points = data.get('points')
    if not isinstance(points, dict):
        points = {}
    ok = _as_int(points.get('SUCCEEDED'))
    skipped = _as_int(points.get('SKIPPED'))
    failed = _as_int(points.get('FAILED'))
    total = ok + skipped + failed
    verdicts = data.get('verdicts')
    if not isinstance(verdicts, dict):
        verdicts = {}
    graded = _as_int(data.get('graded_points'),
                     sum(_as_int(v) for v in verdicts.values()))
    return {
        'name': str(data.get('execution_id') or fallback_name),
        'route_name': str(data.get('route_name') or ''),
        'mode': data.get('mode'),
        'started_at': _as_float(data.get('started_at')),
        'duration_sec': _as_float(data.get('duration_sec')),
        'total': total,
        'ok': ok,
        'skipped': skipped,
        'failed': failed,
        'detections_total': _as_int(data.get('detections_total')),
        'graded_points': graded,
        'recall': _as_float(data.get('recall')),
        'precision': _as_float(data.get('precision')),
        'false_alarm_rate': _as_float(data.get('false_alarm_rate')),
        'battery_v': _as_float(data.get('battery_v')),
    }


def list_runs(reports_dir, limit=50):
    """扫 ~/patrol_reports/*.summary.json，按修改时间倒序，最多 limit 条。"""
    out = []
    try:
        names = [n for n in os.listdir(reports_dir) if n.endswith('.summary.json')]
    except OSError:
        return out
    entries = []
    for name in names:
        path = os.path.join(reports_dir, name)
        try:
            entries.append((os.path.getmtime(path), path))
        except OSError:
            continue
    entries.sort(reverse=True)
    for _, path in entries[:limit]:
        summary = read_json(path)
        row = summarize_run(summary, os.path.basename(path)[:36])
        row['path'] = path
        try:
            row['mtime'] = os.path.getmtime(path)
        except OSError:
            row['mtime'] = None
        out.append(row)
    return out


def read_json(path):
    """读 JSON，坏文件返回 {}（不抛）。"""
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def tail_events(path, limit=200):
    """取最后 limit 条**有效**事件；坏行跳过，不占额度。

    边界用 deque 的语义手工实现，避免先把整个文件读进内存 ——
    跑过几百轮之后 jsonl 可能有几万行。
    """
    if limit <= 0:
        return []
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
                if not isinstance(item, dict):
                    continue
                events.append(item)
                if len(events) > limit:
                    del events[0]
    except OSError:
        return []
    return events


def fmt_duration(seconds):
    """秒 -> '1h23m' / '3m20s' / '45s'；None -> '—'。"""
    if seconds is None:
        return '—'
    try:
        total = int(round(float(seconds)))
    except (TypeError, ValueError):
        return '—'
    if total < 0:
        return '—'
    if total < 60:
        return '%ds' % total
    if total < 3600:
        return '%dm%02ds' % (total // 60, total % 60)
    return '%dh%02dm' % (total // 3600, (total % 3600) // 60)


def fmt_rate(value):
    """比率 -> '92.3%'；None -> '—'。"""
    number = _as_float(value)
    if number is None:
        return '—'
    return '%.1f%%' % (100.0 * number)


def event_line(event):
    """事件 dict -> 一行人类可读文本（给日志面板用）。"""
    if not isinstance(event, dict):
        return ''
    kind = str(event.get('type') or event.get('event') or 'event')
    point = str(event.get('point_id') or '')
    message = str(event.get('message') or '')
    status = event.get('status')
    head = kind
    if point:
        head = '%s %s' % (point, kind)
    if status:
        head = '%s [%s]' % (head, status)
    return '%s %s' % (head, message) if message else head
