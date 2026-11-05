#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制台的纯逻辑：状态归一化、日志缓冲、报告读取、坐标换算。

这里**不 import rclpy / cv2 / numpy**，全部可以在本机直接跑单测。
凡是"可能缺字段、可能是脏数据"的地方一律给默认值，绝不抛异常
—— 一个坏掉的消息不该让整个控制台 500。
"""
import json
import os
import time

BATTERY_UNKNOWN = 'unknown'

# 执行器会发布的状态（见 fishbot_patrol_v2/node.py）
RUNNING_STATES = ('LOADING_ROUTE', 'RUNNING', 'NAVIGATING',
                  'WAITING_TASK', 'PAUSED', 'PREEMPTING')
TERMINAL_STATES = ('SUCCEEDED', 'FAILED', 'CANCELED')

# rcl_interfaces/msg/Log 的 level 常量
LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


def battery_band(volts, warn_v=11.8, crit_v=11.5):
    """电压 -> ok / warn / crit / unknown；坏数据一律 unknown。

    契约：`crit_v` 必须**低于** `warn_v` 才算有效。如果调用方只改了一个参数
    导致 crit >= warn（例如只把 warn 调到 10.8），就视为"只配了告警档"，
    crit 档关闭 —— 否则会出现"10.5 V 低于 warn 10.8 却被判成 critical"
    这种自相矛盾、还会把页面上唯一的告警灯弄红的结论。
    """
    try:
        value = float(volts)
    except (TypeError, ValueError):
        return BATTERY_UNKNOWN
    if value != value or value <= 0.0:
        return BATTERY_UNKNOWN
    warn = float(warn_v)
    crit = float(crit_v)
    if crit >= warn:
        crit = None
    if crit is not None and value <= crit:
        return 'crit'
    if value <= warn:
        return 'warn'
    return 'ok'


def _int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return None if out != out else out


def normalize_status(payload, now=None, warn_v=11.8, crit_v=11.5):
    """执行器状态 JSON（dict 或字符串）-> 前端形状。"""
    now = time.time() if now is None else float(now)
    data = payload
    if isinstance(data, (str, bytes)):
        try:
            data = json.loads(data)
        except (ValueError, TypeError):
            data = None
    if not isinstance(data, dict):
        data = {}
    stamp = _float(data.get('time'))
    volts = _float(data.get('battery_v'))
    if volts is None:
        volts = _float(data.get('battery_voltage'))
    state = data.get('state')
    return {
        'connected': bool(data),
        'state': str(state) if state else None,
        'terminal': state in TERMINAL_STATES,
        'message': str(data.get('message') or ''),
        'execution_id': str(data.get('execution_id') or ''),
        'route_name': str(data.get('route_name') or ''),
        'point_id': str(data.get('point_id') or ''),
        'current_index': _int(data.get('current_index')),
        'completed_points': _int(data.get('completed_points')),
        'current_loop': _int(data.get('current_loop')),
        'battery_v': volts,
        'battery_band': battery_band(volts, warn_v, crit_v),
        'age_s': (None if stamp is None else max(0.0, now - stamp)),
        'running': state in RUNNING_STATES,
    }


# ---------------------------------------------------------------- 日志
class LogRing(object):
    """定长环形缓冲，按级别计数。

    为什么要自己数：/rosout 里 ERROR 常常被 INFO 淹没，页面上需要一个
    "现在有几条错误"的数字，而不是让人往上翻。
    """

    def __init__(self, capacity=400):
        self.capacity = max(1, int(capacity))
        self._items = []
        self._seq = 0
        self._counts = {'DEBUG': 0, 'INFO': 0, 'WARN': 0,
                        'ERROR': 0, 'FATAL': 0}

    def clear_counts(self):
        for key in self._counts:
            self._counts[key] = 0

    def counts(self):
        return dict(self._counts)

    def add(self, node, level, text, stamp=None):
        """level 可以是 rclpy 的整数，也可以是 'ERROR' 这类字符串。"""
        if isinstance(level, int):
            name = LOG_LEVELS.get(level) or (
                'ERROR' if level >= 40 else 'INFO')
        else:
            name = str(level).upper()
        if name not in self._counts:
            name = 'INFO'
        self._seq += 1
        item = {
            'seq': self._seq,
            'time': time.strftime('%H:%M:%S',
                                  time.localtime(stamp if stamp else time.time())),
            'node': str(node or ''),
            'level': name,
            'text': str(text or ''),
        }
        self._items.append(item)
        self._counts[name] += 1
        if len(self._items) > self.capacity:
            del self._items[0]
        return item

    def tail(self, limit=200, min_level=None):
        """返回最近 limit 条；min_level 给 'WARN' 就只回 WARN/ERROR/FATAL。"""
        items = self._items
        if min_level:
            order = ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
            try:
                floor = order.index(str(min_level).upper())
            except ValueError:
                floor = 0
            items = [i for i in items if order.index(i['level']) >= floor]
        if limit and limit > 0:
            items = items[-int(limit):]
        return list(items)


# ---------------------------------------------------------------- 位姿
def pose_age(pose, now=None):
    """位姿的年龄（秒）。

    pose 是 `(x, y, yaw, 时间戳)` 四元组——**时间戳在下标 3，不是 2**。
    这个下标已经写错过两次：把 yaw 当时间戳，算出约 1.8e9 秒，"定位"灯
    永远是红的、页面看着像坏了。收成一个函数，只此一处取下标。
    """
    if not pose:
        return None
    try:
        stamp = float(pose[3])
    except (IndexError, KeyError, TypeError, ValueError):
        return None
    if now is None:
        now = time.time()
    return max(0.0, float(now) - stamp)


# ---------------------------------------------------------------- 报告
def read_json(path):
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def summarize_run(summary, fallback_name=''):
    """单轮 summary.json -> 历史表格一行；比率缺失保持 None（前端显示 —）。"""
    data = summary if isinstance(summary, dict) else {}
    points = data.get('points')
    if not isinstance(points, dict):
        points = {}
    ok = _int(points.get('SUCCEEDED'))
    skipped = _int(points.get('SKIPPED'))
    failed = _int(points.get('FAILED'))
    verdicts = data.get('verdicts')
    if not isinstance(verdicts, dict):
        verdicts = {}
    return {
        'name': str(data.get('execution_id') or fallback_name),
        'route_name': str(data.get('route_name') or ''),
        'mode': data.get('mode'),
        'started_at': _float(data.get('started_at')),
        'duration_sec': _float(data.get('duration_sec')),
        'total': ok + skipped + failed,
        'ok': ok,
        'skipped': skipped,
        'failed': failed,
        'detections_total': _int(data.get('detections_total')),
        'graded_points': _int(data.get('graded_points'),
                              sum(_int(v) for v in verdicts.values())),
        'recall': _float(data.get('recall')),
        'precision': _float(data.get('precision')),
        'false_alarm_rate': _float(data.get('false_alarm_rate')),
        'battery_v': _float(data.get('battery_v')),
    }


def list_runs(reports_dir, limit=50):
    try:
        names = [n for n in os.listdir(reports_dir)
                 if n.endswith('.summary.json')]
    except OSError:
        return []
    entries = []
    for name in names:
        path = os.path.join(reports_dir, name)
        try:
            entries.append((os.path.getmtime(path), path))
        except OSError:
            continue
    entries.sort(reverse=True)
    out = []
    for mtime, path in entries[:max(0, int(limit))]:
        row = summarize_run(read_json(path), os.path.basename(path)[:36])
        row['path'] = path
        row['mtime'] = mtime
        out.append(row)
    return out


def tail_events(path, limit=200):
    """最后 limit 条有效事件；坏行跳过，不占额度。"""
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


# ---------------------------------------------------------------- 格式化
def fmt_duration(seconds):
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
    number = _float(value)
    return '—' if number is None else '%.1f%%' % (100.0 * number)


def event_line(event):
    if not isinstance(event, dict):
        return ''
    kind = str(event.get('type') or event.get('event') or 'event')
    point = str(event.get('point_id') or '')
    status = event.get('status')
    message = str(event.get('message') or '')
    head = ('%s %s' % (point, kind)) if point else kind
    if status:
        head = '%s [%s]' % (head, status)
    return '%s %s' % (head, message) if message else head

def teleop_step(vx, wz, seen, now, timeout):
    """决定这一拍要不要发 Twist，以及发什么。

    返回 (send, out_vx, out_wz, new_vx, new_wz)。

    规则（这就是全部）：
    - 按住方向键（seen 新且速度为非零）→ 每拍都发；
    - 松手/心跳超时且之前有速度 → **只补一帧零速**，随后彻底安静；
    - 空闲（速度为 0）→ **一拍都不发**，把话题让给 Nav2。

    最后一条是关键：原来空闲也发 0，和 Nav2 的指令交替，车就抖。
    """
    try:
        vx = float(vx)
        wz = float(wz)
        seen = float(seen)
        now = float(now)
        timeout = float(timeout)
    except (TypeError, ValueError):
        return False, 0.0, 0.0, 0.0, 0.0
    moving = bool(abs(vx) > 1e-9 or abs(wz) > 1e-9)
    stale = (now - seen) > timeout
    if stale:
        if moving:
            # 松手后的最后一帧零速：让底盘知道“停”，之后不再占用话题
            return True, 0.0, 0.0, 0.0, 0.0
        return False, 0.0, 0.0, 0.0, 0.0
    if moving:
        return True, vx, wz, vx, wz
    return False, 0.0, 0.0, vx, wz

def detect_lan_ip(timeout=0.3):
    """探测本机在局域网里的地址。

    用 UDP connect 而不是 getaddrinfo(gethostname())：后者在 Ubuntu 上常返回
    127.0.1.1，拿不到真实地址。UDP connect 不发包，只是让内核选一张出口网卡。
    """
    import socket
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        sock.connect(('8.8.8.8', 80))
        return sock.getsockname()[0]
    except Exception:
        return ''
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


def public_host(param_value, bind_host, detect=None):
    """决定“给别人看的地址”。

    优先级：参数 public_host > 绑定地址（非 0.0.0.0 时）> 自动探测 > 127.0.0.1。
    绑定地址保留 0.0.0.0（本机和局域网都能连），但 0.0.0.0 不是能点开的地址，
    所以日志里必须换成一个真实 IP，否则用户只能看到“把 0.0.0.0 换成机器人 IP”
    这种要自己动手的提示。
    """
    value = (param_value or '').strip()
    if value:
        return value
    bind = (bind_host or '').strip()
    if bind and bind not in ('0.0.0.0', '::', '[::]'):
        return bind
    found = (detect or detect_lan_ip)()
    if found:
        return found
    return bind or '127.0.0.1'

