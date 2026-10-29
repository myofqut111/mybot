#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检网页控制台：一个进程 = 一个 ROS 节点 + 一个 HTTP 服务。

为什么不用 rosbridge / Flask / FastAPI：
  1. rosbridge 要额外装包，而且网页里得手搓 rosbridge 协议；
  2. Flask/FastAPI 在离线机器上不一定有，还得管依赖版本；
  3. 只需要"几个按钮 + 一个状态面板"，标准库的 http.server 完全够，
     而且 ROS 回调在 rclpy 线程里，HTTP 线程只读快照，不需要额外同步。

启动：
    ros2 run fishbot_patrol_v2 patrol_console
    浏览器打开 http://<本机IP>:8080
"""
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import Float32, String

from fishbot_patrol_interfaces.action import ExecutePatrol
from fishbot_patrol_interfaces.msg import PatrolTaskResult
from fishbot_patrol_interfaces.srv import (
    PausePatrol, RecordPoint, ResumePatrol,
)

from . import console_data as cd
from .console_page import PAGE_HTML

RUNNING_STATES = ('LOADING_ROUTE', 'RUNNING', 'NAVIGATING',
                  'WAITING_TASK', 'PAUSED', 'PREEMPTING')

# 只允许读 reports 目录下的 .jsonl，防止 /api/events 变成任意文件读取
EVENT_SUFFIX = '.jsonl'
MAX_EVENTS = 80


class ConsoleNode(Node):
    """把 ROS 侧的东西收在一处：订阅、动作、服务，都对外暴露成普通方法。"""

    def __init__(self):
        super().__init__('patrol_console')
        self.declare_parameter('status_topic', '/patrol/execution/status')
        self.declare_parameter('result_topic', '/patrol/task/result')
        self.declare_parameter('battery_topic', '/battery/voltage')
        self.declare_parameter('action_name', '/execute_patrol')
        self.declare_parameter('reports_dir', '~/patrol_reports')
        self.declare_parameter('battery_warn_v', 11.8)
        self.declare_parameter('battery_crit_v', 11.5)

        self._warn_v = float(self.get_parameter('battery_warn_v').value)
        self._crit_v = float(self.get_parameter('battery_crit_v').value)
        self._reports_dir = os.path.expanduser(
            str(self.get_parameter('reports_dir').value))

        self._lock = threading.Lock()
        self._status = {}
        self._status_at = 0.0
        self._battery = None
        self._battery_at = 0.0
        self._events = deque(maxlen=MAX_EVENTS)
        self._last_detection = ''
        self._goal_handle = None
        self._busy = False
        self._feedback = {}

        self._group = ReentrantCallbackGroup()

        # 执行器状态是 latched（TRANSIENT_LOCAL），这里必须同样匹配，
        # 否则后启动的控制台永远收不到"当前状态"。
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, str(self.get_parameter('status_topic').value),
            self._on_status, latched, callback_group=self._group)
        self.create_subscription(
            PatrolTaskResult, str(self.get_parameter('result_topic').value),
            self._on_result, 20, callback_group=self._group)
        self.create_subscription(
            Float32, str(self.get_parameter('battery_topic').value),
            self._on_battery, 10, callback_group=self._group)

        self._action = ActionClient(
            self, ExecutePatrol,
            str(self.get_parameter('action_name').value),
            callback_group=self._group)
        self._pause_client = self.create_client(
            PausePatrol, '/patrol/pause', callback_group=self._group)
        self._resume_client = self.create_client(
            ResumePatrol, '/patrol/resume', callback_group=self._group)
        self._record_client = self.create_client(
            RecordPoint, '/patrol_route/record_point', callback_group=self._group)
        self._preempt_pub = self.create_publisher(String, '/patrol/preempt', 10)

        self.get_logger().info('patrol_console ready (reports=%s)' % self._reports_dir)

    # -- 订阅回调 --------------------------------------------------------
    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._status = data if isinstance(data, dict) else {}
            self._status_at = time.time()

    def _on_result(self, msg):
        try:
            message = msg.message
        except AttributeError:
            message = ''
        if message:
            with self._lock:
                self._last_detection = '%s %s' % (msg.point_id, message)
                self._events.append({
                    'time': time.strftime('%H:%M:%S'),
                    'text': '%s -> %s' % (msg.point_id, message),
                })

    def _on_battery(self, msg):
        with self._lock:
            self._battery = float(msg.data)
            self._battery_at = time.time()

    # -- 对外快照 --------------------------------------------------------
    def snapshot(self):
        with self._lock:
            raw = dict(self._status)
            status_at = self._status_at
            battery = self._battery
            battery_at = self._battery_at
            events = list(self._events)
            detection = self._last_detection
            busy = self._busy
        if battery is not None:
            raw.setdefault('battery_v', battery)
        view = cd.normalize_status(raw, now=time.time(),
                                   warn_v=self._warn_v, crit_v=self._crit_v)
        if battery is not None and time.time() - battery_at > 30.0:
            view['battery_band'] = cd.BATTERY_UNKNOWN
        view['running'] = bool(busy or
                               (view.get('state') in RUNNING_STATES))
        view['events'] = events
        view['detection'] = detection
        view['status_age_s'] = (None if status_at <= 0.0
                                else max(0.0, time.time() - status_at))
        return view

    # -- 动作 / 服务 -----------------------------------------------------
    def start_patrol(self, route, mode, repeat):
        if self._busy:
            return False, '已经有一趟在跑'
        if not self._action.wait_for_server(timeout_sec=1.0):
            return False, '执行器还没上线（/execute_patrol 没有服务端）'
        goal = ExecutePatrol.Goal()
        goal.route_name = str(route)
        goal.mode = int(mode)
        goal.repeat_count = int(repeat)
        self._busy = True
        future = self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return True, '目标已下发：%s / mode=%d / %d 轮' % (route, mode, repeat)

    def _on_feedback(self, feedback):
        try:
            self._feedback = {'distance_remaining': float(feedback.feedback.distance_remaining)}
        except AttributeError:
            pass

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:      # noqa: BLE001 - 任何异常都不能让控制台挂
            self._busy = False
            self._append_event('下发目标失败：%s' % error)
            return
        if not handle.accepted:
            self._busy = False
            self._append_event('执行器拒绝了目标')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._busy = False
        self._goal_handle = None
        try:
            result = future.result().result
        except Exception:               # noqa: BLE001
            self._append_event('本轮结束（拿不到结果）')
            return
        self._append_event('本轮结束：success=%s %s'
                           % (result.success, result.message))

    def _append_event(self, text):
        with self._lock:
            self._events.append({'time': time.strftime('%H:%M:%S'), 'text': text})
        self.get_logger().info(text)

    def _call(self, client, request, timeout=3.0):
        """同步调用一个服务：动作线程在 executor 里，所以这里用 future + wait。"""
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('服务 %s 未上线' % client.srv_name)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError('调用 %s 超时' % client.srv_name)
            time.sleep(0.02)
        return future.result()

    def pause(self):
        result = self._call(self._pause_client, PausePatrol.Request())
        return bool(result.success), str(result.message)

    def resume(self):
        result = self._call(self._resume_client, ResumePatrol.Request())
        return bool(result.success), str(result.message)

    def preempt(self, point_id=''):
        self._preempt_pub.publish(String(data=str(point_id)))
        return True, '已发送抢占'

    def record_point(self, route, point_id, task_id, expect):
        request = RecordPoint.Request()
        request.route_name = str(route)
        request.point_id = str(point_id)
        request.task_id = str(task_id)
        request.expect = str(expect)
        result = self._call(self._record_client, request, timeout=5.0)
        return bool(result.success), str(result.message)

    def reports_dir(self):
        return self._reports_dir


_NODE = None


class Handler(BaseHTTPRequestHandler):
    server_version = 'patrol-console/0.4.0'

    def log_message(self, fmt, *args):     # 静音：不然每秒一条 access log
        return

    # -- 工具 -----------------------------------------------------------
    def _send(self, code, body, content_type='application/json; charset=utf-8'):
        data = body if isinstance(body, bytes) else str(body).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, code, payload):
        self._send(code, json.dumps(payload, ensure_ascii=False))

    def _body(self):
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            length = 0
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return {}

    # -- 路由 -----------------------------------------------------------
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ('/', '/index.html'):
            self._send(200, PAGE_HTML.encode('utf-8'), 'text/html; charset=utf-8')
            return
        if path == '/api/state':
            self._json(200, _NODE.snapshot())
            return
        if path == '/api/runs':
            self._json(200, {'runs': cd.list_runs(_NODE.reports_dir(), limit=50)})
            return
        if path == '/api/events':
            query = parse_qs(parsed.query)
            wanted = (query.get('path') or [''])[0]
            safe = self._safe_event_path(wanted)
            if safe is None:
                self._json(400, {'ok': False, 'error': '路径不在报告目录内'})
                return
            events = cd.tail_events(safe, limit=200)
            self._json(200, {'events': [
                {'text': cd.event_line(item),
                 'time': time.strftime('%H:%M:%S', time.localtime(
                     item.get('time') or time.time()))}
                for item in events]})
            return
        self._json(404, {'ok': False, 'error': 'not found'})

    def _safe_event_path(self, requested):
        """只允许 reports_dir 下的 .jsonl，挡住 ../ 之类。"""
        if not requested:
            return None
        base = os.path.realpath(_NODE.reports_dir())
        target = os.path.realpath(os.path.expanduser(requested))
        if not target.startswith(base + os.sep):
            return None
        if not target.endswith(EVENT_SUFFIX):
            return None
        return target

    def do_POST(self):
        path = urlparse(self.path).path
        body = self._body()
        try:
            if path == '/api/run':
                ok, message = _NODE.start_patrol(
                    body.get('route') or 'main',
                    body.get('mode') or 0,
                    body.get('repeat') or 1)
            elif path == '/api/pause':
                ok, message = _NODE.pause()
            elif path == '/api/resume':
                ok, message = _NODE.resume()
            elif path == '/api/preempt':
                ok, message = _NODE.preempt(body.get('point_id') or '')
            elif path == '/api/record_point':
                ok, message = _NODE.record_point(
                    body.get('route') or 'main',
                    body.get('point_id') or '',
                    body.get('task_id') or 'inspect',
                    body.get('expect') or '')
            else:
                self._json(404, {'ok': False, 'error': 'not found'})
                return
        except Exception as error:      # noqa: BLE001 - 任何异常都回给前端
            self._json(500, {'ok': False, 'error': str(error)})
            return
        self._json(200 if ok else 409, {'ok': ok, 'message': message,
                                        'error': None if ok else message})


def main(args=None):
    global _NODE
    rclpy.init(args=args)
    node = ConsoleNode()
    _NODE = node

    port = 8080
    host = '0.0.0.0'
    try:
        port = int(os.environ.get('PATROL_CONSOLE_PORT', '8080'))
        host = os.environ.get('PATROL_CONSOLE_HOST', '0.0.0.0')
    except ValueError:
        port = 8080

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer((host, port), Handler)
    node.get_logger().info(
        '控制台已启动：http://%s:%d  （手机/其他电脑用本机 IP 访问）'
        % ('127.0.0.1' if host == '0.0.0.0' else host, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
