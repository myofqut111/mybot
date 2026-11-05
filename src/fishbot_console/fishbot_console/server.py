#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTP 服务：把 bridge 的数据变成 JSON / PNG / JPEG，把按钮变成调用。

用标准库 http.server —— 机器人上不需要再装 Flask，而且这点并发量
（一个人、手机 + 电脑）根本用不上多进程。
"""
import errno
import json
import os
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import ai, data, points, stacks
from .page import HTML

# rosbridge 的默认 WebSocket 端口。控制台的 HTTP 端口撞上它时给专门提示——
# 那不是"旧实例没退"，而是同一套 launch 里的 rosbridge 先占住了。
# launch 那边的 ROSBRIDGE_PORT 也是 9090，两处要一起改。
ROSBRIDGE_PORT = 9090


def busy_port_message(port):
    """端口被占时的报错文案。抽成纯函数，好在本机直接测（不需要真占端口）。"""
    hint = ''
    if int(port) == ROSBRIDGE_PORT:
        # 9090 是 rosbridge 的默认端口。控制台的 HTTP 端口撞上它，几乎一定是
        # 启动命令里写了 port:=9090 —— 那不是"旧实例没退"，而是同一套 launch
        # 里的 rosbridge 先占住了。不点明的话，用户会去 pkill 一个不存在的进程。
        hint = ('\n注意：%d 是 rosbridge 的端口，不是控制台的。'
                '控制台请用 8080（默认）或 8090。' % ROSBRIDGE_PORT)
    return ('端口 %d 已被占用，控制台起不来。'
            '先看是谁占着：ss -ltnp | grep \':%d\' ；'
            '如果是上一个控制台没退干净：pkill -f robot_console ；'
            '换端口：ros2 launch fishbot_console console.launch.py '
            'port:=9000%s' % (int(port), int(port), hint))

AI_TIMEOUT = 25.0

def _static_dir():
    """找 static/：优先包目录，退回 share/<pkg>/static。

    --symlink-install 时包目录就是源码目录，直接命中；普通 colcon build 只拷
    .py，static/* 由 setup.py 的 data_files 装到 share 下，所以要有第二条路。
    两条都不在就返回包目录，让请求正常 404（页面会提示地图组件缺失）。
    """
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
    if os.path.isfile(os.path.join(here, 'leaflet.js')):
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(
            get_package_share_directory('fishbot_console'), 'static')
        if os.path.isfile(os.path.join(share, 'leaflet.js')):
            return share
    except Exception:
        pass
    return here


STATIC_DIR = _static_dir()
STATIC_TYPES = {
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.png': 'image/png',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
}


def _serve_static(handler, name):
    """只服务 static/ 目录下的白名单文件（地图组件、图标），拒绝任何路径穿越。"""
    safe = os.path.basename(name or '')
    if not safe or safe.startswith('.'):
        return _json_response(handler, {'error': 'bad name'}, 404)
    path = os.path.join(STATIC_DIR, safe)
    if not os.path.isfile(path):
        return _json_response(handler, {'error': 'not found'}, 404)
    kind = STATIC_TYPES.get(os.path.splitext(safe)[1].lower(),
                            'application/octet-stream')
    try:
        with open(path, 'rb') as handle:
            body = handle.read()
    except OSError:
        return _json_response(handler, {'error': 'read failed'}, 500)
    return _bytes_response(handler, body, kind)



def _json_response(handler, payload, code=200):
    body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
    handler.send_response(code)
    handler.send_header('Content-Type', 'application/json; charset=utf-8')
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Cache-Control', 'no-store')
    handler.end_headers()
    handler.wfile.write(body)


def _bytes_response(handler, body, content_type, cache=False):
    handler.send_response(200)
    handler.send_header('Content-Type', content_type)
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Cache-Control',
                        'max-age=2' if cache else 'no-store')
    handler.end_headers()
    handler.wfile.write(body)


def _log_sink_of(logs):
    """把 stacks 的 `(source, text)` 适配成 `logs.add(source, level, text)`。

    stacks 只传两个参数（它不知道严重级别），这里按行首关键字推断级别；
    并且**吞掉任何异常**——写日志的动作绝不能把"启动业务栈"这种业务搞失败。
    """
    def sink(source, *rest):
        if len(rest) >= 2:
            try:
                level = int(rest[0])
            except (TypeError, ValueError):
                level = 20
            text = rest[1]
        elif rest:
            level = 20
            text = rest[0]
        else:
            level = 20
            text = ''
        try:
            logs.add(str(source), level, str(text))
        except Exception:
            pass
    return sink


class ConsoleServer(object):
    """把 bridge + AI 设置包在一个对象里，handler 通过闭包拿到它。"""

    def __init__(self, bridge, settings=None):
        self.bridge = bridge
        self.settings = settings or {}
        self.httpd = None
        self.port = None            # serve() 之后是**实际**端口（可能被顺延）
        self.skipped_ports = []     # 顺延时跳过的端口，给调用方打日志用
        # 一键启停的管理器。日志直接进同一个环形缓冲，这样"启动失败"和
        # "节点报错"在面板上是一处看，不用两头对。
        self.stacks = stacks.StackManager(
            log_sink=_log_sink_of(bridge.logs),
            with_rviz=bool(self.settings.get('stacks_rviz', True)))

    # ------------------------------------------------------------ 路由
    def make_handler(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, fmt, *args):      # 关掉每请求一行日志
                pass

            # ---------------- GET ----------------
            def do_GET(self):
                parsed = urllib.parse.urlparse(self.path)
                path = parsed.path.rstrip('/') or '/'
                query = urllib.parse.parse_qs(parsed.query)

                if path == '/':
                    return _bytes_response(self, HTML.encode('utf-8'),
                                           'text/html; charset=utf-8')
                if path.startswith('/static/'):
                    return _serve_static(self, path[len('/static/'):])
                if path == '/api/state':
                    return _json_response(self, server.state())
                if path == '/api/logs':
                    limit = int((query.get('limit') or ['150'])[0] or 150)
                    level = (query.get('level') or [''])[0] or None
                    return _json_response(self, {
                        'items': server.bridge.logs.tail(limit, level),
                        'counts': server.bridge.logs.counts()})
                if path == '/api/stacks':
                    return _json_response(self, {'items': server.stacks.status(server.bridge.running_nodes())})
                if path == '/api/route_points':
                    name = (query.get('route') or [''])[0]
                    return _json_response(self, {
                        'route': name,
                        'items': server.bridge.route_points(name)})
                if path == '/api/routes':
                    return _json_response(self, {
                        'items': server.bridge.route_points()})
                if path == '/api/markers':
                    name = (query.get('route') or [''])[0]
                    points = server.bridge.route_points(name)
                    suspects = server.point_suspects(name)
                    count, _err = server.bridge.publish_route_markers(
                        points, suspects, 'map')
                    return _json_response(self, {
                        'ok': True, 'route': name, 'published': count,
                        'suspects': sorted(suspects),
                        'points': len(points)})
                if path == '/api/frame.jpg':
                    kind = (query.get('kind') or ['anno'])[0]
                    raw, _stamp = server.bridge.frame(kind == 'anno')
                    if not raw:
                        return _json_response(self, {'error': 'no frame'}, 404)
                    return _bytes_response(self, raw, 'image/jpeg')
                if path == '/api/runs':
                    return _json_response(self, {
                        'items': data.list_runs(server.bridge.reports_dir())})
                if path == '/api/events':
                    limit = int((query.get('limit') or ['120'])[0] or 120)
                    runs = data.list_runs(server.bridge.reports_dir(), 1)
                    if not runs:
                        return _json_response(self, {'items': []})
                    path_ = runs[0]['path'].replace('.summary.json', '.jsonl')
                    return _json_response(self, {
                        'run': runs[0]['name'],
                        'items': data.tail_events(path_, limit)})
                if path == '/api/routes':
                    return _json_response(self, {
                        'items': server.bridge.route_points()})
                return _json_response(self, {'error': 'not found'}, 404)

            # ---------------- POST ----------------
            def do_POST(self):
                parsed = urllib.parse.urlparse(self.path)
                path = parsed.path.rstrip('/')
                raw = b''
                try:
                    length = int(self.headers.get('Content-Length') or 0)
                    raw = self.rfile.read(length) or b'{}'
                    payload = json.loads(raw)
                    if not isinstance(payload, dict):
                        payload = {}
                except Exception:
                    # 解析失败不打死连接（浏览器只会看到 connection reset），
                    # 但也**不能把坏请求当成空 payload 继续执行**——那样一个
                    # 写坏的 body 会静默地真的把巡检启动起来。
                    if raw:
                        return _json_response(
                            self, {'ok': False,
                                   'message': '请求体不是合法 JSON'}, 400)
                    payload = {}

                # 任何一处参数不对都只影响这一次请求：回 400 + JSON，
                # 不能让异常冒到 do_POST 外面（那样浏览器只会看到连接被重置）。
                try:
                    return self._dispatch(path, payload)
                except (TypeError, ValueError) as error:
                    return _json_response(
                        self, {'ok': False, 'message': '参数不合法：%s' % error},
                        400)
                except Exception as error:      # 兜底，别把服务打死
                    return _json_response(
                        self, {'ok': False, 'message': '内部错误：%s' % error},
                        500)

            def _dispatch(self, path, payload):
                server_ = server
                if path == '/api/client_log':
                    # 浏览器里的 JS 异常打回服务端日志。没有这条，前端出错
                    # 只能靠"截图 + 猜"，而 console 报错在页面上是看不见的。
                    body = payload or {}
                    text = str(body.get('text') or '')[:500]
                    if not text:
                        return _json_response(self, {'ok': False}, 400)
                    level = 40 if str(body.get('level') or 'ERROR').upper() \
                        in ('ERROR', 'FATAL') else 30
                    server_.bridge.logs.add('browser', level, text)
                    return _json_response(self, {'ok': True})
                if path == '/api/estop':
                    body = payload or {}
                    # 显式带 active 就按它来；不带就当"按一下急停"
                    wanted = body.get('active')
                    if wanted is None:
                        ok, message = server_.bridge.estop()
                    elif wanted:
                        ok, message = server_.bridge.estop()
                    else:
                        ok, message = server_.bridge.release_estop()
                    return _json_response(
                        self, {'ok': ok, 'message': message,
                               'estop': server_.bridge.estop_state()})
                if path == '/api/stacks/start':
                    ok, message = server.stacks.start(
                        str(payload.get('name') or ''))
                    return _json_response(
                        self, {'ok': ok, 'message': message,
                               'items': server.stacks.status(server.bridge.running_nodes())})
                if path == '/api/stacks/stop':
                    ok, message = server.stacks.stop(
                        str(payload.get('name') or ''))
                    return _json_response(
                        self, {'ok': ok, 'message': message,
                               'items': server.stacks.status(server.bridge.running_nodes())})
                if path == '/api/record_point':
                    ok, message, point = server_.bridge.record_point(
                        payload.get('route') or '',
                        payload.get('point_id') or '',
                        payload.get('task_id') or 'detect',
                        float(payload.get('wait') or 0.0),
                        payload.get('expect') or '')
                    return _json_response(
                        self, {'ok': ok, 'message': message, 'point': point})
                if path == '/api/delete_point':
                    ok, message, remaining = server_.bridge.delete_point(
                        payload.get('route') or '',
                        payload.get('point_id') or '')
                    body = {'ok': ok, 'message': message}
                    # 只有真删掉时才回传剩余点数，失败时留空免得误读成“删完了”
                    if ok and remaining is not None:
                        body['remaining'] = remaining
                    return _json_response(self, body)
                if path == '/api/markers/clear':
                    server.bridge.clear_route_markers()
                    return _json_response(self, {'ok': True})
                if path == '/api/cancel_nav':
                    ok, message = server_.bridge.cancel_nav()
                    return _json_response(self, {'ok': ok, 'message': message})
                if path == '/api/patrol':
                    ok, message = server_.bridge.start_patrol(
                        payload.get('route') or '',
                        int(payload.get('mode') or 0),
                        int(payload.get('repeat') or 1))
                    return _json_response(self, {'ok': ok, 'message': message})
                if path == '/api/patrol/cancel':
                    ok, message = server_.bridge.cancel_patrol()
                    return _json_response(self, {'ok': ok, 'message': message})
                if path == '/api/capture':
                    ok, message = server_.bridge.snapshot()
                    return _json_response(self, {'ok': ok, 'message': message})
                if path == '/api/ask':
                    return _json_response(
                        self, server_.answer(str(payload.get('q') or '')))
                return _json_response(self, {'error': 'not found'}, 404)

        return Handler

    # ------------------------------------------------------------ 业务
    def state(self):
        status = self.bridge.snapshot_status()
        _png, meta = self.bridge.map_view()
        if meta.get('width'):
            meta = dict(meta)
            # ready 是页面的契约字段：没有它，前端会一直显示"还没有收到 /map"。
            # 之前只有 width/height，地图因此永远不出来。
            meta['ready'] = True
            meta['min_x'] = meta['origin_x']
            meta['max_x'] = meta['origin_x'] + meta['width'] * meta['resolution']
            meta['min_y'] = meta['origin_y']
            meta['max_y'] = meta['origin_y'] + meta['height'] * meta['resolution']
        return {
            # 急停状态要在刷新后能还原，否则按钮和红色提示条会「看着没停」
            'estop': self.bridge.estop_state(),
            'status': status,
            'map': meta,
            # 页面在顶层读 pose 画车（`st.pose`），而它实际存在 status 里，
            # 以前只给 status 一份，导致车在路上跑、地图上却没有车标。
            'pose': status.get('pose'),
            'subsystems': self.bridge.subsystems(),
            'camera': {
                'raw_age_s': self.bridge.frame_age(False),
                'anno_age_s': self.bridge.frame_age(True),
            },
            'log_counts': self.bridge.logs.counts(),
            'reports_dir': self.bridge.reports_dir(),
            'stacks': self.stacks.status(),
        }

    def point_suspects(self, route_name=''):
        """这条路线里"从来没检出过目标"的点（RViz 里打黄标、列表里带 ★）。

        为什么看最近 5 轮而不是全部历史：更早的报告可能是换了相机、换了场地之前
        留下的，拿它给点打黄标会误导。5 轮足够覆盖"偶发漏检"，又不至于把几个月前
        的旧账翻出来。
        """
        records = []
        picked = 0
        for run in data.list_runs(self.bridge.reports_dir(), 30):
            if route_name and run.get('route_name') not in ('', route_name):
                continue
            path = str(run.get('path') or '').replace('.summary.json', '.jsonl')
            records.extend(data.tail_events(path, 400))
            picked += 1
            if picked >= 5:
                break
        return points.never_detected(records)

    def _context(self):
        status = self.bridge.snapshot_status()
        _png, meta = self.bridge.map_view()
        if meta.get('width'):
            meta = dict(meta)
            meta['min_x'] = meta['origin_x']
            meta['max_x'] = meta['origin_x'] + meta['width'] * meta['resolution']
            meta['min_y'] = meta['origin_y']
            meta['max_y'] = meta['origin_y'] + meta['height'] * meta['resolution']
        status['map'] = meta
        errors = self.bridge.logs.tail(3, 'WARN')
        status['errors'] = '; '.join(
            '%s: %s' % (i['node'], i['text'][:60]) for i in errors)
        return status

    def answer(self, question):
        """先试大模型，失败/没配就退回规则。动作一律再过一遍白名单。"""
        context = self._context()
        bounds = None
        grid = context.get('map') or {}
        if grid.get('width'):
            bounds = {'min_x': grid['min_x'], 'max_x': grid['max_x'],
                      'min_y': grid['min_y'], 'max_y': grid['max_y']}
        base_url = self.settings.get('base_url') or ''
        api_key = self.settings.get('api_key') or ''
        if not base_url or not api_key:
            result = ai.rule_reply(question, context)
            result['source'] = 'rule'
            return result
        messages = [
            {'role': 'system', 'content': ai.SYSTEM_PROMPT},
            {'role': 'user',
             'content': '%s\n\n用户：%s' % (ai.context_block(context), question)},
        ]
        try:
            text = self._chat(messages, base_url, api_key,
                              self.settings.get('model') or 'gpt-4o-mini')
            result = ai.parse_model_output(text, bounds)
            result['source'] = 'model'
            return result
        except Exception as error:
            result = ai.rule_reply(question, context)
            result['source'] = 'rule'
            result['reason'] = '模型调用失败：%s' % error
            return result

    @staticmethod
    def _chat(messages, base_url, api_key, model):
        url = base_url.rstrip('/')
        if not url.endswith('/chat/completions'):
            url += '/chat/completions'
        body = json.dumps({
            'model': model, 'messages': messages, 'temperature': 0.2,
        }).encode('utf-8')
        request = urllib.request.Request(
            url, data=body,
            headers={'Content-Type': 'application/json',
                     'Authorization': 'Bearer %s' % api_key})
        with urllib.request.urlopen(request, timeout=AI_TIMEOUT) as response:
            payload = json.loads(response.read().decode('utf-8'))
        choices = payload.get('choices') or []
        if not choices:
            raise ValueError('模型没有返回 choices')
        return (choices[0].get('message') or {}).get('content') or ''

    # ------------------------------------------------------------ 启动
    def serve(self, host='0.0.0.0', port=8080, tries=10):
        """绑定端口；被占用时往后顺延，真的一个都占不上才抛。

        为什么要自动顺延：端口冲突是纯环境问题，跟"控制台能不能用"无关。
        现场出现过"没人占 9090 但一启动就报被占用"的情况（上一次进程的 socket
        还在 TIME_WAIT，或者同一个 launch 里的 rosbridge 先抢到），让用户去
        pkill / 换命令行都很绕。顺延一个端口、把实际端口打出来，问题就消失了；
        真的一个都不通（比如权限不够）再报错，那时报错才有用。
        """
        first = int(port)
        last_error = None
        self.skipped_ports = []
        for offset in range(max(1, int(tries))):
            candidate = first + offset
            try:
                self.httpd = ThreadingHTTPServer((host, candidate),
                                                 self.make_handler())
            except OSError as error:
                # Windows 上同一个原因报的是 WSAEACCES(10013) 而不是 EADDRINUSE
                busy = (getattr(error, 'errno', None) in (errno.EADDRINUSE,
                                                          errno.EACCES)
                        or getattr(error, 'winerror', None) in (10048, 10013))
                last_error = error
                if busy and offset + 1 < int(tries):
                    self.skipped_ports.append(candidate)
                    continue
                if busy:
                    raise RuntimeError(
                        busy_port_message(first)) from error
                raise RuntimeError(
                    '无法监听 %s:%d（%s）' % (host, candidate, error)) from error
            self.port = candidate
            self.httpd.daemon_threads = True
            return self.httpd
        raise RuntimeError('没有可用端口（%s）' % last_error)

