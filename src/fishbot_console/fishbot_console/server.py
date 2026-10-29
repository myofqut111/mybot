#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTP 服务：把 bridge 的数据变成 JSON / PNG / JPEG，把按钮变成调用。

用标准库 http.server —— 机器人上不需要再装 Flask，而且这点并发量
（一个人、手机 + 电脑）根本用不上多进程。
"""
import json
import os
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import ai, data
from .page import HTML

AI_TIMEOUT = 25.0


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


class ConsoleServer(object):
    """把 bridge + AI 设置包在一个对象里，handler 通过闭包拿到它。"""

    def __init__(self, bridge, settings=None):
        self.bridge = bridge
        self.settings = settings or {}
        self.httpd = None

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
                if path == '/api/state':
                    return _json_response(self, server.state())
                if path == '/api/logs':
                    limit = int((query.get('limit') or ['150'])[0] or 150)
                    level = (query.get('level') or [''])[0] or None
                    return _json_response(self, {
                        'items': server.bridge.logs.tail(limit, level),
                        'counts': server.bridge.logs.counts()})
                if path == '/api/map.png':
                    png, _meta = server.bridge.map_view()
                    if not png:
                        return _json_response(self, {'error': 'no map'}, 404)
                    return _bytes_response(self, png, 'image/png')
                if path == '/api/layers.png':
                    png = server.bridge.layers_view()
                    if not png:
                        return _json_response(self, {'error': 'no layers'}, 404)
                    return _bytes_response(self, png, 'image/png')
                if path == '/api/scan':
                    points, age = server.bridge.scan_view()
                    return _json_response(self, {
                        'frame': 'map',
                        'age': round(age, 2),
                        'points': [[round(x, 2), round(y, 2)]
                                   for x, y in points]})
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
                try:
                    length = int(self.headers.get('Content-Length') or 0)
                    payload = json.loads(self.rfile.read(length) or b'{}')
                    if not isinstance(payload, dict):
                        payload = {}
                except Exception:
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
                if path == '/api/nav':
                    x = float(payload.get('x'))
                    y = float(payload.get('y'))
                    yaw = payload.get('yaw')
                    ok, message = server_.bridge.send_nav(
                        x, y, None if yaw is None else float(yaw))
                    return _json_response(self, {'ok': ok, 'message': message})
                if path == '/api/cancel_nav':
                    ok, message = server_.bridge.cancel_nav()
                    return _json_response(self, {'ok': ok, 'message': message})
                if path == '/api/initialpose':
                    ok, message = server_.bridge.set_initial_pose(
                        float(payload.get('x')),
                        float(payload.get('y')),
                        float(payload.get('yaw') or 0.0))
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
                if path == '/api/mapping':
                    ok, text = server_.bridge.mapping(
                        payload.get('action'), payload.get('name') or 'map')
                    return _json_response(self, {'ok': ok, 'message': text})
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
            meta['min_x'] = meta['origin_x']
            meta['max_x'] = meta['origin_x'] + meta['width'] * meta['resolution']
            meta['min_y'] = meta['origin_y']
            meta['max_y'] = meta['origin_y'] + meta['height'] * meta['resolution']
        return {
            'status': status,
            'map': meta,
            'subsystems': self.bridge.subsystems(),
            'camera': {
                'raw_age_s': self.bridge.frame_age(False),
                'anno_age_s': self.bridge.frame_age(True),
            },
            'log_counts': self.bridge.logs.counts(),
            'reports_dir': self.bridge.reports_dir(),
        }

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
    def serve(self, host='0.0.0.0', port=8080):
        self.httpd = ThreadingHTTPServer((host, int(port)),
                                         self.make_handler())
        self.httpd.daemon_threads = True
        return self.httpd
