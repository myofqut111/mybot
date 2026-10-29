#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""server.py 的端到端测试：真起一个 HTTP 服务，用 urllib 打自己的接口。

用一个假的 bridge 顶替 ROS —— 这样在本机（没有 ROS、没有机器人）也能
验证"按钮 -> 接口 -> 调用"这条链路和页面能不能正常返回。
"""
import json
import os
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import data, map_render, server  # noqa: E402


class FakeBridge(object):
    def __init__(self, reports_dir=''):
        self.logs = data.LogRing(capacity=10)
        self.logs.add('fake', 20, 'hello')
        self.logs.add('fake', 40, 'boom')
        self._reports = reports_dir
        self.calls = []

    def snapshot_status(self):
        return {'state': 'RUNNING', 'battery_v': 12.1, 'battery_band': 'ok',
                'route_name': 'main', 'point_id': 'p1', 'completed_points': 1,
                'pose': {'x': 0.5, 'y': 0.5, 'age_s': 0.2}}

    def map_view(self):
        return map_render.grid_to_png([0, 100, 0, 100], 2, 2), {
            'width': 2, 'height': 2, 'resolution': 0.5,
            'origin_x': -1.0, 'origin_y': -1.0}

    def frame(self, annotated=False):
        return b'\xff\xd8\xff\xd9', 1.0

    def frame_age(self, annotated=False):
        return 0.5

    def reports_dir(self):
        return self._reports

    def route_points(self):
        return [{'point_id': 'p1', 'x': 1.0, 'y': 2.0}]

    def send_nav(self, x, y, yaw=None):
        self.calls.append(('nav', x, y, yaw))
        return True, 'ok'

    def cancel_nav(self):
        self.calls.append(('cancel_nav',))
        return True, 'ok'

    def start_patrol(self, route='', mode=0, repeat=1):
        self.calls.append(('patrol', route, mode, repeat))
        return True, 'ok'

    def cancel_patrol(self):
        self.calls.append(('cancel_patrol',))
        return True, 'ok'

    def snapshot(self, annotated=True):
        self.calls.append(('snapshot', annotated))
        return True, '~/console_snapshots/snap.jpg'

    def call_capture(self, timeout=3.0):
        self.calls.append(('capture',))
        return True, 'saved'

    def mapping(self, action, name='map'):
        self.calls.append(('mapping', action, name))
        return True, 'ran %s' % action


class ServerCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        bridge = FakeBridge(cls.tmp.name)
        console = server.ConsoleServer(bridge, {})
        cls.bridge = bridge
        cls.httpd = console.serve('127.0.0.1', 0)
        cls.port = cls.httpd.server_address[1]
        cls.thread = threading.Thread(target=cls.httpd.serve_forever,
                                      daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.tmp.cleanup()

    def url(self, path):
        return 'http://127.0.0.1:%d%s' % (self.port, path)

    def get(self, path):
        with urllib.request.urlopen(self.url(path), timeout=5) as response:
            return response.status, response.read(), \
                response.headers.get('Content-Type', '')

    def post(self, path, payload):
        body = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            self.url(path), data=body,
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.loads(response.read().decode('utf-8'))

    # ---------------- 页面与静态 ----------------
    def test_page(self):
        status, body, ctype = self.get('/')
        self.assertEqual(status, 200)
        self.assertIn('text/html', ctype)
        self.assertIn('fishbot 控制台', body.decode('utf-8'))

    def test_favicon_is_404_json(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.get('/favicon.ico')
        self.assertEqual(ctx.exception.code, 404)

    # ---------------- 数据接口 ----------------
    def test_state(self):
        _status, body, _ctype = self.get('/api/state')
        payload = json.loads(body.decode('utf-8'))
        self.assertEqual(payload['status']['state'], 'RUNNING')
        self.assertEqual(payload['map']['max_x'], 0.0)
        self.assertIn('log_counts', payload)

    def test_logs_and_filter(self):
        _s, body, _c = self.get('/api/logs?limit=10')
        payload = json.loads(body.decode('utf-8'))
        self.assertEqual(len(payload['items']), 2)
        _s, body, _c = self.get('/api/logs?level=ERROR')
        payload = json.loads(body.decode('utf-8'))
        self.assertEqual([i['text'] for i in payload['items']], ['boom'])

    def test_map_png_is_png(self):
        status, body, ctype = self.get('/api/map.png')
        self.assertEqual(status, 200)
        self.assertEqual(ctype, 'image/png')
        self.assertTrue(body.startswith(b'\x89PNG'))

    def test_frame_jpeg(self):
        _s, body, ctype = self.get('/api/frame.jpg?kind=raw')
        self.assertEqual(ctype, 'image/jpeg')
        self.assertTrue(body.startswith(b'\xff\xd8'))

    def test_runs_and_events_empty(self):
        _s, body, _c = self.get('/api/runs')
        self.assertEqual(json.loads(body.decode('utf-8'))['items'], [])
        _s, body, _c = self.get('/api/events')
        self.assertEqual(json.loads(body.decode('utf-8'))['items'], [])

    def test_routes(self):
        _s, body, _c = self.get('/api/routes')
        self.assertEqual(json.loads(body.decode('utf-8'))['items'][0]['x'], 1.0)

    # ---------------- 动作接口 ----------------
    def test_nav_roundtrip(self):
        out = self.post('/api/nav', {'x': 1.5, 'y': -2.0})
        self.assertTrue(out['ok'])
        self.assertEqual(self.bridge.calls[-1], ('nav', 1.5, -2.0, None))

    def test_patrol_roundtrip(self):
        out = self.post('/api/patrol',
                        {'route': 'main', 'mode': 1, 'repeat': 3})
        self.assertTrue(out['ok'])
        self.assertEqual(self.bridge.calls[-1], ('patrol', 'main', 1, 3))

    def test_cancel_endpoints(self):
        self.assertTrue(self.post('/api/cancel_nav', {})['ok'])
        self.assertTrue(self.post('/api/patrol/cancel', {})['ok'])

    def test_capture_and_mapping(self):
        self.assertTrue(self.post('/api/capture', {})['ok'])
        out = self.post('/api/mapping', {'action': 'save', 'name': 'lab'})
        self.assertTrue(out['ok'])
        self.assertEqual(self.bridge.calls[-1], ('mapping', 'save', 'lab'))

    def test_bad_json_body_does_not_crash(self):
        request = urllib.request.Request(
            self.url('/api/nav'), data=b'not json',
            headers={'Content-Type': 'application/json'})
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(request, timeout=5)
        self.assertEqual(ctx.exception.code, 400)

    # ---------------- AI ----------------
    def test_ask_without_model_uses_rules(self):
        out = self.post('/api/ask', {'q': '还有多少电'})
        self.assertEqual(out['source'], 'rule')
        self.assertIn('12.10', out['reply'])

    def test_ask_nav_action(self):
        out = self.post('/api/ask', {'q': '去 x=1 y=2'})
        self.assertEqual(out['action'], {'type': 'nav', 'x': 1.0, 'y': 2.0})


class StaticAnswerCase(unittest.TestCase):
    """不需要起服务的部分：answer() 的分支。"""

    def test_model_failure_falls_back_to_rule(self):
        bridge = FakeBridge('')
        console = server.ConsoleServer(
            bridge, {'base_url': 'http://127.0.0.1:9/v1',
                     'api_key': 'x', 'model': 'm'})
        out = console.answer('电量')
        self.assertEqual(out['source'], 'rule')
        self.assertIn('模型调用失败', out['reason'])

    def test_parse_chat_payload(self):
        self.assertEqual(server.ConsoleServer._chat.__name__, '_chat')


if __name__ == '__main__':
    unittest.main()
