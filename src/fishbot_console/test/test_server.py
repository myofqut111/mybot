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
        self.layers = b'\x89PNG-layers'
        self.scan = [(1.25, -0.5), (2.0, 0.75)]
        self.plan = [(0.0, 0.0), (1.0, 0.5), (2.0, 0.75)]
        self._estop = {'active': False, 'since': 0.0}

    def estop(self):
        self.calls.append(('estop',))
        self._estop = {'active': True, 'since': 123.0}
        return True, '已急停（nav 已取消，patrol 已取消）'

    def release_estop(self):
        self.calls.append(('release_estop',))
        self._estop = {'active': False, 'since': 0.0}
        return True, '已解除急停'

    def estop_state(self):
        return dict(self._estop)

    def snapshot_status(self):
        return {'state': 'RUNNING', 'battery_v': 12.1, 'battery_band': 'ok',
                'route_name': 'main', 'point_id': 'p1', 'completed_points': 1,
                'pose': {'x': 0.5, 'y': 0.5, 'age_s': 0.2},
                'estop': dict(self._estop)}

    def map_view(self):
        return map_render.grid_to_png([0, 100, 0, 100], 2, 2), {
            'width': 2, 'height': 2, 'resolution': 0.5,
            'origin_x': -1.0, 'origin_y': -1.0}

    def frame(self, annotated=False):
        return b'\xff\xd8\xff\xd9', 1.0

    def frame_age(self, annotated=False):
        return 0.5

    def subsystems(self):
        return [
            {'key': 'map', 'label': '地图', 'ok': True, 'hint': '', 'detail': ''},
            {'key': 'nav', 'label': '导航', 'ok': False,
             'hint': 'nav2 没起', 'detail': ''},
        ]

    def set_initial_pose(self, x, y, yaw=0.0):
        self.calls.append(('initialpose', x, y, yaw))
        return True, '已发送初始位姿 (%.2f, %.2f) %.0f°' % (x, y, yaw * 57.3)

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

    def layers_view(self):
        self.calls.append(('layers',))
        return self.layers

    def scan_view(self):
        self.calls.append(('scan',))
        return list(self.scan), 0.2

    def plan_view(self):
        self.calls.append(('plan',))
        return list(self.plan), 0.3

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

    def test_static_leaflet_is_served(self):
        """地图组件必须真能取到：取不到的后果是页面白屏，而不是少个按钮。"""
        status, body, ctype = self.get('/static/leaflet.js')
        self.assertEqual(status, 200)
        self.assertIn('javascript', ctype)
        self.assertGreater(len(body), 50000)
        self.assertIn(b'leaflet', body.lower())

        status, body, ctype = self.get('/static/leaflet.css')
        self.assertEqual(status, 200)
        self.assertIn('text/css', ctype)

    def test_static_rejects_traversal(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.get('/static/../server.py')
        self.assertEqual(ctx.exception.code, 404)

    # ---------------- 数据接口 ----------------
    def test_state(self):
        _status, body, _ctype = self.get('/api/state')
        payload = json.loads(body.decode('utf-8'))
        self.assertEqual(payload['status']['state'], 'RUNNING')
        self.assertEqual(payload['map']['max_x'], 0.0)
        self.assertIn('log_counts', payload)

    def test_state_lists_missing_subsystems(self):
        # 顶部那排小灯：缺什么要能一眼看出来，而不是面板空着让人猜
        _status, body, _ctype = self.get('/api/state')
        payload = json.loads(body.decode('utf-8'))
        items = payload['subsystems']
        self.assertEqual([i['key'] for i in items], ['map', 'nav'])
        missing = [i for i in items if not i['ok']]
        self.assertEqual([i['label'] for i in missing], ['导航'])
        self.assertIn('nav2', missing[0]['hint'])

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

    def test_layers_png(self):
        status, body, ctype = self.get('/api/layers.png')
        self.assertEqual(status, 200)
        self.assertEqual(ctype, 'image/png')
        self.assertEqual(body, b'\x89PNG-layers')

    def test_layers_missing_is_404(self):
        saved = self.bridge.layers
        self.bridge.layers = b''
        try:
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                self.get('/api/layers.png')
            self.assertEqual(ctx.exception.code, 404)
        finally:
            self.bridge.layers = saved       # 同一个 bridge 被各用例复用

    def test_scan_points(self):
        status, body, _c = self.get('/api/scan')
        self.assertEqual(status, 200)
        data_ = json.loads(body.decode('utf-8'))
        self.assertEqual(data_['frame'], 'map')
        self.assertEqual(data_['points'], [[1.25, -0.5], [2.0, 0.75]])
        self.assertLessEqual(len(data_['points']), 400)

    def test_plan_json(self):
        status, body, _c = self.get('/api/plan.json')
        self.assertEqual(status, 200)
        payload = json.loads(body.decode('utf-8'))
        self.assertEqual(payload['frame'], 'map')
        self.assertEqual(payload['points'],
                         [[0.0, 0.0], [1.0, 0.5], [2.0, 0.75]])

    def test_nav_with_yaw(self):
        self.post('/api/nav', {'x': 1.0, 'y': 2.0, 'yaw': 0.5})
        self.assertIn(('nav', 1.0, 2.0, 0.5), self.bridge.calls)

    def test_nav_without_yaw_sends_none(self):
        self.post('/api/nav', {'x': 1.0, 'y': 2.0})
        self.assertIn(('nav', 1.0, 2.0, None), self.bridge.calls)

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

    def test_initial_pose_roundtrip(self):
        # 地图上拖一下 -> /api/initialpose -> /initialpose（等价 RViz 的 2D Pose Estimate）
        out = self.post('/api/initialpose', {'x': 1.25, 'y': -0.5, 'yaw': 1.57})
        self.assertTrue(out['ok'])
        self.assertEqual(self.bridge.calls[-1], ('initialpose', 1.25, -0.5, 1.57))

    def test_initial_pose_defaults_yaw(self):
        # 只点不拖时 yaw 缺省为 0，不能因为没传就 500
        out = self.post('/api/initialpose', {'x': 0.0, 'y': 0.0})
        self.assertTrue(out['ok'])
        self.assertEqual(self.bridge.calls[-1][3], 0.0)

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

    # ---------------- 急停 ----------------
    def test_estop_press_and_release(self):
        out = self.post('/api/estop', {'active': True})
        self.assertTrue(out['ok'])
        self.assertTrue(out['estop']['active'])
        self.assertIn(('estop',), self.bridge.calls)

        out = self.post('/api/estop', {'active': False})
        self.assertTrue(out['ok'])
        self.assertFalse(out['estop']['active'])
        self.assertIn(('release_estop',), self.bridge.calls)

    def test_estop_without_body_means_press(self):
        # 前端只发一个空对象也该按"急停"处理，不能静默无动作
        out = self.post('/api/estop', {})
        self.assertTrue(out['estop']['active'])
        self.assertIn(('estop',), self.bridge.calls)

    def test_state_reports_estop(self):
        self.post('/api/estop', {'active': True})
        status, body, _ctype = self.get('/api/state')
        self.assertEqual(status, 200)
        state = json.loads(body.decode('utf-8'))
        self.assertTrue(state['status']['estop']['active'])


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
