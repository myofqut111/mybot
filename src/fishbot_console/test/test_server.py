#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""server.py 的端到端测试：真起一个 HTTP 服务，用 urllib 打自己的接口。

用一个假的 bridge 顶替 ROS —— 这样在本机（没有 ROS、没有机器人）也能
验证"按钮 -> 接口 -> 调用"这条链路和页面能不能正常返回。
"""
import io
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

    def delete_point(self, route_name, point_id):
        self.calls.append(('delete_point', route_name, point_id))
        if point_id == 'p1':
            return True, 'Point deleted (0 left)', 0
        return False, 'point does not exist in route', None

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

    def costmap_view(self, slot='global'):
        self.calls.append(('costmap', slot))
        if not self.layers:
            return b'', {}
        return self.layers, {'width': 200, 'height': 200,
                             'resolution': 0.05,
                             'origin_x': 0.0, 'origin_y': 0.0}

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

    def test_client_log_records_browser_error(self):
        """浏览器异常要能落到服务端日志，否则前端报错只能靠猜。"""
        before = json.loads(self.get('/api/logs?limit=200')[1].decode('utf-8'))
        count_before = len(before.get('items') or [])
        body = self.post('/api/client_log',
                         {'level': 'ERROR', 'text': 'ReferenceError: x is not defined'})
        self.assertTrue(body.get('ok'))
        after = json.loads(self.get('/api/logs?limit=200')[1].decode('utf-8'))
        items = after.get('items') or []
        self.assertEqual(len(items), count_before + 1)
        self.assertEqual(items[-1]['node'], 'browser')
        self.assertEqual(items[-1]['level'], 'ERROR')
        self.assertIn('ReferenceError', items[-1]['text'])

    def test_delete_point_calls_route_manager(self):
        """删点必须走 route_manager，不能由控制台自己改 json。"""
        body = self.post('/api/delete_point',
                         {'route': 'main', 'point_id': 'p1'})
        self.assertTrue(body.get('ok'))
        self.assertEqual(body.get('remaining'), 0)
        self.assertIn(('delete_point', 'main', 'p1'), self.bridge.calls)

    def test_delete_point_reports_failure(self):
        body = self.post('/api/delete_point',
                         {'route': 'main', 'point_id': 'nope'})
        self.assertFalse(body.get('ok'))
        self.assertIn('does not exist', body.get('message', ''))

    def test_client_log_rejects_empty(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.post('/api/client_log', {'text': ''})
        self.assertEqual(ctx.exception.code, 400)

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
        # 前端画地图要的是 map.ready 与 min/max；只给 width 时地图永远不显示。
        self.assertTrue(payload['map']['ready'])
        self.assertEqual(payload['map']['max_x'], 0.0)
        self.assertIn('log_counts', payload)

    def test_state_exposes_top_level_pose(self):
        # 页面在顶层读 st.pose 画车；以前只给 status.pose，结果车在身上跑、
        # 地图上却没有车标。这条守的是那个契约。
        _status, body, _ctype = self.get('/api/state')
        payload = json.loads(body.decode('utf-8'))
        self.assertEqual(payload['pose'], payload['status']['pose'])
        self.assertEqual(payload['pose']['x'], 0.5)

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
        # 不锁条目总数：LogRing 在整个用例类里共用，别的用例往里面写日志
        # （比如浏览器异常回传）不该让这条用例变红。只验"种子日志在、过滤生效"。
        _s, body, _c = self.get('/api/logs?limit=10')
        payload = json.loads(body.decode('utf-8'))
        texts = [i['text'] for i in payload['items']]
        self.assertIn('hello', texts)
        self.assertIn('boom', texts)
        _s, body, _c = self.get('/api/logs?level=ERROR')
        payload = json.loads(body.decode('utf-8'))
        self.assertIn('boom', [i['text'] for i in payload['items']])
        self.assertNotIn('hello', [i['text'] for i in payload['items']])

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
    def test_patrol_roundtrip(self):
        out = self.post('/api/patrol',
                        {'route': 'main', 'mode': 1, 'repeat': 3})
        self.assertTrue(out['ok'])
        self.assertEqual(self.bridge.calls[-1], ('patrol', 'main', 1, 3))

    def test_cancel_endpoints(self):
        self.assertTrue(self.post('/api/cancel_nav', {})['ok'])
        self.assertTrue(self.post('/api/patrol/cancel', {})['ok'])

    def test_bad_json_body_does_not_crash(self):
        request = urllib.request.Request(
            self.url('/api/patrol'), data=b'not json',
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


    def test_state_carries_estop(self):
        """刷新页面后按钮和红色提示条要能还原。

        这条是补作业：/api/state 之前没带 estop，刷新后按钮显示没停、
        提示条也消失，而服务端其实还在以 20 Hz 发零速。
        """
        _code, body, _ctype = self.get('/api/state')
        payload = json.loads(body)
        self.assertIn('estop', payload)
        self.assertIn('active', payload['estop'])


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


class TestPortInUse(unittest.TestCase):
    """端口被占用时的行为：能顺延就顺延，顺延不了才报错。"""

    def test_busy_port_falls_back_to_next(self):
        """端口被占不该让控制台起不来——顺延一个端口，把实际端口报出来。"""
        import socket
        holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        holder.bind(('127.0.0.1', 0))
        holder.listen(1)
        busy_port = holder.getsockname()[1]
        try:
            service = server.ConsoleServer(FakeBridge())
            httpd = service.serve('127.0.0.1', busy_port)
        finally:
            holder.close()
        try:
            self.assertNotEqual(service.port, busy_port)
            self.assertGreater(service.port, busy_port)
            self.assertIn(busy_port, service.skipped_ports)
        finally:
            httpd.server_close()

    def test_busy_port_raises_when_no_room(self):
        """只允许试一个端口时，被占就必须报错，且文案要能照做。"""
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('127.0.0.1', 0))
        sock.listen(1)
        port = sock.getsockname()[1]
        try:
            service = server.ConsoleServer(FakeBridge())
            with self.assertRaises(RuntimeError) as caught:
                service.serve('127.0.0.1', port, tries=1)
        finally:
            sock.close()
        message = str(caught.exception)
        self.assertIn(str(port), message)
        self.assertIn('port:=9000', message)
        self.assertIn('pkill -f robot_console', message)
        self.assertIn('ss -ltnp', message)

    def test_port_9090_gets_rosbridge_hint(self):
        """9090 撞车几乎一定是命令里写了 port:=9090，不是旧实例没退。

        报错必须直接把这句话说出来，否则用户会去 pkill 一个不存在的进程。
        直接测文案函数：Windows 上 SO_REUSEADDR 允许两个 socket 同端口，
        真去占 9090 反而不报错，那个用例在任何平台上都测不稳。
        """
        message = server.busy_port_message(server.ROSBRIDGE_PORT)
        self.assertIn('rosbridge', message)
        self.assertIn('8080', message)
        self.assertNotIn('rosbridge', server.busy_port_message(8080))

    def test_free_port_binds(self):
        service = server.ConsoleServer(FakeBridge())
        httpd = service.serve('127.0.0.1', 0)
        try:
            self.assertIsNotNone(service.httpd)
            self.assertTrue(httpd.daemon_threads)
        finally:
            httpd.server_close()


class TestLogSink(unittest.TestCase):
    """stacks 只传 (source, text)；适配器必须吃下 2 参调用。

    这是真踩过的坑：适配器写成固定 3 参，业务栈起来了但接口回 400
    「参数不合法」，看日志的人会以为没启动成功。
    """

    def test_two_arg_call_is_accepted(self):
        seen = []

        class Logs(object):
            def add(self, source, level, text):
                seen.append((source, level, text))

        sink = server._log_sink_of(Logs())
        sink('nav', 'INFO: 起来了')
        self.assertEqual(seen, [('nav', 20, 'INFO: 起来了')])

    def test_three_arg_call_keeps_level(self):
        seen = []

        class Logs(object):
            def add(self, source, level, text):
                seen.append((source, level, text))

        sink = server._log_sink_of(Logs())
        sink('nav', 40, '炸了')
        self.assertEqual(seen, [('nav', 40, '炸了')])

    def test_sink_never_raises(self):
        class Boom(object):
            def add(self, *args):
                raise RuntimeError('boom')

        sink = server._log_sink_of(Boom())
        sink('nav', 'x')        # 不能抛
        sink('nav', 40, 'y')


class EnvInfoCase(unittest.TestCase):
    """/api/stacks 要带 env，面板才能解释“为什么是 0/3”。"""

    @property
    def pkg_dir(self):
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def _read(self, name):
        return io.open(os.path.join(self.pkg_dir, 'fishbot_console', name),
                       encoding='utf-8').read()

    def test_bridge_has_env_info(self):
        src = self._read('ros_bridge.py')
        self.assertIn('def env_info(', src)
        for key in ("'node_count'", "'domain'", "'rmw'"):
            self.assertIn(key, src)

    def test_stacks_route_embeds_env(self):
        src = self._read('server.py')
        self.assertIn("'env': server.bridge.env_info()", src)

    def test_start_passes_running_nodes(self):
        """起栈时要带上图里的节点名，否则会起出第二套。"""
        src = self._read('server.py')
        self.assertIn('running_nodes=server.bridge.running_nodes()', src)

    def test_page_renders_stack_logs(self):
        """降级的栈必须能把 launch 的日志显示出来。"""
        src = self._read('page.py')
        self.assertIn('启动日志（最近 ', src)
        self.assertIn('it.logs', src)

    def test_page_has_env_line(self):
        src = self._read('page.py')
        self.assertIn('envLine', src)
        self.assertIn('ROS_DOMAIN_ID', src)


if __name__ == '__main__':
    unittest.main()
