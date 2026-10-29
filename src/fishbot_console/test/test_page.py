#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""page.py：页面必须自带全部资源（机器人常常没外网），且元素 id 和 JS 对得上。"""
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import page  # noqa: E402

REQUIRED_IDS = [
    'bState', 'bBatt', 'bPose', 'bCam', 'bErr', 'map', 'cam',
    'navX', 'navY', 'btnSendNav', 'btnCancelNav', 'btnCapture', 'camKind',
    'patrolRoute', 'patrolMode', 'patrolRepeat', 'btnPatrol',
    'btnPatrolCancel', 'mapName', 'btnMapStart', 'btnMapStop', 'btnMapSave',
    'logList', 'logLevel', 'runsBody', 'chatLog', 'askBox', 'btnAsk',
    'teleopPad', 'teleSpeed', 'teleLabel',
    'chkCostmap', 'chkScan', 'chkPlan', 'mapCoord', 'zoomLabel',
]


class TestPage(unittest.TestCase):
    def setUp(self):
        self.html = page.HTML

    def test_is_html(self):
        self.assertTrue(self.html.lstrip().startswith('<!doctype html>'))
        self.assertIn('</html>', self.html)

    def test_all_ids_exist(self):
        for element_id in REQUIRED_IDS:
            self.assertIn('id="%s"' % element_id, self.html,
                          'missing id %s' % element_id)

    def test_no_external_resources(self):
        # 不允许任何 http(s) 外链（CDN/字体/图标），否则没网时页面会卡住
        external = re.findall(r'(?:src|href)="https?://[^"]+"', self.html)
        self.assertEqual(external, [])

    def test_script_is_balanced(self):
        # 一个内联脚本 + 一个 Leaflet 的 src 引用
        self.assertEqual(self.html.count('<script>'), 1)
        self.assertEqual(self.html.count('<script src='), 1)
        self.assertEqual(self.html.count('</script>'), 2)
        self.assertEqual(self.html.count('<style>'), 1)
        self.assertEqual(self.html.count('</style>'), 1)

    def test_api_paths_match_server(self):
        for path in ('/api/state', '/api/logs', '/api/map.png',
                     '/api/frame.jpg', '/api/runs', '/api/nav',
                     '/api/patrol', '/api/capture', '/api/mapping',
                     '/api/ask', '/api/cancel_nav', '/api/patrol/cancel',
                     '/api/teleop', '/api/initialpose'):
            self.assertIn(path, self.html, 'page never calls %s' % path)

    def test_map_layers_and_scan_present(self):
        """RViz 对齐的三样东西必须真的在页面里，否则只是后端有接口。"""
        self.assertIn('id="chkCostmap"', self.html)
        self.assertIn('id="chkScan"', self.html)
        self.assertIn('/api/layers.png', self.html)
        self.assertIn('/api/scan', self.html)
        self.assertIn('LOADLAYERS', self.html.upper().replace('loadLayers', 'LOADLAYERS'))

    def test_rviz_like_interaction(self):
        """手感对齐 RViz 的硬指标（Leaflet 版）。

        换成 Leaflet 之后，滚轮锚定缩放、中键平移、双指缩放都由库负责，
        我们只需要保证：简单坐标系（1 单位 = 1 米）、拖动可开关、
        导航/位姿是"按下-拖出-松手"、代价层和点云各自能关。
        """
        self.assertIn('L.CRS.Simple', self.html)
        self.assertIn('LF.dragging.enable()', self.html)
        self.assertIn('LF.dragging.disable()', self.html)
        self.assertIn("LF.on('zoomend'", self.html)
        self.assertIn("LF.on('mousedown', onDown)", self.html)
        self.assertIn("LF.on('mouseup', onUp)", self.html)
        self.assertIn('chkCostmap', self.html)
        self.assertIn('chkScan', self.html)
    def test_map_click_sends_nav(self):
        # 换 Leaflet 之后不再手写 canvas 事件，改由 Leaflet 的 mousedown/mouseup
        # 回调驱动：按下定位置、拖出朝向、松手发目标。
        self.assertIn("LF.on('mousedown', onDown)", self.html)
        self.assertIn("LF.on('mouseup', onUp)", self.html)
        self.assertIn('sendNav(done.x, done.y, yaw)', self.html)

    def test_teleop_has_deadman(self):
        # 手动控制必须有"松手即停"和页面失焦停，否则手机切走车会一直跑
        self.assertIn("addEventListener('pointerup'", self.html)
        self.assertIn("window.addEventListener('blur', teleStop)", self.html)
        self.assertIn("'/api/teleop'", self.html)

    def test_static_assets_exist(self):
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        static = os.path.join(os.path.dirname(here), 'fishbot_console', 'static')
        for name in ('leaflet.js', 'leaflet.css'):
            self.assertTrue(os.path.isfile(os.path.join(static, name)),
                            'missing static asset %s' % name)


if __name__ == '__main__':
    unittest.main()
