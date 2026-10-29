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
    'bState', 'bBatt', 'bPose', 'bCam', 'bErr', 'mapCanvas', 'cam',
    'navX', 'navY', 'btnSendNav', 'btnCancelNav', 'btnCapture', 'camKind',
    'patrolRoute', 'patrolMode', 'patrolRepeat', 'btnPatrol',
    'btnPatrolCancel', 'mapName', 'btnMapStart', 'btnMapStop', 'btnMapSave',
    'logList', 'logLevel', 'runsBody', 'chatLog', 'askBox', 'btnAsk',
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
        self.assertEqual(self.html.count('<script>'), 1)
        self.assertEqual(self.html.count('</script>'), 1)
        self.assertEqual(self.html.count('<style>'), 1)
        self.assertEqual(self.html.count('</style>'), 1)

    def test_api_paths_match_server(self):
        for path in ('/api/state', '/api/logs', '/api/map.png',
                     '/api/frame.jpg', '/api/runs', '/api/nav',
                     '/api/patrol', '/api/capture', '/api/mapping',
                     '/api/ask', '/api/cancel_nav', '/api/patrol/cancel'):
            self.assertIn(path, self.html, 'page never calls %s' % path)

    def test_map_click_sends_nav(self):
        self.assertIn("getElementById('mapCanvas')", self.html.replace(
            '$(\'mapCanvas\')', "getElementById('mapCanvas')"))
        self.assertIn('addEventListener(\'click\'', self.html)
        self.assertIn('sendNav', self.html)

    def test_vertical_flip_math_present(self):
        # 页面的像素->世界换算必须对 y 做翻转，写反了点击位置会上下颠倒
        self.assertIn('(MAP.height-1)', self.html.replace(' ', ''))

    def test_polling_intervals(self):
        for fn in ('pollState', 'pollFrame', 'pollLogs', 'pollRuns'):
            self.assertIn(fn, self.html)
        self.assertGreaterEqual(self.html.count('setInterval'), 4)

    def test_ai_action_shown(self):
        self.assertIn('/api/ask', self.html)
        self.assertIn('runAction', self.html)

    def test_charset_utf8(self):
        self.assertIn('<meta charset="utf-8">', self.html)


if __name__ == '__main__':
    unittest.main()
