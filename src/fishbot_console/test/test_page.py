# -*- coding: utf-8 -*-
"""page.py 的回归测试：页面必须自带全部资源，且每个控件都真的能连到接口。

0.5.0 换了版面（地图/手动控制/导航/建图整块删掉，改成启停 + 录点 + 报告），
所以这里守的不再是"地图能不能画"，而是：
1. 该有的控件都在（少了就是点了没反应）；
2. 不该有的旧控件**真的没了**（留着就是死按钮，点下去 404）；
3. 页面引用的接口都在服务端真实存在（口径漂移过一次，代价是整页空白）。
"""
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import page  # noqa: E402

REQUIRED_IDS = [
    'bState', 'bBatt', 'bPose', 'bCam', 'bErr', 'bConn',
    'btnEstop', 'estopBar',
    'stackRows',
    'cam', 'camKind', 'btnCapture',
    'patrolRoute', 'patrolMode', 'patrolRepeat', 'btnPatrol',
    'btnPatrolCancel',
    'recRoute', 'recName', 'recExpect', 'btnRecord',
    'btnPoints', 'pointRows',
    'runSel', 'btnReload', 'eventBody', 'runHint',
    'logList', 'logLevel', 'logCounts',
    'chatLog', 'askBox', 'btnAsk',
]

# 0.5.0 明确删掉的：它们对应的服务端路由也已经不存在，留着就是死按钮
REMOVED_IDS = ['mapCanvas', 'navX', 'navY', 'btnSendNav', 'btnCancelNav',
               'chkCostmap', 'chkScan', 'btnMapStart', 'btnMapStop',
               'btnMapSave', 'mapName']


class TestPage(unittest.TestCase):
    def setUp(self):
        self.html = page.HTML

    def test_is_html(self):
        self.assertTrue(self.html.lstrip().startswith('<!doctype html>'))
        self.assertIn('</html>', self.html)

    def test_required_ids_exist(self):
        for element_id in REQUIRED_IDS:
            self.assertIn('id="%s"' % element_id, self.html,
                          'missing id %s' % element_id)

    def test_old_map_controls_are_gone(self):
        for element_id in REMOVED_IDS:
            self.assertNotIn('id="%s"' % element_id, self.html,
                             '%s 应该被删掉' % element_id)

    def test_no_external_resources(self):
        """机器人在场地常常没有外网，页面不能依赖任何 CDN。"""
        external = re.findall(r'(?:src|href)="https?://[^"]+"', self.html)
        self.assertEqual(external, [])

    def test_script_and_style_balanced(self):
        self.assertEqual(self.html.count('<script>'), 1)
        self.assertEqual(self.html.count('</script>'), 1)
        self.assertEqual(self.html.count('<style>'), 1)
        self.assertEqual(self.html.count('</style>'), 1)

    def test_api_paths_exist_on_server(self):
        server_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'fishbot_console', 'server.py')
        with open(server_path, encoding='utf-8') as handle:
            server = handle.read()
        used = set(re.findall(r"'(/api/[a-z_/]*[a-z_])'", self.html))
        self.assertTrue(used, '页面没有引用任何接口？')
        for path in sorted(used):
            self.assertIn("'%s'" % path, server,
                          'page 调了 %s，但 server.py 里没有这个路由' % path)

    def test_estop_button_wired(self):
        self.assertIn("$('btnEstop').onclick", self.html)
        self.assertIn("ev.key === 'Escape'", self.html)

    def test_stack_actions_post(self):
        self.assertIn("/api/stacks/' + action", self.html)
        self.assertIn("stackAction('start'", self.html)
        self.assertIn("stackAction('stop'", self.html)

    def test_browser_errors_are_reported(self):
        self.assertIn('unhandledrejection', self.html)
        self.assertIn("'/api/client_log'", self.html)

    def test_record_point_posts_expect(self):
        self.assertIn('expect:', self.html)
        self.assertIn("'/api/record_point'", self.html)

    def test_javascript_parses(self):
        """内联 JS 必须能被 node 解析。

        这条是补作业：之前 mapBounds(...).pad(...) 这类错误只有在浏览器里
        才会炸，而页面上一片空白、看不出是语法还是逻辑问题。node --check
        能在本机把这类错误挡在打包之前。
        """
        import shutil
        import subprocess
        import tempfile
        node = shutil.which('node')
        if not node:
            self.skipTest('node 不在 PATH 里')
        match = re.search(r'<script>(.*?)</script>', self.html, re.S)
        self.assertIsNotNone(match, '页面里没有内联 script')
        with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False,
                                         encoding='utf-8') as handle:
            handle.write(match.group(1))
            path = handle.name
        try:
            out = subprocess.run([node, '--check', path],
                                 capture_output=True, text=True,
                                 encoding='utf-8', errors='replace')
            self.assertEqual(out.returncode, 0,
                             'JS 语法错误：%s %s' % (out.stdout, out.stderr))
        finally:
            os.unlink(path)


if __name__ == '__main__':
    unittest.main(verbosity=2)
