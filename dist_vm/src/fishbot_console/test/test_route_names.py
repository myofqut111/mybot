# -*- coding: utf-8 -*-
"""路线名读取与面板契约：路线下拉看不到已有路线的问题（回归）。

背景：/api/routes 只返回“当前激活路线”的点，而 active_route 从来没人设过；
面板的下拉又只从历史运行记录里猜名字，于是刚录好的路线在页面上像丢了。
这里锁住新的读取路径和页面契约。
"""
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from fishbot_console import data  # noqa: E402


class ReadRouteNamesTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='routes_')
        self.path = os.path.join(self.dir, 'fishbot_patrol_routes.json')

    def tearDown(self):
        for name in os.listdir(self.dir):
            os.remove(os.path.join(self.dir, name))
        os.rmdir(self.dir)

    def write(self, payload):
        with open(self.path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, ensure_ascii=False)

    def test_missing_file_is_not_an_exception(self):
        info = data.read_route_names(self.path)
        self.assertEqual(info['names'], [])
        self.assertEqual(info['active'], '')
        self.assertIn('还没生成', info['error'])

    def test_lists_routes_with_points(self):
        self.write({'active_route': 'main2', 'routes': {
            'main2': {'points': [{'point_id': 'p1'}]},
            'empty': {'points': []},
            'demo': {'points': [{'point_id': 'a'}]},
        }})
        info = data.read_route_names(self.path)
        self.assertEqual(info['names'], ['demo', 'main2'])
        self.assertEqual(info['active'], 'main2')
        self.assertEqual(info['error'], '')

    def test_active_not_in_names_is_cleared(self):
        self.write({'active_route': 'gone', 'routes': {'main': {'points': [1]}}})
        info = data.read_route_names(self.path)
        self.assertEqual(info['active'], '')

    def test_broken_json_reports_reason(self):
        with open(self.path, 'w', encoding='utf-8') as handle:
            handle.write('{not json')
        info = data.read_route_names(self.path)
        self.assertEqual(info['names'], [])
        self.assertTrue(info['error'])

    def test_page_and_server_contract(self):
        with open(os.path.join(ROOT, 'fishbot_console', 'page.py'),
                  encoding='utf-8') as handle:
            page = handle.read()
        with open(os.path.join(ROOT, 'fishbot_console', 'server.py'),
                  encoding='utf-8') as handle:
            server = handle.read()
        self.assertIn('async function loadRoutes()', page)
        self.assertIn('loadRoutes();', page)
        self.assertIn('先选一条路线再开始巡检', page)
        self.assertIn("'/api/route_names'", server)


if __name__ == '__main__':
    unittest.main()
