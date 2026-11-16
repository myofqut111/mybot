# -*- coding: utf-8 -*-
"""0.2.7 契约测试：SaveMap.name 必须写 .data；存图异常不能拖垮探索"""
import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = os.path.join(ROOT, 'mybot_explore', 'node.py')


class SaveMapContractCase(unittest.TestCase):
    def setUp(self):
        with open(NODE, encoding='utf-8') as handle:
            self.source = handle.read()

    def test_uses_string_data_field(self):
        # slam_toolbox 的 SaveMap.srv 里 name 是 std_msgs/String，
        # 直接 request.name = base 会抛 "must be a sub message of type 'String'"
        self.assertIn('request.name.data = base', self.source)
        self.assertNotIn('request.name = base', self.source)

    def test_auto_save_is_wrapped(self):
        # 存图失败只该打 warning，不能把探索带停
        self.assertIn('def _auto_save_impl', self.source)
        start = self.source.index('def _auto_save(self, grid):')
        end = self.source.index('def _auto_save_impl', start)
        block = self.source[start:end]
        self.assertIn('try:', block)
        self.assertIn('except Exception', block)
        self.assertIn('不影响探索', block)


if __name__ == '__main__':
    unittest.main()
