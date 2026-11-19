# -*- coding: utf-8 -*-
"""静态契约：``_tick_once`` 里用到的每个 ``self._x`` 都必须在 ``__init__`` 里赋过值。

这条用例是拿真机事故换来的（0.2.13）：``self._block_closed_at`` 只写了读取、
没有初始化，单元测试全绿、一启动就每拍抛 ``AttributeError``，探索直接停摆。
"""
import ast
import io
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(os.path.dirname(HERE), 'mybot_explore', 'node.py')

# 这些属性允许懒初始化（用 getattr 兜底或首次赋值），不算错
ALLOWED = {'_warned', '_slip_dbg_at', '_tf', '__dict__'}


class TickAttributesInitializedTest(unittest.TestCase):

    def setUp(self):
        self.source = io.open(NODE, encoding='utf-8').read()
        self.tree = ast.parse(self.source)
        self.cls = next(n for n in self.tree.body
                        if isinstance(n, ast.ClassDef) and n.name == 'ExplorerNode')

    def _method(self, name):
        return next(n for n in self.cls.body
                    if isinstance(n, ast.FunctionDef) and n.name == name)

    def _attrs(self, node, skip_calls=False):
        called = set()
        if skip_calls:
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute):
                    called.add(sub.func.attr)
        out = set()
        for sub in ast.walk(node):
            if isinstance(sub, ast.Attribute) and isinstance(sub.value, ast.Name):
                if sub.value.id == 'self' and sub.attr not in called:
                    out.add(sub.attr)
        return out

    def test_every_attribute_used_in_tick_is_initialized(self):
        initialized = self._attrs(self._method('__init__'), skip_calls=True)
        used = self._attrs(self._method('_tick_once'), skip_calls=True)
        missing = sorted(a for a in used - initialized if a not in ALLOWED)
        self.assertEqual(
            missing, [],
            '这些属性在 _tick_once 里用了，但 __init__ 没赋初值：%s' % missing)

    def test_block_closed_at_is_initialized(self):
        init_src = ast.get_source_segment(self.source, self._method('__init__')) or ''
        self.assertIn('self._block_closed_at', init_src,
                      '__init__ 里必须给 _block_closed_at 赋初值（0.2.13 事故）')


if __name__ == '__main__':
    unittest.main()
