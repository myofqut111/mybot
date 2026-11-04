# -*- coding: utf-8 -*-
"""静态检查：每个模块里"用了但没定义"的名字。

为什么要有这个测试：本机没有 rclpy / nav2_msgs，`ros_bridge.py` 这类文件
import 都会失败，装不起来的错误在本机是**看不见**的——要等你拷到虚拟机上
`ros2 launch` 才炸。

真实事故：`ros_bridge.py` 里写了 `PoseWithCovarianceStamped()`，但 import 那行
写的是 `Pose, TwistWithCovarianceStamped`，于是控制台一启动就
`NameError: name 'PoseWithCovarianceStamped' is not defined`，而 132 个用例全绿。

这个测试用 ast 做一次作用域解析，专门抓这类名字错误，不需要 import 任何 ROS 包。
"""
import ast
import builtins
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(os.path.dirname(HERE), 'fishbot_console')
ROOT = os.path.dirname(os.path.dirname(HERE))       # .../src 这一层

BUILTIN = set(dir(builtins)) | {'__name__', '__file__', '__doc__'}


def _bound(node, bound):
    """把 node 里所有会被绑定的名字收进 bound。"""
    for child in ast.walk(node):
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bound.add(child.name)
        elif isinstance(child, ast.Name) and isinstance(child.ctx, (ast.Store, ast.Del)):
            bound.add(child.id)
        elif isinstance(child, ast.arg):
            bound.add(child.arg)
        elif isinstance(child, ast.Import):
            for alias in child.names:
                bound.add((alias.asname or alias.name).split('.')[0])
        elif isinstance(child, ast.ImportFrom):
            for alias in child.names:
                bound.add(alias.asname or alias.name)
        elif isinstance(child, ast.ExceptHandler) and child.name:
            bound.add(child.name)
        elif isinstance(child, ast.Global):
            bound.update(child.names)


def undefined_names(path):
    """返回 [(函数名, 名字, 行号)]；空列表表示没发现问题。"""
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), filename=path)

    module_bound = set(BUILTIN)
    _bound(tree, module_bound)

    problems = []
    for func in [node for node in ast.walk(tree)
                 if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        local = set()
        _bound(func, local)
        for child in ast.walk(func):
            if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                if child.id not in local and child.id not in module_bound:
                    problems.append((func.name, child.id, child.lineno))
    return problems


def module_files():
    files = []
    for dirpath, dirs, names in os.walk(PKG_DIR):
        dirs[:] = [d for d in dirs if d != '__pycache__']
        for name in sorted(names):
            if name.endswith('.py'):
                files.append(os.path.join(dirpath, name))
    return files


class TestNoUndefinedNames(unittest.TestCase):
    def test_found_the_package(self):
        # 防止路径写错导致"一个文件都没检查"却显示通过
        self.assertTrue(os.path.isdir(PKG_DIR), PKG_DIR)
        self.assertGreaterEqual(len(module_files()), 10)

    def test_every_module_has_no_undefined_names(self):
        failures = []
        for path in module_files():
            problems = undefined_names(path)
            if problems:
                rel = os.path.relpath(path, ROOT)
                for func, name, line in sorted(problems, key=lambda i: i[2]):
                    failures.append('%s:%d %s 在 %s() 里' % (rel, line, name, func))
        self.assertEqual([], failures, '\n'.join(failures))

    def test_catches_a_real_missing_import(self):
        """反向验证：故意缺 import 时这个检查必须报出来。"""
        import tempfile
        source = 'def f():\n    return MissingThing()\n'
        with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False,
                                         encoding='utf-8') as handle:
            handle.write(source)
            path = handle.name
        try:
            problems = undefined_names(path)
        finally:
            os.unlink(path)
        self.assertEqual(1, len(problems))
        self.assertEqual('MissingThing', problems[0][1])


if __name__ == '__main__':
    unittest.main()
