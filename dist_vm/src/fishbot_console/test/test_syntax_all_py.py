# -*- coding: utf-8 -*-
"""每个 .py 都必须能 compile 通过——包括 setup.py、launch/ 和 test/。

为什么单独一条：colcon build 第一步就是跑 `setup.py --dry-run`，它语法错了整个包
根本装不上。而 unittest 只 import 包里的模块，永远碰不到 setup.py 和 launch 文件，
所以「测试全绿、build 失败」是可能的——这次就是这么发生的（一次文本替换把注释的
换行吃掉了，`data_files=[` 少了个收尾的 `]`，本地 139 个用例照样全过）。
"""
import os
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = ('__pycache__', '.pytest_cache', 'build', 'install', 'log')


def iter_python_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for name in filenames:
            if name.endswith('.py'):
                yield os.path.join(dirpath, name)


class TestAllFilesCompile(unittest.TestCase):
    def test_every_python_file_compiles(self):
        files = list(iter_python_files(PACKAGE_ROOT))
        # 至少要有模块 + setup.py + 若干测试，否则说明路径找错了，测试会假绿
        self.assertGreaterEqual(len(files), 15,
                                'only found %d .py files under %s'
                                % (len(files), PACKAGE_ROOT))
        names = {os.path.basename(path) for path in files}
        for expected in ('setup.py', 'node.py', 'ros_bridge.py',
                         'console.launch.py', 'mock.launch.py',
                         'test_server.py', 'test_no_undefined_names.py'):
            self.assertIn(expected, names,
                          'not scanned: %s (路径或过滤条件不对)' % expected)

        failures = []
        for path in files:
            with open(path, encoding='utf-8') as handle:
                source = handle.read()
            try:
                compile(source, path, 'exec')
            except SyntaxError as error:
                failures.append('%s:%s %s' % (
                    os.path.relpath(path, PACKAGE_ROOT), error.lineno,
                    error.msg))
        self.assertEqual(failures, [], '发现语法错误：\n' + '\n'.join(failures))

    def test_package_xml_is_readable(self):
        """package.xml 被别人用文本替换改坏过一次（整段描述变成乱码）。"""
        path = os.path.join(PACKAGE_ROOT, 'package.xml')
        with open(path, encoding='utf-8') as handle:
            text = handle.read()
        self.assertIn('<package format="3">', text)
        self.assertIn('<name>fishbot_console</name>', text)
        # 乱码的典型特征：替换字符
        self.assertNotIn('\ufffd', text, 'package.xml 里有替换字符，编码被写坏了')
        # 描述里不该出现连续的非 ASCII 乱码块（正常应只有维护者等少量中文）
        import re
        for match in re.findall(r'>([^<>\n]{20,})<', text):
            self.assertLess(match.count('?'), 3,
                            '疑似乱码：%r' % match[:60])


if __name__ == '__main__':
    unittest.main()
