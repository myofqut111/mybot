#!/usr/bin/env python3
"""静态检查：找 ``self.<name>(...)`` 调用但类里没有同名方法的情况。

只做这一件事，因为 ``AttributeError: 'X' object has no attribute '_y'``
是 py_compile 和单元测试都抓不到的一类错误（测试里没 import 到 rclpy 就
整个模块跳过，于是我改了一个参数读取函数却忘了写实现，直到上机才崩）。

用法::

    python3 tools/check_attrs.py .          # 检查当前目录下所有模块
    python3 tools/check_attrs.py fishbot_patrol_v2
"""
import ast
import os
import sys


def _class_methods(cls, known_attrs):
    """收集类里定义的方法名、赋值过的属性名，以及基类名。"""
    methods = set()
    attrs = set(known_attrs)
    bases = [ast.unparse(b).split('.')[-1] for b in cls.bases]
    for node in ast.walk(cls):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            methods.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == 'self'):
                    attrs.add(target.attr)
        elif isinstance(node, ast.AnnAssign):
            target = node.target
            if (isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == 'self'):
                attrs.add(target.attr)
        elif isinstance(node, ast.AugAssign):
            target = node.target
            if (isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == 'self'):
                attrs.add(target.attr)
    return methods, attrs, bases


def check_file(path, class_index, problems):
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), filename=path)

    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        methods, attrs, bases = _class_methods(cls, set())
        # 继承来的方法：只看本包内已知的类
        inherited = set()
        seen = set()
        stack = list(bases)
        while stack:
            base = stack.pop()
            if base in seen or base not in class_index:
                continue
            seen.add(base)
            base_methods, base_attrs, base_bases = class_index[base]
            inherited |= base_methods
            attrs |= base_attrs
            stack.extend(base_bases)

        for node in ast.walk(cls):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not (isinstance(func, ast.Attribute)
                    and isinstance(func.value, ast.Name)
                    and func.value.id == 'self'):
                continue
            name = func.attr
            # 只看私有方法/属性：基类（rclpy.Node、unittest.TestCase）提供的
            # 公开方法没法静态索引，检查它们只会产生噪音。
            if not name.startswith('_') or name.startswith('__'):
                continue
            if name in methods or name in inherited or name in attrs:
                continue
            problems.append(
                '%s:%d %s.%s() 调用了但类里没有定义'
                % (path, node.lineno, cls.name, name))


def main(argv):
    roots = argv[1:] or ['.']
    files = []
    for root in roots:
        if os.path.isfile(root):
            files.append(root)
            continue
        for dirpath, _dirnames, filenames in os.walk(root):
            if '__pycache__' in dirpath or '/build/' in dirpath:
                continue
            for name in filenames:
                if name.endswith('.py'):
                    files.append(os.path.join(dirpath, name))

    class_index = {}
    for path in files:
        try:
            with open(path, encoding='utf-8') as handle:
                tree = ast.parse(handle.read(), filename=path)
        except SyntaxError as exc:
            print('%s: 语法错误 %s' % (path, exc))
            return 1
        for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
            methods, attrs, bases = _class_methods(cls, set())
            class_index.setdefault(cls.name, (methods, attrs, bases))

    problems = []
    for path in files:
        check_file(path, class_index, problems)

    if problems:
        print('发现 %d 处可疑调用：' % len(problems))
        for line in problems:
            print('  ' + line)
        return 1
    print('OK：%d 个文件，没有未定义的方法调用' % len(files))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
