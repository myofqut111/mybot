"""静态检查：找出模块里「用了但没定义」的全局名字。

本机没有 rclpy / nav2_msgs，node.py 只能做静态检查。这个小工具用 ast 模拟
一次作用域解析，专门抓 ``NameError: name 'X' is not defined`` 这一类低级错误
（比如忘了 import 的 ActionClient）。

用法::

    python check_names.py fishbot_patrol_v2/node.py ...
"""

import ast
import builtins
import sys


def _bound_names(node, bound):
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


def check(path):
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), filename=path)

    module_bound = set(dir(builtins)) | {'__name__', '__file__', '__doc__'}
    _bound_names(tree, module_bound)

    problems = []
    for func in [node for node in ast.walk(tree)
                 if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        local = set()
        _bound_names(func, local)
        # 嵌套函数 / 类的名字也在 local 里，够了
        for child in ast.walk(func):
            if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                if child.id not in local and child.id not in module_bound:
                    problems.append((func.name, child.id, child.lineno))
    return problems


def main(argv):
    failed = False
    for path in argv[1:]:
        problems = check(path)
        if not problems:
            print('OK   %s' % path)
            continue
        failed = True
        print('FAIL %s' % path)
        for func, name, line in sorted(problems, key=lambda item: item[2]):
            print('     line %-5d %-24s in %s()' % (line, name, func))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
