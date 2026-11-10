# -*- coding: utf-8 -*-
"""把 mybot_console 的版本号统一改掉。

为什么用脚本而不是手改：版本号同时出现在 package.xml 和 setup.py 里，两处不一致时
`ros2 pkg xml` 和 `pip show` 会给出不同答案，排查时会怀疑人生。
"""
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))


def bump(new_version):
    changed = []
    xml_path = os.path.join(ROOT, 'package.xml')
    with io.open(xml_path, encoding='utf-8') as handle:
        text = handle.read()
    text = re.sub(r'<version>[^<]*</version>',
                  '<version>%s</version>' % new_version, text)
    with io.open(xml_path, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(text)
    changed.append(xml_path)

    setup_path = os.path.join(ROOT, 'setup.py')
    with io.open(setup_path, encoding='utf-8') as handle:
        text = handle.read()
    text = re.sub(r"version='[^']*'", "version='%s'" % new_version, text)
    with io.open(setup_path, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(text)
    changed.append(setup_path)
    return changed


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('usage: _bump_version.py 0.4.7')
        sys.exit(2)
    for path in bump(sys.argv[1]):
        print('bumped %s' % path)
