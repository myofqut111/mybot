#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把仓库里的 mybot_* 代码打成一份"改回 fishbot_*"的部署包。

为什么要改回去：虚拟机和板子上装的还是 fishbot_*（用户决定不重装），
仓库对外展示用 mybot_*。这个脚本负责两边对账：

    mybot_console      -> fishbot_console
    mybot_patrol_v2    -> fishbot_patrol_v2
    ~/.ros/mybot_*.json -> ~/.ros/fishbot_*.json   （保住机器上已有的路线/基线）

用法：
    python tools/sync_vm.py            # 生成 dist_vm/fishbot_src.tar.gz
    python tools/sync_vm.py --out DIR  # 指定输出目录（调试用，不打包）

只做文本替换，跳过二进制（.pyc/.png/.onnx/.pt/.pgm/.so），
并且跳过 __pycache__ / build / install / .git。
"""
from __future__ import print_function

import argparse
import io
import os
import shutil
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                    # 仓库根
SRC = os.path.join(ROOT, 'src')

# 要同步的包（顺序无所谓，都是独立的 ament 包）
PACKAGES = [
    'mybot_console',
    'mybot_patrol_v2',
    'mybot_safety',
    'mybot_vision',
    'mybot_patrol_interfaces',
    'mybot_patrol_route_manager',
    'mybot_patrol_task_manager',
    'mybot_navigation',
    'mybot_bringup',
    'mybot_description',
    'mybot_patrol_executor',
    'mybot_explore',
]

TEXT_EXT = {
    '.py', '.xml', '.yaml', '.yml', '.md', '.txt', '.cfg', '.sh', '.rviz',
    '.launch', '.srv', '.msg', '.action', '.json', '.in', '.cmake', '.cpp',
    '.urdf', '.xacro', '.py.in',
    '.h', '.hpp', '.js', '.html', '.css',
}
SKIP_DIRS = {'__pycache__', 'build', 'install', 'log', '.git', 'node_modules'}

# 顺序有讲究：先把带下划线的包名换掉，再兜底换剩下的裸 mybot（如 mybot.urdf、<title>mybot 控制台</title>）
RENAMES = [
    ('mybot_', 'fishbot_'),
    ('mybot', 'fishbot'),
    ('Mybot', 'Fishbot'),
    ('MYBOT', 'FISHBOT'),
]


skipped = []


def rename_text(text):
    for old, new in RENAMES:
        text = text.replace(old, new)
    return text


def rename_path(path):
    return path.replace('mybot', 'fishbot')


def copy_tree(pkg, dst_root, with_models=False):
    """把一个包按 fishbot_ 名字写进 dst_root，返回写出的文件数。

    ``with_models=False`` 跳过 >5 MB 的模型权重（.pt/.onnx/.mnn/.bin/.engine）——
    这些文件机器上本来就有，每次同步没必要来回传。
    """
    heavy = ('.pt', '.onnx', '.mnn', '.bin', '.engine')
    src_pkg = os.path.join(SRC, pkg)
    if not os.path.isdir(src_pkg):
        print('  跳过不存在的包：%s' % pkg)
        return 0
    dst_pkg = os.path.join(dst_root, rename_path(pkg))
    written = 0
    for base, dirs, files in os.walk(src_pkg):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        rel = os.path.relpath(base, src_pkg)
        rel = '' if rel == '.' else rel
        dst_dir = os.path.join(dst_pkg, rename_path(rel)) if rel else dst_pkg
        if not os.path.isdir(dst_dir):
            os.makedirs(dst_dir)
        for name in files:
            if name.endswith('.pyc'):
                continue
            src_file = os.path.join(base, name)
            ext = os.path.splitext(name)[1].lower()
            dst_file = os.path.join(dst_dir, rename_path(name))
            if not with_models and ext in heavy and \
                    os.path.getsize(src_file) > 5 * 1024 * 1024:
                skipped.append(os.path.relpath(src_file, SRC))
                continue
            if ext not in TEXT_EXT:
                shutil.copy2(src_file, dst_file)
                written += 1
                continue
            try:
                text = io.open(src_file, encoding='utf-8').read()
            except UnicodeDecodeError:
                shutil.copy2(src_file, dst_file)
                written += 1
                continue
            io.open(dst_file, 'w', encoding='utf-8', newline='\n').write(
                rename_text(text))
            written += 1
    return written


def make_tar(dst_root, tar_path):
    names = sorted(os.listdir(dst_root))
    with tarfile.open(tar_path, 'w:gz') as tar:
        for name in names:
            tar.add(os.path.join(dst_root, name), arcname=name)
    return tar_path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(ROOT, 'dist_vm'),
                    help='输出目录（默认仓库根 dist_vm）')
    ap.add_argument('--with-models', action='store_true',
                    help='连模型权重一起打包（默认跳过 >5 MB 的 .pt/.onnx/.mnn）')
    args = ap.parse_args(argv)

    out = os.path.abspath(args.out)
    stage = os.path.join(out, 'src')
    if os.path.isdir(stage):
        shutil.rmtree(stage)
    os.makedirs(stage)

    total = 0
    for pkg in PACKAGES:
        n = copy_tree(pkg, stage, with_models=args.with_models)
        if n:
            print('  %-32s -> %-32s %d 个文件' % (pkg, rename_path(pkg), n))
        total += n

    tar_path = make_tar(stage, os.path.join(out, 'fishbot_src.tar.gz'))
    size = os.path.getsize(tar_path) / 1024.0
    print('\n打包完成：%s（%.1f KB，共 %d 个文件）' % (tar_path, size, total))
    print('部署：tools/sync_vm.sh <VM地址>')
    return 0


if __name__ == '__main__':
    sys.exit(main())
