#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""「改坏了」示范：往沙盒源码里打一个故意的 bug，让你练定位。

    python3 break_demo.py list
    python3 break_demo.py apply <名字>     # 打坏 + 自动编译
    python3 break_demo.py restore          # 还原全部改动
    python3 break_demo.py show <名字>      # 看这个 bug 是什么（先自己猜）

每个 bug 都有 .orig 备份，restore 一定回得去。
"""
import argparse
import glob
import io
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simcore                                          # noqa: E402

WS_SRC = os.path.join(simcore.WS, "src")


# ------------------------------------------------------------------ 工具
def find_pkg(suffix):
    """在 ws/src 里按后缀找包目录（兼容 fishbot_/mybot_ 两种命名）。"""
    matches = [p for p in glob.glob(os.path.join(WS_SRC, "*" + suffix))
               if os.path.isdir(p)]
    if not matches:
        raise SystemExit("找不到 *%s 包，先在沙盒里 init" % suffix)
    return matches[0]


def read(path):
    with io.open(path, encoding="utf-8") as fh:
        return fh.read()


def write(path, text):
    tmp = path + ".tmp"
    with io.open(tmp, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    os.replace(tmp, path)


def backup(path):
    if not os.path.exists(path + ".orig"):
        with io.open(path + ".orig", "w", encoding="utf-8", newline="\n") as fh:
            fh.write(read(path))


def patch(path, old, new, note):
    backup(path)
    text = read(path)
    if new in text:
        print("  已经是改过的状态，跳过")
        return
    if old not in text:
        raise SystemExit("锚点没找到（源码可能和你手里的版本不同）：%s" % note)
    write(path, text.replace(old, new, 1))
    print("  已改：%s" % path)


def build(packages):
    code = simcore.main(["build"] + packages)
    return code


# ------------------------------------------------------------------ 三个 bug
def break_count():
    """前沿候选永远为 0：count_unknown 把「未知」判成了「空闲」。"""
    pkg = find_pkg("_explore")
    path = os.path.join(pkg, os.path.basename(pkg), "frontier.py")
    print("① 让前沿检测一个都找不出来（症状：日志一直「前沿 0 个」，车不动）")
    patch(path,
          "            if grid.is_unknown(gx + dx, gy + dy):",
          "            if False and grid.is_unknown(gx + dx, gy + dy):",
          "frontier_cells 的未知邻居判断")
    return [os.path.basename(pkg)]


def break_slip():
    """打滑永远判不出：把 map→odom 阈值抬到天上。"""
    pkg = find_pkg("_explore")
    path = os.path.join(pkg, os.path.basename(pkg), "slip.py")
    print("② 让打滑检测永远不触发（症状：顶住桌腿还在刷「去 frontier」）")
    patch(path,
          "    'slip_map_odom_min_m': 0.08,",
          "    'slip_map_odom_min_m': 100.0,",
          "DEFAULTS.slip_map_odom_min_m")
    return [os.path.basename(pkg)]


def break_import():
    """启动即崩：少 import 一个类型（症状：节点起来 1 秒就 exit code 1）。"""
    pkg = find_pkg("_explore")
    path = os.path.join(pkg, os.path.basename(pkg), "node.py")
    print("③ 让它启动就崩（症状：日志只有一行 traceback，进程退出码 1）")
    patch(path,
          "from std_msgs.msg import Bool, String",
          "from std_msgs.msg import String",
          "node.py 的 Bool 导入")
    return [os.path.basename(pkg)]


BREAKS = {
    "no-frontier": (break_count, "前沿永远 0 个"),
    "no-slip": (break_slip, "打滑永远不触发"),
    "crash-import": (break_import, "启动即崩"),
}


# ------------------------------------------------------------------ 命令
def do_list(args):
    print("可用的「改坏了」示范：")
    for name, (_, desc) in BREAKS.items():
        print("  %-13s %s" % (name, desc))
    print("\n用法：python3 break_demo.py apply no-frontier")
    print("恢复：python3 break_demo.py restore")


def do_apply(args):
    if args.name not in BREAKS:
        raise SystemExit("没有这个示范：%s（用 list 看）" % args.name)
    func, desc = BREAKS[args.name]
    print("打坏：%s" % desc)
    packages = func()
    print("编译沙盒…")
    build(packages)
    print("\n现在去窗口里跑一次，看症状，再回来看答案：")
    print("  python3 break_demo.py show %s" % args.name)
    print("  python3 break_demo.py restore     # 修好")


def do_show(args):
    if args.name not in BREAKS:
        raise SystemExit("没有这个示范：%s" % args.name)
    _, desc = BREAKS[args.name]
    print("症状：%s" % desc)
    print("原因：%s" % (BREAKS[args.name][0].__doc__ or "").strip())
    print("\n怎么找：")
    print("  1. 看日志最后一屏，先分清「没数据」还是「有数据但不动」")
    print("  2. ros2 topic hz /explore/status  —— 有没有在跑")
    print("  3. 看 /explore/status 里的 map_ready / scan_ready / pose_ready")
    print("  4. 还不行就用 diff 看源码：git -C ws/src diff 或对比 .orig 文件")


def do_restore(args):
    count = 0
    for path in glob.glob(os.path.join(WS_SRC, "**", "*.orig"), recursive=True):
        target = path[:-5]
        with io.open(path, encoding="utf-8") as fh:
            text = fh.read()
        write(target, text)
        os.remove(path)
        count += 1
        print("  还原：%s" % os.path.basename(target))
    if not count:
        print("没有需要还原的文件")
        return
    print("重新编译沙盒…")
    build([])


def main(argv=None):
    parser = argparse.ArgumentParser(prog="break_demo")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list").set_defaults(func=do_list)
    p_apply = sub.add_parser("apply")
    p_apply.add_argument("name")
    p_apply.set_defaults(func=do_apply)
    p_show = sub.add_parser("show")
    p_show.add_argument("name")
    p_show.set_defaults(func=do_show)
    sub.add_parser("restore").set_defaults(func=do_restore)
    args = parser.parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
