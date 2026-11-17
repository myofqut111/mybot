#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mybot_sim 沙盒核心。

设计约束（见《沙盒方案》）：
  * 窗口只是遥控器，所有东西都是普通文件
  * 场景 = scenarios/ 下的可执行脚本（.sh 或 .launch.py）
  * 参数 = config/ 下的 yaml，扫到什么就是什么，不写死列表
  * 故障 = faults/ 下的 .py，由 fault_bridge 统一转发
  * 每个子进程单独注入 domain / 端口，不靠全局 export

用法：
    python3 simcore.py init [--src PATH] [--fresh]
    python3 simcore.py run SCENARIO
    python3 simcore.py stop
    python3 simcore.py reset
    python3 simcore.py list
    python3 simcore.py status
"""
import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

HOME = os.path.expanduser("~")
ROOT = os.environ.get("MYBOT_SIM_HOME") or os.path.join(HOME, "mybot_sim")
SRC_DEFAULT = os.path.join(HOME, "fishbot_full_ws", "src")
WS = os.path.join(ROOT, "ws")
STATE = os.path.join(ROOT, "sim.json")

DIRS = ("scenarios", "faults", "config", "data", "logs")
SKIP_DIRS = {"__pycache__", "build", "install", "log", ".git"}


# --------------------------------------------------------------- 基础工具
def ensure_dirs():
    for name in DIRS:
        os.makedirs(os.path.join(ROOT, name), exist_ok=True)
    os.makedirs(os.path.join(WS, "src"), exist_ok=True)


def load_state():
    try:
        with open(STATE, encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return {}


def save_state(state):
    tmp = STATE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(state, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, STATE)


def port_free(port):
    with socket.socket() as sock:
        sock.settimeout(0.2)
        return sock.connect_ex(("127.0.0.1", port)) != 0


def alloc(state):
    """分配 domain 与端口：42 / 8180 起，被占就往后挪。"""
    used_domains = {int(v) for v in state.get("all_domains", [])}
    domain = int(state.get("domain", 42))
    while domain in used_domains and domain < 200:
        domain += 1
    port = int(state.get("http_port", 8180))
    while not port_free(port) and port < 9000:
        port += 1
    state["domain"] = domain
    state["http_port"] = port
    return state


def child_env(domain, port, extra=None):
    """每个子进程单独注入——不依赖父进程的全局 export。"""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["MYBOT_SIM"] = "1"
    env["MYBOT_SIM_ROOT"] = ROOT
    env["MYBOT_HTTP_PORT"] = str(port)
    env.pop("ROS_LOCALHOST_ONLY", None)
    if extra:
        env.update({k: str(v) for k, v in extra.items()})
    return env


# --------------------------------------------------------------- 目录扫描
def list_scenarios():
    out = []
    base = os.path.join(ROOT, "scenarios")
    if not os.path.isdir(base):
        return out
    for name in sorted(os.listdir(base)):
        if name.startswith(".") or not name.endswith((".sh", ".py")):
            continue
        path = os.path.join(base, name)
        out.append({"name": name, "path": path, "executable": os.access(path, os.X_OK)})
    return out


def list_faults():
    out = []
    base = os.path.join(ROOT, "faults")
    if not os.path.isdir(base):
        return out
    for name in sorted(os.listdir(base)):
        if not name.endswith(".py") or name.startswith("_"):
            continue
        path = os.path.join(base, name)
        info = {"name": name[:-3], "path": path, "label": name[:-3],
                "topic": "", "args": {}, "error": ""}
        try:
            source = open(path, encoding="utf-8").read()
            ns = {}
            # 只取模块级的几个常量，不执行 inject()
            exec(compile(source, path, "exec"), {"__name__": "fault_meta"}, ns)
            info["label"] = ns.get("LABEL", info["label"])
            info["topic"] = ns.get("TOPIC", "")
            info["args"] = {k: v for k, v in (ns.get("ARGS") or {}).items()}
        except Exception as error:               # noqa: BLE001
            info["error"] = "%s: %s" % (type(error).__name__, error)
        out.append(info)
    return out


def list_params():
    """扫 config/*.yaml，返回 [{file, key, path, value, kind}]。

    只处理标量；数组/嵌套结构不渲染（按方案 A），交给用户改文件。
    """
    import yaml
    out = []
    base = os.path.join(ROOT, "config")
    if not os.path.isdir(base):
        return out
    for name in sorted(os.listdir(base)):
        if not name.endswith((".yaml", ".yml")):
            continue
        path = os.path.join(base, name)
        try:
            with open(path, encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
        except Exception as error:               # noqa: BLE001
            out.append({"file": name, "key": "<读不出来>", "path": "",
                        "value": "%s: %s" % (type(error).__name__, error),
                        "kind": "error"})
            continue
        out.extend(_walk(name, data, []))
    return out


def _walk(filename, node, prefix):
    rows = []
    if isinstance(node, dict):
        for key, value in node.items():
            rows.extend(_walk(filename, value, prefix + [str(key)]))
    elif isinstance(node, list):
        rows.append({"file": filename, "key": ".".join(prefix[-1:]),
                     "path": ".".join(prefix), "value": "数组，需改 yaml",
                     "kind": "complex"})
    else:
        kind = "bool" if isinstance(node, bool) else (
            "number" if isinstance(node, (int, float)) else "text")
        rows.append({"file": filename, "key": ".".join(prefix[-1:]),
                     "path": ".".join(prefix), "value": node, "kind": kind})
    return rows


def set_param(filename, path, value):
    """按路径回写一个标量；保留其余内容与注释顺序尽量不变。"""
    import yaml
    full = os.path.join(ROOT, "config", filename)
    with open(full, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    keys = path.split(".")
    node = data
    for key in keys[:-1]:
        node = node[key]
    old = node[keys[-1]]
    if isinstance(old, bool):
        new = str(value).lower() in ("1", "true", "yes", "on")
    elif isinstance(old, int) and not isinstance(old, bool):
        new = int(float(value))
    elif isinstance(old, float):
        new = float(value)
    else:
        new = value
    node[keys[-1]] = new
    tmp = full + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    os.replace(tmp, full)
    return old, new


# --------------------------------------------------------------- 源码复制
def copy_src(src):
    target = os.path.join(WS, "src")
    if not os.path.isdir(src):
        raise SystemExit("源码目录不存在：%s" % src)
    if os.path.isdir(target) and os.listdir(target):
        backup = os.path.join(ROOT, "backup",
                              time.strftime("%Y%m%d_%H%M%S") + ".tar.gz")
        os.makedirs(os.path.dirname(backup), exist_ok=True)
        subprocess.run(["tar", "-czf", backup, "-C", WS, "src"], check=False)
        print("已备份原 ws/src -> %s" % backup)
        shutil.rmtree(target)
    os.makedirs(target, exist_ok=True)
    for name in os.listdir(src):
        if name in SKIP_DIRS:
            continue
        source = os.path.join(src, name)
        dest = os.path.join(target, name)
        if os.path.isdir(source):
            shutil.copytree(source, dest,
                            ignore=shutil.ignore_patterns(*SKIP_DIRS))
        else:
            shutil.copy2(source, dest)
    count = sum(len(files) for _, _, files in os.walk(target))
    print("源码已复制 -> %s（%d 个文件）" % (target, count))


TEMPLATES = {
    "scenarios/sim_explore.sh": """#!/usr/bin/env bash
# 场景：mock 探索（正常）
set +u
source /opt/ros/humble/setup.bash
source "$HOME/fishbot_full_ws/install/setup.bash"
SIM_WS="$MYBOT_SIM_ROOT/ws"
if [ -f "$SIM_WS/install/setup.bash" ]; then
  source "$SIM_WS/install/setup.bash"     # 沙盒里改过的代码覆盖上来
fi
BRIDGE="$(python3 -c 'import os;print(os.path.expanduser("~/fishbot_full_ws/tools/sandbox/fault_bridge.py"))')"
MAP="$(python3 -c 'import os;print(os.path.expanduser("~/maps/mock_partial.yaml"))')"
ros2 run fishbot_console mock_robot --ros-args -r /scan:=/scan_raw &
python3 "$BRIDGE" --ros-args -p root:="$MYBOT_SIM_ROOT" &
sleep 2
exec ros2 run fishbot_explore explore_node --ros-args \\
  -p drive:=true -p autostart:=true -p map_yaml:="$MAP" \\
  -p max_duration_s:=120.0 -p save_map_dir:="$MYBOT_SIM_ROOT/data"
""",
    "scenarios/sim_explore_slip.sh": """#!/usr/bin/env bash
# 场景：探索 + 打滑底盘（轮子转但车不动）
set +u
source /opt/ros/humble/setup.bash
source "$HOME/fishbot_full_ws/install/setup.bash"
SIM_WS="$MYBOT_SIM_ROOT/ws"
if [ -f "$SIM_WS/install/setup.bash" ]; then
  source "$SIM_WS/install/setup.bash"     # 沙盒里改过的代码覆盖上来
fi
BRIDGE="$(python3 -c 'import os;print(os.path.expanduser("~/fishbot_full_ws/tools/sandbox/fault_bridge.py"))')"
MAP="$(python3 -c 'import os;print(os.path.expanduser("~/maps/mock_partial.yaml"))')"
ros2 run fishbot_console mock_robot --ros-args -r /scan:=/scan_raw &
python3 "$BRIDGE" --ros-args -p root:="$MYBOT_SIM_ROOT" &
sleep 2
exec ros2 run fishbot_explore explore_node --ros-args \\
  -p drive:=true -p autostart:=true -p map_yaml:="$MAP" \\
  -p max_duration_s:=90.0 -p slip_scan_change_ratio:=0.15 \\
  -p save_map_dir:="$MYBOT_SIM_ROOT/data"
""",
    "faults/drop.py": '''"""丢帧：按概率丢掉消息 / 或整段切断"""
LABEL = "丢帧"
TOPIC = "/scan_raw"
ARGS = {"rate": 0.1}


def inject(msg, args, ctx):
    if ctx.random() < float(args.get("rate", 0.0)):
        return None
    return msg
''',
    "faults/noise.py": '''"""加噪：给 ranges / 位置类字段加高斯噪声"""
import random
LABEL = "加噪"
TOPIC = "/scan_raw"
ARGS = {"sigma": 0.02}


def inject(msg, args, ctx):
    sigma = float(args.get("sigma", 0.0))
    if hasattr(msg, "ranges"):
        msg.ranges = [r + random.gauss(0.0, sigma) for r in msg.ranges]
    elif hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        pos = msg.pose.pose.position
        pos.x += random.gauss(0.0, sigma)
        pos.y += random.gauss(0.0, sigma)
    return msg
''',
    "faults/delay.py": '''"""延迟：攒够 delay_s 秒再放行"""
LABEL = "延迟"
TOPIC = "/scan_raw"
ARGS = {"delay_s": 3.0}


def inject(msg, args, ctx):
    return ctx.delay(msg, float(args.get("delay_s", 0.0)))
''',
}


SEED_CONFIGS = (
    ("fishbot_explore", "config/explore.yaml", "explore.yaml"),
    ("fishbot_patrol_v2", "config/patrol_v2.yaml", "patrol.yaml"),
    ("fishbot_console", "config/console.yaml", "console.yaml"),
)


def seed_configs(ws_install=None):
    """从已安装的 share 目录里挑几份 yaml 放到沙盒 config/（只补缺）。"""
    base = ws_install or os.path.join(HOME, "fishbot_full_ws", "install")
    copied = []
    for package, rel, target in SEED_CONFIGS:
        src = os.path.join(base, package, "share", package, rel)
        dst = os.path.join(ROOT, "config", target)
        if os.path.exists(dst) or not os.path.exists(src):
            continue
        shutil.copy2(src, dst)
        copied.append(target)
    return copied

def write_templates(force=False):
    for rel, text in TEMPLATES.items():
        path = os.path.join(ROOT, rel)
        if os.path.exists(path) and not force:
            continue
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
        if rel.endswith(".sh"):
            os.chmod(path, 0o755)


# --------------------------------------------------------------- 运行控制
def pid_file():
    return os.path.join(ROOT, "run.pid")


def read_pid():
    try:
        with open(pid_file(), encoding="utf-8") as fh:
            return int(fh.read().strip())
    except Exception:
        return None


def alive(pid):
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def do_init(args):
    state = load_state()
    ensure_dirs()
    alloc(state)
    save_state(state)
    if args.fresh or not os.listdir(os.path.join(WS, "src")):
        copy_src(args.src or SRC_DEFAULT)
    write_templates(force=args.fresh)
    seeded = seed_configs()
    if seeded:
        print("已放入示例参数文件：%s" % ", ".join(seeded))
    print("沙盒就绪：%s" % ROOT)
    print("  domain=%s  端口=%s" % (state["domain"], state["http_port"]))
    print("  场景 %d 个 / 故障 %d 个 / 参数文件 %d 个" % (
        len(list_scenarios()), len(list_faults()),
        len([n for n in os.listdir(os.path.join(ROOT, "config"))
             if n.endswith((".yaml", ".yml"))])))
    print("  下一步：python3 simcore.py run sim_explore.sh")


def do_run(args):
    ensure_dirs()
    state = load_state()
    alloc(state)
    save_state(state)
    script = os.path.join(ROOT, "scenarios", args.scenario)
    if not os.path.exists(script):
        raise SystemExit("没有这个场景：%s" % args.scenario)
    if alive(read_pid()):
        raise SystemExit("已经有一个场景在跑（pid=%s），先 stop" % read_pid())

    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(ROOT, "logs", stamp)
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "console.log")
    _snapshot(state, log_dir, args.scenario)

    command = ["bash", script] if script.endswith(".sh") else [script]
    env = child_env(state["domain"], state["http_port"])
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        command, cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)          # 独立进程组，方便整组收掉
    with open(pid_file(), "w") as fh:
        fh.write(str(proc.pid))
    save_state(dict(state, last_scenario=args.scenario, last_run=stamp))
    print("已启动 pid=%s 场景=%s" % (proc.pid, args.scenario))
    print("  日志：%s" % log_path)
    return None


def _snapshot(state, log_dir, scenario):
    """快照：生效参数（合并后）+ 启用的故障 + 场景名。"""
    try:
        import yaml
        merged = {}
        for name in sorted(os.listdir(os.path.join(ROOT, "config"))):
            if name.endswith((".yaml", ".yml")):
                with open(os.path.join(ROOT, "config", name), encoding="utf-8") as fh:
                    merged[name] = yaml.safe_load(fh) or {}
        with open(os.path.join(log_dir, "effective_params.yaml"), "w",
                  encoding="utf-8") as fh:
            yaml.safe_dump(merged, fh, allow_unicode=True, sort_keys=True)
    except Exception as error:               # noqa: BLE001
        print("快照参数失败（忽略）：%s" % error)
    with open(os.path.join(log_dir, "run.json"), "w", encoding="utf-8") as fh:
        json.dump({"scenario": scenario, "domain": state.get("domain"),
                   "http_port": state.get("http_port"),
                   "time": time.strftime("%Y-%m-%dT%H:%M:%S")},
                  fh, ensure_ascii=False, indent=2)


def do_stop(args):
    pid = read_pid()
    state = load_state()
    if not alive(pid):
        print("没有在跑的场景")
        if os.path.exists(pid_file()):
            os.remove(pid_file())
        return
    print("停止 pid=%s：SIGINT -> 等 2.5s -> SIGKILL" % pid)
    try:
        os.killpg(pid, signal.SIGINT)
    except OSError as error:
        print("发 SIGINT 失败：%s" % error)
    deadline = time.time() + 2.5
    while time.time() < deadline and alive(pid):
        time.sleep(0.1)
    if alive(pid):
        try:
            os.killpg(pid, signal.SIGKILL)
            print("已升级为 SIGKILL")
        except OSError as error:
            print("发 SIGKILL 失败：%s" % error)
    time.sleep(0.5)
    try:
        os.remove(pid_file())
    except OSError:
        pass
    # 只停本沙盒 domain 的 daemon；没 source 过 ROS 时 ros2 不在 PATH，走 login shell
    env = child_env(state.get("domain", 42), state.get("http_port", 8180))
    try:
        subprocess.run(["ros2", "daemon", "stop"], env=env,
                       capture_output=True, timeout=10)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        try:
            subprocess.run(
                ["bash", "-lc",
                 "source /opt/ros/humble/setup.bash && ros2 daemon stop"],
                env=env, capture_output=True, timeout=15)
        except Exception:                     # noqa: BLE001
            pass
    # 括号技巧：避免 pgrep 匹配到执行它自己的那条命令行
    left = subprocess.run(
        ["bash", "-lc", "pgrep -fc '[f]ishbot_explore|[m]ock_robot' || true"],
        capture_output=True, text=True).stdout.strip()
    print("已停止。残留相关进程：%s" % (left or "0"))


def do_build(args):
    """在沙盒里编译（改过 ws/src 之后用）。"""
    state = load_state()
    env = child_env(state.get("domain", 42), state.get("http_port", 8180))
    packages = args.packages or []
    command = ["bash", "-lc"]
    inner = ("source /opt/ros/humble/setup.bash && "
             "cd %s/ws && colcon build --symlink-install" % ROOT)
    if packages:
        inner += " --packages-select " + " ".join(packages)
    command.append(inner)
    print("编译中：%s" % ("全部包" if not packages else " ".join(packages)))
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    tail = (result.stdout or "").strip().splitlines()[-12:]
    for line in tail:
        print("  " + line)
    if result.returncode != 0:
        print("编译失败（退出码 %d）" % result.returncode)
        for line in (result.stderr or "").strip().splitlines()[-8:]:
            print("  " + line)
        raise SystemExit(result.returncode)
    print("编译完成")


def do_reset(args):
    do_stop(args)
    for name in ("build", "install", "log"):
        shutil.rmtree(os.path.join(WS, name), ignore_errors=True)
    shutil.rmtree(os.path.join(ROOT, "data"), ignore_errors=True)
    os.makedirs(os.path.join(ROOT, "data"), exist_ok=True)
    print("已清空编译产物与数据（源码与场景保留）")


def do_list(args):
    print("场景：")
    for item in list_scenarios():
        print("  - %s" % item["name"])
    print("故障：")
    for item in list_faults():
        suffix = ("（%s）" % item["error"]) if item["error"] else ""
        print("  - %s  %s  topic=%s  args=%s%s"
              % (item["name"], item["label"], item["topic"] or "-",
                 item["args"] or {}, suffix))
    print("参数文件：")
    seen = []
    for row in list_params():
        if row["file"] not in seen:
            seen.append(row["file"])
    for name in seen:
        print("  - %s" % name)


def do_status(args):
    state = load_state()
    pid = read_pid()
    print("根目录：%s" % ROOT)
    print("domain=%s 端口=%s 上次场景=%s"
          % (state.get("domain"), state.get("http_port"),
             state.get("last_scenario") or "-"))
    print("运行中：%s" % ("是（pid=%s）" % pid if alive(pid) else "否"))


def main(argv=None):
    parser = argparse.ArgumentParser(prog="simcore")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_init = sub.add_parser("init", help="初始化沙盒")
    p_init.add_argument("--src", default=None, help="从哪个 src 复制")
    p_init.add_argument("--fresh", action="store_true", help="重来一遍")
    p_init.set_defaults(func=do_init)

    p_run = sub.add_parser("run", help="跑一个场景")
    p_run.add_argument("scenario")
    p_run.set_defaults(func=do_run)

    p_stop = sub.add_parser("stop", help="停止")
    p_stop.set_defaults(func=do_stop)

    p_build = sub.add_parser("build", help="编译沙盒工作区")
    p_build.add_argument("packages", nargs="*", help="只编这些包")
    p_build.set_defaults(func=do_build)

    p_reset = sub.add_parser("reset", help="清编译产物")
    p_reset.set_defaults(func=do_reset)

    p_list = sub.add_parser("list", help="列出场景/故障/参数文件")
    p_list.set_defaults(func=do_list)

    p_status = sub.add_parser("status", help="状态")
    p_status.set_defaults(func=do_status)

    args = parser.parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
