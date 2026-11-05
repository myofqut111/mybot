# -*- coding: utf-8 -*-
"""业务栈的启停管理：一键起 nav2 / SLAM / 巡检 / 看门狗。

为什么需要这一层：这些栈本来是"另开几个终端敲 ros2 launch"。要交给不懂 ROS 的人
用，就得让它们能在一个网页上点一下起来——并且能点一下停干净。

三条设计决定：

1. **健康看关键节点，不看 launch 父进程。** launch 进程活着但 amcl 已经崩了，那不叫
   运行中，叫部分死亡。判据是 ROS 图里有哪些节点。
2. **外部已经起着的栈也算"运行中"。** 用户很可能先用终端起过一次；这时候面板应该
   显示"运行中（外部启动）"，而不是红着让人再点一次、结果起两份。
3. **必须能在开着的栈上停干净。** SIGINT → SIGTERM → SIGKILL，最后 `pkill -f` 兜底，
   每一步都确认；否则会出现"面板显示已停、进程还在跑"的情况——这种假状态比没有面板
   更危险。

本模块不 import rclpy：节点名的来源由调用方以集合传进来，所以可以在没有 ROS 的
机器上把这套状态机跑完整。
"""
import os
import signal
import subprocess
import threading
import time
from collections import deque

STOPPED = 'stopped'      # 没起
STARTING = 'starting'    # 子进程在，关键节点还没齐
RUNNING = 'running'      # 关键节点齐了
DEGRADED = 'degraded'    # 子进程在、关键节点缺（部分死亡）
EXITED = 'exited'        # 子进程退出过（失败或自己结束）

STARTING_GRACE_SEC = 25.0     # 起来后多久之内允许"节点还没齐"
STOP_INT_SEC = 6.0            # SIGINT 后等多久
STOP_TERM_SEC = 4.0           # SIGTERM 后等多久
LOG_LINES = 400               # 每个栈保留的日志行数


class StackSpec(object):
    """一个业务栈：怎么起、起来之后该看到哪些节点。"""

    def __init__(self, name, label, package, launch_file, probes,
                 args=None, description='', starts_rviz=False):
        self.name = name
        self.label = label
        self.package = package
        self.launch_file = launch_file
        self.probes = tuple(probes)
        self.args = list(args or [])
        self.description = description
        self.starts_rviz = starts_rviz


def default_stacks():
    """四个栈。节点名与各自的 launch 文件对应，改了 launch 就要同步改这里。"""
    return [
        StackSpec(
            'nav', '导航', 'fishbot_navigation', 'navigation.launch.py',
            probes=('amcl', 'controller_server', 'planner_server'),
            args=['rviz:=true'],
            description='AMCL 定位 + Nav2。会一并拉起 RViz。',
            starts_rviz=True),
        StackSpec(
            'slam', '建图', 'fishbot_navigation', 'mapping.launch.py',
            probes=('slam_toolbox',),
            args=['rviz:=true'],
            description='slam_toolbox 建图。会一并拉起 RViz。与导航互斥。',
            starts_rviz=True),
        StackSpec(
            'patrol', '巡检', 'fishbot_patrol_v2', 'patrol_v2.launch.py',
            probes=('route_manager', 'patrol_task_manager',
                    'patrol_executor_v2'),
            args=['nav:=false', 'console:=false'],
            description='路线管理器 + 任务管理器 + 执行器。导航没起时只能排队。'),
        StackSpec(
            'safety', '看门狗', 'fishbot_safety', 'watchdog.launch.py',
            probes=('safety_watchdog',),
            args=[],
            description='车在动但没有速度指令时自己刹停。建议一直开着。'),
    ]


class Stack(object):
    """一个栈的运行时状态：子进程、日志、时间戳。"""

    def __init__(self, spec):
        self.spec = spec
        self.proc = None
        self.started_at = 0.0
        self.exit_code = None
        self.exit_at = 0.0
        self.last_error = ''
        self.lines = deque(maxlen=LOG_LINES)
        self.thread = None

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def tail(self, count=20):
        return list(self.lines)[-count:]


class StackManager(object):
    """起停 + 状态判定。线程安全（一个锁护全部状态）。"""

    def __init__(self, log_sink=None, stacks=None, env=None,
                 popen=None, clock=None, with_rviz=True):
        self._lock = threading.RLock()
        self._stacks = {spec.name: Stack(spec) for spec in (stacks
                                                            or default_stacks())}
        self._log_sink = log_sink or (lambda source, text: None)
        self._env = dict(env if env is not None else os.environ)
        # 没有图形会话时 RViz 起不来；控制台多半是 ssh 起的，DISPLAY 是空的。
        self._env.setdefault('DISPLAY', ':0')
        self._env['PYTHONUNBUFFERED'] = '1'
        self._popen = popen or subprocess.Popen
        self._clock = clock or time.monotonic
        # 是否让导航/建图顺带拉起 RViz。关掉的理由不是"省事"，是**省 CPU**：
        # RViz 在弱虚拟机上会和 Nav2 的 20 Hz 控制循环抢核，表现出来就是
        # "/cmd_vel 内容对，但车一顿一顿"。诊断这个现象时先关掉它排除。
        self._with_rviz = bool(with_rviz)

    def _args_for(self, spec):
        """把 spec 里的 rviz:=true 换成用户真正想要的值。"""
        out = []
        for arg in spec.args:
            if arg.startswith('rviz:='):
                out.append('rviz:=%s' % ('true' if self._with_rviz else 'false'))
            else:
                out.append(arg)
        return out


    # ------------------------------------------------------------ 查询
    def names(self):
        return list(self._stacks)

    def spec(self, name):
        stack = self._stacks.get(name)
        return stack.spec if stack else None

    def _conflict(self, name):
        """导航与建图抢同一个 TF 树和地图话题，同时起必然互相踩。"""
        pairs = {'nav': 'slam', 'slam': 'nav'}
        other = pairs.get(name)
        if not other:
            return None
        stack = self._stacks.get(other)
        if stack and stack.alive():
            return other
        return None

    def status(self, running_nodes=None, now=None):
        """返回每个栈的状态。running_nodes 是 ROS 图里的节点名集合。"""
        now = self._clock() if now is None else now
        nodes = set(running_nodes or ())
        out = {}
        with self._lock:
            for name, stack in self._stacks.items():
                spec = stack.spec
                present = [p for p in spec.probes if p in nodes]
                all_present = len(present) == len(spec.probes)
                if stack.alive():
                    if all_present:
                        state = RUNNING
                    elif now - stack.started_at < STARTING_GRACE_SEC:
                        state = STARTING
                    else:
                        state = DEGRADED
                elif all_present:
                    # 别人起的（终端 / 另一个 launch），照样算运行中
                    state = RUNNING
                elif present:
                    # 只起了一半：报"降级"而不是"未启动"，否则用户看着三个节点
                    # 明明有一个在跑、面板却说没启动，只会更困惑。
                    state = DEGRADED
                elif stack.exit_code is not None:
                    state = EXITED
                else:
                    state = STOPPED
                out[name] = {
                    'label': spec.label,
                    'state': state,
                    'managed': stack.alive(),
                    'probes': {'expect': list(spec.probes),
                               'found': present},
                    'since_s': (round(now - stack.started_at, 1)
                                if stack.alive() else None),
                    'exit_code': stack.exit_code,
                    'last_error': stack.last_error,
                    'description': spec.description,
                    'starts_rviz': spec.starts_rviz,
                    'logs': stack.tail(20) if state in (EXITED, DEGRADED)
                            else [],
                }
        return out

    def logs(self, name, count=200):
        with self._lock:
            stack = self._stacks.get(name)
            return stack.tail(count) if stack else []

    def busy(self, name):
        with self._lock:
            stack = self._stacks.get(name)
            return bool(stack and stack.alive())

    # ------------------------------------------------------------ 起
    def start(self, name, running_nodes=None):
        """起一个栈。返回 (ok, message)。

        running_nodes 传的是 ROS 图里的节点名：如果这套栈的探针已经全在图里，
        说明别人（终端 / 另一个 launch）已经把它起来了。这时候再起一套不会
        "覆盖"，只会**并存**——两套 amcl 抢同一个 TF、两套 controller 抢同一个
        cmd_vel，表现就是车一顿一顿、定位来回跳。所以这里要拦住。
        """
        with self._lock:
            stack = self._stacks.get(name)
            if stack is None:
                return False, '未知的栈：%s' % name
            if stack.alive():
                return False, '%s 已经在运行' % stack.spec.label
            if running_nodes:
                nodes = set(running_nodes)
                present = [p for p in stack.spec.probes if p in nodes]
                if len(present) == len(stack.spec.probes):
                    return False, ('%s 已经在图里跑着了（不是这个面板起的），'
                                   '再起一套会和它抢 TF 与 cmd_vel；'
                                   '先到那个终端把它停掉' % stack.spec.label)
            other = self._conflict(name)
            if other:
                return False, ('%s 正在运行，与 %s 互斥；先停掉它'
                               % (self._stacks[other].spec.label,
                                  stack.spec.label))

            spec = stack.spec
            cmd = ['ros2', 'launch', spec.package, spec.launch_file] + self._args_for(spec)
            try:
                proc = self._popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    env=self._env, start_new_session=True, bufsize=1,
                    universal_newlines=True)
            except Exception as error:
                stack.last_error = '启动失败：%s' % error
                return False, stack.last_error

            stack.proc = proc
            stack.started_at = self._clock()
            stack.exit_code = None
            stack.exit_at = 0.0
            stack.last_error = ''
            stack.lines.clear()
            stack.thread = threading.Thread(
                target=self._pump, args=(name, proc), daemon=True)
            stack.thread.start()

        self._log_sink('launch:%s' % name,
                       '启动 %s：%s' % (spec.label, ' '.join(cmd)))
        if spec.starts_rviz:
            return True, '正在启动 %s…（会一并拉起 RViz）' % spec.label
        return True, '正在启动 %s…' % spec.label

    def _pump(self, name, proc):
        """把子进程输出搬进环形缓冲，同时转给控制台日志面板。"""
        prefix = 'launch:%s' % name
        try:
            for line in proc.stdout:
                line = line.rstrip('\n')
                if not line:
                    continue
                with self._lock:
                    stack = self._stacks.get(name)
                    if stack is not None:
                        stack.lines.append(line)
                self._log_sink(prefix, line)
        except Exception:
            pass
        code = proc.poll()
        with self._lock:
            stack = self._stacks.get(name)
            if stack is not None:
                stack.exit_code = code
                stack.exit_at = self._clock()
                if code:
                    stack.last_error = '退出码 %s' % code
        self._log_sink(prefix, '进程结束（退出码 %s）' % code)

    # ------------------------------------------------------------ 停
    def stop(self, name, kill_timeout=None):
        """停一个栈：SIGINT → SIGTERM → SIGKILL → pkill 兜底。返回 (ok, message)。"""
        with self._lock:
            stack = self._stacks.get(name)
            if stack is None:
                return False, '未知的栈：%s' % name
            proc = stack.proc
            spec = stack.spec
            if proc is None or proc.poll() is not None:
                # 没托管却在跑（外部启动）：只能按 launch 文件名兜底
                if self._pkill(spec.launch_file):
                    stack.exit_code = None
                    return True, '已停止（外部启动的 %s）' % spec.label
                return False, '%s 没有在运行' % spec.label

        ok = self._terminate(proc, kill_timeout)
        with self._lock:
            if stack.alive():
                stack.proc = stack.proc
            else:
                stack.proc = None
                stack.exit_code = None        # 主动停的，不算"异常退出"
        self._pkill(spec.launch_file)     # 兜底：清掉 launch 的孤儿子进程
        return ok, ('已停止 %s' % spec.label) if ok else \
            ('%s 停止超时，已强制结束' % spec.label)

    def _terminate(self, proc, kill_timeout=None):
        """按 SIGINT → SIGTERM → SIGKILL 逐步升级，每步都确认。"""
        for sig, wait in ((signal.SIGINT, STOP_INT_SEC),
                          (signal.SIGTERM, STOP_TERM_SEC)):
            if proc.poll() is not None:
                return True
            try:
                os.killpg(os.getpgid(proc.pid), sig)
            except ProcessLookupError:
                return True
            except Exception:
                try:
                    proc.send_signal(sig)
                except Exception:
                    pass
            end = time.time() + (kill_timeout if kill_timeout else wait)
            while time.time() < end:
                if proc.poll() is not None:
                    return True
                time.sleep(0.1)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                return False
        end = time.time() + 3.0
        while time.time() < end:
            if proc.poll() is not None:
                return True
            time.sleep(0.05)
        return False

    def _pkill(self, pattern):
        """按 launch 文件名清理残留进程。返回是否真的杀到了东西。"""
        try:
            result = subprocess.run(['pkill', '-f', pattern],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
            return result.returncode == 0
        except Exception:
            return False

    # ------------------------------------------------------------ 退出
    def shutdown(self):
        """控制台退出时把托管的栈都停掉，免得留下一堆孤儿。"""
        for name in list(self._stacks):
            with self._lock:
                stack = self._stacks[name]
                alive = stack.alive()
            if alive:
                try:
                    self.stop(name, kill_timeout=2.0)
                except Exception:
                    pass
