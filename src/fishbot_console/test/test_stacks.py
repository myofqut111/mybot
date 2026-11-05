# -*- coding: utf-8 -*-
"""stacks.py：一键启停的状态机与冲突规则。

这一层不碰 ROS，所以能在这里把"点两次开始会怎样""节点该起没起算不算成功"
这些最容易在真机上踩坑的分支全部枚举出来。

用假的 popen 代替真进程：真起 `ros2 launch` 会拉起 Nav2，测试跑不动也不该跑。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import stacks  # noqa: E402


class FakeProc(object):
    """够 Stack 用的最小进程：有 stdout（可迭代）、poll()、wait()。"""

    def __init__(self, lines=None, code=None):
        self.stdout = iter(lines or [])
        self._code = code
        self.killed = False

    def poll(self):
        return self._code

    def wait(self, timeout=None):
        return self._code or 0

    def terminate(self):
        self.killed = True

    def kill(self):
        self.killed = True


def _spec(name='nav', label='导航', probes=None, args=None, rviz=False):
    return stacks.StackSpec(
        name=name, label=label, package='fishbot_navigation',
        launch_file='navigation.launch.py',
        probes=probes or ['amcl'],
        args=list(args or ['rviz:=false']),
        description='test',
        starts_rviz=rviz)


class StatusCase(unittest.TestCase):
    def setUp(self):
        self.now = [1000.0]
        self.manager = stacks.StackManager(
            stacks=[_spec()], env={}, clock=lambda: self.now[0],
            popen=lambda *a, **k: FakeProc())

    def test_initial_is_stopped(self):
        out = self.manager.status([])['nav']
        self.assertEqual(out['state'], stacks.STOPPED)
        self.assertFalse(out['managed'])

    def test_probes_present_without_our_process_is_marked(self):
        """节点在跑但不是我们起的（用户自己在别处 launch 过）→ 要能看出来。"""
        out = self.manager.status(['amcl'])['nav']
        self.assertEqual(out['state'], stacks.RUNNING)
        self.assertFalse(out['managed'])
        self.assertEqual(out['probes']['found'], ['amcl'])

    def test_partial_probes_is_degraded(self):
        spec = _spec(probes=['amcl', 'controller_server'])
        manager = stacks.StackManager(stacks=[spec], env={},
                                      clock=lambda: self.now[0],
                                      popen=lambda *a, **k: FakeProc())
        self.assertEqual(manager.status(['amcl'])['nav']['state'],
                         stacks.DEGRADED)

    def test_unknown_stack_has_no_entry(self):
        self.assertNotIn('slam', self.manager.status([]))

    def test_status_includes_probe_detail(self):
        out = self.manager.status(['amcl'])['nav']
        self.assertEqual(out['probes']['expect'], ['amcl'])
        self.assertEqual(out['probes']['found'], ['amcl'])


class StartCase(unittest.TestCase):
    def setUp(self):
        self.now = [1000.0]
        self.calls = []

        def popen(cmd, **kwargs):
            self.calls.append((cmd, kwargs))
            return FakeProc()

        self.manager = stacks.StackManager(
            stacks=[_spec(name='nav'), _spec(name='slam', label='建图',
                                             probes=['slam_toolbox'])],
            env={}, clock=lambda: self.now[0], popen=popen)

    def test_start_invokes_ros2_launch(self):
        ok, message = self.manager.start('nav')
        self.assertTrue(ok, message)
        self.assertEqual(len(self.calls), 1)
        cmd = self.calls[0][0]
        self.assertEqual(cmd[:3], ['ros2', 'launch', 'fishbot_navigation'])
        self.assertIn('navigation.launch.py', cmd)

    def test_start_uses_session_per_process(self):
        """必须 start_new_session：控制台重启时不能把 business 栈一起带走。"""
        self.manager.start('nav')
        self.assertTrue(self.calls[0][1]['start_new_session'])

    def test_second_start_is_rejected(self):
        self.manager.start('nav')
        ok, message = self.manager.start('nav')
        self.assertFalse(ok)
        self.assertIn('已经在运行', message)
        self.assertEqual(len(self.calls), 1)

    def test_conflict_nav_vs_slam_is_rejected(self):
        self.manager.start('slam')
        ok, message = self.manager.start('nav')
        self.assertFalse(ok)
        self.assertIn('互斥', message)
        self.assertEqual(len(self.calls), 1)

    def test_unknown_name_is_rejected(self):
        ok, message = self.manager.start('nope')
        self.assertFalse(ok)
        self.assertIn('未知', message)
        self.assertEqual(self.calls, [])

    def test_stop_on_stopped_stack_is_harmless(self):
        ok, message = self.manager.stop('nav')
        self.assertFalse(ok)
        self.assertIn('没有在运行', message)

    def test_stop_goes_through_terminate_and_pkill(self):
        seen = []
        self.manager._terminate = lambda proc, timeout=None: seen.append('terminate') or True
        self.manager._pkill = lambda pattern: seen.append('pkill') or True
        self.manager.start('nav')
        ok, message = self.manager.stop('nav')
        self.assertTrue(ok, message)
        self.assertIn('terminate', seen)
        self.assertIn('pkill', seen)


class FailureCase(unittest.TestCase):
    def test_popen_exception_is_reported_not_raised(self):
        def popen(*a, **k):
            raise OSError('boom')
        manager = stacks.StackManager(stacks=[_spec()], env={},
                                      clock=lambda: 1000.0, popen=popen)
        ok, message = manager.start('nav')
        self.assertFalse(ok)
        self.assertIn('boom', message)

    def test_busy_reports_live_stack_only(self):
        manager = stacks.StackManager(stacks=[_spec()], env={},
                                      clock=lambda: 1000.0,
                                      popen=lambda *a, **k: FakeProc())
        self.assertFalse(manager.busy('nav'))
        manager.start('nav')
        self.assertTrue(manager.busy('nav'))        # 起过就算忙，防连点
        self.assertFalse(manager.busy('nope'))


class EnvCase(unittest.TestCase):
    def test_display_defaults_to_zero_when_missing(self):
        captured = {}

        def popen(cmd, **kwargs):
            captured.update(kwargs['env'])
            return FakeProc()

        manager = stacks.StackManager(stacks=[_spec()], env={},
                                      clock=lambda: 1000.0, popen=popen)
        manager.start('nav')
        self.assertEqual(captured.get('DISPLAY'), ':0')
        self.assertEqual(captured.get('PYTHONUNBUFFERED'), '1')

    def test_existing_display_is_kept(self):
        captured = {}

        def popen(cmd, **kwargs):
            captured.update(kwargs['env'])
            return FakeProc()

        manager = stacks.StackManager(stacks=[_spec()], env={'DISPLAY': ':1'},
                                      clock=lambda: 1000.0, popen=popen)
        manager.start('nav')
        self.assertEqual(captured.get('DISPLAY'), ':1')


class LogCase(unittest.TestCase):
    def test_log_sink_gets_start_message(self):
        seen = []
        manager = stacks.StackManager(
            stacks=[_spec()],
            log_sink=lambda source, text: seen.append((source, text)),
            env={}, clock=lambda: 1000.0, popen=lambda *a, **k: FakeProc())
        manager.start('nav')
        self.assertTrue(seen)
        self.assertTrue(seen[0][0].startswith('launch:'))

    def test_child_output_lands_in_ring(self):
        proc = FakeProc(lines=['INFO: 起来了'])
        manager = stacks.StackManager(stacks=[_spec()], env={},
                                      clock=lambda: 1000.0,
                                      popen=lambda *a, **k: proc)
        manager.start('nav')
        manager._stacks['nav'].thread.join(timeout=2.0)
        self.assertIn('INFO: 起来了', manager.logs('nav', 10))

    def test_patrol_probes_match_real_node_names(self):
        """巡检栈的探测名必须和真实节点名一致。

        patrol_route_manager 是想当然写出来的，真实节点叫 
oute_manager。
        后果不是崩溃，而是面板永远显示半启动，看的人以为巡检没起来。
        """
        spec = {item.name: item for item in stacks.default_stacks()}['patrol']
        self.assertEqual(
            spec.probes,
            ('route_manager', 'patrol_task_manager', 'patrol_executor_v2'))

class RvizArgCase(unittest.TestCase):
    """rviz:= 必须跟着参数走：关掉 RViz 是排查“车一顿一顿”的第一步。"""

    def _cmd(self, with_rviz):
        calls = []

        def popen(cmd, **kwargs):
            calls.append(cmd)
            return FakeProc()

        manager = stacks.StackManager(stacks=[_spec()], env={},
                                      clock=lambda: 1000.0, popen=popen,
                                      with_rviz=with_rviz)
        manager.start('nav')
        return calls[0]

    def test_rviz_arg_can_be_disabled(self):
        cmd = self._cmd(False)
        self.assertIn('rviz:=false', cmd)
        self.assertNotIn('rviz:=true', cmd)

    def test_rviz_arg_kept_when_enabled(self):
        cmd = self._cmd(True)
        self.assertIn('rviz:=true', cmd)
        self.assertNotIn('rviz:=false', cmd)

    def test_other_args_untouched(self):
        cmd = self._cmd(False)
        self.assertEqual(cmd[:3], ['ros2', 'launch', 'fishbot_navigation'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
