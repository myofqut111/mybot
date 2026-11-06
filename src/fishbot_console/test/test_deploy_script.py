"""钉住部署脚本的三条铁律。

这三条都是真机上踩出来的事故，改脚本时很容易不小心退回去：

1. 不能用 ``--symlink-install``。develop 模式下 ``fishbot_console.egg-info``
   只落在 ``build/`` 里，而入口脚本 ``robot_console`` 是靠
   ``importlib.metadata`` 查 ``fishbot-console`` 的——元数据不在 ``sys.path``
   上就 ``PackageNotFoundError``，launch 还配了 respawn，会一直重试刷屏。

2. 必须先 TERM 再 KILL，并且要单独收 ``rosbridge_websocket`` / ``rosapi_node``。
   launch 被 ``-9`` 打死时来不及回收子进程，它们变成 PPID=1 的孤儿，
   一直占着 9090，下一次启动就是 ``Address already in use``。
   真机上曾经攒下 21 个孤儿。

3. 元数据要在中性 cwd 下验证。站在 ``src/`` 里查会自欺欺人，因为那里本来
   就有 egg-info，查得到并不代表别的地方查得到。
"""

import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(ROOT))      # src/fishbot_console -> 仓库根
SCRIPT = os.path.join(REPO, 'tools', 'deploy_console.sh')


class DeployScriptCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not os.path.exists(SCRIPT):
            raise unittest.SkipTest('部署脚本只在仓库里，安装后的包里没有')
        with open(SCRIPT, encoding='utf-8') as handle:
            cls.src = handle.read()

    def test_build_is_not_symlink_install(self):
        build_lines = [
            line for line in self.src.splitlines()
            if 'colcon build' in line and not line.lstrip().startswith('#')
        ]
        self.assertTrue(build_lines, '脚本里应该有 colcon build')
        for line in build_lines:
            self.assertNotIn('--symlink-install', line)

    def test_kills_orphaned_bridge_nodes(self):
        for pattern in ('rosbridge_server/rosbridge_websocket',
                        'rosapi/rosapi_node'):
            self.assertIn(pattern, self.src, '没收 %s 会留下占着 9090 的孤儿' % pattern)

    def test_terminates_before_kill(self):
        term = self.src.index('pkill -f "console.launch.py"')
        kill = self.src.index('pkill -9 -f "console.launch.py"')
        self.assertLess(term, kill, '先 TERM 再 KILL，别一上来就 -9')

    def test_verifies_metadata_from_neutral_cwd(self):
        check = self.src.index("m.version('fishbot-console')")
        self.assertLess(
            self.src.rindex('cd /tmp', 0, check), check,
            "验证元数据前要 cd /tmp，站在 src 里查会自欺欺人")

    def test_fails_loudly_when_metadata_missing(self):
        check = self.src.index("m.version('fishbot-console')")
        tail = self.src[check:check + 200]
        self.assertIn('PackageNotFoundError', tail)
        self.assertIn('exit 1', tail)


if __name__ == '__main__':
    unittest.main()
