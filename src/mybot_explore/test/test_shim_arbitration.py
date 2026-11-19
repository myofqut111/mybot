# -*- coding: utf-8 -*-
"""odom_shim 的 TF 仲裁：静态契约 + 纯逻辑 (2026-11-19)

背景：两个节点同时发 odom→base_link 会让 slam 拿到忽 A 忽 B 的位姿，
大量丢帧、建图糊成扇形。所以 shim 先静默 3 秒偷听 /tf：这期间自己一个字
不发，只要看见这对 TF 就让位（退化成"没开 shim"的行为）。
"""

import os
import re
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(os.path.dirname(HERE), "mybot_explore", "odom_shim_node.py")


def _source():
    with open(NODE, encoding="utf-8") as handle:
        return handle.read()


class ArbitrationContractTest(unittest.TestCase):

    def test_declares_arbitration_params(self):
        src = _source()
        for key in ("'arbitration'", "'arbitration_s'",
                    "'foreign_epsilon_m'", "'foreign_epsilon_rad'"):
            self.assertIn(key, src, "缺少参数 %s" % key)

    def test_listens_to_tf(self):
        src = _source()
        self.assertIn("create_subscription(TFMessage, '/tf'", src)
        self.assertIn("def _on_tf", src)

    def test_yield_paths_exist(self):
        src = _source()
        self.assertIn("def _yield", src)
        # 静默窗口内看见别人的 TF → 让位
        self.assertIn("我们还没发过", src)
        # 接管之后发现数值对不上 → 让位
        self.assertIn("_eps_m", src)
        self.assertIn("_eps_rad", src)

    def test_does_not_publish_while_listening_or_yielded(self):
        src = _source()
        # _tick 里必须先处理 listening / yielded 再 sendTransform
        tick = src.split("def _tick", 1)[1]
        send_at = tick.index("self._broadcaster.sendTransform")
        self.assertLess(tick.index("if self._listening"), send_at)
        self.assertLess(tick.index("if self._yielded"), send_at)

    def test_state_reports_arbitration(self):
        src = _source()
        for key in ("state['listening']", "state['yielded']", "state['arbitration']"):
            self.assertIn(key, src)

    def test_yield_message_tells_user_what_to_do(self):
        src = _source()
        self.assertIn("ekf_publish_tf:=false", src)
        self.assertIn("退让", src)


class WrapAngleTest(unittest.TestCase):

    def test_wrap(self):
        import sys
        sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
        from mybot_explore import odom_anchor
        self.assertAlmostEqual(odom_anchor._wrap(0.0), 0.0, places=6)
        self.assertAlmostEqual(odom_anchor._wrap(3.3), 3.3 - 6.283185307179586, places=6)
        self.assertAlmostEqual(odom_anchor._wrap(-3.3), -3.3 + 6.283185307179586, places=6)


if __name__ == "__main__":
    unittest.main()
