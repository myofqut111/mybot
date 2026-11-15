# -*- coding: utf-8 -*-
"""node.py 的静态契约：这些坑真机上踩过，不能再回去。"""
import io
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(os.path.dirname(HERE), "mybot_explore", "node.py")


def _read():
    with io.open(NODE, encoding="utf-8") as fh:
        return fh.read()


class ShadowingTest(unittest.TestCase):
    def test_spin_client_is_not_named_spin(self):
        """self._spin = ActionClient(...) 会把 _spin() 方法遮住，真机炸过。"""
        src = _read()
        self.assertNotIn("self._spin = ", src)
        self.assertIn("self._spin_client", src)

    def test_tick_is_wrapped(self):
        """定时器回调必须包一层异常兜底，否则 traceback 只打到终端。"""
        src = _read()
        self.assertIn("def _tick_once", src)
        self.assertIn("def _tick(self):", src)
        tick = src.split("def _tick(self):", 1)[1].split("def _tick_once", 1)[0]
        self.assertIn("try:", tick)
        self.assertIn("except Exception", tick)
        self.assertIn("traceback.format_exc()", tick)


if __name__ == "__main__":
    unittest.main()
