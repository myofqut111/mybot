# -*- coding: utf-8 -*-
"""node.py 的静态契约：这些坑真机上踩过，不能再回去。"""
import io
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(os.path.dirname(HERE), "mybot_explore", "node.py")
YAML = os.path.join(os.path.dirname(HERE), "config", "explore.yaml")


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


class SlipWiringTest(unittest.TestCase):
    """打滑检测必须真的接在开车循环里，且短墙只在两处出现。"""

    def test_check_slip_called_from_tick(self):
        src = _read()
        self.assertIn("self._check_slip(now, pose)", src)
        self.assertIn("slip_mod.SlipDetector(", src)

    def test_slip_params_declared(self):
        src = _read()
        self.assertIn("for key, value in slip_mod.DEFAULTS.items():", src)
        self.assertIn("'keepout_radius_m'", src)

    def test_wall_stamped_in_grid_and_clean_map(self):
        src = _read()
        self.assertEqual(src.count("blocked_mod.stamp_wall("), 2)

    def test_keepout_only_affects_selection(self):
        """禁试区只塞进 blacklist，不许写进地图。"""
        src = _read()
        self.assertIn("self._explorer.blacklist.append(", src)

    def test_yaml_has_slip_block(self):
        path = os.path.join(os.path.dirname(HERE), "config", "explore.yaml")
        with io.open(path, encoding="utf-8") as fh:
            text = fh.read()
        for key in ("slip_enabled", "slip_scan_near_m", "blocked_wall_len_m",
                    "keepout_radius_m", "blocked_file"):
            self.assertIn(key, text)


if __name__ == "__main__":
    unittest.main()

class GateContractTest(unittest.TestCase):
    """门控接线：不许再出现"整段暂停建图"的写法。"""

    @classmethod
    def setUpClass(cls):
        cls.node = io.open(NODE, encoding='utf-8').read()
        cls.yaml = io.open(YAML, encoding='utf-8').read()

    def test_no_permanent_pause(self):
        self.assertNotIn('保持暂停', self.node,
                         '不允许把"保持暂停"当成终态：那样地图会永久冻结')
        self.assertNotIn('PauseSrv', self.node, '暂停服务已被扫描门控取代')

    def test_gate_wired(self):
        self.assertIn('_close_gate', self.node)
        self.assertIn('_open_gate', self.node)
        self.assertIn("declare_parameter('scan_gate_enabled'", self.node)
        self.assertIn('scan_gate_enabled', self.yaml)

    def test_home_settle_params_shipped(self):
        self.assertIn('home_settle_s', self.yaml)
        self.assertIn('home_timeout_s', self.yaml)
