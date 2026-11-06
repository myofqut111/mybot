# -*- coding: utf-8 -*-
"""日志里给人看的地址必须是能点开的地址。

真实事故：
    控制台启动日志里写的是 `http://0.0.0.0:8080`，用户照着敲进浏览器，
    打不开。0.0.0.0 是**绑定**语义（监听所有网卡），不是能访问的地址。
    主路径改成 public_host() 之后，端口顺延那条分支还留着旧写法，
    端口一冲突就又打印 0.0.0.0 —— 所以这里把两条路径都钉住。
"""

import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = os.path.join(ROOT, 'fishbot_console', 'node.py')


def _read(path):
    with io.open(path, encoding='utf-8') as handle:
        return handle.read()


class ListenUrlTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.src = _read(NODE)

    def test_port_fallback_prints_public_host(self):
        """端口顺延的提示里，URL 必须用 shown（真实 IP），不能用 host。"""
        start = self.src.index('已自动顺延到')
        end = self.src.index('if actual == ROSBRIDGE_PORT')
        block = self.src[start:end]
        self.assertIn('shown, actual', block)
        self.assertNotIn('host, actual', block)

    def test_public_host_computed_before_any_url_log(self):
        """shown 必须在第一次打印 URL 之前就算好，否则会 NameError。"""
        first_url = self.src.index('浏览器请打开')
        declare = self.src.index('shown = bridge.public_host()')
        self.assertLess(declare, first_url)

    def test_listening_line_uses_public_host(self):
        start = self.src.index('console listening on')
        block = self.src[start:start + 200]
        self.assertIn('shown', block)
        self.assertNotIn('http://%s:%d\' % (host', block)

    def test_shipped_config_autodetects(self):
        """出厂的 public_host 必须是空（自动探测）。

        同一台虚拟机在两个网段之间来回切（外网 192.168.202.x / demo
        192.168.1.x），钉死任何一个 IP，另一半时间打印出来的都是错的，
        用户照着敲必然打不开——这个坑已经踩过两次。
        """
        yaml = _read(os.path.join(ROOT, 'config', 'console.yaml'))
        self.assertIn("public_host: ''", yaml)


if __name__ == '__main__':
    unittest.main()
