# -*- coding: utf-8 -*-
"""压缩图订阅的契约测试。

为什么要有这个测试：
    原始图一帧 320x240x3 = 230KB，5fps 就是 9Mbps。控制台和视觉节点分别在
    两台机器上，这 9Mbps 全部要过 WiFi。实测把 FAST 那台入门路由器打到所有
    设备掉线（连路由器后台都进不去），所以控制台改成默认只订阅压缩图。

    本机没有 rclpy，node 建不起来，所以这里检查代码里的契约：
    默认值、订阅分支、回调是否直接透传字节。
"""

import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIDGE = os.path.join(ROOT, 'fishbot_console', 'ros_bridge.py')


def _read(path):
    with io.open(path, encoding='utf-8') as handle:
        return handle.read()


class CompressedStreamTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.src = _read(BRIDGE)

    def test_imports_compressed_message(self):
        self.assertIn(
            'from sensor_msgs.msg import CompressedImage, Image, LaserScan',
            self.src)

    def test_prefers_compressed_by_default(self):
        self.assertIn("self.declare_parameter('prefer_compressed', True)",
                      self.src)
        self.assertIn('compressed_image_topic', self.src)
        self.assertIn('compressed_annotated_topic', self.src)

    def test_compressed_topics_follow_vision_node_names(self):
        # 视觉节点用 ~/image_raw/compressed、~/image_annotated/compressed
        # 发布（节点名 camera_publisher / yolo_detector），这里必须对上。
        self.assertIn("'/camera/camera/image_raw/compressed'", self.src)
        self.assertIn("'/yolo_detector/image_annotated/compressed'", self.src)

    def test_raw_image_only_subscribed_as_fallback(self):
        """prefer_compressed=True 时不能再订阅原始图，否则 9Mbps 照旧。"""
        branch = self.src.index("self.get_parameter('prefer_compressed')")
        raw = self.src.index(
            "Image, self.get_parameter('image_topic').value")
        self.assertLess(branch, raw)
        between = self.src[branch:raw]
        self.assertIn('else:', between)
        self.assertIn('CompressedImage', between)

    def test_compressed_callbacks_pass_bytes_through(self):
        for name, attr in (('_on_image_compressed', '_frame_jpeg'),
                           ('_on_annotated_compressed', '_anno_jpeg')):
            start = self.src.index('def %s' % name)
            end = self.src.index('def ', start + 5)
            body = self.src[start:end]
            self.assertIn('bytes(msg.data)', body)
            self.assertIn(attr, body)
            # 不能再去调用解码：那是 VM 上的额外 CPU
            self.assertNotIn('self._encode(', body)


if __name__ == '__main__':
    unittest.main()
