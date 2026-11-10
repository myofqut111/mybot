#!/usr/bin/env python3
"""采集 YOLO 训练用图像：订阅摄像头话题，按节奏存成 PNG。

为什么要用它，而不是手动截图：
  1. 有"最小间隔"限速，不会一秒存 50 张几乎一样的图；
  2. 有"画面几乎没变就跳过"的去重（避免同一视角存几百份）；
  3. 文件名带帧号 + 时间戳，后面切分 train/val、回溯问题都方便。

用法（Jetson 上，摄像头节点已在跑）：
    # 默认订阅 /camera/camera/image_raw，每秒最多 2 张，存到 ~/datasets/raw
    ros2 run mybot_vision capture_images --out ~/datasets/raw --rate 2

    # 只存 200 张就退出（方便按批次采）
    ros2 run mybot_vision capture_images --out ~/datasets/raw --max 200

    # 关掉去重（每个视角都想留）
    ros2 run mybot_vision capture_images --diff-threshold 0

采集建议：每个类别换 5~10 个位置/角度/距离，每处停 2~3 秒让它存几张；
正样本之外，**一定也要拍"空场景"**（没有目标的画面），能显著降低误检。
"""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# 早于该时刻的墙钟一定没同步过：这块 Jetson 主板没有 RTC 电池，
# 断电重开后系统时间会退回 1970-01-01，此时按墙钟命名文件没有意义。
CLOCK_SANE_AFTER = 1_700_000_000


class ImageCapture(Node):
    def __init__(self, args) -> None:
        super().__init__('image_capture')
        self._out = os.path.expanduser(args.out)
        self._prefix = args.prefix
        self._min_dt = 1.0 / max(args.rate, 0.01)
        self._diff_threshold = float(args.diff_threshold)
        self._max = int(args.max)
        self._bridge = CvBridge()
        # 墙钟不可信时（停在 1970）改用开机计时命名，并在日志里大声提醒
        self._clock_ok = time.time() > CLOCK_SANE_AFTER
        if not self._clock_ok:
            self.get_logger().error(
                '系统时钟未同步（当前 %s，疑似 1970）：文件名改用开机计时；'
                '请执行 sudo /home/xmz/fix_clock.sh 校时'
                % time.strftime('%Y-%m-%d %H:%M:%S'))

        self._last_save = 0.0
        self._prev_gray = None
        self._saved = 0
        self._skipped_same = 0
        self._skipped_rate = 0
        self._reported_size = False
        self.done = False

        os.makedirs(self._out, exist_ok=True)
        self.create_subscription(
            Image, args.topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            'capture_images: topic=%s out=%s rate<=%.1f/s%s%s'
            % (args.topic, self._out, args.rate,
               '' if self._max <= 0 else ' max=%d' % self._max,
               '' if self._diff_threshold <= 0 else
               ' diff>%.1f' % self._diff_threshold))

    def _on_image(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_save < self._min_dt:
            self._skipped_rate += 1
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:      # 编码不匹配等，跳过这一帧
            self.get_logger().warning('imgmsg_to_cv2 failed: %s' % error)
            return

        if not self._reported_size:
            self._reported_size = True
            height, width = frame.shape[:2]
            self.get_logger().info('first frame: %dx%d' % (width, height))
            if width < 640:
                self.get_logger().warning(
                    '分辨率只有 %dx%d：画面里小于约 50px 的目标很难学出来，'
                    '采集/标注时请让目标足够大' % (width, height))

        # 缩略灰图做"画面有没有变"的比较，几乎不变就跳过（省大量重复样本）
        gray = cv2.cvtColor(cv2.resize(frame, (64, 48)), cv2.COLOR_BGR2GRAY)
        if (self._diff_threshold > 0.0 and self._prev_gray is not None and
                float(np.mean(cv2.absdiff(gray, self._prev_gray)))
                < self._diff_threshold):
            self._skipped_same += 1
            return
        self._prev_gray = gray
        self._last_save = now

        self._saved += 1
        stamp = msg.header.stamp
        if self._clock_ok and stamp.sec >= CLOCK_SANE_AFTER:
            stamp_text = '%d.%09d' % (stamp.sec, stamp.nanosec)
        else:
            # 前缀 b = 开机以来的秒数，重启后不会和旧文件混在一起
            stamp_text = 'b%09.3f' % time.monotonic()
        name = '%s_%05d_%s.png' % (self._prefix, self._saved, stamp_text)
        path = os.path.join(self._out, name)
        k = 1
        while os.path.exists(path):      # 绝不静默覆盖已有样本
            k += 1
            path = os.path.join(self._out, name[:-4] + '_%d.png' % k)
        cv2.imwrite(path, frame)

        if self._saved % 10 == 0:
            self.get_logger().info(
                'saved %d (skipped: same=%d rate=%d)'
                % (self._saved, self._skipped_same, self._skipped_rate))

        if self._max > 0 and self._saved >= self._max:
            self.get_logger().info('reached --max %d, stopping' % self._max)
            self.done = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Capture camera images for YOLO training.')
    parser.add_argument('--topic', default='/camera/camera/image_raw')
    parser.add_argument('--out', default='~/datasets/raw')
    parser.add_argument('--prefix', default='img')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='每秒最多存几张（限速）')
    parser.add_argument('--max', type=int, default=0,
                        help='存够多少张就退出，0 = 不限')
    parser.add_argument('--diff-threshold', type=float, default=2.0,
                        help='画面平均差异小于该值就跳过；0 = 不去重')
    return parser


def main(args=None) -> None:
    parser = build_parser()
    parsed = parser.parse_args(args)
    rclpy.init()
    node = ImageCapture(parsed)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            'done: saved=%d, skipped_same=%d, skipped_rate=%d, dir=%s'
            % (node._saved, node._skipped_same, node._skipped_rate, node._out))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
