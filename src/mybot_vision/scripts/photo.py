#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""现场实拍采集脚本（不依赖 ROS，直接找 ESP32 摄像头抓图）。

为什么不用 capture_images.py（那个是 ROS 节点）：
  - 这个脚本不需要先起 vision.launch.py，跑一条命令就能拍；
  - 优先走摄像头自带的 /capture 单张抓拍接口，拿到的永远是一张完整 JPEG，
    不会像读 MJPEG 流那样偶尔攒到半帧；
  - 拍完立刻判糊/判亮度，糊的当场就提示你重拍，不用等标注时才发现。

用法（Jetson 上）：
    # 每 1.5 秒自动拍一张，拍 20 张
    python3 photo.py --count 20 --interval 1.5

    # 手动节奏：摆好一个角度按一次回车
    python3 photo.py --manual

    # 改保存目录 / 摄像头 IP
    python3 photo.py --out ~/datasets/shots --host 192.168.1.100

拍完脚本会打印：存了几张、几张偏糊、平均亮度，以及保存目录。
"""

import argparse
import os
import shutil
import sys
import time
import urllib.request

import cv2
import numpy as np

# 早于该时刻的墙钟一定没同步过（这块板子没有 RTC 电池，断电会退回 1970）
CLOCK_SANE_AFTER = 1_700_000_000
# Laplacian 方差低于这个值视为“糊”（实测数据集里 <60 的占极少数）
SHARP_MIN = 60.0


def clock_is_sane():
    return time.time() > CLOCK_SANE_AFTER


def http_get(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.read()


class Grabber(object):
    """抓一帧 BGR 图像：优先 /capture 单张接口，失败再退到 MJPEG 流。"""

    def __init__(self, host, capture_port=80, stream_port=81, flip=True,
                 timeout=5.0):
        self._host = host
        self._capture_port = capture_port
        self._stream_port = stream_port
        self._flip = flip
        self._timeout = timeout
        self._cap = None
        self._mode = None

    def _try_capture(self):
        url = 'http://%s:%d/capture' % (self._host, self._capture_port)
        try:
            data = http_get(url, self._timeout)
        except Exception:
            return None
        if len(data) < 100 or data[:2] != b'\xff\xd8':
            return None
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def _open_stream(self):
        url = 'http://%s:%d/stream' % (self._host, self._stream_port)
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    def grab(self):
        """返回 (frame, mode)；失败返回 (None, None)。"""
        if self._mode in (None, 'capture'):
            frame = self._try_capture()
            if frame is not None:
                self._mode = 'capture'
                return self._post(frame), 'capture'
            if self._mode == 'capture':
                self._mode = None          # 之前可用、现在不可用：下次重新探测

        if self._mode in (None, 'stream'):
            if self._cap is None:
                self._cap = self._open_stream()
            if self._cap is not None:
                ok, frame = self._cap.read()
                if ok and frame is not None:
                    self._mode = 'stream'
                    return self._post(frame), 'stream'
                self._cap.release()        # 流坏了，关掉重建
                self._cap = None
        return None, None

    def _post(self, frame):
        # 与推理节点保持一致：vision_params.yaml 里 flip_code=-1（旋转 180°）
        if self._flip:
            frame = cv2.flip(frame, -1)
        return frame

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


def quality(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return (float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            float(gray.mean()))


def next_path(out_dir, prefix, index, stamp_text):
    name = '%s_%04d_%s.jpg' % (prefix, index, stamp_text)
    path = os.path.join(out_dir, name)
    k = 1
    while os.path.exists(path):          # 绝不静默覆盖
        k += 1
        path = os.path.join(out_dir, name[:-4] + '_%d.jpg' % k)
    return path


def build_parser():
    p = argparse.ArgumentParser(
        description='直接从 ESP32 摄像头抓图，用于采集训练数据。')
    p.add_argument('--host', default='192.168.1.100',
                   help='摄像头 IP（默认 192.168.1.100）')
    p.add_argument('--out', default='~/datasets/shots',
                   help='保存目录（默认 ~/datasets/shots）')
    p.add_argument('--prefix', default='shot', help='文件名前缀')
    p.add_argument('--count', type=int, default=0,
                   help='拍够多少张就退出；0 = 一直拍直到 Ctrl-C')
    p.add_argument('--interval', type=float, default=1.5,
                   help='自动模式下的间隔秒数（默认 1.5）')
    p.add_argument('--manual', action='store_true',
                   help='手动模式：按回车拍一张，输入 q 退出')
    p.add_argument('--capture-port', type=int, default=80,
                   help='/capture 接口端口（默认 80）')
    p.add_argument('--stream-port', type=int, default=81,
                   help='/stream 接口端口（默认 81）')
    p.add_argument('--no-flip', action='store_true',
                   help='不做 180° 旋转（默认与推理节点一致，会转）')
    p.add_argument('--no-open', action='store_true',
                   help='拍完不尝试打开目录')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    sane = clock_is_sane()
    if not sane:
        print('[提示] 系统时钟未同步（当前 %s），文件名改用开机计时；'
              '建议先跑 sudo /home/xmz/fix_clock.sh'
              % time.strftime('%Y-%m-%d %H:%M:%S'))

    grabber = Grabber(args.host, args.capture_port, args.stream_port,
                      flip=not args.no_flip)
    saved = 0
    blurry = 0
    laplacians = []
    brights = []
    last_mode = '-'

    print('保存目录：%s' % out_dir)
    print('摄像头：%s ｜ %s'
          % (args.host, '手动（回车拍）' if args.manual else
             '自动（每 %.1fs 一张）' % args.interval))
    if args.count:
        print('计划拍摄：%d 张' % args.count)
    print('Ctrl-C 随时退出\n')

    try:
        while True:
            if args.manual:
                try:
                    cmd = input('回车拍一张，输入 q 退出 > ').strip().lower()
                except EOFError:
                    break
                if cmd == 'q':
                    break
            else:
                time.sleep(args.interval)
                if args.count and saved >= args.count:
                    break

            frame, mode = grabber.grab()
            if frame is None:
                print('  [失败] 取不到图像：确认摄像头已上电、IP 是 %s'
                      % args.host)
                time.sleep(1.0)
                continue
            if mode != last_mode:
                last_mode = mode
                print('  （取图方式：%s）'
                      % ('/capture 单张' if mode == 'capture' else 'MJPEG 流'))

            lap, bright = quality(frame)
            h, w = frame.shape[:2]
            stamp = ('%d' % time.time()) if sane else \
                ('b%09.3f' % time.monotonic())
            path = next_path(out_dir, args.prefix, saved + 1, stamp)
            if not cv2.imwrite(path, frame):
                print('  [失败] 写盘失败（磁盘满/无权限？）：%s' % path)
                continue

            saved += 1
            laplacians.append(lap)
            brights.append(bright)
            flag = ''
            if lap < SHARP_MIN:
                blurry += 1
                flag = ' ⚠ 偏糊'
            if bright < 40:
                flag += ' ⚠ 偏暗'
            elif bright > 225:
                flag += ' ⚠ 过曝'
            print('  [%d] %dx%d 清晰度%.0f 亮度%.0f%s -> %s'
                  % (saved, w, h, lap, bright, flag, os.path.basename(path)))
    except KeyboardInterrupt:
        print('')
    finally:
        grabber.close()
        print('-' * 52)
        print('共保存 %d 张到 %s' % (saved, out_dir))
        if laplacians:
            print('清晰度：中位 %.0f（低于 %.0f 的 %d 张，建议重拍）'
                  % (np.median(laplacians), SHARP_MIN, blurry))
            print('平均亮度：%.0f' % np.mean(brights))
        if (not args.no_open and saved and shutil.which('xdg-open')
                and os.environ.get('DISPLAY')):
            try:
                os.system('xdg-open "%s" >/dev/null 2>&1 &' % out_dir)
            except OSError as error:  # 打不开图不影响照片已经存好
                print('打不开图片查看器：%s' % error)
    return 0


if __name__ == '__main__':
    sys.exit(main())
