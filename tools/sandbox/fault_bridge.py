#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""故障桥：把 <topic>_raw 转发到 <topic>，转发途中按 faults.json 里的开关注入故障。

    mock_robot --ros-args -r /scan:=/scan_raw      # 原始发布者改名到 _raw
    fault_bridge                                    # 这里注入后发回 /scan
    explore_node                                    # 照常订阅 /scan

这样开不开故障都不用重启，勾上就能看到效果。

faults/*.py 只需提供：
    LABEL = "丢帧"
    TOPIC = "/scan_raw"
    ARGS  = {"rate": 0.1}
    def inject(msg, args, ctx): ...   # 返回消息 或 None(丢弃)
ctx 提供 random() 与 delay(msg, seconds)。
"""
import importlib.util
import json
import os
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

TYPE_TABLE = {
    "scan": ("sensor_msgs.msg", "LaserScan"),
    "odom": ("nav_msgs.msg", "Odometry"),
    "imu": ("sensor_msgs.msg", "Imu"),
    "image": ("sensor_msgs.msg", "Image"),
    "pose": ("geometry_msgs.msg", "PoseStamped"),
}


def load_fault_modules(directory):
    """扫目录，返回 {name: module}；坏文件跳过并留错误。"""
    modules, errors = {}, {}
    if not os.path.isdir(directory):
        return modules, errors
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".py") or name.startswith("_"):
            continue
        path = os.path.join(directory, name)
        try:
            spec = importlib.util.spec_from_file_location("fault_" + name[:-3], path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            if not hasattr(module, "inject"):
                raise AttributeError("缺少 inject(msg, args, ctx)")
            modules[name[:-3]] = module
        except Exception as error:                    # noqa: BLE001
            errors[name[:-3]] = "%s: %s" % (type(error).__name__, error)
    return modules, errors


class Ctx:
    """给故障脚本的小工具：随机数 + 延迟队列。"""

    def __init__(self):
        self.current_key = ""
        self.pending = []           # [(due_ts, topic_key, msg)]

    def random(self):
        return random.random()

    def delay(self, msg, seconds):
        if seconds <= 0:
            return msg
        self.pending.append((time.time() + seconds, self.current_key, msg))
        return None                 # 现在不发，到点了由桥统一发

    def due(self, now):
        ready, keep = [], []
        for item in self.pending:
            (ready if item[0] <= now else keep).append(item)
        self.pending = keep
        return ready


class FaultBridge(Node):
    def __init__(self):
        super().__init__("fault_bridge")
        self.declare_parameter("root", os.path.expanduser("~/mybot_sim"))
        self.declare_parameter("guard_s", 3.0)
        root = self.get_parameter("root").value
        self._faults_dir = os.path.join(root, "faults")
        self._faults_file = os.path.join(root, "faults.json")
        self._guard_s = float(self.get_parameter("guard_s").value)

        self._modules, self._errors = load_fault_modules(self._faults_dir)
        self._active = {}
        self._stamp = 0.0
        self._ctx = Ctx()
        self._pubs = {}             # topic_key -> publisher
        self._subs = {}             # topic_key -> subscription
        self._last_rx = {}          # topic_key -> ts
        self._warned = set()
        self._counts = {}           # topic_key -> [recv, sent, dropped]

        self._reload(force=True)
        self.create_timer(0.05, self._flush)
        self.create_timer(1.0, self._watch)
        self.create_timer(self._guard_s, self._guard)
        for name, error in self._errors.items():
            self.get_logger().warn("故障脚本 %s 载入失败：%s" % (name, error))
        self.get_logger().info("故障桥就绪：%d 个脚本，当前启用 %s"
                               % (len(self._modules), sorted(self._active) or "无"))

    # ---------------------------------------------------------- 配置
    def _read_active(self):
        try:
            with open(self._faults_file, encoding="utf-8") as fh:
                data = json.load(fh)
            return {k: v for k, v in data.items() if k in self._modules}
        except Exception:                              # noqa: BLE001
            return {}

    def _reload(self, force=False):
        try:
            stamp = os.path.getmtime(self._faults_file)
        except OSError:
            stamp = 0.0
        if not force and stamp == self._stamp:
            return
        self._stamp = stamp
        active = self._read_active()
        if active != self._active:
            self._active = active
            self.get_logger().info("启用故障：%s" % (active or "无"))
        self._ensure_pipeline()

    def _topic_key(self, topic):
        return topic.rstrip("/").rsplit("/", 1)[-1]

    @staticmethod
    def _type_key(key):
        """scan_raw -> scan：类型表按基础名登记。"""
        return key[:-4] if key.endswith("_raw") else key

    def _ensure_pipeline(self):
        """为每个可能被注入的 topic 建好 _raw 订阅与正式话题发布器。"""
        wanted = set()
        for module in self._modules.values():
            if getattr(module, "TOPIC", ""):
                wanted.add(module.TOPIC)
        for name in self._active:
            topic = getattr(self._modules[name], "TOPIC", "")
            if topic:
                wanted.add(topic)
        for topic in wanted:
            key = self._topic_key(topic)
            if key in self._pubs:
                continue
            msg_cls = self._message_class(self._type_key(key))
            if msg_cls is None:
                self.get_logger().warn("不知道 %s 的消息类型，跳过" % topic)
                continue
            out = topic[:-4] if topic.endswith("_raw") else topic + "/out"
            self._pubs[key] = self.create_publisher(msg_cls, out, qos_profile_sensor_data)
            self._subs[key] = self.create_subscription(
                msg_cls, topic, lambda msg, k=key: self._on_raw(k, msg),
                qos_profile_sensor_data)
            self._counts[key] = [0, 0, 0]
            self.get_logger().info("管线：%s -> %s（%s）"
                                   % (topic, out, msg_cls.__name__))

    @staticmethod
    def _message_class(key):
        entry = TYPE_TABLE.get(key)
        if not entry:
            return None
        module_name, class_name = entry
        return getattr(importlib.import_module(module_name), class_name)

    # ---------------------------------------------------------- 数据面
    def _on_raw(self, key, msg):
        self._last_rx[key] = time.time()
        self._counts[key][0] += 1
        out = msg
        for name in list(self._active):
            module = self._modules.get(name)
            if module is None or self._topic_key(getattr(module, "TOPIC", "")) != key:
                continue
            args = dict(getattr(module, "ARGS", {}) or {})
            args.update(self._active.get(name) or {})
            self._ctx.current_key = key
            try:
                out = module.inject(out, args, self._ctx)
            except Exception as error:                 # noqa: BLE001
                self.get_logger().warn("故障 %s 执行出错（本次直通）：%s" % (name, error))
                out = msg
            if out is None:
                self._counts[key][2] += 1
                return
        self._publish(key, out)

    def _publish(self, key, msg):
        publisher = self._pubs.get(key)
        if publisher is None:
            return
        publisher.publish(msg)
        self._counts[key][1] += 1

    def _flush(self):
        if not self._ctx.pending:
            return
        for _, key, msg in self._ctx.due(time.time()):
            self._publish(key, msg)

    def _watch(self):
        self._reload()

    def _guard(self):
        now = time.time()
        for key in self._pubs:
            last = self._last_rx.get(key, 0.0)
            if now - last > 2 * self._guard_s and key not in self._warned:
                self._warned.add(key)
                self.get_logger().warn(
                    "没有收到 /%s_raw：场景里要把发布者 remap 到 _raw 才有数据" % key)
        for key, (recv, sent, dropped) in self._counts.items():
            self.get_logger().debug("%s 收 %d 发 %d 丢 %d" % (key, recv, sent, dropped))


def main():
    rclpy.init()
    node = FaultBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
