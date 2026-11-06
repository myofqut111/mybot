# -*- coding: utf-8 -*-
"""看门狗节点：车在动却没人下指令时，主动发零速把它停下来。

跑在**虚拟机**上（导航、巡检、控制台都在那儿），所以它能看到"谁还在发指令"。
放在 Jetson 上没用：刹车指令要经过网络才能到底盘，而它要防的正是网络/对端挂掉。
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .guard import Guard

# 同一条刹车原因的最短打印间隔（秒）。20 Hz 的 tick 若不节流，odom 断一次
# 就是几千行 ERROR。
BRAKE_LOG_INTERVAL = 5.0


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')

        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('brake_topic', '')
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('move_epsilon', 0.02)
        self.declare_parameter('brake_rate_hz', 20.0)
        self.declare_parameter('state_topic', 'safety/watchdog')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('enable', True)

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        brake_topic = str(self.get_parameter('brake_topic').value) or cmd_topic
        rate = max(1.0, float(self.get_parameter('brake_rate_hz').value))

        self._enabled = bool(self.get_parameter('enable').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._guard = Guard(
            cmd_timeout=float(self.get_parameter('cmd_timeout_sec').value),
            odom_timeout=float(self.get_parameter('odom_timeout_sec').value),
            move_epsilon=float(self.get_parameter('move_epsilon').value))

        # 指令和里程计都用 BEST_EFFORT：来源可能是 nav2、可能是控制台，
        # 也可能是一段临时脚本，QoS 越宽松越不会因为不匹配而"什么都收不到"。
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        self._brake_pub = self.create_publisher(Twist, brake_topic, 10)
        self._state_pub = self.create_publisher(String, str(
            self.get_parameter('state_topic').value), 10)
        self.create_subscription(Twist, cmd_topic, self._on_cmd, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom,
                                 sensor_qos)

        self._beating = False
        self._brake_count = 0
        self._tick_count = 0
        # 刹车原因的打印节流：每 tick 打一条的话，odom 一断就是 20 条/秒，
        # 实测一次断 135 s 打了 2700 行，日志文件几分钟就上兆，还把 VM 拖慢。
        self._last_brake_log = 0.0
        self._last_brake_reason = ''
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'watchdog watching %s + %s -> %s (cmd_timeout %.2fs, '
            'odom_timeout %.1fs, dry_run=%s)'
            % (cmd_topic, odom_topic, brake_topic,
               self._guard.cmd_timeout, self._guard.odom_timeout,
               self._dry_run))

    # ------------------------------------------------------------ 观测
    def _on_cmd(self, msg):
        if not self._enabled:
            return
        self._guard.observe_cmd(self._now(),
                                msg.linear.x, msg.angular.z)

    def _on_odom(self, msg):
        if not self._enabled:
            return
        self._guard.observe_odom(self._now(),
                                 msg.twist.twist.linear.x,
                                 msg.twist.twist.angular.z)

    def _now(self):
        # 用单调时钟而不是 ROS 时间：只比较"多久没收到"，跟绝对时间无关。
        # 这台机器的系统时间被 NTP/手动校时改过（甚至跳到 1970），
        # 用墙钟会让超时判定忽长忽短，安全逻辑不能依赖它。
        return time.monotonic()

    # ------------------------------------------------------------ 判定
    def _tick(self):
        if not self._enabled:
            return
        self._tick_count += 1
        tripped, reason = self._guard.evaluate(self._now())
        if tripped:
            if reason:
                now = self._now()
                # 同一条原因最多每 BRAKE_LOG_INTERVAL 秒打一次；原因变了立刻打，
                # 这样既看得见"什么时候开始刹的"，也不会把日志刷爆。
                if (reason != self._last_brake_reason
                        or (now - self._last_brake_log) >= BRAKE_LOG_INTERVAL):
                    self._last_brake_log = now
                    self._last_brake_reason = reason
                    self.get_logger().error('BRAKING: %s' % reason)
            self._publish_brake()
        elif self._beating:
            self.get_logger().info('watchdog released: robot is under control '
                                   'again')
        self._beating = tripped
        # 变状态时立刻推一次，其余按每秒一次刷心跳，别把 20 Hz 全用来发状态
        self._publish_state(force=bool(reason))

    def _publish_brake(self):
        self._brake_count += 1
        if self._dry_run:
            return
        msg = Twist()          # 全零 = 停
        self._brake_pub.publish(msg)

    def _publish_state(self, force=False):
        if not force and (self._tick_count % 20) != 0:
            return
        snap = self._guard.snapshot()
        snap['brakes_sent'] = self._brake_count
        snap['dry_run'] = self._dry_run
        msg = String()
        msg.data = json.dumps(snap, ensure_ascii=False)
        self._state_pub.publish(msg)

    def destroy_node(self):
        # 节点要退了，也留一帧零速：底盘的 target 是"最后一条指令"，
        # 一帧零就够它停住，不必等别人来接。
        if self._enabled and not self._dry_run:
            try:
                self._brake_pub.publish(Twist())
            except Exception as error:
                self.get_logger().warning('退出时补发零速失败：%s' % error)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
