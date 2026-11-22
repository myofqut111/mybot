#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""执行器：把 /slip/action 变成真正的运动，再把结果报回去。

    slip_node  ──/slip/action (JSON)──>  slip_executor
                                          │
                                          ├─ back   : 直发 /cmd_vel 倒车，按里程计闭环
                                          ├─ escape : 原地反向转，按陀螺/里程计闭环
                                          │
    slip_node  <──/slip/report (JSON)─────┘

为什么直发 cmd_vel 而不是用 Nav2 的 BackUp / Spin：
这两者要走 `navigate_to_pose` 的生命周期与 BT，而打滑时车往往正处于
一次导航任务里——借用 Nav2 动作会和当前任务抢执行权。这里只发速度指令，
Nav2 那边由它自己的碰撞检查兜底（真机验证时再看是否需要接管）。

安全：动作只在 `recovery_enabled` 打开时才会被下发；动作结束（成功/超时/
被新指令打断）一定发一帧 0 速度，绝不让车保持惯性指令。
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from mybot_slip import ranges as ranges_mod
from mybot_slip.motion import TravelMonitor


IDLE = 'idle'
RUNNING = 'running'


def _yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    # roll/pitch 视为 0（地面车），只要 yaw
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SlipExecutor(Node):
    def __init__(self):
        super().__init__('slip_executor')
        gp = self.declare_parameter
        gp('action_topic', '/slip/action')
        gp('report_topic', '/slip/report')
        gp('cmd_topic', '/cmd_vel')
        gp('odom_topic', '/odom')
        gp('imu_topic', '/imu')
        gp('scan_topic', '/scan')
        gp('rate_hz', 20.0)
        gp('back_speed', 0.08)
        gp('escape_speed', 1.2)
        gp('settle_s', 0.3)
        gp('measure', 'scan')           # scan=用激光量退了多少；odom=用里程计
        gp('dry_run', False)            # True：只打日志不发 cmd_vel

        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dry_run = bool(get('dry_run'))
        self.measure = str(get('measure')).lower()
        self.back_speed = float(get('back_speed'))
        self.escape_speed = float(get('escape_speed'))
        self.settle_s = float(get('settle_s'))
        rate = float(get('rate_hz'))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, get('action_topic'), self._on_action, 10)
        self.create_subscription(Odometry, get('odom_topic'), self._on_odom, qos)
        self.create_subscription(Imu, get('imu_topic'), self._on_imu, 20)
        self.create_subscription(LaserScan, get('scan_topic'), self._on_scan, 5)
        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 10)
        self.report_pub = self.create_publisher(String, get('report_topic'), 10)

        self.create_timer(1.0 / rate, self._tick)

        self.status = IDLE
        self.action = None
        self.monitor = None
        self.started_at = 0.0
        self.deadline = 0.0
        self.pose = None
        self.stop_until = 0.0
        self._scan_prev = None
        self._odom_travel = 0.0
        self._odom_last = None
        self._imu_yaw = None
        self._imu_last = None
        self.get_logger().info(
            '执行器就绪：动作来自 %s，结果发 %s（位移量用 %s，%s）'
            % (get('action_topic'), get('report_topic'), self.measure,
               'dry_run，不发速度' if self.dry_run else '会发速度'))

    # ------------------------------------------------------------------
    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self.pose = (pos.x, pos.y, _yaw_from_odom(msg))
        if self._odom_last is not None:
            self._odom_travel += math.hypot(pos.x - self._odom_last[0],
                                            pos.y - self._odom_last[1])
        self._odom_last = (pos.x, pos.y)

    def _on_imu(self, msg):
        """陀螺的偏航角速度：不依赖轮子，脱困转了多少只能信它。"""
        w = msg.angular_velocity.z
        now = self._now()
        if self._imu_last is not None and self.status == RUNNING \
                and (self.action or {}).get('kind') == 'escape':
            dt = max(min(now - self._imu_last, 0.2), 0.0)
            self._imu_last = now
            if self.monitor is not None and dt > 0:
                self.monitor.add_yaw(w * dt)
        else:
            self._imu_last = now

    def _on_scan(self, msg):
        """激光量位移：打滑时里程计正是在骗人的那一个，不能拿它验自己。"""
        prev, self._scan_prev = self._scan_prev, msg
        if self.status != RUNNING or (self.action or {}).get('kind') != 'back':
            return
        if prev is None or self.monitor is None:
            return
        try:
            fit = ranges_mod.fit_translation(
                {'ranges': list(prev.ranges), 'range_max': float(prev.range_max)},
                {'ranges': list(msg.ranges), 'range_max': float(msg.range_max)})
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn('位移测量失败：%s' % exc)
            return
        if not fit.get('ok') or (fit.get('quality') or 0.0) < 0.35:
            return
        self.monitor.add_distance(math.hypot(fit.get('dx') or 0.0,
                                             fit.get('dy') or 0.0))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_action(self, msg):
        try:
            act = json.loads(msg.data)
        except ValueError:
            self.get_logger().error('动作不是合法 JSON：%s' % msg.data[:80])
            return
        kind = act.get('kind')
        if not kind:
            return
        if self.status == RUNNING:
            # 上一次还没做完：先停住再报失败，让上层知道没执行
            self.get_logger().warn('上一次动作未完成，忽略新动作 %s' % kind)
            self._report(act, ok=False, moved=0.0, why='上一次动作还在执行')
            return
        if kind == 'abort':
            self.get_logger().warn('上层要求放弃：%s' % act.get('why'))
            self._report(act, ok=True, moved=0.0, why='abort 已收到')
            return
        self._start(act)

    # ------------------------------------------------------------------
    def _start(self, act):
        kind = act.get('kind')
        if kind == 'back':
            target = float(act.get('distance') or 0.0)
            monitor = TravelMonitor('back', target)
        elif kind == 'escape':
            target = float(act.get('yaw') or 0.9)
            monitor = TravelMonitor('escape', target)
        else:
            self.get_logger().warn('不认识的动作品类：%s' % kind)
            self._report(act, ok=False, moved=0.0, why='不认识的动作品类')
            return

        now = time.monotonic()
        self.action = dict(act)
        self.monitor = monitor
        self.status = RUNNING
        self.started_at = now
        self.deadline = now + float(act.get('timeout_s') or 6.0)
        self._odom_travel = 0.0
        self._odom_last = None
        self._scan_prev = None          # 丢掉动作前的旧帧，第一对差分才算数
        if self.measure == 'odom' and self.pose is not None:
            self.monitor.update(*self.pose)
        self.get_logger().warn('执行动作 %s：目标 %.2f（量法 %s，%s）'
                               % (kind, monitor.target, self.measure,
                                  act.get('why') or ''))

    def _tick(self):
        if self.status != RUNNING:
            if self.stop_until and time.monotonic() >= self.stop_until:
                self.stop_until = 0.0
            return
        now = time.monotonic()
        if self.measure == 'odom' and self.pose is not None:
            self.monitor.update(*self.pose)

        if self.monitor.done():
            self._finish(ok=True, why='动作完成')
            return
        if now > self.deadline:
            self._finish(ok=False, why='动作超时')
            return

        act = self.action
        twist = Twist()
        if act['kind'] == 'back':
            twist.linear.x = -abs(self.back_speed)
        else:
            sign = 1.0 if (act.get('yaw') or 0.0) >= 0 else -1.0
            # 优先往反方向转：卡住时贴着障碍转身更容易脱困
            twist.angular.z = -sign * abs(self.escape_speed)
            # 里程计不可信时也能靠时间兜底：不额外处理，超时即结束
        if not self.dry_run:
            self.cmd_pub.publish(twist)

    def _finish(self, ok, why):
        act = self.action or {}
        moved = self.monitor.progress() if self.monitor else 0.0
        odom_moved = self._odom_travel
        self._stop()
        self.status = IDLE
        self.action = None
        self.monitor = None
        self.get_logger().warn(
            '动作结束：%s（激光 %.3f m，里程计 %.3f m，%s）'
            % ('成功' if ok else '失败', moved, odom_moved, why))
        if ok and self.measure == 'scan' and odom_moved > 0.05 \
                and moved < 0.5 * odom_moved:
            # 轮子转了、车没走：动作"成功"是假的，不能说恢复成功
            ok = False
            why = '里程计说走了 %.2f m，激光只有 %.2f m（轮子空转）' % (odom_moved, moved)
            self.get_logger().warn(why)
        self._report(act, ok=ok, moved=moved, why=why, odom_moved=odom_moved)

    def _stop(self):
        if self.dry_run:
            return
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.stop_until = time.monotonic() + self.settle_s

    def _report(self, act, ok, moved, why, odom_moved=None):
        payload = {
            'id': act.get('id'),
            'kind': act.get('kind'),
            'ok': bool(ok),
            'moved': round(float(moved or 0.0), 4),
            'why': why,
            'at': round(time.time(), 3),
        }
        if odom_moved is not None:
            payload['odom_moved'] = round(float(odom_moved), 4)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.report_pub.publish(msg)
        self.get_logger().info('上报动作结果：%s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SlipExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._stop()
        except Exception:      # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
