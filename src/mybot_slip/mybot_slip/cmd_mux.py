"""把两路速度指令并成一路，恢复动作优先。

ROS 2 的同一个话题上多个发布者没有优先级：谁后发谁生效。导航在发 0.15
的同时恢复动作发 -0.08，底盘看到的是一串来回跳的数，表现为原地抖、或者
被推着往障碍上顶。

这里做的事很简单：导航和恢复各发各的，这个节点按优先级选一路转发到
/cmd_vel。恢复刚发过东西的那 0.5 秒内只听恢复的，其余时间听导航。
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def _is_zero(msg):
    return (abs(msg.linear.x) < 1e-4 and abs(msg.linear.y) < 1e-4
            and abs(msg.linear.z) < 1e-4 and abs(msg.angular.x) < 1e-4
            and abs(msg.angular.y) < 1e-4 and abs(msg.angular.z) < 1e-4)


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('slip_topic', '/cmd_vel_slip')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('slip_hold_s', 0.5)     # 恢复最后一条指令的有效期
        self.declare_parameter('nav_timeout_s', 0.5)   # 导航多久没发就算断了
        self.declare_parameter('rate_hz', 20.0)

        gp = self.get_parameter
        self.slip_hold = float(gp('slip_hold_s').value)
        self.nav_timeout = float(gp('nav_timeout_s').value)

        self._nav = None
        self._nav_at = 0.0
        self._slip = None
        self._slip_at = 0.0
        self._source = None
        self._zero = Twist()

        self.pub = self.create_publisher(Twist, gp('out_topic').value, 10)
        self.create_subscription(Twist, gp('nav_topic').value, self._on_nav, 10)
        self.create_subscription(Twist, gp('slip_topic').value, self._on_slip, 10)
        self.create_timer(1.0 / max(float(gp('rate_hz').value), 1.0), self._tick)

        self.get_logger().info(
            '速度仲裁就绪：%s（优先）> %s → %s'
            % (gp('slip_topic').value, gp('nav_topic').value,
               gp('out_topic').value))

    def _on_nav(self, msg):
        self._nav = msg
        self._nav_at = time.monotonic()

    def _on_slip(self, msg):
        self._slip = msg
        self._slip_at = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        slip_live = self._slip is not None and now - self._slip_at <= self.slip_hold
        if slip_live:
            self.pub.publish(self._slip)
            self._note('slip')
            return
        nav_live = self._nav is not None and now - self._nav_at <= self.nav_timeout
        if nav_live:
            self.pub.publish(self._nav)
            self._note('nav')
            return
        # 两路都不发：停住（底盘自己也有看门狗，这里只是别留最后一条）
        if self._source is not None:
            self._note('idle')
        self.pub.publish(self._zero)

    def _note(self, source):
        if source == self._source:
            return
        self._source = source
        if source == 'slip':
            self.get_logger().warn('速度指令让给恢复动作')
        elif source == 'nav':
            self.get_logger().info('速度指令回到导航')
        else:
            self.get_logger().info('两路都没在发，发零速')


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:        # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
