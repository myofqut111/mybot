"""把两路速度指令并成一路，恢复动作优先。

ROS 2 的同一个话题上多个发布者没有优先级：谁后发谁生效。导航在发 0.15
的同时恢复动作发 -0.08，底盘看到的是一串来回跳的数，表现为原地抖、或者
被推着往障碍上顶。

这里做的事很简单：导航和恢复各发各的，这个节点按优先级选一路转发到
/cmd_vel。恢复刚发过东西的那 0.5 秒内只听恢复的，其余时间听导航。
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


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
        # 《补充说明》§3：S1 限速、S3/S4 禁止导航
        self.declare_parameter('state_topic', '/slip/state')
        self.declare_parameter('suspect_speed_cap', 0.10)
        self.declare_parameter('suspect_yaw_cap', 0.50)

        gp = self.get_parameter
        self.slip_hold = float(gp('slip_hold_s').value)
        self.nav_timeout = float(gp('nav_timeout_s').value)
        self.vx_cap = float(gp('suspect_speed_cap').value)
        self.wz_cap = float(gp('suspect_yaw_cap').value)
        self._trusted = 'S0'
        self._nav_clamped = False

        self._nav = None
        self._nav_at = 0.0
        self._slip = None
        self._slip_at = 0.0
        self._source = None
        self._zero = Twist()

        self.pub = self.create_publisher(Twist, gp('out_topic').value, 10)
        self.create_subscription(Twist, gp('nav_topic').value, self._on_nav, 10)
        self.create_subscription(Twist, gp('slip_topic').value, self._on_slip, 10)
        self.create_subscription(String, gp('state_topic').value,
                                 self._on_state, 10)
        self.create_timer(1.0 / max(float(gp('rate_hz').value), 1.0), self._tick)

        self.get_logger().info(
            '速度仲裁就绪：%s（优先）> %s → %s'
            % (gp('slip_topic').value, gp('nav_topic').value,
               gp('out_topic').value))

    def _on_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:                             # noqa: BLE001
            return
        state = data.get('trusted_state')
        if state and state != self._trusted:
            self.get_logger().info('可信状态 %s → %s，导航%s'
                                   % (self._trusted, state,
                                      '放行' if state in ('S0', 'S1') else '禁止'))
            self._trusted = state

    def _clamp(self, msg):
        """S1 限速：方案只要求"立即限速"，这里按参数压 vx/wz。"""
        out = Twist()
        out.linear.x = max(-self.vx_cap, min(self.vx_cap, msg.linear.x))
        out.angular.z = max(-self.wz_cap, min(self.wz_cap, msg.angular.z))
        return out

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
        # §3 表 3-1 + §11.1：S2 起恢复动作独占车轮，S3/S4 定位受限——
        # 都不许 Nav2 直接驱动。只有 S0/S1 让导航走。
        if self._trusted not in ('S0', 'S1'):
            if self._source != 'blocked':
                self._note('blocked')
            self.pub.publish(self._zero)
            return
        nav_live = self._nav is not None and now - self._nav_at <= self.nav_timeout
        if nav_live:
            if self._trusted == 'S1':
                self.pub.publish(self._clamp(self._nav))
                if not self._nav_clamped:
                    self._nav_clamped = True
                    self.get_logger().warn(
                        '候选异常：导航指令限速到 %.2f m/s / %.2f rad/s'
                        % (self.vx_cap, self.wz_cap))
            else:
                self.pub.publish(self._nav)
                if self._nav_clamped:
                    self._nav_clamped = False
                    self.get_logger().info('可信状态恢复，解除限速')
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
        elif source == 'blocked':
            self.get_logger().warn('定位受限/不可判定：导航指令被丢弃，发零速')
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
