#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假底盘 v2：能听 /cmd_vel 的简易运动学仿真。

用途：在虚拟机里把 slam_toolbox + nav2 + mybot_explore 串起来跑闭环，
不接任何硬件。车身按 /cmd_vel 积分位姿，激光只有前向 180°，
所以身后一直是"未知"，探索节点才有活干。

发：/tf(odom->base_link) /tf_static(base_link->laser_frame) /scan /odom
收：/cmd_vel
"""
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ROOM_W = 10.0
ROOM_H = 8.0
FOV_HALF = math.radians(90.0)   # 前向 180°


def ray_box(px, py, angle, w, h):
    dx, dy = math.cos(angle), math.sin(angle)
    best = float('inf')
    for wall in (w / 2.0, -w / 2.0):
        if abs(dx) > 1e-9:
            t = (wall - px) / dx
            if t > 0:
                y = py + t * dy
                if -h / 2.0 <= y <= h / 2.0 and t < best:
                    best = t
    for wall in (h / 2.0, -h / 2.0):
        if abs(dy) > 1e-9:
            t = (wall - py) / dy
            if t > 0:
                x = px + t * dx
                if -w / 2.0 <= x <= w / 2.0 and t < best:
                    best = t
    return best


class FakeBase(Node):
    def __init__(self):
        super().__init__('fake_base')
        self.declare_parameter('room_w', ROOM_W)
        self.declare_parameter('room_h', ROOM_H)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        self.w = float(self.get_parameter('room_w').value)
        self.h = float(self.get_parameter('room_h').value)
        self.fov = math.radians(float(self.get_parameter('fov_deg').value))
        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = 0.0
        self.v = 0.0
        self.wz = 0.0
        self.last = time.time()

        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_link'
        st.child_frame_id = 'laser_frame'
        st.transform.rotation.w = 1.0
        self.static.sendTransform(st)

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'fake base v2：听 /cmd_vel，房间 %.1fx%.1f m，视野 %.0f°'
            % (self.w, self.h, math.degrees(self.fov)))

    def on_cmd(self, msg):
        self.v = float(msg.linear.x)
        self.wz = float(msg.angular.z)

    def tick(self):
        now = self.get_clock().now()
        t = time.time()
        dt = max(1e-3, min(0.2, t - self.last))
        self.last = t

        # 简单积分（够用就行）
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt
        # 撞墙就停住（被约束在房间里，留 0.3 m 余量）
        lim_x = self.w / 2.0 - 0.3
        lim_y = self.h / 2.0 - 0.3
        self.x = max(-lim_x, min(lim_x, self.x))
        self.y = max(-lim_y, min(lim_y, self.y))

        tr = TransformStamped()
        tr.header.stamp = now.to_msg()
        tr.header.frame_id = 'odom'
        tr.child_frame_id = 'base_link'
        tr.transform.translation.x = self.x
        tr.transform.translation.y = self.y
        tr.transform.rotation.z = math.sin(self.yaw / 2.0)
        tr.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf.sendTransform(tr)

        od = Odometry()
        od.header.stamp = tr.header.stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = tr.transform.rotation
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.wz
        self.odom_pub.publish(od)

        sc = LaserScan()
        sc.header.stamp = now.to_msg()
        sc.header.frame_id = 'laser_frame'
        sc.angle_min = -self.fov / 2.0
        sc.angle_max = self.fov / 2.0
        sc.angle_increment = math.radians(1.0)
        sc.range_min = 0.12
        sc.range_max = 6.0
        sc.scan_time = 0.1
        # slam_toolbox 要求点数 = round(跨度/步长) + 1（首尾都含）
        n = int(round((sc.angle_max - sc.angle_min) / sc.angle_increment)) + 1
        rng = []
        for i in range(n):
            a = sc.angle_min + i * sc.angle_increment + self.yaw
            d = ray_box(self.x, self.y, a, self.w, self.h)
            if d == float('inf') or d > sc.range_max:
                d = sc.range_max
            elif d < sc.range_min:
                d = sc.range_min
            rng.append(d)
        sc.ranges = rng
        self.scan_pub.publish(sc)


def main():
    rclpy.init()
    node = FakeBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
