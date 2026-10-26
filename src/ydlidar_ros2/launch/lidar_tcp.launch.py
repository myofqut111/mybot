#!/usr/bin/env python3
# WiFi 无线雷达一键启动（TCP 直连，无虚拟串口 pty）
# 上位机监听 8889，雷达作为客户端连入，ydlidar 直接解析 TCP 数据发 /scan
# 用法：ros2 launch ydlidar lidar_tcp.launch.py
# 启动顺序：先运行本 launch（节点进入监听等待），再给雷达上电 / 按 RST，让它连入 8889

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ydlidar_share = get_package_share_directory('ydlidar')
    params_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            ydlidar_share, 'params', 'ydlidar_tcp_server.yaml'),
        description='Path to the ROS2 parameters file to use.')

    # ydlidar_node 是普通 rclcpp 节点，不是 ROS 2 lifecycle 节点。
    # 之前用 LifecycleNode 启动会发送 configure/activate 事件，但可执行程序
    # 根本没有实现这些状态，断线和重启时容易留下半初始化状态。
    lidar_node = Node(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        namespace='/')

    return LaunchDescription([
        params_declare,
        lidar_node,
    ])
