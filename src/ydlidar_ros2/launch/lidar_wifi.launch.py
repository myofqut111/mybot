#!/usr/bin/env python3
# WiFi 无线雷达一键启动：tcp_server(ros_serail2wifi) + ydlidar_node(ydlidar)
# 用法：ros2 launch ydlidar lidar_wifi.launch.py

import os

import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ydlidar_share = get_package_share_directory('ydlidar')
    params_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ydlidar_share, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.')
    idle_timeout_declare = DeclareLaunchArgument(
        'idle_timeout_sec', default_value='30.0',
        description='TCP idle watchdog in seconds; set 0 to disable it')

    # 1) WiFi TCP -> 虚拟串口 /tmp/tty_laser（雷达转接板连板子的 8889）
    tcp_node = Node(
        package='ros_serail2wifi',
        executable='tcp_server',
        name='tcp_socket_server_node',
        output='screen',
        parameters=[{
            'tcp_port': 8889,
            'serial_port': '/tmp/tty_laser',
            'idle_timeout_sec': LaunchConfiguration('idle_timeout_sec'),
        }])

    # 2) 普通 ydlidar 节点；延迟 2 秒启动，等 tcp_server 把虚拟串口符号链接建好
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
        idle_timeout_declare,
        tcp_node,
        TimerAction(period=2.0, actions=[lidar_node]),
    ])
