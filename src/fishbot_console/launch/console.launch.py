#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制台启动文件：只起 Web 控制台，导航/巡检由各自的 launch 负责。

用法：
  ros2 launch fishbot_console console.launch.py
  ros2 launch fishbot_console console.launch.py port:=9000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('fishbot_console'), 'config', 'console.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        Node(
            package='fishbot_console',
            executable='robot_console',
            name='fishbot_console',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                params,
                {'http_port': LaunchConfiguration('port'),
                 'http_host': LaunchConfiguration('host')},
            ],
        ),
    ])
