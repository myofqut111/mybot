#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制台启动文件：只起 Web 控制台，导航/巡检由各自的 launch 负责。

用法：
  ros2 launch fishbot_console console.launch.py
  ros2 launch fishbot_console console.launch.py port:=9000
  ros2 launch fishbot_console console.launch.py rosbridge:=false   # 不要实时流

rosbridge 用来把 /tf 和 /scan 推给浏览器（页头那个"实时 已连"）。
它是**可选依赖**：没装 rosbridge_server 时这里只打一条警告，控制台照常起，
页面自动退回轮询，同一份代码在装了/没装的两台机器上都能跑。
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROSBRIDGE_PORT = '9090'


def _rosbridge_include():
    """装了 rosbridge_server 返回 (include, None)，没装返回 (None, 原因)。

    两个坑都在这一个函数里：

    1. rosbridge 官方的 launch 是 **.xml**，必须用 AnyLaunchDescriptionSource
       按后缀分派。用 PythonLaunchDescriptionSource 去解析会直接抛
       `invalid syntax (rosbridge_websocket_launch.xml, line 1)` —— 之前就是这么炸的。
    2. 探测"包装没装"必须**当场**做（get_package_share_directory 会真抛异常），
       不能用 FindPackageShare 这种惰性替换：那样 try/except 永远不触发，
       没装的时候不会走警告分支，而是到运行期才崩。
    """
    try:
        get_package_share_directory('rosbridge_server')
    except Exception as error:              # PackageNotFoundError 等
        return None, '%s' % error

    try:
        include = IncludeLaunchDescription(
            AnyLaunchDescriptionSource((FindPackageShare('rosbridge_server')
                                        / 'launch'
                                        / 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': ROSBRIDGE_PORT}.items(),
            condition=IfCondition(LaunchConfiguration('rosbridge')),
        )
    except Exception as error:
        return None, '%s' % error
    return include, None


def generate_launch_description() -> LaunchDescription:
    params = FindPackageShare('fishbot_console') / 'config' / 'console.yaml'

    include, reason = _rosbridge_include()
    if include is not None:
        bridge_actions = [
            LogInfo(msg='[fishbot_console] rosbridge 实时流已启动（端口 %s）'
                        % ROSBRIDGE_PORT),
            include,
        ]
    else:
        bridge_actions = [
            LogInfo(msg=(
                '[fishbot_console] 没找到 rosbridge_server（%s）。'
                '实时流不可用，页面会自动退回轮询；需要时执行 '
                'sudo apt install ros-$ROS_DISTRO-rosbridge-suite' % reason)),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument(
            'rosbridge', default_value='true',
            description='是否同时启动 rosbridge 实时流（需 rosbridge_server）'),
    ] + bridge_actions + [
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
