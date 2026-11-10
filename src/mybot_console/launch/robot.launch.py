#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整机启动：一条命令把导航 + 巡检 + 控制台全拉起来。

为什么要有这个文件：`console.launch.py` 只起控制台本身，导航和巡检得另外开两个
终端先起好，否则页面打开是一片空白——那不叫控制台，叫遥控器的外壳。这里把三段
串成一条命令，默认全开，需要时再用参数单独关掉。

用法：
  ros2 launch mybot_console robot.launch.py
  ros2 launch mybot_console robot.launch.py map_yaml_file:=~/maps/lab.yaml
  ros2 launch mybot_console robot.launch.py nav:=false      # 只起巡检 + 控制台
  ros2 launch mybot_console robot.launch.py patrol:=false   # 只起导航 + 控制台

注意：视觉节点跑在主控板（Jetson）上，不在这里起。只要两边在同一个 ROS_DOMAIN_ID，
控制台和巡检会自动看到它的 /camera/camera/image_raw 与 /patrol/task/result。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_of(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    nav = LaunchConfiguration('nav')
    patrol = LaunchConfiguration('patrol')
    rviz = LaunchConfiguration('rviz')
    http_port = LaunchConfiguration('http_port')
    host = LaunchConfiguration('host')

    console_params = PathJoinSubstitution(
        [FindPackageShare('mybot_console'), 'config', 'console.yaml'])

    # 导航段：navigation.launch.py 内部已经串好 TF -> EKF -> 定位 -> Nav2 -> 巡检，
    # 所以这里只把 map / 巡检开关透传下去，不重复起 nav2 的节点。
    navigation = _launch_of(
        'mybot_navigation', 'navigation.launch.py',
        {'map_yaml_file': map_yaml_file,
         'enable_patrol': patrol,
         'rviz': rviz},
        condition=IfCondition(nav),
    )

    # nav:=false 但仍要巡检时，单独起巡检栈。
    # 两个条件用嵌套表达成「与」：外层 patrol 开关，内层 nav 关掉才生效，
    # 这样 nav:=true 时不会起两份巡检。
    patrol_only = GroupAction(
        [_launch_of('mybot_patrol_v2', 'patrol_v2.launch.py',
                    {'nav': 'false', 'console': 'false'},
                    condition=UnlessCondition(nav))],
        condition=IfCondition(patrol),
    )

    console = Node(
        package='mybot_console',
        executable='robot_console',
        name='mybot_console',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[console_params,
                    {'http_port': http_port, 'http_host': host}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(os.path.expanduser('~'), 'maps', 'maps.yaml'),
            description='已存好的地图 yaml（pgm 在同一个目录）'),
        DeclareLaunchArgument('nav', default_value='true',
                              description='是否起导航段（nav2 + AMCL + EKF）'),
        DeclareLaunchArgument('patrol', default_value='true',
                              description='是否起巡检栈（路线/任务管理器 + 执行器）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否顺便起 RViz（要装 nav2-bringup）'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='控制台端口'),
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='监听地址，0.0.0.0 = 局域网可访问'),
        navigation,
        patrol_only,
        console,
    ])
