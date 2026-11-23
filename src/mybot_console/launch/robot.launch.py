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
  ros2 launch mybot_console robot.launch.py localization:=slam   # 边跑边建图
  ros2 launch mybot_console robot.launch.py with_slip:=true      # 开打滑检测+恢复（接管 /cmd_vel）

注意：视觉节点跑在主控板（Jetson）上，不在这里起。只要两边在同一个 ROS_DOMAIN_ID，
控制台和巡检会自动看到它的 /camera/camera/image_raw 与 /patrol/task/result。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
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
    localization = LaunchConfiguration('localization')
    with_slip = LaunchConfiguration('with_slip')
    scan_topic = LaunchConfiguration('scan_topic')
    localization_gate = LaunchConfiguration('localization_gate')
    http_port = LaunchConfiguration('http_port')
    host = LaunchConfiguration('host')

    console_params = PathJoinSubstitution(
        [FindPackageShare('mybot_console'), 'config', 'console.yaml'])

    # 导航段：navigation.launch.py 内部已经串好 TF -> EKF -> 定位 -> Nav2 -> 巡检，
    # 所以这里只把 map / 巡检开关透传下去，不重复起 nav2 的节点。
    # with_slip:=true 时 Nav2 的 /cmd_vel 改成 /cmd_vel_nav，由 slip 的仲裁转发，
    # 否则恢复动作和导航会互相覆盖（两边都在发 /cmd_vel）。
    #
    # 这里必须用 OpaqueFunction 在运行时取参数值，不能在 Python 层拼两个 include：
    # 一旦 include 外面套了 GroupAction，被包含文件里的 TimerAction 会在新的作用域里
    # 找不到自己的 launch configuration，报
    # SubstitutionFailure("launch configuration 'localization_start_delay' does not exist")，
    # 结果是 Nav2 节点起来了但永远没人去 configure/activate。
    def _navigation(context):
        want_slip = LaunchConfiguration('with_slip').perform(context).strip().lower()
        args = {'map_yaml_file': LaunchConfiguration('map_yaml_file').perform(context),
                'enable_patrol': LaunchConfiguration('patrol').perform(context),
                'localization': LaunchConfiguration('localization').perform(context),
                'rviz': LaunchConfiguration('rviz').perform(context),
                'controller': LaunchConfiguration('controller').perform(context),
                'enable_localization_gate':
                    LaunchConfiguration('localization_gate').perform(context)}
        if want_slip in ('1', 'true', 'yes', 'on'):
            args['cmd_vel_topic'] = '/cmd_vel_nav'
        return [_launch_of('mybot_navigation', 'navigation.launch.py', args,
                           condition=IfCondition(nav))]

    navigation = OpaqueFunction(function=_navigation)

    # 打滑检测 + 恢复（含速度仲裁）。默认不开，开了才接管 /cmd_vel。
    slip = _launch_of(
        'mybot_slip', 'slip.launch.py',
        {'recovery': 'true', 'with_mux': 'true',
         'scan_topic': scan_topic, 'nav_topic': '/cmd_vel_nav'},
        condition=IfCondition(with_slip),
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
        DeclareLaunchArgument('localization', default_value='amcl',
                              description='amcl = 用事先建好的地图；slam = slam_toolbox 实时建图'),
        DeclareLaunchArgument('slam_params_file', default_value='',
                              description='slam 模式的参数文件，留空用包内默认'),
        DeclareLaunchArgument('with_slip', default_value='false',
                              description='是否开打滑检测+恢复（会接管 /cmd_vel 做仲裁）'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='打滑检测用的激光话题'),
        DeclareLaunchArgument('localization_gate', default_value='true',
                              description='是否等 AMCL 稳定后再激活 Nav2（无真机 mock 时传 false）'),
        DeclareLaunchArgument('controller', default_value='rpp',
                              description='局部控制器：rpp（默认）或 mppi'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='控制台端口'),
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='监听地址，0.0.0.0 = 局域网可访问'),
        navigation,
        slip,
        patrol_only,
        console,
    ])
