"""一键启动巡检系统。

默认拉起三个节点，一条命令就能跑：路线管理器、任务管理器、巡检执行器。
视觉节点和导航默认不拉（它们在别的机器上跑），需要时把开关打开。

用法::

    ros2 launch mybot_patrol_v2 patrol_v2.launch.py
    ros2 launch mybot_patrol_v2 patrol_v2.launch.py vision:=true nav:=true
    ros2 launch mybot_patrol_v2 patrol_v2.launch.py failure_policy:=stop resume:=false
    ros2 launch mybot_patrol_v2 patrol_v2.launch.py task_manager:=false

任何一个节点挂掉时 ``respawn`` 会把它拉起来；Ctrl+C 一次停全部。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, filename, condition):
    """按包名包含另一个 launch；包或文件不存在时只打日志，不中断启动。"""
    try:
        share = get_package_share_directory(package)
    except Exception:                                  # noqa: BLE001
        return [LogInfo(condition=condition,
                        msg='package %r not installed, skipped' % package)]
    path = os.path.join(share, 'launch', filename)
    if not os.path.exists(path):
        return [LogInfo(condition=condition,
                        msg='%s/launch/%s not found, skipped'
                            % (package, filename))]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), condition=condition)]


def generate_launch_description():
    """生成 launch 描述。"""
    share = get_package_share_directory('mybot_patrol_v2')
    params_file = os.path.join(share, 'config', 'patrol_v2.yaml')

    policy = LaunchConfiguration('failure_policy')
    resume = LaunchConfiguration('resume')
    report_dir = LaunchConfiguration('report_dir')
    task_timeout = LaunchConfiguration('task_timeout_s')
    use_route = LaunchConfiguration('route_manager')
    use_task = LaunchConfiguration('task_manager')
    use_vision = LaunchConfiguration('vision')
    use_nav = LaunchConfiguration('nav')
    use_console = LaunchConfiguration('console')

    nodes = [
        # 1. 路线管理器：记录/查询路线（存 ~/.ros/mybot_patrol_routes.json）
        Node(
            package='mybot_patrol_route_manager',
            executable='route_manager',
            name='route_manager',
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(use_route),
        ),
        # 2. 任务管理器：把点位任务派给视觉，收结果、超时兜底
        Node(
            package='mybot_patrol_task_manager',
            executable='patrol_task_manager',
            name='patrol_task_manager',
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'task_timeout_s': task_timeout}],
            condition=IfCondition(use_task),
        ),
        # 3. 巡检执行器：导航 + 调度 + 落盘报告
        Node(
            package='mybot_patrol_v2',
            executable='patrol_executor_v2',
            name='patrol_executor_v2',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'failure_policy': policy,
                    'resume': resume,
                    'report_dir': report_dir,
                },
            ],
        ),
        # 4. 网页控制台：浏览器/手机远程操作（默认不开，占用 8080 端口）
        Node(
            package='mybot_patrol_v2',
            executable='patrol_console',
            name='patrol_console',
            output='screen',
            emulate_tty=True,
            parameters=[{'reports_dir': report_dir}],
            condition=IfCondition(use_console),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'failure_policy', default_value='skip',
            description='单点失败后的行为：stop / skip / retry'),
        DeclareLaunchArgument(
            'resume', default_value='true',
            description='是否从上一次进度继续跑'),
        DeclareLaunchArgument(
            'report_dir', default_value='~/patrol_reports',
            description='巡检报告输出目录'),
        DeclareLaunchArgument(
            'task_manager', default_value='true',
            description='是否同时启动任务管理器'),
        DeclareLaunchArgument(
            'route_manager', default_value='true',
            description='是否同时启动路线管理器'),
        DeclareLaunchArgument(
            'task_timeout_s', default_value='120.0',
            description='单个点位任务的超时（秒）'),
        DeclareLaunchArgument(
            'console', default_value='false',
            description='是否同时启动网页控制台（http://<本机IP>:8080）'),
        DeclareLaunchArgument(
            'vision', default_value='false',
            description='是否同时启动视觉节点（本机装了 mybot_vision 才有用）'),
        DeclareLaunchArgument(
            'nav', default_value='false',
            description='是否同时启动导航（nav2 + 定位，通常跑在另一台机器上）'),
    ] + nodes
        + _include('mybot_vision', 'vision.launch.py', IfCondition(use_vision))
        + _include('mybot_navigation', 'navigation.launch.py', IfCondition(use_nav)))
