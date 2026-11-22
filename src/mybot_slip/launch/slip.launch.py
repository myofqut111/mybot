"""打滑检测节点启动。

只判不动：ros2 launch fishbot_slip slip.launch.py
带恢复：  ros2 launch fishbot_slip slip.launch.py recovery:=true
干跑：    ros2 launch fishbot_slip slip.launch.py recovery:=true executor_dry_run:=true
          （执行器只打日志不发 /cmd_vel，用来先看动作序列对不对）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    recovery = DeclareLaunchArgument(
        'recovery', default_value='false',
        description='是否让 slip_executor 自己动起来（默认只判不动）')
    scan = DeclareLaunchArgument(
        'scan_topic', default_value='/scan_filtered',
        description='用哪一路激光（没有 scan_gate 时传 /scan）')
    cmd = DeclareLaunchArgument(
        'cmd_topic', default_value='/cmd_vel',
        description='执行器往哪里发速度')
    dry = DeclareLaunchArgument(
        'executor_dry_run', default_value='false',
        description='执行器不发速度，只打日志')

    return LaunchDescription([
        recovery,
        scan,
        cmd,
        dry,
        Node(
            package='fishbot_slip',
            executable='slip_node',
            name='slip_node',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'recovery_enabled': LaunchConfiguration('recovery'),
            }],
        ),
        Node(
            package='fishbot_slip',
            executable='slip_executor',
            name='slip_executor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('recovery')),
            parameters=[{
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'dry_run': LaunchConfiguration('executor_dry_run'),
            }],
        ),
    ])
