"""打滑检测节点启动。

单独起：ros2 launch fishbot_slip slip.launch.py
带恢复：ros2 launch fishbot_slip slip.launch.py recovery:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    recovery = DeclareLaunchArgument(
        'recovery', default_value='false',
        description='是否让 slip_executor 自己动起来（默认只判不动）')
    scan = DeclareLaunchArgument(
        'scan_topic', default_value='/scan_filtered',
        description='用哪一路激光（没有 scan_gate 时传 /scan）')

    return LaunchDescription([
        recovery,
        scan,
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
    ])
