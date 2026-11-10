"""Start the mybot patrol task manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    timeout = LaunchConfiguration('task_timeout_s')
    return LaunchDescription([
        DeclareLaunchArgument(
            'task_timeout_s',
            default_value='120.0',
            description='Maximum time to wait for one patrol task result.'),
        Node(
            package='mybot_patrol_task_manager',
            executable='patrol_task_manager',
            name='patrol_task_manager',
            output='screen',
            parameters=[{'task_timeout_s': timeout}],
        ),
    ])
