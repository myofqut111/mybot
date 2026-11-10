# -*- coding: utf-8 -*-
"""看门狗。参数默认值与 `config/watchdog.yaml` 一致，改一处记得改另一处。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mybot_safety')
    params = os.path.join(share, 'config', 'watchdog.yaml')
    return LaunchDescription([
        Node(
            package='mybot_safety',
            executable='safety_watchdog',
            name='safety_watchdog',
            output='screen',
            parameters=[params],
        ),
    ])
