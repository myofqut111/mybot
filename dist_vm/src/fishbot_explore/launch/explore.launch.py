"""自主探索启动文件。

看图（默认）：
    ros2 launch fishbot_explore explore.launch.py
开车：
    ros2 launch fishbot_explore explore.launch.py drive:=true
真正开跑：
    ros2 service call /explore/start std_srvs/srv/Trigger
（想一起就自动跑，加 autostart:=true）

RViz 里加 MarkerArray，话题 /explore/frontiers，就能看到红点。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fishbot_explore')
    default_params = os.path.join(share, 'config', 'explore.yaml')

    params = LaunchConfiguration('params_file')
    drive = LaunchConfiguration('drive')
    autostart = LaunchConfiguration('autostart')

    node = Node(
        package='fishbot_explore',
        executable='explore_node',
        name='explore_node',
        output='screen',
        parameters=[params, {'drive': drive, 'autostart': autostart}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='参数文件（默认包里的 explore.yaml）'),
        DeclareLaunchArgument('drive', default_value='false',
                              description='true=真的发 NavigateToPose 让车走'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true=一起就自动开始，不用调服务'),
        node,
    ])
