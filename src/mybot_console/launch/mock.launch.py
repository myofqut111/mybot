"""mock.launch.py：不用开小车，把控制台 + 实时流 + 假机器人一起拉起来。

验证用，不接真硬件：
    ros2 launch mybot_console mock.launch.py
    # 浏览器打开 http://127.0.0.1:8080

假机器人会发一张 10 m x 10 m 的房间地图、20 Hz 的 TF、10 Hz 的点云，
并且假装一个 NavigateToPose 服务端 —— 于是"地图上点一下发目标"、
"手动按住方向键开"，在没有真车的情况下也能走完整条链路。
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    console = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mybot_console'), 'launch',
                'console.launch.py'])),
    )

    mock = Node(
        package='mybot_console',
        executable='mock_robot',
        name='mock_robot',
        output='screen',
    )

    return LaunchDescription([console, mock])
