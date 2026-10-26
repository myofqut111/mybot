import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    nav_share = get_package_share_directory('fishbot_navigation')
    slam_share = get_package_share_directory('slam_toolbox')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # SLAM needs the static base_link -> laser_frame transform.  Keep it in
    # this launch so the documented one-command mapping flow is self-contained.
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('fishbot_description'), 'urdf', 'fishbot.urdf'])
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    # 1) EKF：融合 /odom(编码器) + /imu，发布 /odometry/filtered 和 odom->base_link TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'ekf.yaml')])

    # 2) slam_toolbox 在线异步建图（真机 use_sim_time=false，并加载钉死 base_link
    #    + 抗漂移的自定义参数，不用官方默认的 base_footprint）
    slam_cfg = os.path.join(nav_share, 'config',
                            'mapper_params_online_async.yaml')
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_cfg,
        }.items())

    # 3) RViz 使用低负载配置；Jetson 上默认不启动。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz; keep false on Jetson and run RViz on a PC'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(
                nav_share, 'config', 'rviz_jetson.rviz')),
        robot_state_publisher,
        ekf_node,
        slam_node,
        rviz_node,
    ])
