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
    """
    一键建图：URDF/TF + EKF + slam_toolbox + RViz，一条命令拉起。
    不在本 launch 内的（需自行先开好）：
      1. micro-ROS Agent（ESP32 接入，发 /odom /imu）
      2. 雷达 docker（菜单选 1 驱动雷达，发 /scan，--network host）
    键盘遥控 teleop_twist_keyboard 因要独占键盘输入，另开一个终端跑。
    """
    nav_share = get_package_share_directory('fishbot_navigation')
    slam_share = get_package_share_directory('slam_toolbox')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # 1) robot_state_publisher：读 URDF，发 base_link -> laser_frame 的 /tf_static
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('fishbot_description'), 'urdf', 'fishbot.urdf'])
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    # 2) EKF：融合 /odom + /imu，发 /odometry/filtered 和 odom -> base_link TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'ekf.yaml')])

    # 3) slam_toolbox 在线异步建图：发 map -> odom TF 和 /map
    #    自定义配置钉死 base_frame=base_link，并加强回环/更信激光
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(
                nav_share, 'config', 'mapper_params_online_async.yaml'),
        }.items())

    # 4) RViz 使用低负载配置；Jetson 上默认不启动。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen')

    # 启动顺序：先 TF/EKF，SLAM/RViz 随后；SLAM 头几秒若提示等 TF 属正常，
    # 等 Agent 连上、EKF 收到 /odom 后即恢复。
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
