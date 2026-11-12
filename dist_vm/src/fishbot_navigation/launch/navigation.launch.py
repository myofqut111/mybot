"""
fishbot 导航主启动文件（ROS2 Humble / 两轮差速 ESP32 小车）

由 Go2 navigation.launch.py 移植，关键差异：
  - 删除 FAST-LIO、pointcloud_to_laserscan、go2 运动桥、go2 bringup
    （2D 雷达直接发 /scan；ESP32 底盘直接订阅 /cmd_vel，无需运动桥）
  - 新增 robot_localization 的 ekf_node：融合 /odom + /imu，
    产出 /odometry/filtered 与 odom -> base_link TF（替代 FAST-LIO 里程计）
  - recoveries_server(Foxy) -> behavior_server(Humble)
  - 控制器不再用自定义 rotation_shim，改由 nav2_params 里的原生 RPP 实现原地转向

启动时序（这套"门控"是原工程的安全精华，予以保留）：
  T0          EKF + map_server/amcl + Nav2四件套 全部启动(configure)
  T0+delay    lifecycle_manager_localization 启动并激活 map_server+amcl
  期间        localization_gate 持续检查 AMCL 协方差与 TF 稳定性
  门控通过    gate 进程退出 -> OnProcessExit 才启动 lifecycle_manager_navigation
              -> 激活 controller/planner/behavior/bt_navigator
  效果：定位没稳定前，Nav2 不会激活，机器人不会乱跑。

前置条件（请在另外终端先启动，本 launch 不负责拉起，方便分步调试）：
  1. micro-ROS agent（ESP32 连上后发布 /odom、/imu）
  2. 2D 激光雷达驱动（发布 /scan）
  3. robot_state_publisher + URDF（提供 base_link -> 雷达 frame 的 TF）

启动示例：
  ros2 launch fishbot_navigation navigation.launch.py \
      map_yaml_file:=/home/<user>/maps/room.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ------------------------- 可配置参数 -------------------------
    declare_args = [
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(os.path.expanduser('~'), 'maps', 'maps.yaml'),
            description='Saved map yaml (pgm 同目录)。默认 ~/maps/maps.yaml，可传参覆盖。'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the Nav2 stack'),
        DeclareLaunchArgument(
            'localization_start_delay', default_value='3.0',
            description='Seconds to wait for EKF/scan/TF before activating map_server+AMCL'),
        DeclareLaunchArgument(
            'enable_localization_gate', default_value='true',
            description='Wait for stable AMCL/TF before activating Nav2; false starts Nav2 directly'),
        # 下面 6 个是定位门控 wait_for_localization.py 的参数
        DeclareLaunchArgument(
            'gate_stable_duration', default_value='2.0',
            description='Gate: seconds of stable TF before Nav2 activation'),
        DeclareLaunchArgument(
            'gate_pose_max_age', default_value='5.0',
            description='Gate: max /amcl_pose age in seconds'),
        DeclareLaunchArgument(
            'gate_max_xy_std', default_value='0.15',
            description='Gate: max allowed AMCL xy standard deviation (m)'),
        DeclareLaunchArgument(
            'gate_max_yaw_std', default_value='0.15',
            description='Gate: max allowed AMCL yaw standard deviation (rad)'),
        DeclareLaunchArgument(
            'gate_max_position_jitter', default_value='0.05',
            description='Gate: max TF position jitter over the stable window (m)'),
        DeclareLaunchArgument(
            'gate_max_yaw_jitter', default_value='0.05',
            description='Gate: max TF yaw jitter over the stable window (rad)'),
        DeclareLaunchArgument(
            'enable_ekf', default_value='true',
            description='Launch robot_localization ekf_node in this file'),
        DeclareLaunchArgument(
            'enable_patrol', default_value='true',
            description='Launch the fishbot patrol stack (route/task manager + executor)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz with the default Nav2 config (needs nav2-bringup)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(
                get_package_share_directory('fishbot_navigation'),
                'config', 'rviz_jetson.rviz'),
            description='RViz config; the bundled config keeps only low-cost displays'),
    ]

    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    localization_start_delay = LaunchConfiguration('localization_start_delay')
    enable_localization_gate = LaunchConfiguration('enable_localization_gate')
    gate_stable_duration = LaunchConfiguration('gate_stable_duration')
    gate_pose_max_age = LaunchConfiguration('gate_pose_max_age')
    gate_max_xy_std = LaunchConfiguration('gate_max_xy_std')
    gate_max_yaw_std = LaunchConfiguration('gate_max_yaw_std')
    gate_max_position_jitter = LaunchConfiguration('gate_max_position_jitter')
    gate_max_yaw_jitter = LaunchConfiguration('gate_max_yaw_jitter')
    enable_ekf = LaunchConfiguration('enable_ekf')
    enable_patrol = LaunchConfiguration('enable_patrol')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # ------------------------- 参数文件 -------------------------
    pkg_share = FindPackageShare(package='fishbot_navigation')
    nav2_params_file = PythonExpression([
        "'", pkg_share, "' + '/config/nav2_params.yaml'"])
    ekf_params_file = PythonExpression([
        "'", pkg_share, "' + '/config/ekf.yaml'"])
    # Humble 系统自带行为树（Foxy 那份自定义树是为补 10ms 超时 bug，Humble 已修复）
    default_bt_xml = PythonExpression([
        "'", FindPackageShare('nav2_bt_navigator'),
        "' + '/behavior_trees/navigate_w_replanning_and_recovery.xml'"])

    # 把地图路径与默认行为树注入 nav2_params
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'yaml_filename': map_yaml_file,
            'default_bt_xml_filename': default_bt_xml,
        },
        convert_types=True,
    )

    # Humble 的 LifecycleNode 强制要求 namespace 关键字，即使空也要显式传 ''
    lifecycle_kwargs = dict(
        namespace='',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    # ------------------------- 0. URDF -> /tf_static（base_link -> laser_frame） -------------------------
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('fishbot_description'), 'urdf', 'fishbot.urdf'])
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    # ------------------------- 1. EKF（替代 FAST-LIO 里程计） -------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_ekf),
    )

    # ------------------------- 2. 定位：map_server + AMCL（先 configure） -------------------------
    map_server = LifecycleNode(
        package='nav2_map_server', executable='map_server',
        name='map_server', **lifecycle_kwargs)
    amcl = LifecycleNode(
        package='nav2_amcl', executable='amcl',
        name='amcl', **lifecycle_kwargs)

    # 定位生命周期管理器：延时后才启动，启动即(autostart)激活 map_server+amcl
    lm_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 15.0,
        }])

    # ------------------------- 3. Nav2 四件套（先 configure） -------------------------
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', **lifecycle_kwargs)
    planner_server = LifecycleNode(
        package='nav2_planner', executable='planner_server',
        name='planner_server', **lifecycle_kwargs)
    controller_server = LifecycleNode(
        package='nav2_controller', executable='controller_server',
        name='controller_server', **lifecycle_kwargs)
    behavior_server = LifecycleNode(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', **lifecycle_kwargs)

    # 导航生命周期管理器：注意它【不直接启动】，
    # 而是等定位门控进程退出后由 OnProcessExit 拉起（此时 autostart 激活四件套）。
    # 激活顺序：bt_navigator 必须最后（它要等 planner/controller 的 action server）。
    lm_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server', 'planner_server',
                'behavior_server', 'bt_navigator'],
            'bond_timeout': 15.0,
        }])

    # ------------------------- 4. 定位门控（硬件无关，沿用 Go2 设计） -------------------------
    localization_gate = Node(
        package='fishbot_navigation', executable='wait_for_localization.py',
        name='navigation_localization_gate', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'stable_duration': gate_stable_duration,
            'pose_max_age': gate_pose_max_age,
            'max_xy_std': gate_max_xy_std,
            'max_yaw_std': gate_max_yaw_std,
            'max_position_jitter': gate_max_position_jitter,
            'max_yaw_jitter': gate_max_yaw_jitter,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
        }],
        condition=IfCondition(enable_localization_gate))

    # 门控进程正常退出（=定位稳定）-> 此时才启动 lm_navigation 激活 Nav2
    start_navigation_when_localized = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_gate,
            on_exit=[lm_navigation],
        ),
        condition=IfCondition(enable_localization_gate),
    )

    # Debug/field mode: skip the localization gate and activate Nav2 after
    # localization nodes have had their normal startup delay.
    start_navigation_without_gate = TimerAction(
        period=localization_start_delay,
        actions=[lm_navigation],
        condition=UnlessCondition(enable_localization_gate),
    )

    # ------------------------- 5. 巡检系统（route/task manager + executor） -------------------------
    # 注意：这里必须指向 fishbot_patrol_v2。旧版本指向 fishbot_patrol_executor 的
    # patrol_system.launch.py，那是 v1 执行器——只要单独起过 patrol_v2，两代就会同时
    # 存在，同一个 /patrol/task/request 出现两个发布者，回执对不上、点位被重复派发。
    patrol_system = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('fishbot_patrol_v2'),
                    'launch', 'patrol_v2.launch.py'))),
    ], condition=IfCondition(enable_patrol))

    # ------------------------- 6. RViz（可选，需 sudo apt install ros-humble-nav2-bringup） -------------------------
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz), output='screen')

    ld = LaunchDescription()
    for arg in declare_args:
        ld.add_action(arg)

    # 顺序：TF -> EKF -> 定位节点(configure) -> 延时激活定位
    #       -> Nav2节点(configure) -> 门控 -> 门控通过后激活导航 -> 巡检/RViz
    ld.add_action(robot_state_publisher)
    ld.add_action(ekf_node)
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(TimerAction(
        period=localization_start_delay,
        actions=[lm_localization]))

    ld.add_action(bt_navigator)
    ld.add_action(planner_server)
    ld.add_action(controller_server)
    ld.add_action(behavior_server)
    # 等待 RViz 设置初始位姿且 AMCL 的 map->base_link 稳定后再激活导航。
    # 固定延迟会让全局 costmap 在 map TF 不存在时堆积 LaserScan，导致 RViz 卡顿。
    ld.add_action(start_navigation_when_localized)
    ld.add_action(localization_gate)
    ld.add_action(start_navigation_without_gate)

    ld.add_action(patrol_system)
    ld.add_action(rviz_node)

    return ld
