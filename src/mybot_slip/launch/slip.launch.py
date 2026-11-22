"""打滑检测节点启动。

只判不动：ros2 launch fishbot_slip slip.launch.py
带恢复：  ros2 launch fishbot_slip slip.launch.py recovery:=true
干跑：    ros2 launch fishbot_slip slip.launch.py recovery:=true executor_dry_run:=true
          （执行器只打日志不发 /cmd_vel，用来先看动作序列对不对）
带仲裁：  ros2 launch fishbot_slip slip.launch.py recovery:=true with_mux:=true
          导航改发 /cmd_vel_nav，恢复动作发 /cmd_vel_slip，
          cmd_mux 按优先级并成 /cmd_vel（恢复刚发过的 0.5 秒内只听恢复的）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    recovery = LaunchConfiguration('recovery').perform(context)
    dry = LaunchConfiguration('executor_dry_run').perform(context)
    mux = LaunchConfiguration('with_mux').perform(context).lower() in ('1', 'true', 'yes')
    scan = LaunchConfiguration('scan_topic').perform(context)
    cmd = LaunchConfiguration('cmd_topic').perform(context)
    nav_in = LaunchConfiguration('nav_topic').perform(context)

    # 开了仲裁：执行器不能直接发 /cmd_vel，改发 /cmd_vel_slip 交给仲裁节点
    executor_cmd = '/cmd_vel_slip' if mux else cmd

    nodes = [
        Node(
            package='fishbot_slip',
            executable='slip_node',
            name='slip_node',
            output='screen',
            parameters=[{
                'scan_topic': scan,
                'recovery_enabled': recovery,
                'alternate_after_fail': LaunchConfiguration('alternate_after_fail'),
            }],
        ),
        Node(
            package='fishbot_slip',
            executable='slip_executor',
            name='slip_executor',
            output='screen',
            parameters=[{
                'cmd_topic': executor_cmd,
                'dry_run': dry,
            }],
        ),
    ]
    if mux:
        nodes.append(Node(
            package='fishbot_slip',
            executable='cmd_mux',
            name='cmd_mux',
            output='screen',
            parameters=[{
                'nav_topic': nav_in,
                'slip_topic': '/cmd_vel_slip',
                'out_topic': cmd,
            }],
        ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'recovery', default_value='false',
            description='是否让 slip_executor 自己动起来（默认只判不动）'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan_filtered',
            description='用哪一路激光（没有 scan_gate 时传 /scan）'),
        DeclareLaunchArgument(
            'cmd_topic', default_value='/cmd_vel',
            description='最终速度发到哪里'),
        DeclareLaunchArgument(
            'nav_topic', default_value='/cmd_vel_nav',
            description='开了仲裁后，导航/手动控制改发这个'),
        DeclareLaunchArgument(
            'executor_dry_run', default_value='false',
            description='执行器不发速度，只打日志'),
        DeclareLaunchArgument(
            'with_mux', default_value='false',
            description='是否起速度仲裁（恢复优先）'),
        DeclareLaunchArgument(
            'alternate_after_fail', default_value='true',
            description='恢复失败后是否换姿势（后退↔转向）'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
