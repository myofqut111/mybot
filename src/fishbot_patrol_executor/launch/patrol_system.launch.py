"""Start the complete fishbot patrol stack: route + task manager + executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route_file = LaunchConfiguration('route_file')
    task_timeout = LaunchConfiguration('task_timeout_s')
    waypoint_timeout = LaunchConfiguration('waypoint_timeout_s')

    route_manager = Node(
        package='fishbot_patrol_route_manager',
        executable='route_manager',
        name='patrol_route_manager',
        output='screen',
        parameters=[{'route_file': route_file}],
    )

    task_manager = Node(
        package='fishbot_patrol_task_manager',
        executable='patrol_task_manager',
        name='patrol_task_manager',
        output='screen',
        parameters=[{'task_timeout_s': task_timeout}],
    )

    executor = Node(
        package='fishbot_patrol_executor',
        executable='patrol_executor',
        name='patrol_executor',
        output='screen',
        parameters=[{
            'task_result_timeout_s': task_timeout,
            'waypoint_timeout_s': waypoint_timeout,
            'endpoint_max_xy_std_m': 0.30,
            'endpoint_max_yaw_std_rad': 0.30,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
            default_value='~/.ros/fishbot_patrol_routes.json',
            description='Persistent patrol route JSON file.'),
        DeclareLaunchArgument(
            'task_timeout_s',
            default_value='120.0',
            description='Maximum time to wait for one point task result.'),
        DeclareLaunchArgument(
            'waypoint_timeout_s',
            default_value='180.0',
            description='Maximum navigation time for one patrol point.'),
        route_manager,
        task_manager,
        executor,
    ])
