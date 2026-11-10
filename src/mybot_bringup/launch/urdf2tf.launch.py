import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # URDF 路径（mybot_description 安装后的 share 目录）
    urdf_tutorial_path = get_package_share_directory('mybot_description')
    mybot_model_path = urdf_tutorial_path + '/urdf/mybot.urdf'

    declare_model = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(mybot_model_path),
        description='URDF 的绝对路径')

    # 读取 URDF 文本作为 robot_description 参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    # 本 URDF 全部是 fixed 关节，robot_state_publisher 会直接发 /tf_static
    # （base_link -> laser_frame），不需要 joint_state_publisher。
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return launch.LaunchDescription([
        declare_model,
        robot_state_publisher_node,
    ])
