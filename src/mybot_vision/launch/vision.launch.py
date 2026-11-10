"""启动 mybot 视觉系统：摄像头取流 + YOLO 目标检测。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mybot_vision')
    params_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')
    # 模型用 install 后的绝对路径，避免依赖启动时的工作目录（相对路径易找不到）。
    # 优先用 TensorRT engine（FP16，Orin 上实测比 CUDA 的 .pt 更快、更省 CPU），
    # 文件不存在就自动退回 .pt —— 所以还没导出 engine 的机器（比如虚拟机）也能照常跑。
    # 注意：engine 与 JetPack/TensorRT 版本绑定，升级系统或换机器后要重新导出。
    engine_path = os.path.join(pkg_share, 'yolo26n.engine')
    pt_path = os.path.join(pkg_share, 'yolo26n.pt')
    model_path = engine_path if os.path.exists(engine_path) else pt_path

    # 摄像头取流：namespace=/camera，name=camera
    # 图像话题 = /camera/camera/image_raw
    camera_node = Node(
        package='mybot_vision',
        executable='camera_publisher',
        name='camera',
        namespace='camera',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file],
    )

    # YOLO 检测：name=yolo_detector，订阅上面的图像话题
    # 带框图像话题 = /yolo_detector/image_annotated
    yolo_node = Node(
        package='mybot_vision',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'model': model_path}],
    )

    return LaunchDescription([
        camera_node,
        yolo_node,
    ])
