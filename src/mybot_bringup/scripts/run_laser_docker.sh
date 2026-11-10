#!/usr/bin/env bash
# ============================================================
# 一键启动鱼香网络激光雷达 docker 容器（host 网络模式）
# 用 --network host：容器和主机共享网络栈，主机 ROS2 才能直接
# 看到容器内 ydlidar 节点发布的 /scan（bridge 模式 DDS 互相发现不到）。
# 启动后若提示输入网络端口，直接回车用默认 8889。
# ============================================================
set -e

# 只停止旧的 mybot_laser 雷达容器；绝不能 docker stop 全部容器，
# 否则会把正在跑的 micro-ros-agent（管 /odom /imu）一起杀掉。两者要并存。
LASER_IMG=registry.cn-hangzhou.aliyuncs.com/fishros/mybot_laser
OLD=$(sudo docker ps -q --filter ancestor="$LASER_IMG")
if [ -n "$OLD" ]; then
  echo "[run_laser] 发现旧雷达容器，先停止（不会动 micro-ros-agent）..."
  sudo docker stop $OLD || true
fi

# 允许容器复用主机 X 显示（雷达可视化窗口）
xhost + >/dev/null 2>&1 || true

sudo docker run -it --rm --network host --ipc host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/snd \
  -e DISPLAY="unix${DISPLAY}" \
  registry.cn-hangzhou.aliyuncs.com/fishros/mybot_laser
