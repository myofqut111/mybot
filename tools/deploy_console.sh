#!/bin/bash
# 干净重部署控制台：先把旧进程全杀干净（含 launch 遗留的 robot_console），
# 再重编译、重启、验证端口与订阅。只同步改动的 .py，不动 src 里的 egg-info。
set +u
source /opt/ros/humble/setup.bash
source /home/my/fishbot_full_ws/install/setup.bash

echo "== 1. stop old =="
pkill -f "console.launch.py" 2>/dev/null
sleep 1
pkill -f "fishbot_console/lib/fishbot_console" 2>/dev/null
sleep 1
pkill -9 -f "console.launch.py" 2>/dev/null
pkill -9 -f "fishbot_console/lib/fishbot_console" 2>/dev/null
sleep 1
ss -ltn 2>/dev/null | grep -E ":8080|:8081" || echo "8080/8081 both free"

echo "== 2. build =="
cd /home/my/fishbot_full_ws
colcon build --packages-select fishbot_console --symlink-install > /tmp/build_console.log 2>&1
echo "build exit=$?"
python3 -c "import importlib.metadata as m; print('version', m.version('fishbot-console'))"

echo "== 3. start =="
nohup ros2 launch fishbot_console console.launch.py > /tmp/console070.log 2>&1 &
sleep 9

echo "== 4. verify =="
grep -E "listening on|被占用" /tmp/console070.log | tail -2
curl -s -o /dev/null -w "http8080=%{http_code}\n" http://127.0.0.1:8080/
ros2 param get /fishbot_console prefer_compressed 2>/dev/null
