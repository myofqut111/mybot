#!/bin/bash
# 干净重部署控制台。
# 三条铁律（都是踩出来的）：
#   1) 先 TERM 再 KILL。launch 被 -9 打死时来不及回收子进程，rosbridge/rosapi
#      会变成 PPID=1 的孤儿，一直占着 9090，下一次启动就 "Address already in use"。
#   2) 必须用非 symlink 构建。develop 模式下 fishbot_console.egg-info 只在 build/ 里，
#      入口脚本 robot_console 靠 importlib.metadata 查 'fishbot-console'，
#      查不到就 PackageNotFoundError（launch 还配了 respawn，会一直刷屏重试）。
#   3) 元数据要在中性 cwd 下验证。站在 src 里查会自欺欺人——那里本来就有 egg-info。
set +u
source /opt/ros/humble/setup.bash
source /home/my/fishbot_full_ws/install/setup.bash

echo "== 1. stop old =="
pkill -f "console.launch.py" 2>/dev/null
pkill -f "fishbot_console/lib/fishbot_console" 2>/dev/null
sleep 2
# launch 已死但子进程可能被 init 收养，单独收一遍
pkill -f "rosbridge_server/rosbridge_websocket" 2>/dev/null
pkill -f "rosapi/rosapi_node" 2>/dev/null
sleep 1
pkill -9 -f "console.launch.py" 2>/dev/null
pkill -9 -f "fishbot_console/lib/fishbot_console" 2>/dev/null
pkill -9 -f "rosbridge_server/rosbridge_websocket" 2>/dev/null
pkill -9 -f "rosapi/rosapi_node" 2>/dev/null
sleep 1
left=$(pgrep -fc "rosbridge_websocket|rosapi_node" 2>/dev/null || echo 0)
echo "leftover rosbridge/rosapi: $left"
ss -ltn 2>/dev/null | grep -E ":8080|:8081|:9090" || echo "8080/8081/9090 all free"

echo "== 2. build =="
cd /home/my/fishbot_full_ws
colcon build --packages-select fishbot_console > /tmp/build_console.log 2>&1
echo "build exit=$?"

# 兜底：把 egg-info 放进 install 的 site-packages，任何 cwd / 任何 shell 都查得到。
# 必须"先删再拷"：只补缺失会让上一版的 egg-info 留着，升版本后
# importlib.metadata 报告的仍是旧版本号（0.7.1 装成 0.7.0 就是这么来的）。
SP=/home/my/fishbot_full_ws/install/fishbot_console/lib/python3.10/site-packages
EI=/home/my/fishbot_full_ws/build/fishbot_console/fishbot_console.egg-info
if [ -d "$EI" ]; then
  rm -rf "$SP"/*.egg-info
  cp -r "$EI" "$SP/fishbot_console.egg-info"
  echo "egg-info refreshed -> $(grep -m1 '^Version:' "$EI/PKG-INFO" | cut -d' ' -f2)"
fi

# 中性 cwd 验证元数据
cd /tmp
if ! python3 -c "import importlib.metadata as m; print('version', m.version('fishbot-console'))"; then
  echo "!! 元数据查不到，入口脚本必然 PackageNotFoundError，停止部署"
  exit 1
fi

echo "== 3. start =="
nohup ros2 launch fishbot_console console.launch.py > /tmp/console070.log 2>&1 &
sleep 9

echo "== 4. verify =="
grep -E "listening on|被占用" /tmp/console070.log | tail -2
curl -s -o /dev/null -w "http8080=%{http_code}\n" http://127.0.0.1:8080/
# 这一步只是"锦上添花"的自检：DDS 发现慢的时候 `ros2 param get` 会一直
# 挂着（实测挂过 13 分钟，把整个部署卡死），所以必须限时，超时就跳过。
timeout 10 ros2 param get /fishbot_console prefer_compressed 2>/dev/null \
  || echo "param get 超时（DDS 发现慢，面板本身已起来，不影响使用）"
echo "port-conflicts in log: $(grep -c 'Address already in use' /tmp/console070.log)"
echo "metadata errors in log: $(grep -c 'PackageNotFoundError' /tmp/console070.log)"
