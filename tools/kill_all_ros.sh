#!/usr/bin/env bash
# 一键停干净：把所有 ROS 相关的残留进程、端口、共享内存清掉。
#
# 用法：bash tools/kill_all_ros.sh          # 只杀，不重启
#       bash tools/kill_all_ros.sh --dry    # 只看会杀谁
#
# 为什么要这个：Ctrl+C 只杀到 launch，launch 死了子进程会被 systemd 收养变成孤儿；
# 反复几次就会看到 ros2 node list 里出现 2 份、3 份同名节点（amcl/planner_server…），
# 它们同时发 TF 和 /cmd_vel，车就会乱走。
set -u

DRY=0
[ "${1:-}" = "--dry" ] && DRY=1

# 只匹配可执行文件路径，避免误杀执行本脚本的 shell（命令行里含这些字样）
PATTERNS=(
  "ros2 launch"
  "/lib/nav2_"
  "/lib/nav2_amcl/amcl"
  "lifecycle_manager"
  "/lib/fishbot_"
  "/lib/mybot_"
  "fishbot_laser_driver_node"
  "mybot_laser_driver_node"
  "rosbridge_websocket|rosapi_node|robot_console"
  "wait_for_localization"
  "explore_node|mock_robot"
)

hits() {
  for p in "${PATTERNS[@]}"; do
    pgrep -af -- "$p" 2>/dev/null | grep -v "kill_all_ros.sh" || true
  done | sort -u
}

echo "=== 会命中的进程 ==="
LIST=$(hits)
if [ -z "$LIST" ]; then
  echo "（没有）"
  exit 0
fi
echo "$LIST" | awk '{printf "  %-7s %s\n", $1, substr($0, index($0,$2))}'
echo "共 $(echo "$LIST" | wc -l) 个"

if [ "$DRY" = "1" ]; then
  echo "（--dry，不动手）"
  exit 0
fi

echo "=== 先 TERM，等 2 秒，再 KILL ==="
echo "$LIST" | awk '{print $1}' | xargs -r kill -TERM 2>/dev/null || true
sleep 2
echo "$LIST" | awk '{print $1}' | xargs -r kill -KILL 2>/dev/null || true
sleep 1

echo "=== 残留 ==="
echo "$(hits | wc -l) 个"

echo "=== 端口 ==="
for port in 8080 8081 8083 9090 9091; do
  if ss -ltn 2>/dev/null | grep -q ":$port "; then echo "  $port 仍被占用"; else echo "  $port 空闲"; fi
done

echo "=== 共享内存（fastrtps 段）==="
BEFORE=$(ls /dev/shm 2>/dev/null | grep -c fastrtps || true)
echo "  清理前 $BEFORE 段"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
AFTER=$(ls /dev/shm 2>/dev/null | grep -c fastrtps || true)
echo "  清理后 $AFTER 段"
