#!/usr/bin/env bash
# 用 mock 机器人跑一遍自主探索：看它是否真的"看 → 走 → 再算 → 收工"。
#
# 用法（在跑 ROS 的那台机器上执行，不要 sudo）：
#     bash tools/run_mock_explore.sh [秒数] [地图 yaml]
#
#     bash tools/run_mock_explore.sh              # 跑 150 秒
#     bash tools/run_mock_explore.sh 60           # 只跑 60 秒
#     bash tools/run_mock_explore.sh 150 /path/to/other.yaml
#
# 需要：mock_robot（mybot_console 里自带）能起；地图带"未知区"（灰），
# 否则前沿检测永远 0 个候选 —— 用 tools/make_partial_map.py 造一张。
#
# 收尾会自动杀干净（只杀自己起的两个进程），日志在 /tmp/mock_explore_*.log
set -u

DURATION=${1:-150}
MAP_YAML=${2:-$HOME/maps/mock_partial.yaml}

# 工作区：$MYBOT_WS > ~/mybot_full_ws > ~/fishbot_full_ws
WS=${MYBOT_WS:-}
for cand in "$HOME/mybot_full_ws" "$HOME/fishbot_full_ws"; do
  [ -n "$WS" ] && break
  [ -f "$cand/install/setup.bash" ] && WS=$cand
done
if [ -z "$WS" ]; then
  echo "找不到工作区（~/mybot_full_ws 或 ~/fishbot_full_ws），先 colcon build"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" 2>/dev/null || true
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# 仓库名/机器上的包名可能不同（mybot_* ↔ fishbot_*），自动判断
if ros2 pkg prefix mybot_console >/dev/null 2>&1; then
  CONSOLE_PKG=mybot_console
  EXPLORE_PKG=mybot_explore
else
  CONSOLE_PKG=fishbot_console
  EXPLORE_PKG=fishbot_explore
fi

LOG=/tmp/mock_explore_$(date +%H%M%S).log

cleanup() {
  echo "--- 收尾 ---" | tee -a "$LOG"
  pkill -INT -f "[m]ock_robot" 2>/dev/null || true
  pkill -INT -f "[e]xplore_node" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "[m]ock_robot" 2>/dev/null || true
  pkill -KILL -f "[e]xplore_node" 2>/dev/null || true
  sleep 1
  echo "mock_robot 残留: $(pgrep -fc '[m]ock_robot' || true)" | tee -a "$LOG"
  echo "explore_node 残留: $(pgrep -fc '[e]xplore_node' || true)" | tee -a "$LOG"
  echo "日志: $LOG"
}
trap cleanup EXIT INT TERM

echo "=== 0. 清场 ===" | tee "$LOG"
pgrep -af "[m]ock_robot|[e]xplore_node" | tee -a "$LOG" || true
pkill -KILL -f "[m]ock_robot" 2>/dev/null || true
pkill -KILL -f "[e]xplore_node" 2>/dev/null || true
sleep 1

echo "=== 1. 地图 ===" | tee -a "$LOG"
if [ ! -f "$MAP_YAML" ]; then
  echo "找不到地图：$MAP_YAML" | tee -a "$LOG"
  echo "用 python3 src/mybot_explore/tools/make_partial_map.py <输出前缀> 造一张带未知区的地图" | tee -a "$LOG"
  exit 1
fi
ls -l "$MAP_YAML" | tee -a "$LOG"
head -4 "$MAP_YAML" | tee -a "$LOG"

echo "=== 2. 起 mock_robot ===" | tee -a "$LOG"
setsid nohup ros2 run "$CONSOLE_PKG" mock_robot > /tmp/mock_robot.out 2>&1 &
sleep 8
echo "mock 进程: $(pgrep -fc '[m]ock_robot' || true)" | tee -a "$LOG"
tail -3 /tmp/mock_robot.out | tee -a "$LOG"

echo "=== 3. 起 explore_node（drive:=true，真发目标）===" | tee -a "$LOG"
setsid nohup ros2 run "$EXPLORE_PKG" explore_node --ros-args \
  -p map_yaml:="$MAP_YAML" -p drive:=true -p autostart:=true \
  > /tmp/explore_node.out 2>&1 &
sleep 5
echo "explore 进程: $(pgrep -fc '[e]xplore_node' || true)" | tee -a "$LOG"

echo "=== 4. 观察 ${DURATION}s ===" | tee -a "$LOG"
for i in $(seq 1 $((DURATION / 10))); do
  sleep 10
  if ! pgrep -f "[e]xplore_node" > /dev/null; then
    echo "[$((i * 10))s] explore_node 已退出" | tee -a "$LOG"
    break
  fi
  st=$(timeout 5 ros2 topic echo --once /explore/status 2>/dev/null | head -20 | tr '\n' ' ')
  echo "[$((i * 10))s] $st" | tee -a "$LOG"
done

echo "=== 5. 探索节点日志 ===" | tee -a "$LOG"
grep -E "前沿|去 \(|导航结束|拉黑|回家|收工" /tmp/explore_node.out | tail -25 | tee -a "$LOG"

echo "=== 6. mock 日志 ===" | tee -a "$LOG"
grep -E "mock nav" /tmp/mock_robot.out | tail -12 | tee -a "$LOG"
