#!/usr/bin/env bash
# 建图"两张图拼一起"时，在**另一个终端**跑这个，30 秒内把关键事实打出来。
# 用法： bash check_map_health.sh
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
WS=${MYBOT_WS:-$HOME/fishbot_full_ws}
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" >/dev/null 2>&1

echo "==================== 1. 谁在发 odom→base_link ===================="
timeout 12 ros2 run tf2_ros tf2_monitor base_link odom 2>/dev/null \
  | grep -E "Broadcasters|Average rate|Buffer length" | head -5 \
  || echo "  (tf2_monitor 没输出，可能 TF 链断了)"

echo
echo "==================== 2. 关键节点的数量（都应该是 1）===================="
for n in async_slam_toolbox_node odom_shim ekf_filter_node scan_gate.py; do
  echo "  $n: $(timeout 8 ros2 node list 2>/dev/null | grep -c "/$n\$")"
done

echo
echo "==================== 3. 频率 ===================="
for t in /scan /map; do
  printf "  %-8s " "$t"
  timeout 8 ros2 topic hz "$t" 2>&1 | grep -m1 "average rate" || echo "(没有数据)"
done

echo
echo "==================== 4. 开关状态 ===================="
printf "  ekf publish_tf = "
timeout 6 ros2 param get /ekf_filter_node publish_tf 2>/dev/null | sed 's/^Boolean value is: //' || echo "?"
printf "  shim state     = "
timeout 6 ros2 topic echo /odom_shim/state --once 2>/dev/null | head -c 300 || echo "?"
echo
printf "  explore status = "
timeout 6 ros2 topic echo /explore/status --once 2>/dev/null | grep -o '"scan_gate":"[a-z]*"' || echo "?"

echo
echo "==================== 5. 激光是不是在丢 ===================="
echo -n "  最近 200 条 /scan 间隔(应 ~0.1s): "
timeout 6 ros2 topic echo /scan --field header.stamp.sec --once 2>/dev/null >/dev/null && echo "有数据" || echo "没数据"

echo
echo "==================== 6. 当前地图尺寸 ===================="
timeout 8 ros2 topic echo /map --field info.width --once 2>/dev/null | head -1
timeout 8 ros2 topic echo /map --field info.height --once 2>/dev/null | head -1

echo
echo "把这段输出整段发我。"
