#!/bin/bash
# run_a0_real.sh —— A0 现场一键脚本（真机）
#
# 它做四件事：
#   1. 体检：/scan 有没有数据、odom->base_link 的 TF 在不在（不在就直接退出，别浪费时间）
#   2. 起 slam_toolbox（自己建图、自己发 map->odom）
#   3. 起 nav2 的「纯导航」部分（不含 amcl、不含 map_server），验证两者能共存
#   4. 等 N 秒 → 存地图快照 → 收诊断包 → 把所有起的进程按进程组关干净
#
# 用法：
#   bash tools/run_a0_real.sh                    # 跑 5 分钟
#   bash tools/run_a0_real.sh --duration 180     # 跑 3 分钟
#   bash tools/run_a0_real.sh --no-nav           # 只测 slam，不起 nav2
#   bash tools/run_a0_real.sh --base-frame base_footprint   # 底盘帧不叫 base_link 时
#
# 前置（你自己先起好，脚本不管）：
#   - 雷达在发 /scan
#   - micro-ROS agent 在跑（odom->base_link 的 TF 有）
#   - 不要在别的终端再起一套 nav2/amcl，否则会抢 map->odom
#
# 跑完把这几样带回来：
#   /tmp/a0real/map.png          地图快照
#   /tmp/a0real/slam.log         slam 日志
#   /tmp/diag_*.tar.gz           诊断包（脚本会自动调 collect_diag.sh 生成）

set +u

DURATION=300
BASE_FRAME=base_link
WITH_NAV=1
OUT="/tmp/a0real"
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2 ;;
    --base-frame) BASE_FRAME="$2"; shift 2 ;;
    --no-nav)     WITH_NAV=0; shift ;;
    --out)        OUT="$2"; shift 2 ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数：$1"; exit 1 ;;
  esac
done

mkdir -p "$OUT"
LOG="$OUT"

source /opt/ros/humble/setup.bash
for ws in "$HOME/fishbot_full_ws" "$HOME/mybot_full_ws"; do
  [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash" && break
done

PIDS=""
cleanup() {
  echo
  echo "---- 收尾：关掉本次起的进程 ----"
  for p in $PIDS; do kill -INT -"$p" 2>/dev/null; done
  sleep 5
  for p in $PIDS; do kill -9 -"$p" 2>/dev/null; done
  pkill -9 -f "async_slam_toolbox_node" 2>/dev/null
  pkill -9 -f "navigation_launch.py" 2>/dev/null
  sleep 1
  LEFT=$(pgrep -fc "async_slam_toolbox_node|navigation_launch.py" 2>/dev/null || true)
  echo "剩余相关进程：${LEFT:-0}"
  echo "产物在：$OUT"
}
trap cleanup EXIT

# ---------- 1. 体检 ----------
echo "== 1/4 体检 =="
SCAN_OK=$(timeout 6 ros2 topic hz /scan 2>&1 | grep -c "average rate" || true)
if [ "${SCAN_OK:-0}" -eq 0 ]; then
  echo "❌ /scan 上没有数据。"
  echo "   先确认：雷达上电了吗？micro-ROS agent 起了吗？"
  echo "   排查：ros2 topic hz /scan   /   ros2 topic echo /scan --once"
  exit 2
fi
echo "✅ /scan 有数据"

TF_OK=$(timeout 8 python3 - "$BASE_FRAME" <<'PY' 2>/dev/null
import sys, time, rclpy, tf2_ros
from rclpy.node import Node
base = sys.argv[1]
rclpy.init(); n = Node('a0_tf_check'); buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)
t0 = time.time(); ok = False
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.3)
    try:
        buf.lookup_transform('odom', base, rclpy.time.Time()); ok = True; break
    except Exception:
        pass
print('ok' if ok else 'no')
n.destroy_node(); rclpy.shutdown()
PY
)
if [ "$TF_OK" != "ok" ]; then
  echo "❌ 找不到 odom -> $BASE_FRAME 的 TF。"
  echo "   如果底盘帧叫 base_footprint，加参数：--base-frame base_footprint"
  echo "   排查：ros2 run tf2_tools view_frames  /  ros2 topic echo /tf --once"
  exit 3
fi
echo "✅ odom -> $BASE_FRAME 的 TF 正常"

LEFT_NAV=$(timeout 15 ros2 node list 2>/dev/null | grep -cE "/(amcl|map_server)$" || true)
if [ "${LEFT_NAV:-0}" -gt 0 ]; then
  echo "⚠️  检测到已在运行的 amcl / map_server —— 它们会和 slam_toolbox 抢 map->odom。"
  echo "   请先在别的终端停掉那套导航，再跑本脚本。"
  read -r -p "   仍然继续？(y/N) " ans
  [ "$ans" = "y" ] || exit 4
fi

# ---------- 2. slam_toolbox ----------
echo
echo "== 2/4 起 slam_toolbox（base_frame=$BASE_FRAME）=="
setsid ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=false \
  -p map_frame:=map -p odom_frame:=odom -p base_frame:="$BASE_FRAME" \
  -p scan_topic:=/scan -p mode:=mapping \
  -p resolution:=0.05 -p transform_timeout:=0.5 -p minimum_time_interval:=0.05 \
  > "$LOG/slam.log" 2>&1 &
PIDS="$PIDS $!"
sleep 10
if ! pgrep -f async_slam_toolbox_node > /dev/null; then
  echo "❌ slam_toolbox 没起来，日志："; tail -20 "$LOG/slam.log"; exit 5
fi
echo "✅ slam 在跑"

# ---------- 3. nav2 ----------
if [ "$WITH_NAV" -eq 1 ]; then
  echo
  echo "== 3/4 起 nav2 纯导航部分（不含 amcl/map_server）=="
  setsid ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
    > "$LOG/nav2.log" 2>&1 &
  PIDS="$PIDS $!"
  sleep 20
  echo "✅ nav2 已起（如果报错看 $LOG/nav2.log）"
else
  echo
  echo "== 3/4 跳过 nav2（--no-nav）=="
fi

# ---------- 4. 跑一段 + 存图 ----------
echo
echo "== 4/4 观察 ${DURATION} 秒 =="
echo "   现在可以让车慢慢走一圈（或者原地转两圈），让地图长起来。"
echo "   剩 30 秒时我会自动存图。"
sleep $((DURATION > 30 ? DURATION - 30 : DURATION))

timeout 30 python3 - "$OUT" <<'PY' 2>&1
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from PIL import Image

out = sys.argv[1]
rclpy.init(); n = Node('a0_dump'); box = {}
qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = ReliabilityPolicy.RELIABLE
n.create_subscription(OccupancyGrid, '/map', lambda m: box.setdefault('m', m), qos)
t0 = time.time()
while 'm' not in box and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.5)
if 'm' not in box:
    print('❌ 没收到 /map：slam 可能没能建图（看 slam.log）')
else:
    m = box['m']; w, h = m.info.width, m.info.height
    img = Image.new('L', (w, h), 128); px = img.load()
    for j in range(h):
        row = j * w
        for i in range(w):
            v = m.data[row + i]
            px[i, h - 1 - j] = 128 if v < 0 else (0 if v >= 50 else 254)
    img = img.resize((w * 3, h * 3), Image.NEAREST)
    img.save(out + '/map.png')
    unknown = sum(1 for v in m.data if v < 0)
    print(f'✅ 地图 {w}x{h} @ {m.info.resolution} m/格，未知 {unknown} 格 → {out}/map.png')
    print('   未知格很多 = 正常（还没走到的地方）；未知格是 0 = 值得怀疑')
n.destroy_node(); rclpy.shutdown()
PY

echo
echo "---- 顺便收个诊断包 ----"
bash "$(dirname "$0")/collect_diag.sh" "$OUT" 2>&1 | tail -2

echo
echo "=== 完成 ==="
echo "带回来："
echo "  $OUT/map.png        地图快照"
echo "  $OUT/slam.log        slam 日志"
echo "  $OUT/diag_*.tar.gz   诊断包"
