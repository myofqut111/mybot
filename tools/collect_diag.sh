#!/bin/bash
# collect_diag.sh —— 一条命令把"现场发生了什么"打包成一个几十 KB 的压缩包。
#
# 用法：
#   bash tools/collect_diag.sh              # 打包到 /tmp/diag_<时间>.tar.gz
#   bash tools/collect_diag.sh /home/my/x   # 打包到指定目录
#
# 特点：
#   - 只读，不改任何东西，不起任何进程
#   - 每一步都有 timeout，某个话题没在发也不会卡住
#   - 没网也能跑（不访问外网）
#
# 把生成的 tar.gz 带回有网的机器上，交给开发看就行。

set +u
OUT_DIR="${1:-/tmp}"
STAMP="$(date +%Y%m%d_%H%M%S)"
WORK="/tmp/diag_${STAMP}"
mkdir -p "$WORK"
TARBALL="${OUT_DIR%/}/diag_${STAMP}.tar.gz"

# 有 ROS 环境就 source，没有也继续（照样能收系统信息）
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
for ws in "$HOME/fishbot_full_ws" "$HOME/mybot_full_ws"; do
  [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash" && break
done

say() { echo "[collect_diag] $*"; }

# ---------- 1. 基本环境 ----------
say "1/9 环境信息"
{
  echo "== host =="; hostname; date; uptime
  echo; echo "== 网卡 =="; ip -brief addr 2>/dev/null
  echo; echo "== 路由 =="; ip route 2>/dev/null
  echo; echo "== 内存/磁盘 =="; free -h; df -h / /home 2>/dev/null
  echo; echo "== ROS 环境变量 =="
  for v in ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE MYBOT_NOTIFY_URL; do
    echo "$v=${!v:-<未设置>}"
  done
  echo; echo "== 关键环境文件 =="
  ls -l ~/no_shm.xml ~/.dds_unicast/ 2>/dev/null
  echo; echo "== ~/.ros 下的状态文件 =="
  ls -l ~/.ros/*.json 2>/dev/null
} > "$WORK/01_env.txt" 2>&1

# ---------- 2. 版本 ----------
say "2/9 包版本"
{
  echo "== 已安装的 fishbot_/mybot_ 包 =="
  timeout 20 ros2 pkg list 2>/dev/null | grep -E "fishbot|mybot" | sort
  echo; echo "== 版本号（以 python 元数据为准）=="
  timeout 20 python3 - <<'PY' 2>&1
import importlib.metadata as m
for name in ('fishbot-console','mybot-console','fishbot-patrol-v2','mybot-patrol-v2',
             'fishbot-safety','mybot-safety','fishbot-vision','mybot-vision'):
    try:
        print(f'{name:22s} {m.version(name)}')
    except Exception as e:
        print(f'{name:22s} <查不到: {type(e).__name__}>')
PY
} > "$WORK/02_versions.txt" 2>&1

# ---------- 3. 进程 ----------
say "3/9 进程与端口"
{
  echo "== ROS 相关进程 =="
  ps -eo pid,ppid,etime,cmd 2>/dev/null | grep -E "ros2|nav2|slam|amcl|map_server|controller_server|planner_server|behavior_server|bt_navigator|route_manager|patrol_|robot_console|rosbridge|rosapi|micro_ros|mock_" | grep -v grep
  echo; echo "== 监听端口 =="
  ss -ltnp 2>/dev/null | grep -E "8080|8081|9090|9091|8888" || echo "（8080/8081/9090/9091/8888 都没人监听）"
  echo; echo "== /dev/shm 里的 fastrtps 残留 =="
  echo "fastrtps 段: $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"
  ls /dev/shm 2>/dev/null | grep sem.fastrtps | head -5
} > "$WORK/03_processes.txt" 2>&1

# ---------- 4. 节点 / 话题 ----------
say "4/9 节点与话题"
{
  echo "== 节点（20s 超时；空说明 DDS 没发现到东西）=="
  timeout 20 ros2 node list 2>&1 | sort
  echo; echo "== 话题类型 =="
  timeout 25 ros2 topic list -t 2>&1 | sort
} > "$WORK/04_graph.txt" 2>&1

# ---------- 5. TF 树 ----------
say "5/9 TF 树"
{
  echo "== /tf 与 /tf_static 发布者 =="
  timeout 15 ros2 topic info /tf 2>&1
  echo; timeout 15 ros2 topic info /tf_static 2>&1
  echo; echo "== map->odom 谁在发（广播者列表）=="
  timeout 15 ros2 run tf2_ros tf2_monitor map odom 2>&1 | sed -n '/Broadcasters/,/^$/p' | head -12
  echo; echo "== 帧名清单 =="
  timeout 25 python3 - <<'PY' 2>&1
import time
import rclpy
from rclpy.node import Node
import tf2_ros

rclpy.init()
n = Node('diag_tf')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)
t0 = time.time()
frames = set()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.5)
    try:
        frames |= set(buf.all_frames_as_yaml().split('\n'))
    except Exception:
        pass
print('\n'.join(sorted(f for f in frames if f.strip())) or '（没读到任何帧）')
n.destroy_node(); rclpy.shutdown()
PY
} > "$WORK/05_tf.txt" 2>&1

# ---------- 6. 关键话题频率 ----------
say "6/9 话题频率（每个 3 秒）"
{
  for t in /scan /odom /map /cmd_vel /battery/voltage /tf; do
    echo "== $t =="
    timeout 6 ros2 topic hz "$t" 2>&1 | grep -E "average rate|no new messages" | head -2
    echo
  done
} > "$WORK/06_hz.txt" 2>&1

# ---------- 7. 地图 ----------
say "7/9 地图信息"
{
  timeout 25 python3 - <<'PY' 2>&1
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
n = Node('diag_map')
box = {}
qos = QoSProfile(depth=1)
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = ReliabilityPolicy.RELIABLE
n.create_subscription(OccupancyGrid, '/map', lambda m: box.setdefault('m', m), qos)
t0 = time.time()
while 'm' not in box and time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.5)
if 'm' not in box:
    print('没收到 /map（没人发地图，或者 QoS 不匹配）')
else:
    m = box['m']
    d = m.data
    print('frame_id :', m.header.frame_id)
    print('尺寸     :', m.info.width, 'x', m.info.height, '@', m.info.resolution, 'm/格')
    print('原点     :', m.info.origin.position.x, m.info.origin.position.y)
    print('未知格   :', sum(1 for v in d if v < 0))
    print('空闲格   :', sum(1 for v in d if 0 <= v < 50))
    print('障碍格   :', sum(1 for v in d if v >= 50))
n.destroy_node(); rclpy.shutdown()
PY
} > "$WORK/07_map.txt" 2>&1

# ---------- 8. 最近的巡检报告 ----------
say "8/9 最近三份巡检报告"
{
  for d in "$HOME/patrol_reports" "$HOME/.ros/patrol_reports"; do
    [ -d "$d" ] || continue
    echo "== $d =="
    ls -t "$d" 2>/dev/null | head -6
    for f in $(ls -t "$d"/*.jsonl 2>/dev/null | head -3); do
      echo; echo "---- $f ----"
      head -c 4000 "$f"
      echo
      s="${f%.jsonl}.summary.txt"
      [ -f "$s" ] && { echo "---- $(basename "$s") ----"; cat "$s"; echo; }
    done
  done
  echo; echo "== 断点/基线 =="
  for f in ~/.ros/fishbot_patrol_progress.json ~/.ros/mybot_patrol_progress.json \
           ~/.ros/fishbot_patrol_expect.json ~/.ros/mybot_patrol_expect.json; do
    [ -f "$f" ] && { echo "---- $f ----"; head -c 1500 "$f"; echo; }
  done
} > "$WORK/08_reports.txt" 2>&1

# ---------- 9. 日志 ----------
say "9/9 最近日志"
{
  LATEST="$(ls -dt ~/.ros/log/* 2>/dev/null | head -1)"
  echo "最近一次 launch 目录: ${LATEST:-无}"
  [ -n "$LATEST" ] && ls -l "$LATEST" 2>/dev/null | head -20
  for f in $(ls -t ~/.ros/log/*.log 2>/dev/null | head -3); do
    echo; echo "---- $(basename "$f")（最后 60 行）----"
    tail -60 "$f"
  done
  echo; echo "== 错误汇总（最近日志里出现最多的 15 条）=="
  grep -hiE "\[ERROR\]|Traceback|Exception" ~/.ros/log/*.log 2>/dev/null \
    | sed 's/\[[0-9.]*\]//g' | sort | uniq -c | sort -rn | head -15
} > "$WORK/09_logs.txt" 2>&1

# ---------- 打包 ----------
{
  echo "collect_diag 生成于 $(date)"
  echo "主机: $(hostname)  工作目录见 01_env.txt"
  echo
  for f in "$WORK"/*.txt; do echo "$(basename "$f")  $(wc -l < "$f") 行"; done
} > "$WORK/00_README.txt"

tar czf "$TARBALL" -C "$(dirname "$WORK")" "$(basename "$WORK")" 2>/dev/null
say "完成：$TARBALL  ($(du -h "$TARBALL" 2>/dev/null | cut -f1))"
say "把这个文件带回有网的机器即可；本机临时目录 $WORK 可以不管（重启会自动清理）。"
