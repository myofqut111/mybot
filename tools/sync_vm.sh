#!/usr/bin/env bash
# 把仓库代码同步到虚拟机（虚拟机里装的是 fishbot_*，仓库是 mybot_*）
#
#   bash tools/sync_vm.sh                  # 默认 192.168.202.128
#   bash tools/sync_vm.sh 192.168.1.103    # 局域网网段
#   bash tools/sync_vm.sh 192.168.202.128 /path/to/key
#
# 做的事：本地打包（mybot_* 改名回 fishbot_*）→ 传上去 → 备份旧 src → 解包
#        → colcon build → 打印装好的版本号
# 不碰：~/.ros 下的路线/基线/进度（文件名本来就叫 fishbot_*），也没重启任何进程。

set -u

HOST="${1:-192.168.202.128}"
KEY="${2:-$HOME/.ssh/id_ed25519_vm}"
USER_NAME="${VM_USER:-my}"
WS="${VM_WS:-/home/$USER_NAME/fishbot_full_ws}"
PKG_LIST="fishbot_console fishbot_patrol_v2 fishbot_safety fishbot_vision \
fishbot_patrol_interfaces fishbot_patrol_route_manager fishbot_patrol_task_manager \
fishbot_navigation fishbot_bringup fishbot_description fishbot_patrol_executor"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAGE="$ROOT/dist_vm"
TAR="$STAGE/fishbot_src.tar.gz"

echo "== 1/5 本地打包（mybot_* → fishbot_*）=="
python "$ROOT/tools/sync_vm.py" --out "$STAGE" | tail -3 || {
  echo "打包失败"; exit 1; }

echo "== 2/5 传到 $HOST =="
scp -i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no \
    "$TAR" "$USER_NAME@$HOST:/tmp/fishbot_src.tar.gz" || {
  echo "传输失败：先确认 $HOST 能 ssh 上去"; exit 1; }

echo "== 3/5 备份旧 src 并解包 =="
ssh -i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no \
    "$USER_NAME@$HOST" "set -e
      cd $WS
      TS=\$(date +%Y%m%d_%H%M%S)
      tar czf /tmp/src_backup_\$TS.tar.gz src 2>/dev/null || true
      echo \"  备份：/tmp/src_backup_\$TS.tar.gz\"
      rm -rf /tmp/fishbot_new && mkdir -p /tmp/fishbot_new
      tar xzf /tmp/fishbot_src.tar.gz -C /tmp/fishbot_new
      mkdir -p src
      for d in /tmp/fishbot_new/*/; do
        name=\$(basename \$d)
        rm -rf \"src/\$name\"
        cp -a \"\$d\" \"src/\$name\"
      done
      echo \"  解包完成：\$(ls src | wc -l) 个包\"
    " || { echo "解包失败"; exit 1; }

echo "== 4/5 colcon build =="
ssh -i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no \
    "$USER_NAME@$HOST" "set +u
      cd $WS
      source /opt/ros/humble/setup.bash
      colcon build --packages-select $PKG_LIST 2>&1 | tail -6
    " || { echo "编译失败：看上面 tail 的最后几行"; exit 1; }

echo "== 5/5 核对版本 =="
ssh -i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no \
    "$USER_NAME@$HOST" "set +u
      cd $WS
      source /opt/ros/humble/setup.bash
      source install/setup.bash
      for p in fishbot-console fishbot-patrol-v2 fishbot-safety fishbot-vision; do
        v=\$(timeout 20 python3 -c \"import importlib.metadata as m;print(m.version('\$p'))\" 2>/dev/null || echo '?')
        printf '  %-24s %s\n' \"\$p\" \"\$v\"
      done
    "

echo
echo "同步完成。启动控制台（在虚拟机里执行）："
echo "  source $WS/install/setup.bash"
echo "  ros2 launch fishbot_console console.launch.py"
