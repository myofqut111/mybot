#!/usr/bin/env bash
# 把 mybot_patrol_v2 升级到 zip 里的版本，并验证装进去的确实是新版。
#
# 用法（在 VM 上）：
#   bash upgrade_patrol.sh ~/mybot_patrol_v2.zip
#
# 做四件事：备份旧包 → 解包覆盖 → 清掉旧 build/install → 重新编译并验证。
set -euo pipefail

ZIP="${1:-$HOME/mybot_patrol_v2.zip}"
WS="${2:-$HOME/mybot_full_ws}"
PKG=mybot_patrol_v2

say() { printf '\n==> %s\n' "$*"; }
die() { printf '\n[失败] %s\n' "$*" >&2; exit 1; }

[ -f "$ZIP" ] || die "找不到 $ZIP"
[ -d "$WS/src" ] || die "$WS/src 不存在（用第二个参数指定工作空间）"

say "1/6 检查包是否重复"
mapfile -t FOUND < <(find "$WS/src" -maxdepth 3 -name package.xml -path "*${PKG}*" -print)
printf '%s\n' "${FOUND[@]}"
[ "${#FOUND[@]}" -eq 0 ] && die "$WS/src 下没有 $PKG，先解包再说"
if [ "${#FOUND[@]}" -gt 1 ]; then
    die "找到 ${#FOUND[@]} 份 $PKG，colcon 会编译错的那份，先删掉多余的"
fi

say "2/6 备份旧包"
STAMP=$(date +%Y%m%d-%H%M%S)
BAK="$WS/src/${PKG}.bak.$STAMP"
mv "$WS/src/$PKG" "$BAK"
echo "旧包已改名为 $BAK"

say "3/6 解包"
unzip -q "$ZIP" -d "$WS/src/"

say "4/6 校验源码版本"
VER=$(grep -m1 -oP '(?<=<version>)[^<]+' "$WS/src/$PKG/package.xml" || true)
echo "package.xml 版本: $VER"
[ "$VER" = "0.2.5" ] || die "版本不是 0.2.5，zip 可能拿错了"

grep -q 'cancel_check=lambda' "$WS/src/$PKG/$PKG/node.py" \
    || die "node.py 里没有 cancel_check=lambda，源码不对"
grep -q 'def as_check' "$WS/src/$PKG/$PKG/util.py" \
    || die "util.py 里没有 as_check，源码不对"
echo "源码检查通过"

say "5/6 清旧构建并重新编译"
rm -rf "$WS/build/$PKG" "$WS/install/$PKG"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select "$PKG" --symlink-install

say "6/6 验证装进去的版本"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
INST=$(python3 -c "import mybot_patrol_v2.node as m; print(m.__file__)")
echo "实际加载: $INST"
grep -q 'cancel_check=lambda' "$INST" \
    || die "加载到的 node.py 仍是旧版（$INST）"
ros2 pkg xml "$PKG" | grep '<version>' || true

say "完成"
cat <<'TIP'
下一步：重启 launch（正在跑的节点还持有旧代码）
  ros2 launch mybot_patrol_v2 patrol_v2.launch.py
就绪日志应为：
  [patrol_executor_v2]: patrol_executor_v2 ready (policy=skip, resume=True)
TIP
