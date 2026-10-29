#!/usr/bin/env bash
# 检查「现在跑的是不是最新版」。逐层往下查，第一处 ❌ 就是断点。
#
# 用法： bash check_version.sh [期望版本，默认 0.3.3]
set -uo pipefail

WANT="${1:-0.3.3}"
WS="${WS:-$HOME/fishbot_full_ws}"
PKG=fishbot_patrol_v2
FAIL=0

pass() { printf '  \033[32m✅\033[0m %s\n' "$*"; }
fail() { printf '  \033[31m❌\033[0m %s\n' "$*"; FAIL=1; }
warn() { printf '  \033[33m⚠ \033[0m %s\n' "$*"; }
head_() { printf '\n\033[1m%s\033[0m\n' "$*"; }

head_ "1. 源码版本（$WS/src/$PKG）"
SRC_XML="$WS/src/$PKG/package.xml"
if [ -f "$SRC_XML" ]; then
    SRC_VER=$(grep -m1 -oP '(?<=<version>)[^<]+' "$SRC_XML")
    [ "$SRC_VER" = "$WANT" ] && pass "源码 $SRC_VER" \
        || fail "源码是 $SRC_VER，期望 $WANT（zip 没覆盖进去？）"
else
    fail "找不到 $SRC_XML"
fi

head_ "2. 装进 install 的代码"
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f "$WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi
NODE_PY=$(python3 -c 'import fishbot_patrol_v2.node as m; print(m.__file__)' 2>/dev/null)
if [ -z "$NODE_PY" ]; then
    fail "import 不到 fishbot_patrol_v2（没 build 或没 source）"
elif [ ! -f "$NODE_PY" ]; then
    fail "模块路径指向不存在的文件：$NODE_PY"
else
    pass "模块路径 $NODE_PY"
    # 每个版本加一个特征串，命中最新的那几个就能判断
    declare -A MARKS=(
        ["cancel_check=lambda: goal_handle.is_cancel_requested"]="0.2.5 及以后"
        ["def _non_negative"]="0.2.6 及以后"
        ["L%d-A%d"]="0.2.4 及以后（任务 id 带轮次和尝试号）"
    )
    for mark in "${!MARKS[@]}"; do
        if grep -qF "$mark" "$NODE_PY"; then
            pass "含 ${MARKS[$mark]} 的特征"
        else
            fail "缺 ${MARKS[$mark]} 的特征：$mark"
        fi
    done
fi

head_ "3. 正在跑的进程"
if pgrep -f "$PKG/lib/$PKG/patrol_executor_v2" >/dev/null 2>&1 \
   || pgrep -f "patrol_executor_v2 --ros-args" >/dev/null 2>&1; then
    pass "patrol_executor_v2 在运行"
    # 进程启动时间 vs 源码 mtime
    PID=$(pgrep -f "patrol_executor_v2" | head -1)
    START=$(ps -o lstart= -p "$PID" 2>/dev/null)
    MTIME=$(date -r "$NODE_PY" '+%Y-%m-%d %H:%M:%S' 2>/dev/null)
    echo "    进程启动: ${START:-未知}"
    echo "    源码改动: ${MTIME:-未知}"
    if [ -n "$START" ] && [ -n "$MTIME" ] \
       && [ "$(date -d "$START" +%s 2>/dev/null || echo 0)" -lt "$(date -d "$MTIME" +%s 2>/dev/null || echo 0)" ]; then
        fail "进程比源码旧 —— 装的是新代码，但内存里跑的是旧的，重启 launch"
    fi
else
    fail "patrol_executor_v2 没在跑"
fi

head_ "4. 有没有旧版本混在里面"
for pat in "fishbot_patrol_executor/lib" "fishbot_patrol_v2/lib"; do
    n=$(pgrep -fc "$pat" 2>/dev/null || echo 0)
    [ "$n" -gt 0 ] && echo "    $pat → $n 个进程"
done
OLD=$(pgrep -f "fishbot_patrol_executor/" | head -3)
if [ -n "$OLD" ]; then
    fail "v1 执行器还在跑（$(echo "$OLD" | tr '\n' ' ')）—— 两套抢同一批话题"
else
    pass "没有 v1 残留"
fi
for name in patrol_task_manager route_manager; do
    cnt=$(ros2 node list 2>/dev/null | grep -c "/$name\$")
    [ "$cnt" -le 1 ] && pass "$name 只有 $cnt 个实例" \
        || fail "$name 有 $cnt 个实例（重复！）"
done

head_ "5. 运行时证据（最近一次启动日志）"
# ros2 launch 把各节点输出并到同一个日志目录，文件名是按 pid 命名的，
# 直接拿节点名去通配是找不到文件的。
LOGDIR="$HOME/.ros/log/latest"
if [ -d "$LOGDIR" ]; then
    echo "    $LOGDIR"
    READY=$(grep -h "patrol_executor_v2 ready" "$LOGDIR"/*.log 2>/dev/null | tail -1)
    [ -n "$READY" ] && pass "$READY" || warn "日志里没有 ready 行（还没启动过？）"
    MODEL=$(grep -h "Loading YOLO model" "$LOGDIR"/*.log 2>/dev/null | tail -1)
    if [ -n "$MODEL" ]; then
        case "$MODEL" in
            *fire_extinguisher*) pass "$MODEL" ;;
            *) fail "视觉加载的不是灭火器模型：$MODEL" ;;
        esac
    else
        warn "没找到 Loading YOLO model（视觉节点没起过？）"
    fi
else
    warn "没有 $LOGDIR，可能从未启动过"
fi

head_ "结论"
if [ "$FAIL" -eq 0 ]; then
    printf '  \033[32m全部通过：跑的就是 %s\033[0m\n' "$WANT"
else
    printf '  \033[31m有不一致，按上面第一个 ❌ 往下查\033[0m\n'
fi
exit "$FAIL"
