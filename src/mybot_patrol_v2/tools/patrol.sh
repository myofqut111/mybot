#!/usr/bin/env bash
# 一键巡检：后台拉起整条链路，前台发任务并看反馈。
#
# 用法:
#   patrol.sh                    # 路线 main，跑一轮
#   patrol.sh main 0 1           # 路线 main，once，1 轮
#   patrol.sh main 1 3           # 路线 main，repeat，3 轮
#
# 退出方式：
#   - 任务自然跑完：脚本自动收尾并关掉后台 launch
#   - 想提前停：本终端 Ctrl+C，脚本会给 launch 发 SIGINT，
#     执行器收到后先取消 Nav2 目标再退出（v0.2.2 起）
set -uo pipefail

ROUTE="${1:-main}"
MODE="${2:-0}"        # 0=once 1=repeat 2=random
LOOPS="${3:-1}"
LOG="${TMPDIR:-/tmp}/patrol_launch_$$.log"
ACTION=/execute_patrol
ACTION_TYPE=mybot_patrol_interfaces/action/ExecutePatrol

if ! command -v ros2 >/dev/null 2>&1; then
    echo "找不到 ros2 命令，先 source /opt/ros/humble/setup.bash 和工作空间里的 install/setup.bash" >&2
    exit 1
fi

echo "==> 启动巡检系统（日志: $LOG）"
ros2 launch mybot_patrol_v2 patrol_v2.launch.py >"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "==> 停止巡检系统"
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        for _ in $(seq 1 40); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.1
        done
        kill -0 "$LAUNCH_PID" 2>/dev/null && kill -TERM "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# 等 action server 上线（最多 20 秒）
for _ in $(seq 1 40); do
    if ros2 action list 2>/dev/null | grep -qx "$ACTION"; then
        break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "启动失败，日志如下：" >&2
        tail -n 40 "$LOG" >&2
        exit 1
    fi
    sleep 0.5
done

if ! ros2 action list 2>/dev/null | grep -qx "$ACTION"; then
    echo "20 秒内没等到 $ACTION，日志：" >&2
    tail -n 40 "$LOG" >&2
    exit 1
fi

echo "==> 发送任务: route=$ROUTE mode=$MODE loops=$LOOPS"
ros2 action send_goal "$ACTION" "$ACTION_TYPE" \
    "{route_name: '$ROUTE', mode: $MODE, repeat_count: $LOOPS}" --feedback
RC=$?

echo
echo "==> 任务结束（返回码 $RC），完整日志：$LOG"
if [ "$RC" -ne 0 ]; then
    tail -n 40 "$LOG"
fi
exit "$RC"
