"""Nav2 导航封装：把「发目标、等结果、中途被打断、到点复核」收在一处。

这样做的好处是 ``node.py`` 里不再混着 Nav2 的 future 轮询细节，导航这一层
可以单独替换（换 Nav2 之外的动作接口时只改这个文件）。

所有 ``cancel_check`` / ``interrupt_check`` 都是无参回调，由调用方决定语义：

- ``cancel_check()``     → True 表示 action 被取消，直接放弃
- ``interrupt_check()``  → 返回 ``NAV_PAUSED`` / ``NAV_PREEMPTED`` / ``None``
"""

import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from .util import as_check, pose_error, stable_residual

# 导航结果：区分「到位」「被暂停打断」「被抢占打断」「被取消」
NAV_OK = 'ok'
NAV_PAUSED = 'paused'
NAV_PREEMPTED = 'preempted'
NAV_CANCELED = 'canceled'


class Navigator:
    """封装一个 Nav2 ``NavigateToPose`` 客户端。"""

    def __init__(self, node, *, callback_group, action_name, server_wait_s,
                 waypoint_timeout_s, cancel_timeout_s, position_tolerance_m,
                 yaw_tolerance_rad, pose_max_age_s, pose_provider,
                 max_xy_std_m=float('inf'), max_yaw_std_rad=float('inf'),
                 stable_duration_s=0.5, require_localization=False,
                 stop_check=None):
        """建客户端并保存超时、容差与位姿来源。

        Args:
            pose_provider: 无参回调，返回 ``(pose, age_s, xy_std, yaw_std)``；
                pose 为 ``None`` 表示此刻没有可信位姿。
            max_xy_std_m / max_yaw_std_rad: 定位不确定性上限，超过就不做
                到点判定（噪声太大的位姿判了也没意义）。
            stable_duration_s: 到点复核的采样窗口，取窗口内最坏残差。
            require_localization: True 时拿不到可信位姿就判导航失败；
                False（默认）则退化成「信 Nav2」，与 v1 的保守策略相反。
        """
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name,
                                    callback_group=callback_group)
        self._server_wait = server_wait_s
        self._waypoint_timeout = waypoint_timeout_s
        self._cancel_timeout = cancel_timeout_s
        self._position_tolerance = position_tolerance_m
        self._yaw_tolerance = yaw_tolerance_rad
        self._pose_max_age = pose_max_age_s
        self._pose_provider = pose_provider
        self._max_xy_std = max_xy_std_m
        self._max_yaw_std = max_yaw_std_rad
        self._stable_duration = stable_duration_s
        self._require_localization = require_localization
        self._endpoint_note = ''
        # 节点正在退出时置位，所有等待循环据此立刻放弃（Ctrl+C 才能真停下来）
        self._stop_check = stop_check or (lambda: False)
        # 当前正在执行的导航目标，退出时用来尽力取消，避免 Nav2 继续把车开走
        self._active_goal = None

    def navigate(self, pose, *, cancel_check, interrupt_check, feedback_cb=None):
        """导航到 ``pose``；返回 ``(ok, reason)``。

        Args:
            pose: ``geometry_msgs/PoseStamped``，frame 跟随路线文件。
            cancel_check: action 是否已被取消。
            interrupt_check: 暂停/抢占检查，返回中断原因或 ``None``。
            feedback_cb: 收到 Nav2 反馈时调用，参数是剩余距离。
        """
        # 入口统一归一成无参回调：调用方误传布尔值（rclpy 的
        # ``ServerGoalHandle.is_cancel_requested`` 是属性不是方法）时，
        # 这里兜住，不让 "'bool' object is not callable" 跑到点位循环里。
        cancel_check = as_check(cancel_check)
        interrupt_check = as_check(interrupt_check)
        if not self._client.wait_for_server(timeout_sec=self._server_wait):
            return False, 'NavigateToPose action server is unavailable'

        goal = NavigateToPose.Goal()
        goal.pose = pose
        # 用 0 表示「以当前时间为准」，避免路线文件里的旧时间戳导致 goal 被丢
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0

        def on_feedback(message):
            # 反馈回调是 executor 里跑的，抛异常会把整个节点带崩；
            # 上层 publish_feedback 出错时只该丢这一帧反馈。
            try:
                if feedback_cb is not None:
                    feedback_cb(float(message.feedback.distance_remaining))
            except Exception:  # noqa: BLE001 - 反馈丢了不影响导航
                return

        send_future = self._client.send_goal_async(
            goal, feedback_callback=on_feedback)
        if not self._wait(send_future, self._server_wait, cancel_check,
                          interrupt_check):
            # 发送阶段被打断时，目标可能已经被 Nav2 接受、只是我们还没拿到
            # handle。不收尾的话它会变成孤儿 goal：车继续开，而退出路径上的
            # abort() 只看 _active_goal，根本发现不了。
            self._cancel_pending(send_future)
            return False, self._interrupt_reason(cancel_check, interrupt_check) \
                or 'timed out sending NavigateToPose goal'

        try:
            nav_goal = send_future.result()
        except Exception as error:  # noqa: BLE001 - 发送失败不应该让节点崩
            return False, 'NavigateToPose goal request failed: %s' % error
        if nav_goal is None or not nav_goal.accepted:
            return False, 'NavigateToPose rejected the goal'

        self._active_goal = nav_goal
        try:
            result_future = nav_goal.get_result_async()
            finished = self._wait(result_future, self._waypoint_timeout,
                                  cancel_check, interrupt_check)
        finally:
            self._active_goal = None
        if not finished:
            self._cancel(nav_goal, result_future)
            return False, self._interrupt_reason(cancel_check, interrupt_check) \
                or 'navigation timed out after %.1fs' % self._waypoint_timeout

        # 导航「成功」之后也可能刚好被暂停/抢占，先让中断优先
        interrupted = self._interrupt_reason(cancel_check, interrupt_check)
        if interrupted is not None:
            return False, interrupted

        wrapper = result_future.result()
        if wrapper is None or wrapper.status != GoalStatus.STATUS_SUCCEEDED:
            status = (status_name(wrapper.status) if wrapper is not None
                      else 'unknown')
            return False, 'NavigateToPose ended with status %s' % status

        residual = self._endpoint_residual(pose.pose)
        if residual is None:
            note = self._endpoint_note or 'localization unavailable'
            if self._require_localization:
                return False, 'endpoint could not be verified: %s' % note
            # 拿不到可信位姿就沿用 v2 的宽容策略，只告警不判失败
            self._node.get_logger().warning(
                'endpoint check skipped (%s); trusting NavigateToPose' % note)
            return True, NAV_OK
        position_error, yaw_error = residual
        if (position_error > self._position_tolerance or
                yaw_error > self._yaw_tolerance):
            return False, (
                'endpoint residual too large: position=%.3fm (limit %.3fm), '
                'yaw=%.3frad (limit %.3frad)'
                % (position_error, self._position_tolerance,
                   yaw_error, self._yaw_tolerance))
        return True, NAV_OK

    # -- 内部工具 --------------------------------------------------
    def abort(self):
        """尽力取消当前导航目标（退出/Ctrl+C 时调用，最多阻塞 2 s）。

        进程被杀掉时 Nav2 不会自动放弃目标，车会继续走到终点，所以退出路径上
        必须显式 cancel 一次。
        """
        goal = self._active_goal
        self._active_goal = None
        if goal is None:
            return
        try:
            future = goal.cancel_goal_async()
        except Exception:  # noqa: BLE001 - 退出路径上不能再抛异常
            return
        deadline = time.monotonic() + min(self._cancel_timeout, 2.0)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

    def _interrupt_reason(self, cancel_check, interrupt_check):
        """按「取消 > 退出 > 暂停/抢占」的优先级给出中断原因。"""
        # cancel_check 也容忍布尔值：rclpy 的 ``ServerGoalHandle
        # .is_cancel_requested`` 是属性而不是方法，历史上误把它当回调传进来，
        # 每个点位一开始导航就抛 "'bool' object is not callable"。
        stop = getattr(self, '_stop_check', None)
        if stop is not None and stop():
            return NAV_CANCELED
        cancelled = cancel_check() if callable(cancel_check) else bool(cancel_check)
        if cancelled:
            return NAV_CANCELED
        return interrupt_check() if callable(interrupt_check) else None

    def _wait(self, future, timeout, cancel_check, interrupt_check):
        """等到 future 完成；被取消/暂停/抢占则提前返回 False。"""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if self._interrupt_reason(cancel_check, interrupt_check) is not None:
                return False
            time.sleep(0.02)
        return future.done()

    def _cancel(self, nav_goal, result_future) -> None:
        """取消导航并等 Nav2 收尾，避免动作堆积。

        三个坑都要防：cancel 请求本身可能失败（goal 已结束、链路断开）；
        等 cancel 和等 result 不能共用一个 deadline，否则后者等于不等；
        cancel 被拒说明 goal 已经结束，再等 result 就是白等一个 cancel_timeout。
        """
        try:
            cancel_future = nav_goal.cancel_goal_async()
        except Exception:  # noqa: BLE001 - 取消失败不该把巡检带崩
            return
        deadline = time.monotonic() + self._cancel_timeout
        while not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        try:
            response = cancel_future.result() if cancel_future.done() else None
        except Exception:  # noqa: BLE001
            response = None
        if response is not None and not list(getattr(response, 'goals_canceling', [])):
            return              # 没有 goal 处于 canceling：已完成或被拒
        finish_deadline = time.monotonic() + self._cancel_timeout
        while not result_future.done() and time.monotonic() < finish_deadline:
            time.sleep(0.02)

    def _cancel_pending(self, send_future) -> None:
        """回收「发送阶段就被打断」的目标。"""
        self._drain(send_future)
        if not send_future.done():
            return
        try:
            nav_goal = send_future.result()
        except Exception:  # noqa: BLE001
            return
        if nav_goal is None or not nav_goal.accepted:
            return
        self._active_goal = nav_goal
        try:
            self._cancel(nav_goal, nav_goal.get_result_async())
        except Exception:  # noqa: BLE001
            pass
        finally:
            self._active_goal = None

    def _drain(self, future) -> None:
        """等一个已经没用的 future 结束，防止回调泄漏。"""
        deadline = time.monotonic() + min(self._cancel_timeout, 2.0)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

    def _endpoint_residual(self, target_pose):
        """到点复核：先筛定位质量，再在稳定窗口内取最坏残差。

        返回 ``(位置误差, 偏航误差)`` 或 ``None``（``self._endpoint_note`` 说明原因）。
        """
        self._endpoint_note = ''
        pose, age, xy_std, yaw_std = self._pose_provider()
        if pose is None:
            self._endpoint_note = 'no localization pose'
            return None
        if age > self._pose_max_age:
            self._endpoint_note = 'localization pose is %.1fs old' % age
            return None
        if xy_std > self._max_xy_std or yaw_std > self._max_yaw_std:
            self._endpoint_note = (
                'localization uncertainty too high: xy_std=%.3f/%.3fm, '
                'yaw_std=%.3f/%.3frad'
                % (xy_std, self._max_xy_std, yaw_std, self._max_yaw_std))
            return None

        def sample():
            if self._stop_check():
                return None
            current, current_age, current_xy, current_yaw = self._pose_provider()
            if current is None or current_age > self._pose_max_age:
                return None
            if current_xy > self._max_xy_std or current_yaw > self._max_yaw_std:
                return None
            return pose_error(current, target_pose)

        residual = stable_residual(sample, self._stable_duration)
        if residual is None and self._stop_check():
            # 关机途中放弃复核：把原因写清楚，别让日志说成“定位不可用”
            self._endpoint_note = 'stopped while verifying endpoint'
        return residual


#: Nav2 ``GoalStatus`` 的数字值到可读名字
_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def status_name(status) -> str:
    """把 ``GoalStatus`` 的数字换成人能读的名字，未知值原样返回。"""
    return _STATUS_NAMES.get(status, 'STATUS_%s' % (status,))
