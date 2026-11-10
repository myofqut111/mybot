"""Execute persistent patrol routes through Nav2 and the task manager."""

# PatrolExecutor = 巡检系统的核心执行器（由 Go2 工程移植，硬件无关）。
# 它做什么？拿到一条巡检路线（一串带朝向的航点），然后：
#   1. 逐点调用 Nav2 的 /navigate_to_pose 把机器人导航过去；
#   2. Nav2 说"到了"还不够，自己再做一次终点校验（位置+朝向残差）；
#   3. 到点后通过任务管理器（/patrol/task/request）分发"点位任务"
#      （比如拍照、识别），等任务结果回来才推进到下一个点；
#   4. 支持暂停/恢复（pause != cancel：暂停保留进度，恢复从原点继续）。
# 数据流：路由管理器(提供路线) <-> 本执行器 -> Nav2(运动) / 任务管理器(点位任务)

import json
import math
import random
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from mybot_patrol_interfaces.action import ExecutePatrol
from mybot_patrol_interfaces.msg import PatrolTaskRequest, PatrolTaskResult
from mybot_patrol_interfaces.srv import GetRoute, PausePatrol, ResumePatrol
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .core import VALID_MODES, loop_limit, pose_residual, visit_order


class PatrolExecutor(Node):
    """Action server that coordinates navigation and per-point inspection."""

    def __init__(self) -> None:
        super().__init__('patrol_executor')
        # -- 参数：全是执行策略的旋钮，可在 launch 里覆盖 --
        self.declare_parameter('server_wait_timeout_s', 10.0)   # 等服务/动作就绪超时
        self.declare_parameter('waypoint_timeout_s', 180.0)     # 单点导航最长时长
        self.declare_parameter('navigation_cancel_timeout_s', 10.0)  # 等 Nav2 取消完成
        self.declare_parameter('task_result_timeout_s', 130.0)  # 等点位任务结果
        self.declare_parameter('amcl_pose_max_age_s', 5.0)      # 定位数据最大年龄
        self.declare_parameter('final_position_tolerance_m', 0.10)  # 到点位置容差
        self.declare_parameter('final_yaw_tolerance_rad', 0.10)     # 到点朝向容差
        self.declare_parameter('endpoint_max_xy_std_m', 0.15)   # 终点校验允许的定位不确定性
        self.declare_parameter('endpoint_max_yaw_std_rad', 0.15)
        self.declare_parameter('endpoint_stable_duration_s', 0.5)   # 残差须持续达标时长
        self.declare_parameter('endpoint_retry_count', 1)       # 终点校验失败重试次数
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('random_seed', -1)               # -1 = 随机打乱不可复现

        self._server_wait = self._positive_parameter('server_wait_timeout_s')
        self._waypoint_timeout = self._positive_parameter('waypoint_timeout_s')
        self._cancel_timeout = self._positive_parameter(
            'navigation_cancel_timeout_s')
        self._task_timeout = self._positive_parameter('task_result_timeout_s')
        self._pose_max_age = self._positive_parameter('amcl_pose_max_age_s')
        self._position_tolerance = self._positive_parameter(
            'final_position_tolerance_m')
        self._yaw_tolerance = self._positive_parameter(
            'final_yaw_tolerance_rad')
        self._endpoint_max_xy_std = self._positive_parameter(
            'endpoint_max_xy_std_m')
        self._endpoint_max_yaw_std = self._positive_parameter(
            'endpoint_max_yaw_std_rad')
        self._endpoint_stable_duration = self._positive_parameter(
            'endpoint_stable_duration_s')
        self._global_frame = str(
            self.get_parameter('global_frame').value).strip()
        self._robot_base_frame = str(
            self.get_parameter('robot_base_frame').value).strip()
        if not self._global_frame or not self._robot_base_frame:
            raise ValueError('global_frame and robot_base_frame must not be empty')
        self._endpoint_retries = int(
            self.get_parameter('endpoint_retry_count').value)
        if self._endpoint_retries < 0:
            raise ValueError('endpoint_retry_count must not be negative')
        seed = int(self.get_parameter('random_seed').value)
        self._random = random.Random(None if seed < 0 else seed)

        self._callbacks = ReentrantCallbackGroup()
        reliable = QoSProfile(depth=20)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        latched = QoSProfile(depth=20)     # latched = 新订阅者也能立刻收到最新值
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # -- 对外接口（都是"接别人的"）--------------------------
        self._route_client = self.create_client(
            GetRoute, '/patrol_route/get', callback_group=self._callbacks)  # 问路由管理器要路线
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self._callbacks)  # 调 Nav2 导航
        self._task_request_pub = self.create_publisher(
            PatrolTaskRequest, '/patrol/task/request', reliable)  # 向任务管理器发点位任务
        self._status_pub = self.create_publisher(
            String, '/patrol/execution/status', latched)  # 对外发布执行状态(JSON)
        self.create_subscription(
            PatrolTaskResult, '/patrol/task/result', self._on_task_result,
            latched, callback_group=self._callbacks)  # 收任务管理器回执
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 20,
            callback_group=self._callbacks)  # 收定位(含协方差，用于终点校验)

        # -- 内部状态 -------------------------------------------
        self._lock = threading.RLock()
        self._latest_pose = None           # 最新 AMCL 位姿
        self._latest_pose_time = 0.0
        self._latest_xy_std = float('inf') # 最新定位 XY 标准差（大 = 定位不准）
        self._latest_yaw_std = float('inf')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._task_waiters = {}            # (execution_id, point_index) -> 等任务结果的事件
        self._active_nav_goal = None       # 当前在跑的 Nav2 目标句柄
        self._patrol_active = False
        self._active_goal_handle = None
        self._active_execution_id = ''     # 本次巡检的唯一执行 ID（防结果串扰）
        self._active_route_name = ''
        self._current_index = 0
        self._current_loop = 0
        self._completed_points = 0
        self._completed_loops = 0
        self._pause_requested = False      # 有人请求暂停
        self._pause_reason = ''
        self._paused = False               # 当前是否真的挂起中
        self._pause_generation = 0         # 暂停次数（用于检测"暂停又恢复"）
        self._pause_condition = threading.Condition(self._lock)  # 暂停/恢复的唤醒条件变量
        # 暂停/恢复服务：任何客户端(含上层编排器)都能调
        self.create_service(
            PausePatrol, '/patrol/pause', self._pause_patrol,
            callback_group=self._callbacks)
        self.create_service(
            ResumePatrol, '/patrol/resume', self._resume_patrol,
            callback_group=self._callbacks)
        # 核心：巡检 Action 服务器，外部发 /execute_patrol 目标即开始巡检
        self._action_server = ActionServer(
            self, ExecutePatrol, '/execute_patrol',
            execute_callback=self._execute,     # 目标被接受后在这里执行
            goal_callback=self._goal_callback,  # 决定接受还是拒绝目标
            cancel_callback=self._cancel_callback,  # 处理取消
            callback_group=self._callbacks,
        )
        self._publish_status('', '', 'IDLE', 'Patrol executor ready')

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('%s must be finite and greater than zero' % name)
        return value

    def _goal_callback(self, goal_request):
        if int(goal_request.mode) not in VALID_MODES:
            self.get_logger().warning(
                'Rejecting unsupported patrol mode %d' % goal_request.mode)
            return GoalResponse.REJECT
        with self._lock:
            if self._patrol_active:
                self.get_logger().warning(
                    'Rejecting patrol goal because another patrol is active')
                return GoalResponse.REJECT
            self._patrol_active = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        with self._lock:
            nav_goal = self._active_nav_goal
            self._pause_condition.notify_all()
        if nav_goal is not None:
            nav_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _control_matches(self, requested_id):
        return (not requested_id.strip() or
                requested_id.strip() == self._active_execution_id)

    def _fill_control_response(self, response, success, message, state=None):
        with self._lock:
            response.success = success
            response.message = message
            response.execution_id = self._active_execution_id
            response.state = state or ('PAUSED' if self._paused else 'RUNNING')
            response.current_index = self._current_index
            response.completed_points = self._completed_points
            response.current_loop = self._current_loop
        return response

    def _pause_patrol(self, request, response):
        with self._lock:
            if not self._patrol_active or self._active_goal_handle is None:
                return self._fill_control_response(
                    response, False, 'No active patrol to pause', 'IDLE')
            if not self._control_matches(request.execution_id):
                return self._fill_control_response(
                    response, False,
                    'execution_id does not match the active patrol')
            if self._pause_requested or self._paused:
                return self._fill_control_response(
                    response, True, 'Patrol is already paused', 'PAUSED')
            self._pause_requested = True
            self._pause_reason = 'Paused by operator'
            self._pause_generation += 1
            nav_goal = self._active_nav_goal
            self._pause_condition.notify_all()
        if nav_goal is not None:
            nav_goal.cancel_goal_async()
        return self._fill_control_response(
            response, True, 'Pause requested', 'PAUSING')

    def _resume_patrol(self, request, response):
        with self._lock:
            if not self._patrol_active or self._active_goal_handle is None:
                return self._fill_control_response(
                    response, False, 'No paused patrol to resume', 'IDLE')
            if not self._control_matches(request.execution_id):
                return self._fill_control_response(
                    response, False,
                    'execution_id does not match the active patrol')
            if not self._pause_requested and not self._paused:
                return self._fill_control_response(
                    response, False, 'Patrol is not paused', 'RUNNING')
            self._pause_requested = False
            self._pause_reason = ''
            self._paused = False
            self._pause_condition.notify_all()
            execution_id = self._active_execution_id
            route_name = self._active_route_name
        self._publish_status(
            execution_id, route_name, 'RESUMED',
            'Patrol resumed from the saved waypoint')
        return self._fill_control_response(
            response, True, 'Patrol resumed', 'RUNNING')

    def _on_amcl_pose(self, message) -> None:
        covariance = message.pose.covariance
        if len(covariance) < 36:
            return
        xy_variance = max(float(covariance[0]), float(covariance[7]))
        yaw_variance = float(covariance[35])
        if (not math.isfinite(xy_variance) or
                not math.isfinite(yaw_variance) or
                xy_variance < 0.0 or yaw_variance < 0.0):
            return
        with self._lock:
            self._latest_pose = message
            self._latest_pose_time = time.monotonic()
            self._latest_xy_std = math.sqrt(xy_variance)
            self._latest_yaw_std = math.sqrt(yaw_variance)

    def _on_task_result(self, message) -> None:
        if message.status not in (
                PatrolTaskResult.SUCCEEDED,
                PatrolTaskResult.FAILED,
                PatrolTaskResult.CANCELED):
            return
        key = (message.execution_id, int(message.point_index))
        with self._lock:
            waiter = self._task_waiters.get(key)
            if waiter is None:
                return
            waiter['result'] = message
            waiter['event'].set()

    def _execute(self, goal_handle):
        with self._lock:
            self._active_goal_handle = goal_handle
        try:
            return self._execute_route(goal_handle)
        except Exception as error:  # Keep the action server reusable after faults.
            self.get_logger().exception('Unhandled patrol execution error')
            result = ExecutePatrol.Result()
            with self._lock:
                execution_id = self._active_execution_id
                route_name = self._active_route_name
            return self._finish(
                goal_handle, result, 'FAILED',
                'Internal patrol executor error: %s' % error, 0, 0,
                execution_id, route_name)

    # == 巡检主流程 ==
    def _execute_route(self, goal_handle):
        request = goal_handle.request
        route_name = request.route_name.strip()
        run_id = uuid.uuid4().hex   # 本次巡检唯一 ID（用于结果去重/防串扰）
        completed_points = 0
        completed_loops = 0
        result = ExecutePatrol.Result()
        with self._lock:
            self._active_execution_id = run_id
            self._active_route_name = route_name
            self._current_index = 0
            self._current_loop = 0
            self._completed_points = 0
            self._completed_loops = 0

        self._publish_status(run_id, route_name, 'LOADING_ROUTE',
                             'Loading patrol route')
        # 第 1 步：向路由管理器要路线（route_name 为空 = 用激活路线）
        route_response, error = self._load_route(route_name, goal_handle)
        if error:
            state = 'CANCELED' if goal_handle.is_cancel_requested else 'FAILED'
            return self._finish(goal_handle, result, state, error,
                                completed_points, completed_loops,
                                run_id, route_name)
        route = route_response.route
        route_name = route.name
        with self._lock:
            self._active_route_name = route_name
        if not route.points:
            return self._finish(
                goal_handle, result, 'FAILED',
                'Patrol route %r has no patrol points' % route_name,
                completed_points, completed_loops, run_id, route_name)

        # 第 2 步：算总循环数，开始巡检
        #   total_loops == 0 表示无限循环（REPEAT + repeat_count=0）
        mode = int(request.mode)
        total_loops = loop_limit(mode, int(request.repeat_count))
        while total_loops == 0 or completed_loops < total_loops:
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, result, 'CANCELED',
                                    'Patrol canceled', completed_points,
                                    completed_loops, run_id, route_name)
            current_loop = completed_loops + 1
            # 每轮一个独立 execution_id，防止上一轮的任务结果串到这一轮
            loop_execution_id = '%s-%d' % (run_id, current_loop)
            order = visit_order(len(route.points), mode, self._random)  # ONCE/REPEAT/RANDOM
            for route_index in order:
                point = route.points[route_index]
                # 暂停门：如果处于暂停状态，在这里挂起等待恢复
                if not self._wait_for_resume(
                        goal_handle, run_id, route_name, route_index,
                        completed_points, current_loop):
                    return self._finish(
                        goal_handle, result, 'CANCELED', 'Patrol canceled',
                        completed_points, completed_loops, run_id, route_name)
                self._set_progress(
                    route_index, current_loop, completed_points,
                    completed_loops)
                if goal_handle.is_cancel_requested:
                    return self._finish(goal_handle, result, 'CANCELED',
                                        'Patrol canceled', completed_points,
                                        completed_loops, run_id, route_name)

                self._feedback(
                    goal_handle, run_id, 'NAVIGATING', route_index,
                    completed_points, current_loop, point.point_id)
                self._publish_status(
                    run_id, route_name, 'NAVIGATING',
                    'Navigating to %s' % point.point_id, route_index,
                    completed_points, completed_loops, point.point_id)
                # 第 3 步：导航到当前点（含失败重试、暂停中断处理）
                while True:
                    ok, message = self._navigate(
                        point.pose, goal_handle,
                        lambda distance: self._feedback(
                            goal_handle, run_id, 'NAVIGATING',
                            route_index, completed_points, current_loop,
                            point.point_id, distance))
                    if ok:
                        break
                    if message == 'paused':
                        # 导航途中被暂停 -> 等恢复后重试同一点
                        if not self._wait_for_resume(
                                goal_handle, run_id, route_name, route_index,
                                completed_points, current_loop):
                            return self._finish(
                                goal_handle, result, 'CANCELED',
                                'Patrol canceled', completed_points,
                                completed_loops, run_id, route_name)
                        # Retry the same point after resume.
                        continue
                    state = 'CANCELED' if goal_handle.is_cancel_requested else 'FAILED'
                    return self._finish(
                        goal_handle, result, state,
                        'Point %s: %s' % (point.point_id, message),
                        completed_points, completed_loops,
                        loop_execution_id, route_name)

                # 第 4 步：到点后执行点位任务（拍照/识别/等待等），
                # 任务成功才算这个点完成，才推进到下一个点
                self._feedback(
                    goal_handle, run_id, 'WAITING_TASK',
                    route_index, completed_points, current_loop,
                    point.point_id, 0.0)
                self._publish_status(
                    run_id, route_name, 'WAITING_TASK',
                    'Waiting for task at %s' % point.point_id, route_index,
                    completed_points, completed_loops, point.point_id)
                ok, message = self._run_task(
                    loop_execution_id, route_name, route_index, point,
                    goal_handle)
                if not ok:
                    state = 'CANCELED' if goal_handle.is_cancel_requested else 'FAILED'
                    return self._finish(
                        goal_handle, result, state,
                        'Point %s task: %s' % (point.point_id, message),
                        completed_points, completed_loops,
                        loop_execution_id, route_name)
                completed_points += 1   # 任务成功 -> 记录完成
                self._set_progress(
                    route_index, current_loop, completed_points,
                    completed_loops)

            completed_loops += 1
            self._set_progress(
                0, completed_loops, completed_points, completed_loops)

        return self._finish(
            goal_handle, result, 'SUCCEEDED',
            'Patrol completed', completed_points, completed_loops,
            run_id, route_name)

    # 保存进度快照（供暂停/状态查询用）
    def _set_progress(self, current_index, current_loop, completed_points,
                      completed_loops):
        with self._lock:
            self._current_index = int(current_index)
            self._current_loop = int(current_loop)
            self._completed_points = int(completed_points)
            self._completed_loops = int(completed_loops)

    def _pause_requested_now(self):
        with self._lock:
            return self._pause_requested

    # == 暂停门 ==
    def _wait_for_resume(self, goal_handle, execution_id, route_name,
                         current_index, completed_points, current_loop):
        publish_paused = False
        with self._lock:
            if not self._pause_requested:
                return not goal_handle.is_cancel_requested   # 没暂停 -> 直接放行
            if not self._paused:
                # 第一次进入暂停：记录暂停时刻的进度，并对外发布 PAUSED
                self._paused = True
                self._current_index = int(current_index)
                self._current_loop = int(current_loop)
                self._completed_points = int(completed_points)
                publish_paused = True
        if publish_paused:
            self._publish_status(
                execution_id, route_name, 'PAUSED',
                'Patrol paused; waiting for resume', current_index,
                completed_points, self._completed_loops)
        with self._pause_condition:
            while self._pause_requested:   # 在这里阻塞，等 resume 唤醒
                if goal_handle.is_cancel_requested:
                    return False
                self._pause_condition.wait(timeout=0.1)
        return not goal_handle.is_cancel_requested

    # 向路由管理器请求路线
    def _load_route(self, route_name, goal_handle):
        if not self._route_client.wait_for_service(
                timeout_sec=self._server_wait):
            return None, 'Route manager service is unavailable'
        request = GetRoute.Request()
        request.route_name = route_name
        future = self._route_client.call_async(request)
        if not self._wait_future(future, self._server_wait, goal_handle):
            if goal_handle.is_cancel_requested:
                return None, 'Patrol canceled while loading route'
            return None, 'Timed out loading patrol route'
        try:
            response = future.result()
        except Exception as error:
            return None, 'Failed to load route: %s' % error
        if response is None or not response.success:
            detail = response.message if response is not None else 'no response'
            return None, 'Unable to load patrol route: %s' % detail
        return response, ''

    # == 导航 + 终点二次校验 ==
    def _navigate(self, pose, patrol_goal_handle, feedback_callback):
        if not self._nav_client.wait_for_server(timeout_sec=self._server_wait):
            return False, 'NavigateToPose action server is unavailable'
        last_error = 'navigation failed'
        for attempt in range(self._endpoint_retries + 1):
            with self._lock:
                pause_generation = self._pause_generation
            if patrol_goal_handle.is_cancel_requested:
                return False, 'patrol canceled'
            goal = NavigateToPose.Goal()
            goal.pose = pose
            # A patrol pose is a spatial target. Zero stamp asks Nav2/TF for
            # the latest map->base_link transform.
            goal.pose.header.stamp.sec = 0
            goal.pose.header.stamp.nanosec = 0
            distance = {'value': float('nan')}

            def on_feedback(message):
                distance['value'] = float(message.feedback.distance_remaining)
                feedback_callback(distance['value'])

            send_future = self._nav_client.send_goal_async(
                goal, feedback_callback=on_feedback)
            if not self._wait_future(
                    send_future, self._server_wait, patrol_goal_handle):
                self._cancel_pending_navigation(send_future)
                if patrol_goal_handle.is_cancel_requested:
                    return False, 'patrol canceled'
                last_error = 'timed out sending NavigateToPose goal'
                continue
            try:
                nav_goal = send_future.result()
            except Exception as error:
                last_error = 'NavigateToPose goal request failed: %s' % error
                continue
            if nav_goal is None or not nav_goal.accepted:
                last_error = 'NavigateToPose rejected the goal'
                continue
            with self._lock:
                self._active_nav_goal = nav_goal
            result_future = nav_goal.get_result_async()
            finished = self._wait_future(
                result_future, self._waypoint_timeout, patrol_goal_handle,
                stop_on_pause=True)
            if not finished:
                self._cancel_navigation(nav_goal, result_future)
            with self._lock:
                self._active_nav_goal = None
            if not finished:
                if patrol_goal_handle.is_cancel_requested:
                    return False, 'patrol canceled'
                with self._lock:
                    pause_interrupted = (
                        pause_generation != self._pause_generation)
                if self._pause_requested_now() or pause_interrupted:
                    return False, 'paused'
                last_error = 'navigation timed out after %.1fs' % self._waypoint_timeout
                continue
            wrapper = result_future.result()
            with self._lock:
                pause_interrupted = pause_generation != self._pause_generation
            if self._pause_requested_now() or pause_interrupted:
                return False, 'paused'
            if wrapper is None or wrapper.status != GoalStatus.STATUS_SUCCEEDED:
                status = wrapper.status if wrapper is not None else 'unknown'
                last_error = 'NavigateToPose ended with status %s' % status
                continue
            residual = self._stable_endpoint_residual(pose.pose)
            if residual is None:
                last_error = 'no current map-frame pose for endpoint validation'
                continue
            position_error, yaw_error = residual
            if (position_error <= self._position_tolerance and
                    yaw_error <= self._yaw_tolerance):
                self.get_logger().info(
                    'Endpoint verified for %.1fs: position=%.3fm, yaw=%.3frad' % (
                        self._endpoint_stable_duration,
                        position_error,
                        yaw_error,
                    ))
                return True, 'endpoint reached'
            last_error = (
                'endpoint residual too large: position=%.3fm (limit %.3fm), '
                'yaw=%.3frad (limit %.3frad)' % (
                    position_error, self._position_tolerance,
                    yaw_error, self._yaw_tolerance))
            self.get_logger().warning(
                '%s; retry %d/%d' % (
                    last_error, attempt + 1, self._endpoint_retries))
        return False, last_error

    def _endpoint_residual(self, target_pose):
        with self._lock:
            current = self._latest_pose
            received = self._latest_pose_time
            xy_std = self._latest_xy_std
            yaw_std = self._latest_yaw_std
        if current is None or time.monotonic() - received > self._pose_max_age:
            return None
        if (xy_std > self._endpoint_max_xy_std or
                yaw_std > self._endpoint_max_yaw_std):
            self.get_logger().warning(
                'Endpoint localization uncertainty too high: '
                'xy_std=%.3f/%.3fm yaw_std=%.3f/%.3frad' % (
                    xy_std, self._endpoint_max_xy_std,
                    yaw_std, self._endpoint_max_yaw_std,
                ))
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time())
            current_pose = Pose()
            current_pose.position.x = transform.transform.translation.x
            current_pose.position.y = transform.transform.translation.y
            current_pose.position.z = transform.transform.translation.z
            current_pose.orientation = transform.transform.rotation
            return pose_residual(current_pose, target_pose)
        except (TransformException, ValueError) as error:
            self.get_logger().debug(
                'TF endpoint residual unavailable, using AMCL pose: %s' % error)

        try:
            return pose_residual(current.pose.pose, target_pose)
        except ValueError as error:
            self.get_logger().error('Invalid endpoint orientation: %s' % error)
            return None

    def _stable_endpoint_residual(self, target_pose):
        deadline = time.monotonic() + self._endpoint_stable_duration
        max_position_error = 0.0
        max_yaw_error = 0.0
        while True:
            residual = self._endpoint_residual(target_pose)
            if residual is None:
                return None
            max_position_error = max(max_position_error, residual[0])
            max_yaw_error = max(max_yaw_error, residual[1])
            if (max_position_error > self._position_tolerance or
                    max_yaw_error > self._yaw_tolerance):
                return max_position_error, max_yaw_error
            if time.monotonic() >= deadline:
                return max_position_error, max_yaw_error
            time.sleep(0.05)

    def _run_task(self, execution_id, route_name, point_index, point,
                  goal_handle):
        key = (execution_id, point_index)
        event = threading.Event()
        with self._lock:
            self._task_waiters[key] = {'event': event, 'result': None}
        request = PatrolTaskRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.execution_id = execution_id
        request.route_name = route_name
        request.point_index = point_index
        request.point_id = point.point_id
        request.task_id = point.task_id
        request.empty_wait_seconds = point.wait_seconds
        self._task_request_pub.publish(request)

        deadline = time.monotonic() + self._task_timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._task_waiters.pop(key, None)
                return False, 'patrol canceled'
            if self._pause_requested_now():
                if not self._wait_for_resume(
                        goal_handle, self._active_execution_id,
                        route_name, point_index, self._completed_points,
                        self._current_loop):
                    with self._lock:
                        self._task_waiters.pop(key, None)
                    return False, 'patrol canceled'
                with self._lock:
                    completed = self._task_waiters.get(key, {}).get('result')
                if completed is None:
                    self._task_request_pub.publish(request)
                deadline = time.monotonic() + self._task_timeout
            if event.wait(timeout=0.05):
                break
        with self._lock:
            waiter = self._task_waiters.pop(key, None)
        result = waiter['result'] if waiter is not None else None
        if result is None:
            return False, 'task result timed out after %.1fs' % self._task_timeout
        if result.status == PatrolTaskResult.SUCCEEDED:
            return True, result.message
        return False, result.message or 'task failed with status %d' % result.status

    def _wait_future(self, future, timeout, goal_handle, stop_on_pause=False):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if stop_on_pause and self._pause_requested_now():
                return False
            time.sleep(0.02)
        return future.done()

    def _cancel_navigation(self, nav_goal, result_future):
        cancel_future = nav_goal.cancel_goal_async()
        deadline = time.monotonic() + self._cancel_timeout
        while not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not cancel_future.done() or not result_future.done():
            self.get_logger().error(
                'Timed out waiting for NavigateToPose cancellation to finish')

    def _cancel_pending_navigation(self, send_future):
        deadline = time.monotonic() + self._cancel_timeout
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not send_future.done():
            self.get_logger().error(
                'NavigateToPose goal request did not finish during cleanup')
            return
        try:
            nav_goal = send_future.result()
        except Exception as error:
            self.get_logger().warning(
                'NavigateToPose goal request cleanup failed: %s' % error)
            return
        if nav_goal is not None and nav_goal.accepted:
            self._cancel_navigation(nav_goal, nav_goal.get_result_async())

    def _feedback(self, goal_handle, execution_id, state, current_index,
                  completed_points, current_loop, point_id,
                  distance_remaining=float('nan')):
        feedback = ExecutePatrol.Feedback()
        feedback.execution_id = execution_id
        feedback.state = state
        feedback.current_index = current_index
        feedback.completed_points = completed_points
        feedback.current_loop = current_loop
        feedback.point_id = point_id
        feedback.distance_remaining = float(distance_remaining)
        goal_handle.publish_feedback(feedback)

    def _publish_status(self, execution_id, route_name, state, message,
                        current_index=0, completed_points=0,
                        completed_loops=0, point_id=''):
        status = String()
        status.data = json.dumps({
            'stamp_ns': self.get_clock().now().nanoseconds,
            'execution_id': execution_id,
            'route_name': route_name,
            'state': state,
            'message': message,
            'current_index': current_index,
            'completed_points': completed_points,
            'completed_loops': completed_loops,
            'point_id': point_id,
        }, ensure_ascii=True, separators=(',', ':'))
        self._status_pub.publish(status)

    def _finish(self, goal_handle, result, state, message, completed_points,
                completed_loops, execution_id='', route_name=''):
        if not execution_id or not route_name:
            with self._lock:
                execution_id = execution_id or self._active_execution_id
                route_name = route_name or self._active_route_name
        result.success = state == 'SUCCEEDED'
        result.message = message
        result.completed_points = completed_points
        result.completed_loops = completed_loops
        if state == 'SUCCEEDED':
            goal_handle.succeed()
        elif state == 'CANCELED':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self._publish_status(
            execution_id, route_name, state, message,
            completed_points=completed_points,
            completed_loops=completed_loops)
        with self._lock:
            self._patrol_active = False
            self._active_goal_handle = None
            self._active_execution_id = ''
            self._active_route_name = ''
            self._current_index = 0
            self._current_loop = 0
            self._completed_points = 0
            self._completed_loops = 0
            self._pause_requested = False
            self._pause_reason = ''
            self._paused = False
            self._active_nav_goal = None
            self._pause_condition.notify_all()
        return result

    def destroy_node(self):
        self._action_server.destroy()
        self._nav_client.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
