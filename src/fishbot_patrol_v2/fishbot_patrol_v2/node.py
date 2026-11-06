"""巡检 v2 的 ROS 适配层：只做「ROS 消息 ↔ 纯逻辑」的翻译。

职责边界很清楚：

- 走哪条路、失败了怎么办、什么时候停：全部交给 ``plan.Planner``
- 跑到哪了：交给 ``progress`` 落盘，支持断电续跑
- 拍到了什么：交给 ``report`` 落盘成 JSONL + summary
- 本文件只负责：查路线、调 Nav2、收发点位任务、暂停/恢复/抢占、发布状态

接口与 v1 兼容：同样是 ``/execute_patrol`` action、``/patrol/pause`` /
``/patrol/resume`` 服务、``/patrol/task/request`` 话题，换上去外部无感。
"""

import json
import math
import os
import signal
import sys
import threading
import time
import traceback
import uuid

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from fishbot_patrol_interfaces.action import ExecutePatrol
from fishbot_patrol_interfaces.msg import PatrolTaskRequest, PatrolTaskResult
from fishbot_patrol_interfaces.srv import GetRoute, PausePatrol, ResumePatrol

from . import baseline as baseline_module
from . import progress as progress_store
from . import recheck as recheck_module
from . import report as report_module
from .battery import LEVEL_CRIT, LEVEL_UNKNOWN, LEVEL_WARN, BatteryMonitor
from .navigation import NAV_CANCELED, NAV_PAUSED, NAV_PREEMPTED, Navigator
from .plan import (
    VALID_MODES, OUTCOME_ABORT, OUTCOME_RETRY, PlanConfig, Planner, PointRef,
    parse_policy)
from .report import PatrolReport
from .util import (normalize_expect, parse_detections, route_index_of,
                   verdict_of, yaw_of)


class PatrolExecutorV2(Node):
    """巡检 v2 执行节点。"""

    def __init__(self) -> None:
        """声明参数、建好接口。"""
        super().__init__('patrol_executor_v2')

        self.declare_parameter('server_wait_timeout_s', 10.0)
        self.declare_parameter('waypoint_timeout_s', 180.0)
        self.declare_parameter('navigation_cancel_timeout_s', 10.0)
        self.declare_parameter('task_result_timeout_s', 130.0)
        self.declare_parameter('final_position_tolerance_m', 0.30)
        self.declare_parameter('final_yaw_tolerance_rad', 0.60)
        self.declare_parameter('amcl_pose_max_age_s', 5.0)
        self.declare_parameter('image_max_age_s', 5.0)
        self.declare_parameter('endpoint_max_xy_std_m', 0.15)
        self.declare_parameter('endpoint_max_yaw_std_rad', 0.15)
        self.declare_parameter('endpoint_stable_duration_s', 0.5)
        self.declare_parameter('endpoint_require_localization', False)
        self.declare_parameter('use_tf_endpoint', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('failure_policy', 'skip')
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('max_consecutive_failures', 3)
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('resume', True)
        self.declare_parameter('progress_file', '~/.ros/patrol_progress.json')
        self.declare_parameter('report_dir', '~/patrol_reports')
        self.declare_parameter('save_frames', False)
        self.declare_parameter('annotated_image_topic',
                               '/yolo_detector/image_annotated')
        self.declare_parameter('action_name', '/execute_patrol')
        self.declare_parameter('preempt_topic', '/patrol/preempt')
        self.declare_parameter('status_topic', '/patrol/execution/status')
        # ---- 主动视角复核：置信度不高时走近平一点再看一眼 ----
        # 0 = 关闭。默认 0.4 m：目标在画面里约放大一倍，灰区判决基本能落地。
        self.declare_parameter('recheck_forward_m', 0.4)
        self.declare_parameter('recheck_max_rounds', 2)
        self.declare_parameter('recheck_conf_low', 0.25)
        self.declare_parameter('recheck_conf_high', 0.60)
        self.declare_parameter('recheck_on_miss', True)

        # ---- 期望值基线：跑完一趟就把看到的结果记下来，下次自动判定 ----
        # 录点时手写 expect 太烦，多数人不会点，报告里的召回率就永远是 null。
        # 打开后：没有手写 expect 的点位，第 1 趟按实际观察记成基线，第 2 趟起
        # 自动比对，召回率/误报率直接出来。
        self.declare_parameter('learn_baseline', True)
        self.declare_parameter('baseline_file', baseline_module.DEFAULT_PATH)
        # fill = 只补空缺（默认，防止一次误检把基线带偏）；update = 每趟覆盖
        self.declare_parameter('baseline_policy', baseline_module.POLICY_FILL)

        # ---- 电池决策：低电时"跑完当前点就收工" ----
        # 固件在 /battery/voltage 上发电压。这里不做充电管理，只回答一个问题：
        # 这一轮还能不能继续跑。warn 只提示，crit 在当前点结束后中止。
        self.declare_parameter('battery_enabled', True)
        self.declare_parameter('battery_topic', '/battery/voltage')
        self.declare_parameter('battery_warn_v', 11.8)
        self.declare_parameter('battery_crit_v', 11.5)
        # 回差：掉到阈值才报，回到 阈值+回差 才解除，避免电压抖动导致反复告警
        self.declare_parameter('battery_hyst_v', 0.4)
        # 超过这么久没有新电压就当作"不知道"，不拿旧值做决策
        self.declare_parameter('battery_stale_sec', 30.0)
        self.declare_parameter('battery_window', 5)

        self._server_wait = self._positive('server_wait_timeout_s')
        self._waypoint_timeout = self._positive('waypoint_timeout_s')
        self._cancel_timeout = self._positive('navigation_cancel_timeout_s')
        self._task_timeout = self._positive('task_result_timeout_s')
        self._position_tolerance = self._positive('final_position_tolerance_m')
        self._yaw_tolerance = self._positive('final_yaw_tolerance_rad')
        self._pose_max_age = self._positive('amcl_pose_max_age_s')
        self._image_max_age = self._positive('image_max_age_s')
        self._endpoint_max_xy_std = self._positive('endpoint_max_xy_std_m')
        self._endpoint_max_yaw_std = self._positive('endpoint_max_yaw_std_rad')
        self._endpoint_stable = self._non_negative('endpoint_stable_duration_s')
        self._require_localization = bool(
            self.get_parameter('endpoint_require_localization').value)
        self._use_tf_endpoint = bool(self.get_parameter('use_tf_endpoint').value)
        self._global_frame = str(self.get_parameter('global_frame').value).strip()
        self._robot_base_frame = str(
            self.get_parameter('robot_base_frame').value).strip()
        if not self._global_frame or not self._robot_base_frame:
            raise ValueError('global_frame and robot_base_frame must not be empty')
        self._policy = parse_policy(str(self.get_parameter('failure_policy').value))
        self._max_retries = int(self.get_parameter('max_retries').value)
        if self._max_retries < 0:
            raise ValueError('max_retries must not be negative')
        self._max_consecutive = int(
            self.get_parameter('max_consecutive_failures').value)
        if self._max_consecutive < 1:
            raise ValueError('max_consecutive_failures must be at least 1')
        seed = int(self.get_parameter('random_seed').value)
        self._seed = None if seed < 0 else seed
        self._resume = bool(self.get_parameter('resume').value)
        self._progress_file = str(self.get_parameter('progress_file').value)
        self._report_dir = str(self.get_parameter('report_dir').value)
        self._save_frames = bool(self.get_parameter('save_frames').value)
        self._annotated_topic = str(
            self.get_parameter('annotated_image_topic').value)
        action_name = str(self.get_parameter('action_name').value)
        preempt_topic = str(self.get_parameter('preempt_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        self._recheck_forward = self._non_negative('recheck_forward_m')
        self._recheck_rounds = int(self.get_parameter('recheck_max_rounds').value)
        if self._recheck_rounds < 0:
            raise ValueError('recheck_max_rounds must not be negative')
        self._recheck_low = float(self.get_parameter('recheck_conf_low').value)
        self._recheck_high = float(self.get_parameter('recheck_conf_high').value)
        if not 0.0 <= self._recheck_low < self._recheck_high <= 1.0:
            raise ValueError(
                'recheck_conf_low/high must satisfy 0 <= low < high <= 1')
        self._recheck_on_miss = bool(
            self.get_parameter('recheck_on_miss').value)
        self._learn_baseline = bool(self.get_parameter('learn_baseline').value)
        self._baseline_file = str(self.get_parameter('baseline_file').value)
        self._baseline_policy = str(self.get_parameter('baseline_policy').value)
        if self._baseline_policy not in baseline_module.POLICIES:
            raise ValueError('baseline_policy must be one of %s'
                             % (baseline_module.POLICIES,))

        self._callbacks = ReentrantCallbackGroup()
        reliable = QoSProfile(depth=20)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        latched = QoSProfile(depth=20)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._route_client = self.create_client(
            GetRoute, '/patrol_route/get', callback_group=self._callbacks)
        self._task_request_pub = self.create_publisher(
            PatrolTaskRequest, '/patrol/task/request', reliable)
        self._status_pub = self.create_publisher(String, status_topic, latched)
        self.create_subscription(
            PatrolTaskResult, '/patrol/task/result', self._on_task_result,
            latched, callback_group=self._callbacks)
        self.create_subscription(
            String, preempt_topic, self._on_preempt, 10,
            callback_group=self._callbacks)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 20,
            callback_group=self._callbacks)
        if self._save_frames:
            image_qos = QoSProfile(depth=1)
            image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(
                Image, self._annotated_topic, self._on_annotated, image_qos,
                callback_group=self._callbacks)

        # 电压：10 Hz 的 Float32，丢了下一帧就来，BEST_EFFORT 足够
        self._battery_enabled = bool(self.get_parameter('battery_enabled').value)
        self._battery = BatteryMonitor(
            warn_v=float(self.get_parameter('battery_warn_v').value),
            crit_v=float(self.get_parameter('battery_crit_v').value),
            hyst_v=float(self.get_parameter('battery_hyst_v').value),
            window=int(self.get_parameter('battery_window').value),
            stale_sec=float(self.get_parameter('battery_stale_sec').value))
        if self._battery_enabled:
            battery_qos = QoSProfile(depth=1)
            battery_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(
                Float32, str(self.get_parameter('battery_topic').value),
                self._on_battery, battery_qos,
                callback_group=self._callbacks)

        self._lock = threading.RLock()
        self._pause_condition = threading.Condition(self._lock)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_pose = None
        self._latest_pose_time = 0.0
        self._latest_xy_std = math.inf
        self._latest_yaw_std = math.inf
        self._latest_image = None
        self._latest_image_time = 0.0
        self._frame_warned = False
        self._progress_warned = False
        self._task_waiters = {}
        self._pause_requested = False
        self._resume_serial = 0
        self._preempt_payload = None
        self._goal_active = False
        self._execution_id = ''
        self._route_name = ''
        self._state = 'IDLE'
        self._point_id = ''
        self._index = 0
        self._loop = 0
        self._completed_points = 0

        # Ctrl+C / 关机时置位：所有 20 ms 轮询循环立刻退出，executor 才能 join
        self._stop_event = threading.Event()

        self._navigator = Navigator(
            self,
            callback_group=self._callbacks,
            action_name='/navigate_to_pose',
            server_wait_s=self._server_wait,
            waypoint_timeout_s=self._waypoint_timeout,
            cancel_timeout_s=self._cancel_timeout,
            position_tolerance_m=self._position_tolerance,
            yaw_tolerance_rad=self._yaw_tolerance,
            pose_max_age_s=self._pose_max_age,
            pose_provider=self._pose_snapshot,
            max_xy_std_m=self._endpoint_max_xy_std,
            max_yaw_std_rad=self._endpoint_max_yaw_std,
            stable_duration_s=self._endpoint_stable,
            require_localization=self._require_localization,
            stop_check=self._stop_event.is_set)
        # SIGINT 触发 rclpy.shutdown() 时同步置位，不必等 main 的 finally
        self.context.on_shutdown(self._stop_event.set)

        self.create_service(PausePatrol, '/patrol/pause', self._pause_patrol,
                            callback_group=self._callbacks)
        self.create_service(ResumePatrol, '/patrol/resume', self._resume_patrol,
                            callback_group=self._callbacks)
        self._action_server = ActionServer(
            self, ExecutePatrol, action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callbacks)
        self._publish_status('IDLE', 'patrol executor v2 ready')
        self.get_logger().info(
            'patrol_executor_v2 ready (policy=%s, resume=%s)'
            % (self._policy.value, self._resume))

    # -- 参数与基础回调 --------------------------------------------
    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('%s must be finite and greater than zero' % name)
        return value

    def _non_negative(self, name: str) -> float:
        """读取必须 >= 0 的浮点参数。

        与 ``_positive`` 的区别：``endpoint_stable_duration_s`` 允许 0，
        表示「到点后不复核停稳，直接算到达」。
        """
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError('%s must be finite and not negative' % name)
        return value

    def _goal_callback(self, goal_request):
        """合法性检查 + 互斥。

        互斥要在**这里**做，不能只看 ``state``：加载路线期间状态是
        ``LOADING_ROUTE``（最长等两个 10s 超时），只看 RUNNING/PAUSED 的话，
        这段时间发来的第二个 goal 会被接受，两个巡检就会并发跑、互相覆盖
        execution_id。
        """
        if int(goal_request.mode) not in VALID_MODES:
            self.get_logger().warning('rejecting unknown patrol mode')
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_active or self._state in ('RUNNING', 'PAUSED',
                                                    'LOADING_ROUTE',
                                                    'PREEMPTING'):
                self.get_logger().warning('rejecting concurrent patrol goal')
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_amcl_pose(self, message) -> None:
        """收定位：位姿 + 协方差（协方差用来判断这次定位值不值得信）。"""
        covariance = message.pose.covariance
        if len(covariance) < 36:
            return
        xy_variance = max(float(covariance[0]), float(covariance[7]))
        yaw_variance = float(covariance[35])
        if (not math.isfinite(xy_variance) or not math.isfinite(yaw_variance)
                or xy_variance < 0.0 or yaw_variance < 0.0):
            return
        with self._lock:
            self._latest_pose = message.pose.pose
            self._latest_pose_time = time.monotonic()
            self._latest_xy_std = math.sqrt(xy_variance)
            self._latest_yaw_std = math.sqrt(yaw_variance)

    def _on_annotated(self, message) -> None:
        """只留最新一帧标注图，供 ``save_frames`` 落盘。"""
        with self._lock:
            self._latest_image = message
            self._latest_image_time = time.monotonic()

    def _on_battery(self, message) -> None:
        """收电压样本。

        这里只入窗口、不做决策：决策发生在每条点位开始之前，这样“低电”
        只会让「下一件事」不开始，不会在导航途中把车停在走廊中间。
        """
        self._battery.update(getattr(message, 'data', None))

    def _battery_abort_reason(self) -> str:
        """低到临界电压时返回中止原因，否则返回空串。"""
        if not self._battery_enabled:
            return ''
        level = self._battery.level()
        if level == LEVEL_CRIT:
            voltage = self._battery.voltage()
            return 'battery critical (%.2fV), stopping after current point' % (
                voltage if voltage is not None else 0.0)
        if level == LEVEL_WARN:
            self.get_logger().warning(
                'battery low: %s — keep going, but stop near a charger'
                % self._battery.describe(), throttle_duration_sec=30.0)
        return ''

    def _battery_fields(self) -> dict:
        """塞进状态 JSON 的两个字段；没数据时给 None（前端好区分）。"""
        if not self._battery_enabled:
            return {'battery_v': None, 'battery_level': 'disabled'}
        voltage = self._battery.voltage()
        return {
            'battery_v': None if voltage is None else round(voltage, 2),
            'battery_level': self._battery.level(),
        }

    def _save_frame(self, job) -> str:
        """把当前标注图存成 PNG；失败时返回空串，不影响巡检。"""
        if not self._save_frames:
            return ''
        with self._lock:
            message = self._latest_image
            received = self._latest_image_time
        if message is None or time.monotonic() - received > self._image_max_age:
            return ''
        try:
            import cv2
            from cv_bridge import CvBridge
        except ImportError as error:
            if not self._frame_warned:
                self._frame_warned = True
                self.get_logger().error(
                    'save_frames is enabled but %s; images will be skipped' % error)
            return ''
        directory = os.path.join(os.path.expanduser(self._report_dir), 'images')
        os.makedirs(directory, exist_ok=True)
        safe_id = str(job.point_id).replace('/', '_') or 'point'
        path = os.path.join(directory, '%s_%s.png'
                            % (safe_id, time.strftime('%Y%m%d_%H%M%S')))
        try:
            frame = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:  # noqa: BLE001 - 存图失败不该中断巡检
            self.get_logger().warning('failed to convert annotated image: %s' % error)
            return ''
        if not cv2.imwrite(path, frame):
            self.get_logger().warning('failed to write frame to %s' % path)
            return ''
        return path

    def _on_preempt(self, message: String) -> None:
        """收到抢占请求：记下来，主循环在下一个检查点处理。"""
        payload = message.data.strip()
        if not payload:
            return
        with self._lock:
            self._preempt_payload = payload
        self.get_logger().warning('preemption requested: %s' % payload)
        self._publish_status(self._state, 'preemption requested: %s' % payload)

    def _on_task_result(self, message) -> None:
        if message.status not in (PatrolTaskResult.SUCCEEDED,
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

    def _pause_patrol(self, request, response):
        allowed, message = self._accept_control(request.execution_id)
        if not allowed:
            response.success = False
            response.message = message
            return response
        with self._pause_condition:
            self._pause_requested = True
            response.success = True
            response.message = 'patrol paused'
            response.execution_id = self._execution_id
            response.state = 'PAUSED'
            response.current_index = self._index
            response.completed_points = self._completed_points
            response.current_loop = self._loop
        self._publish_status('PAUSED', 'paused by request')
        return response

    def _resume_patrol(self, request, response):
        allowed, message = self._accept_control(request.execution_id)
        if not allowed:
            response.success = False
            response.message = message
            return response
        with self._pause_condition:
            self._pause_requested = False
            self._resume_serial += 1      # 唤醒正在等任务结果的点位逻辑
            self._pause_condition.notify_all()
            response.success = True
            response.message = 'patrol resumed'
            response.execution_id = self._execution_id
            response.state = self._state
            response.current_index = self._index
            response.completed_points = self._completed_points
            response.current_loop = self._loop
        self._publish_status(self._state, 'resumed by request')
        return response

    def _accept_control(self, requested_id):
        """判断 pause/resume 能不能作用在当前这次巡检上。

        不能用 ``_state`` 白名单：主循环大部分时间处于 NAVIGATING /
        WAITING_TASK，按状态判断会让「导航中暂停」被拒成
        ``no patrol is running``。真正的判据是「有没有活着的巡检」，
        也就是 ``_goal_active``。
        """
        with self._lock:
            if not self._goal_active:
                return False, 'no patrol is running'
            if requested_id and requested_id != self._execution_id:
                return False, ('execution id mismatch: active=%r'
                               % self._execution_id)
        return True, ''

    # -- 状态发布 --------------------------------------------------
    def _publish_status(self, state, message, point_id=''):
        with self._lock:
            self._state = state
            if point_id:
                self._point_id = point_id
            payload = {
                'execution_id': self._execution_id,
                'route_name': self._route_name,
                'state': state,
                'message': message,
                'current_index': self._index,
                'completed_points': self._completed_points,
                'current_loop': self._loop,
                'point_id': self._point_id,
                'time': time.time(),
            }
            payload.update(self._battery_fields())
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # -- 主流程 ----------------------------------------------------
    def _execute(self, goal_handle):
        """action 执行入口。

        互斥标志一定要在 finally 里释放：中途抛异常也得让下一个 goal 能进来，
        否则节点会一直拒绝新目标。
        """
        try:
            return self._execute_patrol(goal_handle)
        finally:
            with self._lock:
                self._goal_active = False

    def _execute_patrol(self, goal_handle):
        """真正的巡检主流程。"""
        self._execution_id = uuid.uuid4().hex
        request = goal_handle.request
        with self._lock:
            self._route_name = request.route_name.strip()
            self._completed_points = 0
            self._index = 0
            self._loop = 0
            self._point_id = ''
            # 上一轮残留的抢占请求不能影响这一轮：空闲期收到 /patrol/preempt
            # 也会被记下来，不清理的话本次第一个点位会被莫名改道。
            self._preempt_payload = None
        self._publish_status('LOADING_ROUTE', 'loading patrol route')

        try:
            route = self._load_route(request.route_name, goal_handle)
        except Exception as error:  # noqa: BLE001 - 顶部兜住，保证 action 可复用
            self.get_logger().error('failed to load patrol route: %s' % error)
            return self._finish(goal_handle, False, 'load route failed: %s' % error, 0, 0)
        if route is None:
            return self._finish(goal_handle, False,
                                'patrol canceled while loading route', 0, 0)
        if not route.points:
            return self._finish(goal_handle, False,
                                'route %r has no patrol points' % route.name, 0, 0)
        # route_name 为空时服务端会回落到「当前激活路线」，这里以实际路线名为准，
        # 否则进度文件和报告里会记成空名字，续跑时就对不上了。
        self._route_name = route.name

        config = PlanConfig(
            mode=int(request.mode),
            repeat_count=int(request.repeat_count),
            policy=self._policy,
            max_retries=self._max_retries,
            max_consecutive_failures=self._max_consecutive,
            seed=self._seed)
        points = [PointRef(point_id=item.point_id, task_id=item.task_id,
                           wait_seconds=float(item.wait_seconds))
                  for item in route.points]

        already_done, resume_at = self._resume_point(
            route.name, config, [item.point_id for item in route.points])
        planner = Planner(points, config, already_done=already_done,
                          progress=resume_at)
        report = PatrolReport(self._report_dir, route.name, self._execution_id,
                              config.mode, self._policy.value)
        # 基线一次性读进来：循环里每点再读一次文件既慢又可能读到半截。
        baseline_data = (baseline_module.load(self._baseline_file)
                         if self._learn_baseline else baseline_module.empty())
        learned_points = 0
        if already_done:
            report.event('resume', skipped_jobs=len(already_done),
                         loop=planner.loop_index, cursor=planner.cursor)

        with self._lock:
            self._route_name = route.name
            self._state = 'RUNNING'
        self._publish_status('RUNNING', 'patrol started')

        success, message, completed_normally = True, 'patrol completed', False
        try:
            while True:
                if self._stop_event.is_set():
                    success, message = False, 'patrol canceled (shutdown)'
                    try:
                        self._publish_status('CANCELED', message)
                    except Exception:  # noqa: BLE001 - 关机途中发布可能失败
                        pass
                    break
                if goal_handle.is_cancel_requested:
                    success, message = False, 'patrol canceled'
                    self._publish_status('CANCELED', message)
                    report.event('canceled', completed=len(planner.completed_ids))
                    break
                if not self._gate(goal_handle):
                    # 关机与取消要分开记，否则报告里看不出是哪种中断
                    shutdown = self._stop_event.is_set()
                    success = False
                    message = ('patrol canceled (shutdown)' if shutdown
                               else 'patrol canceled')
                    self._publish_status('CANCELED', message)
                    report.event('canceled', reason=message,
                                 completed=len(planner.completed_ids))
                    break
                job = planner.next_job()
                if job is None:
                    completed_normally = True
                    break
                # 低电检查放在「取到下一个作业、但还没起步」这一刻：上一个点已经
                # 完整做完并落了盘，车停在点位附近（而不是走廊中间），此时中止
                # 最安全，续跑时也能从这个点后面接着来。
                battery_reason = self._battery_abort_reason()
                if battery_reason:
                    success, message = False, battery_reason
                    report.event('battery_abort', reason=battery_reason,
                                 completed=len(planner.completed_ids),
                                 **self._battery_fields())
                    self._publish_status('FAILED', battery_reason)
                    break
                self._handle_preempt(route, goal_handle, report)
                with self._lock:
                    self._index = route_index_of(job)
                    self._loop = job.loop_index
                point = route.points[job.route_index]
                manual_expect = normalize_expect(getattr(point, 'expect', ''))
                # 手写的优先；没写就沿用上一趟学到的基线——这是"不用再选"的关键。
                expect, expect_source = baseline_module.resolve(
                    manual_expect,
                    baseline_module.lookup(baseline_data, route.name,
                                           job.point_id))
                self._publish_status('NAVIGATING',
                                     'navigating to %s' % job.point_id, job.point_id)
                nav_started = time.monotonic()
                ok, reason = self._navigate(point.pose, goal_handle, job)
                nav_seconds = time.monotonic() - nav_started
                if reason in (NAV_PAUSED, NAV_PREEMPTED):
                    continue                      # 当前作业不推进，恢复后重来
                if reason == NAV_CANCELED:
                    success, message = False, 'patrol canceled'
                    self._publish_status('CANCELED', message)
                    break
                if not ok:
                    outcome = planner.report_failure(job, reason)
                    # 导航就没到点：确实“没看到”，所以按 detected=False 判定，
                    # 该是 fn 就记 fn——把导航失败算成“没目标”会让召回虚高。
                    report.point(job, report_module.STATUS_SKIPPED,
                                 nav_seconds=nav_seconds, message=reason,
                                 expect=expect,
                                 verdict=verdict_of(expect, False),
                                 expect_source=expect_source)
                    self._save_progress(planner, route.name, config,
                                        [item.point_id for item in route.points])
                    if outcome == OUTCOME_ABORT:
                        success, message = False, 'aborted: %s' % reason
                        self._publish_status('FAILED', message, job.point_id)
                        report.event('aborted', reason=reason,
                                     consecutive_failures=len(planner.failures))
                        break
                    if outcome == OUTCOME_RETRY:
                        report.event('retry', point_id=job.point_id,
                                     attempt=job.attempt + 1)
                    else:
                        self.get_logger().warning(
                            'skipping %s: %s' % (job.point_id, reason))
                    continue

                self._publish_status('WAITING_TASK',
                                     'waiting for task at %s' % job.point_id,
                                     job.point_id)
                task_started = time.monotonic()
                (ok, task_message, task_interrupted, detections,
                 recheck_info) = self._run_task_with_recheck(job, goal_handle)
                task_seconds = time.monotonic() - task_started
                if task_interrupted:
                    # 取消/关机不是「任务失败」：不记 FAILED、不计入连续失败、
                    # 不推进进度，否则续跑会把这个点当成已经处理过。
                    success, message = False, task_message
                    self._publish_status('CANCELED', message)
                    report.event('canceled', completed=len(planner.completed_ids))
                    break
                if recheck_info['moves']:
                    # 走近复核是「额外的物理动作」，必须留痕：事后对不上
                    # 耗时/路径时，这是唯一能解释「为什么这个点多了 3 秒」的证据。
                    report.event('recheck', point_id=job.point_id,
                                 moves=recheck_info['moves'],
                                 moved_m=round(recheck_info['moved_m'], 3),
                                 top_conf=round(recheck_info['top_conf'], 4),
                                 decision=recheck_info['decision'])
                if not ok:
                    outcome = planner.report_failure(job, task_message)
                    report.point(job, report_module.STATUS_FAILED,
                                 nav_seconds=nav_seconds, task_seconds=task_seconds,
                                 message=task_message, detections=detections,
                                 expect=expect,
                                 verdict=verdict_of(expect, bool(detections)),
                                 expect_source=expect_source)
                    self._save_progress(planner, route.name, config,
                                        [item.point_id for item in route.points])
                    if outcome == OUTCOME_ABORT:
                        success, message = False, 'aborted: %s' % task_message
                        self._publish_status('FAILED', message, job.point_id)
                        report.event('aborted', reason=task_message)
                        break
                    if outcome == OUTCOME_RETRY:
                        report.event('retry', point_id=job.point_id,
                                     attempt=job.attempt + 1)
                    continue

                planner.report_success(job)
                with self._lock:
                    self._completed_points = len(planner.completed_ids)
                image_path = self._save_frame(job)
                report.point(job, report_module.STATUS_OK, nav_seconds=nav_seconds,
                             task_seconds=task_seconds, message=task_message,
                             detections=detections, image_path=image_path,
                             expect=expect,
                             verdict=verdict_of(expect, bool(detections)),
                             expect_source=expect_source)
                # 这条点位没手写期望值、而这趟跑成功了：把它看到的结果记成
                # 基线，下一趟起就能自动判定（不用再回录点界面点一遍）。
                if self._learn_baseline and not manual_expect:
                    if self._learn_point(baseline_data, report, route.name, job,
                                         bool(detections)):
                        learned_points += 1
                self._save_progress(planner, route.name, config)
                self._publish_status('RUNNING', 'reached %s' % job.point_id,
                                     job.point_id)
        except Exception as error:  # noqa: BLE001 - 单点异常不该让 action 挂死
            # 只打一行 str(error) 根本看不出是哪一行炸的（"'bool' object is not
            # callable" 这种信息量几乎为零），这里把完整栈也打出来。
            self.get_logger().error('patrol execution error: %s' % error)
            self.get_logger().error(traceback.format_exc())
            success, message = False, 'internal error: %s' % error
            report.event('error', message=str(error))
        finally:
            try:
                summary_path = report.close({
                    'route_name': route.name,
                    'execution_id': self._execution_id,
                    'aborted': planner.aborted,
                    'skipped': len(planner.skipped),
                    # 这趟新学了多少个期望值基线。>0 说明下一趟起这些点会
                    # 自动判定（报告里就该出召回率了），不是"没填期望值"。
                    'baseline_learned': learned_points,
                    # 收尾时的电量快照：事后判断“这趟是跑完的还是电量撑不住了”
                    # 不用再去翻状态话题的日志
                    **self._battery_fields(),
                })
                self.get_logger().info('patrol report: %s' % summary_path)
            except Exception as error:  # noqa: BLE001 - 收尾不让 goal 挂死
                self.get_logger().error('failed to write patrol report: %s' % error)
            if completed_normally:
                try:
                    progress_store.clear(self._progress_file)
                except Exception as error:  # noqa: BLE001
                    self.get_logger().warning(
                        'failed to clear patrol progress: %s' % error)
        return self._finish(goal_handle, success, message,
                            len(planner.completed_ids),
                            planner.completed_loops)

    def _learn_point(self, data, report, route_name, job, detected):
        """把一个点位的观察记进基线；真的改了才落盘并留痕。

        基线只是个缓存，写坏了不该毁掉这趟巡检——所以这里全兜住，失败只
        告警。下一趟重新学即可。
        """
        changed = baseline_module.learn(
            data, route_name, job.point_id, detected,
            policy=self._baseline_policy, source_run=self._execution_id)
        if not changed:
            return False
        try:
            baseline_module.save(data, self._baseline_file)
        except Exception as error:  # noqa: BLE001 - 见 docstring
            self.get_logger().warning('failed to save expect baseline: %s' % error)
            return False
        expect = (baseline_module.EXPECT_PRESENT if detected
                  else baseline_module.EXPECT_ABSENT)
        report.event('baseline', point_id=job.point_id, expect=expect,
                     policy=self._baseline_policy)
        return True

    def _finish(self, goal_handle, success, message, completed_points, loops):
        """收尾：填结果、改 action 状态、把内部状态复位。"""
        result = ExecutePatrol.Result()
        result.success = bool(success)
        result.message = message
        result.completed_points = int(completed_points)
        result.completed_loops = int(loops)
        canceled = bool(goal_handle.is_cancel_requested)
        final_state = 'CANCELED' if canceled else ('SUCCEEDED' if success else 'FAILED')
        with self._lock:
            self._pause_requested = False
            self._preempt_payload = None
        self._publish_status(final_state, message)
        if canceled:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        # _goal_active 必须最后放开：如果先放开，下个 goal 会在
        # 「终态已发布、_state 还没复位」的窗口里被 _goal_callback 接受，
        # 两个 _execute_patrol 并发跑、互相覆盖 execution_id。
        with self._lock:
            self._state = 'IDLE'
            self._goal_active = False
        return result

    # -- 断点续跑 --------------------------------------------------
    def _resume_point(self, route_name, config, point_ids=()):
        """读取上次进度；能续就返回（已完成作业, 游标）。"""
        if not self._resume:
            return (), None
        saved = progress_store.load(self._progress_file)
        if not progress_store.resumable(saved, route_name, config.mode,
                                        config.repeat_count, point_ids):
            if saved is not None:
                # 路线被改过（增删/重排点）：旧游标是「下标」语义，
                # 拿来续跑会静默跳过没巡检的点，宁可从头跑。
                self.get_logger().warning(
                    'patrol progress discarded: route %r changed since last run'
                    % route_name)
            return (), None
        self.get_logger().info(
            'resuming patrol: loop=%d cursor=%d completed=%d'
            % (saved.loop, saved.cursor, len(saved.completed)))
        return tuple(saved.completed), {'loop': saved.loop,
                                        'cursor': saved.cursor,
                                        'order': list(saved.order)}

    def _save_progress(self, planner, route_name, config, point_ids=()):
        """每个点之后存一次进度，断电最多丢一个点。

        存不下（磁盘满/只读挂载）只该失去续跑能力，不该把整趟巡检打断。
        """
        snapshot = planner.snapshot()
        saved = progress_store.save(self._progress_file, progress_store.Progress(
            route_name=route_name,
            mode=config.mode,
            repeat_count=config.repeat_count,
            loop=snapshot['loop'],
            cursor=snapshot['cursor'],
            order=list(snapshot.get('order') or []),
            points=[str(item) for item in point_ids],
            completed=planner.completed_ids,
            skipped=[job.job_id for job in planner.skipped]))
        if not saved and not self._progress_warned:
            self._progress_warned = True
            self.get_logger().error(
                'failed to save patrol progress (%s); resume will not work'
                % self._progress_file)

    # -- 暂停 / 抢占 -----------------------------------------------
    def _gate(self, goal_handle) -> bool:
        """统一等待门：暂停时在这里阻塞；返回 False 表示该退出。"""
        while True:
            if self._stop_event.is_set():
                return False
            if goal_handle.is_cancel_requested:
                return False
            with self._pause_condition:
                if not self._pause_requested:
                    return True
                self._pause_condition.wait(timeout=0.1)

    def _handle_preempt(self, route, goal_handle, report) -> None:
        """处理一次抢占：记录，并在 payload 是点位 id 时先去那里。"""
        with self._lock:
            payload = self._preempt_payload
            self._preempt_payload = None
        if payload is None:
            return
        report.event('preempt', payload=payload)
        target = None
        for point in route.points:
            if point.point_id == payload:
                target = point
                break
        if target is None:
            self.get_logger().warning(
                'preemption payload %r is not a patrol point; recording only'
                % payload)
            return
        self._publish_status('PREEMPTING', 'diverting to %s' % target.point_id,
                             target.point_id)
        ok, reason = self._navigate(target.pose, goal_handle, None)
        report.event('preempt_nav', point_id=target.point_id, ok=bool(ok),
                     reason=reason)

    # -- 路线与导航 ------------------------------------------------
    def _load_route(self, route_name, goal_handle):
        if not self._route_client.wait_for_service(timeout_sec=self._server_wait):
            raise RuntimeError('route manager service unavailable')
        request = GetRoute.Request()
        request.route_name = str(route_name)
        future = self._route_client.call_async(request)
        if not self._wait_future(future, self._server_wait, goal_handle):
            if goal_handle.is_cancel_requested:
                return None
            raise RuntimeError('timed out loading patrol route')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('route %r unavailable: %s'
                               % (route_name,
                                  getattr(response, 'message', 'unknown error')))
        return response.route

    def _navigate(self, pose, goal_handle, job):
        """导航到目标点；返回 ``(ok, reason)``。"""
        if job is None:
            feedback_cb = None
        else:
            def feedback_cb(distance):
                self._feedback(goal_handle, job, distance)

        return self._navigator.navigate(
            pose,
            # is_cancel_requested 是属性不是方法，必须包一层，否则导航层
            # 调用 cancel_check() 时会抛 "'bool' object is not callable"。
            cancel_check=lambda: goal_handle.is_cancel_requested,
            interrupt_check=self._interrupt_reason,
            feedback_cb=feedback_cb)

    def _interrupt_reason(self):
        """暂停 / 抢占检查，导航层每 20 ms 问一次。"""
        if self._pause_requested_now():
            return NAV_PAUSED
        if self._preempt_pending():
            return NAV_PREEMPTED
        return None

    def _pose_snapshot(self):
        """给导航层用的定位快照：``(pose, 距今秒数, xy_std, yaw_std)``。

        优先用 TF：``map->base_link`` 就是 Nav2 自己用的那份变换，比 AMCL 话题
        更贴近「机器人此刻在哪」；TF 查不到时退回 AMCL 位姿。两个都没有就返回
        ``None``，让导航层按策略决定是跳过校验还是判失败。
        """
        with self._lock:
            pose = self._latest_pose
            received = self._latest_pose_time
            xy_std = self._latest_xy_std
            yaw_std = self._latest_yaw_std
        amcl_age = (time.monotonic() - received) if pose is not None else math.inf
        transform = self._tf_snapshot()
        if transform is not None:
            tf_pose, tf_age = transform
            return tf_pose, tf_age, xy_std, yaw_std
        if pose is None:
            return None, math.inf, xy_std, yaw_std
        return pose, amcl_age, xy_std, yaw_std

    def _tf_snapshot(self):
        """查一次 ``map->base_link``；返回 ``(pose, 距今秒数)`` 或 ``None``。"""
        if not self._use_tf_endpoint:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time())
        except (TransformException, ValueError):
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        age = 0.0
        stamp = (float(transform.header.stamp.sec) +
                 float(transform.header.stamp.nanosec) * 1e-9)
        if stamp > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            age = now - stamp
            if not math.isfinite(age) or age < 0.0 or age > 3600.0:
                age = 0.0          # 时钟没对齐就当它是新的，别误判成过期
        return pose, age

    def _wait_future(self, future, timeout, goal_handle):
        """等一个普通 service future（导航的等待在 ``navigation.py`` 里）。"""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False
            time.sleep(0.02)
        return future.done()



    def _pause_requested_now(self) -> bool:
        with self._lock:
            return self._pause_requested

    def _resume_serial_now(self) -> int:
        with self._lock:
            return self._resume_serial

    def _preempt_pending(self) -> bool:
        with self._lock:
            return self._preempt_payload is not None

    # -- 点位任务 --------------------------------------------------
    def _run_task(self, job, goal_handle, attempt=None):
        """把点位任务派给任务管理器并等结果；返回 ``(ok, message, interrupted)``。

        ``interrupted`` 表示这次是因为取消/关机而中断，不属于「任务失败」：
        不能记成 FAILED、不能计入连续失败、也不能把进度推进掉，否则续跑会
        直接跳过这个点。
        """
        # 任务管理器按 (execution_id, point_index) 去重，所以每轮、每次重试
        # 都要换一个 id：同一个 id 重发会被当成「重复请求」直接回放上一次的
        # 结果，重试就变成了假重试（失败点甚至会被回放成成功）。
        # `attempt` 允许调用方覆盖：走近复核每挪一次就要换一个新 id，否则
        # 第二次请求会被当成重复请求回放上一轮的结果——「再检」就白检了。
        task_attempt = job.attempt if attempt is None else int(attempt)
        task_execution_id = '%s-L%d-A%d' % (self._execution_id, job.loop_index,
                                            task_attempt)
        key = (task_execution_id, job.route_index)
        event = threading.Event()
        request = PatrolTaskRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.execution_id = task_execution_id
        request.route_name = self._route_name
        request.point_index = job.route_index
        request.point_id = job.point_id
        request.task_id = job.task_id
        request.empty_wait_seconds = job.wait_seconds
        deadline = time.monotonic() + self._task_timeout
        serial = self._resume_serial_now()
        try:
            # 注册必须和发送放在同一个 try 里：publish 抛异常时 finally 才能
            # 把 waiter 清掉，否则 _task_waiters 会永久泄漏一个 Event。
            with self._lock:
                self._task_waiters[key] = {'event': event, 'result': None}
            self._task_request_pub.publish(request)
            while True:
                if self._stop_event.is_set():
                    return False, 'patrol canceled (shutdown)', True
                if goal_handle.is_cancel_requested:
                    return False, 'patrol canceled', True
                gate_started = time.monotonic()
                if not self._gate(goal_handle):
                    if self._stop_event.is_set():
                        return False, 'patrol canceled (shutdown)', True
                    return False, 'patrol canceled', True
                deadline += time.monotonic() - gate_started   # 暂停不占用任务预算
                current = self._resume_serial_now()
                if current != serial:
                    serial = current
                    with self._lock:
                        waiter = self._task_waiters.get(key)
                    if waiter is not None and waiter['result'] is None:
                        # 暂停期间任务管理器可能已经把请求丢了，恢复后重发一次，
                        # 否则只能干等到超时
                        request.header.stamp = self.get_clock().now().to_msg()
                        self._task_request_pub.publish(request)
                        deadline = time.monotonic() + self._task_timeout
                if time.monotonic() >= deadline:
                    break
                if event.wait(timeout=0.05):
                    break
            with self._lock:
                waiter = self._task_waiters.get(key)
            result = waiter['result'] if waiter is not None else None
            if result is None:
                return (False, 'task result timed out after %.1fs'
                        % self._task_timeout, False)
            if result.status == PatrolTaskResult.SUCCEEDED:
                return True, result.message, False
            return (False, result.message or ('task failed with status %d'
                                              % result.status), False)
        finally:
            with self._lock:
                self._task_waiters.pop(key, None)

    # ------------------------------------------------------------------
    # 主动视角复核：不确定就往前走一点，再看一眼
    # ------------------------------------------------------------------
    def _advance_for_recheck(self, goal_handle, job):
        """沿当前朝向往前挪 ``recheck_forward_m``，返回 ``(ok, reason)``。

        不直接用路线点的坐标：车实际停在哪、朝哪，只有 /amcl_pose 知道。
        以「当前位置 + 朝向 × 距离」重新算目标，才不会因为到点误差把车
        送去一个偏掉 30 cm 的点。
        """
        pose, _age, _count, _stamp = self._pose_snapshot()
        if pose is None:
            return False, 'no pose available for recheck'
        yaw = yaw_of(pose)
        target = PoseStamped()
        target.header.frame_id = self._global_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = (pose.position.x
                                  + self._recheck_forward * math.cos(yaw))
        target.pose.position.y = (pose.position.y
                                  + self._recheck_forward * math.sin(yaw))
        target.pose.position.z = 0.0
        target.pose.orientation = pose.orientation     # 朝向不变，只前进
        return self._navigate(target, goal_handle, job)

    def _run_task_with_recheck(self, job, goal_handle):
        """跑点位任务；结果落在灰区就往前走一点点再看一眼。

        返回 ``(ok, message, interrupted, detections, info)``，``info`` 里带
        ``moves``（走了几次）、``moved_m``（累计前进距离）、``top_conf``。

        三种触发条件（见 ``recheck.evaluate`` 的决策表）：
        1. 有目标但置信度在 [low, high) —— 说不清是灭火器还是红色瓶子；
        2. 干脆没检到 —— 可能太远、太小、被挡住；
        3. 置信度低于 low —— 当没检到处理。
        已经置信度 >= high 就不要再折腾，免得每个点都多花 3 秒。
        """
        ok, message, interrupted = self._run_task(job, goal_handle)
        info = {'moves': 0, 'moved_m': 0.0, 'top_conf': 0.0,
                'decision': recheck_module.DECISION_ACCEPT_ABSENT}
        if interrupted or not ok:
            return ok, message, interrupted, parse_detections(message), info
        if self._recheck_forward <= 0.0 or self._recheck_rounds <= 0:
            info['decision'] = recheck_module.DECISION_ACCEPT_PRESENT
            return ok, message, interrupted, parse_detections(message), info

        parsed = recheck_module.parse_summary(message)
        if parsed is None:
            # 结果格式不认识（比如以后改成 JSON）：老实按原结果收，别乱走。
            info['decision'] = recheck_module.DECISION_ACCEPT_PRESENT
            return ok, message, interrupted, parse_detections(message), info

        rounds = [parsed]
        count, confidences = parsed
        while True:
            decision, top = recheck_module.evaluate(
                count, confidences, info['moves'], self._recheck_rounds,
                self._recheck_low, self._recheck_high, self._recheck_on_miss)
            info['top_conf'] = top
            info['decision'] = decision
            if decision != recheck_module.DECISION_MOVE:
                break
            moved, reason = self._advance_for_recheck(goal_handle, job)
            if not moved:
                # 挪不动（定位丢了 / 被取消 / 前面有障碍）：按已有结果收，
                # 不要把一次「看不准」升级成「这个点失败」。
                self.get_logger().warning(
                    'recheck at %s skipped: %s' % (job.point_id, reason))
                if reason == NAV_CANCELED:
                    return False, 'patrol canceled', True, [], info
                break
            info['moves'] += 1
            info['moved_m'] = self._recheck_forward * info['moves']
            ok, message, interrupted = self._run_task(
                job, goal_handle, attempt=job.attempt + info['moves'])
            if interrupted:
                return ok, message, True, [], info
            if not ok:
                # 走近之后反而失败了：保留上一轮的结论，别把点判死。
                break
            parsed = recheck_module.parse_summary(message)
            if parsed is None:
                break
            count, confidences = parsed
            rounds.append(parsed)

        _count, _confs, top = recheck_module.merge(rounds)
        info['top_conf'] = top
        if info['moves']:
            message = '%s [recheck: +%.2fm x%d, top=%.2f]' % (
                message, info['moved_m'], info['moves'], top)
        return True, message, False, parse_detections(message), info

    def _feedback(self, goal_handle, job, distance):
        feedback = ExecutePatrol.Feedback()
        feedback.execution_id = self._execution_id
        feedback.state = 'NAVIGATING'
        feedback.current_index = route_index_of(job)
        feedback.completed_points = self._completed_points
        feedback.current_loop = job.loop_index + 1
        feedback.point_id = job.point_id
        feedback.distance_remaining = float(distance)
        goal_handle.publish_feedback(feedback)


    # -- 退出 ------------------------------------------------------
    def stop(self):
        """置位退出标志并尽力取消当前导航目标（Ctrl+C / 关机路径调用）。

        只 set 标志并不会让 Nav2 放弃目标，车会继续开到终点；必须显式 cancel。
        """
        self._stop_event.set()
        try:
            self._navigator.abort()
        except Exception as error:  # noqa: BLE001 - 退出路径不能再抛
            self.get_logger().debug('abort on shutdown failed: %s' % error)


def _install_signal_handlers(node):
    """接管 SIGINT/SIGTERM：先撤导航目标，再关 rclpy。

    rclpy 默认的处理是直接 ``rclpy.shutdown()``，此时 context 已经失效，
    再发 cancel 请求根本发不出去，车会一直开到目标点。所以顺序必须是
    「先 abort（context 还活着）→ 再 shutdown」。
    """
    def handler(signum, frame):
        del frame
        node.get_logger().info('signal %d received, cancelling navigation' % signum)
        node.stop()
        rclpy.shutdown()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):      # 非主线程 / 平台不支持
            pass


def main(args=None):
    """启动节点。"""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = PatrolExecutorV2()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        _install_signal_handlers(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - 参数非法也要正常收尾
        print('patrol_executor_v2 failed to start: %s' % error, file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.stop()            # 先取消导航目标，车才会停
        if executor is not None:
            # 必须先 join 掉执行线程再销毁节点：execute 线程可能还卡在
            # _navigate/_run_task 里，destroy_node 之后它再 publish 会抛
            # InvalidHandle，甚至把真实退出码盖掉。
            try:
                executor.shutdown(timeout_sec=5.0)
            except Exception:  # noqa: BLE001 - 老版本 rclpy 没有 timeout 参数
                try:
                    executor.shutdown()
                except Exception:  # noqa: BLE001
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
