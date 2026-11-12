"""YOLO26n 目标检测节点（含巡检任务响应）。

两套工作模式同时跑：

一、持续检测（给人看）
    1. 订阅摄像头图像话题（sensor_msgs/Image）。
    2. 图像回调只存"最新一帧"，不堆积——CPU 推理慢，旧帧直接丢。
    3. 定时器按固定频率对最新帧做 YOLO 推理，画框后发布带框图像。
    4. 同时把最新检测结果缓存在内存里，供巡检任务取用。

二、巡检任务响应（给巡检系统用）
    巡检 executor 到点后，task_manager 会把点位任务发到 /patrol/task/dispatch。
    本节点如果发现 task_id 是视觉类任务（inpect/yolo/detect 等），就：
    1. 后台线程等一小段时间（让车停稳、YOLO 多跑几帧）；
    2. 取最新检测结果，汇总成一句话（类别+数量+置信度）；
    3. 打包成 PatrolTaskResult 发到 /patrol/task/external_result；
    task_manager 收到后转发给 executor，executor 才推进到下一个点。

为什么巡检处理放后台线程：
    dispatch 回调不能阻塞（会卡住 ROS 回调线程），但我们又需要等车停稳
    再取结果，所以用 threading.Thread 异步处理。
"""

import os
import threading
import time
from collections import Counter

import cv_bridge
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool

# 限制 CPU 推理线程数：torch 默认会贪心占满虚拟机所有核，把 rqt、导航、取流
# 节点的 CPU 抢光，表现就是画面卡顿。这里固定只用 2 个核推理，把其余核让出来。
import torch
torch.set_num_threads(2)
try:
    # inter-op 线程池只允许在并行工作开始前设置一次，重复设置会抛 RuntimeError
    torch.set_num_interop_threads(1)
except RuntimeError:
    pass

from ultralytics import YOLO

# 巡检接口（来自 fishbot_patrol_interfaces 包）
from fishbot_patrol_interfaces.msg import PatrolTaskRequest, PatrolTaskResult


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        # ---------------- 参数：检测相关 ----------------
        self.declare_parameter('model', 'yolo26n.pt')
        self.declare_parameter('image_topic', '/camera/camera/image_raw')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('infer_frequency', 5.0)
        # 带框的图也出一份压缩版：原始标注图一帧 230KB，控制台在另一台机器上，
        # 走 WiFi 会把路由器打瘫。压缩图约 12KB，同样的画面，1/20 的流量。
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 60)
        # 断流保护：超过这么久没收到“新”图像帧，就停止推理并作废缓存结果。
        # 摄像头停止发布后缓存帧会一直留着，若继续反复推理，就会不停刷新
        # _latest_detect_time，让下游以为结果很新（巡检拿冻结画面报成功）。
        self.declare_parameter('stale_image_sec', 2.5)

        # ---------------- 参数：巡检任务相关 ----------------
        # 哪些 task_id 算视觉任务（逗号分隔，大小写不敏感）
        self.declare_parameter('patrol_task_ids', 'inpect,inspect,yolo,detect,vision')
        # 收到巡检任务后，等多少秒再取结果（让车停稳、画面清晰）
        self.declare_parameter('patrol_settle_seconds', 2.0)
        # 标定模式：后台线程不固定等待，而是轮询记录“dispatch→首次检出→
        # 连续3帧稳定检出”的真实耗时，用来确定事件驱动识别窗口的时长
        self.declare_parameter('calibration_mode', False)
        # 事件驱动：启动时是否默认“持续推理”。false=平时不推理、最省 CPU，
        # 仅到点收 dispatch 后短暂开识别窗口；需要一直看画框时设 true
        self.declare_parameter('continuous_default', False)
        # 到点停稳（settle）之外，留给识别的窗口时长（秒）。真机标定正常
        # t_stable≈1.0~1.7s，2.5s 覆盖上限+余量；想调只改参数不用动代码
        self.declare_parameter('patrol_capture_window', 2.5)

        self._model_path = self.get_parameter('model').value
        self._image_topic = self.get_parameter('image_topic').value
        self._conf = float(self.get_parameter('conf_threshold').value)
        self._device = self.get_parameter('device').value
        # 没有可用 CUDA 的机器（虚拟机调试、换机）上 cuda:0 会直接抛异常，
        # 这里自动退回 cpu，而不是让整个视觉节点起不来。
        if str(self._device).startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warning(
                'device=%s 但本机没有可用 CUDA，自动改用 cpu' % self._device)
            self._device = 'cpu'
        self._imgsz = int(self.get_parameter('imgsz').value)
        infer_hz = float(self.get_parameter('infer_frequency').value)
        self._publish_compressed = bool(
            self.get_parameter('publish_compressed').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._stale_image_sec = float(
            self.get_parameter('stale_image_sec').value)

        task_ids_str = str(self.get_parameter('patrol_task_ids').value)
        self._patrol_task_ids = frozenset(
            t.strip().lower() for t in task_ids_str.split(',') if t.strip())
        self._settle_seconds = float(
            self.get_parameter('patrol_settle_seconds').value)
        self._calibration = bool(
            self.get_parameter('calibration_mode').value)
        self._capture_window = float(
            self.get_parameter('patrol_capture_window').value)
        continuous_default = bool(
            self.get_parameter('continuous_default').value)

        self._bridge = CvBridge()

        # ---------------- 加载模型（只加载一次） ----------------
        self.get_logger().info('Loading YOLO model: %s ...' % self._model_path)
        try:
            self._model = YOLO(self._model_path)
        except Exception as error:
            # engine 与 TensorRT/JetPack 版本绑定，换机器或升级系统后常出现
            # “文件在、但加载失败”。此时退回同名 .pt，别让节点直接起不来。
            fallback = os.path.splitext(self._model_path)[0] + '.pt'
            if str(self._model_path).endswith('.engine') \
                    and os.path.exists(fallback):
                self.get_logger().warning(
                    'Engine load failed (%s), falling back to %s'
                    % (error, fallback))
                try:
                    self._model = YOLO(fallback)
                    self._model_path = fallback
                except Exception as error2:
                    self.get_logger().fatal(
                        'Failed to load model %s: %s' % (fallback, error2))
                    raise
            else:
                self.get_logger().fatal(
                    'Failed to load model %s: %s. 请先确认权重文件存在。'
                    % (self._model_path, error))
                raise
        self.get_logger().info('YOLO model loaded, ready.')

        # ---------------- 最新帧 + 最新检测结果缓冲 ----------------
        self._lock = threading.Lock()
        # 推理“忙就跳过”闸门（非阻塞锁）：上一帧没算完，本次 tick 直接跳过，
        # 防止慢推理在可重入回调组里并发重入、CPU 超额订阅导致帧率雪崩
        self._infer_gate = threading.Lock()
        # 事件驱动状态：
        # _continuous_enabled —— 当前是否“持续推理”（可由 set_continuous 服务改）
        # _active_vision_until —— 识别窗口截止的 monotonic 时刻，dispatch 后被
        #   往后推；now 早于它就推理，平时（无巡检、非持续）不推理、CPU≈0
        self._continuous_enabled = continuous_default
        self._active_vision_until = 0.0
        self._latest_image = None
        self._latest_header = None
        self._latest_image_time = 0.0      # 最近一次收到新帧的 monotonic
        # 最新一帧的检测结果：[(class_name, confidence), ...]
        self._latest_detections = []
        self._latest_detect_time = 0.0

        # 巡检任务去重：短时间内同一个(execution_id, point_index)只处理一次
        self._handled_tasks = {}
        # 已经回执过的同一个 key -> (status, report)，重复派发时原样重放
        self._completed_results = {}
        self._handled_lock = threading.Lock()

        # 可重入回调组：让 dispatch 回调和定时器回调能并行
        self._callbacks = ReentrantCallbackGroup()

        # ---------------- 订阅 / 发布：图像检测 ----------------
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, image_qos,
            callback_group=self._callbacks)
        self._annotated_pub = self.create_publisher(
            Image, '~/image_annotated', image_qos)
        # 压缩版的标注图：控制台在另一台机器上，只订阅这个
        self._annotated_compressed_pub = self.create_publisher(
            CompressedImage, '~/image_annotated/compressed', image_qos)

        # ---------------- 订阅 / 发布：巡检任务 ----------------
        # task_manager 用 RELIABLE，这里必须对齐
        reliable = QoSProfile(depth=20)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self._dispatch_sub = self.create_subscription(
            PatrolTaskRequest, '/patrol/task/dispatch',
            self._on_patrol_dispatch, reliable,
            callback_group=self._callbacks)
        self._external_result_pub = self.create_publisher(
            PatrolTaskResult, '/patrol/task/external_result', reliable)

        # ---------------- 服务：手动开/关持续推理 ----------------
        # std_srvs/SetBool：request.data=true 持续推理（一直出画框），
        # false 回到事件驱动（平时不推理、到点才识别）
        self.create_service(
            SetBool, '~/set_continuous', self._on_set_continuous,
            callback_group=self._callbacks)

        # 推理定时器
        self.create_timer(
            1.0 / infer_hz, self._infer_and_publish,
            callback_group=self._callbacks)

        self._frame_count = 0
        self._last_infer_ok = 0.0          # 最近一次成功推理的 monotonic
        self._infer_demand_since = 0.0     # 本轮“被要求推理”的起点
        self.get_logger().info(
            'yolo_detector started, subscribe=%s, conf=%.2f, infer=%.1fHz, '
            'continuous=%s, capture_window=%.1fs, patrol_task_ids=%s'
            % (self._image_topic, self._conf, infer_hz,
               self._continuous_enabled, self._capture_window,
               sorted(self._patrol_task_ids)))

    # ==================================================================
    # 事件驱动：本 tick 到底该不该推理？
    # ==================================================================
    def _should_infer(self) -> bool:
        # 三种情况推理：① 手动开了持续模式；② 标定模式（需全程推理当检测源，
        # 否则 watch 8s 会被识别窗口截断）；③ 现在仍处在识别窗口内。
        # 其余时刻（导航行进中）返回 False，定时器回调立刻退出——只做一次
        # 锁判断（微秒级）、完全不碰 torch，所以 YOLO CPU 占用≈0。
        now = time.monotonic()
        with self._lock:
            return (self._continuous_enabled or self._calibration
                    or now < self._active_vision_until)

    def _on_set_continuous(self, request, response):
        enabled = bool(request.data)
        with self._lock:
            self._continuous_enabled = enabled
        response.success = True
        response.message = 'continuous inference %s' % (
            'enabled' if enabled else 'disabled')
        self.get_logger().info(
            'set_continuous -> %s' % ('ON' if enabled else 'OFF'))
        return response

    # ==================================================================
    # 图像回调：只存最新帧
    # ==================================================================
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as error:
            self.get_logger().warning(
                'cv_bridge convert failed: %s' % error,
                throttle_duration_sec=2.0)
            return
        with self._lock:
            self._latest_image = frame
            self._latest_header = msg.header
            self._latest_image_time = time.monotonic()

    # ==================================================================
    # 推理定时器：取最新帧 -> YOLO -> 画框发布 -> 缓存检测结果
    # ==================================================================
    def _infer_and_publish(self) -> None:
        # 事件驱动总开关：导航行进中、且没开持续模式时，本 tick 不该推理，
        # 立刻返回（微秒级、不碰 torch）——这是“平时 YOLO CPU≈0”的关键。
        # 必须放在闸门 acquire 之前：不该算时连闸门都不必碰。
        if not self._should_infer():
            # 退出“被要求推理”的状态：下次重新开始计时，否则空闲十几秒后
            # 一来任务就会把自己误判成“推理卡住”。
            self._infer_demand_since = 0.0
            return
        if self._infer_demand_since <= 0.0:
            self._infer_demand_since = time.monotonic()
        # 看门狗：闸门被卡死（CUDA/TensorRT 卡住、predict 不返回）时，节点会
        # 静默停摆——还活着、话题还在，就是再也不出框。至少让它日志可见。
        # 基准取“上次成功”和“本次开始被要求推理”的较大者：空闲期本来就不推理，
        # 不能算进静默时长。
        baseline = max(self._last_infer_ok, self._infer_demand_since)
        if baseline > 0.0:
            silent = time.monotonic() - baseline
            # 只有“图像是新鲜的、却没出结果”才算推理卡住；断流时的不出
            # 结果由上面的 stale 分支负责说明，别在这里重复报警。
            frame_fresh = (self._latest_image_time > 0.0 and
                           (time.monotonic() - self._latest_image_time)
                           <= self._stale_image_sec)
            if silent > 5.0 and frame_fresh:
                self.get_logger().error(
                    'No successful inference for %.1fs (推理线程可能卡住)' % silent,
                    throttle_duration_sec=10.0)
        # 忙就跳过闸门：本回调在“可重入”回调组里，若上一帧没推理完，新的
        # tick 会并发发起第二次推理；每次推理 torch 又开多线程，多次推理
        # 并发会造成 CPU 超额订阅、单帧越算越慢的雪崩（实测掉到约 0.08Hz）。
        # 这里用非阻塞锁：拿不到锁说明上一帧还在跑，本次 tick 直接丢弃
        # （图像只缓存最新帧，跳过旧 tick 没有任何副作用）。
        if not self._infer_gate.acquire(blocking=False):
            return
        try:
            self._run_inference_once()
        finally:
            # 无论推理成功还是抛异常都必须释放，否则闸门会永久关闭
            self._infer_gate.release()

    def _run_inference_once(self) -> None:
        # 只对“刚收到的新帧”推理：摄像头断流后缓存帧不会变，继续推理会
        # 刷新 _latest_detect_time，等于把冻结画面包装成新鲜结果。
        now = time.monotonic()
        with self._lock:
            image_age = (now - self._latest_image_time
                         if self._latest_image_time > 0.0 else float('inf'))
        if (self._stale_image_sec > 0.0
                and self._latest_image_time > 0.0
                and image_age > self._stale_image_sec):
            self.get_logger().warning(
                'Camera frames stale (%.1fs since last frame, limit %.1fs): skip inference and invalidate cached detections'
                % (image_age, self._stale_image_sec),
                throttle_duration_sec=5.0)
            with self._lock:
                self._latest_detections = []
                self._latest_detect_time = 0.0
            return
        with self._lock:
            if self._latest_image is None:
                self.get_logger().info(
                    'Waiting for camera image...',
                    throttle_duration_sec=2.0)
                return
            frame = self._latest_image.copy()
            header = self._latest_header

        try:
            results = self._model.predict(
                frame,
                conf=self._conf,
                device=self._device,
                imgsz=self._imgsz,
                verbose=False,
            )
        except Exception as error:
            self.get_logger().warning(
                'Inference failed: %s' % error,
                throttle_duration_sec=3.0)
            return

        result = results[0]
        annotated_bgr = result.plot()

        out_msg = self._bridge.cv2_to_imgmsg(annotated_bgr, encoding='bgr8')
        out_msg.header = header
        self._annotated_pub.publish(out_msg)

        if self._publish_compressed:
            try:
                import cv2 as _cv2
                quality = max(1, min(100, self._jpeg_quality))
                ok, buffer = _cv2.imencode(
                    '.jpg', annotated_bgr,
                    [int(_cv2.IMWRITE_JPEG_QUALITY), quality])
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header = header
                    cmsg.format = 'jpeg'
                    cmsg.data = buffer.tobytes()
                    self._annotated_compressed_pub.publish(cmsg)
            except Exception as error:
                self.get_logger().warning(
                    'JPEG encode failed: %s' % error, throttle_duration_sec=5.0)

        # 解析检测结果并缓存
        names = result.names
        boxes = result.boxes
        detections = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_id = int(box.cls[0])
                conf_val = float(box.conf[0])
                detections.append((names[cls_id], conf_val))

        with self._lock:
            self._latest_detections = detections
            self._latest_detect_time = time.monotonic()

        # 日志
        self._frame_count += 1
        if detections:
            items = ['%s:%.2f' % (name, conf) for name, conf in detections]
            self.get_logger().info(
                '[#%d] %d object(s) -> %s'
                % (self._frame_count, len(detections), ', '.join(items)))
        else:
            self.get_logger().info(
                '[#%d] no object detected' % self._frame_count,
                throttle_duration_sec=5.0)
        self._last_infer_ok = time.monotonic()

    # ==================================================================
    # 巡检任务分发回调
    # ==================================================================
    def _on_patrol_dispatch(self, msg: PatrolTaskRequest) -> None:
        task_id = msg.task_id.strip().lower()

        # 不是视觉任务就忽略，让 task_manager 按空等待处理
        if task_id not in self._patrol_task_ids:
            self.get_logger().debug(
                'Ignoring patrol task %r (not a vision task)' % msg.task_id)
            return

        # 去重：同一个点位任务只处理一次（防止重发导致重复检测）
        dedup_key = (msg.execution_id, int(msg.point_index))
        duplicate = False
        with self._handled_lock:
            if dedup_key in self._handled_tasks:
                duplicate = True
            else:
                self._handled_tasks[dedup_key] = time.monotonic()
                # 顺手清理 5 分钟前的旧记录，防止字典无限涨
                now = time.monotonic()
                expired = [k for k, t in self._handled_tasks.items()
                           if now - t > 300.0]
                for k in expired:
                    self._handled_tasks.pop(k, None)
                    self._completed_results.pop(k, None)
            cached = self._completed_results.get(dedup_key)
        if duplicate:
            # 重复派发**不能**再伪造一条结果：真实的回执已经发出（或马上
            # 就发），多出来的那条会让 task_manager 收到互相打架的两条，
            # 表现为 "ignored unmatched external result"。
            #   - 已经回过执 → 原样重放（幂等，执行器重试时有用）
            #   - 还在处理中 → 什么都不发，等它自己那条真实回执
            if cached is not None:
                self.get_logger().info(
                    'Duplicate patrol task for %s point %d, replaying the '
                    'result already sent (%s)'
                    % (msg.execution_id, msg.point_index, cached[1]))
                self._send_task_result(msg, cached[0], cached[1])
            else:
                self.get_logger().warning(
                    'Duplicate patrol task for %s point %d while it is still '
                    'in progress; the running task will reply, no synthetic '
                    'result sent' % (msg.execution_id, msg.point_index))
            return

        self.get_logger().info(
            'Patrol vision task received: point=%s task=%s, '
            'waiting %.1fs for stable view...'
            % (msg.point_id, msg.task_id, self._settle_seconds))

        # 打开识别窗口：从现在（dispatch）起，覆盖“停稳 settle + 识别
        # capture_window”，这段时间定时器才会真正推理；窗口结束自动回到
        # 近零占用。用 max 是为了和上一个尚未结束的窗口取并集，不被缩短。
        window_end = time.monotonic() + self._settle_seconds + self._capture_window
        with self._lock:
            if window_end > self._active_vision_until:
                self._active_vision_until = window_end

        # 后台线程处理，不阻塞 ROS 回调
        worker = threading.Thread(
            target=self._handle_patrol_task,
            args=(msg,),
            daemon=True,
        )
        worker.start()

    # ==================================================================
    # 巡检任务实际处理（后台线程）：等停稳 -> 取结果 -> 发回执
    # ==================================================================
    def _handle_patrol_task(self, msg: PatrolTaskRequest) -> None:
        if self._calibration:
            # 标定模式：轮询打点，测真实的“到稳定检出”耗时
            self._calibrate_patrol_task(msg)
            return

        # 等车停稳（识别窗口已在 dispatch 回调打开，这段时间 YOLO 已在推理）
        time.sleep(self._settle_seconds)

        # 在 capture_window 内轮询最新结果：拿到“新鲜且非空”的检测就提前用，
        # 最晚到窗口结束取最后一帧兜底。正常 t_stable≈1.0~1.7s，settle 后
        # 基本已检出，所以多数点 settle 一到就立刻推进、不额外耗时。
        detections, age = self._read_latest()
        deadline = time.monotonic() + self._capture_window
        while time.monotonic() < deadline and (age > 1.5 or not detections):
            time.sleep(0.2)
            detections, age = self._read_latest()

        if age > 3.0:
            # 检测结果太旧（可能摄像头断了或推理卡住）
            report = 'vision unavailable: no fresh detection (%.1fs old)' % age
            status = PatrolTaskResult.FAILED
            self.get_logger().warning(
                'Patrol task %s failed: %s' % (msg.point_id, report))
        else:
            report = self._summarize_detections(detections)
            status = PatrolTaskResult.SUCCEEDED
            self.get_logger().info(
                'Patrol task %s result: %s' % (msg.point_id, report))

        self._send_task_result(msg, status, report)

    # ==================================================================
    # 读取最新检测结果及其“年龄”（秒）；从无检测时 age=inf
    # ==================================================================
    def _read_latest(self):
        with self._lock:
            detections = list(self._latest_detections)
            age = (time.monotonic() - self._latest_detect_time
                   if self._latest_detect_time > 0 else float('inf'))
        return detections, age

    # ==================================================================
    # 标定模式（后台线程）：轮询每一帧，记录首次/连续3帧稳定检出耗时
    # ==================================================================
    def _calibrate_patrol_task(self, msg: PatrolTaskRequest) -> None:
        t0 = time.monotonic()
        watch_seconds = 8.0       # 最多观察 8 秒兜底
        poll_interval = 0.1
        stable_need = 3           # 连续 3 帧 top1 同类且过阈值算稳定

        frame_log = []            # [(rel, top_name, top_conf)]
        seen_detect_time = -1.0
        t_first = None
        t_stable = None
        streak = 0
        streak_name = None

        deadline = t0 + watch_seconds
        while time.monotonic() < deadline:
            with self._lock:
                detect_time = self._latest_detect_time
                detections = list(self._latest_detections)

            # 只统计 dispatch 之后产生、且上次没见过的新帧
            if detect_time > t0 and detect_time != seen_detect_time:
                seen_detect_time = detect_time
                rel = detect_time - t0
                if detections:
                    top_name, top_conf = max(detections, key=lambda x: x[1])
                    good = top_conf >= self._conf
                else:
                    top_name, top_conf, good = '', 0.0, False
                frame_log.append((rel, top_name, top_conf))

                if good:
                    if t_first is None:
                        t_first = rel
                    if streak_name == top_name:
                        streak += 1
                    else:
                        streak_name = top_name
                        streak = 1
                    if streak >= stable_need and t_stable is None:
                        t_stable = rel
                else:
                    streak = 0
                    streak_name = None

            if t_stable is not None:
                # 已稳定，补看约 0.6 秒就收尾，不必占满 8 秒
                deadline = min(deadline, time.monotonic() + 0.6)
            time.sleep(poll_interval)

        # 逐帧时间线（看停车后置信度恢复曲线）
        seq = ' | '.join(
            '%+.2f %s %.2f' % (rel, (name or 'none'), conf)
            for rel, name, conf in frame_log)
        self.get_logger().info(
            '[CALIB %s] frames: %s' % (msg.point_id, seq))
        self.get_logger().info(
            '[CALIB %s] t_first=%s t_stable=%s suggested_window=%s'
            % (msg.point_id,
               ('%.2f' % t_first) if t_first is not None else 'NA',
               ('%.2f' % t_stable) if t_stable is not None else 'NA',
               ('%.2f' % (t_stable + 0.5)) if t_stable is not None else 'NA'))

        # 用最新检测状态照常发回执，保证巡检能继续走点
        with self._lock:
            detections = list(self._latest_detections)
            age = time.monotonic() - self._latest_detect_time \
                if self._latest_detect_time > 0 else float('inf')
        if t_stable is None:
            report = 'calib: no stable detection (t_first=%s)' % (
                ('%.2f' % t_first) if t_first is not None else 'NA')
        else:
            report = 'calib ok: %s' % self._summarize_detections(detections)
        # 标定以采集耗时为目的，统一回 SUCCEEDED，避免个别点没检出导致整轮
        # 巡检中断、无法连续采多点样本；真实结论以 [CALIB] 日志为准
        self._send_task_result(msg, PatrolTaskResult.SUCCEEDED, report)

    # ==================================================================
    # 统一打包并发送巡检回执（普通模式 / 标定模式共用）
    # ==================================================================
    def _send_task_result(self, msg, status, report) -> None:
        result_msg = PatrolTaskResult()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.execution_id = msg.execution_id
        result_msg.route_name = msg.route_name
        result_msg.point_index = msg.point_index
        result_msg.point_id = msg.point_id
        result_msg.task_id = msg.task_id
        result_msg.status = status
        result_msg.message = report
        self._external_result_pub.publish(result_msg)
        # 记下来：同一个 key 再来一次派发时，可以原样重放（幂等），
        # 而不是伪造一个新结果——伪造会让 task_manager 收到两条互相
        # 打架的回执（一条 synthetic、一条真实），后到的那条变成
        # "ignored unmatched external result"。
        with self._handled_lock:
            self._completed_results[(msg.execution_id, int(msg.point_index))] = (
                int(status), str(report))

    # ==================================================================
    # 把检测列表汇总成一句话报告
    # 例如："person x2 (0.87,0.76); cell phone x1 (0.65)"
    # ==================================================================
    @staticmethod
    def _summarize_detections(detections) -> str:
        if not detections:
            return 'no object detected'

        counts = Counter(name for name, _ in detections)
        parts = []
        for name, count in counts.items():
            confs = ['%.2f' % conf for n, conf in detections if n == name]
            if count == 1:
                parts.append('%s (%s)' % (name, confs[0]))
            else:
                parts.append('%s x%d (%s)' % (name, count, ','.join(confs)))
        return '; '.join(parts)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    # 回调本身很轻（存帧、派发任务），重推理在定时器里低频跑，2 个线程足够；
    # 线程开太多会和 torch 抢核、增加上下文切换，反而更卡。
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # 收到 SIGTERM 时 rclpy 的信号处理已经关过一次上下文，重复关会抛
        # RCLError，退出时刷一屏 Traceback（纯噪音）。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
