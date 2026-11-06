#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 侧的全部 I/O：订阅、调用服务、发动作、把图编码成 JPEG。

只有这一个模块 import rclpy —— 好处是别的模块都能在本机单测，
坏处是这一层必须对"某个包没装""消息类型不同"极度宽容：
nav2 / slam_toolbox / camera 任何一个缺失，都只是少一个功能，
不能让控制台起不来。
"""
import json
import math
import os
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Float32, String

from . import map_render
from . import data
from .data import LogRing, normalize_status

try:                                        # 点云要投到地图坐标系，靠 TF
    import tf2_ros
    from rclpy.duration import Duration
except Exception:                           # pragma: no cover
    tf2_ros = None
    Duration = None

try:                                        # 巡检动作，缺了只是不能发巡检
    from fishbot_patrol_interfaces.action import ExecutePatrol
except Exception:                           # pragma: no cover
    ExecutePatrol = None

try:                                        # nav2 的导航动作
    from nav2_msgs.action import NavigateToPose
except Exception:                           # pragma: no cover
    NavigateToPose = None

try:                                        # 相机单张抓拍服务
    from std_srvs.srv import Trigger
except Exception:                           # pragma: no cover
    Trigger = None

try:                                        # 取路线（只为显示名字/点名）
    from fishbot_patrol_interfaces.srv import GetRoute
except Exception:                           # pragma: no cover
    GetRoute = None

try:                                        # 录点（点位化巡检的入口）
    from fishbot_patrol_interfaces.srv import RecordPoint
except Exception:                           # pragma: no cover
    RecordPoint = None

try:                                        # 删点（录错了要能自己收拾）
    from fishbot_patrol_interfaces.srv import DeletePoint
except Exception:                           # pragma: no cover
    DeletePoint = None

try:                                        # 路线点位在 RViz 里显示
    from visualization_msgs.msg import Marker, MarkerArray
except Exception:                           # pragma: no cover
    Marker = None
    MarkerArray = None


class ConsoleBridge(Node):
    def __init__(self):
        super().__init__('fishbot_console')
        self.declare_parameter('status_topic', '/patrol/execution/status')
        self.declare_parameter('image_topic', '/camera/camera/image_raw')
        self.declare_parameter('annotated_topic', '/yolo_detector/image_annotated')
        # 压缩图：原始图一帧 230KB，5fps 要 9Mbps，会把 WiFi 路由器打瘫。
        # 视觉节点另外发一份 JPEG（约 12KB/帧），这里优先订阅它。
        # 没装新版视觉节点时把 prefer_compressed 设为 false，退回订阅原始图。
        self.declare_parameter('prefer_compressed', True)
        self.declare_parameter('compressed_image_topic',
                               '/camera/camera/image_raw/compressed')
        self.declare_parameter('compressed_annotated_topic',
                               '/yolo_detector/image_annotated/compressed')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_max_points', 400)
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('robot_length', 0.22)
        self.declare_parameter('robot_width', 0.18)
        self.declare_parameter('battery_topic', '/battery/voltage')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('teleop_timeout_sec', 0.6)
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('marker_topic', '/console/route_points')
        self.declare_parameter('record_service', '/patrol_route/record_point')
        self.declare_parameter('delete_service', '/patrol_route/delete_point')
        self.declare_parameter('patrol_action', '/execute_patrol')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('route_service', '/patrol_route/get')
        self.declare_parameter('capture_service', '/camera/camera/capture')
        self.declare_parameter('reports_dir', '~/patrol_reports')
        self.declare_parameter('battery_warn_v', 11.8)
        self.declare_parameter('battery_crit_v', 11.5)
        self.declare_parameter('max_log_lines', 500)
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8080)
        # 一键启停导航/建图时是否顺带拉起 RViz。弱虚拟机上 RViz 会抢 CPU，
        # 让 Nav2 的 20 Hz 控制循环抖动（现象：cmd_vel 内容对，车一顿一顿）。
        self.declare_parameter('stacks_rviz', True)
        # 日志里给人看的地址。留空则自动探测；0.0.0.0 是绑定语义，
        # 不能让用户拿它去点浏览器。
        self.declare_parameter('public_host', '')
        self.declare_parameter('ai_base_url', '')
        self.declare_parameter('ai_api_key', '')
        self.declare_parameter('ai_model', '')

        self._warn_v = float(self.get_parameter('battery_warn_v').value)
        self._crit_v = float(self.get_parameter('battery_crit_v').value)
        self._reports_dir = os.path.expanduser(
            str(self.get_parameter('reports_dir').value))
        # 这两个是遥控链路上的关键参数：漏赋值不是"默认值"而是 AttributeError，
        # 而且发生在 __init__ 里，表现为整个控制台一起退出。
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._teleop_timeout_sec = float(
            self.get_parameter('teleop_timeout_sec').value)

        self._lock = threading.Lock()
        self._status_raw = ''
        # 最近一条状态消息的时刻，以及"用户按过停止"的时刻。后者晚于前者，
        # 就说明执行器还没发出新的状态，但取消请求已经生效——页面不能继续
        # 显示"运行中"（真机上就是"停了但面板还说在跑"）。
        self._status_at = 0.0
        self._patrol_stop_at = 0.0
        self._battery_v = None
        self._pose = None
        self._map_png = b''
        self._map_meta = {}
        self._frame_jpeg = b''
        self._frame_time = 0.0
        self._anno_jpeg = b''
        self._anno_time = 0.0
        self._last_event = None
        self._global_cost = None            # (data, meta)
        self._local_cost = None
        self._layers_png = b''
        self._layers_version = 0            # 每次代价地图更新 +1
        self._layers_built = -1             # 上次成图时的版本号（缓存依据）
        # 单个代价地图各自成图、各自定位。合成一张会让"局部代价地图的移动窗口"
        # 跟着车在地图上滑动，看起来就是地图在漂。
        self._cost_version = {'global': 0, 'local': 0}
        self._cost_png = {'global': b'', 'local': b''}
        self._cost_built = {'global': -1, 'local': -1}
        self._scan_points = []              # [(x, y)]，已投到 map 坐标系
        self._scan_time = 0.0
        self._scan_max = int(self.get_parameter('scan_max_points').value)
        self._plan_points = []              # nav2 全局路径（map 系）
        self._plan_time = 0.0
        self._robot_length = float(self.get_parameter('robot_length').value)
        self._robot_width = float(self.get_parameter('robot_width').value)

        self._tf_buffer = None
        self._tf_listener = None
        if tf2_ros is not None:
            try:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(
                    self._tf_buffer, self, spin_thread=False)
            except Exception:               # pragma: no cover
                self._tf_buffer = None

        self.logs = LogRing(capacity=int(self.get_parameter('max_log_lines').value))
        self._callbacks = ReentrantCallbackGroup()

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=2)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            String, self.get_parameter('status_topic').value,
            self._on_status, status_qos, callback_group=self._callbacks)
        self.create_subscription(
            Float32, self.get_parameter('battery_topic').value,
            self._on_battery, 10, callback_group=self._callbacks)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, map_qos, callback_group=self._callbacks)
        if self.get_parameter('prefer_compressed').value:
            # 只订阅压缩图：原始图根本不过网，空口从 9Mbps 降到 ~0.2Mbps
            self.create_subscription(
                CompressedImage,
                self.get_parameter('compressed_image_topic').value,
                self._on_image_compressed, fast_qos,
                callback_group=self._callbacks)
            self.create_subscription(
                CompressedImage,
                self.get_parameter('compressed_annotated_topic').value,
                self._on_annotated_compressed, fast_qos,
                callback_group=self._callbacks)
        else:
            self.create_subscription(
                Image, self.get_parameter('image_topic').value,
                self._on_image, fast_qos, callback_group=self._callbacks)
            self.create_subscription(
                Image, self.get_parameter('annotated_topic').value,
                self._on_annotated, fast_qos, callback_group=self._callbacks)
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter('pose_topic').value,
            self._on_pose, 10, callback_group=self._callbacks)
        self.create_subscription(Log, '/rosout', self._on_log, 200,
                                 callback_group=self._callbacks)


        # 手动给初始位姿：和 RViz 的 "2D Pose Estimate" 发的是同一个话题、同一套协方差
        self._teleop_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._teleop_lock = threading.Lock()
        self._teleop_vx = 0.0
        self._teleop_wz = 0.0
        self._teleop_seen = 0.0
        self.create_timer(0.1, self._teleop_tick)
        # 急停状态：一旦按下，所有会动的入口（手动/导航/巡检/AI 动作）都被拦住，
        # 直到显式解除。放在这儿而不是页面里，是因为页面可能卡住或网络断掉，
        # 而"不让车动"这件事必须由服务端保证。
        self._estop_lock = threading.Lock()
        self._estop_active = False
        self._estop_since = 0.0
        self._initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self._patrol_client = None
        if ExecutePatrol is not None:
            self._patrol_client = ActionClient(
                self, ExecutePatrol, self.get_parameter('patrol_action').value,
                callback_group=self._callbacks)
        self._nav_client = None
        if NavigateToPose is not None:
            self._nav_client = ActionClient(
                self, NavigateToPose, self.get_parameter('nav_action').value,
                callback_group=self._callbacks)

        self._route_client = None
        if GetRoute is not None:
            self._route_client = self.create_client(
                GetRoute, self.get_parameter('route_service').value,
                callback_group=self._callbacks)
        self._capture_client = None
        if Trigger is not None:
            self._capture_client = self.create_client(
                Trigger, self.get_parameter('capture_service').value,
                callback_group=self._callbacks)

        self._nav_goal_handle = None
        self._patrol_goal_handle = None
        self._snapshot_dir = os.path.expanduser('~/console_snapshots')
        self._ssh = os.environ.get('CONSOLE_SSH', '')

        # 路线点位 marker：只在 RViz 看，所以这里只发、不画。
        # transient_local 是必要的——RViz 常常在控制台之后才打开，用 volatile
        # 的话它永远收不到已经发过的那一帧。
        self._marker_pub = None
        if MarkerArray is not None:
            self._marker_pub = self.create_publisher(
                MarkerArray, self.get_parameter('marker_topic').value,
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST, depth=1))
        # 录点：直接调 route_manager 的 record_point，服务端记一笔
        self._record_client = None
        if RecordPoint is not None:
            self._record_client = self.create_client(
                RecordPoint, self.get_parameter('record_service').value,
                callback_group=self._callbacks)
        # 删点：同样是 route_manager 的活，控制台不自己改 json
        self._delete_client = None
        if DeletePoint is not None:
            self._delete_client = self.create_client(
                DeletePoint, self.get_parameter('delete_service').value,
                callback_group=self._callbacks)

    # ------------------------------------------------------------ 回调
    def _on_status(self, msg):
        with self._lock:
            self._status_raw = msg.data
            self._status_at = time.time()

    def _on_battery(self, msg):
        value = float(msg.data)
        if value == value and value > 0.0:
            with self._lock:
                self._battery_v = value

    def _on_map(self, msg):
        try:
            png = map_render.grid_to_png(msg.data, msg.info.width,
                                         msg.info.height)
        except Exception as error:          # 地图坏了不能影响别的
            self.get_logger().warning('render map failed: %s' % error)
            return
        meta = map_render.map_meta(msg.info)
        with self._lock:
            self._map_png = png
            self._map_meta = meta

    # ---------------------------------------------------------- 代价地图与点云
    def _store_costmap(self, msg, slot):
        """代价地图只留最新的原始数组 + 几何，成图推迟到有人要看时再做。

        刻意不在这里做逐格转换：局部代价地图 5~10 Hz，几百 k 格转一遍就是
        几十毫秒，白烧 CPU。渲染时再按行跳过"整行都没代价"的情况。
        """
        try:
            data = msg.data
            width = int(msg.info.width)
            height = int(msg.info.height)
        except Exception:
            return
        if width <= 0 or height <= 0 or len(data) < width * height:
            return
        meta = map_render.map_meta(msg.info)
        with self._lock:
            if slot == 'global':
                self._global_cost = (data, meta)
            else:
                self._local_cost = (data, meta)
            self._layers_version += 1
            if slot in self._cost_version:
                self._cost_version[slot] += 1

    def _on_global_costmap(self, msg):
        self._store_costmap(msg, 'global')

    def _on_local_costmap(self, msg):
        self._store_costmap(msg, 'local')

    def layers_view(self):
        """兼容旧接口：只返回全局代价地图（不再把局部层合进来）。"""
        return self.costmap_view('global')[0]

    def costmap_view(self, slot='global'):
        """返回 (png, meta)：单个代价地图按自己的尺寸与原点成图。

        为什么不把两层合成一张：局部代价地图是跟着车走的滑动窗口，一旦和全局
        层画进同一张"按地图坐标系对齐"的图里，它就会在地图上滑来滑去——看起来
        就是地图在漂。分开渲染、各用自己的 meta 定位，才和 RViz 一致。
        """
        if slot not in ('global', 'local'):
            raise ValueError('slot must be global or local')
        with self._lock:
            item = self._global_cost if slot == 'global' else self._local_cost
            version = self._cost_version.get(slot, 0)
            if self._cost_png.get(slot) and self._cost_built.get(slot) == version:
                return self._cost_png[slot], (dict(item[1]) if item else {})
            snapshot = (item[0], dict(item[1])) if item else None
        if snapshot is None:
            return b'', {}
        data, meta = snapshot
        width = int(meta.get('width') or 0)
        height = int(meta.get('height') or 0)
        if width <= 0 or height <= 0:
            return b'', {}
        try:
            flat = b''.join(map_render.costmap_rows(data, width, height))
            png = map_render.write_png(width, height, flat, color_type=6)
        except Exception as error:
            self.get_logger().warning(
                'render costmap %s failed: %s' % (slot, error))
            return b'', {}
        with self._lock:
            self._cost_png[slot] = png
            self._cost_built[slot] = version
        return png, meta

    def _on_scan(self, msg):
        """激光点投到 map 坐标系。TF 拿不到就整帧丢掉，绝不假装有数据。"""
        ranges = list(msg.ranges)
        count = len(ranges)
        if count <= 0:
            return
        angle = float(msg.angle_min)
        step = float(msg.angle_increment)
        limit = float(msg.range_max or 0.0)
        points = []
        # 抽稀：一帧几百上千点没必要全画，抽到 scan_max_points 以内
        stride = max(1, count // max(1, self._scan_max))
        for index in range(0, count, stride):
            distance = ranges[index]
            if not (msg.range_min <= distance <= limit) or distance != distance:
                continue
            points.append((distance * math.cos(angle + index * step),
                           distance * math.sin(angle + index * step)))
        if not points:
            with self._lock:
                self._scan_points = []
                self._scan_time = time.time()
            return

        frame = str(msg.header.frame_id or '')
        if frame and frame != 'map' and self._tf_buffer is not None:
            points = self._transform_points(points, frame, msg.header.stamp)
            if points is None:
                return                      # 变换不可用：保持上一帧，别清空
        with self._lock:
            self._scan_points = points
            self._scan_time = time.time()

    def _transform_points(self, points, source_frame, stamp):
        if tf2_ros is None or Duration is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', source_frame.strip('/'), stamp,
                Duration(seconds=0.05))
        except Exception:
            return None
        tr = transform.transform.translation
        quat = transform.transform.rotation
        yaw = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        out = []
        for x, y in points:
            out.append((tr.x + x * cos_y - y * sin_y,
                        tr.y + x * sin_y + y * cos_y))
        return out

    def scan_view(self):
        with self._lock:
            points = list(self._scan_points)
            age = (time.time() - self._scan_time) if self._scan_time else -1.0
        return points, age

    def _on_plan(self, msg):
        """全局路径：只留 (x, y)，抽稀到 200 点以内够画了。"""
        poses = list(getattr(msg, 'poses', []) or [])
        step = max(1, len(poses) // 200)
        points = []
        for index in range(0, len(poses), step):
            position = getattr(getattr(poses[index], 'pose', None),
                               'position', None)
            if position is None:
                continue
            points.append((float(position.x), float(position.y)))
        with self._lock:
            self._plan_points = points
            self._plan_time = time.time()

    def plan_view(self):
        with self._lock:
            points = list(self._plan_points)
            age = (time.time() - self._plan_time) if self._plan_time else -1.0
        return points, age

    def _on_image(self, msg):
        data = self._encode(msg)
        if data:
            with self._lock:
                self._frame_jpeg = data
                self._frame_time = time.time()

    def _on_annotated(self, msg):
        data = self._encode(msg)
        if data:
            with self._lock:
                self._anno_jpeg = data
                self._anno_time = time.time()

    def _on_image_compressed(self, msg):
        """压缩图直接透传：不再解压重编码，省一次 CPU 也省一次内存拷贝。"""
        data = bytes(msg.data)
        if data:
            with self._lock:
                self._frame_jpeg = data
                self._frame_time = time.time()

    def _on_annotated_compressed(self, msg):
        data = bytes(msg.data)
        if data:
            with self._lock:
                self._anno_jpeg = data
                self._anno_time = time.time()

    def _on_pose(self, msg):
        pose = msg.pose.pose
        # 朝向也要存：地图上那个箭头没有 yaw 就只能画成圆点，
        # 而"车头朝哪"恰恰是给初始位姿时最需要看的东西。
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (float(pose.position.x), float(pose.position.y),
                          float(yaw), time.time())

    def _on_log(self, msg):
        self.logs.add(msg.name, int(msg.level), msg.msg)

    @staticmethod
    def _encode(msg):
        """sensor_msgs/Image -> JPEG 字节。优先 cv2，退化纯 Python。"""
        try:
            import numpy as np
            width = int(msg.width)
            height = int(msg.height)
            if width <= 0 or height <= 0:
                return b''
            step = int(msg.step) or width * 3
            raw = bytes(msg.data)
            expected = step * height
            if len(raw) < expected:
                return b''
            encoding = (msg.encoding or '').lower()
            try:
                import cv2
            except Exception:
                cv2 = None
            if cv2 is not None:
                buffer = np.frombuffer(raw, dtype=np.uint8).reshape(
                    (height, step))
                if encoding in ('bgr8', 'rgb8'):
                    image = buffer[:, :width * 3].reshape((height, width, 3))
                    if encoding == 'rgb8':
                        image = image[:, :, ::-1]
                elif encoding in ('mono8', '8uc1'):
                    image = buffer[:, :width]
                else:
                    image = buffer[:, :width]
                ok, encoded = cv2.imencode('.jpg', image,
                                           [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                return encoded.tobytes() if ok else b''
            return b''                       # 没有 cv2 就放弃（页面显示占位图）
        except Exception:
            return b''

    # ------------------------------------------------------------ 查询
    def snapshot_status(self):
        with self._lock:
            raw = self._status_raw
            battery = self._battery_v
            pose = self._pose
        status = normalize_status(raw, warn_v=self._warn_v, crit_v=self._crit_v)
        # 取消请求晚于最后一条状态：执行器还没回报，但这一轮已经不跑了。
        # 等它真发出新状态（_status_at 更新到 stop 之后），这段自动失效。
        with self._lock:
            stop_at = self._patrol_stop_at
            status_at = self._status_at
        if stop_at and stop_at > status_at and status.get('running'):
            status['running'] = False
            status['state'] = 'canceled'
        if status['battery_v'] is None and battery is not None:
            status['battery_v'] = battery
            from .data import battery_band
            status['battery_band'] = battery_band(battery, self._warn_v,
                                                  self._crit_v)
        status['pose'] = ({'x': pose[0], 'y': pose[1], 'yaw': pose[2],
                           'age_s': max(0.0, time.time() - pose[3])}
                          if pose else None)
        # 车体外框的尺寸也给前端，地图上就能画出 RViz 里那个 footprint 方块
        status['robot'] = {'length': self._robot_length,
                           'width': self._robot_width}
        status['estop'] = self.estop_state()
        return status

    def map_view(self):
        with self._lock:
            return self._map_png, dict(self._map_meta)

    def frame(self, annotated=False):
        with self._lock:
            if annotated:
                return self._anno_jpeg, self._anno_time
            return self._frame_jpeg, self._frame_time

    def frame_age(self, annotated=False):
        data, stamp = self.frame(annotated)
        if not data or not stamp:
            return None
        return max(0.0, time.time() - stamp)

    # ------------------------------------------------------------ 急停
    def estop(self):
        """一键停：清手动速度、打断导航与巡检、连发几帧零速。

        在此之前唯一的安全网是 teleop 心跳超时，那只在"没人再按方向键"时生效；
        导航还在跑、巡检还在执行的时候，人其实停不下来。
        """
        with self._estop_lock:
            self._estop_active = True
            self._estop_since = time.time()
        with self._teleop_lock:
            self._teleop_vx = self._teleop_wz = 0.0
            self._teleop_seen = 0.0        # 立刻进入 tick 的"静默"分支
        # 连发 3 帧零速：有的底盘运动控制器会对自己插值，单帧可能被吃掉
        for _ in range(3):
            self._publish_zero()
            time.sleep(0.02)

        done = []
        ok, _ = self.cancel_nav()
        done.append('nav %s' % ('已取消' if ok else '本就没在导航'))
        ok, _ = self.cancel_patrol()
        done.append('patrol %s' % ('已取消' if ok else '本就没在巡检'))
        # 急停本身是正常操作，不该记 ERROR：空转时按一下，页头的"错误"计数
        # 会 +1，用户会以为系统出故障了。真出事要报警的是"刹车没发出去"。
        self.logs.add('console', 20, 'ESTOP pressed: %s' % '; '.join(done))
        return True, '已急停（%s）' % '，'.join(done)

    def release_estop(self):
        with self._estop_lock:
            was_active = self._estop_active
            self._estop_active = False
            self._estop_since = 0.0
        if was_active:
            self.logs.add('console', 20, 'ESTOP released')
        return True, '已解除急停，可以重新操作' if was_active else '当前不在急停状态'

    def estop_state(self):
        with self._estop_lock:
            return {'active': self._estop_active, 'since': self._estop_since}

    def _blocked(self, what):
        """会动的入口统一走这里：急停中直接拒绝，返回 (False, 原因)。"""
        if self.estop_state()['active']:
            return False, '急停中：%s 被拦下，先解除急停' % what
        return None

    def _publish_zero(self):
        message = Twist()
        try:
            self._teleop_pub.publish(message)
        except Exception:
            pass

    # ------------------------------------------------------------ 手动控制
    def set_teleop(self, vx, wz):
        """页面按住方向键时每 100 ms 刷新一次目标速度。"""
        with self._estop_lock:
            if self._estop_active:
                return          # 急停中不接受任何速度，包括"继续按着"
        with self._teleop_lock:
            self._teleop_vx = float(vx)
            self._teleop_wz = float(wz)
            self._teleop_seen = time.time()

    def _teleop_tick(self):
        """按住才发；松手补一帧零速；空闲彻底不发。

        原来这里每 100 ms 无条件发 Twist（空闲发 0），于是控制台一开，
        `/cmd_vel` 上就多了一个 10 Hz 的零速发布者，和 Nav2 那 20 Hz 的
        控制循环交替写入。底盘按"最后一条指令"执行，车表现就是一冲一停、
        一顿一顿——而 Nav2 自己发的速度序列完全正确，所以很难查。
        """
        now = time.time()
        with self._teleop_lock:
            send, vx, wz, self._teleop_vx, self._teleop_wz = data.teleop_step(
                self._teleop_vx, self._teleop_wz, self._teleop_seen, now,
                self._teleop_timeout_sec)
        if not send:
            return
        message = Twist()
        message.linear.x = vx
        message.angular.z = wz
        try:
            self._teleop_pub.publish(message)
        except Exception:
            pass

    def public_host(self):
        """给日志/页面用的可访问地址（解析顺序见 data.public_host）。"""
        return data.public_host(
            self.get_parameter('public_host').value,
            self.get_parameter('http_host').value)

    def reports_dir(self):
        return self._reports_dir

    def subsystems(self):
        """每个面板依赖的东西到底在不在——页面顶部那排小灯就是它。

        以前的做法是让面板空着，用户看不出是「没启动」还是「坏了」。
        这里给每一项一个明确状态和一句怎么补的话。
        """
        with self._lock:
            has_map = bool(self._map_png)
            pose = self._pose
            frame_stamp = self._frame_time
        out = []

        def add(key, label, ok, hint, detail=''):
            out.append({'key': key, 'label': label, 'ok': bool(ok),
                        'hint': '' if ok else hint, 'detail': detail})

        # 地图：有 PNG 就说明 /map 至少来过一帧（transient_local，连上就能拿到）
        add('map', '地图', has_map,
            '没收到 /map：确认 slam_toolbox 或 map_server 在跑 '
            '(ros2 topic hz /map)')

        # 定位：AMCL 位姿是否新鲜（下标取错会把 yaw 当时间戳，见 data.pose_age）
        pose_age = data.pose_age(pose)
        add('pose', '定位', pose_age is not None and pose_age < 10.0,
            '没有 /amcl_pose：确认 AMCL 起来了并给了初始位姿 '
            '(RViz 的 2D Pose Estimate)',
            '' if pose_age is None else '最后一条 %.1fs 前' % pose_age)

        # 导航：action server 有没有人接
        nav_ready = bool(self._nav_client is not None
                         and self._nav_client.server_is_ready())
        add('nav', '导航', nav_ready,
            'navigate_to_pose 没有服务端：nav2 没起，或用 '
            'ros2 launch fishbot_console robot.launch.py 一次性拉起')

        # 巡检：action server
        patrol_ready = bool(self._patrol_client is not None
                            and self._patrol_client.server_is_ready())
        add('patrol', '巡检', patrol_ready,
            'execute_patrol 没有服务端：巡检执行器没起 '
            '(ros2 launch fishbot_patrol_v2 patrol_v2.launch.py)')

        # 相机：帧龄
        age = (time.time() - frame_stamp) if frame_stamp else None
        add('camera', '相机', age is not None and age < 3.0,
            '没有图像：视觉节点没起，或两台机器不在同一个 ROS_DOMAIN_ID',
            '' if age is None else '最后一帧 %.1fs 前' % age)

        return out

    # ------------------------------------------------------------ 动作
    def set_initial_pose(self, x, y, yaw=0.0):
        """发 /initialpose，等价于 RViz 的 2D Pose Estimate。

        协方差按 RViz 的默认值给（位置 0.25 m²、朝向 0.0685 rad² ≈ 15°），
        这两个数不是随便写的：AMCL 用它们当初始粒子分布的宽度，给 0 会让
        滤波器一上来就发散。
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.06853891945200942
        msg.pose.covariance = cov
        self._initial_pub.publish(msg)
        self.logs.add('console', 20,
                      'initial pose -> (%.2f, %.2f) yaw %.2f rad'
                      % (x, y, yaw))
        return True, '已发送初始位姿 (%.2f, %.2f) %.0f°' % (
            x, y, math.degrees(yaw))

    def send_nav(self, x, y, yaw=None):
        blocked = self._blocked('导航')
        if blocked:
            return blocked
        if self._nav_client is None:
            return False, '没有 nav2（nav2_msgs 未安装）'
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return False, '导航动作服务不在线（nav2 没起来？）'
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        if yaw is not None:
            import math
            goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        else:
            goal.pose.pose.orientation.w = 1.0
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_accepted)
        return True, '已发送导航目标 (%.2f, %.2f)' % (float(x), float(y))

    def _nav_accepted(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.logs.add('console', 40, 'nav goal failed: %s' % error)
            return
        self._nav_goal_handle = handle
        if handle is not None and handle.accepted:
            self.logs.add('console', 20, 'nav goal accepted')
        else:
            self.logs.add('console', 30, 'nav goal rejected')

    def cancel_nav(self):
        handle = self._nav_goal_handle
        if handle is None:
            return False, '当前没有导航目标'
        future = handle.cancel_goal_async()
        future.add_done_callback(
            lambda _f: self.logs.add('console', 20, 'nav goal cancel sent'))
        self._nav_goal_handle = None
        return True, '已发送取消请求'

    def start_patrol(self, route='', mode=0, repeat=1):
        blocked = self._blocked('巡检')
        if blocked:
            return blocked
        if self._patrol_client is None:
            return False, '没有巡检动作接口（fishbot_patrol_interfaces 未安装）'
        if not self._patrol_client.wait_for_server(timeout_sec=2.0):
            return False, '巡检执行器不在线'
        goal = ExecutePatrol.Goal()
        goal.route_name = str(route or '')
        goal.mode = int(mode)
        goal.repeat_count = int(repeat)
        future = self._patrol_client.send_goal_async(goal)
        future.add_done_callback(self._patrol_accepted)
        return True, '已发送巡检（路线 %s）' % (route or '默认')

    def _patrol_accepted(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.logs.add('console', 40, 'patrol goal failed: %s' % error)
            return
        self._patrol_goal_handle = handle
        if handle is not None and handle.accepted:
            self.logs.add('console', 20, 'patrol goal accepted')
        else:
            self.logs.add('console', 30, 'patrol goal rejected')

    def cancel_patrol(self):
        handle = self._patrol_goal_handle
        with self._lock:
            # 无论有没有 handle 都记下这个时刻：页面的"运行中"是从状态消息
            # 推出来的，执行器收到取消后可能不再发新状态，光清 handle 不够。
            self._patrol_stop_at = time.time()
        if handle is None:
            return False, '没有可取消的巡检目标（面板不会显示在跑，就是本来没在跑）'
        handle.cancel_goal_async()
        self._patrol_goal_handle = None
        self.logs.add('console', 20, 'patrol cancel requested')
        return True, '已发送取消请求'

    # ------------------------------------------------------------ 服务
    def snapshot(self, annotated=True):
        """把当前缓存的那一帧存成文件，返回 (ok, 路径或错误)。

        为什么不调相机服务：这个仓库的相机节点只发布图像、**没有抓拍服务**
        （唯一的服务是 /yolo_detector/set_continuous）。而控制台一直在收图，
        直接把最近一帧落盘既不需要额外依赖，也保证存下来的就是屏幕上看到的那张。
        """
        data, stamp = self.frame(annotated)
        if not data:
            return False, '还没有收到画面'
        try:
            os.makedirs(self._snapshot_dir, exist_ok=True)
        except OSError as error:
            return False, '建目录失败：%s' % error
        name = 'snap_%s_%s.jpg' % (
            'anno' if annotated else 'raw',
            time.strftime('%Y%m%d-%H%M%S', time.localtime(stamp or time.time())))
        path = os.path.join(self._snapshot_dir, name)
        index = 1
        while os.path.exists(path):
            path = os.path.join(self._snapshot_dir,
                                name.replace('.jpg', '_%d.jpg' % index))
            index += 1
        try:
            with open(path, 'wb') as handle:
                handle.write(data)
        except OSError as error:
            return False, '写文件失败：%s' % error
        self.logs.add('console', 20, 'snapshot saved: %s' % path)
        return True, path

    def call_capture(self, timeout=3.0):
        """可选：如果外部装了抓拍服务（Trigger），用它；否则退回存缓存帧。"""
        if self._capture_client is None:
            return self.snapshot()
        if not self._capture_client.wait_for_service(timeout_sec=timeout):
            return self.snapshot()
        future = self._capture_client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.05)
        if not future.done():
            return self.snapshot()
        try:
            response = future.result()
        except Exception:
            return self.snapshot()
        if not response.success:
            return self.snapshot()
        return True, str(response.message or '已抓拍')

    def route_points(self, name=''):
        """取路线点位（只为在地图上显示）。字段名以 GetRoute.srv 为准：route_name。"""
        if self._route_client is None:
            return []
        try:
            if not self._route_client.wait_for_service(timeout_sec=1.0):
                return []
            request = GetRoute.Request()
            request.route_name = str(name or '')
            future = self._route_client.call_async(request)
            end = time.time() + 2.0
            while time.time() < end and not future.done():
                time.sleep(0.05)
            if not future.done():
                return []
            response = future.result()
        except Exception as error:
            self.logs.add('console', 30, 'route_points failed: %s' % error)
            return []
        points = getattr(getattr(response, 'route', None), 'points', None) or []
        out = []
        for item in points:
            pose = getattr(item, 'pose', None)
            position = getattr(pose, 'position', None)
            out.append({
                'point_id': str(getattr(item, 'point_id', '')),
                'x': float(getattr(position, 'x', 0.0)),
                'y': float(getattr(position, 'y', 0.0)),
            })
        return out

    # ------------------------------------------------------------ 建图
    def _run_shell(self, command, timeout=20.0):
        try:
            done = subprocess.run(command, shell=True, capture_output=True,
                                  text=True, timeout=timeout)
            text = (done.stdout or '') + (done.stderr or '')
            return done.returncode == 0, text.strip()[-400:]
        except Exception as error:
            return False, str(error)

    def mapping(self, action, name='map'):
        """建图的起/停/存。这些动作本身是"跑一条命令"，结果回显给页面。"""
        action = str(action or '').lower()
        if action == 'start':
            cmd = ('nohup ros2 launch slam_toolbox online_async_launch.py '
                   '>~/slam.log 2>&1 &')
        elif action == 'stop':
            cmd = 'pkill -f slam_toolbox'
        elif action == 'save':
            path = os.path.expanduser('~/%s' % (name or 'map'))
            cmd = ('ros2 run nav2_map_server map_saver_cli -f %s'
                   % path)
        else:
            return False, '未知的建图动作：%s' % action
        ok, text = self._run_shell(cmd)
        self.logs.add('console', 20 if ok else 40,
                      'mapping %s -> %s' % (action, 'ok' if ok else 'fail'))
        return ok, text

    # ------------------------------------------------------------ 栈 / 点位
    def running_nodes(self):
        """当前图里活着的节点名（不带命名空间前缀的短名）。

        栈健康判断靠这个，而不是靠"launch 父进程还活着"——父进程在、子节点
        全死了，是最常见也最坑的"看起来在跑"。
        """
        try:
            names = self.get_node_names()
        except Exception:                    # 图查询失败不应该炸掉接口
            return set()
        # names 是 '/nav/amcl' 这种全名，探针是 'amcl'；归一化后再比，
        # 否则面板永远显示 0/N（曾经把整块"一键启停"变成摆设）。
        return data.short_node_names(names)

    def env_info(self):
        """当前进程的 ROS 环境摘要：面板显示这个，比“0/3”有用得多。

        为什么值得单独做一个接口：面板看不见节点时，用户唯一能做的判断就是
        “到底是我没起，还是它看不见”。域号、RMW、图内节点数这三样同时摆出来，
        答案基本自己就出来了——域号不一样，图里当然一个节点都没有。
        """
        nodes = sorted(self.running_nodes())
        return {
            'domain': os.environ.get('ROS_DOMAIN_ID', '') or '0',
            'rmw': os.environ.get('RMW_IMPLEMENTATION', '') or 'rmw_fastrtps_cpp',
            'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY', '') or '0',
            'node_count': len(nodes),
            'nodes': nodes[:12],
        }

    def record_point(self, route_name, point_id='', task_id='detect',
                     wait_seconds=0.0, expect=''):
        """调 route_manager 的 record_point 记一个点。

        为什么不在控制台自己算位姿再写文件：route_manager 才是路线的唯一所有者，
        它还要处理"定位是否可信""点 id 是否重复""frame 是否一致"这些规则。绕开它
        直接写 json，迟早和它自己的校验打架。
        """
        if self._record_client is None:
            return False, 'record_point 服务不可用（缺 RecordPoint 消息类型）', None
        if not self._record_client.wait_for_service(timeout_sec=2.0):
            return False, 'route_manager 不在线（/patrol_route/record_point）', None
        request = RecordPoint.Request()
        request.route_name = str(route_name or '')
        request.point_id = str(point_id or '')
        request.task_id = str(task_id or 'detect')
        request.wait_seconds = float(wait_seconds or 0.0)
        request.expect = str(expect or '')
        future = self._record_client.call_async(request)
        end = time.time() + 5.0
        while time.time() < end and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, '录点超时（5s）', None
        try:
            response = future.result()
        except Exception as error:
            return False, '录点失败：%s' % error, None
        point = getattr(response, 'point', None)
        detail = None
        if point is not None:
            detail = {
                'point_id': str(getattr(point, 'point_id', '')),
                'x': float(getattr(getattr(getattr(point, 'pose', None),
                                           'position', None), 'x', 0.0)),
                'y': float(getattr(getattr(getattr(point, 'pose', None),
                                           'position', None), 'y', 0.0)),
            }
        return bool(response.success), str(response.message or ''), detail

    def delete_point(self, route_name, point_id):
        """删掉路线里的一个点，返回 (ok, 消息, 剩余点数)。

        和录点一样交给 route_manager：它才知道删除后 active_route 要不要清空、
        文件怎么写才原子。控制台只负责把请求转过去、把结果翻译成人话。
        """
        if self._delete_client is None:
            return False, 'delete_point 服务不可用（缺 DeletePoint 消息类型）', None
        if not self._delete_client.wait_for_service(timeout_sec=2.0):
            return False, ('route_manager 不在线（/patrol_route/delete_point），'
                           '或它还停在旧版本'), None
        request = DeletePoint.Request()
        request.route_name = str(route_name or '')
        request.point_id = str(point_id or '')
        future = self._delete_client.call_async(request)
        end = time.time() + 5.0
        while time.time() < end and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, '删点超时（5s）', None
        try:
            response = future.result()
        except Exception as error:
            return False, '删点失败：%s' % error, None
        # 失败时不要回传 remaining：srv 的默认值是 0，照抄会让人以为“删完了”
        ok = bool(response.success)
        remaining = int(getattr(response, 'remaining', 0)) if ok else None
        return ok, str(response.message or ''), remaining

    def publish_route_markers(self, points, suspects=(), frame_id='map'):
        """把路线点位发到 /console/route_points，供 RViz 的 MarkerArray 显示。"""
        from . import points as points_mod
        specs = points_mod.marker_specs(points, suspects)
        if self._marker_pub is None:
            return 0, 'MarkerArray 不可用（缺 visualization_msgs）'
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        for spec in specs:
            if Marker is None:
                break
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'route_points'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = spec['x']
            sphere.pose.position.y = spec['y']
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = spec['diameter']
            if spec['color'] == points_mod.SUSPECT:
                sphere.color.r, sphere.color.g = 1.0, 0.85   # 黄
            else:
                sphere.color.r, sphere.color.g = 0.95, 0.25  # 红
            sphere.color.a = 0.85
            array.markers.append(sphere)
            marker_id += 1

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = 'route_point_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = spec['x']
            label.pose.position.y = spec['y']
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.text = spec['text']
            if spec['color'] == points_mod.SUSPECT:
                label.color.r, label.color.g = 1.0, 0.85
            else:
                label.color.r, label.color.g = 0.95, 0.35
            label.color.a = 1.0
            array.markers.append(label)
            marker_id += 1
        self._marker_pub.publish(array)
        return len(specs), ''

    def clear_route_markers(self):
        """发一个空的 MarkerArray 把 RViz 里的点清掉。"""
        if self._marker_pub is None:
            return
        self._marker_pub.publish(MarkerArray())

    def start(self):
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin,
                                        daemon=True)
        self._thread.start()

    def shutdown(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
