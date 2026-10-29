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

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String

from . import map_render
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


class ConsoleBridge(Node):
    def __init__(self):
        super().__init__('fishbot_console')
        self.declare_parameter('status_topic', '/patrol/execution/status')
        self.declare_parameter('image_topic', '/camera/camera/image_raw')
        self.declare_parameter('annotated_topic', '/yolo_detector/image_annotated')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_max_points', 400)
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('robot_length', 0.22)
        self.declare_parameter('robot_width', 0.18)
        self.declare_parameter('battery_topic', '/battery/voltage')
        self.declare_parameter('pose_topic', '/amcl_pose')
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
        self.declare_parameter('ai_base_url', '')
        self.declare_parameter('ai_api_key', '')
        self.declare_parameter('ai_model', '')

        self._warn_v = float(self.get_parameter('battery_warn_v').value)
        self._crit_v = float(self.get_parameter('battery_crit_v').value)
        self._reports_dir = os.path.expanduser(
            str(self.get_parameter('reports_dir').value))

        self._lock = threading.Lock()
        self._status_raw = ''
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
        self.create_subscription(
            OccupancyGrid, self.get_parameter('global_costmap_topic').value,
            self._on_global_costmap, map_qos, callback_group=self._callbacks)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('local_costmap_topic').value,
            self._on_local_costmap, map_qos, callback_group=self._callbacks)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, fast_qos, callback_group=self._callbacks)
        self.create_subscription(
            Path, self.get_parameter('plan_topic').value,
            self._on_plan, fast_qos, callback_group=self._callbacks)


        # 手动给初始位姿：和 RViz 的 "2D Pose Estimate" 发的是同一个话题、同一套协方差
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

    # ------------------------------------------------------------ 回调
    def _on_status(self, msg):
        with self._lock:
            self._status_raw = msg.data

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

    def _on_global_costmap(self, msg):
        self._store_costmap(msg, 'global')

    def _on_local_costmap(self, msg):
        self._store_costmap(msg, 'local')

    def layers_view(self):
        """把全局 + 局部代价地图叠成一张和底图同尺寸的 RGBA PNG。

        缓存：只有代价地图更新过（version 变了）才重算，否则 5 Hz 轮询也
        不会把 CPU 吃满。
        """
        with self._lock:
            version = self._layers_version
            if self._layers_png and self._layers_built == version:
                return self._layers_png
            meta = dict(self._map_meta or {})
            layers = [item for item in (self._global_cost, self._local_cost)
                      if item]
        if not meta.get('width') or not layers:
            return b''
        try:
            png = map_render.composite_layers(
                int(meta['width']), int(meta['height']), meta, layers)
        except Exception as error:
            self.get_logger().warning('composite costmaps failed: %s' % error)
            return b''
        with self._lock:
            self._layers_png = png
            self._layers_built = version
        return png

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

        # 定位：AMCL 位姿是否新鲜
        pose_age = (time.time() - pose[2]) if pose else None
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
        if handle is None:
            return False, '当前没有巡检任务'
        handle.cancel_goal_async()
        self._patrol_goal_handle = None
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
