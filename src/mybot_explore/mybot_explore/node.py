"""自主探索节点：订阅 /map 看前沿，可选地真的开过去。

两种模式：
    看图模式（默认 drive=false）
        只发 RViz marker，不动车。先用它确认"红点确实落在没探索过的边上"。

    开车模式（drive=true）
        用 actionlib 的 NavigateToPose 让 Nav2 把车带过去，走完重新算前沿，
        直到没有前沿了回起点。状态机全在 explorer.py 里，这里只做接线。

启动：
    ros2 launch mybot_explore explore.launch.py                 # 看图
    ros2 launch mybot_explore explore.launch.py drive:=true     # 开车
    ros2 service call /explore/start std_srvs/srv/Trigger       # 开车模式下开始

话题 / 服务：
    /map                    订阅（TRANSIENT_LOCAL）
    /explore/frontiers      MarkerArray，候选点（分数越高越红）
    /explore/candidates     PoseArray，同上，方便别的节点取
    /explore/blacklist      MarkerArray，失败过的点（灰叉）
    /explore/status         状态（std_msgs/String，JSON）
    /explore/start /stop    Trigger，开始 / 中止
"""
import json
import os
import math
import traceback
import time

import rclpy
from geometry_msgs.msg import Point, PoseArray, Pose, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import blocked as blocked_mod
from . import carve as carve_mod
from . import explorer as explorer_mod
from . import frontier
from . import slip as slip_mod
from . import pgm
from .explorer import _wrap_angle

try:                                    # nav2 没装时也能起"看图模式"
    from nav2_msgs.action import NavigateToPose
except Exception:                       # pragma: no cover
    NavigateToPose = None

try:                                    # 到点转圈用导航栈现成的 Spin action
    from nav2_msgs.action import Spin
except Exception:                       # pragma: no cover
    Spin = None



class ExplorerNode(Node):
    def __init__(self):
        super().__init__('mybot_explore')

        # ---- 参数 ----
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/explore/frontiers')
        self.declare_parameter('candidates_topic', '/explore/candidates')
        self.declare_parameter('blacklist_topic', '/explore/blacklist')
        self.declare_parameter('status_topic', '/explore/status')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('nav_server_wait_s', 5.0)
        self.declare_parameter('map_yaml', '')     # 非空则直读 pgm，不用 /map
        # 擦图：用激光把光束经过的未知格改成空闲，未知边界才会跟着车后退。
        # 关掉它就退化成"只走到已知边界为止"。
        self.declare_parameter('carve_enabled', True)
        self.declare_parameter('carve_max_range_m', 4.0)   # 只信这么远以内的光束
        self.declare_parameter('carve_robot_radius_m', 0.25)
        self.declare_parameter('carve_period_s', 0.5)      # 最快多久擦一次
        # 收工自动存图：slam 模式走 /slam_toolbox/save_map，amcl 模式退回 map_saver_cli
        self.declare_parameter('auto_save', True)
        self.declare_parameter('save_map_dir', '~/maps')
        self.declare_parameter('save_map_prefix', 'explore')
        self.declare_parameter('save_map_service', '/slam_toolbox/save_map')
        self.declare_parameter('map_despike', True)
        self.declare_parameter('drive', False)
        self.declare_parameter('autostart', False)
        self.declare_parameter('period_s', 1.0)
        for key, value in explorer_mod.DEFAULTS.items():
            if key == 'period_s':
                continue
            self.declare_parameter(key, value)
        # 打滑 / 低矮障碍：判据参数全在 slip.DEFAULTS 里
        self.declare_parameter('slip_debug', False)
        for key, value in slip_mod.DEFAULTS.items():
            self.declare_parameter(key, value)
        # 挡住点：只画有物证的那一小段墙，其余保持未知
        self.declare_parameter('blocked_wall_enabled', True)
        self.declare_parameter('blocked_wall_offset_m', 0.22)
        self.declare_parameter('blocked_wall_len_m', 0.50)
        self.declare_parameter('keepout_radius_m', 1.00)
        self.declare_parameter('blocked_persist_hits', 2)
        self.declare_parameter('blocked_file', blocked_mod.DEFAULT_PATH)
        self.declare_parameter('scan_gate_enabled', True)
        self.declare_parameter('scan_gate_topic', '/explore/scan_gate')

        # ---- 状态 ----
        self._grid = None
        self._static_map = False          # 直读 pgm 时，忽略 /map（否则会被整张已知的图覆盖）
        self._scan_stamp = None
        self._last_carve = 0.0
        self._warned = {}                 # 警告节流：key -> 上次打印时间
        self._explorer = explorer_mod.Explorer(self._explorer_params())
        self._goal_handle = None
        self._pending_goal = False
        self._last_signature = None
        self._last_goal = None            # 最近一次下发的目标 (x, y)，写日志用
        self._last_best = None            # 上一次打印的"最佳前沿"，用于日志降噪
        self._last_best_at = 0.0
        # 注意：这里**不能**叫 self._spin，否则会把下面的 _spin() 方法遮住，
        # 触发 TypeError: 'ActionClient' object is not callable（真机踩过）
        self._spin_client = None          # Spin action client（到点转圈）
        self._spin_ready = False
        self._spin_handle = None
        self._spin_left = 0
        self._spin_delta = math.pi
        self._spin_speed = 1.2
        self._spin_timeout = 30.0
        self._done_logged = False
        self._save_inflight = False
        self._saved_once = False
        # ---- 打滑 / 挡住点 ----
        self._slip = slip_mod.SlipDetector(self._slip_params())
        self._blocked = blocked_mod.BlockedStore(
            str(self.get_parameter('blocked_file').value))
        self._blocked.load()
        self._last_scan_msg = None
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._gate_closed = False
        self._gate_changed_at = None
        self._wall_segments = []          # [(segment, cells)]
        self._slips = 0
        # 扫描门控：打滑时只让 scan_gate 丢那几帧，不整段暂停建图
        self._gate_pub = self.create_publisher(
            Bool, str(self.get_parameter('scan_gate_topic').value), 10)
        try:
            from slam_toolbox.srv import SaveMap as _SaveMap
            self._save_map_srv = _SaveMap
        except Exception:
            self._save_map_srv = None
        self._save_client = None
        if self._save_map_srv is not None:
            self._save_client = self.create_client(
                self._save_map_srv, str(self.get_parameter('save_map_service').value))

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        volatile = QoSProfile(depth=1,
                              durability=QoSDurabilityPolicy.VOLATILE,
                              history=QoSHistoryPolicy.KEEP_LAST)
        map_topic = str(self.get_parameter('map_topic').value)
        # 两个订阅同一个话题，靠"谁先收到算谁"：
        #   map_server 发的是 latched（只发一次）→ 必须 TRANSIENT_LOCAL 才收得到；
        #   slam_toolbox 发的是 VOLATILE（周期发）→ 对 TRANSIENT_LOCAL 订阅者不兼容，
        #   它根本不会投递。两个都订上，静态图和 SLAM 图都能用。
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, latched)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, volatile)
        # 只是用来判断"传感器还活着"（暂不参与决策）
        # 传感器数据一律 BEST_EFFORT：订阅端比发布端"宽松"总是能收到，
        # 反过来（RELIABLE 订阅 + BEST_EFFORT 发布）一条都收不到。
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        # 打滑判定要知道"命令让车走多快"，跟实际位移对比
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 1)
        self._poses = self.create_publisher(
            PoseArray, str(self.get_parameter('candidates_topic').value), 1)
        self._black_markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('blacklist_topic').value), 1)
        self._status = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 1)

        self._nav = None
        self._nav_ready = False
        if NavigateToPose is not None:
            self._nav = ActionClient(self, NavigateToPose,
                                     str(self.get_parameter('nav_action').value))

        if Spin is not None:
            self._spin_client = ActionClient(self, Spin, 'spin')

        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)

        mapped = str(self.get_parameter('map_yaml').value or '')
        if mapped:
            try:
                self._grid = pgm.load_map(mapped)
                self._static_map = True
                self.get_logger().info(
                    '已直读地图 %s：%dx%d（未知按 pgm 还原，不受 free_thresh 影响）'
                    '；不再订阅 %s'
                    % (mapped, self._grid.width, self._grid.height,
                       self.get_parameter('map_topic').value))
            except Exception as error:
                self.get_logger().error('读地图失败 %s：%s' % (mapped, error))

        period = float(self.get_parameter('period_s').value) or 1.0
        self.create_timer(period, self._tick)

        drive = bool(self.get_parameter('drive').value)
        self.get_logger().info(
            '探索已启动：%s；等待 %s'
            % ('开车模式' if drive else '看图模式（不动车）',
               self.get_parameter('map_topic').value))

    # ------------------------------------------------------------------ 参数
    def _explorer_params(self):
        out = {}
        for key, value in explorer_mod.DEFAULTS.items():
            param = self.get_parameter(key)
            out[key] = param.value
        return out

    # ------------------------------------------------------------------ 输入
    def _on_map(self, msg):
        if self._static_map:
            return                        # 用直读的地图，别被外部 /map 覆盖
        self._grid = frontier.Grid.from_msg(msg)

    def _on_scan(self, msg):
        self._scan_stamp = msg.header.stamp
        self._last_scan_msg = msg
        self._carve(msg)

    def _on_cmd_vel(self, msg):
        self._cmd_linear = float(msg.linear.x)
        self._cmd_angular = float(msg.angular.z)

    def _carve(self, msg):
        """把这一帧激光擦进内存地图（未知 -> 空闲，命中的格子 -> 障碍）。"""
        if not bool(self.get_parameter('carve_enabled').value):
            return
        if self._grid is None:
            return
        now = time.monotonic()
        period = float(self.get_parameter('carve_period_s').value) or 0.5
        if now - self._last_carve < period:
            return
        pose = self._pose()
        if pose is None:
            return
        self._last_carve = now
        scan = {'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max}
        try:
            used, hit = carve_mod.carve_scan(
                self._grid, scan, pose,
                max_range_m=float(self.get_parameter('carve_max_range_m').value),
                robot_radius_m=float(
                    self.get_parameter('carve_robot_radius_m').value))
        except Exception as error:                       # noqa: BLE001
            self.get_logger().warning('擦图失败：%s' % error)
            return
        self._carve_totals = getattr(self, '_carve_totals', (0, 0))
        self._carve_totals = (self._carve_totals[0] + used,
                              self._carve_totals[1] + hit)

    def _pose(self):
        """map 系下的 (x, y, yaw)；拿不到返回 None。"""
        if not hasattr(self, '_tf'):
            try:
                from tf2_ros import Buffer, TransformListener
            except Exception:                            # pragma: no cover
                return None
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self)
        try:
            from rclpy.time import Time
            tf = self._tf.lookup_transform('map',
                                           str(self.get_parameter('base_frame').value),
                                           Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # 四元数转 yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return (t.x, t.y, math.atan2(siny, cosy))

    # ------------------------------------------------------------------ 服务
    def _on_start(self, request, response):
        pose = self._pose()
        if pose is None:
            response.success = False
            response.message = '拿不到 map→%s 的 TF：先让 Nav2 定位起来' % (
                self.get_parameter('base_frame').value)
            return response
        if not bool(self.get_parameter('drive').value):
            response.success = False
            response.message = '当前是看图模式（drive:=false），不会开车'
            return response
        self._explorer = explorer_mod.Explorer(self._explorer_params())
        self._explorer.start((pose[0], pose[1]), self._now())
        response.success = True
        response.message = '开始探索，起点 (%.2f, %.2f)' % (pose[0], pose[1])
        self.get_logger().info(response.message)
        return response

    def _on_stop(self, request, response):
        was = self._explorer.state
        self._explorer._finish('stopped_by_user')
        self._cancel_goal()
        response.success = True
        response.message = '已停止（原状态 %s）' % was
        return response

    # ------------------------------------------------------------------ 一拍
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        """定时器入口：把真正的逻辑包一层，异常不许把进程带走。

        真机上发生过 "TypeError: 'ActionClient' object is not callable" ——
        rclpy 的定时器回调里抛异常会一路冒到 spin()，整个节点直接退出，
        而 traceback 只打在终端、日志里什么都没有，看起来就像"莫名其妙退出"。
        现在异常会写进节点日志（带堆栈），并主动停下探索、取消当前目标。
        """
        try:
            self._tick_once()
        except Exception as error:                       # noqa: BLE001
            traceback.print_exc()
            self.get_logger().error(
                '探索节点内部异常，已停止探索并取消当前目标：%s' % error)
            self.get_logger().error(traceback.format_exc())
            try:
                self._cancel_goal()
            except Exception:                            # noqa: BLE001
                pass
            try:
                self._explorer.state = explorer_mod.DONE
                self._explorer.done_reason = 'internal_error'
            except Exception:                            # noqa: BLE001
                pass

    def _tick_once(self):
        now = self._now()
        grid = self._grid
        pose = self._pose()
        drive = bool(self.get_parameter('drive').value)
        if drive:
            self._check_slip(now, pose)

        if (drive and self._explorer.state == explorer_mod.IDLE
                and bool(self.get_parameter('autostart').value) and pose):
            self._explorer.start((pose[0], pose[1]), now)
            self.get_logger().info('autostart：开始探索')

        # 开车模式下最常见的"没反应"就是这两件事：没图、没定位。
        # 以前这里是静默 return，用户只看到"前沿 N 个"刷屏却永远不动。
        if drive and grid is None:
            self._warn_at('no_map', '开车模式但没有地图：检查 /map 或 map_yaml 参数')
        if drive and pose is None:
            self._warn_at(
                'no_pose',
                '拿不到 map→%s 的位姿：车还没定位。先在 RViz 里给一个 '
                '2D Pose Estimate（或确认 amcl 已启动），否则不会发目标'
                % self.get_parameter('base_frame').value)

        if grid is not None and drive and pose is not None:
            cmd = self._explorer.step(grid, (pose[0], pose[1], pose[2]), now)
            if cmd:
                self._handle(cmd, pose)

        # 收工必须说清楚为什么——以前直接静默停住，用户只看到"前沿 20 个"
        # 却不再动，完全不知道是探完了还是被掐掉了。
        if self._explorer.finished() and not self._done_logged:
            self._done_logged = True
            snap = self._explorer.snapshot(now=now)
            self.get_logger().info(
                '探索收工：原因 %s；目标 %d 个（成功 %d / 失败 %d / 超时 %d / 卡住 %d），'
                '黑名单 %d 个，走过 %.1f m'
                % (snap.get('done_reason') or '未知', snap.get('goals', 0),
                   snap.get('succeeded', 0), snap.get('failed', 0),
                   snap.get('timeouts', 0), snap.get('stuck', 0),
                   snap.get('blacklist', 0), snap.get('distance_m', 0.0)))
            if not self._saved_once and bool(self.get_parameter('auto_save').value):
                self._saved_once = True
                self._auto_save(grid)

        self._publish_markers(grid, pose)
        self._publish_status(now)

    # ------------------------------------------------------------------
    # 收工存图（B2/B3/B4）
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # 打滑 / 低矮障碍
    # ------------------------------------------------------------------

    def _slip_params(self):
        params = {}
        for key in slip_mod.DEFAULTS:
            try:
                params[key] = self.get_parameter(key).value
            except Exception:                             # noqa: BLE001
                continue
        return params

    def _lookup_xy(self, target, source):
        """取 target→source 的平移；拿不到返回 None。"""
        try:
            tf_buffer = self._tf_buffer()
        except Exception:                                 # noqa: BLE001
            return None
        if tf_buffer is None:
            return None
        try:
            stamp = rclpy.time.Time()
            transform = tf_buffer.lookup_transform(target, source, stamp)
        except Exception:                                 # noqa: BLE001
            return None
        translation = transform.transform.translation
        return (translation.x, translation.y)

    def _tf_buffer(self):
        if not hasattr(self, '_tf'):
            from tf2_ros import Buffer, TransformListener
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self)
        return self._tf

    def _check_slip(self, now, pose):
        if not bool(self.get_parameter('slip_enabled').value):
            return
        scan_msg = self._last_scan_msg
        if scan_msg is None:
            return
        scan = {'ranges': list(scan_msg.ranges),
                'angle_min': scan_msg.angle_min,
                'angle_increment': scan_msg.angle_increment,
                'range_min': scan_msg.range_min,
                'range_max': scan_msg.range_max}
        base = str(self.get_parameter('base_frame').value)
        odom_xy = self._lookup_xy('odom', base)
        map_odom = self._lookup_xy('map', 'odom')
        result = self._slip.update(now, odom_pose=odom_xy, map_odom=map_odom,
                                   scan=scan, linear=self._cmd_linear,
                                   angular=self._cmd_angular)
        if bool(self.get_parameter('slip_debug').value):
            # 排障用：到底哪一路数据没进来（默认关，不影响正常运行）
            if now - getattr(self, '_slip_dbg_at', 0.0) >= 2.0:
                self._slip_dbg_at = now
                self.get_logger().info(
                    '滑移诊断：scan=%s odom=%s map_odom=%s linear=%.2f angular=%.2f '
                    'ratio=%.3f hits=%d block_ratio=%.3f block_moved=%.2f state=%s'
                    % ('有' if scan_msg is not None else '无',
                       '有' if odom_xy is not None else '无',
                       '有' if map_odom is not None else '无',
                       self._cmd_linear, self._cmd_angular,
                       result.get('ratio', 0.0), result.get('hits', 0),
                       result.get('block_ratio', 0.0), result.get('block_odom_moved', 0.0),
                       result.get('state', '')))
        action = result.get('action')
        if action == 'confirmed':
            self._on_slip_confirmed(now, pose, result)
        elif action == 'resume':
            self._open_gate('车已恢复移动')
            self.get_logger().info('车又动起来了，恢复建图')
        elif action == 'blocked':
            self._on_blocked(now, pose, result)
        elif action == 'give_up':
            # 放弃这个点就立刻把门控打开：地图必须继续长，不能因为一个点
            # 就让它一直停在"丢帧"状态（以前这里会一直停着不收帧，整图都冻住）
            self._open_gate('放弃该点')
            self.get_logger().warning(
                '打滑后 %.0f 秒仍没动起来，放弃这个点（已恢复建图）'
                % float(self.get_parameter('slip_hold_timeout_s').value))

    def _on_blocked(self, now, pose, result):
        """推台阶 / 顶矮墙：车在往前开、odom 在涨，但世界没变。

        和打滑的区别：这种情况扫描是**可信的**（车真的没动、墙真的在那儿），
        所以不关扫描门控，只取消目标、标一段短墙、把这一片拉黑。
        """
        self._blocked_hits = getattr(self, '_blocked_hits', 0) + 1
        self.get_logger().warning(
            '疑似被顶住（台阶/矮障）：车在 (%.2f, %.2f)，odom %.2f 秒里涨了 %.2f m，'
            '但 %.1f m 内 %d 束扫描只变了 %.0f%%'
            % (pose[0], pose[1], float(self.get_parameter('block_window_s').value),
               result.get('block_odom_moved', 0.0),
               float(self.get_parameter('block_scan_near_m').value),
               result.get('block_hits', 0),
               result.get('block_ratio', 0.0) * 100.0))
        self._cancel_goal()
        if bool(self.get_parameter('blocked_wall_enabled').value):
            self._mark_blocked_wall(pose)
        keepout = float(self.get_parameter('keepout_radius_m').value)
        spot = self._blocked.add(
            pose[0], pose[1], pose[2], now=self._now_iso(), radius_m=keepout,
            slip_seconds=float(self.get_parameter('block_window_s').value))
        try:
            self._blocked.save()
        except Exception as error:                        # noqa: BLE001
            self.get_logger().warning('写挡住点失败：%s' % error)
        self._explorer.blacklist.append((spot['x'], spot['y']))
        self.get_logger().info(
            '已把这一片列为禁试区（半径 %.1f m），换下一个点' % keepout)

    def _on_slip_confirmed(self, now, pose, result):
        self._slips += 1
        self.get_logger().warning(
            '疑似低矮障碍：车在 (%.2f, %.2f) 打滑——odom 说在走，'
            '扫描只变了 %.0f%%（近场 %.1f m 内 %d 束），map→odom 漂了 %.2f m'
            % (pose[0], pose[1], result.get('ratio', 0.0) * 100.0,
               result.get('near_m', 0.0), result.get('hits', 0),
               result.get('map_odom_moved', 0.0)))
        self._cancel_goal()
        self._close_gate('打滑确认')
        if bool(self.get_parameter('blocked_wall_enabled').value):
            self._mark_blocked_wall(pose)
        keepout = float(self.get_parameter('keepout_radius_m').value)
        spot = self._blocked.add(
            pose[0], pose[1], pose[2], now=self._now_iso(), radius_m=keepout,
            slip_seconds=float(self.get_parameter('slip_confirm_s').value))
        try:
            self._blocked.save()
        except Exception as error:                        # noqa: BLE001
            self.get_logger().warning('写挡住点失败：%s' % error)
        # 只用于选点：把这一片拉黑，避免反复往里试
        self._explorer.blacklist.append((spot['x'], spot['y']))
        self.get_logger().info(
            '已记挡住点 (%.2f, %.2f)，第 %d 次；禁试区半径 %.1f m'
            % (spot['x'], spot['y'], spot['hits'], keepout))

    def _mark_blocked_wall(self, pose):
        """车头前沿画一条 0.5 m 短墙——只标有物证的那一段。"""
        segment = blocked_mod.wall_segment(
            pose[0], pose[1], pose[2],
            offset_m=float(self.get_parameter('blocked_wall_offset_m').value),
            length_m=float(self.get_parameter('blocked_wall_len_m').value))
        self._wall_segments.append(segment)
        if self._grid is not None:
            try:
                cells = blocked_mod.stamp_wall(self._grid, segment)
                self.get_logger().info('已在图上标出 %d 格短墙' % cells)
            except Exception as error:                    # noqa: BLE001
                self.get_logger().warning('标短墙失败：%s' % error)

    def _set_gate(self, closed, reason=''):
        """告诉 scan_gate 要不要丢帧。只丢帧，不暂停建图。"""
        if not bool(self.get_parameter('scan_gate_enabled').value):
            return
        if closed == self._gate_closed:
            return
        self._gate_closed = closed
        self._gate_changed_at = self._now()
        msg = Bool()
        msg.data = bool(closed)
        self._gate_pub.publish(msg)
        if closed:
            self.get_logger().info('扫描门控：关闭（%s）—— 这几帧不进图，建图继续'
                                   % (reason or ''))
        else:
            self.get_logger().info('扫描门控：打开（%s）' % (reason or ''))

    def _close_gate(self, reason=''):
        self._set_gate(True, reason)

    def _open_gate(self, reason=''):
        self._set_gate(False, reason)

    def _now_iso(self):
        return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())

    def _save_base_path(self):
        """~/maps/explore_20261116_1905（不带扩展名）。"""
        directory = os.path.expanduser(str(self.get_parameter('save_map_dir').value))
        try:
            os.makedirs(directory, exist_ok=True)
        except OSError as error:
            self.get_logger().warning('建目录失败 %s：%s' % (directory, error))
            return None
        stamp = time.strftime('%Y%m%d_%H%M')
        prefix = str(self.get_parameter('save_map_prefix').value) or 'explore'
        return os.path.join(directory, '%s_%s' % (prefix, stamp))

    def _auto_save(self, grid):
        """先试 slam_toolbox 的 save_map，不行就退回 map_saver_cli。

        存图是"顺手做的好事"，绝不能因为它出错把探索带停：
        整段包在 try/except 里，失败只打 warning。
        """
        try:
            self._auto_save_impl(grid)
        except Exception as error:
            import traceback
            self.get_logger().warning('自动存图失败（不影响探索）：%s' % error)
            self.get_logger().debug(traceback.format_exc())

    def _auto_save_impl(self, grid):
        base = self._save_base_path()
        if base is None:
            return
        used_service = False
        if self._save_client is not None and self._save_client.service_is_ready():
            request = self._save_map_srv.Request()
            # slam_toolbox 的 SaveMap.srv 里 name 是 std_msgs/String，
            # 直接赋 str 会报 "The 'name' field must be a sub message of type 'String'"
            request.name.data = base
            future = self._save_client.call_async(request)
            future.add_done_callback(lambda f: self._on_save_done(f, base, True))
            used_service = True
            self._save_inflight = True
            self.get_logger().info('存图中（slam_toolbox）：%s' % base)
        if not used_service:
            self._save_with_cli(base)

    def _on_save_done(self, future, base, from_service):
        self._save_inflight = False
        error = future.exception() if hasattr(future, 'exception') else None
        if error is not None:
            self.get_logger().warning('save_map 调用失败（%s），改用 map_saver_cli' % error)
            self._save_with_cli(base)
            return
        if from_service:
            self.get_logger().info('已保存：%s.pgm / %s.yaml' % (base, base))
        self._after_save(base)

    def _save_with_cli(self, base):
        """没有 slam 服务时，用 nav2 的 map_saver_cli 从 /map 导出。"""
        import subprocess
        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                   '-f', base, '-t', '/map']
        try:
            result = subprocess.run(command, capture_output=True, timeout=30.0)
        except Exception as error:
            self.get_logger().warning('map_saver_cli 起不来：%s' % error)
            return
        if result.returncode != 0:
            tail = (result.stderr or b'').decode('utf-8', 'replace').strip()[-200:]
            self.get_logger().warning('map_saver_cli 失败（%s）：%s'
                                      % (result.returncode, tail))
            return
        self.get_logger().info('已保存：%s.pgm / %s.yaml（map_saver_cli）' % (base, base))
        self._after_save(base)

    def _after_save(self, base):
        """去毛刺 + 打孤立障碍率。"""
        if not bool(self.get_parameter('map_despike').value):
            return
        yaml_path = base + '.yaml'
        try:
            grid = pgm.load_map(yaml_path)
        except Exception as error:
            self.get_logger().warning('读回存下的地图失败：%s' % error)
            return
        # 把"有物证的短墙"盖进推断图；原始 .pgm 不动（要严格的用原始图）
        for segment in self._wall_segments:
            try:
                blocked_mod.stamp_wall(grid, segment)
            except Exception:                             # noqa: BLE001
                continue
        before = frontier.isolated_rate(grid)
        clean = frontier.median_despike(grid)
        after = frontier.isolated_rate(clean)
        clean_path = base + '_clean.pgm'
        try:
            pgm.write_map(clean_path, clean)
        except Exception as error:
            self.get_logger().warning('写去毛刺地图失败：%s' % error)
            return
        self.get_logger().info(
            '去毛刺：%s（孤立障碍 %.1f%% → %.1f%%）' % (clean_path, before * 100.0, after * 100.0))

    def _warn_at(self, key, message, period_s=5.0):
        """同一条警告最多每 period_s 打一次，避免刷屏。"""
        now = self._now()
        last = self._warned.get(key, 0.0)
        if now - last < period_s:
            return
        self._warned[key] = now
        self.get_logger().warn(message)

    # ------------------------------------------------------------------ 动作
    def _handle(self, cmd, pose):
        if cmd['action'] == 'cancel':
            self.get_logger().warn('放弃当前目标：%s（点 %.2f, %.2f）'
                                   % (cmd.get('reason', ''),
                                      self._last_goal[0] if self._last_goal else 0.0,
                                      self._last_goal[1] if self._last_goal else 0.0))
            self._cancel_goal()
            return
        if cmd['action'] == 'spin':
            self._spin(cmd)
            return
        if cmd['action'] != 'navigate':
            return

        if self._nav is None:
            self.get_logger().error('没装 nav2_msgs，无法导航')
            return
        if not self._nav_ready:
            # 第一次连 DDS 发现可能要好几百毫秒，给 5 秒；连上之后就不再等
            try:
                self._nav_ready = self._nav.wait_for_server(
                    timeout_sec=float(self.get_parameter('nav_server_wait_s').value))
            except Exception:                            # noqa: BLE001
                # 退出过程中 context 已经失效，这时 wait_for_server 会抛 RCLError
                self._nav_ready = False
            if self._nav_ready:
                self.get_logger().info('已连上 %s 的 action server'
                                       % self.get_parameter('nav_action').value)
        if not self._nav_ready:
            self._warn_at(
                'no_nav_server',
                '连不上 %s（domain=%s, rmw=%s）：确认 Nav2 已经激活，'
                '并且和本节点在同一个 ROS_DOMAIN_ID 里。这里会自己重试'
                % (self.get_parameter('nav_action').value,
                   os.environ.get('ROS_DOMAIN_ID', '0'),
                   os.environ.get('RMW_IMPLEMENTATION', '默认')),
                5.0)
            self._explorer.on_navigation_result(False, self._now(), 'no_server')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = Point(x=cmd['x'], y=cmd['y'], z=0.0)
        yaw = math.atan2(cmd['y'] - pose[1], cmd['x'] - pose[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        kind = cmd.get('kind', 'frontier')
        self._last_goal = (cmd['x'], cmd['y'])
        self.get_logger().info('去 (%s) (%.2f, %.2f) 分数 %.2f'
                               % (kind, cmd['x'], cmd['y'], cmd.get('score', 0.0)))
        self._pending_goal = True
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        self._pending_goal = False
        if not handle.accepted:
            self.get_logger().warn('目标被 Nav2 拒绝')
            self._explorer.on_navigation_result(False, self._now(), 'rejected')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        ok = (status == 4)                       # GoalStatus.STATUS_SUCCEEDED
        self._goal_handle = None
        self._last_goal = None
        self.get_logger().info('导航结束：状态 %d' % status)
        self._explorer.on_navigation_result(ok, self._now(), 'status_%d' % status)

    def _spin(self, cmd):
        """到点后原地转一圈：用 Nav2 的 Spin action，不自己写运动控制。

        nav2 的 Spin 目标 `target_yaw` 是**绝对朝向**：直接给 yaw+2π 会被
        归一化成当前朝向（等于不动）。所以按 delta 拆成若干次 ≤180° 的
        绝对转向，一次转完再转下一次。
        """
        self._spin_delta = float(cmd.get('delta', math.pi))
        self._spin_speed = float(cmd.get('speed', 1.2))
        self._spin_timeout = float(cmd.get('timeout', 30.0))
        self._spin_left = int(cmd.get('rounds', 1))
        self._spin_step()

    def _spin_step(self):
        if self._spin_left <= 0:
            self._explorer.on_spin_result(True, self._now(), 'done')
            return
        if self._spin_client is None:
            self.get_logger().warn('没装 nav2_msgs 的 Spin，跳过转圈')
            self._explorer.on_spin_result(False, self._now(), 'no_spin_action')
            return
        if not self._spin_ready:
            try:
                self._spin_ready = self._spin_client.wait_for_server(
                    timeout_sec=float(self.get_parameter('nav_server_wait_s').value))
            except Exception:                            # noqa: BLE001
                self._spin_ready = False
        if not self._spin_ready:
            self._warn_at('no_spin_server',
                          '连不上 spin 的 action server（Nav2 没激活或 '
                          'behavior_server 没起来），这一轮到点不转圈', 10.0)
            self._explorer.on_spin_result(False, self._now(), 'no_spin_server')
            return

        pose = self._pose()
        yaw0 = pose[2] if pose is not None else 0.0
        goal = Spin.Goal()
        goal.target_yaw = _wrap_angle(yaw0 + self._spin_delta)
        goal.time_allowance.sec = int(self._spin_timeout)
        self.get_logger().info('到点转圈：第 %d/%d 段，目标朝向 %.2f rad'
                               % (self._spin_left, self._spin_left,
                                  goal.target_yaw))
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(self._on_spin_response)

    def _on_spin_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('转圈目标被拒绝')
            self._explorer.on_spin_result(False, self._now(), 'rejected')
            return
        self._spin_handle = handle
        handle.get_result_async().add_done_callback(self._on_spin_done)

    def _on_spin_done(self, future):
        status = future.result().status
        self._spin_handle = None
        ok = (status == 4)
        self._spin_left -= 1
        if ok and self._spin_left > 0:
            self._spin_step()
            return
        self.get_logger().info('转圈结束：状态 %d' % status)
        self._explorer.on_spin_result(ok, self._now(), 'status_%d' % status)

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ------------------------------------------------------------------ 输出
    def _publish_markers(self, grid, pose):
        candidates = []
        if grid is not None:
            candidates = frontier.find_frontiers(
                grid,
                robot_xy=(pose[0], pose[1]) if pose else None,
                min_size=int(self.get_parameter('min_size').value),
                unknown_radius_m=float(self.get_parameter('unknown_radius_m').value),
                max_candidates=int(self.get_parameter('max_candidates').value),
                w_distance=float(self.get_parameter('w_distance').value),
                w_gain=float(self.get_parameter('w_gain').value))

        self._markers.publish(self._marker_array(candidates))
        self._poses.publish(self._pose_array(candidates))
        self._black_markers.publish(self._blacklist_array())

        # 日志降噪：只有"最佳点挪了 ≥0.5 m"或"距上次打印 ≥5 s"才打一行。
        # 以前按 top5 坐标签名判重，坐标每拍都在抖，等于每秒一行，把决策淹了。
        if candidates:
            best = candidates[0]
            now = self._now()
            moved = 1e9
            if self._last_best is not None:
                moved = math.hypot(best['x'] - self._last_best[0],
                                   best['y'] - self._last_best[1])
            if moved >= 0.5 or (now - self._last_best_at) >= 5.0:
                self._last_best = (best['x'], best['y'])
                self._last_best_at = now
                self.get_logger().info(
                    '前沿 %d 个，最佳 (%.2f, %.2f) 分数 %.2f，未知面积 %d 格'
                    % (len(candidates), best['x'], best['y'],
                       best['score'], best['unknown']))
        elif grid is not None:
            now = self._now()
            if (now - self._last_best_at) >= 5.0:
                self._last_best_at = now
                self._last_best = None
                self.get_logger().info('没有前沿：要么地图还没建，要么已经探完')

    def _marker_array(self, candidates):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, cand in enumerate(candidates):
            dot = self._sphere(i, cand['x'], cand['y'], stamp)
            dot.color.r = 1.0
            dot.color.g = max(0.0, 1.0 - cand['score'])
            dot.color.b = 0.1
            dot.color.a = 0.9
            array.markers.append(dot)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = 'frontier_text'
            label.id = 1000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=cand['x'], y=cand['y'], z=0.5)
            label.scale.z = 0.18
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = '#%d %.2f' % (i + 1, cand['score'])
            array.markers.append(label)
        return array

    def _blacklist_array(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for i, (x, y) in enumerate(self._explorer.blacklist):
            cross = Marker()
            cross.header.frame_id = 'map'
            cross.header.stamp = stamp
            cross.ns = 'blacklist'
            cross.id = i
            cross.type = Marker.CUBE
            cross.action = Marker.ADD
            cross.pose.position = Point(x=x, y=y, z=0.1)
            cross.scale.x = cross.scale.y = 0.25
            cross.scale.z = 0.05
            cross.color.r = cross.color.g = cross.color.b = 0.5
            cross.color.a = 0.7
            array.markers.append(cross)
        return array

    def _sphere(self, idx, x, y, stamp):
        dot = Marker()
        dot.header.frame_id = 'map'
        dot.header.stamp = stamp
        dot.ns = 'frontier'
        dot.id = idx
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position = Point(x=x, y=y, z=0.15)
        dot.scale.x = dot.scale.y = 0.15
        dot.scale.z = 0.08
        return dot

    def _pose_array(self, candidates):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for cand in candidates:
            pose = Pose()
            pose.position = Point(x=cand['x'], y=cand['y'], z=0.0)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish_status(self, now):
        snap = self._explorer.snapshot(now)
        snap['map_ready'] = self._grid is not None
        snap['scan_ready'] = self._scan_stamp is not None
        snap['pose_ready'] = self._pose() is not None
        snap['nav_ready'] = bool(self._nav_ready)
        snap['drive'] = bool(self.get_parameter('drive').value)
        totals = getattr(self, '_carve_totals', (0, 0))
        snap['carved_free'] = totals[0]
        snap['carved_occupied'] = totals[1]
        snap['slip_state'] = self._slip.state
        snap['blocked_hits'] = getattr(self, '_blocked_hits', 0)
        snap['slip_ratio'] = round(self._slip.ratio, 3)
        snap['slips'] = self._slips
        snap['blocked_walls'] = len(self._wall_segments)
        snap['keepouts'] = len(self._blocked.spots)
        snap['scan_gate'] = 'closed' if self._gate_closed else 'open'
        self._status.publish(String(data=json.dumps(snap, ensure_ascii=False)))


def main(argv=None):
    rclpy.init(args=argv)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 或被 timeout / systemd 送 SIGTERM 都走这里，不算错误
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as error:                       # noqa: BLE001
            # SIGTERM 和解释器退出撞在一起时 rclpy 会抛，不属于业务错误
            print('explore_node: 销毁节点时出错：%s' % error)
        if rclpy.ok():
            rclpy.shutdown()
