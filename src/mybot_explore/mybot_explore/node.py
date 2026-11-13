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
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import carve as carve_mod
from . import explorer as explorer_mod
from . import frontier
from . import pgm

try:                                    # nav2 没装时也能起"看图模式"
    from nav2_msgs.action import NavigateToPose
except Exception:                       # pragma: no cover
    NavigateToPose = None


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
        self.declare_parameter('map_yaml', '')     # 非空则直读 pgm，不用 /map
        # 擦图：用激光把光束经过的未知格改成空闲，未知边界才会跟着车后退。
        # 关掉它就退化成"只走到已知边界为止"。
        self.declare_parameter('carve_enabled', True)
        self.declare_parameter('carve_max_range_m', 4.0)   # 只信这么远以内的光束
        self.declare_parameter('carve_robot_radius_m', 0.25)
        self.declare_parameter('carve_period_s', 0.5)      # 最快多久擦一次
        self.declare_parameter('drive', False)
        self.declare_parameter('autostart', False)
        self.declare_parameter('period_s', 1.0)
        for key, value in explorer_mod.DEFAULTS.items():
            if key == 'period_s':
                continue
            self.declare_parameter(key, value)

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

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid,
                                 str(self.get_parameter('map_topic').value),
                                 self._on_map, latched)
        # 只是用来判断"传感器还活着"（暂不参与决策）
        # 传感器数据一律 BEST_EFFORT：订阅端比发布端"宽松"总是能收到，
        # 反过来（RELIABLE 订阅 + BEST_EFFORT 发布）一条都收不到。
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)

        self._markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 1)
        self._poses = self.create_publisher(
            PoseArray, str(self.get_parameter('candidates_topic').value), 1)
        self._black_markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('blacklist_topic').value), 1)
        self._status = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 1)

        self._nav = None
        if NavigateToPose is not None:
            self._nav = ActionClient(self, NavigateToPose,
                                     str(self.get_parameter('nav_action').value))

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
        self._carve(msg)

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
        now = self._now()
        grid = self._grid
        pose = self._pose()
        drive = bool(self.get_parameter('drive').value)

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
            cmd = self._explorer.step(grid, (pose[0], pose[1]), now)
            if cmd:
                self._handle(cmd, pose)

        self._publish_markers(grid, pose)
        self._publish_status(now)

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
            self.get_logger().warn('放弃当前目标：%s' % cmd.get('reason', ''))
            self._cancel_goal()
            return
        if cmd['action'] != 'navigate':
            return

        if self._nav is None:
            self.get_logger().error('没装 nav2_msgs，无法导航')
            return
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('navigate_to_pose 还没起来，下拍再试')
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
        self.get_logger().info('导航结束：状态 %d' % status)
        self._explorer.on_navigation_result(ok, self._now(), 'status_%d' % status)

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

        signature = tuple((round(c['x'], 1), round(c['y'], 1)) for c in candidates[:5])
        if signature != self._last_signature:
            self._last_signature = signature
            if candidates:
                best = candidates[0]
                self.get_logger().info(
                    '前沿 %d 个，最佳 (%.2f, %.2f) 分数 %.2f，未知面积 %d 格'
                    % (len(candidates), best['x'], best['y'],
                       best['score'], best['unknown']))
            elif grid is not None:
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
        snap['drive'] = bool(self.get_parameter('drive').value)
        totals = getattr(self, '_carve_totals', (0, 0))
        snap['carved_free'] = totals[0]
        snap['carved_occupied'] = totals[1]
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
