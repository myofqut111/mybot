# -*- coding: utf-8 -*-
"""假机器人：不开车也能把整套链路跑起来。

它做四件事，正好覆盖控制台里所有"需要真硬件才能看"的面板：
  1. 发 /map（latched）——地图面板有东西可画
  2. 20 Hz 发 /tf + /odom——位姿箭头会动，rosbridge 实时流有数据
  3. 10 Hz 发 /scan——点云与地图对得上（射线是真的从这张图上打的）
  4. 订阅 /cmd_vel 做运动积分；再假装一个 NavigateToPose 服务端，
     让"地图上点一下发目标"能真的跑起来

所以验证"小车不动"时的控制台，只需要：
    ros2 launch mybot_console mock.launch.py
然后浏览器打开 http://127.0.0.1:8080 —— 与接真车时的前端完全一致。
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import (PoseWithCovarianceStamped, TransformStamped,
                               Twist)
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from . import mock_world


def _latched_qos():
    """地图和 tf_static 必须 latched，否则晚启动的订阅者永远收不到第一帧。"""
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _sensor_qos():
    return QoSProfile(
        depth=5,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


class MockRobot(Node):
    def __init__(self) -> None:
        super().__init__('mock_robot')
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('scan_hz', 10.0)
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('start_y', 2.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('battery_start', 12.6)
        self.declare_parameter('battery_drain_per_min', 0.06)
        self.declare_parameter('scan_beams', 240)
        self.declare_parameter('max_range', 8.0)

        self._tick_hz = max(1.0, float(self.get_parameter('tick_hz').value))
        self._scan_hz = max(0.5, float(self.get_parameter('scan_hz').value))
        self._beams = max(16, int(self.get_parameter('scan_beams').value))
        self._max_range = float(self.get_parameter('max_range').value)
        self._battery = float(self.get_parameter('battery_start').value)
        self._drain = float(self.get_parameter('battery_drain_per_min').value)

        self._data, self._w, self._h, self._meta = mock_world.build_map(
            resolution=0.05)
        self._resolution = self._meta['resolution']

        self._lock = threading.Lock()
        self._pose = (float(self.get_parameter('start_x').value),
                      float(self.get_parameter('start_y').value),
                      float(self.get_parameter('start_yaw').value))
        self._cmd = (0.0, 0.0)
        self._goal = None
        self._last_tick = time.monotonic()

        # 假车也要有"身体"：半径 0.18 m。障碍按这个半径膨胀，撞上就滑/停。
        # 之前只判中心点、而且根本没有碰撞检查，于是车会直接穿过隔断墙。
        self._robot_radius = 0.18
        radius_cells = max(1, int(round(self._robot_radius / self._resolution)))
        self._blocked = mock_world.blocked_grid(
            self._data, self._w, self._h, radius_cells)
        # 规划用更宽的膨胀（多 10 cm）：贴着的弧线会切角，路径留点余量才不撞。
        self._plan_blocked = mock_world.blocked_grid(
            self._data, self._w, self._h, radius_cells + 2)
        self._nav_goal_xy = None
        self._nav_retries = 0
        self._path = []                  # 待跟随的路径点（世界坐标）
        self._nav_state = 'idle'         # idle / running / done / failed / canceled
        self._nav_detail = ''

        # 假代价地图的参数：0.4 m 全局膨胀 + 4 m 局部窗口 0.2 m 膨胀
        self._inflate_cells = max(1, int(round(0.40 / self._resolution)))
        self._local_half = max(2, int(round(2.0 / self._resolution)))
        self._local_inflate = max(1, int(round(0.20 / self._resolution)))
        self._tf_pub = TransformBroadcaster(self)
        self._tf_static_pub = StaticTransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, 'odom', _sensor_qos())
        self._scan_pub = self.create_publisher(
            LaserScan, 'scan', _sensor_qos())
        self._map_pub = self.create_publisher(
            OccupancyGrid, 'map', _latched_qos())
        self._battery_pub = self.create_publisher(
            Float32, 'battery/voltage', 10)
        # 假车不是执行器：用它自己的主题，别占 /patrol/execution/status。
        # 之前两者共用一个 latched 主题，执行器停下之后假车还在发 RUNNING，
        # 面板就一直显示"运行中"。
        self._status_pub = self.create_publisher(
            String, 'mock/robot/status', _latched_qos())

        # 假代价地图：验证阶段没有 nav2，而膨胀带是判断 RViz 对齐做没做对最直观的
        # 东西——缺了它，「代价地图」那个勾选框就永远是空的。
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, 'global_costmap/costmap', _latched_qos())
        self._local_costmap_pub = self.create_publisher(
            OccupancyGrid, 'local_costmap/costmap', _latched_qos())

        # 假 AMCL：控制台的「定位」探针看的是 /amcl_pose。没有它，地图上有车却
        # 报「定位 ×（确认 AMCL 起来了）」，验证时很容易误判成真的没定位。
        self._amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose', 10)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)

        self._publish_static_tf()
        self._publish_map()
        # 全局代价只算一次：墙格不多，膨胀 0.4 m 也就几十万次操作
        self._global_cost = mock_world.inflate(
            self._data, self._w, self._h, self._inflate_cells)
        self._publish_costmaps()

        self.create_timer(1.0 / self._tick_hz, self._tick)
        self.create_timer(1.0 / self._scan_hz, self._publish_scan)
        self.create_timer(1.0, self._slow)

        self._nav_server = self._start_fake_nav()

        self.get_logger().info(
            'mock_robot ready: map %dx%d, tick %.0fHz, scan %.0fHz, '
            'start pose (%.1f, %.1f, %.2f)'
            % (self._w, self._h, self._tick_hz, self._scan_hz,
               self._pose[0], self._pose[1], self._pose[2]))

    # ------------------------------------------------------------ 假 nav2
    def _start_fake_nav(self):
        """假装 NavigateToPose 服务端；没装 nav2_msgs 就只警告，不影响其他面板。"""
        try:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionServer
        except Exception as error:                      # noqa: BLE001
            self.get_logger().warning(
                'nav2_msgs 不可用，跳过假导航服务（地图上点目标会报 action 超时）: %s'
                % error)
            return None

        class _Server(object):
            pass

        server = _Server()
        server.action = NavigateToPose
        try:
            server.handle = ActionServer(
                self, NavigateToPose, 'navigate_to_pose', self._on_nav_goal)
        except Exception as error:                      # noqa: BLE001
            # 构造失败也不能把整个节点带下去：地图/点云/手动控制都还能用
            self.get_logger().warning(
                '假装导航服务端创建失败，只能不出导航（其他面板照常）: %s' % error)
            return None
        return server

    def _on_nav_goal(self, goal_handle):
        action = self._nav_server.action
        pose = goal_handle.request.pose.pose
        target = (pose.position.x, pose.position.y,
                  mock_world.yaw_of_quaternion(pose.orientation))
        self.get_logger().info(
            'mock nav goal -> (%.2f, %.2f)' % (target[0], target[1]))

        # 先规划：真车走的是 nav2 的全局路径，假车直着走就会穿墙
        with self._lock:
            start = self._pose
        start_cell = mock_world.world_to_cell(
            start[0], start[1], self._resolution,
            self._meta['origin_x'], self._meta['origin_y'])
        goal_cell = mock_world.world_to_cell(
            target[0], target[1], self._resolution,
            self._meta['origin_x'], self._meta['origin_y'])
        cells = mock_world.astar(self._plan_blocked, self._w, self._h,
                                 start_cell, goal_cell)
        if not cells:
            self.get_logger().warning(
                'mock nav: 目标不可达（在障碍里或被围住），拒绝执行')
            goal_handle.abort()
            return action.Result()

        cells = mock_world.simplify_path(cells, self._plan_blocked,
                                         self._w, self._h)
        points = mock_world.cells_to_points(
            cells, self._resolution,
            self._meta['origin_x'], self._meta['origin_y'])
        points.append((target[0], target[1]))
        with self._lock:
            self._path = points
            self._goal = target
            self._nav_goal_xy = (target[0], target[1])
            self._nav_retries = 0
            self._nav_state = 'running'
            self._nav_detail = ''
        self.get_logger().info(
            'mock nav: 规划出 %d 个路径点' % len(cells))

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    with self._lock:
                        self._goal = None
                        self._path = []
                        self._nav_state = 'canceled'
                    goal_handle.canceled()
                    self.get_logger().info('mock nav canceled')
                    return action.Result()
                with self._lock:
                    state = self._nav_state
                    current = self._pose
                    detail = self._nav_detail
                remaining = math.hypot(current[0] - target[0],
                                       current[1] - target[1])
                feedback = action.Feedback()
                feedback.distance_remaining = float(remaining)
                try:
                    goal_handle.publish_feedback(feedback)
                except Exception:                       # noqa: BLE001
                    pass
                if state == 'done' and remaining <= 0.25:
                    goal_handle.succeed()
                    self.get_logger().info('mock nav reached')
                    return action.Result()
                if state == 'failed':
                    # 撞墙了：真车会绕，假车会在这里报失败——报失败比装作到了强
                    self.get_logger().warning('mock nav failed: %s' % detail)
                    goal_handle.abort()
                    return action.Result()
                time.sleep(0.1)
        finally:
            with self._lock:
                self._goal = None
                self._path = []
                if self._nav_state == 'running':
                    self._nav_state = 'idle'

    # ------------------------------------------------------------ 发布
    def _publish_static_tf(self):
        stamp = self.get_clock().now().to_msg()
        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = 'base_link'
        base.child_frame_id = 'laser_frame'
        base.transform.translation.x = 0.10
        base.transform.translation.z = 0.20
        base.transform.rotation.w = 1.0
        self._tf_static_pub.sendTransform([base])

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self._resolution
        grid.info.width = self._w
        grid.info.height = self._h
        grid.info.origin.position.x = self._meta['origin_x']
        grid.info.origin.position.y = self._meta['origin_y']
        grid.info.origin.orientation.w = 1.0
        grid.data = [int(v) for v in self._data]
        self._map_pub.publish(grid)

    def _make_grid(self, data, width, height, origin_x, origin_y):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self._resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [int(v) for v in data]
        return grid

    def _publish_costmaps(self):
        """发全局 + 局部代价地图，局部那张跟着车走。"""
        self._costmap_pub.publish(self._make_grid(
            self._global_cost, self._w, self._h,
            self._meta['origin_x'], self._meta['origin_y']))

        half = self._local_half
        cx = int(round((self._pose[0] - self._meta['origin_x'])
                       / self._resolution)) - half
        cy = int(round((self._pose[1] - self._meta['origin_y'])
                       / self._resolution)) - half
        window = mock_world.crop(self._data, self._w, self._h,
                                 cx, cy, 2 * half, 2 * half)
        cost = mock_world.inflate(window, 2 * half, 2 * half,
                                  self._local_inflate)
        self._local_costmap_pub.publish(self._make_grid(
            cost, 2 * half, 2 * half,
            self._meta['origin_x'] + cx * self._resolution,
            self._meta['origin_y'] + cy * self._resolution))

    def _on_cmd(self, msg: Twist):
        # 有导航目标时以目标为准，手动控制不会把车拽走
        with self._lock:
            if self._goal is None:
                self._cmd = (float(msg.linear.x), float(msg.angular.z))

    def _replan(self):
        """从当前位姿重新规划到本次导航目标。调用方必须已持 self._lock。"""
        if self._nav_goal_xy is None:
            return False
        start_cell = mock_world.world_to_cell(
            self._pose[0], self._pose[1], self._resolution,
            self._meta['origin_x'], self._meta['origin_y'])
        goal_cell = mock_world.world_to_cell(
            self._nav_goal_xy[0], self._nav_goal_xy[1], self._resolution,
            self._meta['origin_x'], self._meta['origin_y'])
        cells = mock_world.astar(self._plan_blocked, self._w, self._h,
                                 start_cell, goal_cell)
        if not cells:
            return False
        cells = mock_world.simplify_path(cells, self._plan_blocked,
                                         self._w, self._h)
        points = mock_world.cells_to_points(
            cells, self._resolution,
            self._meta['origin_x'], self._meta['origin_y'])
        points.append(self._nav_goal_xy)
        self._path = points
        return True

    def _tick(self):
        now = time.monotonic()
        # 首帧 dt 要压住：节点起来后第一次 tick 可能隔了几百毫秒，乘上速度就是
        # 一次肉眼可见的"飞出去"（0.45 m/s × 0.5 s = 22 cm 的瞬移）。
        dt = min(0.15, max(0.0, now - self._last_tick))
        self._last_tick = now
        with self._lock:
            if self._path:
                target = self._path[0]
                vx, wz = mock_world.approach(
                    self._pose, target, max_v=0.35, stop_dist=0.05)
                if math.hypot(self._pose[0] - target[0],
                              self._pose[1] - target[1]) < 0.22:
                    self._path.pop(0)
                    if not self._path:
                        self._nav_state = 'done'
            else:
                vx, wz = self._cmd
            new_pose, hit = mock_world.step_guarded(
                self._pose, vx, wz, dt, self._blocked, self._w, self._h,
                self._resolution, self._meta['origin_x'],
                self._meta['origin_y'])
            if hit and (abs(vx) > 1e-6 or abs(wz) > 1e-6):
                # 撞上了：先按真实导航的做法重新规划（假车也不能一撞就死），
                # 重试 3 次还不行才判失败——否则一个小角落就让整次任务废掉。
                self._cmd = (0.0, 0.0)
                if self._path and self._nav_goal_xy is not None:
                    self._nav_retries += 1
                    if self._nav_retries <= 3 and self._replan():
                        self._nav_detail = ('撞到障碍后重新规划 (%d/3)'
                                            % self._nav_retries)
                    else:
                        self._nav_detail = ('被障碍挡住 (%.2f, %.2f)'
                                            % (self._pose[0], self._pose[1]))
                        self._nav_state = 'failed'
                        self._path = []
            self._pose = new_pose
            pose = self._pose

        stamp = self.get_clock().now().to_msg()
        self._publish_tf(stamp, pose)
        self._publish_odom(stamp, pose, vx, wz)
        self._publish_amcl(stamp, pose)

    def _publish_tf(self, stamp, pose):
        x, y, yaw = pose
        quat = mock_world.quaternion_of_yaw(yaw)

        odom = TransformStamped()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.transform.translation.x = x
        odom.transform.translation.y = y
        odom.transform.rotation.z = quat['z']
        odom.transform.rotation.w = quat['w']

        # map -> odom 给个单位阵：假车没有定位误差，这样 TF 链是完整的
        map_tf = TransformStamped()
        map_tf.header.stamp = stamp
        map_tf.header.frame_id = 'map'
        map_tf.child_frame_id = 'odom'
        map_tf.transform.rotation.w = 1.0

        self._tf_pub.sendTransform([map_tf, odom])

    def _publish_odom(self, stamp, pose, vx, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]
        quat = mock_world.quaternion_of_yaw(pose[2])
        odom.pose.pose.orientation.z = quat['z']
        odom.pose.pose.orientation.w = quat['w']
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(wz)
        self._odom_pub.publish(odom)

    def _publish_amcl(self, stamp, pose):
        """假装 AMCL 报了定位。协方差给个 0.02（约 14 cm 标准差）。"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        quat = mock_world.quaternion_of_yaw(pose[2])
        msg.pose.pose.orientation.z = quat['z']
        msg.pose.pose.orientation.w = quat['w']
        cov = [0.0] * 36
        cov[0] = 0.02
        cov[7] = 0.02
        cov[35] = 0.02
        msg.pose.covariance = cov
        self._amcl_pub.publish(msg)

    def _publish_scan(self):
        with self._lock:
            pose = self._pose
        beams = self._beams
        angles = [(-math.pi + 2.0 * math.pi * i / beams) for i in range(beams)]
        ranges = mock_world.raycast(
            self._data, self._w, self._h, self._resolution,
            self._meta['origin_x'], self._meta['origin_y'],
            pose[0], pose[1], pose[2], angles, max_range=self._max_range)
        fields = mock_world.scan_msg_fields(angles, ranges,
                                            range_max=self._max_range)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = fields['angle_min']
        # 用 angle_min + increment*(N-1) 反算，而不是拿 angles[-1]：两者浮点上
        # 有时差一点点，部分消费者会报 "angle_max inconsistent" 警告。
        scan.angle_max = (scan.angle_min
                          + scan.angle_increment * (len(fields['ranges']) - 1))
        scan.angle_increment = fields['angle_increment']
        scan.range_min = fields['range_min']
        scan.range_max = fields['range_max']
        scan.ranges = fields['ranges']
        self._scan_pub.publish(scan)

    def _slow(self):
        self._battery = max(
            9.5, self._battery - self._drain / 60.0)
        self._battery_pub.publish(Float32(data=float(self._battery)))
        status = String()
        with self._lock:
            moving = self._goal is not None or abs(self._cmd[0]) > 0.01
        status.data = ('{"state":"%s","running":%s,"loop":1,"completed":0,'
                       '"total":0,"message":"mock robot"}'
                       % ('RUNNING' if moving else 'IDLE',
                          'true' if moving else 'false'))
        self._status_pub.publish(status)
        self._publish_costmaps()


def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except Exception:                                   # noqa: BLE001
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
