#!/usr/bin/env python3
"""打滑检测节点。

订阅 odom / wheel_joints / imu / scan / cmd_vel，按 0.3 s 非重叠窗口
判断 P1~P5，把结论写成一行 JSON 发到 /slip/state。

设计要点：
- **只判不救**：动作（后退/转向/调膨胀）由 slip_executor 执行，
  本节点只负责"什么时候该救、救哪一种"。
- 时间戳优先用消息自带的 header.stamp（回放也能用），缺失才用本机时钟。
- /wheel_joints 缺失时自动降级：P3 关掉，其它照跑。
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String

from . import criteria as criteria_mod
from . import ranges as ranges_mod
from . import slam_guard as slam_guard_mod
from . import state as state_mod
from . import windows as windows_mod

WHEEL_WARN_PERIOD = 10.0


class SlipNode(Node):
    def __init__(self):
        super().__init__('slip_node')

        # ---------------- 参数 ----------------
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_topic', '/wheel_joints')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/slip/state')
        self.declare_parameter('action_topic', '/slip/action')
        self.declare_parameter('report_topic', '/slip/report')
        self.declare_parameter('debug_period_s', 5.0)   # 0 = 不打采样日志
        self.declare_parameter('use_header_stamp', False)  # 默认按接收时间切窗（各话题时钟可能不一致）
        self.declare_parameter('max_stamp_offset', 2.0)   # 时间戳与本机差超过它就当不可信
        self.declare_parameter('pause_on_suspect', False)  # 一见到疑似就打住建图
        self.declare_parameter('suspect_hold_s', 1.5)
        self.declare_parameter('max_window_s', 1.5)     # 里程计稀疏时窗口最多拉这么长
        self.declare_parameter('min_odom_samples', 2)   # 窗口里至少要几条 odom
        self.declare_parameter('alternate_after_fail', True)  # 失败后换姿势
        self.declare_parameter('window_s', 0.5)          # 0.3 s 装不下 8 Hz 的 odom
        self.declare_parameter('min_samples', 2)
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('min_cmd_coverage', 0.70)
        self.declare_parameter('min_wheel_hz', 8.0)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('recovery_enabled', False)
        # 打滑期间是否暂停建图（slam 模式才有意义；amcl 模式自动忽略）
        # 方案 §4：SLAM 和 TF 不能中断，打滑期间只降权不中断 → 默认关
        self.declare_parameter('pause_slam_on_slip', False)
        self.declare_parameter('slam_pause_service',
                               '/slam_toolbox/pause_new_measurements')
        self.declare_parameter('slam_pause_max_s', 15.0)
        # 方案 §1/§2：异常时占住速度通道、并取消 Nav2 当前目标
        self.declare_parameter('slip_cmd_topic', '/cmd_vel_slip')
        self.declare_parameter('fault_topic', '/slip/fault')
        self.declare_parameter('cancel_nav_on_fail', True)
        self.declare_parameter('nav_action', 'navigate_to_pose')
        # 方案 §4：打滑期间放大编码器里程协方差（SLAM/TF 不中断）
        self.declare_parameter('covariance_guard', True)
        self.declare_parameter('odom_guard_out', '/odom_guarded')
        self.declare_parameter('covariance_position_factor', 100.0)
        self.declare_parameter('covariance_yaw_factor', 100.0)
        # 方案 §6：位姿校验与重定位（用 EKF 输出，两种定位模式下都有）
        self.declare_parameter('pose_topic', '/odometry/filtered')
        self.declare_parameter('pose_jump_m', 0.15)
        self.declare_parameter('pose_jump_rad', 0.20)
        self.declare_parameter('relocate_on_jump', True)

        gp = self.get_parameter
        self.win_params = {
            'window_s': gp('window_s').value,
            'max_window_s': float(gp('max_window_s').value),
            'min_odom_samples': int(gp('min_odom_samples').value),
            'min_samples': int(gp('min_samples').value),
            'warmup_s': gp('warmup_s').value,
            'min_cmd_coverage': gp('min_cmd_coverage').value,
        }
        self.min_wheel_hz = gp('min_wheel_hz').value
        self.recovery_enabled = bool(gp('recovery_enabled').value)

        self.agg = windows_mod.WindowAggregator(self.win_params)
        self.machine = state_mod.SlipStateMachine(
            {'alternate_after_fail':
             bool(self.get_parameter('alternate_after_fail').value)})
        # 没有执行器时不要进 RECOVERY：否则动作没人接，状态卡在那儿
        self.machine.allow_recovery = self.recovery_enabled
        self.ranges_params = dict(ranges_mod.DEFAULTS)

        # 打滑期间摁住建图：打滑时 odom 在涨，扫描会按错位姿进图，事后拉不回来
        self.slam_guard = slam_guard_mod.SlamPauseGuard(
            self,
            service_name=self.get_parameter('slam_pause_service').value,
            enabled=bool(self.get_parameter('pause_slam_on_slip').value))
        self.pause_policy = slam_guard_mod.PausePolicy(
            max_hold_s=float(self.get_parameter('slam_pause_max_s').value))

        # ---------------- 状态 ----------------
        self._have_wheel = False
        self._wheel_last_t = None
        self._wheel_hz = 0.0
        self._wheel_hz_ts = 0.0
        self._warned_low_rate = 0.0
        self._last_verdict = {}
        self._last_action = None
        self._last_state = 'normal'
        self.wheel_seen = False
        self.scan_name = self.p('scan_topic')
        self._last_reason = ''
        self._alive = {'odom': 0, 'wheel': 0, 'imu': 0, 'scan': 0, 'cmd': 0}
        self._alive_prev = {}
        self._rate_ts = 0.0
        self._rate_cache = {}
        self._last_stamps = {}
        self._stamp_warned = False
        self._suspect_until = 0.0
        self._stamp_offset = 0.0
        self._action_id = 0
        self._last_debug = 0.0
        self._debug_period = float(self.get_parameter('debug_period_s').value)

        # ---------------- 接口 ----------------
        self.pub = self.create_publisher(String, gp('state_topic').value, 10)
        self.act_pub = self.create_publisher(String, gp('action_topic').value, 10)
        self.fault_pub = self.create_publisher(String, gp('fault_topic').value, 10)
        # 异常期间占住速度通道用（走仲裁的 slip 侧，不是直接抢 /cmd_vel）
        self.slip_cmd_pub = self.create_publisher(
            Twist, gp('slip_cmd_topic').value, 10)
        self._last_hold = 0.0
        self._failed_reported = False
        self._nav_cancelled = False
        self._cancel_client = None
        self._nav_action = str(gp('nav_action').value)
        self._cancel_nav = bool(gp('cancel_nav_on_fail').value)
        self.create_subscription(String, gp('report_topic').value,
                                 self._on_report, 10)
        self.create_subscription(Odometry, gp('odom_topic').value,
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, gp('wheel_topic').value,
                                 self._on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, gp('imu_topic').value,
                                 self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, gp('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, gp('cmd_topic').value,
                                 self._on_cmd, QoSProfile(depth=10))
        self.create_subscription(Odometry, gp('pose_topic').value,
                                 self._on_pose, QoSProfile(depth=20))

        # 方案 §4：协方差守门（默认开；amcl 环境下同样有效）
        self.odom_guard = None
        if bool(gp('covariance_guard').value):
            try:
                from .odom_guard import OdomCovarianceGuard
                self.odom_guard = OdomCovarianceGuard(
                    self,
                    {'out_topic': gp('odom_guard_out').value,
                     'position_factor': float(gp('covariance_position_factor').value),
                     'yaw_factor': float(gp('covariance_yaw_factor').value)})
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().warn('协方差守门没起来：%s' % exc)
        self._pose = None
        self._recovery_pose = None
        self._jump_reported = False

        # 20 Hz 轮询：窗口是 0.3 s，20 Hz 足够不丢边界
        self.create_timer(0.05, self._tick)
        self.create_timer(gp('publish_period_s').value, self._heartbeat)

        self.get_logger().info(
            '打滑检测已启动：窗 %.2fs（最长 %.2fs，至少 %d 条 odom），%s，判据 P1~P5，'
            '恢复=%s，建图暂停=%s，激光=%s'
            % (self.win_params['window_s'], self.win_params['max_window_s'],
               self.win_params['min_odom_samples'],
               '含左右轮' if self.wheel_seen else '无左右轮',
               '开' if self.recovery_enabled else '关（只报不动）',
               '开' if bool(self.p('pause_slam_on_slip')) else '关',
               self.scan_name))
    # 订阅
    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def max_stamp_offset(self):
        return float(self.get_parameter('max_stamp_offset').value)

    def _stamp(self, msg, fallback):
        """取消息时间戳。

        默认**不用** header.stamp，一律用接收时间切窗。原因是各话题的时钟并不
        一致：`/odom` 来自 ESP32（板子时钟）、`/scan` 来自 VM，两者差几百毫秒
        甚至 1.9 s 时，2.0 s 的容差判不出"不可信"，窗口却按其中一路的时间轴切，
        另一路在窗里一条样本都没有——现象就是 `odom 18 Hz` 但 `odom 样本不足（本窗 0 条）`。
        接收时间的抖动是毫秒级，对 0.5 s 的窗无所谓。
        """
        if not bool(self.get_parameter("use_header_stamp").value):
            return fallback
        st = getattr(getattr(msg, 'header', None), 'stamp', None)
        if st is None:
            return fallback
        v = st.sec + st.nanosec * 1e-9
        if not (st.sec or st.nanosec):
            return fallback
        off = v - fallback
        if abs(off) > self.max_stamp_offset:
            if not self._stamp_warned:
                self._stamp_warned = True
                self.get_logger().warn(
                    '输入时间戳与本机差 %.1f s（板子时钟没同步？），'
                    '改用接收时间切窗口' % off)
            self._stamp_offset = off
            return fallback
        return v

    def _on_odom(self, msg):
        t = self._stamp(msg, self._now())
        self._alive['odom'] += 1
        self._last_stamps['odom'] = t
        self.agg.add_odom(t, msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_pose(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (px, py, yaw)

    def _on_wheel(self, msg):
        t = self._stamp(msg, self._now())
        self._alive['wheel'] += 1
        self._last_stamps['wheel'] = t
        self.wheel_seen = True
        left = right = None
        names = list(msg.name)
        pos = list(msg.position)
        if len(pos) >= 2 and len(names) >= 2:
            for i, nm in enumerate(names[:len(pos)]):
                low = nm.lower()
                if 'left' in low:
                    left = pos[i]
                elif 'right' in low:
                    right = pos[i]
            if left is None or right is None:
                left, right = pos[0], pos[1]
        if left is None:
            return
        self._have_wheel = True
        if self._wheel_last_t is not None and t > self._wheel_last_t:
            inst = 1.0 / (t - self._wheel_last_t)
            self._wheel_hz = inst if self._wheel_hz <= 0 else \
                0.9 * self._wheel_hz + 0.1 * inst
        self._wheel_last_t = t
        self.agg.add_wheel(t, left, right)

    def _on_imu(self, msg):
        t = self._stamp(msg, self._now())
        self._alive['imu'] += 1
        self._last_stamps['imu'] = t
        self.agg.add_imu(t, msg.angular_velocity.z)

    def _on_scan(self, msg):
        t = self._stamp(msg, self._now())
        self._alive['scan'] += 1
        self._last_stamps['scan'] = t
        if not msg.ranges:
            return
        self.agg.add_scan(t, msg)

    def _on_cmd(self, msg):
        t = self._now()
        self._alive['cmd'] += 1
        self.agg.add_cmd(t, msg.linear.x, msg.angular.z)

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _tick(self):
        now = self._now()
        try:
            wins = self.agg.poll(now)
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error('窗口生成失败：%s' % exc)
            return

        for win in wins:
            verdict = self._evaluate(win)
            if verdict.get('slip'):
                self._suspect_until = now + float(self.p('suspect_hold_s'))
            self._last_verdict = verdict
            # 每个窗都刷新：窗口正常就清空。否则那句话会从启动一直挂到进程退出，
            # 看上去像"一直在报 0 条 odom"，其实早就不报了。
            self._last_reason = win.get('reason') or ''
            action = self.machine.step(now, verdict)
            state_before = self._last_state
            self._last_state = self.machine.state
            if state_before != self.machine.state:
                reasons = (verdict.get('reasons') or [])
                self.get_logger().info(
                    '状态 %s → %s（%s）｜窗 %.2f s，滑移率 %s%s'
                    % (state_before or 'normal', self.machine.state,
                       action.get('reason', ''),
                       win.get('t1', 0.0) - win.get('t0', 0.0),
                       {k: round(v, 2) for k, v in (verdict.get('ratios') or {}).items()},
                       ('｜判据 %s' % reasons[0]) if reasons else ''))
            kind = action.get('action')
            if kind in ('back', 'escape', 'inflate', 'stop', 'probe', 'go'):
                # 只有真的要让车动（或改参数）才发给执行器
                self._last_action = action
                self._dispatch(action)
                self._publish(win, verdict, action)
                self.get_logger().warn(
                    '打滑恢复动作：%s（%s）'
                    % (kind, action.get('reason', '')))
            elif kind == 'replan':
                # §3 过位移：把目标交回 Nav2，让它重新规划
                self._cancel_nav_goal()
                self._publish(win, verdict, action)
            elif kind == 'recovered':
                # 方案 §6：脱困后校验位姿跳变，跳得太多就清图/重定位
                self._publish(win, verdict, action)
                self._check_pose_jump(now)
            elif kind in ('abort', 'reset'):
                # 这些是状态机内部事件，不是给执行器的活
                self._publish(win, verdict, action)
                self.get_logger().warn('状态机：%s（%s）'
                                       % (kind, action.get('reason', '')))
                if kind == 'abort':
                    self._on_failed(now, action)
            elif verdict.get('slip'):
                self._publish(win, verdict, None)

        # 确认前也要先把车摁住：状态机一进 SLIP 就持续发零速，
        # 让 cmd_mux 一直把控制权留在恢复这一侧（否则 Nav2 还在往前开）。
        self._hold_still(now)

        # 方案 §2：一进恢复就取消 Nav2 目标，别让它一边判"无进展"一边调恢复行为。
        # 只对**确认后**的 recovery 生效：suspect 每 1~2 秒就可能来一次，
        # 在疑似阶段取消目标等于把导航掐死（今天 6 次跑里取消了约 100 次）。
        if self.machine.state == 'recovery' and not self._nav_cancelled:
            self._nav_cancelled = True
            self._recovery_pose = self._pose
            self._cancel_nav_goal()
        elif self.machine.state == 'normal':
            self._nav_cancelled = False
            self._failed_reported = False

        # 非 NORMAL 就摁住建图：打滑时 odom 还在涨，扫描会按错位姿进图。
        # 只在"确认后"才暂停的话，确认本身要 1 s 左右，那 1 s 的错误扫描
        # 已经写进图了，所以第一个"疑似"窗口就先摁住。
        abnormal = self.machine.state != 'normal'
        if bool(self.p('pause_on_suspect')) and now < self._suspect_until:
            abnormal = True
        want = self.pause_policy.want_pause(abnormal, now)
        if self.pause_policy.take_capped_event():
            self.get_logger().warn(
                '打滑状态已持续 %.0f s，先恢复建图（一直停着更糟）'
                % self.pause_policy.max_hold_s)
        self.slam_guard.sync(want)

        # 方案 §4：打滑期间放大编码器里程协方差（SLAM/TF 不中断）
        if self.odom_guard is not None:
            self.odom_guard.set_inflated(abnormal)

    def p(self, name):
        return self.get_parameter(name).value

    def _check_pose_jump(self, now):
        """方案 §6：恢复前后位姿差过大 → 清局部代价地图 + 触发重定位。

        为什么用 EKF 输出而不是轮子里程：轮子在打滑时正好是错的，
        自己算不出自己错了多少。
        """
        before = self._recovery_pose
        after = self._pose
        self._recovery_pose = None
        if before is None or after is None:
            return
        dist = math.hypot(after[0] - before[0], after[1] - before[1])
        dyaw = abs(self._wrap(after[2] - before[2]))
        lim_m = float(self.p('pose_jump_m'))
        lim_rad = float(self.p('pose_jump_rad'))
        if dist <= lim_m and dyaw <= lim_rad:
            self.get_logger().info(
                '脱困后位姿校验通过：平移 %.3f m、转角 %.3f rad'
                % (dist, dyaw))
            return
        self.get_logger().error(
            '脱困后位姿跳变 %.3f m / %.3f rad（阈值 %.2f m / %.2f rad）：'
            '地图与代价地图可能已经错位' % (dist, dyaw, lim_m, lim_rad))
        self._clear_costmaps()
        if bool(self.p('relocate_on_jump')):
            self._relocalize()

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _clear_costmaps(self):
        """清掉局部/全局代价地图，避免旧障碍把车困在错误位置。"""
        try:
            from nav2_msgs.srv import ClearEntireCostmap
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('清代价地图不可用：%s' % exc)
            return
        for name in ('local_costmap', 'global_costmap'):
            srv = '/%s/clear_entirely_%s' % (name, name)
            try:
                client = self.create_client(ClearEntireCostmap, srv)
                if not client.service_is_ready():
                    self.get_logger().warn('%s 不在（跳过清理）' % srv)
                    continue
                client.call_async(ClearEntireCostmap.Request())
                self.get_logger().warn('已请求清理 %s' % name)
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().warn('清理 %s 失败：%s' % (name, exc))

    def _relocalize(self):
        """AMCL 模式下让定位重新撒粒子；slam 模式没有这个服务，跳过。"""
        try:
            from std_srvs.srv import Empty
        except Exception:                             # noqa: BLE001
            return
        try:
            client = self.create_client(Empty, '/reinitialize_global_localization')
            if not client.service_is_ready():
                self.get_logger().warn(
                    '全局重定位服务不在（slam 模式正常），只清了代价地图')
                return
            client.call_async(Empty.Request())
            self.get_logger().warn('已请求全局重定位（AMCL 重新撒粒子）')
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('请求重定位失败：%s' % exc)

    def _hold_still(self, now):
        """异常状态且手上没动作时，持续发零速占住速度通道。

        `cmd_mux` 只在"恢复这一侧最近 0.5 s 还有指令"时才让路，所以这里要
        一直续；真机上手抖发一条零速是不够的。
        """
        # 只有"确认了"才摁车。以前连 suspect 也发零速，一条误判就能让
        # cmd_mux 把速度通道抢过去，车原地一顿、Nav2 目标被取消——真机上
        # 表现为"走走停停、建图半天建不完"。疑似阶段只记录，不动车。
        if self.machine.state != 'recovery' or self.machine.pending_action:
            return
        if now - self._last_hold < 0.1:
            return
        self._last_hold = now
        tw = Twist()
        self.slip_cmd_pub.publish(tw)

    def _on_failed(self, now, action):
        """方案 §7：两次恢复都不行就彻底停车，并把故障发出去等人工。"""
        if getattr(self, '_failed_reported', False):
            return
        self._failed_reported = True
        self._cancel_nav_goal()
        payload = {
            'event': 'recovery_failed',
            'time': now,
            'type': self.machine.type,
            'reason': action.get('reason'),
            'stats': self.machine.snapshot().get('stats'),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, default=float)
        self.fault_pub.publish(msg)
        self.get_logger().error(
            '打滑恢复失败，已停车并上报（%s），等人工处理'
            % action.get('reason'))

    def _cancel_nav_goal(self):
        """把 Nav2 手上的目标取消掉，免得它一直判"无进展"再调恢复行为。

        `rclpy.action.ActionClient` 没有 `cancel_goal_async()`——取消是**目标句柄**
        的活，而目标不是我们发的（是 BT 发的），拿不到句柄。所以走 action 自带的
        `/_action/cancel_goal` 服务，用全零 UUID 表示"这个 action 上的所有目标"。
        """
        if not self._cancel_nav:
            return
        try:
            from action_msgs.srv import CancelGoal
            from unique_identifier_msgs.msg import UUID
            name = '%s/_action/cancel_goal' % self._nav_action
            if self._cancel_client is None:
                self._cancel_client = self.create_client(CancelGoal, name)
            if not self._cancel_client.service_is_ready():
                self._cancel_client.wait_for_service(timeout_sec=0.5)
            req = CancelGoal.Request()
            req.goal_info.goal_id = UUID(uuid=[0] * 16)
            req.goal_info.stamp.sec = 0
            req.goal_info.stamp.nanosec = 0
            self._cancel_client.call_async(req)
            self.get_logger().warn('已请求取消 Nav2 当前目标，避免它继续判无进展')
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('取消 Nav2 目标失败：%s' % exc)

    def _dispatch(self, action):
        """把动作发给执行器；执行器做完会往 report_topic 回一条。"""
        self._action_id += 1
        payload = {
            'id': self._action_id,
            'kind': action.get('action'),
            'distance': action.get('distance'),
            'yaw': action.get('yaw'),
            'speed': action.get('speed'),
            'timeout_s': action.get('timeout_s'),
            'type': action.get('type'),
            'why': action.get('reason'),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, default=float)
        self.act_pub.publish(msg)

    def _on_report(self, msg):
        try:
            rep = json.loads(msg.data)
        except ValueError:
            return
        if rep.get('id') != self._action_id:
            return                      # 过期回执（上一轮的动作）
        now = self._now()
        ok = bool(rep.get('ok'))
        moved = rep.get('moved')
        self.get_logger().warn(
            '动作回执：%s ok=%s moved=%.3f（%s）'
            % (rep.get('kind'), ok, float(moved or 0.0), rep.get('why') or ''))
        self.machine.on_action_done(now, ok, moved=moved)

    def _evaluate(self, win):
        fit = None
        if win.get('scan_ok'):
            try:
                fit = ranges_mod.fit_translation(
                    win['scan_prev'], win['scan_cur'], self.ranges_params)
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().error('扫描配准失败：%s' % exc)
                fit = None
        if not win.get('wheel_ok'):
            # 没有左右轮数据时 P3 自动失效（criteria 里靠 wheel_ok 判断）
            win['wheel_ok'] = False
        return criteria_mod.evaluate(win, None, fit)

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _snapshot(self):
        snap = {
            'state': self.machine.state,
            'type': self.machine.type,
            'recoveries': self.machine.recoveries,
            'slip_count': self.machine.slip_count,
        }
        return snap

    def _publish(self, win, verdict, action=None):
        snap = self._snapshot()
        fit = (verdict.get('detail') or {}).get('fit') or {}
        act = dict(action or {})
        if act.get('action'):
            act_out = {'kind': act.get('action'), 'why': act.get('reason')}
            for k in ('distance', 'yaw', 'speed', 'timeout_s'):
                if act.get(k) is not None:
                    act_out[k] = _round(act.get(k))
        elif self.machine.state == 'slip' and verdict.get('slip'):
            # 别把原因写死成 recovery_enabled=false：真机上恢复是开着的，
            # 只是还没凑够确认条件（2 窗且跨度够），那句话误导了半天。
            span = 0.0
            if self.machine.first_slip_at is not None and win.get('t1') is not None:
                span = win['t1'] - self.machine.first_slip_at
            act_out = {'kind': None,
                       'why': '确认中（%d/%d 窗，跨度 %.2f/%.2f s）'
                              % (self.machine.slip_count,
                                 int(self.machine.p['confirm_windows']),
                                 span, self.machine.p['confirm_window_s'])}
        else:
            act_out = {'kind': None, 'why': '无动作'}

        payload = {
            'stamp': round(win.get('t1', 0.0), 3),
            'state': snap['state'],
            'slip': bool(verdict.get('slip')),
            'type': verdict.get('type'),
            'primary_type': snap['type'],
            'recoveries': snap['recoveries'],
            'ratios': {k: round(float(v), 3)
                       for k, v in (verdict.get('ratios') or {}).items()},
            'reasons': list(verdict.get('reasons') or [])[:4],
            'action': act_out,
            'fit': {
                'ok': bool(fit.get('ok')),
                'dx': _round(fit.get('dx')),
                'dy': _round(fit.get('dy')),
                'dth': _round(fit.get('dtheta')),
                'quality': _round(fit.get('quality')),
                'coverage': _round(fit.get('coverage')),
            },
            'inputs': {
                'wheel_hz': round(self._wheel_hz, 1),
                'have_wheel': self._have_wheel,
                'rates': self._rates(),
                'stamp_offset': round(self._stamp_offset, 2),
            },
            'last_reason': self._last_reason,
            'map_paused': self.slam_guard.paused,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

    def _heartbeat(self):
        """每秒一条：让上层（控制台/日志）知道节点活着、看得见采样率。

        即使还没有任何窗口（一路数据都没来），也要发——否则上层看到的是
        一片沉默，分不清"没滑"和"没数据"。
        """
        now = self._now()
        if self._have_wheel and self._wheel_hz < self.min_wheel_hz:
            if now - self._warned_low_rate > WHEEL_WARN_PERIOD:
                self._warned_low_rate = now
                self.get_logger().warn(
                    '/wheel_joints 只有 %.1f Hz（要求 >= %.1f），'
                    '单轮判据 P3 可能误判' % (self._wheel_hz, self.min_wheel_hz))
        verdict = self._last_verdict or {
            'slip': False, 'type': None, 'ratios': {}, 'reasons': [],
            'detail': {}, 'usable': False,
        }
        self._publish({'t1': now}, verdict, None)

        # 每 debug_period_s 往日志里打一行：分不清"没滑"和"没数据"时，
        # 这行是唯一能让人一眼看出来的证据
        period = self._debug_period
        if period > 0 and now - self._last_debug >= period:
            self._last_debug = now
            r = self._rates()
            self.get_logger().info(
                '采样 odom %.1f wheel %.1f imu %.1f scan %.1f cmd %.1f Hz｜'
                '状态 %s｜最近原因 %s｜窗 t0=%.2f now=%.2f 样本 odom=%d cmd=%d 首 %.2f 末 %.2f'
                % (r.get('odom', 0.0), r.get('wheel', 0.0), r.get('imu', 0.0),
                   r.get('scan', 0.0), r.get('cmd', 0.0),
                   (self._last_verdict or {}).get('type') or '无',
                   self._last_reason or '无',
                   self.agg.t0 if self.agg.t0 is not None else -1.0, now,
                   len(self.agg.odom.items), len(self.agg.cmd.items),
                   (self.agg.odom.items[0][0] if self.agg.odom.items else -1.0),
                   (self.agg.odom.items[-1][0] if self.agg.odom.items else -1.0)))

    def _rates(self):
        """最近一秒各路输入的实际频率（一路没来就是 0.0）。"""
        now = self._now()
        if not self._rate_ts:
            self._rate_ts = now
            self._alive_prev = dict(self._alive)
            self._rate_cache = {k: 0.0 for k in self._alive}
            return self._rate_cache
        span = now - self._rate_ts
        if span < 0.2:
            return self._rate_cache
        self._rate_cache = {
            k: round((self._alive[k] - self._alive_prev.get(k, 0)) / span, 1)
            for k in self._alive
        }
        self._alive_prev = dict(self._alive)
        self._rate_ts = now
        return self._rate_cache


def _round(v):
    return None if v is None else round(float(v), 3)


def main(argv=None):
    rclpy.init(args=argv)
    node = SlipNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                          # noqa: BLE001
            pass
        try:
            rclpy.try_shutdown()
        except Exception:                          # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
