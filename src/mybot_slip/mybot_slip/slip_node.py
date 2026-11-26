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

from collections import deque
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
        self.declare_parameter('pause_on_suspect', False)  # 已废弃：方案 §10.1 不允许停建图
        self.declare_parameter('suspect_hold_s', 1.5)
        self.declare_parameter('max_window_s', 1.5)     # 里程计稀疏时窗口最多拉这么长
        self.declare_parameter('min_odom_samples', 2)   # 窗口里至少要几条 odom
        self.declare_parameter('alternate_after_fail', True)  # 失败后换姿势
        self.declare_parameter('window_s', 0.4)          # 方案 §12：Tw 0.25~0.40 s，取上限
        self.declare_parameter('min_samples', 2)
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('min_cmd_coverage', 0.70)
        self.declare_parameter('min_wheel_hz', 8.0)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('recovery_enabled', False)
        # 打滑期间是否暂停建图——方案 §10.1 明确禁止停 SLAM/切激光，保留只为兼容
        self.declare_parameter('pause_slam_on_slip', False)
        self.declare_parameter('slam_pause_service',
                               '/slam_toolbox/pause_new_measurements')
        self.declare_parameter('slam_pause_max_s', 15.0)
        # 方案 §11.2：进恢复就冻结 Nav2 的 progress_checker 计时
        self.declare_parameter('freeze_progress_checker', True)
        self.declare_parameter('progress_checker_node', '/controller_server')
        self.declare_parameter('progress_checker_param',
                               'progress_checker.movement_time_allowance')
        self.declare_parameter('progress_checker_hold_s', 3600.0)
        self.declare_parameter('progress_checker_normal_s', 25.0)
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
        # 方案 §12：IMU 超时 100 ms / 车身倾斜 10°
        self.declare_parameter('imu_timeout_s', 0.10)
        self.declare_parameter('max_tilt_deg', 10.0)
        # 《补充说明》§3/§4：可信运动状态 + 建图门控 + 指标
        self.declare_parameter('trusted_state', True)
        self.declare_parameter('map_gate', True)
        self.declare_parameter('map_scan_topic', '/scan_safe')
        self.declare_parameter('metrics_topic', '/slip/metrics')
        self.declare_parameter('suspect_speed_cap', 0.10)
        self.declare_parameter('suspect_yaw_cap', 0.50)

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

        # 《补充说明》§3：可信运动状态与打滑状态机并行维护
        from . import trusted as trusted_mod
        self.trusted_mod = trusted_mod
        self.trusted = trusted_mod.TrustedState({
            'suspect_speed_cap': float(gp('suspect_speed_cap').value),
            'suspect_yaw_cap': float(gp('suspect_yaw_cap').value),
        })
        self._map_allowed = True
        # §9 六项指标（只记在节点里，不上页面）
        self.metrics = {
            'pose_pollution_m': 0.0,   # 确认前定位污染量（疑似窗里被当成位移的编码器距离）
            'windows': 0,
            'windows_blocked': 0,      # 被门控判为不可用的窗
            'admission_try': 0,
            'admission_reject': 0,
            's4_safe_stops': 0,
            'nav_blocked_s': 0.0,
            'map_frames_dropped': 0,   # 因不可信而被挡在建图之外的帧
        }
        self._last_nav_blocked_at = None
        self._last_trusted = 'S0'
        self._s4_reported = False

        # 方案 §10.1 明确要求「而不是停止 SLAM 节点或切断激光输入」，
        # 所以不再构造 slam_guard / PausePolicy；保护地图靠 odom_guard 放协方差。
        # 这两个参数只为兼容旧 launch 保留，读到也只告警。
        if bool(self.get_parameter('pause_slam_on_slip').value):
            self.get_logger().warn(
                'pause_slam_on_slip 已按方案 §10.1 停用：打滑时不再暂停 SLAM，'
                '改用放大里程协方差把错误位移挡在位姿链外')

        # ---------------- 冻结 Nav2 progress_checker ----------------
        # 方案 §11.2：进 SLIP_DETECTED 时冻结 progress_checker 计时，
        # 免得 Nav2 判"无进展"去调内建 recovery 抢控制权。
        self._pc_frozen = None          # None=还没设过，True/False=当前状态
        self._pc_pending = False
        self._pc_client = None
        self._pc_warned = False

        # ---------------- 状态 ----------------
        self._have_wheel = False
        self._wheel_last_t = None
        self._wheel_hz = 0.0
        self._wheel_hz_ts = 0.0
        self._warned_low_rate = 0.0
        self._last_verdict = {}
        self._last_action = None
        self._pending_dispatch = None
        self._last_state = 'normal'
        self.wheel_seen = False
        self.scan_name = self.p('scan_topic')
        self._last_reason = ''
        self._alive = {'odom': 0, 'wheel': 0, 'imu': 0, 'scan': 0, 'cmd': 0}
        self._alive_prev = {}
        self._rate_ts = 0.0
        self._rate_cache = {}
        self._last_stamps = {}
        self._imu_tilt = None       # 最近一次由加速度估出的车身倾角（度）
        self._acc_hist = deque(maxlen=300)   # (t, ||a||-g)，给 P1-b 判"车体是否真的没动"
        self._imu_tilt_at = 0.0
        self._acc_f = None          # 加速度一阶低通的状态（滤掉行进冲击，只留重力方向）
        self._tilt_bias = None      # IMU 安装倾角基准（这台车静止时就有 ~16°）
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

        # 《补充说明》§3：地图门控——只有可信状态允许时才把激光转给 slam
        self.map_scan_pub = None
        self.map_gate = bool(gp('map_gate').value)
        if self.map_gate:
            self.map_scan_pub = self.create_publisher(
                LaserScan, gp('map_scan_topic').value, qos_profile_sensor_data)
        self.metrics_pub = self.create_publisher(
            String, gp('metrics_topic').value, QoSProfile(depth=10))

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
        self._conservative_reported = False
        self._guard_warned = False

        # 20 Hz 轮询：窗口是 0.3 s，20 Hz 足够不丢边界
        self.create_timer(0.05, self._tick)
        self.create_timer(gp('publish_period_s').value, self._heartbeat)

        self.get_logger().info(
            '打滑检测已启动：窗 %.2fs（最长 %.2fs，至少 %d 条 odom），%s，判据 P1~P5，'
            '恢复=%s，协方差降权=%s，冻结progress=%s，激光=%s'
            % (self.win_params['window_s'], self.win_params['max_window_s'],
               self.win_params['min_odom_samples'],
               '含左右轮' if self.wheel_seen else '无左右轮',
               '开' if self.recovery_enabled else '关（只报不动）',
               '开' if bool(self.p('covariance_guard')) else '关',
               '开' if bool(self.p('freeze_progress_checker')) else '关',
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
        # 方案 §12：tilt 用加速度的重力方向估，超过 10° 说明 2D 假设不成立
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 1e-3:
            # 加速度计测到的是"重力 + 运动加速度"。加速/顶住箱子时水平分量能到 1.7 m/s²，
            # 直接 atan2 会算成 10° 以上、把整窗判成"倾斜不可用"（实测就卡在这）。
            # 用一阶低通（≈0.4 s）只留缓慢的重力方向变化，滤掉行进中的冲击。
            if self._acc_f is None:
                self._acc_f = [ax, ay, az]
            else:
                a = 0.15
                self._acc_f[0] += a * (ax - self._acc_f[0])
                self._acc_f[1] += a * (ay - self._acc_f[1])
                self._acc_f[2] += a * (az - self._acc_f[2])
            fx, fy, fz = self._acc_f
            horiz = math.sqrt(fx * fx + fy * fy)
            raw = math.degrees(math.atan2(horiz, abs(fz)))
            # IMU 装歪 16° 是这台车的常态（实测静止时 ax=-2.8、az=10.2）。
            # 方案 §12 说的是"车身倾斜"，所以要减掉安装基准，只留**变化量**。
            if self._tilt_bias is None:
                self._tilt_bias = raw
            else:
                self._tilt_bias += 0.02 * (raw - self._tilt_bias)
            self._imu_tilt = abs(raw - self._tilt_bias)
            self._imu_tilt_at = t
        # P1-b 的前提：车体是否真的静止。记 |a| 相对重力的偏离，按窗算 RMS。
        # 注意用未滤波的原始值——滤波会把振动抹掉，就没法区分"车没动"和"车在跑"。
        self._acc_hist.append((t, abs(norm - 9.81)))
        while len(self._acc_hist) > 2 and t - self._acc_hist[0][0] > 2.0:
            self._acc_hist.popleft()

    def _on_scan(self, msg):
        t = self._stamp(msg, self._now())
        self._alive['scan'] += 1
        self._last_stamps['scan'] = t
        if not msg.ranges:
            return
        self.agg.add_scan(t, msg)
        # §3：S1/S3/S4 暂停持久地图写入（原样给 slam 的那一路掐掉）
        if self.map_scan_pub is not None:
            if self._map_allowed:
                self.map_scan_pub.publish(msg)
            else:
                self.metrics['map_frames_dropped'] += 1

    def _on_cmd(self, msg):
        t = self._now()
        self._alive['cmd'] += 1
        self.agg.add_cmd(t, msg.linear.x, msg.angular.z)

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _tick(self):
        now = self._now()
        verdict = None
        # 上一轮动作的后续（第二次后退 / 换姿势 / inflate）在这里下发：
        # 状态机在 recovery 里不会再返回动作，只能靠这个接。
        if self._pending_dispatch is not None:
            act = self._pending_dispatch
            self._pending_dispatch = None
            self._last_action = act
            self._dispatch(act)
            self.get_logger().warn(
                '打滑恢复动作：%s（%s）'
                % (act.get('action'), act.get('reason') or ''))
        try:
            wins = self.agg.poll(now)
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error('窗口生成失败：%s' % exc)
            return

        for win in wins:
            # 方案 §12：IMU 超时 100 ms / 倾斜 >10° 时这一窗不判
            imu_at = self._last_stamps.get('imu')
            win['imu_stale'] = (imu_at is None) or (now - imu_at > self.p('imu_timeout_s'))
            win['tilt_deg'] = self._imu_tilt
            # P1-b 的前提量：这一窗里车体"晃不晃"。顶着箱子时 1~2 m/s²，
            # 正常行驶 4~6 m/s²；长走廊直行靠它把"视野没变"和"车没动"分开。
            t0w, t1w = win.get('t0'), win.get('t1')
            if t0w is not None and t1w is not None and self._acc_hist:
                devs = [d for (tt, d) in self._acc_hist if t0w <= tt <= t1w]
                if devs:
                    win['acc_rms'] = math.sqrt(
                        sum(d * d for d in devs) / len(devs))
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

            # 《补充说明》§3：每窗更新一次可信运动状态
            self.metrics['windows'] += 1
            if not win.get('valid'):
                self.metrics['windows_blocked'] += 1
            confirmed_now = self.machine.slip_count >= int(
                self.machine.p.get('confirm_windows', 2))
            tstate, treason = self.trusted.update(
                verdict,
                machine_state=self.machine.state,
                confirmed=confirmed_now,
                scan_ok=self._scan_fresh(now),
                imu_ok=not win.get('imu_stale'),
                known_reason=action.get('reason', ''))
            if tstate in (self.trusted_mod.S1, self.trusted_mod.S3) and verdict.get('slip'):
                # §9「确认前定位污染量」：疑似窗里被当成位移的编码器距离
                self.metrics['pose_pollution_m'] += float(win.get('denc') or 0.0)
            if tstate != getattr(self, '_last_trusted', None):
                self.get_logger().info('可信状态 %s → %s（%s）'
                                       % (getattr(self, '_last_trusted', 'S0') or 'S0',
                                          tstate, treason))
                self._last_trusted = tstate
            self._map_allowed = tstate in self.trusted_mod.MAP_ALLOWED

            # §5.2：要动之前先过准入；不过就停车上报，不再试第二次
            kind = action.get('action')
            if kind in ('back', 'escape', 'go', 'probe'):
                self.metrics['admission_try'] += 1
                ok, why = self.trusted_mod.check_admission(
                    verdict, state=tstate,
                    scan_ok=self._scan_fresh(now),
                    imu_ok=not win.get('imu_stale'),
                    tilt_deg=float(self._imu_tilt or 0.0),
                    consecutive_fail=self.machine.recoveries - 1,
                    max_fail=int(self.machine.p['max_recovery']),
                    max_tilt_deg=float(self.p('max_tilt_deg')))
                if not ok:
                    self.metrics['admission_reject'] += 1
                    self.get_logger().warn('恢复准入不通过：%s' % why)
                    action = self.machine.deny(now, '恢复准入不通过：%s' % why)
                    kind = 'abort'

            if state_before != self.machine.state and kind != 'abort':
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

        # 方案 §11.2：一确认就往"恢复"状态，冻结 Nav2 的 progress_checker 计时，
        # 免得它一边判"无进展"一边调内建 recovery 来抢控制权。
        # 方案 §11.3 才提到取消目标（台阶卡死→重规划），所以这里不取消。
        if self.machine.state == 'recovery':
            if not self._nav_cancelled:
                self._nav_cancelled = True
                self._recovery_pose = self._pose
            self._freeze_progress(True)
        else:
            if self._nav_cancelled:
                self._nav_cancelled = False
                self._freeze_progress(False)
            if self.machine.state == 'normal':
                self._failed_reported = False

        # 方案 §10 三档处理（不是"停 SLAM、切激光"）：
        #   轻度(is_normal=False 但还没确认) → 只放大协方差
        #   严重(确认后 recovery)            → 冻结里程对融合位姿的更新
        #   激光+IMU 都不可信                → 冻结 + 停车 + 上报，等人工
        abnormal = self.machine.state != 'normal'
        conservative = bool((verdict or {}).get('type') == 'SENSOR_CONFLICT')
        # "严重"= 已经确认（连续 Nconfirm 个窗都是滑移），不只是疑似；
        # 关掉恢复动作时状态机停在 SLIP，但冻结照样要生效。
        confirmed = self.machine.slip_count >= int(
            self.machine.p.get('confirm_windows', 2))
        # 《补充说明》§5.1：S1（候选）就要"暂缓位姿修正"——等确认再冻结已经晚了一窗，
        # 实测 20:30 那个包里确认前后各一窗的错误位移照样进了图（422 s 那 0.33 m）。
        trusted = getattr(self, '_last_trusted', 'S0') or 'S0'
        severe = trusted != 'S0' or conservative

        if self.odom_guard is not None:
            self.odom_guard.set_inflated(abnormal)
            self.odom_guard.set_frozen(severe)

        if conservative:
            # 保守模式：不再自动往前开，停车并上报一次
            self._hold_still(now, force=True)
            if not self._conservative_reported:
                self._conservative_reported = True
                self._cancel_nav_goal()
                self._on_failed(now, {'reason': '激光与 IMU 都不可信，进入保守模式：'
                                                '停车、限速、禁止自动导航，等人工'})
        elif self.machine.state == 'normal':
            self._conservative_reported = False

        # §3 S4：不可判定 —— 安全停车、取消任务、上报，且禁止自动恢复
        if self.trusted.state == self.trusted_mod.S4:
            self._hold_still(now, force=True)
            if not self._s4_reported:
                self._s4_reported = True
                self.metrics['s4_safe_stops'] += 1
                self._cancel_nav_goal()
                self._on_failed(now, {'reason': 'S4 不可判定（%s）：安全停车、'
                                                '取消当前任务并上报，禁止自动恢复'
                                                % self.trusted.reason})
        elif self.trusted.state in (self.trusted_mod.S0, self.trusted_mod.S1):
            self._s4_reported = False

    def p(self, name):
        return self.get_parameter(name).value

    def _scan_fresh(self, now):
        """独立位移观测是否还在（《补充说明》§5.1）。"""
        at = self._last_stamps.get('scan')
        return at is not None and (now - at) <= 1.0

    def _publish_metrics(self, now):
        """§9：单独一档指标，方便验收脚本直接读，不用解析状态串。"""
        state = self.trusted.state
        if not self.trusted.nav_allowed:
            if self._last_nav_blocked_at is None:
                self._last_nav_blocked_at = now
        else:
            if self._last_nav_blocked_at is not None:
                self.metrics['nav_blocked_s'] += now - self._last_nav_blocked_at
                self._last_nav_blocked_at = None
        msg = String()
        msg.data = json.dumps({
            'stamp': round(now, 3),
            'trusted_state': state,
            'trusted_reason': self.trusted.reason,
            **{k: (round(v, 3) if isinstance(v, float) else v)
               for k, v in self.metrics.items()},
        }, ensure_ascii=False)
        self.metrics_pub.publish(msg)

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

    def _hold_still(self, now, force=False):
        """异常状态且手上没动作时，持续发零速占住速度通道。

        `cmd_mux` 只在"恢复这一侧最近 0.5 s 还有指令"时才让路，所以这里要
        一直续；真机上手抖发一条零速是不够的。
        """
        # 只有"确认了"才摁车。以前连 suspect 也发零速，一条误判就能让
        # cmd_mux 把速度通道抢过去，车原地一顿、Nav2 目标被取消——真机上
        # 表现为"走走停停、建图半天建不完"。疑似阶段只记录，不动车。
        # force=True 是保守模式（激光+IMU 都不可信）用的：那时不管状态机
        # 在哪一档，都必须先停住。
        if not force:
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

    def _freeze_progress(self, want):
        """方案 §11.2：冻结/恢复 Nav2 progress_checker 的计时。

        做法是把 `controller_server` 上的 `progress_checker.movement_time_allowance`
        临时调到很大（默认 3600 s），这样它不会因为"车没动够 0.25 m"去判失败、
        进而调内建 recovery 跟我们的恢复动作抢 `/cmd_vel`（§11.1 的优先级只在我们
        自己这一侧成立，Nav2 不知道）。回 NORMAL 再改回原值。
        """
        if not bool(self.p('freeze_progress_checker')):
            return
        if self._pc_frozen == want or self._pc_pending:
            return
        node = str(self.p('progress_checker_node'))
        param = str(self.p('progress_checker_param'))
        value = (float(self.p('progress_checker_hold_s')) if want
                 else float(self.p('progress_checker_normal_s')))
        try:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            if self._pc_client is None:
                self._pc_client = self.create_client(
                    SetParameters, '%s/set_parameters' % node)
            if not self._pc_client.service_is_ready():
                if not self._pc_warned:
                    self._pc_warned = True
                    self.get_logger().warn(
                        '%s 不在，冻结 progress_checker 跳过（不影响打滑检测）' % node)
                return
            req = SetParameters.Request()
            val = ParameterValue()
            val.type = ParameterType.PARAMETER_DOUBLE
            val.double_value = value
            one = Parameter()
            one.name = param
            one.value = val
            req.parameters = [one]
            self._pc_pending = True
            future = self._pc_client.call_async(req)
            future.add_done_callback(
                lambda fut, w=want: self._on_pc_done(fut, w))
        except Exception as exc:                      # noqa: BLE001
            self._pc_pending = False
            self.get_logger().warn('冻结 progress_checker 失败：%s' % exc)

    def _on_pc_done(self, future, want):
        self._pc_pending = False
        try:
            future.result()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('冻结 progress_checker 请求失败：%s' % exc)
            return
        self._pc_frozen = want
        self.get_logger().info(
            'Nav2 progress_checker %s（防止它判无进展去抢控制权）'
            % ('已冻结计时' if want else '计时已恢复'))

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
        if rep.get('blocked') and rep.get('kind') == 'back':
            # §9.4 后方不安全：不许后退，改成原地转向脱困（同一次恢复机会）
            self.get_logger().warn('后方不放行，改为原地转向脱困')
            self.machine.pending_action = {
                'action': 'escape',
                'yaw': float(self.machine.p['escape_yaw_rad']),
                'speed': float(self.machine.p['escape_speed']),
                'timeout_s': 6.0,
                'why': '后方 0.30 m 内有障碍，改原地转向',
            }
            self.machine.last_action = 'escape'
            return
        self.machine.on_action_done(now, ok, moved=moved)
        # on_action_done 可能已经排了下一个动作（第二次后退 / 换姿势 / inflate），
        # 但状态机在 recovery 里只会回 "waiting action"——这里接住它，
        # 交给下一次 _tick 真正下发（否则会一直挂着，直到超时）。
        nxt = self.machine.pending_action
        if (isinstance(nxt, dict)
                and nxt.get('action') in ('back', 'escape', 'inflate',
                                          'stop', 'probe', 'go')):
            self._pending_dispatch = dict(nxt)

    def _evaluate(self, win):
        fit = None
        if win.get('scan_ok'):
            try:
                # 两个初值各跑一次，取质量高的那个：
                # - 零初值：打滑时扫描没动，残差最小，能如实报出"没动"；
                # - 里程计初值：正常行驶时避开长走廊局部极小，横向估计更准。
                fit = ranges_mod.fit_translation(
                    win['scan_prev'], win['scan_cur'], self.ranges_params)
                if win.get('odom_ok'):
                    init = (float(win.get('denc') or 0.0), 0.0,
                            float(win.get('dtheta_enc') or 0.0))
                    fit2 = ranges_mod.fit_translation(
                        win['scan_prev'], win['scan_cur'],
                        self.ranges_params, init=init)
                    if (fit2.get('quality', 0.0) > fit.get('quality', 0.0)
                            + 1e-6):
                        fit = fit2
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
            # 《补充说明》§3：可信运动状态（S0~S4）与打滑状态并行
            'trusted_state': self.trusted.state,
            'trusted_reason': self.trusted.reason,
            'nav_allowed': self.trusted.nav_allowed,
            'map_allowed': self.trusted.map_allowed,
            # 冻结要真生效，前提是 EKF 的 odom0 接在 /odom_guarded 上；
            # 订阅者为 0 时这里为 0，等于"保护没接上"，别再信"已冻结"。
            'guard_subs': (self.odom_guard.subscriber_count()
                           if self.odom_guard is not None else 0),
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
            # 方案 §10.1：建图不再暂停，这里保留字段给旧页面兼容，恒为 False
            'map_paused': False,
            'progress_frozen': bool(self._pc_frozen),
            # 《补充说明》§9：六项指标随状态一起发
            'metrics': {
                'pose_pollution_m': round(self.metrics['pose_pollution_m'], 3),
                'windows': self.metrics['windows'],
                'windows_blocked': self.metrics['windows_blocked'],
                'admission_try': self.metrics['admission_try'],
                'admission_reject': self.metrics['admission_reject'],
                's4_safe_stops': self.metrics['s4_safe_stops'],
                'nav_blocked_s': round(self.metrics['nav_blocked_s'], 1),
                'map_frames_dropped': self.metrics['map_frames_dropped'],
            },
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
        self._publish_metrics(now)

        # 每 debug_period_s 往日志里打一行：分不清"没滑"和"没数据"时，
        # 这行是唯一能让人一眼看出来的证据
        period = self._debug_period
        if period > 0 and now - self._last_debug >= period:
            self._last_debug = now
            r = self._rates()
            fit = ((self._last_verdict or {}).get('detail') or {}).get('fit') or {}
            if fit:
                fit_txt = ('配准 q=%.2f 内点=%d 覆盖=%.2f 扫描位移=%.3f m'
                           ' dx=%+.3f dy=%+.3f' % (
                               fit.get('quality', 0.0), fit.get('beams', 0),
                               fit.get('coverage', 0.0), fit.get('motion', -1.0),
                               fit.get('dx') or 0.0, fit.get('dy') or 0.0))
            else:
                fit_txt = '配准 无'
            if self.odom_guard is not None:
                subs = self.odom_guard.subscriber_count()
                if subs == 0 and not self._guard_warned:
                    self._guard_warned = True
                    self.get_logger().error(
                        '没人订阅 %s：EKF 的 odom0 还在听 /odom，'
                        '协方差放大和冻结都不会生效！'
                        '请用 robot.launch.py（with_slip:=true）启动，'
                        '或手动传 navigation 的 ekf_odom_topic:=%s'
                        % (self.p('odom_guard_out'), self.p('odom_guard_out')))
                elif subs > 0:
                    self._guard_warned = False
            self.get_logger().info(
                '采样 odom %.1f wheel %.1f imu %.1f scan %.1f cmd %.1f Hz｜'
                '状态 %s｜冻结 %s｜%s｜最近原因 %s｜窗 t0=%.2f now=%.2f 样本 odom=%d cmd=%d 首 %.2f 末 %.2f｜'
                'scan 帧 %d 最新 %.2f 窗内指令覆盖 %.2f'
                % (r.get('odom', 0.0), r.get('wheel', 0.0), r.get('imu', 0.0),
                   r.get('scan', 0.0), r.get('cmd', 0.0),
                   self._last_state or 'normal',
                   '是' if (self.odom_guard is not None
                            and self.odom_guard.frozen) else '否',
                   fit_txt,
                   self._last_reason or '无',
                   self.agg.t0 if self.agg.t0 is not None else -1.0, now,
                   len(self.agg.odom.items), len(self.agg.cmd.items),
                   (self.agg.odom.items[0][0] if self.agg.odom.items else -1.0),
                   (self.agg.odom.items[-1][0] if self.agg.odom.items else -1.0),
                   len(self.agg.scan.items),
                   (self.agg.scan.items[-1][0] if self.agg.scan.items else -1.0),
                   float(((self._last_verdict or {}).get('detail') or {})
                         .get('cmd_coverage', -1.0))))

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
