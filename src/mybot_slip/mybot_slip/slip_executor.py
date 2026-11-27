#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""执行器：把 /slip/action 变成真正的运动，再把结果报回去。

    slip_node  ──/slip/action (JSON)──>  slip_executor
                                          │
                                          ├─ back   : 直发 /cmd_vel 倒车，按里程计闭环
                                          ├─ escape : 原地反向转，按陀螺/里程计闭环
                                          │
    slip_node  <──/slip/report (JSON)─────┘

为什么直发 cmd_vel 而不是用 Nav2 的 BackUp / Spin：
这两者要走 `navigate_to_pose` 的生命周期与 BT，而打滑时车往往正处于
一次导航任务里——借用 Nav2 动作会和当前任务抢执行权。这里只发速度指令，
Nav2 那边由它自己的碰撞检查兜底（真机验证时再看是否需要接管）。

安全：动作只在 `recovery_enabled` 打开时才会被下发；动作结束（成功/超时/
被新指令打断）一定发一帧 0 速度，绝不让车保持惯性指令。
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from mybot_slip import ranges as ranges_mod
from mybot_slip.motion import TravelMonitor


IDLE = 'idle'
RUNNING = 'running'


def _yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    # roll/pitch 视为 0（地面车），只要 yaw
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SlipExecutor(Node):
    def __init__(self):
        super().__init__('slip_executor')
        gp = self.declare_parameter
        gp('action_topic', '/slip/action')
        gp('report_topic', '/slip/report')
        gp('cmd_topic', '/cmd_vel')
        gp('odom_topic', '/odom')
        gp('imu_topic', '/imu')
        gp('scan_topic', '/scan')
        gp('rate_hz', 20.0)
        gp('back_speed', 0.08)
        gp('escape_speed', 1.2)
        gp('settle_s', 0.3)
        gp('measure', 'scan')           # scan=用激光量退了多少；odom=用里程计
        gp('foreign_tol', 0.03)         # /cmd_vel 上别人发的值与我们的差多少算"有人在抢"
        gp('stuck_after_s', 2.0)        # 动作开始这么久还没动静就提前放弃
        gp('dry_run', False)            # True：只打日志不发 cmd_vel
        gp('inflate_radius', 0.55)      # inflate 动作把膨胀半径临时改成多少
        gp('inflate_hold_s', 8.0)       # 保持多久后还原
        gp('inflate_costmaps', 'local_costmap,global_costmap')   # 要动的代价地图（逗号分隔）
        gp('inflate_param', 'inflation_layer.inflation_radius')
        gp('costmap_prefix', '')        # 多机加命名空间时填 /robot1 之类

        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dry_run = bool(get('dry_run'))
        self.measure = str(get('measure')).lower()
        self._measure_default = self.measure
        self.foreign_tol = float(get('foreign_tol'))
        self.stuck_after_s = float(get('stuck_after_s'))
        self.back_speed = float(get('back_speed'))
        self.escape_speed = float(get('escape_speed'))
        self.settle_s = float(get('settle_s'))
        self.inflate_radius = float(get('inflate_radius'))
        self.inflate_hold_s = float(get('inflate_hold_s'))
        self.inflate_param = str(get('inflate_param'))
        self.costmap_prefix = str(get('costmap_prefix')).rstrip('/')
        self.inflate_costmaps = [s.strip() for s in
                                 str(get('inflate_costmaps')).split(',') if s.strip()]
        rate = float(get('rate_hz'))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, get('action_topic'), self._on_action, 10)
        self.create_subscription(Odometry, get('odom_topic'), self._on_odom, qos)
        self.create_subscription(Imu, get('imu_topic'), self._on_imu, 20)
        self.create_subscription(LaserScan, get('scan_topic'), self._on_scan, 5)
        self.create_subscription(Twist, get('cmd_topic'), self._on_cmd, 20)
        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 10)
        self.report_pub = self.create_publisher(String, get('report_topic'), 10)

        self.create_timer(1.0 / rate, self._tick)

        self.status = IDLE
        self.action = None
        self.monitor = None
        self.started_at = 0.0
        self.deadline = 0.0
        self.pose = None
        self.stop_until = 0.0
        self._scan_prev = None
        self._scan_used = 0          # 本动作累计用了多少帧扫描
        self._scan_drop = 0          # 配准质量不够、被丢掉的帧
        self._q_sum = 0.0            # 配准质量累加，用来算平均
        self._odom_travel = 0.0
        self._odom_last = None
        self._odom_start = None      # 动作开始时位姿，_finish 里算带符号位移要用
        self._imu_yaw = None
        self._imu_last = None
        self._expected_lin = None
        self._foreign = None
        self._inflate_saved = {}
        self._inflate_queue = []
        self._param_clients = {}
        self._rear_warned = False     # §9.4 没有激光时只告警一次
        self.get_logger().info(
            '执行器就绪：动作来自 %s，结果发 %s（位移量用 %s，%s）'
            % (get('action_topic'), get('report_topic'), self.measure,
               'dry_run，不发速度' if self.dry_run else '会发速度'))

    # ------------------------------------------------------------------
    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self.pose = (pos.x, pos.y, _yaw_from_odom(msg))
        if self._odom_last is not None:
            self._odom_travel += math.hypot(pos.x - self._odom_last[0],
                                            pos.y - self._odom_last[1])
        self._odom_last = (pos.x, pos.y)

    def _on_imu(self, msg):
        """陀螺的偏航角速度：不依赖轮子，脱困转了多少只能信它。"""
        w = msg.angular_velocity.z
        now = self._now()
        if self._imu_last is not None and self.status == RUNNING \
                and (self.action or {}).get('kind') == 'escape':
            dt = max(min(now - self._imu_last, 0.2), 0.0)
            self._imu_last = now
            if self.monitor is not None and dt > 0:
                self.monitor.add_yaw(w * dt)
        else:
            self._imu_last = now

    def _rear_blocked(self, need):
        """方案 §9.4：车体正后方 ±30° 扇区里有没有比 need 更近的东西。

        返回最近的米数（< need 就是挡住了），测不了就返回 None（放行并告警一次）。
        """
        scan = self._scan_prev
        if scan is None or not getattr(scan, 'ranges', None):
            if not self._rear_warned:
                self.get_logger().warn('没有激光数据，无法做后方安全检查（这次放行）')
                self._rear_warned = True
            return None
        a0 = float(getattr(scan, 'angle_min', 0.0) or 0.0)
        inc = float(getattr(scan, 'angle_increment', 0.0) or 0.0)
        best = None
        for i, r in enumerate(scan.ranges):
            if r is None or r != r or r == float('inf'):
                continue
            a = a0 + i * inc
            # 归一到 [-pi, pi]，只看背后 ±30°
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            if abs(abs(a) - math.pi) > math.radians(30.0):
                continue
            if best is None or r < best:
                best = r
        if best is None:
            return None
        return best if best < need else None

    def _on_scan(self, msg):
        """激光量位移：打滑时里程计正是在骗人的那一个，不能拿它验自己。"""
        prev, self._scan_prev = self._scan_prev, msg
        if self.status != RUNNING or (self.action or {}).get('kind') != 'back':
            return
        if prev is None or self.monitor is None:
            return
        try:
            fit = ranges_mod.fit_translation(
                {'ranges': list(prev.ranges), 'range_max': float(prev.range_max)},
                {'ranges': list(msg.ranges), 'range_max': float(msg.range_max)})
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn('位移测量失败：%s' % exc)
            return
        if not fit.get('ok') or (fit.get('quality') or 0.0) < 0.35:
            self._scan_drop += 1
            return
        self._scan_used += 1
        self._q_sum += float(fit.get('quality') or 0.0)
        self.monitor.add_distance(math.hypot(fit.get('dx') or 0.0,
                                             fit.get('dy') or 0.0))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        """盯 /cmd_vel 上有没有「别人」在发。

        ROS 2 没有优先级：只要还有一个节点在发前进指令，我们发的后退就会被
        它抵掉（实测表现为车一边被要求后退、一边被推着往前撞东西）。
        自己的回声和期望值一致，差得多的就是外人。
        """
        if self.dry_run or self.status != RUNNING:
            return
        expect = self._expected_lin
        if expect is None:
            return
        val = float(msg.linear.x)
        if abs(val) <= 0.02:
            # 别人只是在踩刹车（发 0），不算抢——只有真的在驱车才叫抢
            return
        diff = abs(val - expect)
        if diff <= self.foreign_tol:
            return
        first = self._foreign is None
        self._foreign = (time.monotonic(), val)
        if first:
            self.get_logger().warn(
                '/cmd_vel 上还有别人在发：他要 %.2f m/s，我要 %.2f m/s'
                % (val, expect))

    def _on_action(self, msg):
        try:
            act = json.loads(msg.data)
        except ValueError:
            self.get_logger().error('动作不是合法 JSON：%s' % msg.data[:80])
            return
        kind = act.get('kind')
        if not kind:
            return
        if self.status == RUNNING:
            # 上一次还没做完：先停住再报失败，让上层知道没执行
            self.get_logger().warn('上一次动作未完成，忽略新动作 %s' % kind)
            self._report(act, ok=False, moved=0.0, why='上一次动作还在执行')
            return
        if kind == 'abort':
            self.get_logger().warn('上层要求放弃：%s' % act.get('why'))
            if self._inflate_saved:
                self._restore_inflate()      # 别把膨胀半径留在改过的值上
            self._report(act, ok=True, moved=0.0, why='abort 已收到')
            return
        self._start(act)

    # ------------------------------------------------------------------
    def _start(self, act):
        kind = act.get('kind')
        if kind == 'inflate':
            self._start_inflate(act)
            return
        if kind == 'back':
            # 方案 §9.4：后退前先看车体后方 rear_check_m 扇区，有人/墙就别退
            blocked = self._rear_blocked(float(act.get('rear_check_m') or 0.30))
            if blocked is not None:
                self.get_logger().warn('后方安全检查不通过：%.2f m（要求 ≥%.2f m）'
                                       % (blocked, float(act.get('rear_check_m') or 0.30)))
                self._report(act, ok=False, moved=0.0,
                             why='后方 %.2f m 内有障碍，不能后退' % blocked, blocked=True)
                return
            target = float(act.get('distance') or 0.0)
            monitor = TravelMonitor('back', target)
        elif kind == 'probe':
            # 方案 §5：恢复后试探前进一小步（默认 0.05 m），走够就算完成
            target = float(act.get('distance') or 0.05)
            monitor = TravelMonitor('probe', target)
        elif kind == 'go':
            # 方案 §3：单轮打滑/横向侧滑后的"低速重试"
            target = float(act.get('distance') or 0.10)
            monitor = TravelMonitor('go', target)
        elif kind == 'escape':
            target = float(act.get('yaw') or 0.9)
            monitor = TravelMonitor('escape', target)
        elif kind == 'stop':
            # 方案 §1/§3：不再驱动，保持零速一段时间，然后回执
            self.action = dict(act)
            self.monitor = None
            self.status = RUNNING
            now = time.monotonic()
            self.started_at = now
            self.deadline = now + float(act.get('timeout_s') or 1.0)
            self._expected_lin = 0.0
            self._foreign = None
            self.get_logger().warn('执行动作 stop：停车 %.1f s（%s）'
                                   % (self.deadline - now, act.get('why') or ''))
            return
        else:
            self.get_logger().warn('不认识的动作品类：%s' % kind)
            self._report(act, ok=False, moved=0.0, why='不认识的动作品类')
            return

        now = time.monotonic()
        self.action = dict(act)
        self.monitor = monitor
        self.status = RUNNING
        self.started_at = now
        self.deadline = now + float(act.get('timeout_s') or 6.0)
        self._odom_travel = 0.0
        self._odom_last = None
        self._odom_start = self.pose     # 动作开始时的 (x, y, yaw)
        self._scan_prev = None          # 丢掉动作前的旧帧，第一对差分才算数
        self._scan_used = 0
        self._scan_drop = 0
        self._q_sum = 0.0
        self._expected_lin = None
        self._foreign = None
        self.measure = self._measure_default     # 上一个动作可能降级过，重置
        if self.measure == 'odom' and self.pose is not None:
            self.monitor.update(*self.pose)
        self.get_logger().warn('执行动作 %s：目标 %.2f（量法 %s，%s）'
                               % (kind, monitor.target, self.measure,
                                  act.get('why') or ''))

    def _tick(self):
        if self.status != RUNNING:
            if self.stop_until and time.monotonic() >= self.stop_until:
                self.stop_until = 0.0
            return
        if (self.action or {}).get('kind') == 'inflate':
            now = time.monotonic()
            if now >= self.deadline:
                self._restore_inflate()
                self._finish(ok=True, why='保持时间到，膨胀半径已还原')
            elif now > self.started_at + self.inflate_hold_s + 10.0:
                # 兜底：正常路径没走到（比如回调丢了），别一直挂着改过的值
                self._restore_inflate()
                self._finish(ok=False, why='inflate 超时未收尾，已强制还原')
            return
        now = time.monotonic()
        if (self.action or {}).get('kind') == 'stop':
            # 停车就是"持续发零速"：发一帧不够，底盘可能还在按上一条指令走
            if now >= self.deadline:
                self._stop()
                self._finish(ok=True, why='已停车 %.1f s' % (now - self.started_at))
            elif not self.dry_run:
                self._stop()
            return

        if self.measure == 'odom' and self.pose is not None:
            self.monitor.update(*self.pose)

        if self.monitor.done():
            self._finish(ok=True, why='动作完成')
            return
        if now > self.deadline:
            self._finish(ok=False, why='动作超时')
            return
        if self._foreign is not None and now - self._foreign[0] < 0.5:
            self._finish(ok=False,
                         why='别人也在发 /cmd_vel（%.2f m/s），先停掉再恢复'
                             % self._foreign[1])
            return
        if now - self.started_at > self.stuck_after_s and self.monitor.progress() < 0.01 \
                and self._odom_travel < 0.02:
            # 激光量不出、里程计也没动，才算真卡住；只看激光会把
            # "ICP 没跟上"误判成"车没动"，然后一转身就再也不后退了
            self._finish(ok=False, why='动作 %.1fs 一点没动' % (now - self.started_at))
            return

        # 激光测不出来（视野太窄、贴墙、ICP 失败）时退回里程计：
        # 少一次精确验收，也好过永远判"没动"然后不后退
        if self.measure == 'scan' and self._scan_used == 0 and self._odom_travel > 0.02 \
                and now - self.started_at > 1.0:
            self.get_logger().warn('激光这一窗没量到位移，改用里程计验收（结果仅供参考）')
            self.measure = 'odom'
            if self.pose is not None:
                self.monitor.update(*self.pose)

        act = self.action
        twist = Twist()
        if act['kind'] == 'back':
            # 按动作给的速度走（方案 §9.3：−0.15 m/s、超时 1.5 s）；
            # 用执行器自己的 back_speed 会和 1.5 s 超时打架，永远差一点点。
            twist.linear.x = -abs(float(act.get('speed') or self.back_speed))
            # 方案 §3：后退的同时小幅反向调整，别退回原路再撞同一个角
            rate = act.get('yaw_rate')
            if rate:
                twist.angular.z = float(rate)
        elif act['kind'] == 'probe':
            # 试探前进：慢速小步，走够就用激光验收（恢复到"能正常走"的证据）
            twist.linear.x = abs(float(act.get('speed') or 0.06))
        elif act['kind'] == 'go':
            # 低速重试：按动作给定的速度（默认 retry_speed）
            twist.linear.x = abs(float(act.get('speed') or 0.08))
        else:
            sign = 1.0 if (act.get('yaw') or 0.0) >= 0 else -1.0
            # 优先往反方向转：卡住时贴着障碍转身更容易脱困
            twist.angular.z = -sign * abs(self.escape_speed)
            # 里程计不可信时也能靠时间兜底：不额外处理，超时即结束
        self._expected_lin = twist.linear.x
        if not self.dry_run:
            self.cmd_pub.publish(twist)

    def _signed_odom(self):
        """里程计位移在「动作开始时车头方向」上的投影，带符号。

        后退应该是负数。如果指令是后退、这个数却是正的大数，
        说明底盘把线速度方向搞反了（或者根本不是我们这一路在动）。
        """
        if self._odom_start is None or self.pose is None:
            return None
        x0, y0, yaw0 = self._odom_start
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        return dx * math.cos(yaw0) + dy * math.sin(yaw0)

    # ------------------------------------------------------------------
    # inflate：临时调代价地图的膨胀半径
    # ------------------------------------------------------------------
    def _costmap_node(self, name):
        # nav2 里的节点全名就是 /local_costmap/local_costmap 这种
        return '%s/%s/%s' % (self.costmap_prefix, name, name)

    def _param_client(self, kind, costmap):
        """按需建、永久留着：每调一次就建一次会把发现过程重走一遍。"""
        key = (kind, costmap)
        cli = self._param_clients.get(key)
        if cli is None:
            srv = GetParameters if kind == 'get' else SetParameters
            cli = self.create_client(srv, '%s/%s_parameters'
                                     % (self._costmap_node(costmap), kind))
            self._param_clients[key] = cli
        return cli

    def _call_param_service(self, kind, costmap, values):
        """异步调一次 get/set_parameters。

        不能在这里 spin_until_future_complete：调用发生在定时器回调里，
        同一个 executor 正忙着跑这个回调，回应永远等不到（曾因此一直报
        "调用没回"）。所以一律 call_async + 回调推进。
        """
        cli = self._param_client(kind, costmap)
        if not cli.service_is_ready():
            self.get_logger().warn('%s 的 %s_parameters 还没就绪（导航没起？）'
                                   % (self._costmap_node(costmap), kind))
            return None
        if kind == 'get':
            req = GetParameters.Request()
            req.names = [self.inflate_param]
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                double_value=float(values))
            req = SetParameters.Request()
            req.parameters = [Parameter(name=self.inflate_param, value=pv)]
        return cli.call_async(req)

    def _inflate_step(self):
        """按队列逐个把代价地图改成新值。"""
        if not self._inflate_queue:
            if not self._inflate_saved:
                act = self.action or {}
                self._report(act, ok=False, moved=0.0,
                             why='代价地图参数不可用（%s），inflate 没生效'
                                 % ','.join(self.inflate_costmaps))
                self.status = IDLE
                self.action = None
                return
            now = time.monotonic()
            self.status = RUNNING
            self.started_at = now
            self.deadline = now + self.inflate_hold_s
            self.get_logger().warn('执行动作 inflate：膨胀半径改为 %.2f，保持 %.1fs'
                                   % (self.inflate_radius, self.inflate_hold_s))
            return
        name = self._inflate_queue.pop(0)
        fut = self._call_param_service('get', name, None)
        if fut is None:
            self._inflate_step()
            return
        fut.add_done_callback(
            lambda f, n=name: self._on_got_inflation(n, f))

    def _on_got_inflation(self, name, future):
        try:
            old = float(future.result().values[0].double_value)
        except (AttributeError, IndexError, TypeError):
            self.get_logger().warn('%s 读不到膨胀半径，跳过' % name)
            self._inflate_step()
            return
        fut = self._call_param_service('set', name, self.inflate_radius)
        if fut is None:
            self._inflate_step()
            return
        fut.add_done_callback(
            lambda f, n=name, o=old: self._on_set_inflation(n, o, f))

    def _on_set_inflation(self, name, old, future):
        try:
            res = future.result()
            ok = bool(res.results) and res.results[0].successful
        except (AttributeError, TypeError):
            ok = False
        if ok:
            self._inflate_saved[name] = old
            self.get_logger().warn('%s 膨胀半径 %.2f → %.2f'
                                   % (name, old, self.inflate_radius))
        else:
            self.get_logger().warn('%s 拒绝改膨胀半径，跳过' % name)
        self._inflate_step()

    def _start_inflate(self, act):
        """把膨胀半径临时改成 inflate_radius，hold 到期再还原。

        为什么要动它：贴墙卡住时车往往比 inflation_radius 还近，
        局部代价全 lethal，控制器判"前面全是障碍"直接放弃；
        调大则是为了让它下一次规划离障碍更远。两个方向都可能有用，
        所以数值由参数给，不写死。
        """
        self.action = dict(act)
        self.monitor = None
        self.status = RUNNING
        self.started_at = time.monotonic()
        self.deadline = self.started_at + 30.0      # 参数调用本身的上限
        self._inflate_saved = {}
        self._inflate_queue = list(self.inflate_costmaps)
        self._odom_travel = 0.0
        self._odom_start = self.pose
        self._scan_prev = None
        self._scan_used = 0
        self._scan_drop = 0
        self._q_sum = 0.0
        self._inflate_step()

    def _restore_inflate(self):
        for name, old in (self._inflate_saved or {}).items():
            fut = self._call_param_service('set', name, old)
            if fut is None:
                self.get_logger().error('%s 的膨胀半径没还原成 %.2f，请手动检查'
                                        % (name, old))
                continue
            fut.add_done_callback(
                lambda f, n=name, o=old: self._on_restored(n, o, f))
        self._inflate_saved = {}

    def _on_restored(self, name, old, future):
        try:
            res = future.result()
            ok = bool(res.results) and res.results[0].successful
        except (AttributeError, TypeError):
            ok = False
        if ok:
            self.get_logger().warn('%s 膨胀半径还原为 %.2f' % (name, old))
        else:
            self.get_logger().error('%s 没还原成 %.2f，请手动检查' % (name, old))

    def _finish(self, ok, why):
        act = self.action or {}
        moved = self.monitor.progress() if self.monitor else 0.0
        odom_moved = self._odom_travel
        signed = self._signed_odom()
        q = (self._q_sum / self._scan_used) if self._scan_used else 0.0
        self._stop()
        self.status = IDLE
        self.action = None
        self.monitor = None
        self.get_logger().warn(
            '动作结束：%s（激光 %.3f m，里程计 %.3f m 带符号 %+.3f m，%s）'
            % ('成功' if ok else '失败', moved, odom_moved,
               signed if signed is not None else 0.0, why))
        if signed is not None and (act.get('kind') == 'back') \
                and signed > 0.05 and odom_moved > 0.05:
            self.get_logger().error(
                '指令是后退，里程计却往前走了 %.2f m：底盘线速度符号可能是反的'
                % signed)
        if ok and act.get('kind') in ('back', 'go', 'probe', 'escape') \
                and self.measure == 'scan' and odom_moved > 0.05 \
                and moved < 0.5 * odom_moved:
            # 轮子转了、车没走：动作"成功"是假的，不能说恢复成功
            ok = False
            why = '里程计说走了 %.2f m，激光只有 %.2f m（轮子空转）' % (odom_moved, moved)
            self.get_logger().warn(why)
        self._report(act, ok=ok, moved=moved, why=why, odom_moved=odom_moved,
                     signed=signed, scan_used=self._scan_used,
                     scan_drop=self._scan_drop, quality=q)

    def _stop(self):
        if self.dry_run:
            return
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.stop_until = time.monotonic() + self.settle_s

    def _report(self, act, ok, moved, why, odom_moved=None, signed=None,
                scan_used=0, scan_drop=0, quality=0.0, blocked=False):
        payload = {
            'id': act.get('id'),
            'kind': act.get('kind'),
            'ok': bool(ok),
            'moved': round(float(moved or 0.0), 4),
            'why': why,
            'blocked': bool(blocked),
            'at': round(time.time(), 3),
        }
        if odom_moved is not None:
            payload['odom_moved'] = round(float(odom_moved), 4)
        if signed is not None:
            payload['odom_signed'] = round(float(signed), 4)
        payload['scan_frames'] = int(scan_used)
        payload['scan_dropped'] = int(scan_drop)
        payload['scan_quality'] = round(float(quality), 3)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.report_pub.publish(msg)
        self.get_logger().info('上报动作结果：%s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SlipExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._stop()
        except Exception:      # noqa: BLE001
            pass
        # 崩了/被 Ctrl+C 也不能把膨胀半径留在改过的值上
        try:
            if node._inflate_saved:
                for name, old in list(node._inflate_saved.items()):
                    fut = node._call_param_service('set', name, old)
                    if fut is not None:
                        t_end = time.monotonic() + 2.0
                        while not fut.done() and time.monotonic() < t_end:
                            rclpy.spin_once(node, timeout_sec=0.05)
                        node.get_logger().warn('%s 退出前还原膨胀半径为 %.2f'
                                               % (name, old))
                node._inflate_saved = {}
        except Exception:      # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
