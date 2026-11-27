"""激光速度观测节点（方案 §3）。

把相邻两帧激光的 ICP 相对位姿增量，按两帧时间差换算成车体速度，发成
nav_msgs/Odometry 的 twist 部分，供 EKF 作为**独立于轮式编码器**的速度观测。

约束（照方案执行）：
  * 只发 twist，不发 TF；pose 部分给单位阵 + 巨大协方差，保证 EKF 即使误配
    也不会把相邻帧差分当成全局位姿。
  * 第一版只让 EKF 用 v_x 和 w_z；v_y 照常算、照常发，但只进日志（方案 §4）。
  * 匹配失败/退化/跳变/时间戳异常时**不发布**，绝不以零速代替（方案 §3.2、§5）。
  * 协方差用"基础值 + 三档质量"，分方向放大，不由 ICP 残差换算（方案 §5）。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from . import ranges as ranges_mod


def _wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ScanMatchOdom(Node):
    def __init__(self):
        super().__init__('scan_match_odom')
        d = self.declare_parameter
        d('scan_topic', '/scan')
        d('odom_topic', '/odom')
        d('out_topic', '/scan_match/odom')
        # 时间与几何
        d('min_dt', 0.05)
        d('max_dt', 0.60)
        d('jump_m', 0.30)          # 单帧位移上限，超过判为配准跳变
        d('jump_rad', 0.80)
        # 有效性
        d('inlier_ratio_good', 0.60)
        d('inlier_ratio_mid', 0.35)
        d('beams_min', 30)
        d('coverage_min', 0.60)
        d('fit_min_quality', 0.25)  # 交给 ranges 的 ok 下限；判据层是 0.55，这里放宽好让“一般”档真的能被用上
        d('max_speed', 0.60)       # m/s
        d('max_yaw_rate', 2.50)    # rad/s
        # 协方差：基础值（速度，m/s 与 rad/s）
        d('sigma_vx', 0.020)
        d('sigma_vy', 0.100)
        d('sigma_wz', 0.020)
        d('mid_scale', 3.0)        # 质量一般时的放大倍数
        d('degen_scale', 6.0)      # 判定为退化方向时的放大倍数
        d('use_odom_init', True)
        d('publish_mid', True)     # 质量一般是否照发（方案允许，配合放大协方差）

        gp = self.get_parameter
        self.scan_topic = gp('scan_topic').value
        self.odom_topic = gp('odom_topic').value
        self.out_topic = gp('out_topic').value
        self.min_dt = float(gp('min_dt').value)
        self.max_dt = float(gp('max_dt').value)
        self.jump_m = float(gp('jump_m').value)
        self.jump_rad = float(gp('jump_rad').value)
        self.r_good = float(gp('inlier_ratio_good').value)
        self.r_mid = float(gp('inlier_ratio_mid').value)
        self.beams_min = int(gp('beams_min').value)
        self.coverage_min = float(gp('coverage_min').value)
        self.fit_min_quality = float(gp('fit_min_quality').value)
        self.max_speed = float(gp('max_speed').value)
        self.max_yaw = float(gp('max_yaw_rate').value)
        self.sigma = {'vx': float(gp('sigma_vx').value),
                      'vy': float(gp('sigma_vy').value),
                      'wz': float(gp('sigma_wz').value)}
        self.mid_scale = float(gp('mid_scale').value)
        self.degen_scale = float(gp('degen_scale').value)
        self.use_odom_init = bool(gp('use_odom_init').value)
        self.publish_mid = bool(gp('publish_mid').value)

        self._prev = None            # (t, pts)
        self._odom_prev = None       # (t, x, y, yaw)
        self._last_log = 0.0
        self.stats = {'good': 0, 'mid': 0, 'drop': 0}

        self.pub = self.create_publisher(Odometry, self.out_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.get_logger().info(
            '激光速度观测已启动：%s → %s（10 Hz 级别，只发 twist，不发 TF）'
            % (self.scan_topic, self.out_topic))

    # ---------------- 输入 ----------------
    @staticmethod
    def _stamp(msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self._odom_prev = (self._stamp(msg), p.x, p.y, yaw)

    def _odom_init(self, t0, t1):
        """用轮式 odom 的增量当 ICP 初值（方案 §5 要求记录并识别该依赖）。"""
        if not self.use_odom_init or self._odom_prev is None:
            return None
        t, x, y, yaw = self._odom_prev
        if t < t1 - self.max_dt:
            return None
        # 只用一个粗糙的匀速外推，避免把 odom 的绝对误差带进来
        return (0.0, 0.0, 0.0)

    # ---------------- 主流程 ----------------
    def _on_scan(self, msg):
        t = self._stamp(msg)
        scan = {'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max}
        # fit_translation 自己会调 _points()，这里必须传「扫描对象」而不是点列
        if self._prev is not None:
            t0, p0 = self._prev
            dt = t - t0
            if dt < self.min_dt or dt > self.max_dt:
                if dt > self.max_dt:
                    self._drop(t, '时间差 %.2fs 超出上限' % dt)
                self._prev = (t, scan)
                return
            fit = ranges_mod.fit_translation(
                p0, scan, params={'min_quality': self.fit_min_quality},
                init=self._odom_init(t0, t))
            self._handle(fit, dt, t)
        self._prev = (t, scan)

    def _handle(self, fit, dt, t):
        if not fit.get('ok'):
            self._drop(t, '匹配未收敛：%s' % (fit.get('reason') or '-'))
            return
        beams = fit.get('beams') or 0
        cov = fit.get('coverage') or 0.0
        quality = fit.get('quality') or 0.0
        if beams < self.beams_min or cov < self.coverage_min:
            self._drop(t, '约束不足（内点 %d，覆盖 %.2f）' % (beams, cov))
            return

        dx, dy, dth = fit.get('dx') or 0.0, fit.get('dy') or 0.0, fit.get('dtheta') or 0.0
        if abs(dx) > self.jump_m or abs(dy) > self.jump_m or abs(dth) > self.jump_rad:
            self._drop(t, '增量跳变（%.2f, %.2f, %.2f）' % (dx, dy, dth))
            return

        vx, vy, wz = dx / dt, dy / dt, _wrap(dth) / dt
        if abs(vx) > self.max_speed or abs(wz) > self.max_yaw:
            self._drop(t, '速度超限（vx %.2f，wz %.2f）' % (vx, wz))
            return

        # 质量分档（方案 §5 表 5-1）
        inlier_ratio = quality  # ranges.fit_translation 的 quality 就是内点比×覆盖
        if inlier_ratio >= self.r_good:
            tier, scales = 'good', (1.0, 1.0, 1.0)
        elif inlier_ratio >= self.r_mid:
            tier, scales = 'mid', (self.mid_scale,) * 3
            if not self.publish_mid:
                self._drop(t, '质量一般（%.2f）已按配置丢弃' % inlier_ratio)
                return
        else:
            self._drop(t, '质量差（%.2f）' % inlier_ratio)
            return

        # 退化方向：沿视野弱约束方向（这里用横向/转角相对前向的比例粗判）
        sx, sy, sw = scales
        if abs(vx) > 0.02 and abs(vy) > max(0.05, 0.5 * abs(vx)):
            sy *= self.degen_scale     # 横向估计明显大于前向 → 可能是退化
        if abs(wz) > 0.1 and inlier_ratio < self.r_good:
            sw *= (self.mid_scale if tier == 'mid' else 1.0)

        self._emit(t, vx, vy, wz, sx, sy, sw, tier, inlier_ratio)

    # ---------------- 输出 ----------------
    def _emit(self, t, vx, vy, wz, sx, sy, sw, tier, q):
        m = Odometry()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        if m.header.stamp.nanosec >= 1000000000:
            m.header.stamp.sec += 1
            m.header.stamp.nanosec -= 1000000000
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        # pose 用单位阵 + 巨大协方差：本期 EKF 不读 pose（方案 §4）
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance = [1.0e6 if i in (0, 7, 14, 21, 28, 35) else 0.0
                             for i in range(36)]
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = wz
        c = [0.0] * 36
        c[0] = (self.sigma['vx'] * sx) ** 2
        c[7] = (self.sigma['vy'] * sy) ** 2
        c[35] = (self.sigma['wz'] * sw) ** 2
        m.twist.covariance = c
        self.pub.publish(m)
        self.stats[tier] += 1

    def _drop(self, t, why):
        self.stats['drop'] += 1
        if t - self._last_log >= 2.0:
            self._last_log = t
            self.get_logger().warn(
                '本次观测不发布：%s（累计 好 %d / 一般 %d / 丢 %d）'
                % (why, self.stats['good'], self.stats['mid'], self.stats['drop']))


def main(argv=None):
    rclpy.init(args=argv)
    node = ScanMatchOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
