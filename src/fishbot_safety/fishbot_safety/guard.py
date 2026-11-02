# -*- coding: utf-8 -*-
"""看门狗的核心逻辑：车在动，但已经没人发运动指令了 → 该刹车。

为什么单独一个文件：这是安全逻辑，必须能在没有 ROS 的机器上跑单测。
写在节点里就只能靠"推车试"，而推车恰恰是它要防的场景。

判断规则只用两条，都是可观测的：

1. **里程计说车在动，但最近 N 毫秒没有任何非零速度指令**
   → 发布者（nav2 / 巡检执行器 / 控制台）中途死掉了，车会按最后一帧一直跑。
2. **里程计本身断了**
   → 无法判断车是否在动，按安全侧假设它可能在动，也刹车。

为什么不把"视觉节点死了"算进来：视觉挂了只会让巡检报失败，不会让车乱跑；
刹车反而会制造假故障。看门狗只管"会不会动"，不管"会不会干活"。
"""
OK = 'OK'
TRIPPED = 'TRIPPED'

# 指令只认"非零"，因为我们自己刹车时会往同一个话题发零速；
# 如果零速也算心跳，看门狗会把自己发的零速当成"还活着"，永远解除不了。
_ZERO_EPS = 1e-3


class Guard(object):
    """没有 ROS 的状态机。只用单调时间戳，不依赖 wall clock。"""

    def __init__(self, cmd_timeout=0.5, odom_timeout=1.0,
                 move_epsilon=0.02, brake_settle=0.2):
        self.cmd_timeout = float(cmd_timeout)
        self.odom_timeout = float(odom_timeout)
        self.move_epsilon = float(move_epsilon)
        # 刚启动时里程计还没来，给它这个宽限，否则一开机就刹
        self.brake_settle = float(brake_settle)

        self.started_at = None
        self.last_cmd_move = None      # 最近一次"非零"指令的时刻
        self.last_odom = None
        self.odom_speed = (0.0, 0.0)
        self.tripped = False
        self.reason = ''
        self.trips = 0

    # ------------------------------------------------------------ 观测
    def observe_cmd(self, now, vx, wz):
        """收到一帧速度指令。只有非零才算"还有人在开车"。"""
        if self.started_at is None:
            self.started_at = now
        if abs(vx) > _ZERO_EPS or abs(wz) > _ZERO_EPS:
            self.last_cmd_move = now

    def observe_odom(self, now, vx, wz):
        if self.started_at is None:
            self.started_at = now
        self.last_odom = now
        self.odom_speed = (float(vx), float(wz))

    # ------------------------------------------------------------ 判定
    def moving(self):
        vx, wz = self.odom_speed
        return (abs(vx) > self.move_epsilon
                or abs(wz) > self.move_epsilon)

    def evaluate(self, now):
        """返回 (是否该刹车, 原因)。原因只在"刚进入刹车"或"原因变了"时非空。"""
        if self.started_at is None:
            # 一帧都没收到也要能判：否则"里程计根本没起来"会变成永远不刹车
            self.started_at = now
        if now - self.started_at < self.brake_settle:
            return False, ''

        # 1) 里程计断了：不知道车在哪、是不是在动，按安全侧假设
        if self.last_odom is None or (now - self.last_odom) > self.odom_timeout:
            if self.last_odom is None:
                detail = '从未收到里程计'
            else:
                detail = 'odom 断开 %.1fs' % (now - self.last_odom)
            return self._trip(now, '%s，无法确认车是否在动' % detail)

        # 2) 车在动但没人下指令
        if self.moving():
            if (self.last_cmd_move is None
                    or (now - self.last_cmd_move) > self.cmd_timeout):
                idle_ms = 1000.0 * (now - (self.last_cmd_move or now))
                return self._trip(
                    now, '车在动（%.2f m/s）但已 %.0f ms 没有速度指令'
                    % (abs(self.odom_speed[0]), idle_ms))

        # 3) 恢复正常
        if self.tripped:
            self.tripped = False
            self.reason = ''
        return False, ''

    def _trip(self, now, reason):
        first = not self.tripped
        changed = first or reason != self.reason
        self.tripped = True
        self.reason = reason
        if first:
            self.trips += 1
        return True, (reason if changed else '')

    # ------------------------------------------------------------ 诊断
    def snapshot(self):
        return {
            'tripped': self.tripped,
            'reason': self.reason,
            'trips': self.trips,
            'moving': self.moving(),
            'speed': {'vx': self.odom_speed[0], 'wz': self.odom_speed[1]},
        }
