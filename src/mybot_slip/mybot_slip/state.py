"""恢复状态机：把「判据层的逐窗结论」变成「该做什么动作」。

方案 v1.1 第 6 章的做法，按类型分流（这点最关键）：

    STALL（轮子被卡住）  -> 直接脱困转向，**绝不后退**
       原因：轮子本来就转不动，后退指令同样发不出去，只会白等 6 秒。
    LINEAR / ANGULAR / WHEEL / LATERAL
        -> 后退一段（d_back = clamp(0.5·d_enc, 0.10, 0.30)），
           再停一下重判；后退本身也用激光闭环（计划 7.3），
           这里只负责「要求退多少、退完判是否真的退了」。
    维持滑移（连续 3 次恢复失败）
        -> 要求上层把局部膨胀半径临时调大，再试一次。

状态：
    NORMAL     正常
    SLIP       本窗有滑移，等连续确认
    RECOVERY   正在执行动作（back / escape / inflate）
    CHECK      动作完成，观察 release_windows 个窗口
    FAILED     放弃（交给上层：报警 / 跳过 / 原地等）

对外只有两个入口：
    step(now, verdict)        每个车窗调用一次
    on_action_done(ok, moved) 动作执行完（或超时）回调
"""
import math

DEFAULTS = {
    'confirm_windows': 2,      # 连续几个窗有滑移才算确认（再叠加 confirm_window_s 防抖）
    'confirm_window_s': 1.0,   # 且跨时不小于此值（防抖）
    'release_windows': 2,      # 连续几个窗干净才回 NORMAL
    'back_min_m': 0.10,
    'back_max_m': 0.30,
    'back_ratio': 0.5,         # d_back = back_ratio * 本窗编码器位移
    'back_verify_ratio': 0.6,  # 后退实测 >= 期望*此值 才算成功
    'escape_yaw_rad': 0.9,     # STALL 脱困转多少
    'escape_speed': 1.2,
    'inflate_try': 3,          # 第几次恢复时请求调大膨胀
    'max_recovery': 4,         # 一个恢复周期内最多试几次
    'alternate_after_fail': True,   # 失败后换姿势（后退<->转向）；关掉就重复同一种
    'recovery_cooldown_s': 60.0,
    'settle_s': 0.8,           # 动作后静置（等滤波器/里程计稳定）
    'stall_no_back': True,
    'action_timeout_s': 8.0,   # 动作发出去但没人执行/没回调时的超时（防止卡死在 recovery）
}

NORMAL = 'normal'
SLIP = 'slip'
RECOVERY = 'recovery'
CHECK = 'check'
FAILED = 'failed'

# 需要后退的类型
BACK_TYPES = ('LINEAR_SLIP', 'ANGULAR_SLIP', 'WHEEL_L', 'WHEEL_R',
              'LATERAL_SLIP', 'OVERRUN')


class SlipStateMachine(object):
    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.state = NORMAL
        self.type = None                 # 当前认定的滑移类型
        self.slip_count = 0
        self.clean_count = 0
        self.first_slip_at = None
        self.recoveries = 0
        self.recovery_started_at = None
        self.pending_action = None       # 等外部执行
        self.action_deadline = None
        self.settle_until = None
        self.cooldown_until = None
        self.last_action = None
        self.allow_recovery = True       # 执行器不在时由节点置 False：只报不动
        self.failed_at = None
        self.history = []                # [(t, state, type)]，限量
        self.stats = {'recoveries': 0, 'failures': 0, 'escapes': 0, 'backs': 0,
                      'timeouts': 0}

    # ------------------------------------------------------------------
    def reset_failure(self):
        self.state = NORMAL
        self.type = None
        self.slip_count = 0
        self.clean_count = 0
        self.first_slip_at = None
        self.recoveries = 0
        self.failed_at = None

    # ------------------------------------------------------------------
    def step(self, now, verdict):
        """verdict 是 criteria.evaluate() 的返回，必须含 'slip' 与 'type'。

        返回 {'action': 'back'|'escape'|'inflate'|'abort'|None, ...}
        """
        if self.cooldown_until and now < self.cooldown_until:
            return self._idle('cooldown')

        if self.state == FAILED:
            # 失败不是终点：过了冷却期回到观察态，否则一次失败就永久躺平
            if self.failed_at is not None and \
                    now - self.failed_at >= self.p['recovery_cooldown_s']:
                self.reset_failure()
                self.cooldown_until = None
                return {'action': 'reset', 'state': self.state,
                        'reason': '失败冷却结束，重新观察'}
            return self._idle('failed')

        if self.state == NORMAL:
            return self._step_normal(now, verdict)
        if self.state == SLIP:
            return self._step_slip(now, verdict)
        if self.state == CHECK:
            return self._step_check(now, verdict)
        if self.state == RECOVERY:
            return self._step_recovery(now)
        return self._idle('failed')

    def _step_recovery(self, now):
        """等执行器回调；没人执行就超时退出，绝不停在这里。"""
        if self.pending_action is None:
            return self._idle('waiting action')
        if self.action_deadline and now > self.action_deadline:
            self.pending_action = None
            self.action_deadline = None
            self.last_action = None
            self.stats['timeouts'] += 1
            self.state = CHECK
            self.clean_count = 0
            self.settle_until = now + self.p['settle_s']
            return {'action': 'none', 'state': self.state,
                    'reason': '动作超时未被执行（没有执行器？），本轮作废'}
        return self._idle('waiting action')

    def on_action_done(self, now, ok, moved=None):
        """动作结束回调：ok 表示动作本身成功（后退走够了 / 转向完成）。"""
        if self.state != RECOVERY:
            return {'action': None}
        self.pending_action = None
        self.action_deadline = None

        kind = self.last_action or {}
        if not ok:
            # 动作失败：先用「调大膨胀」再试，次数用尽才放弃
            self.recoveries += 1
            if self.recoveries >= self.p['max_recovery']:
                self.state = FAILED
                self.failed_at = now
                self.stats['failures'] += 1
                return {'action': 'abort', 'reason': '恢复次数用尽'}
            if self.recoveries >= self.p['inflate_try']:
                self.state = RECOVERY
                self.pending_action = {'action': 'inflate', 'reason': '连续恢复失败'}
                self.last_action = self.pending_action
                return self.pending_action
            self.pending_action = self._alternate(kind)
            self.last_action = self.pending_action
            return self.pending_action

        if kind.get('action') == 'back':
            expect = abs(kind.get('distance') or 0.0)
            if moved is not None and expect > 0:
                if moved < expect * self.p['back_verify_ratio']:
                    # 退了但没退够：说明还在打滑，升级
                    self.recoveries += 1
                    if self.recoveries >= self.p['max_recovery']:
                        self.state = FAILED
                        self.failed_at = now
                        self.stats['failures'] += 1
                        return {'action': 'abort', 'reason': '后退距离不足且次数用尽'}
                    if self.recoveries >= self.p['inflate_try']:
                        self.pending_action = {'action': 'inflate', 'reason': '连续恢复失败'}
                        self.last_action = self.pending_action
                        return self.pending_action
                    self.pending_action = self._alternate(kind)
                    self.last_action = self.pending_action
                    return self.pending_action

        self.state = CHECK
        self.clean_count = 0
        self.settle_until = now + self.p['settle_s']
        return {'action': None, 'state': self.state}

    # ------------------------------------------------------------------
    def _step_normal(self, now, verdict):
        if not verdict.get('slip'):
            self.slip_count = 0
            self.first_slip_at = None
            return self._idle('normal')
        if self.slip_count == 0 or self.first_slip_at is None:
            self.first_slip_at = now
        self.slip_count += 1
        self.type = verdict.get('type')
        self.state = SLIP
        return self._idle('suspect')

    def _step_slip(self, now, verdict):
        if verdict.get('slip'):
            self.slip_count += 1
            self.type = verdict.get('type')
            first = self.first_slip_at if self.first_slip_at is not None else now
            span = now - first
            if (self.slip_count >= self.p['confirm_windows']
                    and span >= self.p['confirm_window_s'] - 1e-9):
                if not self.allow_recovery:
                    # 没有执行器：只报警不动手，但状态留住，别让人以为已经好了
                    return self._idle('slip confirmed (recovery disabled)')
                return self._start_recovery(now, verdict)
            return self._idle('confirming')
        # 中间闪了一窗干净：退回 NORMAL，但保留类型供日志
        self.state = NORMAL
        self.slip_count = 0
        return self._idle('dropped')

    def _step_check(self, now, verdict):
        if self.settle_until and now < self.settle_until:
            return self._idle('settling')
        if verdict.get('slip'):
            # 还没好：回到恢复
            self.clean_count = 0
            self.state = SLIP
            self.slip_count = 1
            self.first_slip_at = now
            self.type = verdict.get('type')
            return self._idle('still slipping')
        self.clean_count += 1
        if self.clean_count >= self.p['release_windows']:
            self.state = NORMAL
            self.type = None
            self.slip_count = 0
            self.recoveries = 0
            self.recovery_started_at = None
            self.cooldown_until = now + 0.5
            return {'action': 'recovered', 'state': self.state}
        return self._idle('releasing')

    # ------------------------------------------------------------------
    def _start_recovery(self, now, verdict):
        if self.recovery_started_at is None:
            self.recovery_started_at = now
        self.recoveries += 1
        self.stats['recoveries'] += 1
        if self.recoveries > self.p['max_recovery']:
            self.state = FAILED
            self.failed_at = now
            self.stats['failures'] += 1
            return {'action': 'abort', 'reason': '恢复次数超过上限 %d' % self.p['max_recovery']}
        act = self._make_action(self.type, verdict)
        self.state = RECOVERY
        self.pending_action = act
        self.action_deadline = now + max(act.get('timeout_s') or 6.0,
                                         self.p['action_timeout_s'])
        self.last_action = act
        return dict(act)

    def _alternate(self, last):
        """上一次动作没成：换个姿势再来。

        后退顶不动（台阶/被抵住）就改成原地转向——先脱离这个方向，
        而不是反复用同一种方式撞同一个东西；转向还不行就退回去再试。
        """
        slip_type = (last or {}).get('type') or self.type
        if slip_type == 'STALL':
            return self._make_action('STALL')      # 堵转永远不后退
        if not self.p.get('alternate_after_fail', True):
            return self._make_action(slip_type)    # 调试用：关掉换姿势
        if (last or {}).get('action') == 'back':
            self.stats['escapes'] += 1
            return {
                'action': 'escape',
                'yaw': self.p['escape_yaw_rad'],
                'speed': self.p['escape_speed'],
                'timeout_s': 4.0,
                'type': slip_type,
                'reason': '后退没解决问题：改原地转向，换个方向再试',
            }
        act = self._make_action(slip_type)
        act['reason'] = '转向没解决问题：再退一次看看'
        return act

    def _make_action(self, slip_type, verdict=None):
        p = self.p
        if slip_type == 'STALL':
            # 轮子转不动，后退也没用：直接反向脱困
            self.stats['escapes'] += 1
            return {
                'action': 'escape',
                'yaw': p['escape_yaw_rad'],
                'speed': p['escape_speed'],
                'timeout_s': 4.0,
                'type': slip_type,
                'reason': '轮子被卡住：直接转向脱困，不后退',
            }
        # 其余类型：按本窗位移决定退多远
        denc = 0.0
        if verdict and verdict.get('ratios') is not None:
            denc = abs(verdict.get('_denc') or 0.0)
        dist = min(max(p['back_ratio'] * denc, p['back_min_m']), p['back_max_m'])
        self.stats['backs'] += 1
        return {
            'action': 'back',
            'distance': dist,
            'speed': 0.08,
            'timeout_s': 6.0,
            'type': slip_type,
            'verify': True,
            'reason': '%s：后退 %.2fm 再判' % (slip_type, dist),
        }

    def _idle(self, reason):
        return {'action': None, 'state': self.state, 'reason': reason}

    # ------------------------------------------------------------------
    def snapshot(self):
        return {
            'state': self.state,
            'type': self.type,
            'slip_count': self.slip_count,
            'clean_count': self.clean_count,
            'recoveries': self.recoveries,
            'cooldown': bool(self.cooldown_until),
            'stats': dict(self.stats),
        }
