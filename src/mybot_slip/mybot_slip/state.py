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
    'confirm_window_s': 0.0,   # 方案 §12：只要求 Nconfirm=2 个连续窗（Tw 0.40 s 即 0.8 s）
    'release_windows': 2,      # 连续几个窗干净才回 NORMAL
    'back_verify_ratio': 0.6,  # 后退实测 >= 期望*此值 才算成功
    'escape_yaw_rad': 0.9,     # STALL 脱困转多少
    'escape_speed': 1.2,
    'inflate_try': 3,          # 第几次恢复时请求调大膨胀
    'max_recovery': 2,         # 方案 §7：一个恢复周期最多试两次
    'alternate_after_fail': True,   # 失败后换姿势（后退<->转向）；关掉就重复同一种
    'recovery_cooldown_s': 5.0,   # 方案 §9.6：FAILED 复位后 5 s 冷却
    'settle_s': 0.3,           # 方案 §9.5：CHECK 前静止 0.3 s
    'stall_hold_s': 0.6,       # 方案 §12 Tstall：堵转要持续 0.6 s
    'stall_no_back': True,
    'retry_m': 0.10,           # 单轮打滑：低速重试走多远
    'retry_speed': 0.08,
    'micro_yaw_rad': 0.30,     # 旋转卡死：反向微转多少
    'reverse_adjust_yaw_rate': 0.15,   # 后退时的小幅反向调整（rad/s）
    'probe_m': 0.05,           # 方案 §5：恢复后试探前进的距离
    'probe_speed': 0.06,
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
        self.first_stall_at = None
        self.recoveries = 0
        self.recovery_started_at = None
        self.pending_action = None       # 等外部执行
        self.action_deadline = None
        self.settle_until = None
        self.cooldown_until = None
        self.last_action = None
        self.allow_recovery = True       # 执行器不在时由节点置 False：只报不动
        self.failed_at = None
        self.probed = False              # 本次 CHECK 是否已经做过试探前进
        self.stall_stops = 0             # 本轮堵转已经"停车上报"过几次
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
        self.first_stall_at = None
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
        if self.state == CHECK:
            # 试探前进的回执：不管走没走够，接下来都靠"连续干净窗"判定
            kind = self.last_action or {}
            self.pending_action = None
            self.action_deadline = None
            self.last_action = None
            self.clean_count = 0
            self.settle_until = now + self.p['settle_s']
            if kind.get('action') == 'probe':
                return {'action': None, 'state': self.state,
                        'reason': '试探前进完成（实测 %.3f m）' % (moved or 0.0)}
            return {'action': None, 'state': self.state}
        if self.state != RECOVERY:
            return {'action': None}
        self.pending_action = None
        self.action_deadline = None

        kind = self.last_action or {}
        if kind.get('terminal'):
            # 方案 §3：堵转这类"只能人工处理"的类型，执行完停车就收工
            self.recoveries += 1
            self.state = FAILED
            self.failed_at = now
            self.stats['failures'] += 1
            return {'action': 'abort', 'reason': '%s，等人工复位'
                    % (kind.get('reason') or '需要人工处理')}
        if not ok:
            # 动作失败：先用「调大膨胀」再试，次数用尽才放弃
            self.recoveries += 1
            if self.recoveries > self.p['max_recovery']:
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
                    if self.recoveries > self.p['max_recovery']:
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
        self.probed = False
        self.settle_until = now + self.p['settle_s']
        nxt = kind.get('then')
        if nxt == 'slow_go':
            # §3：停车/微转之后总是接一次低速重试（而不是直接宣布好了）
            act = {
                'action': 'go',
                'distance': self.p['retry_m'],
                'speed': self.p['retry_speed'],
                'timeout_s': 6.0,
                'reset_odom': bool(kind.get('reset_odom')),
                'type': self.type,
                'reason': '低速试探 %.2fm，确认能正常走' % self.p['retry_m'],
            }
            self.state = RECOVERY
            self.pending_action = act
            self.last_action = act
            self.action_deadline = now + max(act['timeout_s'], self.p['action_timeout_s'])
            return dict(act)
        if nxt == 'replan':
            # §3：过位移交回 Nav2 重新规划（节点负责取消当前目标）
            self.state = CHECK
            return {'action': 'replan', 'state': self.state,
                    'reason': '交回 Nav2 重新规划'}
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
        if self.type == 'STALL' and self.first_stall_at is None:
            # 方案 §12 Tstall：从第一个堵转窗开始计时
            self.first_stall_at = now
        self.state = SLIP
        return self._idle('suspect')

    def _stall_ready(self, now, verdict):
        """方案 §12 Tstall：堵转必须持续 0.6 s 才算，短暂停一下不算。"""
        if verdict.get('type') != 'STALL':
            self.first_stall_at = None
            return True
        if self.first_stall_at is None:
            self.first_stall_at = now
        return (now - self.first_stall_at) >= self.p['stall_hold_s'] - 1e-9

    def _step_slip(self, now, verdict):
        if verdict.get('slip'):
            self.slip_count += 1
            self.type = verdict.get('type')
            first = self.first_slip_at if self.first_slip_at is not None else now
            span = now - first
            if (self.slip_count >= self.p['confirm_windows']
                    and span >= self.p['confirm_window_s'] - 1e-9
                    and self._stall_ready(now, verdict)):
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
            self.probed = False
            self.state = SLIP
            self.slip_count = 1
            self.first_slip_at = now
            self.type = verdict.get('type')
            return self._idle('still slipping')
        if not self.probed:
            # 方案 §9.5：静止 0.3 s 后先采两个非重叠窗，全满足才做 0.05 m 试探
            if self.clean_count + 1 < self.p['release_windows']:
                self.clean_count += 1
                return self._idle('releasing %d/%d'
                                  % (self.clean_count, self.p['release_windows']))
            self.clean_count = 0
            # 方案 §5：两个干净窗通过后，试探前进一小步，看三个源还对不对得上
            act = {
                'action': 'probe',
                'distance': self.p['probe_m'],
                'speed': self.p['probe_speed'],
                'timeout_s': 4.0,
                'type': self.type,
                'reason': '恢复后试探前进 %.2fm，确认编码器/IMU/激光重新一致'
                          % self.p['probe_m'],
            }
            self.probed = True
            self.pending_action = act
            self.last_action = act
            self.action_deadline = now + max(act['timeout_s'], self.p['action_timeout_s'])
            return dict(act)
        # 方案 §9.5：试探前进后「复检一次」，所以探针之后只需要一个干净窗
        self.clean_count += 1
        need = 1 if self.probed else self.p['release_windows']
        if self.clean_count >= need:
            self.state = NORMAL
            self.type = None
            self.slip_count = 0
            self.recoveries = 0
            self.recovery_started_at = None
            self.stall_stops = 0
            self.probed = False
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
        """方案 §7：最多两次恢复。第二次还是同类型动作——不发明新姿势。"""
        slip_type = (last or {}).get('type') or self.type
        act = self._make_action(slip_type)
        act['reason'] = '%s（第 %d 次）' % (act.get('reason') or '',
                                           self.recoveries)
        return act

    def _make_action(self, slip_type, verdict=None):
        """方案 §3：恢复动作必须按异常类型分流，不能一律后退。"""
        p = self.p
        if slip_type == 'STALL':
            # 堵转：不后退、不加力，停车上报，等待人工处理（终局，不再重试）
            return {
                'action': 'stop',
                'hold_s': 3.0,
                'timeout_s': 3.5,
                'terminal': True,
                'type': slip_type,
                'reason': '堵转：不后退不加力，停车上报，等人工处理',
            }

        if slip_type in ('WHEEL_L', 'WHEEL_R'):
            # 表 9-1：降速、清除该轮里程积分、小幅反向微转后低速重试
            return {
                'action': 'escape',
                'yaw': -p['micro_yaw_rad'],
                'speed': p['escape_speed'],
                'timeout_s': 4.0,
                'then': 'slow_go',
                'reset_odom': True,
                'type': slip_type,
                'reason': '单轮打滑：降速、清里程积分，反向微转 %.2f rad 后低速重试'
                          % p['micro_yaw_rad'],
            }

        if slip_type == 'LATERAL_SLIP':
            # 横向侧滑：先停车，降低速度与加速度限制，再重新起步
            return {
                'action': 'stop',
                'hold_s': 1.0,
                'timeout_s': 1.5,
                'then': 'slow_go',
                'type': slip_type,
                'reason': '横向侧滑：先停车，再按降速限加速重新起步',
            }

        if slip_type == 'OVERRUN':
            # 下坡过位移：降速、增强制动并重新规划
            return {
                'action': 'stop',
                'hold_s': 1.0,
                'timeout_s': 1.5,
                'then': 'replan',
                'type': slip_type,
                'reason': '过位移（下坡/被推）：停车制动后交回 Nav2 重新规划',
            }

        if slip_type == 'ANGULAR_SLIP':
            # 旋转卡死：小幅反向微转，再低速试探
            return {
                'action': 'escape',
                'yaw': -p['micro_yaw_rad'],
                'speed': p['escape_speed'],
                'timeout_s': 4.0,
                'then': 'slow_go',
                'type': slip_type,
                'reason': '旋转卡死：反向微转 %.2f rad 后低速试探'
                          % p['micro_yaw_rad'],
            }

        # 直行空转 / 顶住障碍物：确认后方安全后短距离后退，并小幅反向调整
        # 方案 §9.3 + 表 12-1：第一次固定 0.12 m、−0.15 m/s、超时 1.5 s；
        # 第二次 0.20 m。不再按本窗位移比例算（那是 v1.0 的做法）。
        # 第几次恢复按"本轮异常"算（self.recoveries 在 _start_recovery 里已 +1），
        # 不能用累计的 stats['backs']——上一条路线的恢复次数会污染下一条。
        second = self.recoveries >= 2
        dist = 0.20 if second else 0.12
        self.stats['backs'] += 1
        return {
            'action': 'back',
            'distance': dist,
            'speed': 0.15,
            'yaw_rate': p['reverse_adjust_yaw_rate'],
            'timeout_s': 1.5,
            'rear_check_m': 0.30,
            'then': 'slow_go',
            'type': slip_type,
            'verify': True,
            'reason': '直行空转：后退 %.2fm（第 %s 次）并小幅反向调整'
                      % (dist, '二' if second else '一'),
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
