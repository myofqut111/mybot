# -*- coding: utf-8 -*-
"""可信运动状态 S0~S4（《补充说明》§3、§4）。

为什么单独一层：打滑判据只回答"这一窗像不像滑"，而定位、地图、导航
要不要继续吃这份数据，是另一件事。补充稿要求任何"不可信"标志都必须
绑定实际动作（协方差、冻结、隔离、限速、禁导航），所以把判决集中在这里，
由 slip_node 每窗更新一次，再分发给 cmd_mux（限速/禁导航）、scan 门控
（暂停建图）和 odom_guard（降权/冻结）。

状态定义（照抄表 3-1）：
    S0 正常可信   各来源有效且一致
    S1 候选异常   单窗明显矛盾，未连续确认 → 限速 + 暂停建图，不判打滑
    S2 确认打滑   连续确认 → odom 降权/冻结 + 停车 + 恢复准入
    S3 定位受限   观测不足/位姿误差扩大/来源持续冲突 → 禁导航、暂停建图
    S4 不可判定   原因不明或缺独立观测 → 停车、取消任务、上报、禁止自动恢复
"""

S0 = 'S0'
S1 = 'S1'
S2 = 'S2'
S3 = 'S3'
S4 = 'S4'

#: 允许写持久地图（slam_toolbox）的状态。S1 候选保护、S2 位姿冻结、S3/S4
#: 定位受限——都不得把扫描写进图（表 3-1 + §4：冻结期间的帧写进去就是脏图）。
MAP_ALLOWED = (S0,)
#: 允许导航正常控制的状态。S2 起恢复动作独占车轮，S3/S4 定位受限；
#: 只有 S0/S1 让 Nav2 直接驱动（§11.1 打滑恢复优先于 Nav2）。
NAV_ALLOWED = (S0, S1)

DEFAULTS = {
    # 离开 S1/S3 需要连续几个干净窗（§4 滞回）
    'clean_windows_exit': 2,
    # S1 限速（方案只说"立即限速"，数值取自保守估计）
    'suspect_speed_cap': 0.10,
    'suspect_yaw_cap': 0.50,
    # 连续几个 SENSOR_CONFLICT 窗算"来源持续冲突"→ S3，再多就到 S4
    'conflict_windows_s3': 2,
    'conflict_windows_s4': 4,
    # 恢复后位姿跳变阈值（与方案 §12 一致）
    'pose_jump_m': 0.15,
    'pose_jump_rad': 0.20,
}


class TrustedState(object):
    """把每窗的判据结果翻译成 S0~S4，并带滞回。"""

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.state = S0
        self.reason = ''
        self._clean = 0
        self._conflict = 0
        self._reloc_pending = False
        self._pose_jump = 0.0

    # ---- 外部事件 -------------------------------------------------
    def note_pose_jump(self, dist):
        """恢复后位姿跳变超限：进入 S3，直到重定位完成。"""
        if dist >= float(self.p['pose_jump_m']):
            self._pose_jump = dist
            self._reloc_pending = True

    def note_relocalized(self):
        self._reloc_pending = False
        self._pose_jump = 0.0

    def note_clean_window(self):
        self._clean += 1

    # ---- 每窗更新 -------------------------------------------------
    def update(self, verdict, *, machine_state='normal', confirmed=False,
               scan_ok=True, imu_ok=True, known_reason=''):
        """返回 (state, reason)。"""
        vtype = (verdict or {}).get('type')
        slip = bool((verdict or {}).get('slip'))

        if vtype == 'SENSOR_CONFLICT':
            self._conflict += 1
        else:
            self._conflict = 0

        # S4：原因不明（传感器互相矛盾且持续）或缺独立观测还要动
        if self._conflict >= int(self.p['conflict_windows_s4']):
            return self._enter(S4, '多源持续冲突且无法判断原因（%d 窗）' % self._conflict)
        if not scan_ok and (slip or machine_state != 'normal'):
            return self._enter(S4, '没有独立位移观测（激光不可用），不能判断也不能恢复')

        # S3：定位受限——来源持续冲突 / 恢复后位姿跳变待重定位
        if self._reloc_pending:
            return self._enter(
                S3, '恢复后位姿跳变 %.2f m，等重定位完成' % self._pose_jump)
        if self._conflict >= int(self.p['conflict_windows_s3']):
            return self._enter(
                S3, '来源持续冲突（%d 窗），只保留必要 TF' % self._conflict)
        if not imu_ok and slip:
            return self._enter(S3, 'IMU 不可用，旋转判据失效，降级到定位受限')

        # S2：确认打滑
        if confirmed or machine_state in ('recovery', 'check'):
            return self._enter(S2, known_reason or '确认打滑：%s' % (vtype or ''))

        # S1：候选异常（单窗矛盾），带滞回退出
        if slip or machine_state == 'slip':
            self._clean = 0
            return self._enter(S1, known_reason or '候选异常：%s' % (vtype or ''))
        if self.state in (S1, S3):
            self._clean += 1
            if self._clean < int(self.p['clean_windows_exit']):
                return self.state, '滞回中（%d/%d 干净窗）' % (
                    self._clean, int(self.p['clean_windows_exit']))
        return self._enter(S0, '各来源一致')

    # ---- 便捷查询 -------------------------------------------------
    def _enter(self, state, reason):
        if state not in (S1, S3):
            self._clean = 0
        if state != self.state:
            self.reason = reason
        else:
            self.reason = reason
        self.state = state
        return state, reason

    @property
    def map_allowed(self):
        return self.state in MAP_ALLOWED

    @property
    def nav_allowed(self):
        return self.state in NAV_ALLOWED

    @property
    def speed_cap(self):
        if self.state == S1:
            return float(self.p['suspect_speed_cap']), float(self.p['suspect_yaw_cap'])
        return None


def check_admission(verdict, *, state, scan_ok, imu_ok, tilt_deg=0.0,
                    consecutive_fail=0, max_fail=2, max_tilt_deg=10.0):
    """《补充说明》§5.2：进入任何自动恢复前的七条准入。

    返回 (ok, reason)。任一条件不满足都不得自动恢复——按方案是停车上报，
    而不是"再试一次"。这里只做能落地的判断：我们没有电流/悬空/地面材质
    传感器，所以"托底、悬空、持续湿滑"用排除式近似（原因不明的堵转、
    倾斜超限、同目标连续失败都归到"不适合自动恢复"）。
    """
    vtype = (verdict or {}).get('type')
    if not vtype:
        return False, '异常类型不可信（没判出具体类型）'
    if state in (S3, S4):
        return False, '定位%s，禁止自动恢复' % ('受限' if state == S3 else '不可判定')
    if not scan_ok:
        return False, '没有独立位移观测（激光不可用），不能做定量位移恢复'
    if not imu_ok:
        return False, 'IMU 不可用，无法监测再次打滑'
    if tilt_deg > max_tilt_deg:
        return False, '车身倾斜 %.0f°（可能托底/悬空），不自动恢复' % tilt_deg
    if consecutive_fail >= max_fail:
        return False, '同一目标已连续失败 %d 次，判为持续湿滑/卡滞，停手上报' % consecutive_fail
    return True, ''
