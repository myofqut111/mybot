"""电池电压监控：中值滤波 + 迟滞判定，纯逻辑、不 import ROS。

为什么不直接拿瞬时值和阈值比：

- ESP32 的 ADC 有噪声，±0.1 V 的抖动很常见，单帧比较会让告警反复横跳；
- 电机启动/加速会把电池拉低 0.3~0.8 V，那是**压降**不是**没电**，等车停下
  电压自己就回去了。所以先取最近 N 个样本的中值（抗单点尖峰），再叠一层
  迟滞（掉到阈值才报，回到阈值+回差才解除）。

三个状态：``ok`` → ``warn``（能继续，但要提示）→ ``crit``（跑完当前点就中止）。
读不到数据、或数据太旧时一律返回 ``unknown``——**不能让一个卡死的发布者
把车留在“以为有电”的状态里**。
"""

LEVEL_UNKNOWN = 'unknown'
LEVEL_OK = 'ok'
LEVEL_WARN = 'warn'
LEVEL_CRIT = 'crit'

_LEVELS = (LEVEL_UNKNOWN, LEVEL_OK, LEVEL_WARN, LEVEL_CRIT)


def _valid(voltage) -> bool:
    """只接受有限的正数。NaN/inf/负数都是坏数据，直接丢掉。"""
    try:
        value = float(voltage)
    except (TypeError, ValueError):
        return False
    return value == value and value not in (float('inf'), float('-inf')) \
        and value > 0.0


class BatteryMonitor:
    """最近若干个电压样本 → 平滑值 → 带迟滞的等级。"""

    def __init__(self, warn_v: float, crit_v: float, hyst_v: float = 0.4,
                 window: int = 5, stale_sec: float = 30.0,
                 clock=None):
        """``window`` 个样本的中值；``stale_sec`` 内没有新样本就算数据过期。"""
        self._warn = float(warn_v)
        self._crit = float(crit_v)
        # 回差为负会变成“报完立刻解除”，等于没有迟滞
        self._hyst = max(0.0, float(hyst_v))
        self._window = max(1, int(window))
        self._stale = max(0.0, float(stale_sec))
        self._clock = clock or (lambda: __import__('time').monotonic())
        self._samples = []
        self._last_stamp = 0.0
        self._level = LEVEL_UNKNOWN
        self._rejected = 0

    # -- 输入 -------------------------------------------------------
    def update(self, voltage) -> bool:
        """喂一个样本；坏数据返回 False 且不入窗口。"""
        if not _valid(voltage):
            self._rejected += 1
            return False
        self._samples.append(float(voltage))
        if len(self._samples) > self._window:
            del self._samples[0]
        self._last_stamp = self._clock()
        self._level = self._compute_level(self.voltage())
        return True

    def reset(self) -> None:
        """清空历史（换电池、重新开始一趟巡检时用）。"""
        self._samples = []
        self._last_stamp = 0.0
        self._level = LEVEL_UNKNOWN

    # -- 输出 -------------------------------------------------------
    def voltage(self):
        """窗口内中值；没有样本返回 None。"""
        if not self._samples:
            return None
        ordered = sorted(self._samples)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    def fresh(self, now=None) -> bool:
        """最近一次样本是否还在有效期内。"""
        if self._last_stamp <= 0.0:
            return False
        moment = self._clock() if now is None else float(now)
        return (moment - self._last_stamp) <= self._stale

    def level(self, now=None) -> str:
        """当前等级；数据过期时返回 ``unknown``（别拿旧值做决策）。"""
        if not self.fresh(now):
            return LEVEL_UNKNOWN
        return self._level

    def describe(self, now=None) -> str:
        """一行给人看的描述，塞进日志/状态都合适。"""
        voltage = self.voltage()
        if voltage is None:
            return 'battery unknown (no data)'
        return 'battery %.2fV %s' % (voltage, self.level(now))

    # -- 内部 -------------------------------------------------------
    def _compute_level(self, voltage) -> str:
        """迟滞转移：ok↔warn↔crit 只能相邻跳，且回升要多走一个回差。

        浮点比较全部留 1e-9 的余量：``11.8 + 0.4`` 在 float 里是
        ``12.200000000000001``，直接拿 12.2 去比会差一个 ULP，导致
        “明明回到 12.2 了却还卡在 warn”。
        """
        if voltage is None:
            return LEVEL_UNKNOWN
        eps = 1e-9
        current = self._level
        warn_clear = self._warn + self._hyst - eps
        crit_clear = self._crit + self._hyst - eps

        # 已经在 crit：必须先越过 crit+回差，否则一直停在这一档
        if current == LEVEL_CRIT and voltage < crit_clear:
            return LEVEL_CRIT
        if voltage <= self._crit + eps:
            return LEVEL_CRIT
        # 从 warn/crit 下来：要真正回到 warn+回差之上才算恢复
        if current in (LEVEL_WARN, LEVEL_CRIT):
            return LEVEL_OK if voltage >= warn_clear else LEVEL_WARN
        if voltage <= self._warn + eps:
            return LEVEL_WARN
        return LEVEL_OK
