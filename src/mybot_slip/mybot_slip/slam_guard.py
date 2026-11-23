"""打滑期间暂停建图：调用 slam_toolbox 的 pause 服务。

为什么需要它
------------
打滑时轮子在转、车没动，`/odom` 照样在涨。slam_toolbox 会把这几秒的扫描
按"车走了"的位姿塞进地图；等它事后靠扫描匹配把位姿拉回来，那几帧按错误
位姿画上去的墙已经留在图上了 —— 表现就是"两张图叠在一起"。

所以打滑判定成立时先把建图摁住，恢复动作做完、状态回 NORMAL 再放开。

服务语义（实测，Humble 1.1.x）
------------------------------
`slam_toolbox/srv/Pause`：请求为空，响应里的 `bool status` **恒为 true**，
没有信息量。服务本身是 **toggle**：

    调用 1 次 → "Toggled to pause taking new measurements."
    再调 1 次 → "Toggled to actively taking new measurements."

（用 /map 的占用格数验证过：调用后 6~10 s 地图一个格子都不变，再调一次
又开始长。）

因此这里不能"看返回状态对不对、不对就补一次"——那会连翻两次，等于没暂停。
正确做法是**自己记账**：只在我们认为需要改变时才调，调完就翻转本地状态。
slam_toolbox 启动时一定是"在收数据"的状态，所以初始本地状态记为 False。
"""

DEFAULT_SERVICE = '/slam_toolbox/pause_new_measurements'
DEFAULT_MAX_HOLD_S = 15.0


class PausePolicy(object):
    """决定"这一刻该不该摁住建图"，含一个最长暂停时长兜底。

    为什么要兜底：如果状态机一直没能回到 NORMAL（比如在长走廊里 ICP 反复
    判成滑移），建图就一直停着，那比"带着一点漂移继续建"更糟——用户看到的
    是地图再也不动。所以超过 `max_hold_s` 就强制恢复，等下次回到普通状态后
    再次允许暂停。
    """

    def __init__(self, max_hold_s=DEFAULT_MAX_HOLD_S):
        self.max_hold_s = float(max_hold_s)
        self._since = None
        self.capped = False
        self._just_capped = False

    def want_pause(self, abnormal, now):
        if not abnormal:
            self._since = None
            self.capped = False
            self._just_capped = False
            return False
        if self._since is None:
            self._since = now
        if self.capped:
            return False
        if now - self._since > self.max_hold_s:
            self.capped = True
            self._just_capped = True
            return False
        return True

    def take_capped_event(self):
        """取一次"刚刚因为超时被放行"的事件，供上层打一条日志。"""
        if self._just_capped:
            self._just_capped = False
            return True
        return False


class SlamPauseGuard(object):
    """幂等的 pause/resume 封装（服务是 toggle 且无反馈，只能自己记账）。"""

    def __init__(self, node, service_name=DEFAULT_SERVICE, enabled=True,
                 srv_type=None):
        self.node = node
        self.enabled = bool(enabled)
        self.service_name = service_name
        self._client = None
        self._desired = False
        # slam_toolbox 一起来就是在收数据，所以初始就是 False
        self._known = False
        self._pending = False
        self._warned_missing = False
        self._srv_type = srv_type   # 测试可注入；正常情况懒加载
        self._import_failed = False

    # ---------------- 对外 ----------------
    @property
    def paused(self):
        return bool(self._known)

    def sync(self, want_paused):
        """把服务拉到想要的状态。非阻塞；已经一致就什么都不做。"""
        if not self.enabled:
            return
        want = bool(want_paused)
        self._desired = want
        if self._pending or self._known == want:
            return
        if not self._ensure_client():
            return
        if not self._client.service_is_ready():
            if not self._warned_missing:
                self._warned_missing = True
                self.node.get_logger().info(
                    '建图暂停服务还没就绪（%s）：amcl 模式下正常，忽略'
                    % self._client_name())
            return
        self._call(want)

    # ---------------- 内部 ----------------
    def _client_name(self):
        return self.service_name

    def _ensure_srv_type(self):
        """懒加载 slam_toolbox 的服务类型；没装就关掉自己，别拖累打滑检测。"""
        if self._srv_type is not None:
            return True
        if self._import_failed:
            return False
        try:
            from slam_toolbox.srv import Pause
        except Exception:                          # noqa: BLE001
            self._import_failed = True
            self.enabled = False
            self.node.get_logger().warn(
                '找不到 slam_toolbox 的 Pause 服务类型，'
                '打滑期间不会暂停建图')
            return False
        self._srv_type = Pause
        return True

    def _ensure_client(self):
        if self._client is not None:
            return True
        if not self._ensure_srv_type():
            return False
        self._client = self.node.create_client(self._srv_type,
                                               self._client_name())
        return True

    def _call(self, want):
        self._pending = True
        req = self._srv_type.Request()
        future = self._client.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_done(fut, want))

    def _on_done(self, future, want):
        self._pending = False
        try:
            future.result()
        except Exception as exc:                   # noqa: BLE001
            self.node.get_logger().warn('建图暂停调用失败：%s' % exc)
            return
        # 返回的 status 恒为 true，不可信；按 toggle 记账
        self._known = want
        self.node.get_logger().warn(
            '打滑：建图已%s' % ('暂停' if want else '继续'))
