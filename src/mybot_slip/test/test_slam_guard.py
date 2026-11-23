"""SlamPauseGuard 的单测。

关键事实（已在真 slam_toolbox 上实测）：服务是 toggle，返回的 status 恒为
true，没有任何信息量。所以这里验证的是"自己记账"是否正确：
- 状态一致时一次都不调
- 需要改变时只调一次（不能连调两次，否则等于没暂停）
- 调用失败不改账，下次再试
"""

from types import SimpleNamespace

from mybot_slip.slam_guard import SlamPauseGuard


class StubSrv(object):
    class Request(object):
        pass


class FakeFuture(object):
    def __init__(self, resp, exc=None):
        self._resp = resp
        self._exc = exc

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._resp


class FakeClient(object):
    def __init__(self, ready=True, fail=False):
        self.calls = 0
        self._ready = ready
        self._fail = fail

    def service_is_ready(self):
        return self._ready

    def call_async(self, _req):
        self.calls += 1
        if self._fail:
            return FakeFuture(None, exc=RuntimeError('boom'))
        return FakeFuture(SimpleNamespace(status=True))   # 恒为 true


class FakeNode(object):
    def __init__(self):
        self.logs = []
        self.client = None

    def create_client(self, srv_type, name):
        return self.client

    def get_logger(self):
        logs = self.logs

        class _L(object):
            def warn(self, msg):
                logs.append(('warn', msg))

            def info(self, msg):
                logs.append(('info', msg))

            def error(self, msg):
                logs.append(('error', msg))
        return _L()


def _guard(ready=True, fail=False, enabled=True):
    node = FakeNode()
    node.client = FakeClient(ready=ready, fail=fail)
    return SlamPauseGuard(node, enabled=enabled, srv_type=StubSrv), node


def test_pause_calls_service_once():
    g, node = _guard()
    g.sync(True)
    assert node.client.calls == 1
    assert g.paused is True


def test_no_double_call_despite_useless_status():
    # 返回恒 True，但绝不能因为"状态对不上"再补一次
    g, node = _guard()
    g.sync(True)
    assert node.client.calls == 1


def test_already_in_state_no_call():
    g, node = _guard()
    g.sync(True)
    g.sync(True)
    assert node.client.calls == 1


def test_resume_after_pause():
    g, node = _guard()
    g.sync(True)
    g.sync(False)
    assert node.client.calls == 2
    assert g.paused is False


def test_starts_unpaused():
    g, _ = _guard()
    assert g.paused is False
    g.sync(False)          # 一开始就是运行态，不该调服务
    assert g._client is None or g._client.calls == 0


def test_service_not_ready_is_quiet():
    g, node = _guard(ready=False)
    g.sync(True)
    assert node.client.calls == 0
    assert g.paused is False
    g.sync(True)           # 只提示一次
    assert len([x for x in node.logs if x[0] == 'info']) == 1


def test_failure_keeps_state_and_retries():
    g, node = _guard(fail=True)
    g.sync(True)
    assert g.paused is False          # 调用失败，账不改
    node.client._fail = False
    g.sync(True)                      # 下次照样会试
    assert node.client.calls == 2
    assert g.paused is True


def test_disabled_guard_does_nothing():
    g, node = _guard(enabled=False)
    g.sync(True)
    assert node.client.calls == 0
    assert g.paused is False
