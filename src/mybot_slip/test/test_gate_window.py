"""建图门控里"按时间窗取平均"的判据（slip_node._window_mean）。"""

from collections import deque

from mybot_slip.slip_node import _trim_hist, _window_mean


def test_mean_over_recent_window():
    h = deque([(0.0, 0.0, 0.0), (0.2, 0.10, 0.2), (0.4, 0.20, 0.4)])
    got = _window_mean(h, 0.4, 0.4)
    assert got is not None
    mean_vx, mean_wz, n = got
    assert n == 3                      # 0.0 也算在 [0.0, 0.4] 里
    assert abs(mean_vx - 0.10) < 1e-9
    assert abs(mean_wz - 0.20) < 1e-9


def test_empty_or_stale_returns_none():
    assert _window_mean(deque(), 1.0, 0.4) is None
    assert _window_mean(deque([(0.0, 0.1, 0.0)]), 5.0, 0.4) is None


def test_trim_hist_keeps_window():
    h = deque([(0.0, 0, 0), (1.0, 0, 0), (2.5, 0, 0)])
    _trim_hist(h, 3.0, keep_s=2.0)
    assert list(h)[0][0] == 1.0
