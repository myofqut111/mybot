"""PausePolicy 的单测：什么时候该摁住建图、什么时候该放行。"""

from mybot_slip.slam_guard import PausePolicy


def test_normal_state_never_pauses():
    p = PausePolicy(max_hold_s=15.0)
    assert p.want_pause(False, 0.0) is False
    assert p.want_pause(False, 100.0) is False


def test_pauses_while_abnormal():
    p = PausePolicy(max_hold_s=15.0)
    assert p.want_pause(True, 10.0) is True
    assert p.want_pause(True, 12.0) is True


def test_caps_after_max_hold():
    p = PausePolicy(max_hold_s=15.0)
    assert p.want_pause(True, 10.0) is True
    assert p.want_pause(True, 20.0) is True
    # 超过 15 s：放行，并且只报一次
    assert p.want_pause(True, 26.0) is False
    assert p.take_capped_event() is True
    assert p.take_capped_event() is False
    # 之后一直放行，不再反复暂停
    assert p.want_pause(True, 40.0) is False


def test_recovers_cap_when_back_to_normal():
    p = PausePolicy(max_hold_s=5.0)
    assert p.want_pause(True, 0.0) is True
    assert p.want_pause(True, 6.0) is False      # 超时放行
    assert p.want_pause(False, 7.0) is False     # 回到普通
    # 下一次打滑重新开始计时
    assert p.want_pause(True, 20.0) is True


def test_short_slip_never_caps():
    p = PausePolicy(max_hold_s=15.0)
    assert p.want_pause(True, 0.0) is True
    assert p.want_pause(True, 3.0) is True
    assert p.want_pause(False, 4.0) is False
    assert p.take_capped_event() is False
