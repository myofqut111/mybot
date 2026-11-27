# -*- coding: utf-8 -*-
"""差速轮打滑检测与恢复（方案 v1.1）。

模块划分（都是纯逻辑，不依赖 ROS，便于离线回放与单测）：
    windows  —— 时间窗聚合
    ranges   —— 两帧激光求真实平移（ICP 一步）
    criteria —— P1~P5 判据与滞回
    state    —— 恢复状态机（按类型分流）
    slip_node—— ROS 节点：订阅五路输入，发 /slip/state
"""

__version__ = "0.13.1"
