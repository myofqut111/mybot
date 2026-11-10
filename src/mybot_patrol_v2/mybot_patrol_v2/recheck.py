"""主动视角复核：置信度不高时，走近平一点再看一眼。

为什么需要它：单帧检测在远距离、小目标、遮挡下会落进 0.25~0.6 的灰区。
这个区间里「有」和「没有」都说不准，而机器人的移动能力正好能补上感知的
不确定性——走近 0.4 m，目标在画面里大约大一倍，判决就落地了。

刻意做成纯函数（不 import ROS）：决策表可以单测，节点里只负责调导航和
发任务。这一层不碰任何消息类型，换话题换服务都不用改它。
"""

import re

DECISION_ACCEPT_PRESENT = 'accept_present'
DECISION_ACCEPT_ABSENT = 'accept_absent'
DECISION_MOVE = 'move'

# 视觉节点「什么都没检到」时的固定说法（yolo_detector._summarize_detections）
_EMPTY_HINTS = ('no object', 'none', 'nothing')

# fire_extinguisher x2 (0.92,0.91)  /  fire_extinguisher (0.88)
_ITEM = re.compile(
    r'^\s*([A-Za-z_][A-Za-z0-9_\- ]*?)\s*(?:x\s*(\d+))?\s*\(([^)]*)\)\s*$')


def parse_summary(message):
    """把视觉节点的 message 解析成 ``(数量, [置信度...])``。

    认不出来就返回 ``None``——调用方据此「不复核」，宁可按原结果收，也别
    拿解析失败的字符串去决定车往哪开。

    格式来自 ``yolo_detector._summarize_detections``：
      - 空结果：``no object detected``
      - 单个：``fire_extinguisher (0.92)``
      - 同类多个：``fire_extinguisher x2 (0.92,0.91)``
      - 不同类：用 ``;`` 连接
    """
    if not message:
        return None
    text = str(message).strip()
    if not text:
        return None
    lowered = text.lower()
    for hint in _EMPTY_HINTS:
        if lowered.startswith(hint):
            return 0, []

    total = 0
    confidences = []
    for part in text.split(';'):
        part = part.strip()
        if not part:
            continue
        match = _ITEM.match(part)
        if not match:
            return None
        count = int(match.group(2) or 1)
        values = []
        for token in match.group(3).split(','):
            token = token.strip()
            if not token:
                continue
            try:
                values.append(float(token))
            except ValueError:
                return None
        if not values:
            return None
        total += count
        confidences.extend(values)
    if not total:
        return None
    return total, confidences


def evaluate(count, confidences, moves_done, max_moves, low, high,
             on_miss=True):
    """决策表：返回 ``(decision, top_conf)``。

    ==============  =====================================
    情形            决策
    ==============  =====================================
    top >= high     接受（确定有），不再走
    low <= top < high  还有预算就走近；没预算按「不确定地接受」
    没检到 / top < low  还有预算且 on_miss 就走近；否则接受「没有」
    ==============  =====================================
    """
    top = max(confidences) if confidences else 0.0
    can_move = moves_done < max(0, int(max_moves))
    if not count or top < low:
        if can_move and on_miss:
            return DECISION_MOVE, top
        return DECISION_ACCEPT_ABSENT, top
    if top >= high:
        return DECISION_ACCEPT_PRESENT, top
    if can_move:
        return DECISION_MOVE, top
    return DECISION_ACCEPT_PRESENT, top


def merge(rounds):
    """多轮结果取「最可信的那一轮」：返回 ``(count, confidences, top_conf)``。

    取最大置信度而不是平均：走近之后目标应该更清楚，平均会把「近处的确定」
    和「远处的模糊」搅在一起，反而让结论变糊。
    """
    best = (0, [], 0.0)
    for count, confidences in rounds:
        top = max(confidences) if confidences else 0.0
        if top > best[2]:
            best = (int(count), list(confidences), top)
    return best
