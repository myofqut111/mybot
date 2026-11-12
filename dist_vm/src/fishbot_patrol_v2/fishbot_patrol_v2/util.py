"""纯函数工具：不 import ROS，可以直接单测。

这些函数原本长在 ``node.py`` 里，但 ``node.py`` 需要 rclpy 才能 import，
在没有 ROS 的机器上就没法测。挪出来之后，跑到点校验、结果解析、位姿换算
都能用普通 ``python3`` 验证。
"""

import json
import math
import re
import time

# 视觉节点「什么都没有」时的固定说法（yolo_detector._summarize_detections）。
_EMPTY_HINTS = ('no object', 'none', 'nothing')
# fire_extinguisher x2 (0.92,0.91) / fire_extinguisher (0.88)
_ITEM_RE = re.compile(
    r'^\s*([A-Za-z_][A-Za-z0-9_\- ]*?)\s*(?:x\s*(\d+))?\s*\(([^)]*)\)\s*$')
# 复核过的点位会在结果后面追加 [recheck: +0.80m x2, top=0.00]
_TRAILING_NOTE_RE = re.compile(r'\s*\[[^\]]*\]\s*$')


def route_index_of(job) -> int:
    """作业对应的路线下标。"""
    return int(getattr(job, 'route_index', 0))


def as_check(check):
    """把「无参回调 or 现成布尔值」统一成一个无参回调。

    rclpy 的 ``ServerGoalHandle.is_cancel_requested`` 是**属性**不是方法，
    历史上被误当成回调传给导航层，于是每个点位一开始导航就抛
    ``'bool' object is not callable``，整轮巡检在第一个点就结束。
    把归一化收在一处，任何调用方传布尔都不会再炸。
    """
    if check is None:
        return lambda: False
    if callable(check):
        return check
    frozen = bool(check)
    return lambda: frozen


EXPECT_PRESENT = 'present'
EXPECT_ABSENT = 'absent'
VALID_EXPECTS = ('', EXPECT_PRESENT, EXPECT_ABSENT)


def normalize_expect(value) -> str:
    """把期望值规范成 ``present`` / ``absent`` / ``''``。

    存路线的人可能写 ``Present``、``PRESENT``，也可能手改 json 写了不认识的
    词。认不出来的一律当“没填期望值”，绝不猜——猜错会让报告的准确率指标
    直接失真，比没有指标更糟。
    """
    text = str(value or '').strip().lower()
    return text if text in (EXPECT_PRESENT, EXPECT_ABSENT) else ''


def verdict_of(expect, detected) -> str:
    """按“这点该有什么”与“实际看到了什么”算出 tp/fp/fn/tn。

    这是自动化指标的地基：点位元数据当 ground truth，每跑一轮自动产出一批
    带标签的样本，不用人再去数。没填期望值时返回 ``unknown``，既不进分子
    也不进分母。
    """
    expect = normalize_expect(expect)
    if not expect:
        return 'unknown'
    present = bool(detected)
    if expect == EXPECT_PRESENT:
        return 'tp' if present else 'fn'
    return 'fp' if present else 'tn'


def parse_detections(message: str):
    """把任务结果解析成检测列表（``[{'label':..., 'confidence':...}]``）。

    视觉节点给过两种格式，两种都得认：
      - JSON：``{"detections": [{"label": "fire_extinguisher", "confidence": 0.9}]}``
      - 人读文本：``fire_extinguisher x2 (0.92,0.91)`` / ``no object detected``

    以前只认 JSON，而视觉节点回的是文本，于是**每一轮都解析成空**——报告里
    检测总数恒为 0、tt/fp/fn/tn 全判成「未发现」，基线也就把每个点都学成
    absent，之后每次巡检都“正确地”什么都没发现。
    """
    if not message:
        return []
    text = str(message).strip()
    if not text:
        return []
    # 复核过的点位尾巴上带一段注释，先把注释剥掉再解析。
    text = _TRAILING_NOTE_RE.sub('', text).strip()
    if text.startswith('{'):
        try:
            payload = json.loads(text)
        except ValueError:
            return []
        if not isinstance(payload, dict):
            return []
        detections = payload.get('detections')
        return detections if isinstance(detections, list) else []

    lowered = text.lower()
    for hint in _EMPTY_HINTS:
        if lowered.startswith(hint):
            return []

    out = []
    for part in text.split(';'):
        part = part.strip()
        if not part:
            continue
        match = _ITEM_RE.match(part)
        if not match:
            return []
        label = match.group(1).strip()
        try:
            count = int(match.group(2) or 1)
        except ValueError:
            return []
        scores = []
        for token in match.group(3).split(','):
            token = token.strip()
            if not token:
                continue
            try:
                scores.append(round(float(token), 4))
            except ValueError:
                return []
        if not scores:
            return []
        for index in range(max(count, 1)):
            score = scores[index] if index < len(scores) else scores[0]
            out.append({'label': label, 'confidence': score})
    return out


def yaw_of(pose) -> float:
    """从四元数取平面偏航角。

    先归一化：发布端给出非单位四元数（有些 TF 广播只填了粗略值）时，
    直接套 atan2 公式会算出错误角度（实测 2 倍放大的 90° 会被算成 126.9°）。
    全零四元数按 0 处理——路线点没填 orientation 时它就是单位朝向。
    """
    quaternion = pose.orientation
    norm = math.sqrt(quaternion.x ** 2 + quaternion.y ** 2
                     + quaternion.z ** 2 + quaternion.w ** 2)
    if not math.isfinite(norm) or norm < 1e-6:
        return 0.0
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def angle_diff(first: float, second: float) -> float:
    """两角之间的最短距离（绝对值，弧度）。"""
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def pose_error(current, target):
    """返回 ``(位置误差 m, 偏航误差 rad)``。"""
    position_error = math.hypot(current.position.x - target.position.x,
                                current.position.y - target.position.y)
    return position_error, angle_diff(yaw_of(current), yaw_of(target))


def stable_residual(sample, duration_s=0.5, interval_s=0.05):
    """在 ``duration_s`` 内反复采样，返回窗口内**最坏**的 ``(位置误差, 偏航误差)``。

    ``sample()`` 每次返回 ``(position_error, yaw_error)``，返回 ``None`` 表示这一次
    拿不到可信位姿；只要有一次拿不到，整体就返回 ``None``（算「无法校验」），
    不用单帧蒙混过关。

    为什么要取最坏值而不是只测一帧：Nav2 报成功的那一刻，车可能还在飘，
    单帧恰好落进容差就会误判「到位了」。半个秒的窗口能抓住这种抖。
    """
    deadline = time.monotonic() + max(0.0, float(duration_s))
    worst_position = 0.0
    worst_yaw = 0.0
    while True:
        current = sample()
        if current is None:
            return None
        position_error = float(current[0])
        yaw_error = float(current[1])
        # NaN/inf 会让 max() 静默丢弃这一帧（NaN 与任何数比较都是 False），
        # 一路下来返回 (0.0, 0.0) 变成「误差为 0」的假到点，所以这里显式拒绝。
        if not (math.isfinite(position_error) and math.isfinite(yaw_error)):
            return None
        worst_position = max(worst_position, position_error)
        worst_yaw = max(worst_yaw, yaw_error)
        if time.monotonic() >= deadline:
            return worst_position, worst_yaw
        time.sleep(interval_s)
