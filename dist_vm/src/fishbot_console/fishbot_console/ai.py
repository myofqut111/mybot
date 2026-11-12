#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AI 助手：把"人话"变成"可以执行的动作"，但**动作必须过白名单**。

设计原则：模型只会输出 JSON，真正执行的是服务端的白名单校验器。
模型说错话最多是不回话，不可能让它去发一个不存在的动作或者把车开进墙里
（坐标会被夹到地图范围内；不在白名单的 type 直接丢弃）。

本模块不联网、不 import rclpy，可单测；真正的 HTTP 调用在 `chat()` 里，
失败就抛异常，由上层退回规则回答。
"""
import json
import math
import re

ALLOWED_TYPES = ('nav', 'patrol', 'pause', 'resume', 'cancel_nav', 'snapshot')

SYSTEM_PROMPT = """你是一台消防巡检机器人的操作助手。用户会说中文，你要用中文回答，简短、具体。

你可以建议一个动作（也可以什么都不建议）。如果要建议动作，必须在回答的最后输出一个 JSON 对象，
严格按下面的格式，不要加注释、不要加别的字段：

```json
{"reply": "给用户看的一句话", "action": {"type": "nav", "x": 1.2, "y": -0.5}}
```

可用的动作类型：
- nav: 去某个地图坐标。字段 x, y（米），可选 yaw（弧度）
- patrol: 开始巡检。可选 route（路线名）、mode（0=单次,1=重复,2=随机）、repeat（轮数）
- pause: 暂停当前巡检
- resume: 恢复巡检
- cancel_nav: 取消当前导航目标
- snapshot: 不需要动作，只是想把当前状态读一遍

规则：
1. 只有用户明确要求"去/走/过去/导航到"某处时才用 nav；坐标要来自地图或用户给的数字，
   没有依据就不要猜，直接回答"我需要一个坐标"。
2. 用户只是问状态、问电量、聊天，就只回话，不要给 action。
3. 安全相关的操作（暂停、取消）可以直接建议。
4. `reply` 里不要说"我将要执行"，因为动作要先经过校验，可能被拒绝。
"""


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or math.isinf(number):
        return None
    return number


def clamp(value, low, high):
    return max(low, min(high, value))


def validate_action(action, bounds=None):
    """白名单校验 + 归一化。返回 (action|None, reason)。

    bounds 形如 {'min_x':..,'max_x':..,'min_y':..,'max_y':..}，
    给了就把 nav 坐标夹进去 —— 模型算错一个数量级的坐标是常见错误。
    """
    if not isinstance(action, dict):
        return None, 'action 不是对象'
    kind = str(action.get('type') or '').strip().lower()
    if kind not in ALLOWED_TYPES:
        return None, '不支持的动作：%s' % (kind or '(空)')

    if kind == 'nav':
        x = _finite(action.get('x'))
        y = _finite(action.get('y'))
        if x is None or y is None:
            return None, 'nav 需要数字 x/y'
        yaw = _finite(action.get('yaw'))
        if bounds:
            x = clamp(x, bounds.get('min_x', -100.0), bounds.get('max_x', 100.0))
            y = clamp(y, bounds.get('min_y', -100.0), bounds.get('max_y', 100.0))
        out = {'type': 'nav', 'x': x, 'y': y}
        if yaw is not None:
            out['yaw'] = yaw
        return out, None

    if kind == 'patrol':
        route = str(action.get('route') or '').strip()[:32]
        mode = _finite(action.get('mode'))
        repeat = _finite(action.get('repeat'))
        out = {'type': 'patrol', 'route': route or 'main'}
        if mode is not None:
            out['mode'] = int(clamp(mode, 0, 2))
        if repeat is not None:
            out['repeat'] = int(clamp(repeat, 1, 50))
        return out, None

    return {'type': kind}, None


def _extract_json_candidates(text):
    """从回答里挖出 JSON 对象候选（先找 ``` 围栏，再退化为整体扫描）。"""
    if not text:
        return []
    candidates = []
    for block in re.findall(r'```(?:json)?\s*(.*?)```', text, re.S):
        block = block.strip()
        if block.startswith('{'):
            candidates.append(block)
    stripped = text.strip()
    if stripped.startswith('{'):
        candidates.append(stripped)
    for match in re.finditer(r'\{[^{}]*"action"[^{}]*\}', text, re.S):
        candidates.append(match.group(0))
    return candidates


def parse_model_output(text, bounds=None):
    """模型回答 -> {'reply': str, 'action': dict|None, 'reason': str}。

    解析失败也算成功（reply 用原文），只是没有 action —— 宁可不动，不可乱动。
    """
    text = '' if text is None else str(text)
    for candidate in _extract_json_candidates(text):
        try:
            payload = json.loads(candidate)
        except ValueError:
            continue
        if not isinstance(payload, dict):
            continue
        reply = payload.get('reply')
        action, reason = validate_action(payload.get('action'), bounds)
        if reply is None:
            reply = text
        return {
            'reply': str(reply).strip(),
            'action': action,
            'reason': reason,
            'parsed': True,
        }
    plain = re.sub(r'```.*?```', '', text, flags=re.S).strip()
    return {'reply': plain or '（模型没有给出内容）', 'action': None,
            'reason': '', 'parsed': False}


def rule_reply(question, context):
    """没接模型时的兜底：只回答能从状态里直接读出来的问题。

    这一层存在的意义：**没有 API key 的机器上，AI 面板也不是一块死板子**。
    """
    text = (question or '').strip()
    ctx = context or {}
    state = ctx.get('state') or '未知'
    battery = ctx.get('battery_v')
    band = ctx.get('battery_band') or 'unknown'
    point = ctx.get('point_id') or ''
    progress = '%d 点' % int(ctx.get('completed_points') or 0)
    route = ctx.get('route_name') or ''

    if not text:
        return {'reply': '你可以问我「现在什么状态」「还剩多少电」「去 x=1 y=2」。',
                'action': None, 'reason': '', 'parsed': False}

    if any(key in text for key in ('电量', '电压', '电池', '多少电')):
        if battery is None:
            return {'reply': '还没有收到电池电压（固件没发或话题没起来）。',
                    'action': None, 'reason': '', 'parsed': False}
        level = {'ok': '正常', 'warn': '偏低', 'crit': '告警'}.get(band, '未知')
        return {'reply': '当前 %.2f V，%s。' % (battery, level),
                'action': None, 'reason': '', 'parsed': False}

    if any(key in text for key in ('状态', '在跑', '进度', '到哪', '第几个')):
        extra = ''
        if route:
            extra = '，路线 %s' % route
        if point:
            extra += '，当前点位 %s' % point
        return {'reply': '现在 %s%s，已跑 %s。' % (state, extra, progress),
                'action': None, 'reason': '', 'parsed': False}

    # 先判"恢复/继续"：它本身不含"停"字，但"别停，继续"这种话里两个都出现，
    # 顺序放在前面才不会被误判成暂停。
    if any(key in text for key in ('恢复', '继续', '接着')):
        return {'reply': '好的，恢复巡检。', 'action': {'type': 'resume'},
                'reason': '', 'parsed': False}

    if any(key in text for key in ('停', '暂停')):
        return {'reply': '好的，暂停巡检。', 'action': {'type': 'pause'},
                'reason': '', 'parsed': False}

    if any(key in text for key in ('取消导航', '别走了', '停下导航')):
        return {'reply': '好的，取消当前导航。', 'action': {'type': 'cancel_nav'},
                'reason': '', 'parsed': False}

    coords = _parse_xy(text)
    if coords and any(key in text for key in ('去', '走', '导航', '到')):
        return {'reply': '准备去 (%.2f, %.2f)。' % coords,
                'action': {'type': 'nav', 'x': coords[0], 'y': coords[1]},
                'reason': '', 'parsed': False}

    return {'reply': '（没接大模型）我能答状态、电量、暂停/恢复、取消导航，'
                     '也能按“去 x=1.2 y=-0.5”发导航。要自由对话需要配 AI 的 '
                     'OPENAI_BASE_URL / OPENAI_API_KEY。',
            'action': None, 'reason': '', 'parsed': False}


def _parse_xy(text):
    """从「去 x=1.2 y=-0.5」「去 x:1.5 y 2.5」「去 (1.2, -0.5)」里抠坐标。

    分隔符是可选的：人手打字时 `x=1 y=2`、`x:1 y:2`、`x 1 y 2` 都会出现。
    """
    match = re.search(r'x\s*[=:：]?\s*(-?\d+(?:\.\d+)?)[^0-9\-+]{0,12}?'
                      r'y\s*[=:：]?\s*(-?\d+(?:\.\d+)?)', text, re.S)
    if match:
        return float(match.group(1)), float(match.group(2))
    match = re.search(r'[（(]\s*(-?\d+(?:\.\d+)?)\s*[,，]\s*'
                      r'(-?\d+(?:\.\d+)?)\s*[）)]', text)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None


def context_block(context):
    """给模型的状态摘要（小而准，别把几千行日志塞进去）。"""
    ctx = context or {}
    lines = [
        '当前状态：%s' % (ctx.get('state') or '未知'),
        '路线：%s' % (ctx.get('route_name') or '未选'),
        '已完成点数：%s' % ctx.get('completed_points', 0),
        '当前点位：%s' % (ctx.get('point_id') or '无'),
    ]
    if ctx.get('battery_v') is not None:
        lines.append('电池：%.2f V（%s）' % (ctx['battery_v'],
                                           ctx.get('battery_band') or 'unknown'))
    if ctx.get('map'):
        grid = ctx['map']
        lines.append('地图：%dx%d 分辨率 %.3f m，范围 x[%.1f, %.1f] y[%.1f, %.1f]'
                     % (grid.get('width', 0), grid.get('height', 0),
                        grid.get('resolution', 0.05),
                        grid.get('min_x', 0), grid.get('max_x', 0),
                        grid.get('min_y', 0), grid.get('max_y', 0)))
    if ctx.get('errors'):
        lines.append('最近错误：%s' % ctx['errors'])
    return '\n'.join(lines)
