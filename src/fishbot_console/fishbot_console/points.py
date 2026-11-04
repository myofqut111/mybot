# -*- coding: utf-8 -*-
"""点位显示用的纯数据：Marker 规格 + "从没检出过"的判定。

放这里而不放 ros_bridge 的原因：这两件事是本项目的"业务判断"（哪个点算可疑、
点位画多大、颜色怎么定），不是 ROS 细节。它们能在没有 ROS 的机器上测，
而"把 dict 拼成 MarkerArray"那一层交给 ros_bridge，那边本来就连不上 rclpy 之外
的东西。

关于颜色约定（和 RViz 里看的一致）：
- 红色 = 正常点位
- 黄色 = ★ 这条路线历史上从没在这个点检出过目标（可能真没有，也可能漏检）
"""
NORMAL = 'normal'
SUSPECT = 'suspect'

DEFAULT_DIAMETER = 0.25      # 球直径，0.25 m 在 rviz 里一眼能看见又不挡视线


def marker_specs(points, suspects=(), diameter=DEFAULT_DIAMETER,
                 label_prefix=''):
    """路线点位 -> marker 规格列表。

    points 里每项是 {'point_id': str, 'x': float, 'y': float}。
    返回的每项是 {'x','y','text','color','diameter'}，由调用方转成 Marker。
    """
    suspect_ids = set(suspects or ())
    specs = []
    for index, point in enumerate(points or ()):
        try:
            x = float(point.get('x', 0.0))
            y = float(point.get('y', 0.0))
        except (TypeError, ValueError):
            continue
        point_id = str(point.get('point_id') or '#%d' % (index + 1))
        specs.append({
            'x': x,
            'y': y,
            'text': '%s%s' % (label_prefix, point_id),
            'color': SUSPECT if point_id in suspect_ids else NORMAL,
            'diameter': float(diameter),
            'index': index,
        })
    return specs


def never_detected(records):
    """扫巡检报告里的 point 记录，返回"从来没检出过目标"的 point_id 集合。

    records 可以是多轮的报告记录混在一起；只要有一次检出过，就不算可疑——
    宁可漏标可疑点，也不要给一个其实检出过的点打黄标（那会让用户去查一个
    不存在的问题）。
    """
    seen = {}
    for record in records or ():
        if not isinstance(record, dict):
            continue
        if record.get('type') != 'point':
            continue
        point_id = record.get('point_id') or record.get('point')
        if not point_id:
            continue
        key = str(point_id)
        detections = record.get('detections') or []
        seen[key] = seen.get(key, False) or bool(detections)
    return set(key for key, detected in seen.items() if not detected)


def point_rows(records):
    """按点位汇总：每点一行（最近一次结果 + 累计次数 + 有无检出）。

    面板要的是"这条线路上哪些点一直没检出"，不是原始事件流，所以在这里聚合。
    """
    table = {}
    order = []
    for record in records or ():
        if not isinstance(record, dict) or record.get('type') != 'point':
            continue
        point_id = str(record.get('point_id') or record.get('point') or '?')
        if point_id not in table:
            table[point_id] = {
                'point_id': point_id,
                'visits': 0,
                'detected_times': 0,
                'last_status': '',
                'last_detections': [],
                'last_time': 0.0,
            }
            order.append(point_id)
        row = table[point_id]
        row['visits'] += 1
        detections = record.get('detections') or []
        if detections:
            row['detected_times'] += 1
        row['last_status'] = str(record.get('status', ''))
        row['last_detections'] = detections
        row['last_time'] = float(record.get('time') or 0.0)
    return [table[point_id] for point_id in order]
