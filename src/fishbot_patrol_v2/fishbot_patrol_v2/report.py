"""巡检报告：把「哪个点、什么时候、导航多久、拍到什么」落盘，事后可查。

输出两份文件：

- ``patrol_<时间>_<exec>.jsonl``  每个点位一行，逐行追加，中途断电也不丢已写的行
- ``patrol_<时间>_<exec>.summary.json``  结束时写入的汇总（各状态计数、发现清单）

同样不依赖 ROS，方便单测。
"""

import json
import os
from datetime import datetime
from typing import Callable, Dict, List, Optional

STATUS_OK = 'ok'
STATUS_SKIPPED = 'skipped'
STATUS_FAILED = 'failed'
STATUS_CANCELED = 'canceled'


def _now_iso() -> str:
    """当前时间，精确到秒。"""
    return datetime.now().isoformat(timespec='seconds')


def _as_float(value, default: float = 0.0) -> float:
    """把外部来的值转成 float；转不了就用默认值。

    detections 来自另一个节点，字段可能是 ``null``/``"high"``/缺失，直接
    ``float()`` 会抛异常并让整趟巡检记成 internal error。
    """
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _normalize_detections(detections) -> List[dict]:
    """把视觉结果统一成 ``[{'label','confidence','box'}, ...]``。"""
    normalized = []
    for item in detections or []:
        if isinstance(item, dict):
            label = str(item.get('label', ''))
            confidence = _as_float(item.get('confidence'))
            box = item.get('box') or []
        else:
            label = str(getattr(item, 'label', ''))
            confidence = _as_float(getattr(item, 'confidence', None))
            box = getattr(item, 'box', []) or []
        normalized.append({
            'label': label,
            'confidence': round(confidence, 4),
            'box': [round(_as_float(value), 2) for value in box],
        })
    return normalized


def summarize(records: List[dict]) -> Dict[str, object]:
    """从点位记录算出汇总：状态计数 + 发现次数 + 期望值判定的混淆矩阵。"""
    status_counts: Dict[str, int] = {}
    detections_total = 0
    found_by_point: Dict[str, int] = {}
    verdict_counts = {'tp': 0, 'fp': 0, 'fn': 0, 'tn': 0}
    for record in records:
        if record.get('type') != 'point':
            continue
        status = str(record.get('status', ''))
        status_counts[status] = status_counts.get(status, 0) + 1
        detections = record.get('detections') or []
        detections_total += len(detections)
        if detections:
            point_id = str(record.get('point_id', ''))
            found_by_point[point_id] = found_by_point.get(point_id, 0) + len(detections)
        verdict = str(record.get('verdict') or '')
        if verdict in verdict_counts:
            verdict_counts[verdict] += 1

    summary = {
        'points': status_counts,
        'detections_total': detections_total,
        'points_with_detections': found_by_point,
        'verdicts': dict(verdict_counts),
    }
    summary.update(_metrics_from_verdicts(verdict_counts))
    return summary


def _metrics_from_verdicts(counts: Dict[str, int]) -> Dict[str, object]:
    """把混淆矩阵（只统计填了 expect 的点）换算成三个比率。

    分母为 0 时给 ``None`` 而不是 0.0：0.0 会被读成“准确率为零”，
    而实际含义是“这趟没有可判定的点”。
    """
    true_positive = counts.get('tp', 0)
    false_positive = counts.get('fp', 0)
    false_negative = counts.get('fn', 0)
    true_negative = counts.get('tn', 0)
    graded = true_positive + false_positive + false_negative + true_negative
    if graded <= 0:
        return {'graded_points': 0, 'recall': None, 'precision': None,
                'false_alarm_rate': None}

    def ratio(numerator: int, denominator: int):
        return round(numerator / denominator, 4) if denominator else None

    return {
        'graded_points': graded,
        # 召回看的是“该有的有没有找到”：tp / (tp + fn)
        'recall': ratio(true_positive, true_positive + false_negative),
        # 精确率看的是“报出来的有多少是真的”：tp / (tp + fp)
        'precision': ratio(true_positive, true_positive + false_positive),
        # 误报率看的是“本来没有的地方报了几次”：fp / (fp + tn)
        'false_alarm_rate': ratio(false_positive, false_positive + true_negative),
    }


class PatrolReport:
    """一次巡检的报告写入器。"""

    def __init__(self, directory: str, route_name: str, execution_id: str,
                 mode: int, policy: str, clock: Callable[[], str] = _now_iso):
        """创建报告文件并写入表头。"""
        self._clock = clock
        self._directory = os.path.expanduser(directory)
        os.makedirs(self._directory, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        short_id = (execution_id or 'run')[:8]
        base = 'patrol_%s_%s' % (stamp, short_id)
        self.path = os.path.join(self._directory, base + '.jsonl')
        self.summary_path = os.path.join(self._directory, base + '.summary.json')
        self._records: List[dict] = []
        self._closed = False
        self._writable = True
        self._stream = open(self.path, 'w', encoding='utf-8')
        self._write({
            'type': 'header',
            'time': self._clock(),
            'route_name': route_name,
            'execution_id': execution_id,
            'mode': int(mode),
            'policy': str(policy),
        })

    # -- 写入 -------------------------------------------------------
    def point(self, job, status: str, nav_seconds: float = 0.0,
              task_seconds: float = 0.0, message: str = '',
              detections=None, image_path: str = '',
              expect: str = '', verdict: str = '',
              expect_source: str = '') -> dict:
        """记录一个点位的执行结果。

        ``expect``/``verdict`` 只在该点位写了期望值时才填：一个是路线里声明的
        “这里应该有没有”，一个是实际比对出来的 tp/fp/fn/tn。事后统计准确率
        全靠这两列，所以哪怕点位失败也要记（失败时 detected 视为 False）。

        ``expect_source`` 说明这个期望值哪来的：``manual`` 是录点时手选的，
        ``baseline`` 是上一趟学出来的。事后怀疑“是不是自己学歪了”，就看这列。
        """
        record = {
            'type': 'point',
            'time': self._clock(),
            'job_id': str(getattr(job, 'job_id', '')),
            'point_id': str(getattr(job, 'point_id', '')),
            # 任务类型必须记下来：报告页的「任务」列一直是空的，因为事件里
            # 根本没有这个字段，空单元格会把后面的「轮/序号」挤到前面去。
            'task_id': str(getattr(job, 'task_id', '')),
            'loop_index': int(getattr(job, 'loop_index', 0)),
            'route_index': int(getattr(job, 'route_index', 0)),
            'attempt': int(getattr(job, 'attempt', 1)),
            'status': status,
            'nav_seconds': round(float(nav_seconds), 3),
            'task_seconds': round(float(task_seconds), 3),
            'message': str(message),
            'detections': _normalize_detections(detections),
        }
        if expect:
            record['expect'] = str(expect)
            record['verdict'] = str(verdict or 'unknown')
            if expect_source:
                record['expect_source'] = str(expect_source)
        if image_path:
            record['image'] = str(image_path)
        self._write(record)
        return record

    def event(self, kind: str, **fields) -> dict:
        """记录一条非点位事件（暂停/恢复/抢占/中止等）。"""
        record = {'type': 'event', 'time': self._clock(), 'kind': str(kind)}
        record.update(fields)
        self._write(record)
        return record

    def close(self, extra: Optional[dict] = None) -> str:
        """写汇总并关闭文件，返回本次可用的文件路径。

        报告层只负责「记录」，不负责控制流：调用点在 ``finally`` 里，任何
        I/O 异常抛出去都会跳过 set_succeeded/abort，让 goal 永远不结束。
        所以这里全兜住，summary 写不出来就退回 jsonl 路径。
        """
        if self._closed:
            return self.summary_path
        summary = summarize(self._records)
        summary['type'] = 'summary'
        summary['time'] = self._clock()
        if extra:
            summary.update(extra)
        self._write(summary)
        try:
            self._stream.flush()
            self._stream.close()
        except Exception:  # noqa: BLE001 - 关不掉也要继续往下走
            pass
        self._closed = True
        try:
            temporary = self.summary_path + '.tmp'
            with open(temporary, 'w', encoding='utf-8') as stream:
                json.dump(summary, stream, ensure_ascii=False, indent=2,
                          sort_keys=True)
            os.replace(temporary, self.summary_path)
        except Exception:  # noqa: BLE001 - 汇总失败就只用 jsonl
            return self.path
        return self.summary_path

    # -- 内部 -------------------------------------------------------
    def _write(self, record: dict) -> None:
        self._records.append(record)
        if self._closed or not self._writable:
            return
        try:
            self._stream.write(json.dumps(record, ensure_ascii=False) + '\n')
            self._stream.flush()
        except Exception:  # noqa: BLE001 - 磁盘满/权限变化不该中断巡检
            self._writable = False

    def __del__(self):
        """兜底关闭文件句柄，避免节点异常退出时泄漏 fd。"""
        try:
            if not self._closed:
                self._stream.close()
                self._closed = True
        except Exception:
            pass
