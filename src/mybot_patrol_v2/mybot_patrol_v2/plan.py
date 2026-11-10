"""巡检作业规划：把一条路线编译成一个可逐步推进的作业队列。

这个模块**不依赖 ROS**，只做纯逻辑，因此可以直接用 ``python3`` 跑单测：

    python3 -m pytest test/ -q
    python3 test/test_plan.py

核心概念
--------
- ``PointRef``  一个巡检点里与规划有关的字段（pose 由节点自己按下标取）。
- ``Job``       一次「去某个点、做某件事」的作业；重试时 ``attempt`` 递增。
- ``Planner``   状态机：``next_job()`` 取下一个作业，``report_success()`` /
                ``report_failure()`` 回报结果，内部据此决定重试、跳过还是中止。

与旧实现最大的差别：**单点失败不再直接终止整轮**，而是按 ``FailurePolicy``
决定（默认跳过继续跑，并记录失败明细）。
"""

import random
from dataclasses import dataclass, replace
from enum import Enum
from typing import Dict, Iterable, List, Optional, Sequence

# 与 mybot_patrol_interfaces/action/ExecutePatrol.action 里的常量保持一致
MODE_ONCE = 0
MODE_REPEAT = 1
MODE_RANDOM = 2
VALID_MODES = frozenset((MODE_ONCE, MODE_REPEAT, MODE_RANDOM))

class FailurePolicy(Enum):
    """单点失败之后怎么办。"""

    STOP = 'stop'      # 立刻中止整轮（旧行为）
    SKIP = 'skip'      # 记一笔、跳过、继续下一个点（默认）
    RETRY = 'retry'    # 先重试 max_retries 次，仍失败则跳过


# 失败处理的结果，节点据此决定接下来做什么
OUTCOME_RETRY = 'retry'
OUTCOME_SKIP = 'skip'
OUTCOME_ABORT = 'abort'
OUTCOME_DONE = 'done'


@dataclass(frozen=True)
class PointRef:
    """规划层看到的一个巡检点（不含位姿）。"""

    point_id: str
    task_id: str = ''
    wait_seconds: float = 0.0


@dataclass(frozen=True)
class Job:
    """一次要去执行的点位作业。"""

    job_id: str
    loop_index: int
    route_index: int
    point_id: str
    task_id: str
    wait_seconds: float
    attempt: int = 1

    @property
    def is_retry(self) -> bool:
        """是否是一次重试。"""
        return self.attempt > 1


@dataclass(frozen=True)
class PlanConfig:
    """一轮巡检的规划参数，全部来自 action goal 或节点参数。"""

    mode: int = MODE_ONCE
    repeat_count: int = 1
    policy: FailurePolicy = FailurePolicy.SKIP
    max_retries: int = 1
    max_consecutive_failures: int = 3
    seed: Optional[int] = None

    def __post_init__(self) -> None:
        """参数合法性在构造时就卡住，避免跑到一半才发现。"""
        if self.mode not in VALID_MODES:
            raise ValueError('unsupported patrol mode: %r' % (self.mode,))
        # policy 必须是枚举：report_failure 用 `policy is FailurePolicy.RETRY`
        # 做身份比较，传字符串会静默退化成「跳过」，配置的 stop/retry 全部失效。
        if not isinstance(self.policy, FailurePolicy):
            raise ValueError('policy must be a FailurePolicy, got %r' % (self.policy,))
        if self.repeat_count < 0:
            raise ValueError('repeat_count must not be negative')
        if self.max_retries < 0:
            raise ValueError('max_retries must not be negative')
        if self.max_consecutive_failures < 1:
            raise ValueError('max_consecutive_failures must be at least 1')


def parse_policy(value: str) -> FailurePolicy:
    """把字符串参数解析成失败策略，非法值直接报错。"""
    try:
        return FailurePolicy(str(value).strip().lower())
    except ValueError:
        raise ValueError(
            'unknown failure policy %r (expected one of %s)' % (
                value, ', '.join(member.value for member in FailurePolicy)))


def total_loops(mode: int, repeat_count: int) -> Optional[int]:
    """返回总轮数；``None`` 表示一直跑下去（REPEAT/RANDOM 且次数为 0）。"""
    if mode not in VALID_MODES:
        raise ValueError('unsupported patrol mode: %r' % (mode,))
    if repeat_count < 0:
        raise ValueError('repeat_count must not be negative')
    if mode == MODE_ONCE:
        return 1
    return repeat_count or None


def order_for_loop(loop_index: int, point_count: int, mode: int, rng=None) -> List[int]:
    """返回第 ``loop_index`` 轮的点位访问顺序（RANDOM 每轮重新洗牌）。"""
    if point_count < 1:
        raise ValueError('route has no patrol points')
    if mode not in VALID_MODES:
        raise ValueError('unsupported patrol mode: %r' % (mode,))
    order = list(range(point_count))
    if mode == MODE_RANDOM:
        (rng or random).shuffle(order)
    return order


def is_permutation(order: Iterable[int], point_count: int) -> bool:
    """判断 ``order`` 是否正好是 ``0..point_count-1`` 的一个排列。"""
    try:
        values = [int(item) for item in order]
    except (TypeError, ValueError):
        return False
    return len(values) == point_count and sorted(values) == list(range(point_count))


class Planner:
    """按 ``PlanConfig`` 逐轮产出作业，并根据回报决定重试/跳过/中止。"""

    def __init__(self, points: Sequence[PointRef], config: PlanConfig,
                 already_done: Iterable[str] = (), progress: Dict[str, int] = None):
        """构造规划器。

        Args:
            points: 路线上的点，顺序即路线顺序。
            config: 规划参数。
            already_done: 断点续跑时已经完成的 job_id，遇到就跳过。
            progress: 断点续跑时恢复的游标
                ``{'loop': n, 'cursor': m, 'order': [...]}``；``order`` 可选，
                没有时按模式重新生成（RANDOM 模式下就复现不了原来的顺序）。
        """
        self._points = list(points)
        if not self._points:
            raise ValueError('route has no patrol points')
        self._config = config
        self._rng = random.Random(config.seed)
        self._total = total_loops(config.mode, config.repeat_count)
        self._done_ids = set(already_done)
        self._loop = -1
        self._order: List[int] = []
        self._cursor = 0
        self._current: Optional[Job] = None
        self._consecutive_failures = 0
        # 续跑时把上一轮已完成的作业也计入 completed_ids：
        # 报告的 completed_points 和进度文件的 completed 都读它，
        # 不补进来的话续跑后的数字会从 0 重新数。
        self._completed: List[str] = list(already_done)
        self._skipped: List[Job] = []
        self._failures: List[dict] = []
        self._terminated = False
        if progress:
            self._loop = int(progress.get('loop', -1))
            self._cursor = int(progress.get('cursor', 0))
            beyond_total = (self._total is not None
                            and self._loop >= self._total)
            if beyond_total:
                # 快照的轮次已经超过本次请求的轮数（例如上个任务跑的是
                # repeat_count=0 的无限模式），直接当作「已完成」，
                # 否则会把整条路线再跑一遍。
                self._loop = self._total - 1
            if self._loop >= 0:
                saved_order = progress.get('order') or []
                if is_permutation(saved_order, len(self._points)):
                    # 优先用上次存下的真实访问顺序：RANDOM 模式洗过牌也对得上
                    self._order = [int(index) for index in saved_order]
                else:
                    # 老快照没存顺序（或长度不符）：只能按模式重算。
                    # RANDOM 模式下随机序列不可复现，可能跳过没做完的点。
                    self._order = order_for_loop(
                        self._loop, len(self._points), config.mode, self._rng)
                # 无论走哪个分支都要钳制游标：损坏的快照里 cursor 可能是负数，
                # 负下标会让 _order[-1] 静默指到路线最后一个点，或直接 IndexError。
                self._cursor = min(max(self._cursor, 0), len(self._order))
                if beyond_total:
                    # 越过本次请求的最后一轮：把游标顶到本轮末尾，
                    # 让 next_job 直接判定「跑完了」，而不是再开一轮。
                    self._cursor = len(self._order)

    # -- 只读状态 ---------------------------------------------------
    @property
    def total_loops(self) -> Optional[int]:
        """总轮数，``None`` 表示无限。"""
        return self._total

    @property
    def total_jobs(self) -> Optional[int]:
        """这一轮要跑多少个作业；``None`` 表示无限。

        点数 × 轮数，再扣掉断点续跑时已经做完的作业——页面上的
        “成功 N/共 M”用的就是它，而这个 M 应该是**本次**要跑的量，
        不是整个路线的总量。
        """
        if self._total is None:
            return None
        planned = len(self._points) * self._total
        return max(planned - len(self._done_ids), 0)

    @property
    def loop_index(self) -> int:
        """当前正在跑第几轮（从 0 开始）。"""
        return max(self._loop, 0)

    @property
    def completed_loops(self) -> int:
        """已经完全跑完的轮数（跑满一轮才算）。

        ``loop_index`` 在整轮跑完后仍停在第 N-1 轮，直接拿它当
        ``completed_loops`` 上报会永远少一轮，所以单独算。
        """
        handled = set(self._completed)
        handled |= {job.job_id for job in self._skipped}
        point_count = len(self._points)
        top = self._loop + 1 if self._current is None else self._loop
        if self._total is not None:
            top = min(top, self._total)
        count = 0
        for loop in range(max(top, 0)):
            if all('L%dN%d' % (loop, index) in handled
                   for index in range(point_count)):
                count += 1
            else:
                break
        return count

    @property
    def cursor(self) -> int:
        """当前轮已经推进到第几个点。"""
        return self._cursor

    @property
    def completed_ids(self) -> List[str]:
        """已完成的作业 id 列表。"""
        return list(self._completed)

    @property
    def skipped(self) -> List[Job]:
        """被跳过的作业列表。"""
        return list(self._skipped)

    @property
    def failures(self) -> List[dict]:
        """失败明细（job_id / point_id / message / attempt）。"""
        return list(self._failures)

    @property
    def aborted(self) -> bool:
        """是否因为失败策略而中止。"""
        return self._terminated

    @property
    def finished(self) -> bool:
        """是否已经跑完（含中止）。"""
        return self._terminated or self._exhausted()

    def snapshot(self) -> Dict[str, object]:
        """返回可持久化的游标，用于断点续跑。

        除了 ``loop`` / ``cursor``，还带上当前这一轮的实际顺序 ``order``：
        RANDOM 模式每轮都重新洗牌，不存顺序的话续跑时游标会指到别的点上。
        """
        return {'loop': self._loop, 'cursor': self._cursor,
                'order': list(self._order)}

    # -- 推进 -------------------------------------------------------
    def next_job(self) -> Optional[Job]:
        """取下一个要执行的作业；``None`` 表示整轮结束。"""
        if self._terminated:
            return None
        if self._current is not None:
            return self._current
        while True:
            if self._exhausted():
                return None
            while self._cursor >= len(self._order):
                self._loop += 1
                if self._total is not None and self._loop >= self._total:
                    return None
                self._order = order_for_loop(
                    self._loop, len(self._points), self._config.mode, self._rng)
                self._cursor = 0
            route_index = self._order[self._cursor]
            point = self._points[route_index]
            job = Job(
                job_id='L%dN%d' % (self._loop, route_index),
                loop_index=self._loop,
                route_index=route_index,
                point_id=point.point_id,
                task_id=point.task_id,
                wait_seconds=point.wait_seconds,
            )
            if job.job_id in self._done_ids:
                self._cursor += 1          # 断点续跑：这个点上次已经做完
                continue
            self._current = job
            return job

    def report_success(self, job: Job) -> str:
        """回报一个作业成功，推进游标。"""
        if self._current is None or job.job_id != self._current.job_id:
            raise ValueError('job %r is not the active job' % (job.job_id,))
        self._completed.append(job.job_id)
        self._consecutive_failures = 0
        self._current = None
        self._cursor += 1
        return OUTCOME_DONE

    def report_failure(self, job: Job, message: str = '') -> str:
        """回报一个作业失败，按策略返回 ``retry`` / ``skip`` / ``abort``。"""
        if self._current is None or job.job_id != self._current.job_id:
            raise ValueError('job %r is not the active job' % (job.job_id,))
        self._failures.append({
            'job_id': job.job_id,
            'point_id': job.point_id,
            'attempt': job.attempt,
            'message': str(message),
        })
        policy = self._config.policy
        # 重试不算「连续失败」，否则重试会把熔断计数吃满
        if policy is FailurePolicy.RETRY and job.attempt <= self._config.max_retries:
            self._current = replace(job, attempt=job.attempt + 1)
            return OUTCOME_RETRY
        self._consecutive_failures += 1
        if (policy is FailurePolicy.STOP or
                self._consecutive_failures >= self._config.max_consecutive_failures):
            self._terminated = True
            self._current = None
            return OUTCOME_ABORT
        self._skipped.append(job)
        self._current = None
        self._cursor += 1
        return OUTCOME_SKIP

    def _exhausted(self) -> bool:
        if self._total is None:
            return False
        if self._loop < 0:
            return False
        if self._loop + 1 < self._total:
            return False
        return self._cursor >= len(self._order)
