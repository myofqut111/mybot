"""ROS-independent state machine used by the patrol task manager node."""

from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum
import threading
from typing import List, Optional, Tuple


UNKNOWN = 0
RUNNING = 1
SUCCEEDED = 2
FAILED = 3
CANCELED = 4
TERMINAL_STATUSES = frozenset((SUCCEEDED, FAILED, CANCELED))


@dataclass(frozen=True)
class TaskRequest:
    """Normalized task request consumed by the core state machine."""

    execution_id: str
    route_name: str
    point_index: int
    point_id: str
    task_id: str
    empty_wait_seconds: float

    @property
    def key(self) -> Tuple[str, int]:
        """Return the execution-scoped point identifier."""
        return self.execution_id, self.point_index


@dataclass(frozen=True)
class TaskResult:
    """Normalized task status or terminal result."""

    execution_id: str
    route_name: str
    point_index: int
    point_id: str
    task_id: str
    status: int
    message: str

    @property
    def key(self) -> Tuple[str, int]:
        """Return the execution-scoped point identifier."""
        return self.execution_id, self.point_index


class EventKind(Enum):
    """Output channel selected for a core event."""

    DISPATCH = 'dispatch'
    STATUS = 'status'
    RESULT = 'result'


@dataclass(frozen=True)
class Event:
    """State-machine output emitted through one ROS topic."""

    kind: EventKind
    result: TaskResult


@dataclass
class _Pending:
    request: TaskRequest
    deadline: float
    empty_due: Optional[float]
    last_status: int = RUNNING


class TaskManagerCore:
    """
    Track task requests without depending on ROS or wall-clock time.

    The caller supplies monotonic timestamps. Requests are keyed by
    ``(execution_id, point_index)``. An exact duplicate is idempotent; a
    conflicting duplicate is rejected while the original remains active.
    """

    def __init__(self, task_timeout_s: float, completed_cache_size: int = 256):
        """Create a manager with a fixed task timeout and result cache."""
        if task_timeout_s <= 0.0:
            raise ValueError('task_timeout_s must be greater than zero')
        if completed_cache_size < 1:
            raise ValueError('completed_cache_size must be positive')
        self._task_timeout_s = task_timeout_s
        self._completed_cache_size = completed_cache_size
        self._pending = {}
        self._completed = OrderedDict()
        self._lock = threading.Lock()

    def accept(self, request: TaskRequest, now: float) -> List[Event]:
        """Accept, replay, or reject one request at monotonic time ``now``."""
        with self._lock:
            pending = self._pending.get(request.key)
            if pending is not None:
                if pending.request == request:
                    return [Event(EventKind.STATUS,
                                  self._status(pending.request,
                                               pending.last_status,
                                               'duplicate request ignored'))]
                return [Event(
                    EventKind.STATUS,
                    self._status(request, FAILED,
                                 'conflicting duplicate rejected; first '
                                 'request remains active'))]

            completed = self._completed.get(request.key)
            if completed is not None and completed.task_id == request.task_id:
                self._completed.move_to_end(request.key)
                return [Event(EventKind.STATUS, completed),
                        Event(EventKind.RESULT, completed)]

            wait_s = request.empty_wait_seconds
            if wait_s <= 0.0:
                wait_s = 1.0
            pending = _Pending(
                request=request,
                deadline=now + self._task_timeout_s,
                empty_due=now + wait_s if not request.task_id else None,
            )
            self._pending[request.key] = pending

            if not request.task_id:
                return []

            running = self._status(request, RUNNING, 'task accepted')
            return [Event(EventKind.STATUS, running),
                    Event(EventKind.DISPATCH, running)]

    def receive_external(
            self, external: TaskResult, now: float) -> List[Event]:
        """Consume an external status matching an active execution point."""
        del now  # Kept in the API so callers consistently use monotonic time.
        with self._lock:
            pending = self._pending.get(external.key)
            if pending is None or not pending.request.task_id:
                return []

            normalized = TaskResult(
                execution_id=pending.request.execution_id,
                route_name=pending.request.route_name,
                point_index=pending.request.point_index,
                point_id=pending.request.point_id,
                task_id=pending.request.task_id,
                status=external.status,
                message=external.message,
            )
            if external.status not in TERMINAL_STATUSES:
                pending.last_status = external.status
                return [Event(EventKind.STATUS, normalized)]

            self._finish(external.key, normalized)
            return [Event(EventKind.STATUS, normalized),
                    Event(EventKind.RESULT, normalized)]

    def tick(self, now: float) -> List[Event]:
        """Complete empty tasks and expire overdue tasks."""
        events = []
        with self._lock:
            for key, pending in list(self._pending.items()):
                if (pending.empty_due is not None and
                        pending.empty_due <= now and
                        pending.empty_due <= pending.deadline):
                    result = self._status(
                        pending.request, SUCCEEDED,
                        'empty task wait completed')
                elif pending.deadline <= now:
                    result = self._status(
                        pending.request, FAILED,
                        'task timed out after {:.3f}s'.format(
                            self._task_timeout_s))
                else:
                    continue

                self._finish(key, result)
                events.extend((Event(EventKind.STATUS, result),
                               Event(EventKind.RESULT, result)))
        return events

    @property
    def pending_count(self) -> int:
        """Return the number of active tasks."""
        with self._lock:
            return len(self._pending)

    def _finish(self, key: Tuple[str, int], result: TaskResult) -> None:
        self._pending.pop(key, None)
        self._completed[key] = result
        self._completed.move_to_end(key)
        while len(self._completed) > self._completed_cache_size:
            self._completed.popitem(last=False)

    @staticmethod
    def _status(request: TaskRequest, status: int, message: str) -> TaskResult:
        return TaskResult(
            execution_id=request.execution_id,
            route_name=request.route_name,
            point_index=request.point_index,
            point_id=request.point_id,
            task_id=request.task_id,
            status=status,
            message=message,
        )
