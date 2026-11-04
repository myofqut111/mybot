"""Atomic JSON persistence for patrol routes."""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Any, Dict, Optional


SCHEMA_VERSION = 1


def _initial_state() -> Dict[str, Any]:
    return {'version': SCHEMA_VERSION, 'active_route': '', 'routes': {}}


def _require_finite(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('%s must be finite' % field)
    return result


def _validate_pose(pose: Dict[str, Any]) -> None:
    header = pose.get('header', {})
    if not isinstance(header.get('frame_id'), str) or not header['frame_id']:
        raise ValueError('pose frame_id must not be empty')
    stamp = header.get('stamp', {})
    int(stamp.get('sec', 0))
    int(stamp.get('nanosec', 0))
    for field in ('x', 'y', 'z'):
        _require_finite(pose.get('position', {}).get(field), 'position.' + field)
    for field in ('x', 'y', 'z', 'w'):
        _require_finite(
            pose.get('orientation', {}).get(field), 'orientation.' + field
        )


class RouteStore:
    """Thread-safe route database committed atomically after each mutation."""

    def __init__(self, route_file: str) -> None:
        self.path = Path(route_file).expanduser().resolve()
        self._lock = threading.RLock()
        self._state = self._load()

    def _load(self) -> Dict[str, Any]:
        if not self.path.exists():
            return _initial_state()
        with self.path.open('r', encoding='utf-8') as stream:
            state = json.load(stream)
        if state.get('version') != SCHEMA_VERSION:
            raise ValueError('unsupported patrol route file version')
        if not isinstance(state.get('routes'), dict):
            raise ValueError('patrol route file has invalid routes data')
        if not isinstance(state.get('active_route'), str):
            raise ValueError('patrol route file has invalid active_route')
        for name, route in state['routes'].items():
            if not isinstance(name, str) or not name:
                raise ValueError('route name must not be empty')
            if route.get('name') != name or not isinstance(route.get('points'), list):
                raise ValueError('route %s is malformed' % name)
            for point in route['points']:
                _validate_pose(point.get('pose', {}))
        active = state['active_route']
        if active and active not in state['routes']:
            raise ValueError('active route does not exist')
        return state

    def _commit(self, state: Dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary_name = ''
        try:
            with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=str(self.path.parent),
                prefix=self.path.name + '.', suffix='.tmp', delete=False
            ) as stream:
                temporary_name = stream.name
                json.dump(
                    state, stream, ensure_ascii=True, indent=2,
                    sort_keys=True, allow_nan=False
                )
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self.path)
            self._state = state
        finally:
            if temporary_name and os.path.exists(temporary_name):
                os.unlink(temporary_name)

    @property
    def active_route(self) -> str:
        with self._lock:
            return str(self._state['active_route'])

    def get_route(self, name: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            route = self._state['routes'].get(name)
            return deepcopy(route) if route is not None else None

    def record_point(
        self, route_name: str, point_id: str, task_id: str,
        wait_seconds: float, pose: Dict[str, Any], expect: str = ''
    ) -> Dict[str, Any]:
        route_name = route_name.strip()
        point_id = point_id.strip()
        if not route_name:
            raise ValueError('route_name must not be empty')
        wait_seconds = _require_finite(wait_seconds, 'wait_seconds')
        if wait_seconds < 0.0:
            raise ValueError('wait_seconds must not be negative')
        # 只认这三个值：写错的话宁可直接报错，也不要把一个拼错的
        # "preasent" 存进路线，等跑起来才发现所有点的准确率都是 unknown
        expect = str(expect or '').strip().lower()
        if expect not in ('', 'present', 'absent'):
            raise ValueError(
                "expect must be '', 'present' or 'absent', got %r" % expect)
        _validate_pose(pose)

        with self._lock:
            state = deepcopy(self._state)
            route = state['routes'].setdefault(route_name, {
                'name': route_name,
                'header': deepcopy(pose['header']),
                'points': [],
            })
            if route['header']['frame_id'] != pose['header']['frame_id']:
                raise ValueError('all points in a route must use the same frame')
            if not point_id:
                used = {item['point_id'] for item in route['points']}
                number = len(route['points']) + 1
                point_id = 'point_%03d' % number
                while point_id in used:
                    number += 1
                    point_id = 'point_%03d' % number
            if any(item['point_id'] == point_id for item in route['points']):
                raise ValueError('point_id already exists in route')
            point = {
                'point_id': point_id,
                'pose': deepcopy(pose),
                'task_id': task_id.strip(),
                'wait_seconds': wait_seconds,
                'expect': expect,
            }
            route['header'] = deepcopy(pose['header'])
            route['points'].append(point)
            self._commit(state)
            return deepcopy(point)

    def delete_point(self, route_name: str, point_id: str) -> Dict[str, Any]:
        """删掉一个点，返回被删掉的那条记录。

        删空的路线一起清掉：留着 0 个点的路线只会在下拉框里制造噪音，
        `set_active` 本来也拒绝空路线，留着没有意义。
        """
        route_name = route_name.strip()
        point_id = point_id.strip()
        if not route_name:
            raise ValueError('route_name must not be empty')
        if not point_id:
            raise ValueError('point_id must not be empty')

        with self._lock:
            route = self._state['routes'].get(route_name)
            if route is None:
                raise ValueError('route does not exist')
            index = None
            for position, item in enumerate(route['points']):
                if item['point_id'] == point_id:
                    index = position
                    break
            if index is None:
                raise ValueError('point does not exist in route')

            state = deepcopy(self._state)
            removed = state['routes'][route_name]['points'].pop(index)
            if not state['routes'][route_name]['points']:
                del state['routes'][route_name]
                if state['active_route'] == route_name:
                    state['active_route'] = ''
            self._commit(state)
            return deepcopy(removed)

    def set_active(self, name: str) -> None:
        name = name.strip()
        with self._lock:
            if name not in self._state['routes']:
                raise ValueError('route does not exist')
            if not self._state['routes'][name]['points']:
                raise ValueError('route has no patrol points')
            state = deepcopy(self._state)
            state['active_route'] = name
            self._commit(state)

    def clear_route(self, name: str) -> bool:
        name = name.strip()
        with self._lock:
            if name not in self._state['routes']:
                return False
            state = deepcopy(self._state)
            del state['routes'][name]
            if state['active_route'] == name:
                state['active_route'] = ''
            self._commit(state)
            return True
