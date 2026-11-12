"""探索状态机：决定"下一步去哪"，但不碰 ROS。

为什么单独拆出来
----------------
导航会失败、会超时、会卡死，这些分支在真车上很难复现。把它们放在一个
纯逻辑类里，用"假导航函数"就能把每种失败都跑一遍。

对外只有一个驱动口：
    cmd = explorer.step(grid, pose, now)
    cmd 为 None 表示"这一拍什么都不用做"
    cmd = {'action': 'navigate', 'x': .., 'y': .., 'kind': 'frontier'|'home'}
    cmd = {'action': 'cancel'}

导航结果由外面的节点回灌：
    explorer.on_navigation_result(ok, now, reason='')

状态流转：
    idle ──start──> select
    select ──有候选──> navigate ──成功──> settle ──> select
                            └──失败/超时/卡死──> 拉黑 ──> select
    select ──无候选──> return_home ──> done
    select ──连续失败/超时──> done(aborted)
"""
from __future__ import print_function

import math

from . import frontier as frontier_mod
from . import visited as visited_mod

IDLE = 'idle'
SELECT = 'select'
NAVIGATE = 'navigate'
SETTLE = 'settle'
RETURN_HOME = 'return_home'
DONE = 'done'

DEFAULTS = {
    'period_s': 1.0,
    'min_size': 4,
    'occupied_thresh': 50,
    'unknown_radius_m': 1.0,
    'max_candidates': 20,
    'w_distance': 0.6,
    'w_gain': 0.4,
    'visited_radius_m': 1.2,
    'blacklist_radius_m': 0.6,
    'min_goal_distance_m': 0.8,
    'min_separation_m': 1.0,
    'max_goal_distance_m': 15.0,
    'nav_timeout_s': 90.0,
    'settle_s': 1.0,
    'stuck_window_s': 10.0,
    'stuck_move_m': 0.05,
    'progress_epsilon_m': 0.15,
    'max_consecutive_failures': 3,
    'max_duration_s': 1200.0,
    'return_home': True,
}


class Explorer(object):
    def __init__(self, params=None):
        self.params = dict(DEFAULTS)
        if params:
            self.params.update(params)

        self.state = IDLE
        self.done_reason = ''
        self.start_pose = None
        self.pose = None

        self.visited = None          # visited.VisitedLayer，第一次拿到 grid 时建
        self.blacklist = []           # [(x, y), ...]

        self.nav_target = None
        self.nav_kind = ''
        self.nav_started = 0.0
        self.last_progress = 0.0
        self.settle_at = 0.0

        self.consecutive_failures = 0
        self.started_at = None
        self.too_many_failures = False

        self.stats = {
            'goals': 0,
            'succeeded': 0,
            'failed': 0,
            'timeouts': 0,
            'stuck': 0,
            'blacklisted': 0,
            'distance_m': 0.0,
        }

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self, pose, now):
        self.start_pose = (float(pose[0]), float(pose[1]))
        self.pose = self.start_pose
        self.started_at = now
        self.last_progress = now
        self.state = SELECT
        self.done_reason = ''

    def finished(self):
        return self.state == DONE

    # ------------------------------------------------------------------
    # 一拍
    # ------------------------------------------------------------------

    def step(self, grid, pose, now):
        """推进状态机；需要动作时返回命令字典，否则返回 None。"""
        if self.state == IDLE or self.state == DONE:
            return None

        self._observe(pose, now, grid)

        if self.state == SELECT:
            return self._select(grid, now)
        if self.state == NAVIGATE:
            return self._watch_progress(now)
        if self.state == SETTLE:
            if now - self.settle_at >= self.params['settle_s']:
                self.state = SELECT
            return None
        if self.state == RETURN_HOME:
            return self._watch_progress(now)
        return None

    # ------------------------------------------------------------------
    # 观测：位置、覆盖层、里程
    # ------------------------------------------------------------------

    def ensure_visited(self, grid):
        """按地图尺寸建覆盖层（幂等）。测试里可以先手动建好再预标记。"""
        if self.visited is None and grid is not None:
            self.visited = visited_mod.VisitedLayer.from_grid(grid)
        return self.visited

    def _observe(self, pose, now, grid):
        x, y = float(pose[0]), float(pose[1])
        self.ensure_visited(grid)

        if self.pose is not None:
            moved = math.hypot(x - self.pose[0], y - self.pose[1])
            if moved >= self.params['progress_epsilon_m']:
                self.stats['distance_m'] += moved
                self.last_progress = now
        self.pose = (x, y)

        if self.visited is not None:
            self.visited.mark_circle(x, y, self.params['visited_radius_m'], grid)

    # ------------------------------------------------------------------
    # 选点
    # ------------------------------------------------------------------

    def _select(self, grid, now):
        if now - self.started_at > self.params['max_duration_s']:
            return self._finish('timeout')
        if self.consecutive_failures >= self.params['max_consecutive_failures']:
            self.too_many_failures = True
            return self._finish('too_many_failures')
        if grid is None:
            return None

        candidates = frontier_mod.find_frontiers(
            grid,
            robot_xy=self.pose,
            min_size=self.params['min_size'],
            occupied_thresh=self.params['occupied_thresh'],
            unknown_radius_m=self.params['unknown_radius_m'],
            max_candidates=self.params['max_candidates'],
            w_distance=self.params['w_distance'],
            w_gain=self.params['w_gain'],
            visited=self.visited,
            blacklist=self.blacklist,
            blacklist_radius_m=self.params['blacklist_radius_m'],
            min_goal_distance_m=self.params['min_goal_distance_m'],
            max_goal_distance_m=self.params['max_goal_distance_m'],
            min_separation_m=self.params['min_separation_m'],
        )
        if not candidates:
            if self.params['return_home'] and self.start_pose is not None:
                self.state = RETURN_HOME
                self.nav_kind = 'home'
                self.nav_target = self.start_pose
                self.nav_started = now
                self.stats['goals'] += 1
                return {'action': 'navigate', 'x': self.start_pose[0],
                        'y': self.start_pose[1], 'kind': 'home'}
            return self._finish('no_frontiers')

        best = candidates[0]
        self.state = NAVIGATE
        self.nav_kind = 'frontier'
        self.nav_target = (best['x'], best['y'])
        self.nav_started = now
        self.last_progress = now
        self.stats['goals'] += 1
        return {'action': 'navigate', 'x': best['x'], 'y': best['y'],
                'kind': 'frontier', 'score': best['score'],
                'cells': best['cells'], 'unknown': best['unknown']}

    # ------------------------------------------------------------------
    # 导航中的看护：超时、卡死
    # ------------------------------------------------------------------

    def _watch_progress(self, now):
        if now - self.nav_started > self.params['nav_timeout_s']:
            self._abandon('timeout')
            return {'action': 'cancel', 'reason': 'timeout'}

        idle_for = now - self.last_progress
        if idle_for > self.params['stuck_window_s']:
            self._abandon('stuck')
            return {'action': 'cancel', 'reason': 'stuck'}
        return None

    def _abandon(self, reason):
        """放弃当前目标：拉黑、计数、回到选点。"""
        if self.nav_kind == 'home':
            self.state = DONE
            self.done_reason = 'return_home_failed'
            return
        if self.nav_target is not None:
            self.blacklist.append(self.nav_target)
            self.stats['blacklisted'] = len(self.blacklist)
        if reason == 'timeout':
            self.stats['timeouts'] += 1
        elif reason == 'stuck':
            self.stats['stuck'] += 1
        self.consecutive_failures += 1
        self.nav_target = None
        self.nav_kind = ''
        self.state = SELECT

    # ------------------------------------------------------------------
    # 导航结果回灌
    # ------------------------------------------------------------------

    def on_navigation_result(self, ok, now, reason=''):
        if self.state not in (NAVIGATE, RETURN_HOME):
            return

        if self.nav_kind == 'home':
            self.state = DONE
            self.done_reason = 'returned_home' if ok else 'return_home_failed'
            return

        if ok:
            self.stats['succeeded'] += 1
            self.consecutive_failures = 0
            self.nav_target = None
            self.nav_kind = ''
            self.state = SETTLE
            self.settle_at = now
            return

        self.stats['failed'] += 1
        self._abandon(reason or 'aborted')

    # ------------------------------------------------------------------
    # 收工
    # ------------------------------------------------------------------

    def _finish(self, reason):
        self.state = DONE
        self.done_reason = reason
        return None

    # ------------------------------------------------------------------
    # 给页面 / 日志用
    # ------------------------------------------------------------------

    def snapshot(self, now=None):
        out = {
            'state': self.state,
            'done_reason': self.done_reason,
            'blacklist': len(self.blacklist),
            'consecutive_failures': self.consecutive_failures,
        }
        out.update(self.stats)
        if now is not None and self.started_at is not None:
            out['elapsed_s'] = round(now - self.started_at, 1)
        if self.visited is not None:
            out['visited_cells'] = self.visited.count()
        return out
