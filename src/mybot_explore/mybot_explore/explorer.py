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
    cmd = {'action': 'spin', 'yaw': .., 'speed': .., 'timeout': ..}

导航结果由外面的节点回灌：
    explorer.on_navigation_result(ok, now, reason='')
    explorer.on_spin_result(ok, now, reason='')

状态流转：
    idle ──start──> select
    select ──有候选──> navigate ──成功──> settle ──> look_around ──> select
                            └──失败/超时/卡死──> 拉黑 ──> select
    select ──无候选──> return_home ──> done
    select ──连续失败/超时──> done(aborted)
"""
from __future__ import print_function

import math

from . import frontier as frontier_mod
from . import visited as visited_mod


def _wrap_angle(a):
    """把角度折到 (-pi, pi]，转向差值才不会因为 ±2π 跳变。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a

IDLE = 'idle'
SELECT = 'select'
NAVIGATE = 'navigate'
SETTLE = 'settle'
LOOK_AROUND = 'look_around'
RETURN_HOME = 'return_home'
SETTLE_HOME = 'settle_home'
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
    'blacklist_radius_m': 1.5,          # 到点后整片都别再去了
    'min_goal_distance_m': 0.8,
    'min_separation_m': 1.0,
    'goal_clearance_m': 0.35,
    'max_goal_distance_m': 15.0,
    'nav_timeout_s': 90.0,
    'settle_s': 0.3,
    'stuck_window_s': 15.0,
    'stuck_move_m': 0.08,
    'stuck_move_rad': 0.30,
    'retarget_cooldown_s': 3.0,
    'final_approach_m': 0.5,
    'near_goal_timeout_s': 15.0,
    # 可达性：车半径 + 余量。激光能从凳子腿中间穿过去，车过不去，
    # 所以前沿点必须落在"从车这里连得过去"的格子上。
    'traversable_clearance_m': 0.30,
    'reach_search_m': 0.6,
    # 封缝：把腿与腿之间的细缝算成障碍（2D 激光看不见椅子面）
    'seal_gaps': True,
    'seal_radius_m': 0.25,
    'look_around': True,
    'look_around_yaw': 6.283,
    'look_around_speed': 2.0,
    'look_around_timeout_s': 30.0,
    'progress_epsilon_m': 0.15,
    'max_consecutive_failures': 10,
    'max_stuck_streak': 5,
    'max_blacklist': 60,
    # 收工判定：没有候选时先等这么久。期间未知格子还在变少，说明只是
    # 暂时没有前沿（车刚走过去、地图还没更新），不该急着回家。
    'no_frontier_grace_s': 30.0,
    'unknown_drop_cells': 20,
    'max_duration_s': 1200.0,
    'return_home': True,
    'home_settle_s': 3.0,
    'home_timeout_s': 120.0,
}


class Explorer(object):
    def __init__(self, params=None):
        self.params = dict(DEFAULTS)
        if params:
            self.params.update(params)

        self.state = IDLE
        self.done_reason = ''
        self.stop_reason = ''
        self.returned_home = False
        self.loop_closure = ''
        self.home_arrived_at = None
        self.settle_until = None
        self.start_pose = None
        self.pose = None

        self.visited = None          # visited.VisitedLayer，第一次拿到 grid 时建
        self.blacklist = []           # [(x, y), ...]

        self.nav_target = None
        self.nav_kind = ''
        self.nav_started = 0.0
        self.last_progress = 0.0
        self.settle_at = 0.0
        self.look_around_at = 0.0
        self.select_not_before = 0.0
        self._no_frontier_since = None
        self._unknown_at_grace = None
        self._near_since = None
        self._now = 0.0
        self.last_cancel_reason = ''

        # 卡住判定用的参考点：位置 + 朝向。转向本来就是"在动"，
        # 只看位移会把原地对准误判成卡死（真机日志里 10.0 s 必 stuck 就是这个）。
        self._ref_x = None
        self._ref_y = None
        self._ref_yaw = None

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
        self.stop_reason = ''
        self.returned_home = False
        self.loop_closure = ''
        self.home_arrived_at = None
        self.settle_until = None

    def finished(self):
        return self.state == DONE

    # ------------------------------------------------------------------
    # 一拍
    # ------------------------------------------------------------------

    def step(self, grid, pose, now):
        """推进状态机；需要动作时返回命令字典，否则返回 None。"""
        if self.state == IDLE or self.state == DONE:
            return None

        self._now = now
        self._observe(pose, now, grid)

        if self.state == SELECT:
            return self._select(grid, now)
        if self.state == NAVIGATE:
            return self._watch_progress(now)
        if self.state == SETTLE:
            if now - self.settle_at >= self.params['settle_s']:
                if self.params.get('look_around', True):
                    self.state = LOOK_AROUND
                    self.look_around_at = now
                    # nav2 的 Spin 是"转到绝对朝向"，不是"转多少度"：
                    # 直接给 current+2π 会被归一化成 0（原地不动）。
                    # 所以拆成若干次 ≤180° 的绝对转向。
                    total = abs(float(self.params['look_around_yaw']))
                    rounds = max(1, int(math.ceil(total / math.pi - 1e-6)))
                    return {
                        'action': 'spin',
                        'yaw': total,
                        'delta': total / rounds,
                        'rounds': rounds,
                        'speed': self.params['look_around_speed'],
                        'timeout': self.params['look_around_timeout_s'],
                        'kind': 'look_around',
                    }
                self.state = SELECT
            return None
        if self.state == LOOK_AROUND:
            # 转圈有硬超时：Spin 服务不应答或转不动时不能卡死在这里
            if now - self.look_around_at > self.params['look_around_timeout_s']:
                self.stats['look_around_timeouts'] = \
                    self.stats.get('look_around_timeouts', 0) + 1
                self.state = SELECT
            return None
        if self.state == RETURN_HOME:
            return self._watch_progress(now)
        if self.state == SETTLE_HOME:
            # 到家后再等 home_settle_s：slam 需要这几秒把回环后的位姿写进图
            if self.settle_until is not None and now >= self.settle_until:
                return self._finish(self.stop_reason or 'returned_home')
            return None
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
        yaw = float(pose[2]) if len(pose) > 2 and pose[2] is not None else None
        self.ensure_visited(grid)

        if self.pose is not None:
            moved = math.hypot(x - self.pose[0], y - self.pose[1])
            if moved >= self.params['progress_epsilon_m']:
                self.stats['distance_m'] += moved
        self.pose = (x, y)

        # 位移或转角任一够大就说明"车在动"，重置卡住计时
        if self._ref_x is None:
            self._ref_x, self._ref_y, self._ref_yaw = x, y, yaw
            self.last_progress = now
        else:
            moved_ref = math.hypot(x - self._ref_x, y - self._ref_y)
            if yaw is not None and self._ref_yaw is None:
                # 第一次拿到朝向：只补上参考值，不算"转过"
                self._ref_yaw = yaw
            turned = 0.0
            if yaw is not None and self._ref_yaw is not None:
                turned = abs(_wrap_angle(yaw - self._ref_yaw))
            if moved_ref >= self.params['stuck_move_m'] or \
                    turned >= self.params['stuck_move_rad']:
                self._ref_x, self._ref_y, self._ref_yaw = x, y, yaw
                self.last_progress = now

        if self.visited is not None:
            self.visited.mark_circle(x, y, self.params['visited_radius_m'], grid)

    # ------------------------------------------------------------------
    # 选点
    # ------------------------------------------------------------------

    def _select(self, grid, now):
        if now - self.started_at > self.params['max_duration_s']:
            return self._begin_return_home('timeout')
        if self.consecutive_failures >= self.params['max_consecutive_failures']:
            self.too_many_failures = True
            return self._begin_return_home('too_many_failures')
        # 黑名单上限才是真正的"到处都试过了"的判据：每个失败的点都会被拉黑，
        # 剩下的前沿点越来越少，最后自然走到"没有前沿"。绝不因为"失败几次"
        # 就把还有前沿的探索掐掉——真机上一片区域连着不可达是很常见的。
        if len(self.blacklist) >= self.params['max_blacklist']:
            return self._begin_return_home('too_many_blacklisted')
        # 刚放弃一个目标就立刻派下一个，车会原地再转一次——看起来就是"打转"。
        # 留几秒冷却，让控制器先把上一次的取消收干净。
        if now < self.select_not_before:
            return None
        if grid is None:
            return None

        grid_for_plan = grid
        if self.params.get('seal_gaps') and self.params.get('seal_radius_m', 0) > 0:
            grid_for_plan = frontier_mod.seal_gaps(
                grid, self.params['seal_radius_m'],
                self.params['occupied_thresh'])

        candidates = frontier_mod.find_frontiers(
            grid_for_plan,
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
            goal_clearance_m=self.params['goal_clearance_m'],
            reach_clearance_m=self.params['traversable_clearance_m'],
            reach_search_m=self.params['reach_search_m'],
        )
        if not candidates:
            # 没有候选不代表建完了：车刚走过一片区域时，地图要一两秒才更新，
            # 前沿会"消失一下又冒出来"。所以观察一个宽限期，期间未知格子
            # 还在变少就重新计时；真停了才收工。
            unknown_now = frontier_mod.count_unknown(grid)
            grace = self.params['no_frontier_grace_s']
            if self._no_frontier_since is None:
                self._no_frontier_since = now
                self._unknown_at_grace = unknown_now
                return None
            drop = self._unknown_at_grace - unknown_now
            if drop >= self.params['unknown_drop_cells']:
                self._no_frontier_since = now
                self._unknown_at_grace = unknown_now
                return None
            if now - self._no_frontier_since < grace:
                return None
            return self._begin_return_home('map_complete')

        self._no_frontier_since = None
        self._unknown_at_grace = None
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
        limit = self.params['nav_timeout_s']
        if self.nav_kind == 'home':
            limit = self.params.get('home_timeout_s', limit)
        if now - self.nav_started > limit:
            self._abandon('timeout')
            return {'action': 'cancel', 'reason': 'timeout'}

        # 快到目标时不做卡住判定：最后 0.5 m 往往要反复对准，
        # 位移本来就接近零，这时候判卡死会把"快到了"当成"卡住了"。
        # 但**不能无限豁免**：真卡在目标门口时，以前会一直等到 90 s 超时。
        if self._near_goal(now):
            self.last_progress = now
            return None

        idle_for = now - self.last_progress
        if idle_for > self.params['stuck_window_s']:
            self._abandon('stuck')
            return {'action': 'cancel', 'reason': 'stuck'}
        return None

    def _near_goal(self, now=None):
        """离目标很近时给卡住判定一个"豁免窗口"，但最多豁免这么久。

        参数 now 为 None 时只回答"近不近"（老调用方式）。
        """
        if self.nav_target is None or self.pose is None:
            return False
        reach = self.params.get('final_approach_m', 0.0)
        if reach <= 0.0:
            return False
        near = math.hypot(self.nav_target[0] - self.pose[0],
                          self.nav_target[1] - self.pose[1]) <= reach
        if not near:
            self._near_since = None
            return False
        if now is None:
            return True
        if self._near_since is None:
            self._near_since = now
        limit = float(self.params.get('near_goal_timeout_s', 0.0) or 0.0)
        if limit <= 0.0:
            return True
        # 豁免到期后不再"续命"，让 stuck 判定正常触发，最多再等一个 stuck_window
        return (now - self._near_since) < limit

    def _abandon(self, reason):
        """放弃当前目标：拉黑 + 回选点。

        注意 consecutive_failures 只统计"真的走不过去"（abort / 超时）。
        自己判定 stuck 主动取消属于策略行为，不算目标失败——否则真机上
        "转向时被判卡住 → 三次 → 收工"就会把整轮探索提前掐死。
        """
        self.last_cancel_reason = reason
        if self.nav_kind == 'home':
            self.state = DONE
            self.returned_home = False
            self.loop_closure = 'failed'
            self.done_reason = 'return_home_failed'
            return
        if self.nav_target is not None:
            self.blacklist.append(self.nav_target)
            self.stats['blacklisted'] = len(self.blacklist)
        if reason == 'timeout':
            self.stats['timeouts'] += 1
            self.consecutive_failures += 1
            self.stats['stuck_streak'] = 0
        elif reason == 'stuck':
            self.stats['stuck'] += 1
            # stuck 只统计，**不再收工**：真机上一片区域连着不可达很常见，
            # 以前"连续 5 次 stuck 就结束"会把还有 20 个前沿的探索直接掐死。
            # 每个 stuck 的点照样拉黑，选点会自然绕开它；真正的收工判据是
            # "没有前沿了"或黑名单到上限（max_blacklist）。
            self.stats['stuck_streak'] = self.stats.get('stuck_streak', 0) + 1
        else:
            self.consecutive_failures += 1
            self.stats['stuck_streak'] = 0
        self.nav_target = None
        self.nav_kind = ''
        self.state = SELECT
        self.select_not_before = self._now + float(
            self.params['retarget_cooldown_s'])

    # ------------------------------------------------------------------
    # 导航结果回灌
    # ------------------------------------------------------------------

    def on_navigation_result(self, ok, now, reason=''):
        self._now = now
        if self.state not in (NAVIGATE, RETURN_HOME):
            return

        if reason == 'no_server':
            # 导航服务还没起来，属于"环境没就绪"，不是这个点位的错：
            # 不计失败、不拉黑，回到选点，下一拍再试同一个。
            self.nav_target = None
            self.nav_kind = ''
            self.state = SELECT
            self.nav_started = now
            return

        if self.nav_kind == 'home':
            self.nav_target = None
            self.nav_kind = ''
            if ok:
                self.returned_home = True
                self.loop_closure = 'settled'
                self.home_arrived_at = now
                self.settle_until = now + float(self.params['home_settle_s'])
                self.state = SETTLE_HOME
            else:
                self.returned_home = False
                self.loop_closure = 'failed'
                self._finish(self.stop_reason or 'return_home_failed')
            return

        if ok:
            self.stats['succeeded'] += 1
            self.consecutive_failures = 0
            self.stats['stuck_streak'] = 0
            # 到过的点也拉黑：前沿点到了以后往往还剩一点未知（只扫掉一部分），
            # 不拉黑的话下一轮又会选中它，车挪 20 cm 再转一圈——看着就是原地转。
            if self.nav_target is not None:
                self.blacklist.append(self.nav_target)
                self.stats['blacklisted'] = len(self.blacklist)
            self.nav_target = None
            self._near_since = None
            self.nav_kind = ''
            self.state = SETTLE
            self.settle_at = now
            return

        self.stats['failed'] += 1
        self._abandon(reason or 'aborted')

    def on_spin_result(self, ok, now, reason=''):
        """到点转圈结束（成功、失败、超时都算结束），回选点继续。"""
        if self.state != LOOK_AROUND:
            return
        if ok:
            self.stats['look_arounds'] = self.stats.get('look_arounds', 0) + 1
        else:
            self.stats['look_around_failures'] = \
                self.stats.get('look_around_failures', 0) + 1
        self.look_around_at = now
        self.state = SELECT

    # ------------------------------------------------------------------
    # 收工
    # ------------------------------------------------------------------

    def _begin_return_home(self, stop_reason):
        """自然收工：先回起点（为了回环），再等几秒才真正结束。"""
        self.stop_reason = stop_reason
        if self.params.get('return_home') and self.start_pose is not None:
            self.state = RETURN_HOME
            self.nav_kind = 'home'
            self.nav_target = (self.start_pose[0], self.start_pose[1])
            self.nav_started = self._now
            self.last_progress = self._now
            self.loop_closure = 'pending'
            self.stats['goals'] = self.stats.get('goals', 0) + 1
            return {'action': 'navigate', 'x': self.start_pose[0],
                    'y': self.start_pose[1], 'kind': 'home'}
        self.loop_closure = 'skipped'
        return self._finish(stop_reason)

    def _finish(self, reason):
        self.state = DONE
        self.done_reason = reason
        if not self.stop_reason:
            self.stop_reason = reason
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
            'last_cancel_reason': self.last_cancel_reason,
            'stop_reason': self.stop_reason,
            'returned_home': self.returned_home,
            'loop_closure': self.loop_closure,
        }
        out.update(self.stats)
        if now is not None and self.started_at is not None:
            out['elapsed_s'] = round(now - self.started_at, 1)
        if self.visited is not None:
            out['visited_cells'] = self.visited.count()
        return out
