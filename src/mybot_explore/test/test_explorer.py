# -*- coding: utf-8 -*-
"""探索状态机的单元测试：用假地图 + 假时钟，把每条失败分支都跑一遍。"""
import unittest

from mybot_explore import explorer as ex
from mybot_explore.frontier import Grid


def make_room(width=60, height=60, res=0.05):
    """三段互不相连的空闲口袋（gx 4..19），每段东边都是未知。

    分成三段是为了让前沿聚成 3 个独立的簇——拉黑一个还能选下一个，
    这样才测得出"连续失败"的计数逻辑。
    """
    data = [-1] * (width * height)
    for lo, hi in ((4, 14), (22, 32), (40, 50)):
        for gy in range(lo, hi):
            for gx in range(4, 20):
                data[gy * width + gx] = 0
    return Grid(width, height, res, 0.0, 0.0, data)


class BaseCase(unittest.TestCase):
    def setUp(self):
        self.grid = make_room()
        self.params = {
            'min_goal_distance_m': 0.3,
            'visited_radius_m': 0.4,   # 小一点，免得一开场就把前沿标掉
            'settle_s': 1.0,
        }

    def make(self, **extra):
        p = dict(self.params)
        p.update(extra)
        return ex.Explorer(p)


class StartTest(BaseCase):
    def test_start_enters_select(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        self.assertEqual(e.state, ex.SELECT)
        self.assertFalse(e.finished())


class HappyPathTest(BaseCase):
    def test_first_step_issues_frontier_goal(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        cmd = e.step(self.grid, (0.3, 1.5), now=1.0)
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['action'], 'navigate')
        self.assertEqual(cmd['kind'], 'frontier')
        self.assertEqual(e.state, ex.NAVIGATE)
        self.assertGreater(cmd['x'], 0.5)   # 前沿在走廊右侧

    def test_success_goes_to_settle_then_select(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        e.on_navigation_result(True, now=5.0)
        self.assertEqual(e.state, ex.SETTLE)
        self.assertEqual(e.stats['succeeded'], 1)
        e.step(self.grid, (0.3, 1.5), now=5.5)     # 还没到 settle_s
        self.assertEqual(e.state, ex.SETTLE)
        cmd = e.step(self.grid, (0.3, 1.5), now=6.5)   # 过了：先去转一圈
        self.assertEqual(e.state, ex.LOOK_AROUND)
        self.assertEqual(cmd['action'], 'spin')
        self.assertEqual(cmd['rounds'], 2)             # 2π 拆成两段 π
        self.assertAlmostEqual(cmd['delta'], 3.1415, places=3)
        e.on_spin_result(True, now=12.0)
        self.assertEqual(e.state, ex.SELECT)
        self.assertEqual(e.stats['look_arounds'], 1)

    def test_visited_grows_as_robot_moves(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        first = e.visited.count()
        e.step(self.grid, (0.3, 2.5), now=2.0)
        self.assertGreater(e.visited.count(), first)
        self.assertGreater(e.stats['distance_m'], 0.9)


class FailurePathTest(BaseCase):
    def test_failure_blacklists_and_reselects(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        cmd = e.step(self.grid, (0.3, 1.5), now=1.0)
        target = (cmd['x'], cmd['y'])
        e.on_navigation_result(False, now=3.0, reason='aborted')
        self.assertEqual(e.state, ex.SELECT)
        self.assertEqual(len(e.blacklist), 1)
        self.assertEqual(e.consecutive_failures, 1)
        self.assertEqual(e.stats['failed'], 1)

        # 再选一次，不能选到刚拉黑的那个点附近
        cmd2 = e.step(self.grid, (0.3, 1.5), now=4.0)
        if cmd2 is not None and cmd2['kind'] == 'frontier':
            moved = ((cmd2['x'] - target[0]) ** 2 + (cmd2['y'] - target[1]) ** 2) ** 0.5
            self.assertGreater(moved, e.params['blacklist_radius_m'])

    def test_no_server_is_not_a_failure(self):
        """没有 nav2 时不能记失败、不能拉黑，否则几拍就自己收工了。"""
        e = self.make(max_consecutive_failures=3)
        e.start((0.3, 1.5), now=0.0)
        cmd = e.step(self.grid, (0.3, 1.5), now=1.0)
        target = (cmd['x'], cmd['y'])

        for i in range(6):
            e.on_navigation_result(False, now=2.0 + i, reason='no_server')
            cmd = e.step(self.grid, (0.3, 1.5), now=2.5 + i)
            self.assertNotEqual(e.state, ex.DONE)

        self.assertEqual(e.blacklist, [])
        self.assertEqual(e.consecutive_failures, 0)
        self.assertEqual(e.stats['failed'], 0)
        self.assertIsNotNone(cmd)
        if cmd['kind'] == 'frontier':
            self.assertAlmostEqual(cmd['x'], target[0], places=6)
            self.assertAlmostEqual(cmd['y'], target[1], places=6)

    def test_three_failures_stop_everything(self):
        e = self.make(max_consecutive_failures=3)
        e.start((0.3, 1.5), now=0.0)
        now = 1.0
        for _ in range(3):
            e.step(self.grid, (0.3, 1.5), now=now)
            e.on_navigation_result(False, now=now + 1.0)
            now += 3.0
        e.step(self.grid, (0.3, 1.5), now=now)
        self.assertEqual(e.state, ex.DONE)
        self.assertEqual(e.done_reason, 'too_many_failures')

    def test_nav_timeout_cancels_and_blacklists(self):
        e = self.make(nav_timeout_s=30.0)
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        cmd = e.step(self.grid, (0.3, 1.5), now=40.0)
        self.assertEqual(cmd['action'], 'cancel')
        self.assertEqual(cmd['reason'], 'timeout')
        self.assertEqual(len(e.blacklist), 1)
        self.assertEqual(e.stats['timeouts'], 1)

    def test_stuck_detected_when_pose_never_moves(self):
        e = self.make(stuck_window_s=10.0)
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        self.assertIsNone(e.step(self.grid, (0.3, 1.5), now=5.0))
        cmd = e.step(self.grid, (0.3, 1.5), now=13.0)
        self.assertEqual(cmd['action'], 'cancel')
        self.assertEqual(cmd['reason'], 'stuck')
        self.assertEqual(e.stats['stuck'], 1)

    def test_progress_resets_stuck_timer(self):
        e = self.make(stuck_window_s=10.0)
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        e.step(self.grid, (1.5, 1.5), now=8.0)     # 动了
        self.assertIsNone(e.step(self.grid, (1.5, 1.5), now=15.0))
        cmd = e.step(self.grid, (1.5, 1.5), now=19.0)
        self.assertEqual(cmd['action'], 'cancel')


class FinishTest(BaseCase):
    def test_no_candidates_returns_home(self):
        e = self.make(return_home=True)
        e.start((0.3, 1.5), now=0.0)
        e.ensure_visited(self.grid)
        # 把整条走廊标成走过
        for gy in range(self.grid.height):
            for gx in range(self.grid.width):
                e.visited.mark(gx, gy)
        cmd = e.step(self.grid, (0.3, 1.5), now=1.0)
        self.assertEqual(cmd['kind'], 'home')
        self.assertEqual(e.state, ex.RETURN_HOME)
        e.on_navigation_result(True, now=9.0)
        self.assertEqual(e.state, ex.DONE)
        self.assertEqual(e.done_reason, 'returned_home')

    def test_no_candidates_without_home_finishes(self):
        e = self.make(return_home=False)
        e.start((0.3, 1.5), now=0.0)
        e.ensure_visited(self.grid)
        for gy in range(self.grid.height):
            for gx in range(self.grid.width):
                e.visited.mark(gx, gy)
        cmd = e.step(self.grid, (0.3, 1.5), now=1.0)
        self.assertIsNone(cmd)
        self.assertEqual(e.state, ex.DONE)
        self.assertEqual(e.done_reason, 'no_frontiers')

    def test_max_duration_stops(self):
        e = self.make(max_duration_s=100.0)
        e.start((0.3, 1.5), now=0.0)
        e.state = ex.SELECT
        cmd = e.step(self.grid, (0.3, 1.5), now=200.0)
        self.assertIsNone(cmd)
        self.assertEqual(e.done_reason, 'timeout')

    def test_done_is_terminal(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        e._finish('no_frontiers')
        self.assertIsNone(e.step(self.grid, (0.3, 1.5), now=1.0))


class LookAroundTest(BaseCase):
    def test_look_around_can_be_disabled(self):
        e = self.make(look_around=False)
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        e.on_navigation_result(True, now=5.0)
        e.step(self.grid, (0.3, 1.5), now=6.5)
        self.assertEqual(e.state, ex.SELECT)

    def test_spin_timeout_returns_to_select(self):
        e = self.make(look_around_timeout_s=5.0)
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        e.on_navigation_result(True, now=5.0)
        e.step(self.grid, (0.3, 1.5), now=6.5)          # 进 look_around
        self.assertEqual(e.state, ex.LOOK_AROUND)
        e.step(self.grid, (0.3, 1.5), now=20.0)         # 超时
        self.assertEqual(e.state, ex.SELECT)
        self.assertEqual(e.stats.get('look_around_timeouts'), 1)

    def test_spin_failure_still_returns_to_select(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        e.on_navigation_result(True, now=5.0)
        e.step(self.grid, (0.3, 1.5), now=6.5)
        e.on_spin_result(False, now=8.0, reason='no_spin_server')
        self.assertEqual(e.state, ex.SELECT)
        self.assertEqual(e.stats.get('look_around_failures'), 1)


class StuckPolicyTest(BaseCase):
    """A 批的核心：转向算进展、stuck 不累加失败、快到点不判卡住。"""

    def _navigate(self, e, now=1.0, pose=(0.3, 1.5)):
        e.start(pose, now=0.0)
        cmd = e.step(self.grid, pose, now=now)
        return cmd

    def test_pure_rotation_counts_as_progress(self):
        e = self.make(stuck_window_s=3.0, stuck_move_rad=0.15)
        self._navigate(e)
        # 位置不动、只转一点：不应判卡住
        for i in range(1, 8):
            e.step(self.grid, (0.3, 1.5, 0.10 * i), now=1.0 + i)
        self.assertEqual(e.state, ex.NAVIGATE)
        self.assertEqual(e.stats['stuck'], 0)

    def test_no_move_no_rotate_triggers_stuck(self):
        e = self.make(stuck_window_s=3.0)
        self._navigate(e)
        cmds = []
        for i in range(1, 6):
            cmds.append(e.step(self.grid, (0.3, 1.5, 0.0), now=1.0 + i))
        self.assertEqual(e.stats['stuck'], 1)
        self.assertTrue(any(c and c.get('action') == 'cancel' for c in cmds))

    def test_stuck_does_not_add_consecutive_failure(self):
        e = self.make(stuck_window_s=3.0)
        self._navigate(e)
        e._abandon('stuck')
        self.assertEqual(e.consecutive_failures, 0)

    def test_stuck_never_finishes_while_frontiers_remain(self):
        """真机教训：连 stuck 5 次就收工，会把还有 20 个前沿的探索掐死。"""
        e = self.make(max_stuck_streak=2)
        e.start((0.3, 1.5), now=0.0)
        for i in range(8):
            e.state = ex.NAVIGATE
            e.nav_kind = 'frontier'
            e.nav_target = (1.0 + i * 0.1, 1.0)
            e._abandon('stuck')
            self.assertFalse(e.finished(), 'stuck 第 %d 次就收工了' % (i + 1))
        self.assertEqual(e.stats['stuck'], 8)

    def test_blacklist_cap_finishes(self):
        """真正的收工判据：试过的点太多（黑名单到上限）。"""
        e = self.make(max_blacklist=3)
        e.start((0.3, 1.5), now=0.0)
        for i in range(3):
            e.state = ex.NAVIGATE
            e.nav_kind = 'frontier'
            e.nav_target = (1.0 + i * 0.5, 1.0)
            e._abandon('aborted')
        e.state = ex.SELECT
        self.assertIsNone(e.step(self.grid, (0.3, 1.5), now=2.0))
        self.assertTrue(e.finished())
        self.assertEqual(e.done_reason, 'too_many_blacklisted')

    def test_near_goal_skips_stuck_check(self):
        e = self.make(stuck_window_s=1.0, final_approach_m=0.5)
        e.start((0.3, 1.5), now=0.0)
        e.state = ex.NAVIGATE
        e.nav_kind = 'frontier'
        e.nav_target = (0.5, 1.5)          # 距离 0.2 m < 0.5
        e.nav_started = 0.0
        e.last_progress = 0.0
        for i in range(1, 6):
            e.step(self.grid, (0.3, 1.5, 0.0), now=float(i))
        self.assertEqual(e.stats['stuck'], 0)


class SnapshotTest(BaseCase):
    def test_snapshot_has_keys(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        e.step(self.grid, (0.3, 1.5), now=1.0)
        s = e.snapshot(now=2.0)
        for key in ('state', 'done_reason', 'blacklist', 'goals', 'succeeded',
                    'failed', 'timeouts', 'stuck', 'distance_m',
                    'visited_cells', 'elapsed_s'):
            self.assertIn(key, s)
        self.assertEqual(s['goals'], 1)

    def test_navigation_result_ignored_when_not_navigating(self):
        e = self.make()
        e.start((0.3, 1.5), now=0.0)
        e.on_navigation_result(True, now=1.0)      # 还在 SELECT
        self.assertEqual(e.stats['succeeded'], 0)


if __name__ == '__main__':
    unittest.main()
