# -*- coding: utf-8 -*-
"""探索状态机的单元测试：用假地图 + 假时钟，把每条失败分支都跑一遍。"""
import unittest

from fishbot_explore import explorer as ex
from fishbot_explore.frontier import Grid


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
        e.step(self.grid, (0.3, 1.5), now=6.5)     # 过了
        self.assertEqual(e.state, ex.SELECT)

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
