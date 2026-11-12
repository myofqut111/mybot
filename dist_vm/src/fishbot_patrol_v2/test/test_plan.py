"""plan.py 的单测：不依赖 ROS，直接 ``python3 test/test_plan.py`` 就能跑。"""

import io
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_patrol_v2 import plan  # noqa: E402
from fishbot_patrol_v2.plan import (  # noqa: E402
    FailurePolicy, MODE_REPEAT, PlanConfig, Planner, PointRef)


def points(count=3):
    return [PointRef(point_id='p%d' % index, task_id='t%d' % index,
                     wait_seconds=1.0 + index) for index in range(count)]


class OnceModeTest(unittest.TestCase):
    """ONCE 模式按路线顺序跑一遍就结束。"""

    def test_runs_each_point_once(self):
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE))
        seen = []
        while True:
            job = planner.next_job()
            if job is None:
                break
            seen.append(job.point_id)
            planner.report_success(job)
        self.assertEqual(seen, ['p0', 'p1', 'p2'])
        self.assertTrue(planner.finished)
        self.assertEqual(planner.completed_ids, ['L0N0', 'L0N1', 'L0N2'])

    def test_empty_route_rejected(self):
        with self.assertRaises(ValueError):
            Planner([], PlanConfig())

    def test_bad_mode_rejected(self):
        with self.assertRaises(ValueError):
            PlanConfig(mode=7)


class RepeatAndRandomTest(unittest.TestCase):
    """REPEAT 跑固定轮数，RANDOM 每轮洗牌且可复现。"""

    def test_repeat_runs_all_loops(self):
        planner = Planner(points(2), PlanConfig(mode=plan.MODE_REPEAT, repeat_count=2))
        loops = []
        while True:
            job = planner.next_job()
            if job is None:
                break
            loops.append(job.loop_index)
            planner.report_success(job)
        self.assertEqual(loops, [0, 0, 1, 1])

    def test_repeat_zero_means_infinite(self):
        planner = Planner(points(2), PlanConfig(mode=plan.MODE_REPEAT, repeat_count=0))
        self.assertIsNone(planner.total_loops)
        for _ in range(9):
            job = planner.next_job()
            self.assertIsNotNone(job)
            planner.report_success(job)
        self.assertFalse(planner.finished)

    def test_random_is_reproducible_and_a_permutation(self):
        first = Planner(points(4), PlanConfig(mode=plan.MODE_RANDOM, repeat_count=1, seed=7))
        second = Planner(points(4), PlanConfig(mode=plan.MODE_RANDOM, repeat_count=1, seed=7))
        order_first, order_second = [], []
        for planner, sink in ((first, order_first), (second, order_second)):
            while True:
                job = planner.next_job()
                if job is None:
                    break
                sink.append(job.point_id)
                planner.report_success(job)
        self.assertEqual(order_first, order_second)
        self.assertEqual(sorted(order_first), ['p0', 'p1', 'p2', 'p3'])


class FailurePolicyTest(unittest.TestCase):
    """三种失败策略的行为差异。"""

    def test_skip_continues_after_failure(self):
        planner = Planner(points(3), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.SKIP))
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'nav failed'), plan.OUTCOME_SKIP)
        job = planner.next_job()
        self.assertEqual(job.point_id, 'p1')
        self.assertEqual(len(planner.skipped), 1)
        self.assertFalse(planner.aborted)

    def test_stop_aborts_immediately(self):
        planner = Planner(points(3), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.STOP))
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'nav failed'), plan.OUTCOME_ABORT)
        self.assertTrue(planner.aborted)
        self.assertIsNone(planner.next_job())

    def test_retry_then_success(self):
        planner = Planner(points(2), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.RETRY, max_retries=1))
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'timeout'), plan.OUTCOME_RETRY)
        retry = planner.next_job()
        self.assertEqual(retry.attempt, 2)
        self.assertTrue(retry.is_retry)
        self.assertEqual(planner.report_success(retry), plan.OUTCOME_DONE)
        self.assertEqual(planner.next_job().point_id, 'p1')

    def test_retry_exhausted_falls_back_to_skip(self):
        planner = Planner(points(2), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.RETRY, max_retries=1))
        job = planner.next_job()
        planner.report_failure(job, 'first')
        retry = planner.next_job()
        self.assertEqual(planner.report_failure(retry, 'second'), plan.OUTCOME_SKIP)
        self.assertEqual(len(planner.skipped), 1)

    def test_consecutive_failures_break_the_run(self):
        planner = Planner(points(5), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.SKIP,
            max_consecutive_failures=2))
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'x'), plan.OUTCOME_SKIP)
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'y'), plan.OUTCOME_ABORT)
        self.assertTrue(planner.aborted)

    def test_retries_do_not_consume_the_breaker(self):
        planner = Planner(points(3), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.RETRY, max_retries=1,
            max_consecutive_failures=2))
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'x'), plan.OUTCOME_RETRY)
        retry = planner.next_job()
        self.assertEqual(planner.report_failure(retry, 'x'),
                         plan.OUTCOME_SKIP)   # 只记 1 次连续失败
        self.assertFalse(planner.aborted)

    def test_success_resets_the_breaker(self):
        planner = Planner(points(4), PlanConfig(
            mode=plan.MODE_ONCE, policy=FailurePolicy.SKIP,
            max_consecutive_failures=2))
        job = planner.next_job()
        planner.report_failure(job, 'x')
        job = planner.next_job()
        planner.report_success(job)          # 计数器归零
        job = planner.next_job()
        self.assertEqual(planner.report_failure(job, 'z'), plan.OUTCOME_SKIP)
        self.assertFalse(planner.aborted)

    def test_reporting_a_stale_job_is_rejected(self):
        planner = Planner(points(2), PlanConfig())
        job = planner.next_job()
        planner.report_success(job)
        with self.assertRaises(ValueError):
            planner.report_success(job)


class ResumeTest(unittest.TestCase):
    """断点续跑：跳过已完成的点，或从游标处继续。"""

    def test_already_done_ids_are_skipped(self):
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          already_done=['L0N0'])
        self.assertEqual(planner.next_job().point_id, 'p1')

    def test_progress_cursor_restores_position(self):
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          progress={'loop': 0, 'cursor': 2})
        self.assertEqual(planner.next_job().point_id, 'p2')

    def test_all_done_finishes_immediately(self):
        planner = Planner(points(2), PlanConfig(mode=plan.MODE_ONCE),
                          already_done=['L0N0', 'L0N1'])
        self.assertIsNone(planner.next_job())
        self.assertTrue(planner.finished)

    def test_snapshot_round_trips(self):
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE))
        job = planner.next_job()
        planner.report_success(job)
        self.assertEqual(planner.snapshot(),
                         {'loop': 0, 'cursor': 1, 'order': [0, 1, 2]})
        resumed = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          progress=planner.snapshot())
        self.assertEqual(resumed.next_job().point_id, 'p1')

    def test_random_order_survives_restart(self):
        """RANDOM 模式续跑必须沿用存档里的顺序，不能重新洗牌。"""
        config = PlanConfig(mode=plan.MODE_RANDOM, repeat_count=2, seed=7)
        planner = Planner(points(4), config)
        first = planner.next_job()
        planner.report_success(first)
        expected = planner.next_job()
        self.assertIsNotNone(expected)

        resumed = Planner(points(4), config, progress=planner.snapshot())
        self.assertEqual(resumed.next_job().job_id, expected.job_id)

    def test_invalid_order_falls_back(self):
        """存档里的顺序坏掉时退回按路线顺序，不能崩。"""
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          progress={'loop': 0, 'cursor': 1, 'order': [0, 0]})
        self.assertEqual(planner.next_job().point_id, 'p1')

    def test_is_permutation(self):
        self.assertTrue(plan.is_permutation([2, 0, 1], 3))
        self.assertFalse(plan.is_permutation([0, 0, 1], 3))
        self.assertFalse(plan.is_permutation([0, 1], 3))
        self.assertFalse(plan.is_permutation([0, 1, 3], 3))


class HardeningTest(unittest.TestCase):
    """审查发现的问题：坏快照、类型穿参、轮数上报。"""

    def test_policy_must_be_enum(self):
        """传字符串会静默退化成 skip（身份比较），必须当场报错。"""
        with self.assertRaises(ValueError):
            PlanConfig(policy='stop')

    def test_negative_cursor_is_clamped(self):
        """损坏快照里的负游标不能让 _order[-1] 静默指到别的点。"""
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          progress={'loop': 0, 'cursor': -2,
                                    'order': [0, 1, 2]})
        self.assertEqual(planner.next_job().point_id, 'p0')

    def test_cursor_beyond_order_is_clamped(self):
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          progress={'loop': 0, 'cursor': 99,
                                    'order': [0, 1, 2]})
        self.assertIsNone(planner.next_job())

    def test_loop_beyond_total_finishes_instead_of_rerunning(self):
        """快照轮次超过本次请求轮数时，不能把整条路线再跑一遍。"""
        planner = Planner(points(2),
                          PlanConfig(mode=plan.MODE_REPEAT, repeat_count=2),
                          progress={'loop': 5, 'cursor': 0, 'order': [0, 1]})
        self.assertIsNone(planner.next_job())

    def test_completed_ids_include_resumed_jobs(self):
        """续跑后完成数不能从 0 重新数。"""
        planner = Planner(points(3), PlanConfig(mode=plan.MODE_ONCE),
                          already_done=['L0N0'],
                          progress={'loop': 0, 'cursor': 1,
                                    'order': [0, 1, 2]})
        self.assertEqual(planner.completed_ids, ['L0N0'])
        planner.report_success(planner.next_job())
        self.assertEqual(planner.completed_ids, ['L0N0', 'L0N1'])

    def test_completed_loops_counts_whole_loops(self):
        """整轮跑完后 completed_loops 必须是 1，不能因为 loop 停在 0 就报 0。"""
        planner = Planner(points(2),
                          PlanConfig(mode=plan.MODE_REPEAT, repeat_count=2))
        planner.report_success(planner.next_job())
        planner.report_success(planner.next_job())
        self.assertEqual(planner.completed_loops, 1)
        self.assertEqual(planner.loop_index, 0)
        planner.report_success(planner.next_job())
        planner.report_success(planner.next_job())
        self.assertEqual(planner.completed_loops, 2)


class TotalJobsTest(unittest.TestCase):
    """0.4.6 的真 bug：node.py 用 planner.total，属性不存在，建报告就崩。"""

    def points(self, n):
        return [PointRef(point_id='p%d' % i) for i in range(n)]

    def test_single_loop(self):
        planner = Planner(self.points(3), PlanConfig())
        self.assertEqual(planner.total_jobs, 3)

    def test_repeat_multiplies(self):
        planner = Planner(self.points(3),
                          PlanConfig(mode=MODE_REPEAT, repeat_count=5))
        self.assertEqual(planner.total_jobs, 15)

    def test_resume_subtracts_done(self):
        planner = Planner(self.points(3),
                          PlanConfig(mode=MODE_REPEAT, repeat_count=5),
                          already_done=['L0N0', 'L0N1'])
        self.assertEqual(planner.total_jobs, 13)

    def test_infinite_is_none(self):
        planner = Planner(self.points(2),
                          PlanConfig(mode=MODE_REPEAT, repeat_count=0))
        self.assertIsNone(planner.total_jobs)


class NodeUsesRealPlannerAttrsTest(unittest.TestCase):
    """静态钉住：node.py 引用的 planner 属性必须真的存在。

    0.4.6 的事故就是 node.py 写了 ``planner.total``（不存在），
    机器人在"路线已加载、车还没动"的位置直接 AttributeError。
    """

    def setUp(self):
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        self.node_src = io.open(
            os.path.join(os.path.dirname(here), 'fishbot_patrol_v2', 'node.py'),
            encoding='utf-8').read()

    def test_no_bare_planner_total(self):
        self.assertIsNone(re.search(r'planner\.total(?!_|[a-z])', self.node_src),
                          'node.py 又写了不存在的 planner.total')

    def test_uses_total_jobs(self):
        self.assertIn('planner.total_jobs', self.node_src)


if __name__ == '__main__':
    unittest.main(verbosity=2)
