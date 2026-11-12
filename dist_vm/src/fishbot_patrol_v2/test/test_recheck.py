#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""recheck 纯逻辑单测：解析视觉结果 + 灰区决策表。

直接跑：``python3 test/test_recheck.py``（不需要 ROS / pytest）。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_patrol_v2 import recheck  # noqa: E402


class ParseSummaryTest(unittest.TestCase):

    def test_empty_message_is_ignored(self):
        self.assertIsNone(recheck.parse_summary(''))

    def test_none_is_ignored(self):
        self.assertIsNone(recheck.parse_summary(None))

    def test_no_object(self):
        self.assertEqual(recheck.parse_summary('no object detected'), (0, []))

    def test_single(self):
        count, confs = recheck.parse_summary('fire_extinguisher (0.92)')
        self.assertEqual(count, 1)
        self.assertEqual(confs, [0.92])

    def test_two_of_same_class(self):
        count, confs = recheck.parse_summary('fire_extinguisher x2 (0.92,0.91)')
        self.assertEqual(count, 2)
        self.assertEqual(confs, [0.92, 0.91])

    def test_multiple_classes(self):
        count, confs = recheck.parse_summary(
            'fire_extinguisher (0.80); bottle (0.45)')
        self.assertEqual(count, 2)
        self.assertEqual(confs, [0.80, 0.45])

    def test_unknown_format_returns_none(self):
        self.assertIsNone(recheck.parse_summary('something unexpected'))

    def test_json_result_returns_none(self):
        # 以后视觉改成 JSON 也不该崩，只是这次不做走近复核
        self.assertIsNone(recheck.parse_summary('{"detections": []}'))

    def test_bad_confidence_returns_none(self):
        self.assertIsNone(recheck.parse_summary('fire_extinguisher (abc)'))


class EvaluateTest(unittest.TestCase):

    def test_confident_present_never_moves(self):
        decision, top = recheck.evaluate(1, [0.9], 0, 2, 0.25, 0.6)
        self.assertEqual(decision, recheck.DECISION_ACCEPT_PRESENT)
        self.assertAlmostEqual(top, 0.9)

    def test_grey_zone_moves_when_budget_left(self):
        decision, _ = recheck.evaluate(1, [0.4], 0, 2, 0.25, 0.6)
        self.assertEqual(decision, recheck.DECISION_MOVE)

    def test_grey_zone_accepts_when_out_of_budget(self):
        decision, _ = recheck.evaluate(1, [0.4], 2, 2, 0.25, 0.6)
        self.assertEqual(decision, recheck.DECISION_ACCEPT_PRESENT)

    def test_absent_moves_to_confirm_when_on_miss(self):
        decision, _ = recheck.evaluate(0, [], 0, 2, 0.25, 0.6, on_miss=True)
        self.assertEqual(decision, recheck.DECISION_MOVE)

    def test_absent_stays_when_on_miss_disabled(self):
        decision, _ = recheck.evaluate(0, [], 0, 2, 0.25, 0.6, on_miss=False)
        self.assertEqual(decision, recheck.DECISION_ACCEPT_ABSENT)

    def test_below_low_is_treated_as_absent(self):
        decision, _ = recheck.evaluate(1, [0.10], 2, 2, 0.25, 0.6)
        self.assertEqual(decision, recheck.DECISION_ACCEPT_ABSENT)

    def test_zero_budget_disables_moving(self):
        decision, _ = recheck.evaluate(1, [0.4], 0, 0, 0.25, 0.6)
        self.assertEqual(decision, recheck.DECISION_ACCEPT_PRESENT)


class MergeTest(unittest.TestCase):

    def test_picks_highest_confidence_round(self):
        rounds = [(1, [0.35]), (2, [0.88, 0.81]), (1, [0.70])]
        count, confs, top = recheck.merge(rounds)
        self.assertEqual(count, 2)
        self.assertEqual(confs, [0.88, 0.81])
        self.assertAlmostEqual(top, 0.88)

    def test_empty_rounds(self):
        self.assertEqual(recheck.merge([]), (0, [], 0.0))

    def test_ignores_empty_later_round(self):
        # 走近之后什么都没看到，不能把上一轮的低分结论抹掉
        rounds = [(1, [0.42]), (0, [])]
        count, _confs, top = recheck.merge(rounds)
        self.assertEqual(count, 1)
        self.assertAlmostEqual(top, 0.42)


if __name__ == '__main__':
    unittest.main(verbosity=2)
