"""期望值判定（expect → tp/fp/fn/tn）与报告指标的单测。

这套东西的价值全在「口径别错」：fn 和 fp 弄反了，报告里的召回率会很高，
现场却是到处漏检。所以每个分支都要钉死。
"""

import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_patrol_v2 import report  # noqa: E402
from fishbot_patrol_v2.plan import Job  # noqa: E402
from fishbot_patrol_v2.util import (  # noqa: E402
    normalize_expect, verdict_of)


def make_job(point_id='p0'):
    return Job(job_id='L0N0', loop_index=0, route_index=0, point_id=point_id,
               task_id='', wait_seconds=0.0, attempt=1)


class NormalizeExpectTest(unittest.TestCase):

    def test_accepts_known_values(self):
        self.assertEqual(normalize_expect('present'), 'present')
        self.assertEqual(normalize_expect('ABSENT'), 'absent')
        self.assertEqual(normalize_expect('  Present '), 'present')

    def test_unknown_becomes_empty(self):
        """拼错的词一律当“没填”，绝不猜——猜错会让指标失真。"""
        for value in ('preasent', 'yes', '1', None, '', '  '):
            self.assertEqual(normalize_expect(value), '')


class VerdictTest(unittest.TestCase):

    def test_present_expected(self):
        self.assertEqual(verdict_of('present', True), 'tp')
        self.assertEqual(verdict_of('present', False), 'fn')

    def test_absent_expected(self):
        self.assertEqual(verdict_of('absent', True), 'fp')
        self.assertEqual(verdict_of('absent', False), 'tn')

    def test_no_expectation_is_unknown(self):
        self.assertEqual(verdict_of('', True), 'unknown')
        self.assertEqual(verdict_of(None, False), 'unknown')

    def test_counts_not_confidence(self):
        """只要有框就算“看到了”：置信度高低由视觉节点自己卡阈值。"""
        self.assertEqual(verdict_of('present', [{'label': 'x'}]), 'tp')
        self.assertEqual(verdict_of('present', []), 'fn')


class ReportMetricsTest(unittest.TestCase):

    def setUp(self):
        self.directory = tempfile.mkdtemp(prefix='patrol_metrics_')

    def tearDown(self):
        shutil.rmtree(self.directory, ignore_errors=True)

    def _make_report(self):
        return report.PatrolReport(
            self.directory, 'r', 'exec0001', 0, 'skip')

    def test_summary_counts_and_rates(self):
        report_writer = self._make_report()
        job = make_job()
        # tp / fn / fp / tn 各一条
        report_writer.point(job, report.STATUS_OK, detections=[{'label': 'x'}],
                            expect='present', verdict='tp')
        report_writer.point(job, report.STATUS_OK, detections=[],
                            expect='present', verdict='fn')
        report_writer.point(job, report.STATUS_OK, detections=[{'label': 'x'}],
                            expect='absent', verdict='fp')
        report_writer.point(job, report.STATUS_OK, detections=[],
                            expect='absent', verdict='tn')
        summary_path = report_writer.close()
        with open(summary_path, encoding='utf-8') as stream:
            summary = json.load(stream)

        self.assertEqual(summary['verdicts'],
                         {'tp': 1, 'fp': 1, 'fn': 1, 'tn': 1})
        self.assertEqual(summary['graded_points'], 4)
        self.assertAlmostEqual(summary['recall'], 0.5)
        self.assertAlmostEqual(summary['precision'], 0.5)
        self.assertAlmostEqual(summary['false_alarm_rate'], 0.5)

    def test_points_without_expect_are_excluded(self):
        report_writer = self._make_report()
        report_writer.point(make_job(), report.STATUS_OK,
                            detections=[{'label': 'x'}])
        summary_path = report_writer.close()
        with open(summary_path, encoding='utf-8') as stream:
            summary = json.load(stream)
        self.assertEqual(summary['graded_points'], 0)
        # 没有可判定点位时给 None，而不是 0（0 会被读成“准确率为零”）
        self.assertIsNone(summary['recall'])
        self.assertIsNone(summary['precision'])
        self.assertIsNone(summary['false_alarm_rate'])

    def test_record_keeps_expect_and_verdict(self):
        report_writer = self._make_report()
        record = report_writer.point(make_job(), report.STATUS_OK,
                                     expect='present', verdict='fn')
        self.assertEqual(record['expect'], 'present')
        self.assertEqual(record['verdict'], 'fn')
        report_writer.close()

    def test_record_omits_fields_when_no_expect(self):
        report_writer = self._make_report()
        record = report_writer.point(make_job(), report.STATUS_OK)
        self.assertNotIn('expect', record)
        self.assertNotIn('verdict', record)
        report_writer.close()


if __name__ == '__main__':
    unittest.main()
