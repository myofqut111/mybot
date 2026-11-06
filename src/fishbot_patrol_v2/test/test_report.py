"""report.py 的单测：JSONL 逐行写入、检测结果归一化、汇总计数。"""

import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_patrol_v2 import report  # noqa: E402
from fishbot_patrol_v2.plan import Job  # noqa: E402


def make_job(point_id='p0', loop_index=0, route_index=0, attempt=1):
    return Job(job_id='L%dN%d' % (loop_index, route_index),
               loop_index=loop_index, route_index=route_index,
               point_id=point_id, task_id='', wait_seconds=0.0,
               attempt=attempt)


class ReportTest(unittest.TestCase):
    """报告写入器行为。"""

    def setUp(self):
        self.directory = tempfile.mkdtemp(prefix='patrol_report_test_')

    def tearDown(self):
        shutil.rmtree(self.directory, ignore_errors=True)

    def make_report(self, clock=None):
        return report.PatrolReport(
            self.directory, route_name='floor1', execution_id='abcdef123456',
            mode=0, policy='skip', clock=clock or (lambda: '2026-10-19T12:00:00'))

    def read_records(self, path):
        with open(path, 'r', encoding='utf-8') as stream:
            return [json.loads(line) for line in stream if line.strip()]

    def test_header_and_point_lines(self):
        writer = self.make_report()
        writer.point(make_job('p0'), report.STATUS_OK, nav_seconds=12.345,
                     task_seconds=2.0, message='done')
        writer.point(make_job('p1', route_index=1), report.STATUS_SKIPPED,
                     message='nav failed')
        records = self.read_records(writer.path)
        self.assertEqual(records[0]['type'], 'header')
        self.assertEqual(records[0]['route_name'], 'floor1')
        self.assertEqual([item['type'] for item in records],
                         ['header', 'point', 'point'])
        self.assertAlmostEqual(records[1]['nav_seconds'], 12.345)
        self.assertEqual(records[2]['status'], report.STATUS_SKIPPED)

    def test_header_records_jobs_total(self):
        """表头要记下"这一轮总共几个作业"，否则多轮巡检进度会显示成 13/3。"""
        writer = report.PatrolReport(
            self.directory, route_name='floor1', execution_id='abcdef123456',
            mode=0, policy='skip', clock=lambda: '2026-10-19T12:00:00',
            jobs_total=15)
        records = self.read_records(writer.path)
        self.assertEqual(records[0]['jobs_total'], 15)

    def test_detections_are_normalized(self):
        writer = self.make_report()
        writer.point(make_job(), report.STATUS_OK, detections=[
            {'label': 'fire_extinguisher', 'confidence': 0.861234,
             'box': [1.23456, 2.0, 3.0, 4.0]},
        ])
        records = self.read_records(writer.path)
        detection = records[1]['detections'][0]
        self.assertEqual(detection['label'], 'fire_extinguisher')
        self.assertAlmostEqual(detection['confidence'], 0.8612)
        self.assertAlmostEqual(detection['box'][0], 1.23)

    def test_summary_counts_and_findings(self):
        writer = self.make_report()
        writer.point(make_job('p0'), report.STATUS_OK, detections=[
            {'label': 'fire_extinguisher', 'confidence': 0.9, 'box': [0, 0, 1, 1]}])
        writer.point(make_job('p1', route_index=1), report.STATUS_OK)
        writer.point(make_job('p2', route_index=2), report.STATUS_FAILED,
                     message='timeout')
        writer.event('preempt', reason='fire')
        summary_path = writer.close(extra={'route_name': 'floor1'})
        self.assertTrue(os.path.exists(summary_path))
        with open(summary_path, 'r', encoding='utf-8') as stream:
            summary = json.load(stream)
        self.assertEqual(summary['points'], {'ok': 2, 'failed': 1})
        self.assertEqual(summary['detections_total'], 1)
        self.assertEqual(summary['points_with_detections'], {'p0': 1})
        self.assertEqual(summary['route_name'], 'floor1')
        # summary 也会作为最后一行写进 JSONL
        self.assertEqual(self.read_records(writer.path)[-1]['type'], 'summary')

    def test_close_is_idempotent(self):
        writer = self.make_report()
        first = writer.close()
        second = writer.close()
        self.assertEqual(first, second)
        self.assertEqual(self.read_records(writer.path)[-1]['type'], 'summary')

    def test_summarize_ignores_non_point_records(self):
        summary = report.summarize([
            {'type': 'header'},
            {'type': 'point', 'status': 'ok', 'detections': [{'label': 'x'}]},
            {'type': 'event', 'kind': 'pause'},
        ])
        self.assertEqual(summary['points'], {'ok': 1})
        self.assertEqual(summary['detections_total'], 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
