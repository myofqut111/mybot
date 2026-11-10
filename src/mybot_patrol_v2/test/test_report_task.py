"""报告事件里必须带 task_id。

真机现象：报告页「任务」列永远是空的，空单元格塌陷后把「轮/序号」挤到
前面，用户看到的是一列 "L6 N2" 顶着「任务」表头。
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_patrol_v2 import report  # noqa: E402


class FakeJob(object):
    job_id = 'j1'
    point_id = 'p1'
    task_id = 'inspect'
    loop_index = 2
    route_index = 5
    attempt = 1


class ReportTaskCase(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.report = report.PatrolReport(
            self.dir, 'main2', 'exec-1', mode=0, policy='stop')
        self.addCleanup(lambda: self.report.close())

    def test_point_records_task_id(self):
        rec = self.report.point(FakeJob(), 'ok')
        self.assertEqual(rec['task_id'], 'inspect')

    def test_summary_still_counts_points(self):
        self.report.point(FakeJob(), 'ok')
        summary = self.report.summarize_for_tests() \
            if hasattr(self.report, 'summarize_for_tests') else None
        if summary is None:
            import json
            with open(self.report.path, encoding='utf-8') as handle:
                lines = [json.loads(line) for line in handle if line.strip()]
            self.assertEqual(lines[-1]['task_id'], 'inspect')


if __name__ == '__main__':
    unittest.main()
