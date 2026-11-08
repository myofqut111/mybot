"""跑完巡检输出的那份人读报告：内容对、失败不影响主流程。"""

import os
import shutil
import tempfile
import unittest
from types import SimpleNamespace

from fishbot_patrol_v2 import report as report_module


def job(point_id='p1', task_id='detect'):
    return SimpleNamespace(job_id='j-%s' % point_id, point_id=point_id,
                           task_id=task_id, loop_index=0, route_index=0,
                           attempt=1)


class RenderCase(unittest.TestCase):

    def test_full_summary(self):
        summary = {
            'route_name': 'main2', 'execution_id': 'exec-1604',
            'duration_sec': 192.4,
            'points': {'ok': 5, 'skipped': 1},
            'detections_total': 11,
            'verdicts': {'tp': 5, 'fp': 1, 'fn': 0, 'tn': 0},
            'graded_points': 6, 'recall': 1.0, 'precision': 0.8333,
            'false_alarm_rate': 0.1667, 'battery_v': 12.3,
        }
        text = report_module.render_summary(summary)
        self.assertIn('巡检报告 · main2 · exec-1604', text)
        self.assertIn('耗时 3 分 12 秒', text)
        self.assertIn('成功 5 · 跳过 1 · 失败 0（共 6）', text)
        self.assertIn('11 个目标', text)
        self.assertIn('命中 5 · 误报 1 · 漏检 0 · 正确排除 0', text)
        self.assertIn('召回 1.00', text)
        self.assertIn('精确 0.83', text)
        self.assertIn('电量     12.30V', text)

    def test_learned_baseline_is_explained(self):
        text = report_module.render_summary({
            'route_name': 'main', 'points': {'ok': 3}, 'baseline_learned': 3})
        self.assertIn('本轮新学 3 个点（下次巡检起自动判定）', text)
        self.assertNotIn('召回', text)

    def test_no_baseline_no_verdict_is_quiet(self):
        text = report_module.render_summary({
            'route_name': 'main', 'points': {'ok': 2}})
        self.assertNotIn('期望值', text)
        self.assertNotIn('基线', text)

    def test_aborted_and_canceled_wording(self):
        self.assertIn('已中止', report_module.render_summary(
            {'aborted': True, 'points': {'failed': 1}}))
        self.assertIn('被中断', report_module.render_summary(
            {'points': {'canceled': 2}}))
        self.assertIn('部分失败', report_module.render_summary(
            {'points': {'ok': 1, 'failed': 1}}))

    def test_empty_summary_does_not_raise(self):
        self.assertIn('巡检报告', report_module.render_summary({}))
        self.assertIn('巡检报告', report_module.render_summary(None))

    def test_point_lines_and_truncation(self):
        records = [{'type': 'point', 'point_id': 'p%d' % i, 'status': 'ok',
                    'task_id': 'detect', 'nav_seconds': 1.0,
                    'task_seconds': 0.5, 'detections': []} for i in range(5)]
        text = report_module.render_summary({'points': {'ok': 5}}, records,
                                            limit=2)
        self.assertIn('点位明细', text)
        self.assertIn('还有 3 个点未列出', text)

    def test_point_line_shows_confidence_and_verdict_source(self):
        record = {'type': 'point', 'point_id': 'p9', 'status': 'ok',
                  'task_id': 'inspect', 'nav_seconds': 3.4, 'task_seconds': 1.2,
                  'detections': [{'confidence': 0.92}, {'confidence': 0.71},
                                 {'confidence': 0.55}, {'confidence': 0.4}],
                  'verdict': 'tp', 'expect_source': 'baseline'}
        line = report_module._point_line(record)
        self.assertIn('4 个目标 (0.92, 0.71, 0.55 …)', line)
        self.assertIn('命中(基线)', line)
        self.assertIn('导航 3.4s', line)

    def test_duration_formats(self):
        self.assertEqual(report_module._duration(12.4), '12 秒')
        self.assertEqual(report_module._duration(90), '1 分 30 秒')
        self.assertEqual(report_module._duration(None), '未知')


class CloseWritesTextCase(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)

    def test_close_writes_human_report(self):
        writer = report_module.PatrolReport(self.dir, 'main2', 'exec-1', 1,
                                            'retry')
        writer.point(job('p1'), 'ok', nav_seconds=2.0, detections=[
            {'confidence': 0.9, 'class_id': 0, 'label': 'fire_extinguisher',
             'bbox': [1, 2, 3, 4]}])
        writer.point(job('p2', 'inspect'), 'ok', nav_seconds=1.0)
        writer.close({'route_name': 'main2', 'baseline_learned': 2})

        self.assertTrue(os.path.exists(writer.text_path))
        with open(writer.text_path, encoding='utf-8') as handle:
            saved = handle.read()
        self.assertIn('巡检报告 · main2', saved)
        self.assertIn('p1', saved)
        self.assertEqual(saved.strip(), writer.text.strip())
        leftovers = [n for n in os.listdir(self.dir) if n.endswith('.tmp')]
        self.assertEqual(leftovers, [])


class AbortReasonCase(unittest.TestCase):
    """一个点都没跑时，报告要说清是电量还是空路线。"""

    def test_battery_abort_before_start(self):
        summary = {
            'route_name': 'main4', 'execution_id': 'exec-1913',
            'duration_sec': 0.02, 'points': {}, 'battery_v': 11.44,
            'battery_level': 'crit',
            'abort_reason': 'battery critical (11.44V), stopping after current point',
        }
        text = report_module.render_summary(summary)
        self.assertIn('未开始', text)
        self.assertIn('battery critical', text)
        self.assertNotIn('无点位', text)

    def test_abort_reason_collected_from_events(self):
        records = [
            {'type': 'header', 'route_name': 'main4'},
            {'type': 'event', 'kind': 'battery_abort',
             'reason': 'battery critical (11.44V), stopping after current point'},
        ]
        summary = report_module.summarize(records)
        self.assertEqual(
            summary['abort_reason'],
            'battery critical (11.44V), stopping after current point')

    def test_no_reason_still_says_no_points(self):
        summary = {'route_name': 'empty', 'points': {}, 'duration_sec': 0.0}
        text = report_module.render_summary(summary)
        self.assertIn('无点位', text)


if __name__ == '__main__':
    unittest.main()
