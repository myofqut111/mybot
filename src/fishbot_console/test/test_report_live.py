"""报告页的四个真机问题，各钉一条契约。

1. 明明每行 ok，表头却是「成功 0/0」——执行器写小写 `points = {'ok': n}`，
   而 summarize_run 只认大写 `SUCCEEDED`。
2. 巡检进行中报告页永远空——list_runs 只扫 `*.summary.json`，而汇总要等
   整趟跑完才写。
3. 事件接口忽略 `?run=`——页面切了运行，表格还是最新那轮。
4. 按了停止，面板还写着"运行中"——状态消息不更新 + 前端从不清理这句提示。
"""

import json
import os
import shutil
import tempfile
import unittest

from fishbot_console import data


class SummaryKeysCase(unittest.TestCase):

    def test_lowercase_counts(self):
        row = data.summarize_run({'points': {'ok': 3, 'failed': 1}, 'execution_id': 'x'})
        self.assertEqual((row['ok'], row['failed'], row['total']), (3, 1, 4))

    def test_uppercase_still_supported(self):
        row = data.summarize_run({'points': {'SUCCEEDED': 2, 'SKIPPED': 1}})
        self.assertEqual((row['ok'], row['skipped'], row['total']), (2, 1, 3))

    def test_canceled_counts_into_total(self):
        row = data.summarize_run({'points': {'ok': 2, 'canceled': 4}})
        self.assertEqual(row['canceled'], 4)
        self.assertEqual(row['total'], 6)


class LiveRunsCase(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)
        self.base = os.path.join(self.dir, 'patrol_20260101_000000_abcd')
        with open(self.base + '.jsonl', 'w', encoding='utf-8') as handle:
            for item in (
                {'type': 'header', 'time': '2026-01-01T00:00:00',
                 'route_name': 'main2', 'execution_id': 'exec-1'},
                {'type': 'point', 'status': 'ok', 'point_id': 'p1',
                 'detections': [{'label': 'fire_extinguisher'}]},
                {'type': 'point', 'status': 'ok', 'point_id': 'p2',
                 'detections': [{'label': 'fire_extinguisher'},
                                {'label': 'fire_extinguisher'}]},
            ):
                handle.write(json.dumps(item, ensure_ascii=False) + '\n')

    def test_unfinished_run_is_listed(self):
        runs = data.list_runs(self.dir)
        self.assertEqual(len(runs), 1)
        self.assertTrue(runs[0]['live'])
        self.assertEqual(runs[0]['ok'], 2)
        self.assertEqual(runs[0]['route_name'], 'main2')
        self.assertEqual(runs[0]['detections_total'], 3)

    def test_finished_run_prefers_summary(self):
        with open(self.base + '.summary.json', 'w', encoding='utf-8') as handle:
            json.dump({'execution_id': 'exec-1', 'route_name': 'main2',
                       'points': {'ok': 2}, 'duration_sec': 12.0}, handle)
        runs = data.list_runs(self.dir)
        self.assertEqual(len(runs), 1, '同一轮不该出现两行')
        self.assertFalse(runs[0]['live'])
        self.assertEqual(runs[0]['duration_sec'], 12.0)

    def test_find_run_path_by_execution_id(self):
        path = data.find_run_path(self.dir, 'exec-1')
        self.assertTrue(path.endswith('.jsonl'))
        self.assertTrue(os.path.exists(path))

    def test_find_run_path_by_file_stem(self):
        stem = os.path.basename(self.base)
        path = data.find_run_path(self.dir, stem)
        self.assertEqual(os.path.basename(path), stem + '.jsonl')

    def test_find_run_path_unknown_is_empty(self):
        self.assertEqual(data.find_run_path(self.dir, 'nope'), '')


class PageContractCase(unittest.TestCase):
    """页面里的两处提示，改坏了用户又要来问。"""

    @classmethod
    def setUpClass(cls):
        from fishbot_console import page
        cls.src = page.HTML

    def test_record_dialog_has_task_selector(self):
        self.assertIn('id="recTask"', self.src)
        self.assertIn("$('recTask').value", self.src,
                      '录点时要把选中的任务发上去，不能写死 detect')

    def test_patrol_hint_clears_after_stop(self):
        self.assertIn('patrolWasRunning', self.src)
        self.assertIn('巡检已停止', self.src)

    def test_run_summary_shows_live_and_planned(self):
        self.assertIn('run.planned', self.src)
        self.assertIn('进行中', self.src)


if __name__ == '__main__':
    unittest.main()
