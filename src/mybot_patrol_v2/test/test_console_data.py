#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""console_data 的单元测试：不依赖 ROS，python3 直接跑。"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_patrol_v2 import console_data as cd  # noqa: E402


class BatteryBandTest(unittest.TestCase):
    def test_three_bands(self):
        self.assertEqual(cd.battery_band(12.6), 'ok')
        self.assertEqual(cd.battery_band(11.8), 'warn')     # 等于阈值算 warn
        self.assertEqual(cd.battery_band(11.5), 'crit')
        self.assertEqual(cd.battery_band(11.4), 'crit')

    def test_bad_values_are_unknown(self):
        for bad in (None, 'abc', 0, -1.0, float('nan')):
            self.assertEqual(cd.battery_band(bad), cd.BATTERY_UNKNOWN)

    def test_custom_thresholds(self):
        self.assertEqual(cd.battery_band(10.8, warn_v=10.8, crit_v=10.2), 'warn')


class NormalizeStatusTest(unittest.TestCase):
    def test_full_payload(self):
        raw = {
            'execution_id': 'abc', 'route_name': 'main', 'state': 'WAITING_TASK',
            'message': 'waiting', 'current_index': 3, 'completed_points': 2,
            'current_loop': 1, 'point_id': 'p3', 'time': 1000.0,
            'battery_v': 11.4, 'battery_level': 'crit',
        }
        view = cd.normalize_status(raw, now=1005.0)
        self.assertTrue(view['connected'])
        self.assertEqual(view['state'], 'WAITING_TASK')
        self.assertTrue(view['known_state'])
        self.assertFalse(view['terminal'])
        self.assertEqual(view['completed_points'], 2)
        self.assertAlmostEqual(view['age_s'], 5.0)
        self.assertEqual(view['battery_band'], 'crit')

    def test_json_string_and_terminal(self):
        view = cd.normalize_status('{"state": "SUCCEEDED", "time": 10}',
                                   now=12.0)
        self.assertEqual(view['state'], 'SUCCEEDED')
        self.assertTrue(view['terminal'])
        self.assertAlmostEqual(view['age_s'], 2.0)

    def test_broken_inputs(self):
        for bad in (None, '', '{', 'null', [], 42):
            view = cd.normalize_status(bad)
            self.assertIsNone(view['state'])
            self.assertFalse(view['connected'])

    def test_missing_fields_do_not_raise(self):
        view = cd.normalize_status({'state': 'RUNNING'})
        self.assertEqual(view['completed_points'], 0)
        self.assertIsNone(view['battery_v'])
        self.assertIsNone(view['age_s'])

    def test_unknown_state_flag(self):
        view = cd.normalize_status({'state': 'WHATEVER'})
        self.assertFalse(view['known_state'])


class SummarizeRunTest(unittest.TestCase):
    def test_full(self):
        summary = {
            'execution_id': 'e1', 'route_name': 'main', 'mode': 0,
            'started_at': 1.0, 'duration_sec': 83.0,
            'points': {'SUCCEEDED': 3, 'SKIPPED': 1, 'FAILED': 0},
            'detections_total': 4, 'verdicts': {'tp': 3, 'fp': 0, 'fn': 1, 'tn': 1},
            'graded_points': 5, 'recall': 0.75, 'precision': 1.0,
            'false_alarm_rate': 0.0, 'battery_v': 12.1,
        }
        row = cd.summarize_run(summary)
        self.assertEqual(row['total'], 4)
        self.assertEqual(row['ok'], 3)
        self.assertEqual(row['graded_points'], 5)
        self.assertAlmostEqual(row['recall'], 0.75)

    def test_missing_verdicts_keeps_none(self):
        row = cd.summarize_run({'points': {'SUCCEEDED': 1}})
        self.assertEqual(row['total'], 1)
        self.assertIsNone(row['recall'])
        self.assertIsNone(row['precision'])
        self.assertEqual(row['graded_points'], 0)

    def test_garbage_input(self):
        row = cd.summarize_run(None)
        self.assertEqual(row['total'], 0)
        self.assertEqual(row['name'], '')

    def test_points_not_dict(self):
        row = cd.summarize_run({'points': [1, 2, 3]})
        self.assertEqual(row['total'], 0)


class FileHelpersTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='console_test_')

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def _write(self, name, text):
        path = os.path.join(self.dir, name)
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(text)
        return path

    def test_list_runs_sorted_and_limited(self):
        for i in range(3):
            path = self._write('run%d.summary.json' % i,
                               json.dumps({'execution_id': 'e%d' % i,
                                           'points': {'SUCCEEDED': i}}))
            os.utime(path, (time.time() - i, time.time() - i))
        rows = cd.list_runs(self.dir, limit=2)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['name'], 'e0')     # 最新（mtime 最大）

    def test_list_runs_bad_json_skipped(self):
        self._write('bad.summary.json', '{not json')
        rows = cd.list_runs(self.dir)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['total'], 0)

    def test_list_runs_missing_dir(self):
        self.assertEqual(cd.list_runs(os.path.join(self.dir, 'nope')), [])

    def test_tail_events(self):
        lines = '\n'.join(json.dumps({'type': 'point', 'point_id': 'p%d' % i})
                          for i in range(10))
        path = self._write('a.jsonl', lines + '\n{broken\n')
        events = cd.tail_events(path, limit=3)
        self.assertEqual(len(events), 3)
        self.assertEqual(events[-1]['point_id'], 'p9')

    def test_tail_events_missing_file(self):
        self.assertEqual(cd.tail_events(os.path.join(self.dir, 'x.jsonl')), [])


class FormatTest(unittest.TestCase):
    def test_duration(self):
        self.assertEqual(cd.fmt_duration(None), '—')
        self.assertEqual(cd.fmt_duration(45), '45s')
        self.assertEqual(cd.fmt_duration(200), '3m20s')
        self.assertEqual(cd.fmt_duration(5000), '1h23m')

    def test_rate(self):
        self.assertEqual(cd.fmt_rate(None), '—')
        self.assertEqual(cd.fmt_rate(0.9234), '92.3%')

    def test_event_line(self):
        text = cd.event_line({'type': 'point', 'point_id': 'p1',
                              'status': 'SUCCEEDED', 'message': 'fire x2'})
        self.assertIn('p1', text)
        self.assertIn('SUCCEEDED', text)
        self.assertIn('fire x2', text)
        self.assertEqual(cd.event_line(None), '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
