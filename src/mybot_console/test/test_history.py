#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""history.py 的测试：把 jsonl 折叠成点位时间线，脏数据不能抛。"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_console import history  # noqa: E402


def write_run(directory, name, route, points, when='2026-11-06T16:49:00', mtime=None):
    """写一份最小可用的报告：表头 + 若干 point 行。"""
    path = os.path.join(directory, name + '.jsonl')
    rows = [{'type': 'header', 'execution_id': name, 'route_name': route,
             'mode': 1, 'time': when, 'jobs_total': len(points)}]
    for item in points:
        row = {'type': 'point', 'point_id': item['point'], 'status': item.get('status', 'SUCCEEDED'),
               'task_id': item.get('task', 'inspect'), 'detections': item.get('detections', [])}
        if item.get('expect'):
            row['expect'] = item['expect']
            row['verdict'] = item.get('verdict', '')
        rows.append(row)
    with open(path, 'w', encoding='utf-8') as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + '\n')
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


class PointHistoryCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='hist-')

    def test_empty_dir(self):
        out = history.point_history(self.tmp)
        self.assertTrue(out['ok'])
        self.assertEqual(out['runs'], 0)
        self.assertEqual(out['points'], [])

    def test_missing_dir(self):
        out = history.point_history(os.path.join(self.tmp, 'nope'))
        self.assertEqual(out['points'], [])
        self.assertEqual(out['runs'], 0)

    def test_aggregates_and_orders_missing_first(self):
        write_run(self.tmp, 'a', 'main2', [
            {'point': 'p1', 'expect': 'present', 'verdict': 'tp', 'detections': [{'label': 'x'}]},
            {'point': 'p2', 'expect': 'present', 'verdict': 'fn', 'detections': []},
        ])
        write_run(self.tmp, 'b', 'main2', [
            {'point': 'p1', 'expect': 'present', 'verdict': 'tp', 'detections': [{'label': 'x'}]},
            {'point': 'p2', 'expect': 'present', 'verdict': 'fn', 'detections': []},
        ])
        out = history.point_history(self.tmp)
        self.assertEqual(out['runs'], 2)
        self.assertEqual(out['routes'], ['main2'])
        self.assertEqual(len(out['points']), 2)
        # 缺两次的 p2 排第一
        self.assertEqual(out['points'][0]['point'], 'p2')
        self.assertEqual(out['points'][0]['missing'], 2)
        self.assertEqual(out['points'][0]['runs'], 2)
        self.assertEqual(out['points'][1]['point'], 'p1')
        self.assertEqual(out['points'][1]['missing'], 0)

    def test_dedup_within_one_run(self):
        # 同一轮里同一点重试了两次，只记第一次
        write_run(self.tmp, 'a', 'main2', [
            {'point': 'p1', 'expect': 'present', 'verdict': 'fn', 'detections': []},
            {'point': 'p1', 'expect': 'present', 'verdict': 'tp', 'detections': [{'label': 'x'}]},
        ])
        out = history.point_history(self.tmp)
        self.assertEqual(out['points'][0]['runs'], 1)

    def test_route_filter(self):
        write_run(self.tmp, 'a', 'main2', [{'point': 'p1', 'expect': 'present', 'verdict': 'tp',
                                            'detections': [{'label': 'x'}]}])
        write_run(self.tmp, 'b', 'main3', [{'point': 'q1', 'expect': 'present', 'verdict': 'fn'}])
        out = history.point_history(self.tmp, route='main3')
        self.assertEqual(len(out['points']), 1)
        self.assertEqual(out['points'][0]['route'], 'main3')
        self.assertEqual(out['runs'], 1)

    def test_days_filter(self):
        import time as _time
        old = _time.time() - 40 * 86400
        write_run(self.tmp, 'old', 'main2', [{'point': 'p1', 'expect': 'present',
                                              'verdict': 'fn'}],
                  when='2026-09-01T10:00:00', mtime=old)
        write_run(self.tmp, 'new', 'main2', [{'point': 'p1', 'expect': 'present',
                                              'verdict': 'tp',
                                              'detections': [{'label': 'x'}]}])
        out = history.point_history(self.tmp, days=7)
        self.assertEqual(out['runs'], 1, '30 天前那轮应被 days 过滤掉')

    def test_no_expect_means_unknown(self):
        write_run(self.tmp, 'a', 'main2', [{'point': 'p1', 'detections': [{'label': 'x'}]}])
        out = history.point_history(self.tmp)
        self.assertEqual(out['points'][0]['unknown'], 1)
        self.assertEqual(out['points'][0]['samples'][0]['verdict'], '')

    def test_bad_verdict_falls_back_to_unknown(self):
        write_run(self.tmp, 'a', 'main2', [{'point': 'p1', 'expect': 'present',
                                            'verdict': 'weird'}])
        out = history.point_history(self.tmp)
        self.assertEqual(out['points'][0]['unknown'], 1)

    def test_run_label_uses_uuid_when_header_has_no_id(self):
        """报告还没 close 时表头没有 execution_id，标签要取文件名里的 uuid 段，
        不能变成 "patrol_2" 这种谁都认不出的东西。"""
        path = os.path.join(self.tmp, 'patrol_20261109_191500_013b39eb.jsonl')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(json.dumps({'type': 'header', 'route_name': 'main2',
                                     'time': '2026-11-09T19:15:00'}) + '\n')
            handle.write(json.dumps({'type': 'point', 'point_id': 'p1',
                                     'status': 'ok', 'expect': 'present',
                                     'verdict': 'tp', 'detections': [{'label': 'x'}],
                                     'time': '2026-11-09T19:15:41'}) + '\n')
        out = history.point_history(self.tmp)
        self.assertEqual(out['points'][0]['samples'][0]['run'], '013b39eb')

    def test_sample_time_uses_point_time_not_run_time(self):
        path = os.path.join(self.tmp, 'patrol_20261109_191500_x.jsonl')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(json.dumps({'type': 'header', 'execution_id': 'x',
                                     'route_name': 'main2',
                                     'time': '2026-11-09T19:15:00'}) + '\n')
            handle.write(json.dumps({'type': 'point', 'point_id': 'p1',
                                     'status': 'ok', 'expect': 'present',
                                     'verdict': 'tp', 'detections': [{'label': 'x'}],
                                     'time': '2026-11-09T19:17:33'}) + '\n')
        out = history.point_history(self.tmp)
        self.assertEqual(out['points'][0]['last_time'], '2026-11-09T19:17:33')

    def test_detected_falls_back_to_verdict(self):
        """老报告没有 detections 字段时，不能把"看到了"显示成"没看到"。"""
        path = os.path.join(self.tmp, 'patrol_20261109_191500_y.jsonl')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(json.dumps({'type': 'header', 'execution_id': 'y',
                                     'route_name': 'main2'}) + '\n')
            handle.write(json.dumps({'type': 'point', 'point_id': 'p1',
                                     'status': 'ok', 'expect': 'present',
                                     'verdict': 'tp'}) + '\n')
        out = history.point_history(self.tmp)
        self.assertTrue(out['points'][0]['samples'][0]['detected'])

    def test_broken_lines_are_skipped(self):
        path = write_run(self.tmp, 'a', 'main2', [{'point': 'p1', 'expect': 'present',
                                                   'verdict': 'tp',
                                                   'detections': [{'label': 'x'}]}])
        with open(path, 'a', encoding='utf-8') as handle:
            handle.write('{"type": "point", "point_id": "p2"')   # 被 kill 时常见的半行
        out = history.point_history(self.tmp)
        self.assertEqual(len(out['points']), 1)

    def test_samples_capped(self):
        for index in range(5):
            write_run(self.tmp, 'run%d' % index, 'main2',
                      [{'point': 'p1', 'expect': 'present', 'verdict': 'tp',
                        'detections': [{'label': 'x'}]}])
        out = history.point_history(self.tmp, per_point=3)
        self.assertEqual(out['points'][0]['runs'], 5)
        self.assertEqual(len(out['points'][0]['samples']), 3)

    def test_detected_flag(self):
        write_run(self.tmp, 'a', 'main2', [
            {'point': 'p1', 'expect': 'present', 'verdict': 'tp', 'detections': [{'label': 'x'}]},
            {'point': 'p2', 'expect': 'absent', 'verdict': 'tn', 'detections': []},
        ])
        out = {row['point']: row for row in history.point_history(self.tmp)['points']}
        self.assertTrue(out['p1']['samples'][0]['detected'])
        self.assertFalse(out['p2']['samples'][0]['detected'])


if __name__ == '__main__':
    unittest.main()
