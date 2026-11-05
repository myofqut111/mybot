#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""data.py 的纯逻辑测试：脏数据不能抛异常，缺失字段不能崩页面。"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import data  # noqa: E402


class TestBatteryBand(unittest.TestCase):
    def test_bands(self):
        self.assertEqual(data.battery_band(12.6), 'ok')
        self.assertEqual(data.battery_band(11.8), 'warn')
        self.assertEqual(data.battery_band(11.7), 'warn')
        self.assertEqual(data.battery_band(11.5), 'crit')
        self.assertEqual(data.battery_band(9.0), 'crit')

    def test_thresholds_are_configurable(self):
        self.assertEqual(data.battery_band(11.0, warn_v=10.8), 'ok')
        self.assertEqual(data.battery_band(10.5, warn_v=10.8), 'warn')

    def test_bad_values_are_unknown(self):
        for value in (None, '', 'abc', float('nan'), 0, -3):
            self.assertEqual(data.battery_band(value), data.BATTERY_UNKNOWN)


class TestPoseAge(unittest.TestCase):
    """时间戳下标写错过两次（取成 yaw），单独钉住。"""

    def test_uses_timestamp_not_yaw(self):
        # yaw=0.0 时若误取 pose[2]，"年龄"会是约 1.8e9 秒
        self.assertAlmostEqual(data.pose_age((1.0, 2.0, 0.0, 1000.0), 1000.5),
                               0.5, places=6)

    def test_yaw_does_not_affect_age(self):
        self.assertAlmostEqual(data.pose_age((1.0, 2.0, 3.0, 1000.0), 1002.0),
                               2.0, places=6)

    def test_missing_pose(self):
        self.assertIsNone(data.pose_age(None))
        self.assertIsNone(data.pose_age(()))
        self.assertIsNone(data.pose_age((1.0, 2.0)))

    def test_clock_skew_never_negative(self):
        # 时间戳略超前（两机时钟不同步）不能返回负数，否则新鲜度判断会反着来
        self.assertEqual(data.pose_age((0.0, 0.0, 0.0, 1010.0), 1000.0), 0.0)


class TestNormalizeStatus(unittest.TestCase):
    def test_empty(self):
        out = data.normalize_status('')
        self.assertFalse(out['connected'])
        self.assertIsNone(out['state'])
        self.assertFalse(out['running'])
        self.assertEqual(out['battery_band'], 'unknown')

    def test_broken_json(self):
        out = data.normalize_status('{"state": ')
        self.assertFalse(out['connected'])

    def test_full(self):
        payload = json.dumps({
            'state': 'RUNNING', 'route_name': 'r1', 'point_id': 'p2',
            'current_index': 1, 'completed_points': 3, 'current_loop': 2,
            'execution_id': 'abc', 'message': 'ok', 'battery_v': 11.6,
            'time': 1000.0,
        })
        out = data.normalize_status(payload, now=1002.5)
        self.assertTrue(out['connected'])
        self.assertTrue(out['running'])
        self.assertEqual(out['point_id'], 'p2')
        self.assertEqual(out['completed_points'], 3)
        self.assertEqual(out['battery_band'], 'warn')
        self.assertAlmostEqual(out['age_s'], 2.5)

    def test_terminal_state(self):
        out = data.normalize_status(json.dumps({'state': 'SUCCEEDED'}))
        self.assertTrue(out['terminal'])
        self.assertFalse(out['running'])

    def test_battery_alternate_key_and_dict_input(self):
        out = data.normalize_status({'state': 'IDLE', 'battery_voltage': 12.9})
        self.assertEqual(out['battery_v'], 12.9)
        self.assertEqual(out['battery_band'], 'ok')

    def test_garbage_types(self):
        out = data.normalize_status({'current_index': 'x',
                                     'completed_points': None,
                                     'battery_v': 'nope'})
        self.assertEqual(out['current_index'], 0)
        self.assertEqual(out['completed_points'], 0)
        self.assertIsNone(out['battery_v'])


class TestLogRing(unittest.TestCase):
    def test_capacity_and_order(self):
        ring = data.LogRing(capacity=3)
        for index in range(5):
            ring.add('n', 20, 'line%d' % index)
        items = ring.tail()
        self.assertEqual([i['text'] for i in items],
                         ['line2', 'line3', 'line4'])

    def test_counts(self):
        ring = data.LogRing()
        ring.add('a', 20, 'i')
        ring.add('a', 30, 'w')
        ring.add('a', 40, 'e')
        ring.add('a', 'error', 'e2')
        counts = ring.counts()
        self.assertEqual(counts['INFO'], 1)
        self.assertEqual(counts['WARN'], 1)
        self.assertEqual(counts['ERROR'], 2)

    def test_level_filter(self):
        ring = data.LogRing()
        ring.add('a', 20, 'i')
        ring.add('a', 30, 'w')
        ring.add('a', 40, 'e')
        self.assertEqual([i['text'] for i in ring.tail(min_level='WARN')],
                         ['w', 'e'])
        self.assertEqual(ring.tail(min_level='WARN', limit=1)[0]['text'], 'e')

    def test_unknown_level_falls_back_to_info(self):
        ring = data.LogRing()
        item = ring.add('a', 'nonsense', 'x')
        self.assertEqual(item['level'], 'INFO')


class TestReports(unittest.TestCase):
    def test_summarize_run(self):
        row = data.summarize_run({
            'execution_id': 'e1', 'route_name': 'r', 'mode': 1,
            'started_at': 100.0, 'duration_sec': 61.0,
            'points': {'SUCCEEDED': 3, 'SKIPPED': 1, 'FAILED': 0},
            'verdicts': {'present': 2, 'absent': 1},
            'detections_total': 7, 'recall': 0.5, 'precision': 0.8,
        })
        self.assertEqual(row['total'], 4)
        self.assertEqual(row['ok'], 3)
        self.assertEqual(row['graded_points'], 3)
        self.assertAlmostEqual(row['recall'], 0.5)

    def test_summarize_missing(self):
        row = data.summarize_run({})
        self.assertEqual(row['total'], 0)
        self.assertIsNone(row['recall'])

    def test_list_runs_sorted_and_tolerates_bad_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, 'a.summary.json'), 'w',
                      encoding='utf-8') as handle:
                json.dump({'execution_id': 'a'}, handle)
            with open(os.path.join(tmp, 'b.summary.json'), 'w',
                      encoding='utf-8') as handle:
                handle.write('{broken')
            with open(os.path.join(tmp, 'ignore.txt'), 'w',
                      encoding='utf-8') as handle:
                handle.write('x')
            rows = data.list_runs(tmp)
            self.assertEqual(len(rows), 2)
            self.assertEqual(data.list_runs('/definitely/missing'), [])

    def test_tail_events_skips_bad_lines(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'r.jsonl')
            with open(path, 'w', encoding='utf-8') as handle:
                handle.write('{"type":"loop","loop":0}\n')
                handle.write('not json\n')
                handle.write('[1,2,3]\n')
                handle.write('{"type":"point","point_id":"p1"}\n')
            events = data.tail_events(path, 10)
            self.assertEqual(len(events), 2)
            self.assertEqual(events[-1]['point_id'], 'p1')
        self.assertEqual(data.tail_events('/nope.jsonl'), [])

    def test_formatters(self):
        self.assertEqual(data.fmt_duration(0), '0s')
        self.assertEqual(data.fmt_duration(59), '59s')
        self.assertEqual(data.fmt_duration(61), '1m01s')
        self.assertEqual(data.fmt_duration(3661), '1h01m')
        self.assertEqual(data.fmt_duration(None), '—')
        self.assertEqual(data.fmt_rate(0.5), '50.0%')
        self.assertEqual(data.fmt_rate(None), '—')

    def test_event_line(self):
        line = data.event_line({'type': 'point', 'point_id': 'p1',
                                'status': 'SUCCEEDED', 'message': 'done'})
        self.assertIn('p1', line)
        self.assertIn('done', line)
        self.assertEqual(data.event_line(None), '')


class TeleopStepCase(unittest.TestCase):
    """空闲必须不发——否则控制台一开就多一个 10 Hz 的零速发布者，车会抖。"""

    def test_idle_sends_nothing(self):
        send, vx, wz, nvx, nwz = data.teleop_step(0.0, 0.0, 100.0, 100.1, 0.6)
        self.assertFalse(send)
        self.assertEqual((vx, wz, nvx, nwz), (0.0, 0.0, 0.0, 0.0))

    def test_holding_sends_command(self):
        send, vx, wz, nvx, nwz = data.teleop_step(0.2, 0.0, 100.0, 100.1, 0.6)
        self.assertTrue(send)
        self.assertEqual((vx, wz), (0.2, 0.0))

    def test_release_sends_one_zero_frame(self):
        send, vx, wz, nvx, nwz = data.teleop_step(0.2, 0.0, 100.0, 100.7, 0.6)
        self.assertTrue(send)
        self.assertEqual((vx, wz), (0.0, 0.0))
        self.assertEqual((nvx, nwz), (0.0, 0.0))
        # 下一拍就彻底安静
        send2, _, _, _, _ = data.teleop_step(nvx, nwz, 100.0, 100.8, 0.6)
        self.assertFalse(send2)

    def test_timeout_boundary_is_not_stale(self):
        send, _, _, _, _ = data.teleop_step(0.2, 0.0, 100.0, 100.6, 0.6)
        self.assertTrue(send)

    def test_turn_only_counts_as_moving(self):
        send, vx, wz, _, _ = data.teleop_step(0.0, 0.5, 100.0, 100.1, 0.6)
        self.assertTrue(send)
        self.assertEqual(wz, 0.5)

    def test_bad_input_never_publishes(self):
        send, vx, wz, _, _ = data.teleop_step('x', None, 'y', 'z', 0.6)
        self.assertFalse(send)
        self.assertEqual((vx, wz), (0.0, 0.0))


class PublicHostCase(unittest.TestCase):
    """日志里要打印能点开的地址，不能打印 0.0.0.0。"""

    def test_param_wins(self):
        self.assertEqual(
            data.public_host('192.168.1.103', '0.0.0.0', detect=lambda: '1.1.1.1'),
            '192.168.1.103')

    def test_bind_used_when_specific_and_param_empty(self):
        self.assertEqual(
            data.public_host('', '10.0.0.5', detect=lambda: '1.1.1.1'),
            '10.0.0.5')

    def test_detect_used_when_bound_to_all(self):
        self.assertEqual(
            data.public_host('', '0.0.0.0', detect=lambda: '192.168.202.128'),
            '192.168.202.128')

    def test_falls_back_to_loopback_when_detect_fails(self):
        self.assertEqual(
            data.public_host('', '0.0.0.0', detect=lambda: ''),
            '0.0.0.0')

    def test_detect_lan_ip_never_raises(self):
        value = data.detect_lan_ip(timeout=0.05)
        self.assertIsInstance(value, str)


class ShortNodeNameCase(unittest.TestCase):
    """探针要用短名比对，否则永远匹配不上 rclpy 给的带命名空间的名字。"""

    def test_strips_namespace(self):
        self.assertEqual(data.short_node_name('/nav/controller_server'),
                         'controller_server')

    def test_strips_leading_slash_only(self):
        self.assertEqual(data.short_node_name('/amcl'), 'amcl')

    def test_short_name_unchanged(self):
        self.assertEqual(data.short_node_name('amcl'), 'amcl')

    def test_empty_is_empty(self):
        self.assertEqual(data.short_node_name(''), '')
        self.assertEqual(data.short_node_name(None), '')

    def test_trailing_slash(self):
        self.assertEqual(data.short_node_name('/amcl/'), 'amcl')

    def test_batch_drops_empty_and_dedupes(self):
        got = data.short_node_names(['/amcl', 'amcl', '', None, '/nav/planner_server'])
        self.assertEqual(got, {'amcl', 'planner_server'})

    def test_batch_of_none(self):
        self.assertEqual(data.short_node_names(None), set())


if __name__ == '__main__':
    unittest.main()
