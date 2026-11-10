#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ai.py：白名单必须真的拦住模型，坐标必须被夹进地图。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_console import ai  # noqa: E402

BOUNDS = {'min_x': -5.0, 'max_x': 5.0, 'min_y': -5.0, 'max_y': 5.0}


class TestValidateAction(unittest.TestCase):
    def test_unknown_type_rejected(self):
        action, reason = ai.validate_action({'type': 'self_destruct'})
        self.assertIsNone(action)
        self.assertIn('不支持', reason)

    def test_missing_type_rejected(self):
        self.assertIsNone(ai.validate_action({'x': 1})[0])
        self.assertIsNone(ai.validate_action('nav')[0])

    def test_nav_ok(self):
        action, reason = ai.validate_action(
            {'type': 'nav', 'x': 1.5, 'y': -2.0}, BOUNDS)
        self.assertIsNone(reason)
        self.assertEqual(action, {'type': 'nav', 'x': 1.5, 'y': -2.0})

    def test_nav_clamped(self):
        action, _ = ai.validate_action(
            {'type': 'nav', 'x': 999, 'y': -999}, BOUNDS)
        self.assertEqual(action['x'], 5.0)
        self.assertEqual(action['y'], -5.0)

    def test_nav_rewritten_not_rejected(self):
        # 夹取本身是"纠正"，但坐标完全没法解释时必须拒绝
        self.assertIsNone(ai.validate_action(
            {'type': 'nav', 'x': 'left', 'y': 1}, BOUNDS)[0])

    def test_nav_with_yaw(self):
        action, _ = ai.validate_action(
            {'type': 'nav', 'x': 0, 'y': 0, 'yaw': 1.57})
        self.assertAlmostEqual(action['yaw'], 1.57)

    def test_patrol_defaults(self):
        action, _ = ai.validate_action({'type': 'patrol'})
        self.assertEqual(action['route'], 'main')
        self.assertNotIn('mode', action)

    def test_patrol_clamped(self):
        action, _ = ai.validate_action(
            {'type': 'patrol', 'mode': 99, 'repeat': 999})
        self.assertEqual(action['mode'], 2)
        self.assertEqual(action['repeat'], 50)

    def test_patrol_route_truncated(self):
        action, _ = ai.validate_action({'type': 'patrol', 'route': 'x' * 100})
        self.assertEqual(len(action['route']), 32)

    def test_simple_actions(self):
        for kind in ('pause', 'resume', 'cancel_nav', 'snapshot'):
            action, reason = ai.validate_action({'type': kind})
            self.assertEqual(action, {'type': kind})
            self.assertIsNone(reason)

    def test_case_and_space_insensitive(self):
        action, _ = ai.validate_action({'type': '  NAV ', 'x': 0, 'y': 0})
        self.assertEqual(action['type'], 'nav')


class TestParseModelOutput(unittest.TestCase):
    def test_fenced_json(self):
        text = ('好的，去 (1.2, -0.5)。\n```json\n'
                '{"reply": "去这里", "action": {"type": "nav", "x": 1.2, "y": -0.5}}\n```')
        out = ai.parse_model_output(text, BOUNDS)
        self.assertTrue(out['parsed'])
        self.assertEqual(out['reply'], '去这里')
        self.assertEqual(out['action']['x'], 1.2)

    def test_bare_json(self):
        out = ai.parse_model_output(
            '{"reply": "暂停了", "action": {"type": "pause"}}')
        self.assertEqual(out['action'], {'type': 'pause'})

    def test_plain_text(self):
        out = ai.parse_model_output('现在电量 12.4V，一切正常。')
        self.assertFalse(out['parsed'])
        self.assertIsNone(out['action'])
        self.assertIn('12.4V', out['reply'])

    def test_invalid_action_keeps_reply(self):
        out = ai.parse_model_output(
            '{"reply": "好的", "action": {"type": "reboot"}}')
        self.assertIsNone(out['action'])
        self.assertEqual(out['reply'], '好的')
        self.assertIn('不支持', out['reason'])

    def test_broken_json_falls_back(self):
        out = ai.parse_model_output('{"reply": "x", "action": }')
        self.assertFalse(out['parsed'])

    def test_empty(self):
        out = ai.parse_model_output('')
        self.assertFalse(out['parsed'])
        self.assertIsNone(out['action'])

    def test_reply_missing_uses_raw_text(self):
        out = ai.parse_model_output('{"action": {"type": "pause"}}')
        self.assertIn('pause', out['reply'])


class TestRuleReply(unittest.TestCase):
    CONTEXT = {'state': 'RUNNING', 'battery_v': 11.6, 'battery_band': 'warn',
               'route_name': 'main', 'point_id': 'p3', 'completed_points': 2}

    def test_battery(self):
        out = ai.rule_reply('还有多少电', self.CONTEXT)
        self.assertIn('11.60', out['reply'])
        self.assertIn('偏低', out['reply'])
        self.assertIsNone(out['action'])

    def test_battery_missing(self):
        out = ai.rule_reply('电量', {})
        self.assertIn('还没有收到', out['reply'])

    def test_status(self):
        out = ai.rule_reply('现在什么状态', self.CONTEXT)
        self.assertIn('RUNNING', out['reply'])
        self.assertIn('p3', out['reply'])

    def test_pause_and_resume(self):
        self.assertEqual(ai.rule_reply('暂停', self.CONTEXT)['action'],
                         {'type': 'pause'})
        self.assertEqual(ai.rule_reply('恢复', self.CONTEXT)['action'],
                         {'type': 'resume'})

    def test_cancel_nav(self):
        self.assertEqual(ai.rule_reply('取消导航', self.CONTEXT)['action'],
                         {'type': 'cancel_nav'})

    def test_nav_from_text(self):
        out = ai.rule_reply('去 x=1.2 y=-0.5', self.CONTEXT)
        self.assertEqual(out['action'], {'type': 'nav', 'x': 1.2, 'y': -0.5})

    def test_nav_from_parens(self):
        out = ai.rule_reply('导航到 (3, 4)', self.CONTEXT)
        self.assertEqual(out['action'], {'type': 'nav', 'x': 3.0, 'y': 4.0})

    def test_unknown_question(self):
        out = ai.rule_reply('今天天气怎么样', self.CONTEXT)
        self.assertIsNone(out['action'])
        self.assertIn('没接大模型', out['reply'])

    def test_empty_question(self):
        out = ai.rule_reply('', {})
        self.assertIn('你可以问我', out['reply'])


class TestParseXY(unittest.TestCase):
    def test_variants(self):
        self.assertEqual(ai._parse_xy('x=1 y=2'), (1.0, 2.0))
        self.assertEqual(ai._parse_xy('x:1.5 y 2.5'), (1.5, 2.5))
        self.assertEqual(ai._parse_xy('(1,2)'), (1.0, 2.0))
        self.assertEqual(ai._parse_xy('（-1.5， 2）'), (-1.5, 2.0))
        self.assertIsNone(ai._parse_xy('没有坐标'))


class TestContextBlock(unittest.TestCase):
    def test_contains_key_lines(self):
        text = ai.context_block({
            'state': 'RUNNING', 'route_name': 'r', 'completed_points': 1,
            'battery_v': 12.0, 'battery_band': 'ok',
            'map': {'width': 10, 'height': 12, 'resolution': 0.05,
                    'min_x': 0, 'max_x': 0.5, 'min_y': 0, 'max_y': 0.6},
            'errors': 'amcl: lost',
        })
        self.assertIn('RUNNING', text)
        self.assertIn('12.00 V', text)
        self.assertIn('地图', text)
        self.assertIn('amcl: lost', text)

    def test_empty_context(self):
        self.assertIn('未知', ai.context_block({}))


if __name__ == '__main__':
    unittest.main()
