"""期望值基线：跑完自动记，下次自动判。

关键不是"能存能读"，而是两条保护：
* 手写的期望值永远优先，基线不许覆盖它；
* 默认 ``fill`` 策略下，一次误检（该有的没看到）不会把基线改写成
  absent——否则以后每次都"正确"地漏检，召回率反而好看。
"""

import json
import os
import shutil
import tempfile
import unittest

import fishbot_patrol_v2.baseline as baseline


class LoadSaveCase(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)
        self.path = os.path.join(self.dir, 'expect.json')

    def test_missing_file_is_empty(self):
        data = baseline.load(self.path)
        self.assertEqual(data['points'], {})

    def test_broken_file_is_empty(self):
        with open(self.path, 'w', encoding='utf-8') as handle:
            handle.write('{not json')
        self.assertEqual(baseline.load(self.path)['points'], {})

    def test_roundtrip_leaves_no_temp(self):
        data = baseline.empty()
        baseline.learn(data, 'main', 'p1', True)
        baseline.save(data, self.path)
        again = baseline.load(self.path)
        self.assertEqual(baseline.lookup(again, 'main', 'p1'), 'present')
        leftovers = [n for n in os.listdir(self.dir) if n.startswith('.expect-')]
        self.assertEqual(leftovers, [], '原子写不该留下临时文件')

    def test_saved_file_is_utf8_json(self):
        data = baseline.empty()
        baseline.learn(data, '主路线', 'p1', False)
        baseline.save(data, self.path)
        with open(self.path, encoding='utf-8') as handle:
            raw = json.load(handle)
        self.assertIn('主路线/p1', raw['points'])


class ResolveCase(unittest.TestCase):

    def test_manual_wins(self):
        expect, source = baseline.resolve('present', 'absent')
        self.assertEqual((expect, source), ('present', 'manual'))

    def test_baseline_used_when_manual_empty(self):
        expect, source = baseline.resolve('', 'absent')
        self.assertEqual((expect, source), ('absent', 'baseline'))

    def test_nothing_anywhere(self):
        self.assertEqual(baseline.resolve('', ''), ('', ''))

    def test_unknown_manual_ignored(self):
        # 录点时传了奇怪的值（比如拼错的 'presentt'），不能被当成期望值
        expect, source = baseline.resolve('presentt', 'present')
        self.assertEqual((expect, source), ('present', 'baseline'))


class LearnCase(unittest.TestCase):

    def test_fill_does_not_overwrite(self):
        data = baseline.empty()
        self.assertTrue(baseline.learn(data, 'main', 'p1', True))
        self.assertFalse(baseline.learn(data, 'main', 'p1', False))
        self.assertEqual(baseline.lookup(data, 'main', 'p1'), 'present',
                         '默认策略下第二次的漏检不能改写已有基线')

    def test_update_overwrites(self):
        data = baseline.empty()
        baseline.learn(data, 'main', 'p1', True, policy=baseline.POLICY_UPDATE)
        changed = baseline.learn(data, 'main', 'p1', False,
                                 policy=baseline.POLICY_UPDATE)
        self.assertTrue(changed)
        self.assertEqual(baseline.lookup(data, 'main', 'p1'), 'absent')

    def test_samples_accumulate(self):
        data = baseline.empty()
        baseline.learn(data, 'main', 'p1', True, policy=baseline.POLICY_UPDATE)
        baseline.learn(data, 'main', 'p1', True, policy=baseline.POLICY_UPDATE)
        entry = data['points']['main/p1']
        self.assertEqual(entry['samples'], 2)

    def test_points_are_per_route(self):
        data = baseline.empty()
        baseline.learn(data, 'main', 'p1', True)
        baseline.learn(data, 'main2', 'p1', False)
        self.assertEqual(baseline.lookup(data, 'main', 'p1'), 'present')
        self.assertEqual(baseline.lookup(data, 'main2', 'p1'), 'absent')

    def test_source_run_recorded(self):
        data = baseline.empty()
        baseline.learn(data, 'main', 'p1', True, source_run='exec-7')
        self.assertEqual(data['points']['main/p1']['source_run'], 'exec-7')

    def test_describe_counts(self):
        data = baseline.empty()
        baseline.learn(data, 'main', 'p1', True)
        baseline.learn(data, 'main', 'p2', False)
        text = baseline.describe(data)
        self.assertIn('2 个点', text)
        self.assertIn('1 个应该有', text)


if __name__ == '__main__':
    unittest.main()
