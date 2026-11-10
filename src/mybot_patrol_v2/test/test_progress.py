"""progress.py 的单测：原子写、损坏文件容错、指纹校验。"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_patrol_v2 import progress  # noqa: E402


class ProgressTest(unittest.TestCase):
    """进度快照的读写与兼容性判断。"""

    def setUp(self):
        self.directory = tempfile.mkdtemp(prefix='patrol_progress_test_')
        self.path = os.path.join(self.directory, 'sub', 'progress.json')

    def tearDown(self):
        for root, _dirs, files in os.walk(self.directory, topdown=False):
            for name in files:
                os.unlink(os.path.join(root, name))
            os.rmdir(root)

    def test_save_and_load_round_trip(self):
        saved = progress.Progress(
            route_name='floor1', mode=1, repeat_count=2, loop=1, cursor=3,
            completed=['L0N0', 'L0N1'], skipped=['L1N2'])
        progress.save(self.path, saved)
        loaded = progress.load(self.path)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.route_name, 'floor1')
        self.assertEqual(loaded.loop, 1)
        self.assertEqual(loaded.cursor, 3)
        self.assertEqual(loaded.completed, ['L0N0', 'L0N1'])
        self.assertEqual(loaded.skipped, ['L1N2'])
        self.assertTrue(loaded.updated_at)

    def test_save_creates_missing_directories(self):
        progress.save(self.path, progress.Progress('r', 0, 1))
        self.assertTrue(os.path.exists(self.path))

    def test_missing_file_returns_none(self):
        self.assertIsNone(progress.load(self.path))

    def test_corrupt_file_returns_none(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w', encoding='utf-8') as stream:
            stream.write('{not json')
        self.assertIsNone(progress.load(self.path))

    def test_wrong_schema_returns_none(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w', encoding='utf-8') as stream:
            json.dump({'schema': 99, 'route_name': 'r', 'mode': 0,
                       'repeat_count': 1}, stream)
        self.assertIsNone(progress.load(self.path))

    def test_resumable_matches_fingerprint(self):
        saved = progress.Progress('floor1', 0, 1)
        self.assertTrue(progress.resumable(saved, 'floor1', 0, 1))
        self.assertFalse(progress.resumable(saved, 'floor2', 0, 1))
        self.assertFalse(progress.resumable(saved, 'floor1', 1, 1))
        self.assertFalse(progress.resumable(None, 'floor1', 0, 1))

    def test_save_is_atomic_leaves_no_temp_file(self):
        progress.save(self.path, progress.Progress('floor1', 0, 1))
        leftovers = [name for name in os.listdir(os.path.dirname(self.path))
                     if name.startswith('.patrol_progress.')]
        self.assertEqual(leftovers, [])

    def test_clear_removes_file_and_tolerates_missing(self):
        progress.save(self.path, progress.Progress('floor1', 0, 1))
        progress.clear(self.path)
        self.assertFalse(os.path.exists(self.path))
        progress.clear(self.path)   # 再删一次不应抛异常

    def test_clear_tolerates_unremovable_path(self):
        """目标删不掉（权限/被占用）也不能把巡检收尾搅黄。"""
        progress.clear(self.directory)   # 传目录：unlink 会抛 OSError
        progress.clear('')
        self.assertTrue(os.path.isdir(self.directory))

    def test_route_composition_changes_invalidate_resume(self):
        """同名路线改了点位组成，旧快照必须作废。"""
        saved = progress.Progress('floor1', 0, 1,
                                  points=['p1', 'p2', 'p3'])
        self.assertTrue(progress.resumable(saved, 'floor1', 0, 1,
                                           ['p1', 'p2', 'p3']))
        self.assertFalse(progress.resumable(saved, 'floor1', 0, 1,
                                            ['p1', 'p2']))       # 删了点
        self.assertFalse(progress.resumable(saved, 'floor1', 0, 1,
                                            ['p2', 'p1', 'p3']))  # 换了顺序

    def test_negative_cursor_snapshot_is_rejected(self):
        """负游标会让 plan 用负下标取到别的点，读的时候就该丢掉。"""
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w', encoding='utf-8') as stream:
            json.dump({'schema': progress.SCHEMA_VERSION, 'route_name': 'floor1',
                       'mode': 0, 'repeat_count': 1, 'loop': 0, 'cursor': -3,
                       'completed': [], 'skipped': [], 'order': [0, 1],
                       'points': ['p1', 'p2']}, stream)
        self.assertIsNone(progress.load(self.path))

    def test_save_reports_failure_instead_of_raising(self):
        """写不进去只返回 False，不能让整轮巡检以异常收场。"""
        self.assertFalse(
            progress.save(self.directory, progress.Progress('floor1', 0, 1)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
