"""方案 §5：协方差分档与退化方向的纯函数测试（不起 ROS）。"""
import math
import unittest

from mybot_slip import scan_match_odom as sm

CFG = {'r_good': 0.60, 'r_mid': 0.25, 'mid_scale': 3.0, 'degen_scale': 6.0,
       'degen_cond': 0.20, 'along_cos': 0.80, 'publish_mid': True}


class TestDecideCov(unittest.TestCase):

    def test_good(self):
        r = sm.decide_cov(0.70, 1.0, 0.0, CFG)
        self.assertEqual(r['tier'], 'good')
        self.assertEqual((r['sx'], r['sy'], r['sw']), (1.0, 1.0, 1.0))
        self.assertEqual(r['degen'], 'none')

    def test_mid_scales_all_axes(self):
        r = sm.decide_cov(0.30, 1.0, 0.0, CFG)
        self.assertEqual(r['tier'], 'mid')
        self.assertAlmostEqual(r['sx'], 3.0)
        self.assertAlmostEqual(r['sw'], 3.0)

    def test_poor_is_dropped(self):
        r = sm.decide_cov(0.10, 1.0, 0.0, CFG)
        self.assertIn('drop', r)

    def test_degenerate_along_x(self):
        r = sm.decide_cov(0.70, 0.05, 0.0, CFG)      # 弱方向贴 x 轴
        self.assertEqual(r['degen'], 'along')
        self.assertAlmostEqual(r['sx'], 6.0)
        self.assertAlmostEqual(r['sy'], 1.0)

    def test_degenerate_lateral(self):
        r = sm.decide_cov(0.70, 0.05, math.pi / 2, CFG)
        self.assertEqual(r['degen'], 'lateral')
        self.assertAlmostEqual(r['sy'], 6.0)

    def test_degenerate_unknown_is_dropped(self):
        r = sm.decide_cov(0.70, 0.05, math.pi / 4, CFG)   # 45°：认不出是哪一路
        self.assertIn('drop', r)

    def test_cond_missing_is_not_degenerate(self):
        r = sm.decide_cov(0.70, None, None, CFG)
        self.assertEqual(r['degen'], 'none')


if __name__ == '__main__':
    unittest.main()
