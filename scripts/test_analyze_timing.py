import unittest
from analyze_timing import interpolate


class TimingTests(unittest.TestCase):
    def test_constant_velocity_irregular_samples(self):
        self.assertEqual(interpolate([0,.2,.3],[[0,0,0],[2,4,6],[3,6,9]],.1),[1,2,3])

    def test_no_extrapolation(self):
        for target in (-.1,.4):
            self.assertIsNone(interpolate([0,.2],[[0,0,0],[2,4,6]],target))

    def test_large_gap_rejected_but_exact_observation_retained(self):
        self.assertIsNone(interpolate([0,1],[[0,0,0],[1,1,1]],.5))
        self.assertEqual(interpolate([0,1],[[0,0,0],[1,1,1]],1),[1,1,1])


if __name__=='__main__':
    unittest.main()
