import unittest
from analyze_raw import stream_stats, nearest, correlation


class RawTests(unittest.TestCase):
    def test_source_rate_does_not_use_host_rate(self):
        rows=[dict(data={'time_boot_ms':i*100},host_monotonic_s=i*.5) for i in range(11)]
        stats=stream_stats(rows,'time_boot_ms',.001)
        self.assertAlmostEqual(stats['rate_source_hz'],10)
        self.assertAlmostEqual(stats['source_to_host_rate'],.2)
        self.assertAlmostEqual(stats['interval_host_max'],.5)

    def test_duplicates_and_backwards_are_visible(self):
        rows=[dict(data={'t':t},host_monotonic_s=i) for i,t in enumerate([0,.1,.1,.05,.2])]
        stats=stream_stats(rows,'t')
        self.assertEqual(stats['duplicates'],1)
        self.assertEqual(stats['backwards'],1)

    def test_nearest_requires_close_sample(self):
        self.assertEqual(nearest([0,1],['a','b'],.9),'b')
        self.assertIsNone(nearest([0,1],['a','b'],.5))

    def test_constant_quality_is_not_a_correlation(self):
        self.assertIsNone(correlation([(1,0),(1,1),(1,2)]))
        self.assertAlmostEqual(correlation([(0,0),(1,2),(2,4)]),1)


if __name__=='__main__':
    unittest.main()
