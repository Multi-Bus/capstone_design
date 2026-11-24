from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import random
import tempfile
import unittest

from clock_sync import CYC_WRAP, DW_HZ, DW_WRAP, analyze

CPU_HZ = 600_000_000
SESSION = 99
# tag: (MCU error, DW3000 error) as fractions of true time
CLOCKS = {1: (0.0, 2e-6), 2: (150e-6, -5e-6), 3: (-2000e-6, 10e-6)}
PROPAGATION_NS = {1: (10, 20, 30), 2: (25, 5, 40), 3: (33, 12, 8)}  # per tag, anchors 1..3


def write_logs(folder, seed=1):
    rng = random.Random(seed)
    start = datetime(2026, 11, 23, 12, 0, tzinfo=timezone(timedelta(hours=9)))
    logs = {tag: [f"{start.isoformat()}\tTWR3_READY,channel=5,tag={tag},cpu_hz={CPU_HZ}"] for tag in CLOCKS}
    t = 0.0
    for round_number in range(1, 101):
        for polled_tag in (1, 2, 3):
            for anchor in (1, 2, 3):
                t += 0.12 + rng.uniform(0, 0.05)  # serialized J-Link calls
                for tag, (mcu_error, dw_error) in CLOCKS.items():
                    arrival = t + PROPAGATION_NS[tag][anchor - 1] * 1e-9
                    rx = round((1 + dw_error) * arrival * DW_HZ + 1e9 + rng.gauss(0, 6)) % DW_WRAP
                    cycles = round((1 + mcu_error) * (arrival + rng.uniform(1e-6, 4e-6)) * CPU_HZ + 12345) % CYC_WRAP
                    host = start + timedelta(seconds=t + 0.005 + rng.uniform(0, 0.04))
                    logs[tag].append(f"{host.isoformat(timespec='milliseconds')}\tHEAR3,{SESSION},{round_number},"
                                     f"{polled_tag},{anchor},{rx:010X},{cycles:08X},{-150 * tag}")
    for tag, lines in logs.items():
        (folder / f"tag{tag}.raw.log").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (folder / "run.json").write_text(json.dumps({"session": SESSION}), encoding="utf-8")
    return t


def expected_ppm(a, b):
    return ((1 + b) / (1 + a) - 1) * 1e6


class ClockSyncTest(unittest.TestCase):
    def test_recovers_drift_through_counter_wraps(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            span = write_logs(folder)
            self.assertGreater(span, 3 * CYC_WRAP / CPU_HZ)  # MCU counter wraps several times
            self.assertGreater(span, DW_WRAP / DW_HZ)  # DW counter wraps too
            result = analyze(folder)
        tags = {row["tag"]: row for row in result["tags"]}
        for tag, (mcu_error, dw_error) in CLOCKS.items():
            self.assertEqual(tags[tag]["events"], 900)
            self.assertAlmostEqual(tags[tag]["mcu_vs_own_dw_ppm"], expected_ppm(dw_error, mcu_error), delta=0.1)
        offsets = {(row["tag"], row["anchor"]): row["dw_offset_ppm"] for row in result["anchor_offsets"]}
        self.assertAlmostEqual(offsets[(2, 1)], -300 * 1e6 / (1 << 26), places=6)
        pairs = {(row["tag_a"], row["tag_b"]): row for row in result["pairs"]}
        for (a, b), row in pairs.items():
            self.assertEqual(row["common_events"], 900)
            self.assertAlmostEqual(row["mcu_drift_ppm"], expected_ppm(CLOCKS[a][0], CLOCKS[b][0]), delta=0.1)
            self.assertAlmostEqual(row["dw_drift_ppm"], expected_ppm(CLOCKS[a][1], CLOCKS[b][1]), delta=0.001)
            self.assertLess(row["mcu_fit_rms_us"], 2.0)  # uniform 1..4 us polling latency on both sides
            self.assertLess(row["mcu_predict_rms_us"], 3.0)
            self.assertLess(row["dw_fit_rms_ns"], 0.5)  # per-anchor propagation removed, 6-tick noise left
            self.assertLess(row["dw_predict_rms_ns"], 1.0)


if __name__ == "__main__":
    unittest.main()
