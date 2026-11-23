import csv
import json
from pathlib import Path
import tempfile
import unittest

from capture import tag_records, words_in_dump, write_results


def twr(session, round_number, tag, anchor):
    return (f"2026-11-23T01:00:00+09:00\tTWR3,{session},{round_number},{tag},{anchor},1,1,1,3528,"
            "-20000,-23000,-20100,-23100,00004000,00004080")


def attempt(round_number, tag, anchor, stages, start):
    return {"time": "2026-11-23T01:00:00+09:00", "start_s": start, "end_s": start + 0.1, "anchor": anchor,
            "tag": tag, "round": round_number, "stages": stages, "response_status": 0x20000, "final_status": 0,
            "response_rssi": -20000, "response_fp": -23000}


class CaptureTest(unittest.TestCase):
    def test_split_jlink_words(self):
        dump = "J-Link>20004000 = 55574233 00000001 \n20004008 = 00000001 00000000 \nJ-Link>"
        self.assertEqual(words_in_dump(dump, 0x20004000, 4), [0x55574233, 1, 1, 0])

    def test_two_tags_strict_join_and_incomplete_rounds(self):
        tag1, _ = tag_records([twr(7, 1, 1, a) for a in (1, 2, 3)] + [twr(7, 2, 1, 1), twr(8, 2, 1, 2)], 7, 1)
        tag2, _ = tag_records([twr(7, 1, 2, a) for a in (1, 2, 3)], 7, 2)
        records = {**tag1, **tag2}
        attempts, start = [], 0.0
        for round_number in (1, 2):
            for tag in (1, 2):
                for anchor in (1, 2, 3):
                    stages = 7 if (round_number, tag, anchor) in records else 1
                    attempts.append(attempt(round_number, tag, anchor, stages, start))
                    start += 0.2
        attempts.append(attempt(3, 1, 1, 1, start))
        anomalies = {"foreign_records": 0, "duplicate_records": 0, "invalid_records": 0}
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            write_results(output, 7, attempts, records, [1, 2], 3, anomalies)
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["per_tag"]["1"]["complete_rounds"], 2)
            self.assertEqual(summary["per_tag"]["1"]["successful_rounds"], 1)
            self.assertEqual(summary["per_tag"]["2"]["successful_rounds"], 1)
            self.assertEqual(summary["per_tag"]["2"]["max_consecutive_failed_rounds"], 1)
            self.assertEqual(summary["packets"]["1"]["3"]["poll"]["lost_after_tx"], 1)
            checks = summary["schedule_checks"]
            self.assertEqual(checks["overlapping_calls"], 0)
            self.assertEqual(checks["unrequested_tag_records"], 0)
            self.assertAlmostEqual(checks["mean_update_interval_s"]["1"], 1.2, places=6)
            with (output / "packets.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 13 * 3)
            self.assertEqual(rows[2]["distance_m"], "3.528")
            self.assertEqual(rows[9]["tag_id"], "2")
            with (output / "rounds.csv").open(newline="") as stream:
                round_rows = list(csv.DictReader(stream))
            self.assertEqual([(row["tag_id"], row["status"]) for row in round_rows],
                             [("1", "ok"), ("2", "ok"), ("1", "failed"), ("2", "failed"), ("1", "partial")])

    def test_schedule_anomalies_are_counted(self):
        lines = [twr(7, 1, 1, 1), twr(7, 1, 1, 1), twr(7, 1, 2, 1), twr(7, 5, 1, 2), "TWR3,7,1"]
        records, anomalies = tag_records(lines, 7, 1)
        self.assertEqual(anomalies, {"foreign_records": 1, "duplicate_records": 1, "invalid_records": 1})
        attempts = [attempt(1, 1, 1, 7 | 8, 0.0), attempt(1, 1, 2, 1, 0.05)]  # second call overlaps the first
        with tempfile.TemporaryDirectory() as directory:
            checks = write_results(Path(directory), 7, attempts, records, [1], 1, anomalies)["schedule_checks"]
        self.assertEqual(checks["other_tag_responses_at_anchor"], 1)
        self.assertEqual(checks["unrequested_tag_records"], 1)  # round 5 was never requested
        self.assertEqual(checks["overlapping_calls"], 1)
        self.assertEqual(checks["duplicate_records"], 1)


if __name__ == "__main__":
    unittest.main()
