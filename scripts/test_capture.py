import json
from pathlib import Path
import tempfile
import threading
import unittest
from capture import SampleClock, Recorder


class CaptureTests(unittest.TestCase):
    def test_irregular_ticks_do_not_accumulate_schedule_drift(self):
        clock = SampleClock(.1)
        logged = [clock.due(i*.07) for i in range(101)]
        self.assertEqual(sum(v is not None for v in logged), 71)

    def test_missing_ticks_are_marked_not_fabricated(self):
        clock = SampleClock(.1)
        self.assertEqual(clock.due(0), (0, 0))
        self.assertIsNone(clock.due(.05))
        target, missed = clock.due(.35)
        self.assertAlmostEqual(target, .3)
        self.assertEqual(missed, 2)
        with self.assertRaises(ValueError):
            clock.due(.2)

    def test_close_drains_records_and_counts(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'raw.jsonl'
            rec = Recorder(path)
            for i in range(100):
                rec.emit('estimate', 0, {'time_boot_ms':i})
            rec.close()
            rows = [json.loads(s) for s in path.read_text().splitlines()]
            self.assertEqual(len(rows),100)
            self.assertEqual([r['seq'] for r in rows], list(range(100)))
            self.assertEqual(rec.stats()['dropped'],0)
            self.assertEqual(rec.stats()['written'],100)

    def test_overflow_is_counted_and_does_not_block_producer(self):
        gate=threading.Event()
        class PausedRecorder(Recorder):
            def _write(self):
                gate.wait(2)
                super()._write()
        with tempfile.TemporaryDirectory() as tmp:
            rec=PausedRecorder(Path(tmp)/'raw.jsonl',capacity=1)
            rec.emit('test',0,{'v':1})
            rec.emit('test',0,{'v':2})
            self.assertEqual(rec.stats()['dropped'],1)
            gate.set()
            rec.close()
            self.assertEqual(rec.stats()['written'],1)


if __name__=='__main__':
    unittest.main()
