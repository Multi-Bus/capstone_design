"""Bounded asynchronous diagnostic recording and drift-free sample scheduling."""
import json
import math
import queue
import threading
import time
from pathlib import Path


class SampleClock:
    def __init__(self, period=.1):
        if period <= 0:
            raise ValueError('positive period required')
        self.period, self.next, self.last = period, None, None

    def due(self, now):
        if self.last is not None and now < self.last-1e-9:
            raise ValueError('simulation clock moved backwards')
        self.last = now
        if self.next is None:
            self.next = now
        if now+1e-9 < self.next:
            return None
        skipped = max(0, math.floor((now-self.next+1e-9)/self.period))
        target = self.next+skipped*self.period
        self.next = target+self.period
        return target, skipped


class Recorder:
    def __init__(self, path, capacity=20000):
        self.path = Path(path)
        self.queue = queue.Queue(capacity)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.accepted = self.written = self.dropped = self.sequence = 0
        self.error = None
        self.closed = False
        self.stream = self.path.open('w',encoding='utf-8')
        self.thread = threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def emit(self, source, drone_id, payload, host=None):
        with self.lock:
            if self.closed:
                return
            row = dict(seq=self.sequence, source=source, drone_id=drone_id,
                       host_monotonic_s=time.monotonic() if host is None else host,
                       data=payload)
            self.sequence += 1
            try:
                self.queue.put_nowait(row)
                self.accepted += 1
            except queue.Full:
                self.dropped += 1

    def _write(self):
        flush_at = time.monotonic()+1
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    row = None
                if row is not None:
                    self.stream.write(json.dumps(row,ensure_ascii=False)+'\n')
                    self.written += 1
                if time.monotonic() >= flush_at:
                    self.stream.flush()
                    flush_at = time.monotonic()+1
        except Exception as exc:
            self.error = repr(exc)
        finally:
            self.stream.close()

    def stats(self):
        return dict(accepted=self.accepted,written=self.written,dropped=self.dropped,
                    error=self.error,pending=self.queue.qsize(),closed=self.closed,
                    writer_alive=self.thread.is_alive())

    def close(self):
        with self.lock:
            self.closed = True
        self.stop.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            self.error = 'recorder drain timeout'
        self.path.with_suffix('.summary.json').write_text(json.dumps(self.stats(),indent=2))
