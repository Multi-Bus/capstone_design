"""Offline flight-sequence tests with a virtual clock and one simulated FC per link."""
import asyncio
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gcs" / "tools"))
import n6_fly as single
import n6_fly_multi as multi


class StatusDisplay(unittest.TestCase):
    def test_two_rows_across_flights_and_events(self):
        with tempfile.TemporaryDirectory() as tmp, patch("sys.stdout", new_callable=io.StringIO) as out:
            board = multi.StatusBoard(["D3", "D6"])
            consoles = [multi.TaggedConsole(Path(tmp) / tag, tag, board) for tag in board.rows]
            for console in consoles:
                console._live = True
                self.addCleanup(console.close)
            with patch.object(multi.shutil, "get_terminal_size", return_value=os.terminal_size((80, 24))):
                for phase in ("대기", "이륙", "착륙", "재이륙"):
                    for console in reversed(consoles):
                        console.status(f"EKF높이 +0.90m | 요 20° 피치 +1° 롤 +2° | {phase}")
                    out.seek(0)
                    out.truncate()
                    consoles[0].event(phase)
                    message, first, second = out.getvalue().splitlines()[-3:]
                    self.assertIn("\x1b[1A\x1b[J", message)
                    self.assertIn(f"[D3] {phase}", message)
                    self.assertTrue(first.startswith("[D3] EKF높이"))
                    self.assertTrue(second.startswith("[D6] EKF높이"))
                    self.assertTrue(first.endswith(phase) and second.endswith(phase))
                board.finish()
            for console in consoles:
                console.close()
                self.assertEqual(console.log_path.read_text("utf8").count("[상태]"), 4)


class SimFly(single.N6Fly):
    def __init__(self, args, console, link):
        super().__init__(args, console, link)
        self.takeoff = None
        self.land_times = []
        self.nav_targets = []
        self.force_disarms = 0
        self.auto_disarm = True
        self.delay = 1
        self.online = True
        self.fs_checked = self.s.uplink_ok = True
        self.tick()

    def tick(self):
        now = asyncio.get_running_loop().time()
        if self.link.connected and self.online:
            self.s.last_rx = self.s.last_hb = self.s.rf_t = now
            if self.auto_disarm and self.land_times and now >= self.land_times[0] + 0.5:
                self.s.armed = False
            self.s.rf_m = 0.02
            if self.takeoff and now >= self.takeoff + self.delay:
                self.s.rf_m += 1

    async def send(self, msg):
        if msg.get_type() != "COMMAND_LONG":
            return True
        now = asyncio.get_running_loop().time()
        if msg.command == single.mavlink2.MAV_CMD_NAV_TAKEOFF:
            self.takeoff = now
            self.nav_targets.append(msg.param7)
        elif msg.command == single.mavlink2.MAV_CMD_DO_SET_MODE and msg.param2 == single.MODE_LAND:
            self.land_times.append(now)
            self.s.custom_mode = single.MODE_LAND
        elif (msg.command == single.mavlink2.MAV_CMD_COMPONENT_ARM_DISARM
              and msg.param1 == 0 and msg.param2 == single.FORCE_DISARM_MAGIC):
            self.force_disarms += 1
            self.s.armed = False
        for waiter in self._ack_waiters.get(msg.command, []):
            if not waiter.done():
                waiter.set_result(single.ACCEPTED)
        return True

    async def run(self, queue):
        self._quit = asyncio.Event()
        while not self._quit.is_set():
            await self._handle_key(await queue.get())


class Flights(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.loop = asyncio.get_running_loop()
        self.loop.set_debug(False)
        self.now = self.loop.time()
        self.enterContext(patch.object(self.loop, "time", lambda: self.now))
        self.enterContext(patch.object(single, "time", SimpleNamespace(monotonic=self.loop.time)))
        self.flies = []

    async def advance(self, seconds):
        for _ in range(round(seconds * 100)):
            self.now += 0.01
            for fly in self.flies:
                fly.tick()
            await asyncio.sleep(0)

    async def start(self, delay=4, solo=False, floor=False):
        args = SimpleNamespace(alt=1.0, hover=10, no_arm=False, names=["D-1", "D-6"],
                               log_dir=".", scan_timeout=1)

        def make_fly(*params):
            fly = SimFly(*params)
            self.flies.append(fly)
            return fly

        def key_reader(loop, queue):
            self.key = queue.put_nowait
            self.keys = Mock()
            return self.keys

        if solo:
            fly = make_fly(args, Mock(), SimpleNamespace(name="D-6", connected=True))
            fly.delay = delay
            if floor:
                fly._floor_test = True
                fly.a.alt = 1.3
                fly.s.rf_max_m = 8.0
                fly.s.rf_floor_valid = True
            await fly._handle_key("t")
            await fly._handle_key("y")
            return
        self.enterContext(patch.object(multi, "KeyReader", key_reader))
        self.enterContext(patch.object(multi, "TaggedConsole", side_effect=lambda *_: Mock()))
        self.enterContext(patch.object(multi, "BleLink", lambda name, *_: SimpleNamespace(name=name, connected=True)))
        self.enterContext(patch.object(multi, "N6Fly", make_fly))
        self.enterContext(patch.object(self.loop, "add_signal_handler"))
        self.app = asyncio.create_task(multi.amain(args))
        await self.advance(0.05)
        self.flies[1].delay = delay
        self.key("t")
        self.key("y")

    async def finish(self):
        self.key("q")
        await self.advance(1)
        self.assertTrue(self.app.done())
        await self.app

    async def test_different_arrivals(self):
        await self.start()
        await self.advance(13)
        self.assertFalse(any(f.land_times for f in self.flies))
        await self.advance(2)
        first, last = self.flies
        self.assertEqual(first.land_times, last.land_times)
        self.assertEqual(len(first.land_times), 1)
        self.assertAlmostEqual(last.land_times[0] - last.takeoff, 14, delta=0.25)
        self.assertEqual([f.nav_targets for f in self.flies], [[1.0], [1.0]])
        await self.finish()

    async def test_arrival_timeout_aborts_both(self):
        await self.start(delay=9)
        await self.advance(8)
        self.assertFalse(any(f.land_times for f in self.flies))
        await self.advance(1)
        self.assertEqual([len(f.land_times) for f in self.flies], [3, 3])
        await self.finish()

    async def test_disconnect_while_waiting(self):
        await self.start()
        await self.advance(2)
        peer, lost = self.flies
        lost.link.connected = False
        lost._on_link_state(False)
        await self.advance(1)
        self.assertEqual(len(peer.land_times), 3)
        self.assertEqual(lost.land_times, [])
        self.assertTrue(lost._land_after_relink)
        await self.finish()

    async def test_stale_fc_during_hover(self):
        await self.start()
        await self.advance(5)
        self.flies[0].online = False
        await self.advance(single.FC_STALE_S + 1)
        self.assertEqual([len(f.land_times) for f in self.flies], [3, 3])
        await self.finish()

    async def test_unexpected_mode_during_hover(self):
        await self.start()
        await self.advance(5)
        self.flies[0].s.custom_mode = 0
        await self.advance(1)
        self.assertEqual([len(f.land_times) for f in self.flies], [3, 3])
        await self.finish()

    async def test_manual_all(self):
        await self.start()
        await self.advance(2)
        self.key(" ")
        await self.advance(15)
        self.assertEqual([len(f.land_times) for f in self.flies], [3, 3])
        await self.finish()

    async def test_manual_individual(self):
        await self.start()
        await self.advance(2)
        self.key("2")
        await self.advance(1)
        peer, selected = self.flies
        self.assertFalse(peer.land_times)
        self.assertEqual(len(selected.land_times), 3)
        await self.advance(10)
        self.assertEqual(len(peer.land_times), 1)
        await self.finish()

    async def test_solo_timeout_still_hovers(self):
        await self.start(delay=20, solo=True)
        await self.advance(18)
        fly, = self.flies
        self.assertFalse(fly.land_times)
        await self.advance(1)
        await fly._test_task
        self.assertAlmostEqual(fly.land_times[0] - fly.takeoff, 18, delta=0.25)

    async def test_floor_reference_and_normal_landing(self):
        await self.start(delay=1, solo=True, floor=True)
        fly, = self.flies
        tick = fly.tick
        def floor_samples():
            tick()
            # Absolute range passes at 1.21 m; rise from the 0.15 m baseline is only 1.06 m.
            fly.s.rf_m = 1.21 if fly.s.rf_m > 1 else 0.15
            fly.s.lpn_z, fly.s.lpn_t = -0.86, self.now
        fly.tick = floor_samples
        await self.advance(13)
        await fly._test_task
        self.assertEqual(fly.nav_targets, [1.3])
        self.assertEqual(len(fly.land_times), 1)
        self.assertAlmostEqual(fly.land_times[0] - fly.takeoff, 11, delta=0.25)
        self.assertFalse(fly.s.armed)

    async def test_floor_rejects_ekf_only_arrival(self):
        await self.start(solo=True, floor=True)
        fly, = self.flies
        await self.advance(1)
        tick = fly.tick
        def low_range():
            tick()
            fly.s.lpn_z, fly.s.lpn_t = -1.30, self.now
            fly.s.rf_m = 0.75
        fly.tick = low_range
        await self.advance(9)
        self.assertEqual(len(fly.land_times), 3)
        self.assertFalse(any("4/6" in str(call) for call in fly.c.event.call_args_list))

    async def test_floor_stale_during_hover_requests_land(self):
        await self.start(solo=True, floor=True)
        fly, = self.flies
        await self.advance(1)
        def sample():
            fly.s.rf_m, fly.s.rf_t, fly.s.last_rx = 1.30, self.now, self.now
        fly.tick = sample
        await self.advance(1)
        self.assertEqual(fly._phase, "호버")
        fly.tick = lambda: setattr(fly.s, "last_rx", self.now)
        await self.advance(1)
        self.assertEqual(len(fly.land_times), 3)

    async def test_floor_rejects_invalid_range_frames(self):
        await self.start(solo=True, floor=True)
        fly, = self.flies
        for stamp, (distance, quality) in enumerate([(0, 0), (65535, 0), (801, 0), (130, 1)]):
            msg = single.mavlink2.MAVLink_distance_sensor_message(
                stamp, 1, 800, distance, 0, 0, single.RANGEFINDER_DOWN, 0, signal_quality=quality)
            msg._header.srcSystem = single.FC_SYSID
            msg._header.srcComponent = single.FC_COMPID
            fly._handle(msg, self.now)
            self.assertIsNone(fly._floor_height())
        fly._cancel_test("test finished")
        await asyncio.gather(fly._test_task, return_exceptions=True)

    async def test_double_x_during_group_hover(self):
        await self.start()
        await self.advance(5)
        self.key("x")
        await self.advance(0.2)
        self.assertTrue(all(f._phase == "호버" and not f.force_disarms for f in self.flies))
        self.key("x")
        await self.advance(12)
        await self.finish()
        self.assertEqual([f.force_disarms for f in self.flies], [3, 3])
        self.assertEqual([f.land_times for f in self.flies], [[], []])

    async def test_first_drone_descends_then_recovers(self):
        await self.start()
        await self.advance(2)
        first, last = self.flies
        first.delay = 6
        await self.advance(3)
        phases = [f._phase for f in self.flies]
        await self.advance(12)
        await self.finish()
        self.assertNotIn("호버", phases)
        self.assertEqual(first.land_times, last.land_times)
        self.assertAlmostEqual(first.land_times[0] - first.takeoff, 16, delta=0.25)

    async def test_first_drone_stays_below_threshold(self):
        await self.start()
        await self.advance(2)
        self.flies[0].delay = 20
        await self.advance(7)
        counts = [len(f.land_times) for f in self.flies]
        await self.finish()
        self.assertEqual(counts, [3, 3])

    async def test_disarm_timeout_reported_to_group(self):
        await self.start()
        await self.advance(2)
        stuck = self.flies[1]
        stuck.auto_disarm = False
        await self.advance(44)
        await self.finish()
        self.assertIsInstance(stuck._test_task.exception(), single.TestFailed)
        self.assertIn("30초", str(stuck._test_task.exception()))
        self.assertTrue(any("공통 시험 중단" in str(c) for c in stuck.c.event.call_args_list))

    async def test_monitor_error_restores_keyboard_and_closes_logs(self):
        async def broken_monitor(group):
            await asyncio.gather(*group.tasks.values(), return_exceptions=True)
            raise RuntimeError("monitor error")

        self.enterContext(patch.object(multi.GroupFlight, "run", broken_monitor))
        await self.start()
        await self.advance(2)
        with self.assertRaisesRegex(RuntimeError, "monitor error"):
            await self.finish()
        self.keys.restore.assert_called_once()
        for fly in self.flies:
            fly.c.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
