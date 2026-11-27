"""Offline flight-sequence tests with a virtual clock and one simulated FC per link."""
import asyncio
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gcs" / "tools"))
import n6_fly as single
import n6_fly_multi as multi


class SimFly(single.N6Fly):
    def __init__(self, args, console, link):
        super().__init__(args, console, link)
        self.takeoff = None
        self.land_times = []
        self.nav_targets = []
        self.delay = 1
        self.online = True
        self.fs_checked = self.s.uplink_ok = True
        self.tick()

    def tick(self):
        now = asyncio.get_running_loop().time()
        if self.link.connected and self.online:
            self.s.last_rx = self.s.last_hb = self.s.rf_t = now
            if self.land_times and now >= self.land_times[0] + 0.5:
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

    async def start(self, delay=4, solo=False):
        args = SimpleNamespace(alt=1.0, hover=10, no_arm=False, names=["D-1", "D-6"],
                               log_dir=".", scan_timeout=1)

        def make_fly(*params):
            fly = SimFly(*params)
            self.flies.append(fly)
            return fly

        def key_reader(loop, queue):
            self.key = queue.put_nowait
            return Mock()

        if solo:
            fly = make_fly(args, Mock(), SimpleNamespace(name="D-6", connected=True))
            fly.delay = delay
            await fly._handle_key("t")
            await fly._handle_key("y")
            return
        self.enterContext(patch.object(multi, "KeyReader", key_reader))
        self.enterContext(patch.object(multi, "TaggedConsole", Mock()))
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


if __name__ == "__main__":
    unittest.main()
