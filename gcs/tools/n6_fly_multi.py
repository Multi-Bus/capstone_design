#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
n6_fly_multi.py — n6_fly.py 를 여러 대(최대 4대)에 동시에. 키 하나가 모든 기체에 간다.

  t → y      : 모든 기체가 준비됐을 때만 동시에 이륙 → 호버 → 착륙 (한 대라도 안 되면 막음)
  스페이스/엔터/k : 모든 기체 그 자리 LAND
  1 ~ 4      : 그 번호 기체만 LAND (--names 순서)
  x x        : 모든 기체 강제 DISARM (공중이면 추락!)
  q          : 종료 (시동 상태면 LAND 먼저)      h = 도움말

각 기체가 상승 기준에 도달하면 공통 호버 시간을 재고 함께 LAND를 요청한다.
번호 키는 해당 기체를 시험에서 빼고 바로 착륙시킨다. 남은 기체는 시험을 계속한다.

  python n6_fly_multi.py --names N6-DRONE-1,N6-DRONE-4 --alt 0.6 --hover 20
  python n6_fly_multi.py --names N6-DRONE-1,N6-DRONE-3,N6-DRONE-4,N6-DRONE-5 --no-arm
"""
from __future__ import annotations

import argparse
import asyncio
import datetime as _dt
import shutil
import signal
from pathlib import Path
from typing import Callable, List

import n6_fly
from n6_fly import BleLink, Console, KeyReader, N6Fly, HANGUL_KEYS, KILL_KEYS

MAX_DRONES = 4
HELP = (
    "키:  t = 전체 이륙 시험 (y 로 확인)     스페이스/엔터/k = 전체 킬 (즉시 LAND)\n"
    "     1~4 = 그 기체만 킬     x x (1초 안에 두 번) = 전체 비상 모터 정지 — 공중이면 추락!\n"
    "     q = 종료 (시동 상태면 LAND 먼저)     h = 도움말"
)
n6_fly.HELP = "연결 시작"   # 기체마다 찍는 단일 기체 도움말 대신


class StatusBoard:
    """Keep the last terminal rows for telemetry, in command-line order."""

    def __init__(self, tags):
        self.rows = dict.fromkeys(tags, "연결 대기")
        self.drawn = False

    @staticmethod
    def row(tag: str, text: str, width: int) -> str:
        line = f"[{tag}] {text}"
        if n6_fly._clip(line, width) == line:
            return line
        for label, short in ((" | ", " "), ("BLE OK", "BLE"), ("링크 ", ""),
                             ("시동 ", ""), ("EKF높이 ", "EKF"), ("거리센서 ", "RF"),
                             ("요 ", "Y"), ("피치 ", "P"), ("롤 ", "R")):
            line = line.replace(label, short)
        return n6_fly._clip(line, width)

    def redraw(self, event: str = "") -> None:
        output = ""
        if self.drawn:
            output = "\r"
            if len(self.rows) > 1:
                output += f"\x1b[{len(self.rows) - 1}A"
            output += "\x1b[J"
        if event:
            output += event + "\n"
        width = max(1, shutil.get_terminal_size().columns - 1)
        output += "\n".join(self.row(tag, text, width)
                            for tag, text in self.rows.items())
        print(output, end="", flush=True)
        self.drawn = True

    def finish(self) -> None:
        if self.drawn:
            print(flush=True)
            self.drawn = False


class TaggedConsole(Console):

    def __init__(self, log_path: Path, tag: str, board: StatusBoard):
        super().__init__(log_path)
        self.tag = tag
        self.board = board

    def event(self, text: str) -> None:
        message = f"[{self.tag}] {text}"
        if self._live:
            self._to_log(message)
            self.board.redraw(f"{self._stamp(False)} {message}")
        else:
            super().event(message)

    def status(self, text: str) -> None:
        self._to_log(f"[상태] {text}")
        if self._live:
            self.board.rows[self.tag] = text
            self.board.redraw()
        else:
            print(f"{self._stamp(False)} [{self.tag}] {text}", flush=True)


class GroupFlight:
    """One confirmed trial. Only the coordinator releases the flight tasks to LAND."""

    def __init__(self, flies: List[N6Fly]):
        self.hover_seconds = flies[0].a.hover
        self.members = set(flies)
        self.height_checks = {}
        self.land = asyncio.Event()
        self.tasks = {}
        for fly in flies:
            task = asyncio.create_task(fly._takeoff_test(self.hold))
            fly._test_task = self.tasks[fly] = task
        self.monitor = asyncio.create_task(self.run())

    async def hold(self, fly: N6Fly, ready_to_hover: Callable[[], bool]) -> None:
        fly._phase = "다른 기체 대기"
        self.height_checks[fly] = ready_to_hover
        await self.land.wait()

    def withdraw(self, fly: N6Fly) -> None:
        if fly in self.members:
            self.members.remove(fly)
            fly._cancel_test("수동 조작")

    async def abort(self, source: N6Fly, exc: Exception) -> None:
        reason = f"{source.link.name} 공통 시험 중단: {exc}"
        for member in self.members:
            member.c.event(reason)
            member._cancel_test(reason)
            if not member.link.connected or (member is source and getattr(exc, "link", False)):
                member._land_after_relink = True
        await asyncio.gather(*(member._send_mode_burst(n6_fly.MODE_LAND, "공통 시험 중단")
                               for member in self.members if member.link.connected))

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        deadline = None
        try:
            while self.members:
                ready = set()
                for fly in tuple(self.members):
                    task = self.tasks[fly]
                    try:
                        if task.cancelled():
                            if fly._cancel_reason == n6_fly.EMERGENCY_STOP_REASON:
                                # xx was broadcast. End the trial while the key handlers finish DISARM.
                                for member in tuple(self.members):
                                    self.withdraw(member)
                                return
                            raise n6_fly.TestFailed(fly._cancel_reason)
                        if task.done():
                            task.result()
                            self.members.remove(fly)
                        else:
                            fly._check_guards()
                            check = self.height_checks.get(fly)
                            if deadline is None and check is not None and check():
                                ready.add(fly)
                    except Exception as exc:
                        await self.abort(fly, exc)
                        return
                if not self.members:
                    break
                if deadline is None and self.members == ready:
                    deadline = loop.time() + self.hover_seconds
                    for fly in self.members:
                        fly._phase = "호버"
                        fly.c.event(f"4/6 전체 도달 — 공통 호버 {fly._hover_txt()}")
                if deadline is not None and loop.time() >= deadline:
                    self.land.set()
                await asyncio.sleep(0.05)
        finally:
            await asyncio.gather(*self.tasks.values(), return_exceptions=True)


async def amain(args: argparse.Namespace) -> None:
    stamp = f"{_dt.datetime.now():%Y%m%d_%H%M%S}"
    flies: List[N6Fly] = []
    queues: List["asyncio.Queue[str]"] = []
    tags = ["D" + name.rsplit("-", 1)[-1] for name in args.names]
    board = StatusBoard(tags)
    for name, tag in zip(args.names, tags):
        console = TaggedConsole(Path(args.log_dir) / f"n6_fly_{tag}_{stamp}.log", tag, board)
        fly = N6Fly(args, console, BleLink(name, console, args.scan_timeout))
        flies.append(fly)
        queues.append(asyncio.Queue())

    loop = asyncio.get_running_loop()
    keys_q: "asyncio.Queue[str]" = asyncio.Queue()
    keys = KeyReader(loop, keys_q)
    keys.start()
    try:
        loop.add_signal_handler(signal.SIGINT, keys_q.put_nowait, "CTRL_C")
    except (NotImplementedError, RuntimeError, AttributeError):
        pass
    say = flies[0].c.event
    runs = [loop.create_task(f.run(q)) for f, q in zip(flies, queues)]
    say(f"대상 {', '.join(args.names)} — 키 입력 {keys.mode}\n{HELP}")

    group = None

    async def dispatch() -> None:
        nonlocal group
        pending = False
        while True:
            key = await keys_q.get()
            key = HANGUL_KEYS.get(key, key)
            low = key.lower() if len(key) == 1 else key
            if low in KILL_KEYS or low in ("x", "q", "CTRL_C", "EOF"):
                pending = False
                if group and (low in KILL_KEYS or low in ("q", "CTRL_C")):
                    for fly in flies:
                        group.withdraw(fly)
                for q in queues:
                    q.put_nowait(low)
            elif low.isdigit() and 1 <= int(low) <= len(flies):
                pending = False
                if group:
                    group.withdraw(flies[int(low) - 1])
                queues[int(low) - 1].put_nowait("k")
            elif pending:
                pending = False
                if low != "y":
                    say("전체 이륙 시험 취소")
                elif (group is None or group.monitor.done()) and all(not f._test_preconditions() for f in flies):
                    if len(flies) == 1:
                        flies[0]._start_test()
                    else:
                        group = GroupFlight(flies)
                else:
                    say("이륙 시험 못 함: 그사이 준비가 풀린 기체가 있음 (t 부터 다시)")
            elif low == "t":
                blocked = [(f, f._test_preconditions()) for f in flies]
                blocked = [(f, why) for f, why in blocked if why]
                for f, why in blocked:
                    f.c.event("이륙 못 함: " + "; ".join(why))
                if not blocked:
                    pending = True
                    say(f"전체 {len(flies)}대 이륙 {args.alt:.2f} m / 호버 {args.hover:g} s — "
                        "프롭·간격·안전줄 확인했으면 y, 취소는 다른 키. (언제든 킬 = 스페이스)")
            elif low in ("h", "?"):
                say(HELP)

    disp = loop.create_task(dispatch())
    try:
        await asyncio.gather(*runs)
    finally:
        disp.cancel()
        try:
            if group:
                await group.monitor
        finally:
            keys.restore()
            board.finish()
            for f in flies:
                f.c.close()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="N6 드론 여러 대 동시 이륙 → 호버 → 착륙 (킬 = 스페이스)")
    p.add_argument("--names", required=True,
                   help="XIAO BLE 이름, 쉼표로 (예: N6-DRONE-1,N6-DRONE-4). 최대 4대")
    p.add_argument("--alt", type=float, default=0.6, help="이륙 높이 m (기본 0.6, 허용 0.3~2.0)")
    p.add_argument("--hover", type=float, default=10.0, help="호버 시간 s (기본 10, 허용 0~30)")
    p.add_argument("--no-arm", action="store_true", help="연결·텔레메트리만 (이륙 키 비활성)")
    p.add_argument("--scan-timeout", type=float, default=8.0, help="BLE 스캔 한 번의 시간 s")
    p.add_argument("--log-dir", default=str(Path(__file__).resolve().parent / "logs"),
                   help="로그 폴더 (기본 gcs/tools/logs, 기체마다 파일 하나)")
    a = p.parse_args()
    a.names = [n.strip() for n in a.names.split(",") if n.strip()]
    if not 1 <= len(a.names) <= MAX_DRONES:
        p.error(f"--names 는 1~{MAX_DRONES}대")
    if len(set(a.names)) != len(a.names):
        p.error("--names 에 같은 이름이 두 번 있음")
    if len({"D" + n.rsplit("-", 1)[-1] for n in a.names}) != len(a.names):
        p.error("--names 끝 번호가 겹침 (기체 표시가 같아짐)")
    if not 0.3 <= a.alt <= 2.0:
        p.error("--alt 는 0.3 ~ 2.0 m 범위로")
    if not 0.0 <= a.hover <= 30.0:
        p.error("--hover 는 0 ~ 30 s 범위로")
    return a


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
