#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
n6_fly_multi.py — n6_fly.py 를 여러 대(최대 4대)에 동시에. 키 하나가 모든 기체에 간다.

  t → y      : 모든 기체가 준비됐을 때만 동시에 이륙 → 호버 → 착륙 (한 대라도 안 되면 막음)
  스페이스/엔터/k : 모든 기체 그 자리 LAND
  1 ~ 4      : 그 번호 기체만 LAND (--names 순서)
  x x        : 모든 기체 강제 DISARM (공중이면 추락!)
  q          : 종료 (시동 상태면 LAND 먼저)      h = 도움말

기체마다 BLE 연결, 하트비트, GCS 페일세이프, 시험 순서, 로그 파일은 n6_fly.py 그대로 따로 돈다.

  python n6_fly_multi.py --names N6-DRONE-1,N6-DRONE-4 --alt 0.6 --hover 20
  python n6_fly_multi.py --names N6-DRONE-1,N6-DRONE-3,N6-DRONE-4,N6-DRONE-5 --no-arm
"""
from __future__ import annotations

import argparse
import asyncio
import datetime as _dt
import signal
from pathlib import Path
from typing import Dict, List

import n6_fly
from n6_fly import BleLink, Console, KeyReader, N6Fly, HANGUL_KEYS, KILL_KEYS, mode_name

MAX_DRONES = 4
HELP = (
    "키:  t = 전체 이륙 시험 (y 로 확인)     스페이스/엔터/k = 전체 킬 (즉시 LAND)\n"
    "     1~4 = 그 기체만 킬     x x (1초 안에 두 번) = 전체 비상 모터 정지 — 공중이면 추락!\n"
    "     q = 종료 (시동 상태면 LAND 먼저)     h = 도움말"
)
n6_fly.HELP = "연결 시작"   # 기체마다 찍는 단일 기체 도움말 대신


class TaggedConsole(Console):
    """이벤트 앞에 [D4] 를 붙이고, 상태줄은 모든 기체 요약을 한 줄로 보여 준다."""

    def __init__(self, log_path: Path, tag: str, board: Dict[str, str]):
        super().__init__(log_path)
        self.tag = tag
        self.board = board
        self.summary = lambda: ""

    def event(self, text: str) -> None:
        super().event(f"[{self.tag}] {text}")

    def status(self, text: str) -> None:
        self._to_log(f"[상태] {text}")          # 로그에는 자세한 상태
        self.board[self.tag] = self.summary()
        self._status = "  ‖  ".join(self.board.values())
        if self._live:
            print("\r\x1b[2K" + self._status_line(), end="", flush=True)


def summary(tag: str, fly: N6Fly) -> str:
    s = fly.s
    if not fly.link.connected:
        return f"{tag} BLE끊김"
    rf = f"{s.rf_m:.2f}m" if (s.rf_m is not None and fly._fresh(s.rf_t)) else "-m"
    v = f"{s.vbat:.1f}V" if (s.vbat is not None and fly._fresh(s.vbat_t)) else "-V"
    phase = f" {fly._phase}" if fly._phase else ""
    return f"{tag} {mode_name(s.custom_mode)} {'ON' if s.armed else 'off'} {rf} {v}{phase}"


async def amain(args: argparse.Namespace) -> None:
    stamp = f"{_dt.datetime.now():%Y%m%d_%H%M%S}"
    board: Dict[str, str] = {}
    flies: List[N6Fly] = []
    queues: List["asyncio.Queue[str]"] = []
    for name in args.names:
        tag = "D" + name.rsplit("-", 1)[-1]
        console = TaggedConsole(Path(args.log_dir) / f"n6_fly_{tag}_{stamp}.log", tag, board)
        fly = N6Fly(args, console, BleLink(name, console, args.scan_timeout))
        console.summary = lambda t=tag, f=fly: summary(t, f)
        board[tag] = f"{tag} 연결 대기"
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

    async def dispatch() -> None:
        pending = False
        while True:
            key = await keys_q.get()
            key = HANGUL_KEYS.get(key, key)
            low = key.lower() if len(key) == 1 else key
            if low in KILL_KEYS or low in ("x", "q", "CTRL_C", "EOF"):
                pending = False
                for q in queues:
                    q.put_nowait(low)
            elif low.isdigit() and 1 <= int(low) <= len(flies):
                pending = False
                queues[int(low) - 1].put_nowait("k")
            elif pending:
                pending = False
                if low != "y":
                    say("전체 이륙 시험 취소")
                elif all(not f._test_preconditions() for f in flies):
                    for f in flies:
                        f._start_test()
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
        keys.restore()
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
