#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
n6_fly.py — N6 드론: 노트북에서 BLE 로 0.8 m 이륙 -> 5 s 호버 -> 착륙 시험

경로
  노트북(이 스크립트: bleak + pymavlink)
    --BLE Nordic UART Service--> XIAO nRF52840
    --USART2 115200--> NUCLEO-N657X0-Q (투명 브리지)
    --UART7 921600--> FC (ArduPilot Copter 4.7.x, SERIAL4 = MAVLink2)

안전 설계
  * 연결돼 있는 동안 0.2 s 마다 지상국 HEARTBEAT (sysid 250) 를 보낸다.
    노트북/BLE/XIAO/NUCLEO 어디서 끊기든 FC 의 GCS 페일세이프
    (FS_GCS_ENABLE=5, FS_GCS_TIMEOUT=3) 가 3 s 뒤 LAND 한다.
  * 스페이스 / 엔터 / k = 킬: 그 자리 LAND (착지하면 FC 가 시동을 끔). 진행 중 시험 취소.
  * x 를 1 s 안에 두 번 = 비상 모터 정지 (강제 DISARM). 공중에서 쓰면 추락!
  * 시험 중 모드가 GUIDED 가 아니게 바뀌면 (페일세이프 LAND, 조종기 등) 즉시 시험 중단.
    절대 자동으로 GUIDED 로 되돌리지 않는다.

사용법
  python n6_fly.py                       # N6-DRONE-1 에 연결
  python n6_fly.py --no-arm              # 연결/텔레메트리만 (이륙 키 막힘)
  python n6_fly.py --name N6-DRONE-2 --alt 0.8 --hover 5
  python n6_fly.py --alt 1.3 --hold      # 1.3 m 로 떠서 스페이스(LAND) 누를 때까지 유지
  python n6_fly.py --yaw 90              # 호버 뒤 제자리에서 90° 씩 4번 돌기 (방위각/자세 측정)
  python n6_fly.py --alt 1.3 --move 0.3  # 호버 뒤 앞·뒤·오른쪽·왼쪽으로 0.3 m 씩 갔다가 돌아오기
  python n6_fly.py --alt 1.3 --manual    # 호버 뒤 방향키/wasd 로 0.2 m 씩 이동, r = 시작점, 스페이스 = 착륙

방위각/자세 기록
  ATTITUDE (롤, 피치, 요) 를 10 Hz 로 받아 로그 옆 *_att.csv 에 모두 적는다 (--no-arm 이어도).
  --yaw 를 주면 회전마다 목표 방위각 대비 오차와, 멈춰 있는 동안 요·피치·롤의 평균과 흔들림(표준편차)을 찍는다.

이동 기록
  --move 를 주면 이동마다 도달 시간, 정지 중 목표 대비 위치 오차(평균, 흔들림), 높이 변화를 찍는다.
  위치는 FC 의 LOCAL_POSITION_NED (지금은 광류 추정)라서 실제 이동 거리와 다를 수 있다.
"""
from __future__ import annotations

import argparse
import asyncio
import codecs
import datetime as _dt
import math
import os
import statistics
import shutil
import signal
import sys
import threading
import time
import unicodedata
from pathlib import Path
from typing import Awaitable, Callable, Dict, List, Optional

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover
    sys.exit("bleak 가 설치되지 않았습니다:  pip install -r requirements.txt")
try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2
except ImportError:  # pragma: no cover
    sys.exit("pymavlink 가 설치되지 않았습니다:  pip install -r requirements.txt")


# --------------------------------------------------------------------------
# 상수
# --------------------------------------------------------------------------
NUS_WRITE_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # 노트북 -> XIAO (쓰기)
NUS_NOTIFY_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # XIAO -> 노트북 (알림)

GCS_SYSID = 250                                   # = FC 의 MAV_GCS_SYSID
GCS_COMPID = mavlink2.MAV_COMP_ID_MISSIONPLANNER  # 190
FC_SYSID = 1
FC_COMPID = 1

MODE_GUIDED = 4
MODE_LAND = 9
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL", 28: "TURTLE",
}

HEARTBEAT_PERIOD_S = 0.2       # 5 Hz
FS_GCS_TIMEOUT_S = 3           # 기준값: FC 의 FS_GCS_TIMEOUT 과 같아야 함 (다르면 연결 시 경고)
FC_STALE_S = float(FS_GCS_TIMEOUT_S)  # FC 메시지가 이만큼 없으면 "끊김" 으로 본다
FRESH_S = 2.0                  # 텔레메트리 값이 이보다 오래되면 표시 안 함
# LAND 인데 기압계/거리센서 오차로 FC 가 착지를 감지 못해 시동이 안 꺼질 때 (3호기 2026-11-25):
# 센서와 상관없이 LAND + 시동 ON 이 이만큼 이어지면 강제 DISARM. 0.5 m 에서 LAND 는 3.5~5 초, 1.3 m 는 하강 2.6 초가 더 걸림.
# 링크가 살아 있을 때만 동작. BLE 가 끊긴 경우는 NUCLEO 브리지(firmware/nucleo/n6_bridge/FSBL/Inc/link_failsafe.h)가 맡음.
LAND_DISARM_S = 9.0
EKF_ORIGIN = (37.5665, 126.9780, 0.0)   # 임의 고정 좌표 (위도, 경도, 고도 m). GPS 없이 GUIDED 하려고 필요
FORCE_DISARM_MAGIC = 21196
LAND_REPEAT = 3                # 킬: LAND 명령을 3번
LAND_REPEAT_GAP_S = 0.1        #      0.1 s 간격
DOUBLE_X_WINDOW_S = 1.0
EMERGENCY_STOP_REASON = "비상 모터 정지"
STREAM_INTERVALS = (           # (메시지 ID, Hz)
    (mavlink2.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5),   # 32
    (mavlink2.MAVLINK_MSG_ID_DISTANCE_SENSOR, 5),      # 132
    (mavlink2.MAVLINK_MSG_ID_SYS_STATUS, 1),           # 1
    (mavlink2.MAVLINK_MSG_ID_ATTITUDE, 10),            # 30 (롤/피치/요 -> *_att.csv)
)
RANGEFINDER_DOWN = mavlink2.MAV_SENSOR_ROTATION_PITCH_270  # 25 (RNGFND1_ORIENT)
# Verified D6 floor-test configuration. Read-only checks before a trial.
FLOOR_PARAMS = {"AHRS_EKF_TYPE": 3, "EK3_SRC1_POSZ": 2, "EK3_SRC1_VELXY": 5,
                "WP_RFND_USE": 1, "GUID_OPTIONS": 0, "RNGFND1_ORIENT": RANGEFINDER_DOWN}
YAW_RATE_DPS = 30.0            # 제자리 회전 속도 (CONDITION_YAW param2)
YAW_REACHED_DEG = 5.0          # 목표 방위각과 이만큼 안이면 도달로 봄
MOVE_REACHED_M = 0.1           # 이동 목표와 수평 거리가 이만큼 안이면 도달로 봄
MOVE_TIMEOUT_S = 8.0           # 이동 하나의 도달 대기 시간
MOVE_RESEND_S = 1.0            # 도달 전까지 같은 위치 목표를 이 간격으로 다시 보냄 (ACK 가 없는 메시지라 손실 대비)
# 이동 순서: (이름, 앞 배수, 오른쪽 배수). 시작점 기준이라 "원위치" 는 매번 시작점으로 돌아간다.
MOVE_STEPS = (("앞", 1, 0), ("원위치", 0, 0), ("뒤", -1, 0), ("원위치", 0, 0),
              ("오른쪽", 0, 1), ("원위치", 0, 0), ("왼쪽", 0, -1), ("원위치", 0, 0))
# 위치 + 방위각만 쓰고 속도·가속도·회전 속도는 무시. 방위각을 비우면 WP_YAW_BEHAVIOR 때문에 진행 방향으로 돌 수 있다.
MOVE_TYPE_MASK = (mavlink2.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavlink2.POSITION_TARGET_TYPEMASK_VY_IGNORE
                  | mavlink2.POSITION_TARGET_TYPEMASK_VZ_IGNORE | mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE
                  | mavlink2.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavlink2.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                  | mavlink2.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

KILL_KEYS = (" ", "\r", "\n", "k")
# 한/영 키가 한글일 때 (두벌식) 같은 자리 글자도 받아 준다
HANGUL_KEYS = {"ㅅ": "t", "ㅌ": "x", "ㅂ": "q", "ㅏ": "k", "ㅛ": "y", "ㅗ": "h",
               "ㅈ": "w", "ㅁ": "a", "ㄴ": "s", "ㅇ": "d", "ㄱ": "r"}
# --manual 이동 키: (이름, 앞 칸, 오른쪽 칸). None 이면 시작점으로. 소문자만 받는다
# (리눅스 터미널 화살표는 ESC [ A 처럼 대문자가 오므로 a 로 잘못 읽히지 않게).
MANUAL_KEYS = {"UP": ("앞", 1, 0), "w": ("앞", 1, 0), "DOWN": ("뒤", -1, 0), "s": ("뒤", -1, 0),
               "LEFT": ("왼쪽", 0, -1), "a": ("왼쪽", 0, -1), "RIGHT": ("오른쪽", 0, 1), "d": ("오른쪽", 0, 1),
               "r": ("시작점", None, None)}
WIN_ARROWS = {"H": "UP", "P": "DOWN", "K": "LEFT", "M": "RIGHT"}
MANUAL_MAX_M = 3.0             # --manual: 시작점에서 이보다 먼 목표는 거부 (FC 추정 위치 기준)
MANUAL_SETTLE_S = 1.0          # --manual: 도달 뒤 이만큼 멈춘 다음에야 다음 키를 받음
MANUAL_ABORT_M = 0.5           # --manual: 도달 시간 안에 목표와 이보다 멀면 LAND

HELP = (
    "키:  t = 이륙 시험 (y 로 확인)     스페이스/엔터/k = 킬 (즉시 LAND)\n"
    "     x x (1초 안에 두 번) = 비상 모터 정지 — 공중이면 추락!\n"
    "     q = 종료 (시동 상태면 LAND 먼저)     h = 도움말\n"
    "     --manual 호버 중: 방향키/w a s d = 한 칸 이동, r = 시작점 (이동 중 누른 키는 무시)"
)

ACCEPTED = mavlink2.MAV_RESULT_ACCEPTED
IN_PROGRESS = getattr(mavlink2, "MAV_RESULT_IN_PROGRESS", 5)


def mode_name(mode: Optional[int]) -> str:
    if mode is None:
        return "?"
    return COPTER_MODES.get(mode, f"MODE{mode}")


def _enum_name(enum: str, value: int, prefix: str) -> str:
    entry = mavlink2.enums.get(enum, {}).get(value)
    return entry.name.replace(prefix, "") if entry is not None else str(value)


def cmd_name(cmd: int) -> str:
    return _enum_name("MAV_CMD", cmd, "MAV_CMD_")


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def result_name(res: Optional[int]) -> str:
    return "ACK 없음" if res is None else _enum_name("MAV_RESULT", res, "MAV_RESULT_")


class TestAborted(Exception):
    """외부 요인(모드 변경, 시동 꺼짐, 링크 끊김)으로 중단. 더 이상 아무 명령도 보내지 않는다."""

    def __init__(self, why: str, link: bool = False):
        super().__init__(why)
        self.link = link


class TestFailed(Exception):
    """우리 쪽 실패(거부, 시간 초과). 시동 상태 + GUIDED 이면 LAND 를 보낸다."""


# --------------------------------------------------------------------------
# 화면 + 로그 파일
# --------------------------------------------------------------------------
def _char_w(ch: str) -> int:
    return 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1


def _clip(text: str, width: int) -> str:
    out, w = [], 0
    for ch in text:
        cw = _char_w(ch)
        if w + cw > width:
            break
        out.append(ch)
        w += cw
    return "".join(out)


def _enable_ansi() -> bool:
    if os.name != "nt":
        return True
    try:  # Windows 10+ 콘솔에서 ANSI(줄 지우기) 켜기
        import ctypes

        k32 = ctypes.windll.kernel32
        handle = k32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if not k32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False
        return bool(k32.SetConsoleMode(handle, mode.value | 0x0004))
    except Exception:
        return False


class Console:
    """이벤트는 시간 붙여 한 줄씩, 상태줄은 맨 아래 한 줄을 덮어쓴다. 전부 로그 파일에도 기록."""

    def __init__(self, log_path: Path):
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path = log_path
        self._log = open(log_path, "a", encoding="utf-8", buffering=1)
        self._status = ""
        self._live = sys.stdout.isatty() and _enable_ansi()

    @staticmethod
    def _stamp(with_date: bool) -> str:
        now = _dt.datetime.now()
        fmt = "%Y-%m-%d %H:%M:%S" if with_date else "%H:%M:%S"
        return f"{now.strftime(fmt)}.{now.microsecond // 1000:03d}"

    def _to_log(self, text: str) -> None:
        try:
            self._log.write(f"{self._stamp(True)} {text}\n")
        except Exception:
            pass

    def _status_line(self) -> str:
        width = shutil.get_terminal_size((100, 20)).columns - 1
        return _clip(self._status, max(width, 20))

    def event(self, text: str) -> None:
        self._to_log(text)
        line = f"{self._stamp(False)} {text}"
        if self._live:
            sys.stdout.write("\r\x1b[2K" + line + "\n")
            if self._status:
                sys.stdout.write(self._status_line())
        else:
            sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def status(self, text: str) -> None:
        self._status = text
        self._to_log(f"[상태] {text}")
        if self._live:
            # 좁은 터미널에서는 전체 상태를 줄바꿈해 자세와 전압도 볼 수 있게 한다.
            sys.stdout.write("\r\x1b[2K" + text)
            if self._status_line() != text:
                sys.stdout.write("\n")
                self._status = ""
        else:
            sys.stdout.write(f"{self._stamp(False)} [상태] {text}\n")
        sys.stdout.flush()

    def close(self) -> None:
        if self._live and self._status:
            sys.stdout.write("\n")
            sys.stdout.flush()
        try:
            self._log.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# 키 입력 (별도 스레드 -> asyncio 큐)
# --------------------------------------------------------------------------
class KeyReader:
    def __init__(self, loop: asyncio.AbstractEventLoop, queue: "asyncio.Queue[str]"):
        self._loop = loop
        self._q = queue
        self._restore: Optional[Callable[[], None]] = None
        self.mode = ""

    def start(self) -> None:
        target = self._run_lines
        self.mode = "줄 입력 (글자 + 엔터, 빈 엔터 = 킬)"
        if os.name == "nt":
            try:
                import msvcrt  # noqa: F401

                target = self._run_windows
                self.mode = "한 글자 즉시 (msvcrt)"
            except ImportError:
                pass
        elif sys.stdin.isatty():
            try:
                import termios
                import tty

                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                tty.setcbreak(fd)  # 에코 끄고 한 글자씩. Ctrl-C 는 그대로 SIGINT
                self._restore = lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old)
                target = self._run_posix
                self.mode = "한 글자 즉시 (termios)"
            except Exception:
                pass
        threading.Thread(target=target, name="keys", daemon=True).start()

    def restore(self) -> None:
        if self._restore is not None:
            try:
                self._restore()
            except Exception:
                pass
            self._restore = None

    def _put(self, key: str) -> None:
        try:
            self._loop.call_soon_threadsafe(self._q.put_nowait, key)
        except RuntimeError:  # 루프가 이미 닫힘
            pass

    def _run_posix(self) -> None:
        fd = sys.stdin.fileno()
        dec = codecs.getincrementaldecoder("utf-8")("ignore")
        while True:
            try:
                b = os.read(fd, 1)
            except OSError:
                return
            if not b:
                self._put("EOF")
                return
            for ch in dec.decode(b):
                self._put(ch)

    def _run_windows(self) -> None:
        import msvcrt

        while True:
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # 기능키/화살표: 화살표만 이름으로, 나머지는 버림
                arrow = WIN_ARROWS.get(msvcrt.getwch())
                if arrow:
                    self._put(arrow)
                continue
            if ch == "\x03":
                self._put("CTRL_C")
                continue
            self._put(ch)

    def _run_lines(self) -> None:
        for line in sys.stdin:
            text = line.strip()
            if not text:
                self._put("\n")  # 빈 엔터 = 킬
            else:
                for ch in text:
                    self._put(ch)
        self._put("EOF")


# --------------------------------------------------------------------------
# BLE (Nordic UART Service)
# --------------------------------------------------------------------------
class BleLink:
    """이름으로 스캔 -> 연결 -> NUS 알림 구독. 끊기면 자동 재연결.

    on_bytes(bytes): XIAO 에서 온 바이트 (MAVLink 조각)
    on_state(bool) : 연결됨 / 끊김
    """

    def __init__(self, name: str, console: Console, scan_timeout: float = 8.0):
        self.name = name
        self.c = console
        self.scan_timeout = scan_timeout
        self.connected = False
        self.on_bytes: Callable[[bytes], None] = lambda data: None
        self.on_state: Callable[[bool], None] = lambda up: None
        self._client: Optional[BleakClient] = None
        self._char = None
        self._chunk = 20
        self._response = False
        self._lock = asyncio.Lock()
        self._lost: Optional[asyncio.Event] = None
        self._closing = False

    async def run(self) -> None:
        misses = 0
        while not self._closing:
            if misses % 5 == 0:
                self.c.event(f"BLE: '{self.name}' 찾는 중...")
            try:
                dev = await BleakScanner.find_device_by_name(self.name, timeout=self.scan_timeout)
            except Exception as e:  # 어댑터 꺼짐 등
                self.c.event(f"BLE 스캔 오류: {e!r} — 블루투스가 켜져 있는지 확인")
                misses += 1
                await asyncio.sleep(2.0)
                continue
            if dev is None:
                misses += 1
                continue
            misses = 0
            await self._session(dev)
            if not self._closing:
                await asyncio.sleep(1.0)

    async def _session(self, dev) -> None:
        # 끊김 이벤트는 "이 세션" 전용. 이전 BleakClient 의 끊김 콜백이 늦게 와도
        # 새 세션을 끊지 못하게 한다 (실기 시험에서 재연결 직후 바로 끊기던 문제).
        lost = asyncio.Event()
        self._lost = lost

        def on_disconnect(_client) -> None:
            if self._client is client:   # 현재 세션일 때만 쓰기를 즉시 막음
                self.connected = False
            lost.set()

        client = BleakClient(dev, disconnected_callback=on_disconnect, timeout=20.0)
        announced_up = False             # on_state(True) 를 알렸는지 (끊김 알림 여부는 이걸로 판단)
        try:
            self.c.event(f"BLE: {dev.address} 에 연결 중...")
            await client.connect()
            char = client.services.get_characteristic(NUS_WRITE_UUID)
            if char is None or client.services.get_characteristic(NUS_NOTIFY_UUID) is None:
                self.c.event("BLE: Nordic UART Service 가 없음 — XIAO 펌웨어 확인")
                return
            self._response = "write-without-response" not in char.properties
            self._chunk = await self._pick_chunk(char)
            self._client, self._char = client, char
            await client.start_notify(NUS_NOTIFY_UUID, self._on_notify)
            if lost.is_set() or not client.is_connected:
                self.c.event("BLE: 연결 설정 중에 끊김 — 다시 시도")
                return
            self.connected = True
            kind = "write-with-response" if self._response else "write-without-response"
            self.c.event(f"BLE 연결됨 ({dev.address}, 조각 {self._chunk} B, {kind})")
            announced_up = True
            self.on_state(True)
            await lost.wait()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.c.event(f"BLE 연결 실패: {e!r}")
        finally:
            self.connected = False
            self._client, self._char = None, None
            if announced_up:
                self.on_state(False)
            try:
                await asyncio.wait_for(client.disconnect(), 3.0)
            except BaseException:
                pass

    async def _pick_chunk(self, char) -> int:
        """MTU-3 (bleak 의 max_write_without_response_size). 모르면 20 B."""
        size = 20
        try:
            for _ in range(10):  # BlueZ 는 MTU 교환 직후 잠깐 20 을 돌려줄 수 있음
                size = int(char.max_write_without_response_size)
                if size > 20:
                    break
                await asyncio.sleep(0.1)
        except Exception:
            size = 20
        return max(20, min(size, 244))

    def _on_notify(self, _sender, data: bytearray) -> None:
        self.on_bytes(bytes(data))

    async def write(self, data: bytes) -> bool:
        """MAVLink 프레임 하나를 조각내어 쓴다.
        Lock 으로 묶어서 한 프레임의 조각 사이에 다른 프레임이 끼어들지 않게 한다."""
        async with self._lock:
            for attempt in (1, 2):
                client, char = self._client, self._char
                if not self.connected or client is None:
                    return False
                try:
                    for i in range(0, len(data), self._chunk):
                        await asyncio.wait_for(
                            client.write_gatt_char(char, data[i:i + self._chunk], response=self._response),
                            timeout=2.0,
                        )
                    return True
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    msg = str(e).lower()
                    # 어댑터가 꺼진 직후엔 bleak 의 is_connected 가 아직 True 인데 BlueZ 는
                    # "Not connected" 를 돌려준다 (실기 시험에서 확인) -> 오류 내용으로도 판단
                    if ("not connected" in msg or "disconnect" in msg
                            or not self._still_connected(client)):
                        # 링크가 끊겨서 실패한 것 -> 세션을 바로 끝내 "BLE 끊김" 처리 (끊김 콜백이 늦어도)
                        self._drop(client)
                        return False
                    if attempt == 1 and self._chunk > 20 and ("length" in msg or "size" in msg):
                        self.c.event(f"BLE 쓰기 실패 ({e!r}) — 조각을 20 B 로 줄여 다시 보냄")
                        self._chunk = 20
                        continue
                    self.c.event(f"BLE 쓰기 실패: {e!r}")
                    return False
        return False

    def _still_connected(self, client) -> bool:
        try:
            return bool(self.connected and self._client is client and client.is_connected)
        except Exception:
            return False

    def _drop(self, client) -> None:
        if self._client is client:
            self.connected = False
            if self._lost is not None:
                self._lost.set()

    async def close(self) -> None:
        self._closing = True
        client = self._client
        if client is not None:
            try:
                await asyncio.wait_for(client.disconnect(), 3.0)
            except BaseException:
                pass


# --------------------------------------------------------------------------
# FC 상태
# --------------------------------------------------------------------------
class FcState:
    def __init__(self) -> None:
        self.last_rx = 0.0          # FC(sys 1) 에서 마지막으로 뭐든 받은 시각 (monotonic)
        self.last_hb = 0.0
        self.custom_mode: Optional[int] = None
        self.armed = False
        self.lpn_x: Optional[float] = None   # LOCAL_POSITION_NED 북쪽 m
        self.lpn_y: Optional[float] = None   # 동쪽 m
        self.lpn_z: Optional[float] = None
        self.lpn_t = 0.0
        self.rf_m: Optional[float] = None
        self.rf_t = 0.0
        self.rf_floor_valid = False
        self.rf_max_m = 0.0
        self.rf_boot_ms: Optional[int] = None
        self.vbat: Optional[float] = None
        self.vbat_t = 0.0
        self.att: Optional[tuple] = None   # (롤, 피치, 요) 도. 요는 0~360
        self.att_t = 0.0
        self.origin_seen = False
        self.uplink_ok = False      # COMMAND_ACK 를 받아서 노트북 -> FC 방향이 확인됨


class _Sink:
    """pymavlink 의 MAVLink.send() 가 쓰는 파일 대용 (seq 증가를 pymavlink 에 맡기려고)."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def write(self, data: bytes) -> None:
        self.buf += data

    def take(self) -> bytes:
        out = bytes(self.buf)
        self.buf.clear()
        return out


# --------------------------------------------------------------------------
# 본체
# --------------------------------------------------------------------------
class N6Fly:
    def __init__(self, args: argparse.Namespace, console: Console, link):
        self.a = args
        self.c = console
        self.link = link
        link.on_bytes = self._on_bytes
        link.on_state = self._on_link_state

        self._sink = _Sink()
        self.tx = mavlink2.MAVLink(self._sink, srcSystem=GCS_SYSID, srcComponent=GCS_COMPID)
        self.rx = mavlink2.MAVLink(None, srcSystem=GCS_SYSID, srcComponent=GCS_COMPID)
        self.rx.robust_parsing = True   # 깨진 프레임은 예외 대신 BAD_DATA 로

        self.s = FcState()
        self.params: Dict[str, float] = {}
        self.fs_problems: List[str] = []
        self.fs_checked = False        # 이번 연결에서 페일세이프 파라미터 확인이 끝났는지
        self._fs_task: Optional[asyncio.Task] = None
        self.bad_frames = 0

        self._ack_waiters: Dict[int, List[asyncio.Future]] = {}
        self._config_task: Optional[asyncio.Task] = None
        self._test_task: Optional[asyncio.Task] = None
        self._cancel_reason = ""
        self._phase = ""
        self._guard_mode: Optional[int] = None
        self._guard_armed = False
        self._land_after_relink = False
        self._pending_confirm = False
        self._last_x = 0.0
        self._land_since = 0.0         # LAND + 시동 ON 이 시작된 시각
        self._quitting = False
        self._land_sent_on_quit = False
        self._quit: Optional[asyncio.Event] = None
        self._link_up_at = 0.0
        self._yaw_target: Optional[float] = None   # 회전 시험 중 목표 방위각 (CSV 기록용)
        self._att_samples: Optional[list] = None   # 회전 뒤 멈춰 있는 동안 모으는 (롤, 피치, 요)
        self._att_csv = None
        self._floor_test = getattr(args, "floor_height", False)
        self._floor_baseline = None
        self._manual_on = False        # --manual 이동 키를 받는 중
        self._manual_busy = False      # 이동 중 (이때 누른 이동 키는 버림, 쌓지 않음)
        self._manual_req: Optional[tuple] = None

    # ---------------- 송신 ----------------
    def _encode(self, msg) -> bytes:
        self.tx.send(msg)
        return self._sink.take()

    async def send(self, msg) -> bool:
        data = self._encode(msg)
        # shield: 시험 태스크가 취소돼도 이미 시작한 프레임은 끝까지 보낸다.
        # (프레임이 중간에 잘리면 FC 파서가 뒤따르는 LAND 프레임까지 버릴 수 있음)
        return await asyncio.shield(self.link.write(data))

    async def command(self, cmd: int, *params: float, retries: int = 2,
                      timeout: float = 1.0, quiet: bool = False) -> Optional[int]:
        """COMMAND_LONG 을 보내고 COMMAND_ACK 결과를 돌려준다 (None = ACK 없음)."""
        p = [float(x) for x in params] + [0.0] * (7 - len(params))
        loop = asyncio.get_running_loop()
        for attempt in range(retries + 1):
            fut: asyncio.Future = loop.create_future()
            self._ack_waiters.setdefault(cmd, []).append(fut)
            try:
                msg = self.tx.command_long_encode(FC_SYSID, FC_COMPID, cmd, attempt, *p)
                if not await self.send(msg):
                    return None  # BLE 끊김
                try:
                    return await asyncio.wait_for(fut, timeout)
                except asyncio.TimeoutError:
                    continue
            finally:
                waiters = self._ack_waiters.get(cmd, [])
                if fut in waiters:
                    waiters.remove(fut)
        if not quiet:
            self.c.event(f"[ACK 없음] {cmd_name(cmd)} ({retries + 1}번 보냄)")
        return None

    async def _send_mode_burst(self, mode: int, why: str) -> None:
        """DO_SET_MODE 를 0.1 s 간격으로 3번 (ACK 는 기다리지 않음, 오면 화면에 찍힘)."""
        if not self.link.connected:
            self.c.event(f"{why}: BLE 끊김 상태라 명령을 못 보냄 — FC 가 GCS 페일세이프로 {FS_GCS_TIMEOUT_S}초 뒤 스스로 LAND")
            return
        for i in range(LAND_REPEAT):
            await self.send(self.tx.command_long_encode(
                FC_SYSID, FC_COMPID, mavlink2.MAV_CMD_DO_SET_MODE, i,
                mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0))
            if i < LAND_REPEAT - 1:
                await asyncio.sleep(LAND_REPEAT_GAP_S)

    async def send_origin(self) -> None:
        lat, lon, alt = EKF_ORIGIN
        await self.send(self.tx.set_gps_global_origin_encode(
            FC_SYSID, int(round(lat * 1e7)), int(round(lon * 1e7)), int(round(alt * 1000))))

    # ---------------- 수신 ----------------
    def _on_bytes(self, data: bytes) -> None:
        try:
            msgs = self.rx.parse_buffer(data)
        except Exception:
            self.bad_frames += 1
            return
        if not msgs:
            return
        now = time.monotonic()
        for msg in msgs:
            try:
                self._handle(msg, now)
            except Exception as e:  # 메시지 하나 때문에 수신이 멈추지 않게
                self.c.event(f"(수신 처리 오류: {msg.get_type()} {e!r})")

    def _handle(self, msg, now: float) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            self.bad_frames += 1
            return
        s = self.s
        src = msg.get_srcSystem()
        if src == FC_SYSID:
            if s.last_rx and now - s.last_rx > FC_STALE_S:
                self.c.event(f"FC 링크 복구 ({now - s.last_rx:.1f} s 동안 끊겼었음) — 설정 다시 보냄")
                self._schedule_config()
            s.last_rx = now

        if mtype == "HEARTBEAT":
            self._on_heartbeat(msg, now)
        elif mtype == "COMMAND_ACK":
            self._on_ack(msg)
        elif mtype == "STATUSTEXT":
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode("utf-8", "replace")
            sev = _enum_name("MAV_SEVERITY", msg.severity, "MAV_SEVERITY_")
            self.c.event(f"[FC {sev}] {text.rstrip(chr(0))}")
        elif src != FC_SYSID:
            return
        elif mtype == "LOCAL_POSITION_NED":
            s.lpn_x, s.lpn_y, s.lpn_z, s.lpn_t = msg.x, msg.y, msg.z, now
        elif mtype == "DISTANCE_SENSOR":
            if msg.orientation == RANGEFINDER_DOWN:
                if self._floor_test:
                    if msg.get_srcComponent() != FC_COMPID:
                        return
                    if s.rf_boot_ms is not None and not 0 < (msg.time_boot_ms - s.rf_boot_ms) % 2**32 < 2**31:
                        return
                s.rf_boot_ms = msg.time_boot_ms
                s.rf_m, s.rf_t = msg.current_distance / 100.0, now
                s.rf_max_m = msg.max_distance / 100.0
                s.rf_floor_valid = (msg.get_srcComponent() == FC_COMPID and
                                    0 < msg.current_distance < 65535 and
                                    msg.min_distance <= msg.current_distance <= msg.max_distance and
                                    getattr(msg, "signal_quality", 0) != 1)
        elif mtype == "ATTITUDE":
            self._on_attitude(msg, now)
        elif mtype == "SYS_STATUS":
            if msg.voltage_battery not in (0xFFFF, 0):
                s.vbat, s.vbat_t = msg.voltage_battery / 1000.0, now
        elif mtype == "GPS_GLOBAL_ORIGIN":
            if not s.origin_seen:
                s.origin_seen = True
                self.c.event(f"EKF 원점 설정됨: {msg.latitude / 1e7:.6f}, {msg.longitude / 1e7:.6f}")
        elif mtype == "PARAM_VALUE":
            pid = msg.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode("ascii", "ignore")
            self.params[pid.rstrip("\x00")] = float(msg.param_value)

    def _on_attitude(self, msg, now: float) -> None:
        s = self.s
        att = (math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw) % 360.0)
        s.att, s.att_t = att, now
        if self._att_samples is not None:
            self._att_samples.append(att)
        if self._att_csv is None:
            path = self.c.log_path.with_name(self.c.log_path.stem + "_att.csv")
            self._att_csv = open(path, "a", encoding="utf-8", buffering=1)
            extra = (",fc_att_boot_ms,fc_rf_boot_ms,cmd_alt_m,fc_takeoff_frame,rf0_m,"
                     "decision_floor_m,threshold_m" if self._floor_test else "")
            self._att_csv.write("time,phase,target_yaw_deg,roll_deg,pitch_deg,yaw_deg,ekf_h_m,rangefinder_m" + extra + "\n")
            self.c.event(f"자세 기록: {path}")
        h = -s.lpn_z if (s.lpn_z is not None and self._fresh(s.lpn_t)) else None
        rf = s.rf_m if (s.rf_m is not None and self._fresh(s.rf_t)) else None
        tgt = self._yaw_target
        cells = [_dt.datetime.now().strftime("%H:%M:%S.%f")[:-3], self._phase,
                 "" if tgt is None else f"{tgt:.1f}", f"{att[0]:.2f}", f"{att[1]:.2f}", f"{att[2]:.2f}",
                 "" if h is None else f"{h:.3f}", "" if rf is None else f"{rf:.3f}"]
        try:
            if self._floor_test:
                floor = self._floor_height()
                cells.extend([str(msg.time_boot_ms), "" if s.rf_boot_ms is None else str(s.rf_boot_ms),
                              str(self.a.alt), "FC_auto_unobserved",
                              "" if self._floor_baseline is None else str(self._floor_baseline),
                              "" if floor is None else str(floor), f"{self.a.alt - 0.1:.2f}"])
            self._att_csv.write(",".join(cells) + "\n")
        except Exception:
            pass

    def _on_heartbeat(self, msg, now: float) -> None:
        if msg.get_srcSystem() != FC_SYSID or msg.get_srcComponent() != FC_COMPID:
            return
        if msg.type == mavlink2.MAV_TYPE_GCS:
            return
        s = self.s
        armed = bool(msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = msg.custom_mode
        if s.last_hb == 0.0:
            self.c.event(f"FC HEARTBEAT 수신: 모드 {mode_name(mode)}, 시동 {'ON' if armed else 'OFF'}")
        else:
            if mode != s.custom_mode:
                self.c.event(f"모드: {mode_name(s.custom_mode)} → {mode_name(mode)}")
            if armed != s.armed:
                self.c.event("시동 ON" if armed else "시동 OFF")
        s.custom_mode, s.armed, s.last_hb = mode, armed, now

        # 링크가 끊겨 시험이 중단됐는데 그 사이 FC 가 아직 GUIDED 로 떠 있다면 (3 s 안에 다시 붙은 경우)
        # 그대로 두면 아무 명령 없이 호버하므로 LAND 를 보낸다. (GUIDED 로 되돌리는 일은 절대 없음)
        if self._land_after_relink:
            self._land_after_relink = False
            if armed and mode == MODE_GUIDED:
                self.c.event("링크 복구: 중단된 시험의 기체가 아직 GUIDED + 시동 상태 → LAND 전송")
                asyncio.get_running_loop().create_task(self._send_mode_burst(MODE_LAND, "LAND"))

    def _on_ack(self, msg) -> None:
        src = msg.get_srcSystem()
        tgt = getattr(msg, "target_system", 0)
        who = "" if src == FC_SYSID else f" (sys {src})"
        self.c.event(f"[ACK] {cmd_name(msg.command)} → {result_name(msg.result)}{who}")
        if src != FC_SYSID or tgt not in (0, GCS_SYSID):
            return
        if not self.s.uplink_ok:
            self.s.uplink_ok = True
            self.c.event("노트북→FC 방향 확인됨 (COMMAND_ACK 수신)")
        if msg.result == IN_PROGRESS:
            return
        for fut in list(self._ack_waiters.get(msg.command, [])):
            if not fut.done():
                fut.set_result(msg.result)

    # ---------------- 연결 / FC 설정 ----------------
    def _on_link_state(self, up: bool) -> None:
        if self._floor_test:
            self.s.rf_boot_ms = None
            self.s.rf_floor_valid = False
        if up:
            self._link_up_at = time.monotonic()
            self.fs_checked = False
            self._schedule_config(restart=True)  # 이전 연결의 설정 작업이 남아 있으면 버리고 새로
            return
        self.s.uplink_ok = False
        if self._quitting:
            return
        self.c.event(f"BLE 끊김 — FC가 {FS_GCS_TIMEOUT_S}초 뒤 스스로 착륙 (GCS 페일세이프, 시동 상태일 때). "
                     "재연결 시도 중...")
        if self._test_task is not None and not self._test_task.done():
            self._land_after_relink = True
            self._cancel_test("BLE 끊김 — 재연결돼도 자동으로 다시 시작하지 않음")

    def _schedule_config(self, restart: bool = False) -> None:
        if self._config_task is not None and not self._config_task.done():
            if not restart:
                return
            self._config_task.cancel()
        self._config_task = asyncio.get_running_loop().create_task(self._configure_fc())

    async def _configure_fc(self) -> None:
        up_at = self._link_up_at
        t0 = time.monotonic()
        warned = False
        while self.s.last_rx < up_at:  # 이번 연결에서 FC 가 말하는 걸 한 번은 듣고 시작
            if not self.link.connected:
                return
            if not warned and time.monotonic() - t0 > 5.0:
                self.c.event("FC 응답 대기 중... (NUCLEO 브리지 LED1, FC 전원, SERIAL4 설정 확인)")
                warned = True
            await asyncio.sleep(0.1)
        if not self.link.connected:
            return

        self.c.event("FC 설정: 기본 스트림 끄기 → 필요한 메시지 간격 설정 → EKF 원점")
        # 1) 기본 스트림 끄기. ArduPilot 은 스트림 속도를 바꾸면 그 스트림에 속한 메시지 간격을
        #    덮어쓰므로, 반드시 SET_MESSAGE_INTERVAL 보다 "먼저" 보낸다. (ACK 없는 메시지라 2번)
        for _ in range(2):
            await self.send(self.tx.request_data_stream_encode(
                FC_SYSID, FC_COMPID, mavlink2.MAV_DATA_STREAM_ALL, 0, 0))
            await asyncio.sleep(0.1)
        # 2) 필요한 것만
        for msg_id, hz in STREAM_INTERVALS:
            await self.command(mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
        # 3) EKF 원점 (GPS 없이 GUIDED 이륙에 필요. 이미 있으면 FC 가 무시)
        await self.send_origin()
        # 4) 페일세이프가 실제로 이 스크립트의 하트비트를 보는지 확인
        await self._check_failsafe_params()

    async def _check_failsafe_params(self) -> None:
        # 한꺼번에 요청하면 FC 응답(PARAM_VALUE)이 몰려 와서 XIAO 구간에서 일부가 빠지는 것을
        # 실기에서 확인함 -> 하나씩 간격을 두고 요청하고, 못 받은 것만 최대 3번 다시 요청
        floor_params = FLOOR_PARAMS if self._floor_test else {}
        for name in ("MAV_GCS_SYSID", "SYSID_MYGCS", "FS_GCS_ENABLE", "FS_GCS_TIMEOUT", *floor_params):
            self.params.pop(name, None)

        def missing() -> List[str]:
            want = ["FS_GCS_ENABLE", "FS_GCS_TIMEOUT", *floor_params]
            if "MAV_GCS_SYSID" not in self.params and "SYSID_MYGCS" not in self.params:
                want = ["MAV_GCS_SYSID", "SYSID_MYGCS"] + want   # 4.7 은 MAV_GCS_SYSID, 4.6 이하는 SYSID_MYGCS
            return [n for n in want if n not in self.params]

        for _round in range(3):
            todo = missing()
            if not todo or not self.link.connected:
                break
            for name in todo:
                await self.send(self.tx.param_request_read_encode(FC_SYSID, FC_COMPID, name.encode(), -1))
                await asyncio.sleep(0.3)
            end = time.monotonic() + 1.5
            while time.monotonic() < end and missing():
                await asyncio.sleep(0.1)
        if not self.link.connected:
            return

        sysid = self.params.get("MAV_GCS_SYSID", self.params.get("SYSID_MYGCS"))
        fs_en = self.params.get("FS_GCS_ENABLE")
        fs_to = self.params.get("FS_GCS_TIMEOUT")
        problems, notes = [], []
        for name, expected in floor_params.items():
            if name not in self.params:
                problems.append(f"{name} 못 읽음")
            elif self.params[name] != expected:
                problems.append(f"{name}={self.params[name]:g} (바닥 기준 시험 설정 {expected})")
        if sysid is None:
            problems.append("지상국 sysid(MAV_GCS_SYSID) 못 읽음 — t 를 다시 누르면 다시 읽음")
        elif int(sysid) != GCS_SYSID:
            problems.append(f"지상국 sysid={int(sysid)} (250 이어야 이 스크립트 하트비트가 페일세이프에 잡힘)")
        if fs_en is None:
            problems.append("FS_GCS_ENABLE 못 읽음 — t 를 다시 누르면 다시 읽음")
        elif int(fs_en) == 0:
            problems.append("FS_GCS_ENABLE=0 (링크가 끊겨도 착륙 안 함)")
        elif int(fs_en) != 5:
            notes.append(f"FS_GCS_ENABLE={int(fs_en)} (권장 5 = 항상 LAND)")
        if fs_to is None:
            notes.append("FS_GCS_TIMEOUT 못 읽음")
        elif abs(fs_to - FS_GCS_TIMEOUT_S) > 0.05:
            notes.append(f"FS_GCS_TIMEOUT={fs_to:g} s (이 스크립트 기준 {FS_GCS_TIMEOUT_S} s — 화면의 'N초 뒤 착륙' 안내와 다름)")
        self.fs_problems = problems
        self.fs_checked = True
        if problems:
            self.c.event("⚠ 페일세이프 설정 문제 → 이륙(t) 막음: " + "; ".join(problems))
        if notes:
            self.c.event("참고: " + "; ".join(notes))
        if not problems and not notes:
            self.c.event(f"페일세이프 파라미터 OK (지상국 sysid {int(sysid)}, "
                         f"FS_GCS_ENABLE {int(fs_en)}, FS_GCS_TIMEOUT {fs_to:g} s)")

    # ---------------- 주기 작업 ----------------
    async def _heartbeat_loop(self) -> None:
        loop = asyncio.get_running_loop()
        next_t = loop.time()
        while True:
            if self.link.connected:
                await self.send(self.tx.heartbeat_encode(
                    mavlink2.MAV_TYPE_GCS, mavlink2.MAV_AUTOPILOT_INVALID, 0, 0,
                    mavlink2.MAV_STATE_ACTIVE))
            next_t += HEARTBEAT_PERIOD_S
            delay = next_t - loop.time()
            if delay < 0:  # 밀렸으면 따라잡지 말고 지금부터 다시
                next_t = loop.time()
                delay = 0
            await asyncio.sleep(delay)

    def _fresh(self, t: float) -> bool:
        return t > 0 and time.monotonic() - t < FRESH_S

    def _floor_height(self) -> Optional[float]:
        s = self.s
        if (s.rf_floor_valid and s.rf_t > 0 and 0 <= time.monotonic() - s.rf_t < 0.5
                and self.a.alt < s.rf_max_m):
            return s.rf_m
        return None

    def height(self) -> Optional[float]:
        if self.s.lpn_z is not None and self._fresh(self.s.lpn_t):
            return -self.s.lpn_z
        if self.s.rf_m is not None and self._fresh(self.s.rf_t):
            return self.s.rf_m
        return None

    def _status_text(self) -> str:
        s = self.s
        now = time.monotonic()
        parts = ["BLE OK" if self.link.connected else "BLE 끊김"]
        if s.last_rx == 0.0:
            parts.append("FC 응답 없음")
        else:
            age = now - s.last_rx
            parts.append(f"링크 {age:.1f}s" + (" 끊김!" if age > FC_STALE_S else ""))
        parts.append(mode_name(s.custom_mode))
        parts.append("시동 ON" if s.armed else "시동 OFF")
        h = None
        if s.lpn_z is not None and self._fresh(s.lpn_t):
            h = -s.lpn_z
        rf = s.rf_m if (s.rf_m is not None and self._fresh(s.rf_t)) else None
        # EKF 높이는 FC 가 켜진 곳(원점) 기준이고 모터 바람에 흔들림 -> 거리센서(지면까지)와 나란히 표시
        parts.append(f"EKF높이 {h:+.2f}m" if h is not None else "EKF높이 -")
        parts.append(f"거리센서 {rf:.2f}m" if rf is not None else "거리센서 -")
        if s.att is not None and self._fresh(s.att_t):
            parts.append(f"요 {s.att[2]:5.1f}° 피치 {s.att[1]:+.1f}° 롤 {s.att[0]:+.1f}°")
        else:
            parts.append("자세 -")
        parts.append(f"{s.vbat:.2f}V" if (s.vbat is not None and self._fresh(s.vbat_t)) else "배터리 -")
        if self._test_task is not None and not self._test_task.done():
            parts.append(f"시험:{self._phase}")
        return " | ".join(parts)

    async def _status_loop(self) -> None:
        while True:
            self.c.status(self._status_text())
            await self._land_disarm_check()
            await asyncio.sleep(1.0)

    async def _land_disarm_check(self) -> None:
        s, now = self.s, time.monotonic()
        if not (s.armed and s.custom_mode == MODE_LAND):
            self._land_since = 0.0
            return
        if self._land_since == 0.0:
            self._land_since = now
            return
        if now - self._land_since >= LAND_DISARM_S and self.link.connected:
            self._land_since = now  # 안 꺼지면 다시 같은 시간 뒤
            self.c.event(f"LAND {LAND_DISARM_S:g}초가 지났는데 시동 ON → FC 가 착지를 감지 못함. 강제 DISARM 전송")
            await self._force_disarm()

    async def _force_disarm(self) -> None:
        for i in range(LAND_REPEAT):
            await self.send(self.tx.command_long_encode(
                FC_SYSID, FC_COMPID, mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, i,
                0, FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0))
            if i < LAND_REPEAT - 1:
                await asyncio.sleep(LAND_REPEAT_GAP_S)

    # ---------------- 키 ----------------
    async def _key_loop(self, q: "asyncio.Queue[str]") -> None:
        while True:
            key = await q.get()
            try:
                await self._handle_key(key)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.c.event(f"(키 처리 오류: {e!r})")

    async def _handle_key(self, key: str) -> None:
        key = HANGUL_KEYS.get(key, key)
        low = key.lower() if len(key) == 1 else key
        if low in KILL_KEYS:  # 킬은 언제나 최우선
            self._pending_confirm = False
            self._cancel_test("킬")
            self.c.event("킬! LAND 3회 전송 — 그 자리 수직 하강, 착지하면 FC 가 시동 끔")
            await self._send_mode_burst(MODE_LAND, "킬")
            return
        if low == "CTRL_C":
            await self.request_quit("Ctrl-C")
            return
        if low == "EOF":
            self.c.event("키 입력이 끝났습니다 (stdin EOF). 종료는 Ctrl-C")
            return
        if self._pending_confirm:
            self._pending_confirm = False
            if low == "y":
                self._start_test()
            else:
                self.c.event("이륙 시험 취소")
            return
        if self._manual_on and key in MANUAL_KEYS:
            if self._manual_busy:
                self.c.event("    (이동 중 — 입력 무시)")
            else:
                self._manual_busy = True
                self._manual_req = MANUAL_KEYS[key]
            return
        if low == "t":
            self._request_test()
        elif low == "x":
            await self._x_pressed()
        elif low == "q":
            await self.request_quit("q")
        elif low in ("h", "?"):
            self.c.event(HELP)

    async def _x_pressed(self) -> None:
        now = time.monotonic()
        if now - self._last_x > DOUBLE_X_WINDOW_S:
            self._last_x = now
            self.c.event("x: 1초 안에 x 를 한 번 더 누르면 모터 즉시 정지 (강제 DISARM). 공중이면 추락!")
            return
        self._last_x = 0.0
        self._cancel_test(EMERGENCY_STOP_REASON)
        if not self.link.connected:
            self.c.event("비상 정지: BLE 끊김 상태라 못 보냄")
            return
        self.c.event("비상 모터 정지! 강제 DISARM 전송")
        await self._force_disarm()

    async def request_quit(self, why: str) -> None:
        self._cancel_test(f"종료({why})")
        if self.s.armed and self.link.connected:
            self.c.event("종료 전에 LAND 전송 (시동 상태)")
            await self._send_mode_burst(MODE_LAND, "종료")
            self._land_sent_on_quit = True
        if self._quit is not None:
            self._quit.set()

    # ---------------- 이륙 시험 ----------------
    def _test_preconditions(self) -> List[str]:
        s = self.s
        why = []
        if self._floor_test and self._floor_height() is None:
            why.append("검증용 하향 거리센서 측정 없음/범위 초과")
        if self.a.no_arm:
            why.append("--no-arm 으로 실행 중")
        if not self.link.connected:
            why.append("BLE 연결 안 됨")
        if s.last_hb == 0.0 or time.monotonic() - s.last_hb > FC_STALE_S:
            why.append("FC HEARTBEAT 없음")
        if not s.uplink_ok:
            why.append("노트북→FC 방향 확인 안 됨 (COMMAND_ACK 를 아직 못 받음)")
        if s.armed:
            why.append("이미 시동 상태")
        if self._test_task is not None and not self._test_task.done():
            why.append("이미 시험 중")
        if not self.fs_checked:
            why.append("페일세이프 파라미터 확인 중 (몇 초 뒤 다시 t)")
        why.extend(self.fs_problems)
        return why

    def _request_test(self) -> None:
        why = self._test_preconditions()
        if why:
            self.c.event("이륙 시험 못 함: " + "; ".join(why))
            # 페일세이프 값을 못 읽어서 막힌 경우 -> 다시 읽기 시작 (FC->노트북 손실 대비)
            if (self.fs_checked and any("못 읽음" in w for w in self.fs_problems)
                    and self.link.connected and (self._fs_task is None or self._fs_task.done())):
                self.fs_checked = False
                self._fs_task = asyncio.get_running_loop().create_task(self._check_failsafe_params())
            return
        self._pending_confirm = True
        yaw = getattr(self.a, "yaw", 0.0)
        turn = f" / 회전 {yaw:g}° x {self._yaw_steps()}번" if yaw else ""
        move = getattr(self.a, "move", 0.0)
        turn += f" / 앞·뒤·오른쪽·왼쪽 {move:g} m 이동 (사방 1 m 비워 둘 것)" if move else ""
        if getattr(self.a, "manual", False):
            fb = getattr(self.a, "step_fb", None) or self.a.step
            turn += (f" / 호버 뒤 키로 앞뒤 {fb:g} m, 좌우 {self.a.step:g} m 씩 이동, 내릴 때 스페이스 "
                     f"(사방 {MANUAL_MAX_M + 0.5:g} m 비워 둘 것)")
        self.c.event(f"이륙 시험 {self.a.alt:.2f} m / 호버 {self._hover_txt()}{turn} — 프롭·주변·안전줄 확인했으면 y, "
                     "취소는 다른 키. (언제든 킬 = 스페이스)")

    def _hover_txt(self) -> str:
        return "착륙 명령(스페이스)까지" if self.a.hover == math.inf else f"{self.a.hover:g} s"

    def _yaw_steps(self) -> int:
        yaw = getattr(self.a, "yaw", 0.0)
        return max(1, round(360.0 / yaw)) if yaw else 0

    async def _yaw_test(self) -> None:
        """호버 중 제자리 회전: +yaw° 씩 시계 방향으로 돌아 한 바퀴. 회전마다 도달 시간, 방위각 오차,
        멈춰 있는 동안 요·피치·롤 평균과 표준편차를 찍는다."""
        a, s, c = self.a, self.s, self.c
        steps = self._yaw_steps()
        if s.att is None or not self._fresh(s.att_t):
            c.event("    ⚠ ATTITUDE 가 안 들어와서 회전 시험 건너뜀")
            return
        yaw0 = s.att[2]
        c.event(f"    회전 시작: 시작 방위각 {yaw0:.1f}°, {a.yaw:g}° 씩 {steps}번, {YAW_RATE_DPS:g}°/s, "
                f"매번 {a.yaw_hold:g} s 정지")
        for k in range(1, steps + 1):
            target = (yaw0 + k * a.yaw) % 360.0
            self._yaw_target = target
            self._phase = f"회전{k}/{steps}"
            # CONDITION_YAW: 각도, 속도, 방향(1 = 시계), 상대(1)
            res = await self.command(mavlink2.MAV_CMD_CONDITION_YAW, a.yaw, YAW_RATE_DPS, 1, 1,
                                     retries=0, timeout=1.5)
            if res is not None and res != ACCEPTED:
                raise TestFailed(f"회전 명령 거부 ({result_name(res)})")
            t0 = time.monotonic()
            reached = await self._wait(
                lambda: s.att is not None and abs(wrap180(s.att[2] - target)) < YAW_REACHED_DEG,
                a.yaw / YAW_RATE_DPS + 5.0)
            t_reach = time.monotonic() - t0
            self._att_samples = []
            try:
                await self._wait(lambda: False, a.yaw_hold)
                samples = self._att_samples
            finally:
                self._att_samples = None
            if len(samples) < 2:
                c.event(f"    회전 {k}/{steps}: 목표 {target:.1f}° — 정지 중 ATTITUDE {len(samples)}개뿐, 통계 없음")
                continue
            err = [wrap180(y - target) for (_, _, y) in samples]
            pitch = [p for (_, p, _) in samples]
            roll = [r for (r, _, _) in samples]
            how = f"{t_reach:.1f} s 만에 도달" if reached else f"{t_reach:.1f} s 안에 ±{YAW_REACHED_DEG:g}° 도달 못 함"
            c.event(f"    회전 {k}/{steps}: 목표 {target:.1f}°, {how} | 정지 {len(samples)}개: "
                    f"요 오차 {statistics.fmean(err):+.1f}° (흔들림 {statistics.pstdev(err):.1f}°), "
                    f"피치 {statistics.fmean(pitch):+.1f}° ({statistics.pstdev(pitch):.1f}°), "
                    f"롤 {statistics.fmean(roll):+.1f}° ({statistics.pstdev(roll):.1f}°)")
        self._yaw_target = None

    async def _move_test(self) -> None:
        """호버 중 수평 이동: 시작점에서 앞·뒤·오른쪽·왼쪽으로 --move m 씩 갔다가 매번 시작점으로 돌아온다.
        방향은 시작할 때의 기체 방위각 기준이고 높이와 방위각은 시작 값으로 고정한다."""
        a, s, c = self.a, self.s, self.c
        if s.att is None or not self._fresh(s.att_t):
            c.event("    ⚠ ATTITUDE 가 안 들어와서 이동 시험 건너뜀")
            return
        if s.lpn_x is None or not self._fresh(s.lpn_t):
            c.event("    ⚠ LOCAL_POSITION_NED 가 안 들어와서 이동 시험 건너뜀")
            return
        x0, y0, z0, yaw0 = s.lpn_x, s.lpn_y, s.lpn_z, s.att[2]
        psi = math.radians(yaw0)
        c.event(f"    이동 시작: 시작점 북 {x0:+.2f} 동 {y0:+.2f} m, 높이 {-z0:.2f} m, 방위각 {yaw0:.1f}°, "
                f"{a.move:g} m, 매번 {a.move_hold:g} s 정지")

        def dist(tx: float, ty: float) -> float:
            return math.hypot(s.lpn_x - tx, s.lpn_y - ty)

        n = len(MOVE_STEPS)
        for k, (name, fwd, right) in enumerate(MOVE_STEPS, 1):
            f, r = fwd * a.move, right * a.move
            tx = x0 + f * math.cos(psi) - r * math.sin(psi)
            ty = y0 + f * math.sin(psi) + r * math.cos(psi)
            self._phase = f"이동{k}/{n}"
            reached, t_reach = await self._go_to(tx, ty, z0, psi)
            samples = []
            end = time.monotonic() + a.move_hold
            last_t = None
            while time.monotonic() < end:
                await self._wait(lambda: False, 0.05)
                if s.lpn_t != last_t and self._fresh(s.lpn_t):
                    last_t = s.lpn_t
                    samples.append((dist(tx, ty), -(s.lpn_z - z0)))
            how = f"{t_reach:.1f} s 만에 도달" if reached else f"{t_reach:.1f} s 안에 {MOVE_REACHED_M:g} m 안으로 못 옴"
            if len(samples) < 2:
                c.event(f"    이동 {k}/{n} {name}: {how} — 정지 중 위치 {len(samples)}개뿐, 통계 없음")
                continue
            err = [e for e, _ in samples]
            dz = [d for _, d in samples]
            c.event(f"    이동 {k}/{n} {name}: {how} | 정지 {len(samples)}개: "
                    f"목표와 거리 {statistics.fmean(err):.2f} m (흔들림 {statistics.pstdev(err):.2f}), "
                    f"높이 변화 {statistics.fmean(dz):+.2f} m ({statistics.pstdev(dz):.2f})")

    async def _go_to(self, tx: float, ty: float, z: float, psi: float):
        """위치 목표를 보내고 MOVE_REACHED_M 안으로 올 때까지 MOVE_RESEND_S 마다 다시 보낸다 (최대 MOVE_TIMEOUT_S).
        돌려주는 값: (도달 여부, 걸린 시간 s)."""
        s = self.s
        msg = mavlink2.MAVLink_set_position_target_local_ned_message(
            0, FC_SYSID, FC_COMPID, mavlink2.MAV_FRAME_LOCAL_NED, MOVE_TYPE_MASK,
            tx, ty, z, 0, 0, 0, 0, 0, 0, psi, 0)
        t0 = time.monotonic()
        reached = False
        while not reached and time.monotonic() - t0 < MOVE_TIMEOUT_S:
            await self.send(msg)
            reached = await self._wait(
                lambda: self._fresh(s.lpn_t) and math.hypot(s.lpn_x - tx, s.lpn_y - ty) < MOVE_REACHED_M,
                min(MOVE_RESEND_S, MOVE_TIMEOUT_S - (time.monotonic() - t0)))
        return reached, time.monotonic() - t0

    async def _manual_move(self) -> None:
        """--manual: 키 하나에 --step m 씩 이동. 이동이 끝나고 MANUAL_SETTLE_S 멈출 때까지 다음 이동 키는 버린다.
        방향은 시작할 때의 방위각 기준, 높이와 방위각은 시작 값으로 고정. 스페이스(킬)로 끝낸다."""
        a, s, c = self.a, self.s, self.c
        if s.att is None or not self._fresh(s.att_t) or s.lpn_x is None or not self._fresh(s.lpn_t):
            c.event("    ⚠ 자세/위치가 안 들어와서 수동 이동 못 함 — 그대로 호버. 내리려면 스페이스")
            await self._wait(lambda: False, math.inf)
        x0, y0, z0, yaw0 = s.lpn_x, s.lpn_y, s.lpn_z, s.att[2]
        psi = math.radians(yaw0)
        f, r = 0.0, 0.0    # 지금 목표: 시작점에서 앞, 오른쪽 m
        step_fb = getattr(a, "step_fb", None) or a.step
        c.event(f"    수동 이동: ↑/w 앞, ↓/s 뒤, ←/a 왼쪽, →/d 오른쪽, r 시작점. 한 번에 앞뒤 {step_fb:g} m, 좌우 {a.step:g} m, "
                f"시작점에서 최대 {MANUAL_MAX_M:g} m. 이동 중 누른 키는 무시. 내리려면 스페이스")
        self._manual_on = True
        try:
            while True:
                self._phase = "수동대기"
                self._manual_req, self._manual_busy = None, False
                await self._wait(lambda: self._manual_req is not None, math.inf)
                name, df, dr = self._manual_req
                nf, nr = (0.0, 0.0) if df is None else (f + df * step_fb, r + dr * a.step)
                if math.hypot(nf, nr) > MANUAL_MAX_M + 1e-6:
                    c.event(f"    {name}: 시작점에서 {math.hypot(nf, nr):.2f} m 라서 거부 (최대 {MANUAL_MAX_M:g} m)")
                    continue
                f, r = nf, nr
                tx = x0 + f * math.cos(psi) - r * math.sin(psi)
                ty = y0 + f * math.sin(psi) + r * math.cos(psi)
                self._phase = f"수동:{name}"
                reached, t_reach = await self._go_to(tx, ty, z0, psi)
                err = math.hypot(s.lpn_x - tx, s.lpn_y - ty)
                if not reached and err > MANUAL_ABORT_M:
                    raise TestFailed(f"{name} 이동: {t_reach:.1f} s 뒤에도 목표와 {err:.2f} m — 위치 추정 이상 의심")
                c.event(f"    {name} → 앞 {f:+.2f} 오른쪽 {r:+.2f} m: "
                        + (f"{t_reach:.1f} s 만에 도달" if reached else f"{t_reach:.1f} s 안에 못 옴 (목표와 {err:.2f} m)")
                        + f", 높이 변화 {-(s.lpn_z - z0):+.2f} m")
                await self._wait(lambda: False, MANUAL_SETTLE_S)
        finally:
            self._manual_on = self._manual_busy = False
            self._manual_req = None

    def _start_test(self) -> None:
        why = self._test_preconditions()
        if why:
            self.c.event("이륙 시험 못 함: " + "; ".join(why))
            return
        self._test_task = asyncio.get_running_loop().create_task(self._takeoff_test())

    def _cancel_test(self, reason: str) -> None:
        t = self._test_task
        if t is not None and not t.done():
            self._cancel_reason = reason
            t.cancel()
        self._guard_mode = None
        self._guard_armed = False

    def _check_guards(self) -> None:
        s = self.s
        if not self.link.connected:
            raise TestAborted("BLE 끊김", link=True)
        if s.last_rx and time.monotonic() - s.last_rx > FC_STALE_S:
            raise TestAborted(f"FC 응답 없음 {FC_STALE_S:g} 초 이상", link=True)
        if self._guard_mode is not None and s.custom_mode != self._guard_mode:
            raise TestAborted(f"모드가 {mode_name(s.custom_mode)}(으)로 바뀜")
        if self._guard_armed and not s.armed:
            raise TestAborted("시동이 꺼짐")
        if self._floor_test and self._phase == "호버" and self._floor_height() is None:
            raise TestFailed("호버 도중 하향 거리센서 무효/갱신 중단")

    async def _wait(self, cond: Callable[[], bool], timeout: float) -> bool:
        """cond 가 참이 될 때까지 기다림 (50 ms 마다 안전 조건도 검사)."""
        end = time.monotonic() + timeout
        while True:
            self._check_guards()
            if cond():
                return True
            if time.monotonic() >= end:
                return False
            await asyncio.sleep(0.05)

    async def _takeoff_test(
        self, group_hover: Optional[Callable[["N6Fly", Callable[[], bool]], Awaitable[None]]] = None
    ) -> None:
        a, s, c = self.a, self.s, self.c
        try:
            c.event(f"=== 이륙 시험 시작: {a.alt:.2f} m, 호버 {self._hover_txt()} (킬 = 스페이스/엔터/k) ===")
            self._phase = "원점"
            await self.send_origin()
            await asyncio.sleep(0.2)

            self._phase = "GUIDED"
            c.event("1/6 GUIDED 모드 요청")
            res = await self.command(mavlink2.MAV_CMD_DO_SET_MODE,
                                     mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED)
            if res is not None and res != ACCEPTED:
                raise TestFailed(f"GUIDED 거부 ({result_name(res)}) — 위 FC 메시지 확인 (위치 추정/EKF 원점?)")
            # ArduPilot 은 모드가 실제로 바뀐 뒤에만 ACCEPTED 를 보냄 -> ACK 로 확인 (HEARTBEAT 는 XIAO 구간에서
            # 잘 빠지므로 보조). 같은 링크라 ACK 이전에 보낸 HEARTBEAT 가 ACK 뒤에 도착하는 일은 없음.
            if res == ACCEPTED:
                s.custom_mode = MODE_GUIDED
                how = "ACK"
            elif await self._wait(lambda: s.custom_mode == MODE_GUIDED, 5.0):
                how = "HEARTBEAT"
            else:
                raise TestFailed("ACK 도 HEARTBEAT 도 없어 GUIDED 확인 안 됨")
            self._guard_mode = MODE_GUIDED   # 이제부터 모드가 바뀌면 즉시 중단
            c.event(f"    GUIDED 확인 ({how})")

            self._phase = "시동"
            c.event("2/6 시동(ARM) 요청")
            res = await self.command(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if res is not None and res != ACCEPTED:
                raise TestFailed(f"시동 거부 ({result_name(res)}) — 위 PreArm/Arm 메시지 확인")
            if res == ACCEPTED:
                s.armed = True
                how = "ACK"
            elif await self._wait(lambda: s.armed, 5.0):
                how = "HEARTBEAT"
            else:
                raise TestFailed("ACK 도 HEARTBEAT 도 없어 시동 확인 안 됨")
            self._guard_armed = True
            z0 = s.lpn_z if (s.lpn_z is not None and self._fresh(s.lpn_t)) else None
            # 시동 순간 거리센서 값 (바닥에 놓인 높이). 낮은 높이에서는 이것 기준으로 올라간 양을 본다.
            # 실기: 모터가 돌기 시작하면 프롭 바람 때문에 기압계 기반 EKF 높이가 바닥에서도 0.1~0.2 m
            # 떨어져서, EKF 기준으로만 재면 7 cm 떴을 때 "0.24 m 도달" 로 잘못 판단했음.
            rf0 = s.rf_m if (s.rf_m is not None and self._fresh(s.rf_t)) else None
            if self._floor_test:
                self._floor_baseline = self._floor_height()
                if self._floor_baseline is None or self._floor_baseline >= a.alt - 0.1:
                    raise TestFailed("이륙 전 하향 거리 기준값 미확인 또는 이미 도달 범위")
                c.event(f"바닥 기준 검증: 이륙 전 거리 {self._floor_baseline:.3f} m, "
                        f"NAV_TAKEOFF {a.alt:.2f} m. FC 기준 자동 선택(직접 관측 불가), "
                        "FC 제어 목표는 기내 CTUN.DAlt에 기록")
            c.event(f"    시동 확인 ({how})")

            self._phase = "이륙"
            if group_hover:
                self._check_guards()
            target = a.alt - 0.1
            c.event(f"3/6 이륙 명령 (목표 {a.alt:.2f} m, {target:.2f} m 넘으면 도달로 봄)")
            # 재전송 안 함: 첫 명령이 먹었는데 ACK 만 잃으면 재전송이 '이미 비행 중' 으로 거부됨
            res = await self.command(mavlink2.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, a.alt,
                                     retries=0, timeout=2.0, quiet=True)
            if res is not None and res != ACCEPTED:
                raise TestFailed(f"이륙 명령 거부 ({result_name(res)})")

            def climbed() -> Optional[float]:
                if self._floor_test:
                    return self._floor_height()
                # 거리센서가 살아 있고 낮은 높이(4 m 미만)면 거리센서 기준 (지면에서 실제로 올라간 양)
                if s.rf_m is not None and self._fresh(s.rf_t) and s.rf_m < 4.0:
                    return s.rf_m - (rf0 if rf0 is not None else 0.0)
                if s.lpn_z is not None and self._fresh(s.lpn_t):
                    return -(s.lpn_z - z0) if z0 is not None else -s.lpn_z
                return None

            def at_height() -> bool:
                return (climbed() or 0.0) >= target

            t0 = time.monotonic()
            reached = await self._wait(at_height, 8.0)
            h = climbed()
            h_txt = f"{h:.2f} m" if h is not None else "알 수 없음"
            if reached:
                c.event(f"    {h_txt} 도달 ({time.monotonic() - t0:.1f} s)")
            elif group_hover or self._floor_test:
                raise TestFailed(f"8초 안에 {target:.2f} m 도달 못 함 (현재 {h_txt})")
            else:
                c.event(f"    8초 안에 {target:.2f} m 도달 못 함 (현재 {h_txt}) — 그대로 호버 후 착륙")

            if group_hover:
                def ready_to_hover() -> bool:
                    if at_height():
                        return True
                    if time.monotonic() >= t0 + 8.0:
                        raise TestFailed("8초 판정 기한 경과 후 공통 호버 상승 기준 미충족")
                    return False

                await group_hover(self, ready_to_hover)
            else:
                self._phase = "호버"
                c.event(f"4/6 호버 {self._hover_txt()}" + (" — 내리려면 스페이스(킬 = LAND)" if a.hover == math.inf else ""))
                await self._wait(lambda: False, a.hover)
            if getattr(a, "yaw", 0.0):
                await self._yaw_test()
            if getattr(a, "move", 0.0):
                await self._move_test()
            if getattr(a, "manual", False):
                await self._manual_move()   # 스페이스(킬)로만 끝남

            self._phase = "착륙"
            c.event("5/6 LAND 모드 요청")
            self._guard_mode = None          # 이제 우리가 모드를 바꿈
            res = await self.command(mavlink2.MAV_CMD_DO_SET_MODE,
                                     mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_LAND)
            if res is not None and res != ACCEPTED:
                if group_hover:
                    raise TestFailed(f"LAND 거부 ({result_name(res)})")
                c.event(f"    ⚠ LAND 거부 ({result_name(res)}) — 스페이스(킬)로 다시 시도하세요")
            if res == ACCEPTED:
                s.custom_mode = MODE_LAND
            if await self._wait(lambda: s.custom_mode == MODE_LAND, 5.0):
                self._guard_mode = MODE_LAND   # 누가 LAND 를 풀면(조종기 등) 시험 종료
                c.event("    LAND 확인")
            else:
                if group_hover:
                    raise TestFailed("5초 안에 LAND 확인 안 됨")
                c.event("    ⚠ 5초 안에 LAND 확인 안 됨 — 스페이스(킬)로 다시 시도하세요")

            self._phase = "착지대기"
            c.event("6/6 착지 → 시동 꺼짐 대기 (최대 30 s)")
            self._guard_armed = False
            if await self._wait(lambda: not s.armed, 30.0):
                c.event("=== 시험 완료: 착지, 시동 꺼짐 ===")
            else:
                c.event("⚠ 30초가 지나도 시동 상태 — 높이 확인 후 스페이스(킬), 바닥이면 x x 로 정지")
                if group_hover:
                    raise TestFailed("LAND 후 30초 동안 시동 해제 미확인")
        except TestAborted as e:
            if e.link:
                self._land_after_relink = True
            if group_hover:
                raise
            c.event(f"■ 시험 중단: {e} — 자동으로 GUIDED 로 되돌리지 않음. 필요하면 스페이스(킬)")
        except TestFailed as e:
            if group_hover:
                raise
            c.event(f"■ 시험 실패: {e}")
            if s.armed and s.custom_mode == MODE_GUIDED and self.link.connected:
                c.event("    시동 상태라 LAND 전송")
                await self._send_mode_burst(MODE_LAND, "LAND")
        except asyncio.CancelledError:
            c.event(f"■ 시험 취소: {self._cancel_reason}")
            raise
        finally:
            self._guard_mode = None
            self._guard_armed = False
            self._phase = ""
            self._yaw_target = None
            self._floor_baseline = None
            self._att_samples = None

    # ---------------- 실행 ----------------
    async def run(self, key_queue: "Optional[asyncio.Queue[str]]" = None) -> None:
        loop = asyncio.get_running_loop()
        self._quit = asyncio.Event()
        keys: Optional[KeyReader] = None
        if key_queue is None:
            key_queue = asyncio.Queue()
            keys = KeyReader(loop, key_queue)
            keys.start()
            self.c.event(f"키 입력 방식: {keys.mode}")
            try:
                loop.add_signal_handler(signal.SIGINT, key_queue.put_nowait, "CTRL_C")
            except (NotImplementedError, RuntimeError, AttributeError):
                pass  # Windows: msvcrt 가 Ctrl-C 를 받거나 KeyboardInterrupt 로 처리
        self.c.event(HELP)
        tasks = [
            loop.create_task(self.link.run()),
            loop.create_task(self._heartbeat_loop()),
            loop.create_task(self._status_loop()),
            loop.create_task(self._key_loop(key_queue)),
        ]
        try:
            await self._quit.wait()
        finally:
            self._quitting = True
            self._cancel_test("종료")
            if self.s.armed and self.link.connected and not self._land_sent_on_quit:
                try:  # Ctrl-C 등으로 갑자기 끝나는 경우에도 최선을 다해 LAND
                    await asyncio.wait_for(self._send_mode_burst(MODE_LAND, "종료"), 1.5)
                except BaseException:
                    pass
            try:
                await asyncio.wait_for(self.link.close(), 4.0)
            except BaseException:
                pass
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if keys is not None:
                keys.restore()
            if self._att_csv is not None:
                self._att_csv.close()
            self.c.event(f"종료. 하트비트가 멈췄으므로, 시동 상태였다면 FC 가 {FS_GCS_TIMEOUT_S}초 뒤 스스로 LAND")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="N6 드론 BLE 비행 스크립트 (이륙 → 호버 → 착륙, 킬 = 스페이스)")
    p.add_argument("--name", default="N6-DRONE-1", help="XIAO BLE 광고 이름 (기본 N6-DRONE-1)")
    p.add_argument("--floor-height", action="store_true",
                   help="순정 GUIDED 이륙을 검증: 신선한 하향 거리 자체로 도달 판정")
    p.add_argument("--alt", type=float, default=0.8, help="이륙 높이 m (기본 0.8, 허용 0.3~1.6)")
    p.add_argument("--hover", type=float, default=5.0, help="호버 시간 s (기본 5, 허용 0~30)")
    p.add_argument("--hold", action="store_true",
                   help="시간 제한 없이 호버. 스페이스(킬 = LAND)를 누를 때까지 떠 있음 (--hover 무시)")
    p.add_argument("--yaw", type=float, default=0.0,
                   help="호버 뒤 제자리 회전 각도 °. 이만큼씩 시계 방향으로 한 바퀴 (기본 0 = 회전 안 함, 허용 0~180)")
    p.add_argument("--yaw-hold", type=float, default=3.0, help="회전마다 멈춰서 자세를 재는 시간 s (기본 3)")
    p.add_argument("--move", type=float, default=0.0,
                   help="호버 뒤 앞·뒤·오른쪽·왼쪽으로 이만큼 m 갔다가 돌아오기 (기본 0 = 이동 안 함, 허용 0.2~0.5)")
    p.add_argument("--move-hold", type=float, default=3.0, help="이동마다 멈춰서 위치를 재는 시간 s (기본 3)")
    p.add_argument("--manual", action="store_true",
                   help="호버 뒤 방향키(또는 w/a/s/d)로 한 칸씩 이동, r = 시작점. 이동 중 누른 키는 무시. 스페이스로 착륙")
    p.add_argument("--step", type=float, default=0.2, help="--manual 한 칸 m (기본 0.2, 허용 0.1~0.5)")
    p.add_argument("--step-fb", type=float, default=None,
                   help="--manual 앞뒤 한 칸 m (주지 않으면 --step 과 같음, 허용 0.1~0.5). 좌우는 --step")
    p.add_argument("--no-arm", action="store_true", help="연결·텔레메트리만 (이륙 키 비활성)")
    p.add_argument("--scan-timeout", type=float, default=8.0, help="BLE 스캔 한 번의 시간 s")
    p.add_argument("--log-dir", default=str(Path(__file__).resolve().parent / "logs"),
                   help="로그 폴더 (기본 gcs/tools/logs)")
    a = p.parse_args(argv)
    if not 0.0 <= a.yaw <= 180.0:
        p.error("--yaw 는 0 ~ 180° 범위로")
    if not 0.5 <= a.yaw_hold <= 10.0:
        p.error("--yaw-hold 는 0.5 ~ 10 s 범위로")
    if a.move and not 0.2 <= a.move <= 0.5:
        p.error("--move 는 0.2 ~ 0.5 m 범위로 (0 = 이동 안 함)")
    if not 0.5 <= a.move_hold <= 10.0:
        p.error("--move-hold 는 0.5 ~ 10 s 범위로")
    if not 0.1 <= a.step <= 0.5:
        p.error("--step 은 0.1 ~ 0.5 m 범위로")
    if a.step_fb is not None and not 0.1 <= a.step_fb <= 0.5:
        p.error("--step-fb 는 0.1 ~ 0.5 m 범위로")
    if a.manual and (a.hold or a.move or a.yaw):
        p.error("--manual 은 --hold / --move / --yaw 와 같이 못 씀 (--manual 이 스페이스까지 떠 있음)")
    if not 0.3 <= a.alt <= 1.6:
        p.error("--alt 는 0.3 ~ 1.6 m 범위로 (FC 고도 펜스가 1.7 m 위로 못 올라가게 막음)")
    if not 0.0 <= a.hover <= 30.0:
        p.error("--hover 는 0 ~ 30 s 범위로")
    if a.hold:
        a.hover = math.inf
    return a


async def amain(args: argparse.Namespace) -> None:
    log_path = Path(args.log_dir) / f"n6_fly_{_dt.datetime.now():%Y%m%d_%H%M%S}.log"
    console = Console(log_path)
    try:
        console.event(f"n6_fly 시작 — 대상 '{args.name}', 로그 {log_path}"
                      + ("  [--no-arm: 이륙 비활성]" if args.no_arm else ""))
        link = BleLink(args.name, console, args.scan_timeout)
        await N6Fly(args, console, link).run()
    finally:
        console.close()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
