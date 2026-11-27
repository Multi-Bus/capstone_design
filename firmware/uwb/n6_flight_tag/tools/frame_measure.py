"""프레임 방식(태그 여러 대, tag_f.c / main_f.c) 기록을 프레임 번호로 합쳐 통신 값을 계산합니다 (Windows / Linux 공용).

기록하면서 분석 (태그마다 USB 한 줄씩, pyserial 필요. 태그는 921600, 앵커는 115200 bit/s):
    python tools/frame_measure.py --ports COM5 COM6 COM7 --anchor-ports COM8 --seconds 120 --out results/stage0

저장한 기록 분석 (USB 로 받은 기록, 모의시험 기록, SD 에서 꺼낸 기록을 섞어 줘도 됨):
    python tools/frame_measure.py --tag-logs results/stage0/port*.raw.log --anchor-logs results/stage0/anchor*.raw.log --out results/stage0
    python tools/frame_measure.py --tag-logs sd/tag*_run*.raw.log --out results/flight1

읽는 줄
  태그: TF_READY, TF_INFO, TF_SD, F 프레임, R 거리, P 패킷, K 제어, L LED, S 통계, D SD 상태 (tag/tag_f.c 머리말),
        비행용 빌드의 C 비행 제어기 상태와 명령, E 사건.  1판(tagf-v1)의 짧은 F 줄도 읽습니다.
  앵커: A<번호>,... (프레임마다 한 줄), A<번호>_BOOT, A<번호>_STAT
  줄 앞의 "<PC 시각>\\t" 은 있어도 되고 없어도 됩니다(SD 에서 꺼낸 기록에는 없음).
  모르는 줄은 종류별로 세어 보고서 끝에 적습니다.

견디는 것
  - 깨진 줄, 빠진 줄: 버리고 셉니다. 프레임 번호가 혼자 튀는 줄은 깨진 줄로 봅니다.
  - 기체 재시작(TF_READY 가 다시 나옴): 꺼져 있던 프레임은 손실로 세지 않습니다.
  - 앵커 1 재시작(프레임 번호가 1 부터 다시): "구간"을 나눕니다. 모든 기록이 같은 횟수의 재시작을 봤다고 가정합니다.
  - DW 시계 한 바퀴(상위 32비트는 약 17.2 초마다 넘어감): 시각 차이는 모두 넘김을 감안해 계산합니다.
  - 설정이 바뀜(K 줄): 구간을 나눠 따로 계산합니다.

나오는 표 (보고서의 장 번호)
  1) 프레임 수신: 태그마다 앵커 1 의 Poll(프레임의 기준)을 받은 비율, 연달아 놓친 길이, 기록 버림, SD 상태
  2) 앵커 → 태그: 앵커별 Poll 수신율, 앵커 1 의 Poll 을 연달아 잃은 길이
  3) 위치 측정: 태그·앵커별 거리 성공률, 연달아 실패한 길이, 거리 평균과 흩어짐
  4) 태그 → 태그: 차례인 태그의 Response 를 다른 태그가 들은 비율
  5) 태그 → 앵커(Response), 앵커 → 태그(Final): 앵커 기록이 있을 때
  6) 시각: 프레임 간격, 앵커 1 과의 시계 차이(ppm)와 그 변화, Poll 자리, Response 예약 여유, 되돌림
  아래는 2판 기록(P, K 줄)이 있을 때, 설정 구간마다
  7) 드론 → 드론 방송(heartbeat, 추가 슬롯): 방향별 조각 손실률, 방송(조각 둘 다) 손실률, 연달아 잃은 길이
  8) 교환(패킷 3개): 짝별로 단계마다의 성공과 셋 다 된 비율
  9) 일정(5절): 드론별 일정 수신율, 일정을 놓쳐 못 보낸 차례의 비율, 연달아 놓친 길이, 일정 패킷의 수신율
  10) 구간 시각: 예약 시각과 실제 시각의 차이(보낸 것/받은 것, 종류별), 예약이 늦어 못 보낸 수
  11) 무선이 쓴 시간: 프레임당 무선이 끝나는 때와 공중에 있던 시간, 드론 한 대가 1초에 쓴 송신 시간
  12) 시계: 보드 사이 시계 차이(ppm)와 그 변화
  13) 조용한 구간에 쓴 시간, 기록 버림, 제어 계산에 걸린 시간, 받은 패킷이 계산에 쓰이기까지의 시간
  14) 이웃 정보가 오래된 정도: 다른 드론의 방송을 마지막으로 받은 뒤 지난 프레임 수
  15) 전달 확률 추정식의 적중률: 시뮬레이터의 식(mac.py: p <- 0.9 p + 0.1 x 받음, 처음 0.7)을 기록에 돌린 것
  16) 손실 모형의 값: 평균 손실률과 연달아 잃는 평균 길이 (링크 종류별)
  17) 비행 기록(C, E 줄)과 LED(L 줄)의 요약, 명령이 반영되는 지연(C 줄), 모르는 줄

결과 파일 (--out 폴더)
  frame_summary.txt / .json   위 표 전체
  ranges.csv, frames.csv      거리 줄, 프레임 줄
  packets.csv                 P 줄 전부 (태그, 구간, 프레임, 종류, 방향, ...)
  links.csv                   링크별 손실률과 연달아 잃은 길이
  sync_led.csv                LED 가 바뀐 프레임 (영상과 맞추는 데 씀)
  flight.csv, events.csv      C 줄, E 줄 (프레임 번호와, 있으면 PC 시각)
  sim_params.json             시뮬레이터에 넣을 값 (값마다 표본 수. 표본이 없으면 null)

알아 둘 점
  - 손실은 "보낸 쪽의 P..T 줄"과 "받은 쪽의 P..R 줄"을 프레임·슬롯·조각으로 맞춰 셉니다. 보낸 쪽 기록이 없는 태그는
    누가 한 번이라도 들은 적이 있으면 일정(차례대로)으로 보낸 것을 추정하고, 표에 "추정"이라고 표시합니다.
    이때는 그 태그가 앵커 1 의 Poll 을 놓쳐 안 보낸 것도 손실로 셉니다.
  - 기록이 버려지면(F 줄의 "기록 버림", S 줄) 받은 줄이 빠져 손실이 실제보다 크게 나옵니다. 1장의 "기록 버림"이 0 인지 보세요.
  - 보호 시간: sim_params.json 의 guard 는 "어긋남 절댓값의 99 % 값 + --guard-margin-us". 여유를 얼마로 할지는 설계서에
    정해져 있지 않아 기본은 0 입니다.
  - "패킷을 받아 해석을 마칠 때까지의 시간"은 기록에 없습니다. 6장의 "되돌림"(Poll 을 받고 수신을 다시 켤 때까지)이 가장 가까운 값입니다.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import re
import statistics
import sys
import threading
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import twr3_measure as m  # noqa: E402

SEEN_RESPONSE, SEEN_FINAL = 0x08, 0x10
FLAG_COORD_DRONE, FLAG_EXCHANGE = 0x02, 0x04
NOBODY = 15
DT_UNKNOWN = 0x7FFFFFFF
EPOCH = 10 ** 9                 # 프레임 열쇠 = 구간(앵커 1 을 켠 횟수) x EPOCH + 프레임 번호
BACK_OK, AHEAD_OK = 5, 5000     # 프레임 번호가 이만큼 뒤로/앞으로 가는 것은 보통 일로 본다
TAG_BAUD, ANCHOR_BAUD = 921600, 115200
SHR_US = 136 * 1.01763          # 프리앰블 128 + SFD 8 심볼 = 138.4 µs (sim/sim.c 와 같은 값)
POLL_PSDU, RESPONSE_PSDU, FINAL_PSDU = 52, 36, 32   # FCS 포함 (frames.h: Poll 50, Response 34, Final 30 바이트 + 2)
DATA_BYTES = 110                # 방송 조각 (FCS 빼고)
EWMA_START, EWMA_KEEP = 0.7, 0.9  # 시뮬레이터 mac.py 의 _p(): p <- 0.9 p + 0.1 x (받음), 처음 0.7
ANCHOR_F_RE = re.compile(r"^A([123]),(\d+),(\d+),(-?\d+),(-?\d+),([01]),([0-9A-Fa-f]{8}),(-?\d+),(-?\d+),(\d+),(\d+)$")
ANCHOR_INFO_RE = re.compile(r"^A([123])_(BOOT|STAT|READY),(.*)$")
KIND_NAMES = {"H": "heartbeat", "B": "추가 슬롯 방송", "X": "교환", "C": "일정"}
SCRIPT_STATES = {0: "대기", 1: "준비됨", 2: "실행", 3: "끝남", 4: "멈춤", 5: "빠짐"}


def key_of(epoch, frame):
    return epoch * EPOCH + frame


def fnum(key):
    return key % EPOCH


def epoch_of(key):
    return key // EPOCH


def nominal_frame_us(frame_us):
    """앵커 1 이 실제로 쓰는 프레임 길이. 펌웨어는 1 µs 를 63898 DTU 로 환산해(정확히는 63897.6) 20000 µs 가 20000.124 µs 가 된다"""
    return ((int(frame_us) * 63898) >> 8) * m.HI_S * 1e6


def tail_us(psdu_bytes):
    """RMARKER 뒤로 이어지는 시간: PHR 21 심볼 + 데이터(330비트마다 리드솔로몬 48비트). sim/sim.c 의 tail_ps 와 같은 식"""
    bits = 8 * psdu_bytes
    return 21 * 1.02564 + (bits + 48 * ((bits + 329) // 330)) * 0.12821


def airtime_us(psdu_bytes):
    return SHR_US + tail_us(psdu_bytes)


# ---------------------------------------------------------------------------------------------- 기록 읽기
@dataclass
class FrameRec:
    frame: int
    t0_hi: int
    turn: int
    seen: int
    response: str
    lead_us: int
    turn_us: int
    rx_err: int
    host: float | None
    sched: str | None = None       # 2판: A, D, N, -
    extra: int | None = None
    flags: int | None = None
    radio_end_us: int | None = None
    work_us: int | None = None
    work_max_us: int | None = None
    dropped: int | None = None
    ctl_start_us: int | None = None   # 제어 계산을 시작한/끝낸 때 (앵커 1 의 Poll 기준 µs). 안 했으면 둘 다 0
    ctl_end_us: int | None = None
    rx_broadcasts: int | None = None  # 이 프레임에 두 조각을 다 받은 방송 수
    key: int = 0


@dataclass
class RangeRec:
    frame: int
    anchor: int
    mm: int
    valid: int
    end: str
    rssi: int
    fp: int
    poll_hi: int
    end_ns: int


@dataclass
class PacketRec:
    frame: int
    type: str          # H, B, X, C
    dir: str           # T, R
    slot: int
    part: int
    src: int
    dst: int
    serial: int
    bytes: int
    ok: int
    sched_us: int
    dt_ns: int | None  # None = 기준이 없음
    t_hi: int
    rssi: int
    fp: int
    cko: int
    host: float | None
    key: int = 0


@dataclass
class ControlRec:
    frame: int
    seq: int
    flags: int
    extra: int
    post: int          # 10 µs 단위
    frag: int
    xchg: int
    script: int
    command: int
    start: int
    mover: int
    speed: int
    limit: int
    s: int
    d1: int
    d2: int
    host: float | None
    key: int = 0

    def config(self):
        """통신 설정 (대본에 관한 칸은 뺀 것). 이것이 바뀌면 구간을 나눈다"""
        return (self.flags, self.extra, self.post, self.frag, self.xchg, self.s, self.d1, self.d2)


@dataclass
class TagLog:
    path: str
    tag: int | None = None
    tags: int | None = None
    info: dict = field(default_factory=dict)
    frames: dict = field(default_factory=dict)      # 프레임 열쇠 -> FrameRec (구간 0 에서는 열쇠 = 프레임 번호)
    ranges: dict = field(default_factory=dict)      # 프레임 열쇠 -> [RangeRec]
    packets: list = field(default_factory=list)     # PacketRec
    controls: list = field(default_factory=list)    # ControlRec
    leds: list = field(default_factory=list)        # (열쇠, 켬, PC 시각)
    fc: list = field(default_factory=list)          # (열쇠, [C 줄의 칸들], PC 시각)
    events: list = field(default_factory=list)      # (열쇠, 이름, a, b, PC 시각)
    stats: list = field(default_factory=list)
    sd: list = field(default_factory=list)          # D 줄: [프레임, 상태, 다음 섹터, 쓴 섹터, 버린 글자, 오류, ...]
    gaps: list = field(default_factory=list)        # 재시작으로 비는 곳: (꺼지기 전 마지막 열쇠, 켠 뒤 첫 열쇠)
    notes: Counter = field(default_factory=Counter)
    unknown: Counter = field(default_factory=Counter)
    _present: list | None = None
    _present_set: set | None = None

    def present(self):
        """이 태그가 켜져 있던 프레임 열쇠들 (구간마다 처음~끝, 재시작으로 꺼져 있던 사이는 뺌). 정렬된 목록"""
        if self._present is None:
            spans = {}
            for key in self.frames:
                e = epoch_of(key)
                lo, hi = spans.get(e, (key, key))
                spans[e] = (min(lo, key), max(hi, key))
            keys = []
            for e in sorted(spans):
                lo, hi = spans[e]
                holes = [(a, b) for a, b in self.gaps if epoch_of(a) == e and epoch_of(b) == e and b - a > 1]
                if not holes:
                    keys.extend(range(lo, hi + 1))
                else:
                    keys.extend(k for k in range(lo, hi + 1) if not any(a < k < b for a, b in holes))
            self._present = keys
            self._present_set = set(keys)
        return self._present

    def is_present(self, key):
        self.present()
        return key in self._present_set


def _int_or_none(text):
    return None if text == "" else int(text)


def _parse_tag_line(text, host, log):
    """줄 하나를 (종류, 프레임 번호 또는 None, 내용) 으로. 모르는 줄은 None. 깨진 줄은 ValueError"""
    c = text.split(",")
    head = c[0]
    if head == "F" and len(c) in (9, 16, 19):
        rec = FrameRec(int(c[1]), int(c[2], 16), int(c[3]), int(c[4], 16), c[5], int(c[6]), int(c[7]), int(c[8]), host)
        if len(c[5]) != 1 or rec.turn > 15 or rec.frame < 0:
            raise ValueError(text)
        if len(c) >= 16:
            if len(c[9]) != 1:
                raise ValueError(text)
            rec.sched, rec.extra, rec.flags = c[9], int(c[10]), int(c[11], 16)
            rec.radio_end_us, rec.work_us, rec.work_max_us, rec.dropped = int(c[12]), int(c[13]), int(c[14]), int(c[15])
        if len(c) == 19:
            rec.ctl_start_us, rec.ctl_end_us, rec.rx_broadcasts = int(c[16]), int(c[17]), int(c[18])
        return "F", rec.frame, rec
    if head == "R" and len(c) == 10:
        rec = RangeRec(int(c[1]), int(c[2]), int(c[3]), int(c[4]), c[5], int(c[6]), int(c[7]), int(c[8], 16), int(c[9]))
        if rec.anchor not in (1, 2, 3) or rec.frame < 0:
            raise ValueError(text)
        return "R", rec.frame, rec
    if head == "P" and len(c) == 17:
        dt = int(c[12])
        rec = PacketRec(int(c[1]), c[2], c[3], int(c[4]), int(c[5]), int(c[6]), int(c[7]), int(c[8]), int(c[9]),
                        int(c[10]), int(c[11]), None if dt == DT_UNKNOWN else dt, int(c[13], 16), int(c[14]),
                        int(c[15]), int(c[16]), host)
        if rec.type not in KIND_NAMES or rec.dir not in ("T", "R") or not 0 <= rec.src <= 15 or \
                not 0 <= rec.slot <= 5 or not 0 <= rec.part <= 2 or rec.frame < 0:
            raise ValueError(text)
        return "P", rec.frame, rec
    if head == "K" and len(c) == 17:
        v = [int(c[1]), int(c[2]), int(c[3], 16)] + [int(x) for x in c[4:]]
        if v[0] < 0:
            raise ValueError(text)
        return "K", v[0], ControlRec(*v, host)
    if head == "L" and len(c) == 3:
        return "L", int(c[1]), (int(c[2]), host)
    if head == "C" and len(c) == 20:
        values = [_int_or_none(x) for x in c[2:]]
        return "C", int(c[1]), (values, host)
    if head == "E" and len(c) >= 3 and c[2]:
        rest = c[3:] + ["", ""]
        return "E", int(c[1]), (c[2], rest[0], rest[1], host)
    if head == "S" and len(c) == 6:
        log.stats.append([int(x) for x in c[1:]])
        return "-", None, None
    if head == "D" and len(c) == 11:
        log.sd.append([int(x) for x in c[1:]])
        return "-", None, None
    if head in ("F", "R", "P", "K", "L", "C", "E", "S", "D"):
        raise ValueError(text)          # 아는 종류인데 칸 수가 다르다: 잘리거나 깨진 줄
    return None


def parse_tag_log(path):
    log = TagLog(str(path))
    items = []          # (종류, 프레임 번호, 내용)  또는 ("restart", None, None)
    bad = 0
    for host, text in m.read_log(path):
        try:
            if text.startswith("TF_READY"):
                log.notes["restarts"] += 1
                items.append(("restart", None, None))
            elif text.startswith("TF_INFO,"):
                info = dict(kv.split("=", 1) for kv in text.split(",")[1:] if "=" in kv)
                tag, tags = int(info["id"]), int(info["tags"])
                if log.tag is not None and tag != log.tag:
                    log.notes["tag_id_changed"] += 1
                log.info.update(info)
                log.tag, log.tags = tag, tags
            elif text.startswith("TF_SD,"):
                log.info.update(("sd_" + k, v) for k, v in (kv.split("=", 1) for kv in text.split(",")[1:] if "=" in kv))
            elif text.startswith("TF_INIT_FAILED"):
                log.notes["init_failed"] += 1
            else:
                parsed = _parse_tag_line(text, host, log)
                if parsed is None:
                    log.notes["other_lines"] += 1
                    log.unknown[re.split(r"[,=\s]", text, maxsplit=1)[0][:16] or "?"] += 1
                elif parsed[0] != "-":
                    items.append(parsed)
        except (ValueError, KeyError, IndexError):
            bad += 1

    # 프레임 번호의 흐름을 따라가며 구간(앵커 1 을 켠 횟수)을 매긴다. 혼자 튀는 번호는 깨진 줄로 버린다.
    state = {"epoch": 0, "cur": None, "restart_from": None}
    pending = []

    def accept(kind, frame, rec):
        key = key_of(state["epoch"], frame)
        if state["restart_from"] is not None:
            log.gaps.append((state["restart_from"], key))
            state["restart_from"] = None
        if kind == "F":
            rec.key = key
            if key in log.frames:
                log.notes["duplicate_frames"] += 1
            log.frames[key] = rec
        elif kind == "R":
            log.ranges.setdefault(key, []).append(rec)
        elif kind == "P":
            rec.key = key
            log.packets.append(rec)
        elif kind == "K":
            rec.key = key
            log.controls.append(rec)
        elif kind == "L":
            log.leds.append((key, rec[0], rec[1]))
        elif kind == "C":
            log.fc.append((key, rec[0], rec[1]))
        elif kind == "E":
            log.events.append((key, rec[0], rec[1], rec[2], rec[3]))
        state["cur"] = frame if state["cur"] is None else max(state["cur"], frame)

    for kind, frame, rec in items:
        if kind == "restart":
            bad += len(pending)
            pending = []
            if state["cur"] is not None:
                state["restart_from"] = key_of(state["epoch"], state["cur"])
            continue
        cur = state["cur"]
        if cur is None or cur - BACK_OK <= frame <= cur + AHEAD_OK:
            bad += len(pending)                 # 앞의 튄 줄들은 깨진 것이었다
            pending = []
            accept(kind, frame, rec)
            continue
        pending.append((kind, frame, rec))
        base = pending[0][1]
        if len(pending) >= 3 and all(base - BACK_OK <= f <= base + AHEAD_OK for _, f, _ in pending):
            if base < cur - BACK_OK:
                state["epoch"] += 1             # 프레임 번호가 처음으로 돌아갔다 = 앵커 1 이 다시 켜졌다
                log.notes["anchor1_restarts"] += 1
            else:
                log.notes["frame_jumps"] += 1
            state["cur"] = None
            for item in pending:
                accept(*item)
            pending = []
        elif len(pending) >= 3:
            bad += 1                            # 서로도 안 맞는 줄: 가장 오래된 것을 버린다
            pending.pop(0)
    bad += len(pending)
    if bad:
        log.notes["bad_lines"] = bad
    return log


class AnchorLog(defaultdict):
    """{앵커: {프레임 열쇠: (단계, 차례인 태그)}}. info 에 BOOT/STAT 줄의 값"""

    def __init__(self):
        super().__init__(dict)
        self.info = defaultdict(dict)
        self.notes = Counter()


def parse_anchor_log(path):
    """프레임 방식 앵커 기록: {앵커: {프레임 열쇠: (단계, 차례인 태그)}}"""
    out = AnchorLog()
    epoch, cur = defaultdict(int), {}
    for _, text in m.read_log(path):
        g = ANCHOR_F_RE.match(text)
        if g:
            a, frame = int(g[1]), int(g[10])
            last = cur.get(a)
            if last is not None and frame < last - 100:
                epoch[a] += 1
            cur[a] = frame
            out[a][key_of(epoch[a], frame)] = (int(g[3]), int(g[11]))
            continue
        g = ANCHOR_INFO_RE.match(text)
        if g:
            out.info[int(g[1])].update(kv.split("=", 1) for kv in g[3].split(",") if "=" in kv)
            out.notes[f"A{g[1]}_{g[2]}"] += 1
    return out


# ---------------------------------------------------------------------------------------------- 셈
def rate(ok, total):
    return ok / total if total else None


def pct(x, digits=2):
    return "-" if x is None else f"{100 * x:.{digits}f}%"


def runs(flags):
    """성공 True / 실패 False 목록에서 실패가 이어진 길이들"""
    return m.runs_of_failure([flags])


def dist(values, digits=1):
    """평균, 흩어짐, 최소, 중앙, 95 %, 99 %, 최대 (값이 없으면 None)"""
    if not values:
        return None
    v = sorted(values)

    def at(q):
        return v[min(len(v) - 1, int(q * len(v)))]
    return {"n": len(v), "mean": round(statistics.fmean(v), digits), "median": round(statistics.median(v), digits),
            "std": round(statistics.pstdev(v), digits), "min": round(v[0], digits), "p95": round(at(0.95), digits),
            "p99": round(at(0.99), digits), "max": round(v[-1], digits)}


def loss_stats(sequences):
    """sequences: 링크마다의 성공(True)/실패(False) 목록. 평균 손실률, 연달아 잃은 길이의 분포"""
    total = sum(len(s) for s in sequences)
    lost = sum(1 for s in sequences for ok in s if not ok)
    lengths = m.runs_of_failure(sequences)
    return {"sent": total, "lost": lost, "loss_rate": rate(lost, total),
            "mean_burst": round(statistics.fmean(lengths), 3) if lengths else None,
            "max_burst": max(lengths, default=0), "burst_lengths": dict(sorted(Counter(lengths).items()))}


def round_robin(frame, tags, extra):
    """frames.h 의 uwbf_round_robin 과 같은 일정: [(주인, 교환 상대), ...]"""
    turn = frame % tags
    others = tags - 1 if tags > 1 else 1
    out = []
    for j in range(extra):
        if j + 1 >= tags:
            break
        owner = (turn + 1 + j) % tags
        out.append((owner, (owner + 1 + (frame // tags) % others) % tags))
    return out


def slot_times(cfg):
    """설정에서 구간 시각(µs, 앵커 1 의 Poll RMARKER 기준): frames.h 의 "한 프레임의 시각" """
    flags, _, post, frag, xchg, s, d1, d2 = cfg
    position_end = 4 * s + d1 + d2
    sched = position_end + 10 * post
    hb = sched + (10 * frag if flags & FLAG_COORD_DRONE else 0)
    width = 30 * xchg if flags & FLAG_EXCHANGE else 20 * frag
    return {"position_end": position_end, "sched": sched, "heartbeat": hb, "extra0": hb + 20 * frag, "width": width,
            "frag": 10 * frag, "xchg": 10 * xchg}


def expected_us(cfg, kind, slot, part):
    t = slot_times(cfg)
    if kind == "C":
        return t["sched"]
    if kind == "X":
        return t["extra0"] + (slot - 1) * t["width"] + part * t["xchg"]
    return (t["heartbeat"] if slot == 0 else t["extra0"] + (slot - 1) * t["width"]) + part * t["frag"]


def ewma_score(outcomes):
    """시뮬레이터의 추정식을 차례로 돌린다: 다음 결과를 p >= 0.5 면 '받는다'로 예측. (맞힌 수, 수, 제곱 오차의 합)"""
    p, hits, sq = EWMA_START, 0, 0.0
    for ok in outcomes:
        hits += (p >= 0.5) == bool(ok)
        sq += (p - (1.0 if ok else 0.0)) ** 2
        p = EWMA_KEEP * p + (1.0 - EWMA_KEEP) * (1.0 if ok else 0.0)
    return hits, len(outcomes), sq


def baseline_scores(outcomes, window=10):
    """단순한 기준들이 맞힌 수: 늘 받는다 / 직전 결과 / 최근 window 개의 성공률 >= 0.5"""
    always = sum(1 for ok in outcomes if ok)
    last_hits, prev = 0, True
    win_hits, recent = 0, []
    for ok in outcomes:
        last_hits += prev == bool(ok)
        prev = bool(ok)
        guess = (sum(recent) / len(recent) >= 0.5) if recent else True
        win_hits += guess == bool(ok)
        recent = (recent + [1 if ok else 0])[-window:]
    return always, last_hits, win_hits


def _legacy_tag_part(log, tags, frame_us):
    """1~3장, 6장: 태그 하나의 값 (1판부터 있던 계산)"""
    span = log.present()
    got_p1 = [f in log.frames and log.frames[f].t0_hi != 0 for f in span]
    miss_runs = runs(got_p1)
    entry = {
        "frames_first": fnum(span[0]), "frames_last": fnum(span[-1]), "frames": len(span),
        "epochs": sorted({epoch_of(k) for k in span}),
        "master_poll_rate": rate(sum(got_p1), len(span)),
        "frames_without_any_packet": sum(1 for f in span if f not in log.frames),
        "max_consecutive_missed_frames": max(miss_runs, default=0),
        "missed_run_lengths": dict(sorted(Counter(miss_runs).items())),
        "poll_rate_by_anchor": {a: rate(sum(1 for f in span if f in log.frames and log.frames[f].seen & (1 << (a - 1))),
                                        len(span)) for a in (1, 2, 3)},
        "log_dropped": log.stats[-1][1] if log.stats else None,
        "rx_errors": log.stats[-1][2] if log.stats else None,
        "other_mode_packets": log.stats[-1][4] if log.stats else None,
        "restarts": log.notes.get("restarts", 0), "bad_lines": log.notes.get("bad_lines", 0),
        "anchor1_restarts": log.notes.get("anchor1_restarts", 0),
        "sd": {"file": log.info.get("sd_file"), "ready": log.info.get("sd_ready"),
               "state": log.sd[-1][1], "blocks_written": log.sd[-1][3], "dropped_bytes": log.sd[-1][4],
               "write_errors": log.sd[-1][5], "max_write_ms": round(max(x[6] for x in log.sd) / 1000, 1),
               "max_start_us": max(x[7] for x in log.sd), "uart_skipped": log.sd[-1][9]} if log.sd else None,
    }
    turns = [f for f in span if tags and fnum(f) % tags == log.tag]
    ranging = {}
    for a in (1, 2, 3):
        ok, outcomes, mm = [], Counter(), []
        for f in turns:
            r = next((x for x in log.ranges.get(f, []) if x.anchor == a), None)
            fr = log.frames.get(f)
            if r is not None and r.valid and r.end == "F":
                outcomes["ok"] += 1
                mm.append(r.mm)
            elif fr is None or fr.t0_hi == 0:
                outcomes["frame_missed"] += 1            # 앵커 1 의 Poll 을 못 받아 답하지 않음
            elif r is None:
                outcomes["poll_lost"] += 1               # 이 앵커의 Poll 을 못 받음
            elif r.end == "L":
                outcomes["response_late"] += 1
            else:
                outcomes["response_or_final_lost"] += 1
            ok.append(r is not None and bool(r.valid) and r.end == "F")
        fail_runs = runs(ok)
        ranging[a] = {
            "turns": len(turns), "ok": sum(ok), "success_rate": rate(sum(ok), len(turns)),
            "outcomes": dict(outcomes), "max_consecutive_failures": max(fail_runs, default=0),
            "failure_run_lengths": dict(sorted(Counter(fail_runs).items())),
            "distance_mean_m": round(statistics.fmean(mm) / 1000, 4) if mm else None,
            "distance_std_cm": round(statistics.pstdev(mm) / 10, 2) if mm else None, "distances": len(mm),
        }
    entry["ranging"] = ranging
    all3 = [all(any(x.anchor == a and x.valid and x.end == "F" for x in log.ranges.get(f, [])) for a in (1, 2, 3))
            for f in turns]
    entry["position"] = {"turns": len(turns), "ok": sum(all3), "success_rate": rate(sum(all3), len(turns)),
                         "max_consecutive_failures": max(runs(all3), default=0),
                         "max_gap_s": round((max(runs(all3), default=0) + 1) * tags * frame_us * 1e-6, 3) if tags else None}
    # 시각: 프레임 간격(넘김을 감안)과 앵커 1 과의 시계 차이
    steps, prev = [], None
    for f in span:
        fr = log.frames.get(f)
        if fr is None or fr.t0_hi == 0:
            prev = None
            continue
        if prev is not None and f - prev.key == 1:
            steps.append(((fr.t0_hi - prev.t0_hi) % (1 << 32)) * m.HI_S * 1e6)
        prev = fr
    mine = [log.frames[f] for f in turns if f in log.frames and log.frames[f].response in "RN"]
    offsets = {a: [] for a in (2, 3)}
    for f in turns:
        fr = log.frames.get(f)
        for r in log.ranges.get(f, []):
            if fr is not None and fr.t0_hi and r.anchor in offsets:
                offsets[r.anchor].append(((r.poll_hi - fr.t0_hi) % (1 << 32)) * m.HI_S * 1e6)
    # 시계 차이가 시간에 따라 변한 정도: 프레임 간격 500개(10 s)씩 묶어 ppm 을 구한다
    nominal = nominal_frame_us(frame_us)
    windows = [(statistics.fmean(steps[i:i + 500]) / nominal - 1) * 1e6 for i in range(0, len(steps) - 499, 500)]
    entry["timing"] = {
        "frame_step_us": m.describe(steps, digits=3),
        "frame_nominal_us": round(nominal, 4),
        "clock_vs_anchor1_ppm": round((statistics.fmean(steps) / nominal - 1) * 1e6, 2) if steps else None,
        "clock_vs_anchor1_ppm_windows": {"n": len(windows), "min": round(min(windows), 2), "max": round(max(windows), 2),
                                         "first": round(windows[0], 2), "last": round(windows[-1], 2)} if windows else None,
        "poll_offset_us": {a: m.describe(v, digits=2) for a, v in offsets.items()},
        "response_lead_us": m.describe([fr.lead_us for fr in mine], digits=0),
        "turnaround_us": m.describe([fr.turn_us for fr in mine if fr.turn_us], digits=0),
        "responses": dict(Counter(log.frames[f].response for f in turns if f in log.frames)),
    }
    return entry


def _segments(logs):
    """K 줄에서 설정 구간을 만든다: [{"epoch", "from", "to"(열쇠, 양 끝 포함), "config", "control"}]"""
    changes = defaultdict(list)       # 구간(epoch) -> [(열쇠, ControlRec)]
    for log in logs:
        for k in log.controls:
            changes[epoch_of(k.key)].append((k.key, k))
    spans = {}
    for log in logs:
        for key in log.present():
            e = epoch_of(key)
            lo, hi = spans.get(e, (key, key))
            spans[e] = (min(lo, key), max(hi, key))
    out = []
    last = None
    for e in sorted(spans):
        lo, hi = spans[e]
        points = sorted(changes.get(e, []), key=lambda x: x[0])
        cuts = []                      # (시작 열쇠, ControlRec): 설정이 실제로 바뀐 곳만
        if last is not None:
            cuts.append((lo, last))    # 앵커 1 이 같은 설정으로 다시 켜지면 태그는 K 줄을 다시 내지 않는다: 앞의 설정을 잇는다
        for key, k in points:
            if not cuts or cuts[-1][1].config() != k.config():
                cuts.append((key, k))
        if cuts:
            last = cuts[-1][1]
        for i, (key, k) in enumerate(cuts):
            start = lo if i == 0 else key
            end = cuts[i + 1][0] - 1 if i + 1 < len(cuts) else hi
            if end >= start:
                out.append({"epoch": e, "from": start, "to": end, "config": k.config(), "control": k})
    return out


def _segment_part(seg, by_tag, tags, frame_us):
    cfg = seg["config"]
    flags, extra = cfg[0], cfg[1]
    lo, hi = seg["from"], seg["to"]
    times = slot_times(cfg)
    k = seg["control"]
    out = {
        "epoch": seg["epoch"], "frames": [fnum(lo), fnum(hi)],
        "config": {"extra_slots": extra, "extra_type": "exchange" if flags & FLAG_EXCHANGE else "broadcast",
                   "coordinator": "drone0" if flags & FLAG_COORD_DRONE else "anchor1", "post_us": 10 * cfg[2],
                   "frag_us": 10 * cfg[3], "xchg_us": 10 * cfg[4], "S_us": cfg[5], "D1_us": cfg[6], "D2_us": cfg[7],
                   "flags_hex": f"{flags:02X}",
                   "script": k.script, "script_command": k.command, "script_start_frame": k.start,
                   "script_mover": k.mover, "script_speed_cm_s": k.speed, "script_limit_cm_s": k.limit},
        "slot_times_us": times,
    }
    present = {t: [key for key in log.present() if lo <= key <= hi] for t, log in by_tag.items()}
    present_set = {t: set(v) for t, v in present.items()}

    # ---- 보낸 것과 받은 것을 모은다
    sent = {}                          # (열쇠, 종류, 슬롯, 조각, 보낸 태그) -> {"dst", "bytes", "sched_us", "by_sender"}
    received = defaultdict(dict)       # 받은 태그 -> {같은 열쇠: PacketRec}
    sent_frames = set()                # (열쇠, 태그): 그 태그가 보냈다고 스스로 적은 프레임
    late = Counter()                   # (태그, 종류) -> 예약이 늦어 못 보낸 수
    dt = defaultdict(list)             # (방향, 종류) -> 어긋남 ns
    cko = defaultdict(list)            # (받은 태그, 보낸 태그) -> [(열쇠, 값)]
    tx_air = Counter()                 # 태그 -> 송신 시간 µs
    heard_from = Counter()             # 보낸 태그 -> 누가 들은 횟수
    for t, log in by_tag.items():
        for p in log.packets:
            if not lo <= p.key <= hi:
                continue
            ident = (p.key, p.type, p.slot, p.part, p.src)
            if p.dir == "T":
                if p.src != t:
                    continue
                if not p.ok:
                    late[(t, p.type)] += 1
                    continue
                sent[ident] = {"dst": p.dst, "bytes": p.bytes, "sched_us": p.sched_us, "by_sender": True}
                sent_frames.add((p.key, t))
                tx_air[t] += airtime_us(p.bytes + 2)
            else:
                received[t][ident] = p
                heard_from[p.src] += 1
                sent.setdefault(ident, {"dst": p.dst, "bytes": p.bytes,
                                        "sched_us": p.sched_us or expected_us(cfg, p.type, p.slot, p.part),
                                        "by_sender": False})
                cko[(t, p.src)].append((p.key, p.cko))
            if p.dt_ns is not None:
                dt[(p.dir, p.type)].append(p.dt_ns)
    # 보낸 쪽 기록이 없는 태그: 누가 들은 적이 있으면 일정으로 추정한다 (방송만. 일정을 드론이 보내는 설정에서는 하지 않는다)
    estimated = sorted(t for t in range(tags) if t not in by_tag and heard_from[t])
    if estimated and not flags & FLAG_COORD_DRONE:
        for key in range(lo, hi + 1):
            f = fnum(key)
            wanted = [("H", 0, f % tags)] if f % tags in estimated else []
            if not flags & FLAG_EXCHANGE:
                wanted += [("B", j + 1, owner) for j, (owner, _) in enumerate(round_robin(f, tags, extra))
                           if owner in estimated]
            for kind, slot, src in wanted:
                for part in (0, 1):
                    sent.setdefault((key, kind, slot, part, src),
                                    {"dst": NOBODY, "bytes": DATA_BYTES, "by_sender": False,
                                     "sched_us": expected_us(cfg, kind, slot, part)})
    out["senders_estimated_from_schedule"] = estimated

    # ---- 7) 방송: 방향별 조각 손실, 방송(조각 둘 다) 손실
    broadcast = {}
    link_rows = []
    frag_seqs = {"H": [], "B": []}
    msg_seqs = {"H": [], "B": []}
    msg_outcomes = {}                  # (보낸, 받은, 종류) -> [(열쇠, 슬롯, 다 받음)]
    by_src = defaultdict(list)
    for ident in sent:
        by_src[(ident[4], ident[1])].append(ident)
    for (src, kind), idents in sorted(by_src.items()):
        if kind not in "HB":
            continue
        idents.sort()
        for dst in sorted(by_tag):
            if dst == src:
                continue
            seq = [ident in received[dst] for ident in idents if ident[0] in present_set[dst]]
            if not seq:
                continue
            messages = defaultdict(lambda: [False, False])
            for ident in idents:
                if ident[0] in present_set[dst] and ident[3] in (0, 1):
                    messages[(ident[0], ident[2])][ident[3]] = ident in received[dst]
            ordered = sorted(messages.items())
            mseq = [all(v) for _, v in ordered]
            msg_outcomes[(src, dst, kind)] = [(key, slot, all(v)) for (key, slot), v in ordered]
            got = [received[dst][i] for i in idents if i in received[dst]]
            rssi = [p.rssi for p in got if abs(p.rssi) < 32767]
            fp = [p.fp for p in got if abs(p.fp) < 32767]
            fs, ms = loss_stats([seq]), loss_stats([mseq])
            row = {"from": src, "to": dst, "kind": kind, "fragments_sent": fs["sent"], "fragments_lost": fs["lost"],
                   "fragment_loss": fs["loss_rate"], "fragment_max_burst": fs["max_burst"],
                   "messages": ms["sent"], "messages_lost": ms["lost"], "message_loss": ms["loss_rate"],
                   "message_mean_burst": ms["mean_burst"], "message_max_burst": ms["max_burst"],
                   "message_burst_lengths": ms["burst_lengths"],
                   "rssi_dbm": round(statistics.fmean(rssi) / 256, 1) if rssi else None,
                   "first_path_dbm": round(statistics.fmean(fp) / 256, 1) if fp else None,
                   "sender_estimated": src in estimated}
            broadcast[f"{src}->{dst} {kind}"] = row
            link_rows.append(row)
            frag_seqs[kind].append(seq)
            msg_seqs[kind].append(mseq)
    out["broadcast_links"] = broadcast
    out["broadcast_totals"] = {
        kind: {"fragment": loss_stats(frag_seqs[kind]), "message": loss_stats(msg_seqs[kind])} for kind in "HB"}
    out["broadcast_totals"]["all"] = {"fragment": loss_stats(frag_seqs["H"] + frag_seqs["B"]),
                                      "message": loss_stats(msg_seqs["H"] + msg_seqs["B"])}

    # ---- 8) 교환: (열쇠, 슬롯)마다 0 단계를 보낸 쪽이 시작한 태그
    exchanges = defaultdict(Counter)
    exchange_seqs = defaultdict(list)
    for key, _, slot, _, owner in sorted(i for i in sent if i[1] == "X" and i[3] == 0):
        peer = sent[(key, "X", slot, 0, owner)]["dst"]
        if owner not in by_tag and peer not in by_tag:
            continue
        if peer in by_tag and key not in present_set[peer]:
            continue
        got0 = peer in by_tag and (key, "X", slot, 0, owner) in received[peer]
        got1 = owner in by_tag and (key, "X", slot, 1, peer) in received[owner]
        got2 = peer in by_tag and (key, "X", slot, 2, owner) in received[peer]
        if owner not in by_tag:            # 시작한 쪽 기록이 없으면 1 단계가 갔는지 모른다: 2 단계가 왔으면 간 것
            got1 = got2
        c = exchanges[(owner, peer)]
        c["started"] += 1
        if peer not in by_tag:             # 상대 기록이 없으면 1 단계가 온 것까지만 안다
            c["step0"] += bool(got1)
            c["step1"] += bool(got1)
            continue
        c["step0"] += bool(got0)
        c["step1"] += bool(got0 and got1)
        complete = bool(got0 and got1 and got2)
        c["judged"] += 1
        c["complete"] += complete
        exchange_seqs[(owner, peer)].append(complete)
    out["exchange_pairs"] = {
        f"{o}<->{p}": {"owner": o, "partner": p, "started": c["started"], "step0_delivered": c["step0"],
                       "step1_delivered": c["step1"], "complete": c["complete"], "judged": c["judged"],
                       "complete_rate": rate(c["complete"], c["judged"]),
                       "max_consecutive_failures": max(runs(exchange_seqs[(o, p)]), default=0)}
        for (o, p), c in sorted(exchanges.items())}
    total = Counter()
    for c in exchanges.values():
        total.update(c)
    xs = loss_stats(list(exchange_seqs.values()))
    out["exchange_total"] = {"started": total["started"], "judged": total["judged"],
                             "step0_rate": rate(total["step0"], total["started"]),
                             "step1_rate": rate(total["step1"], total["started"]),
                             "complete_rate": rate(total["complete"], total["judged"]),
                             "mean_burst": xs["mean_burst"], "max_burst": xs["max_burst"],
                             "burst_lengths": xs["burst_lengths"]}

    # ---- 9) 일정 (5절)
    schedule = {}
    for t, log in by_tag.items():
        keys = present[t]
        if not keys:
            continue
        has = [(key in log.frames and log.frames[key].sched in ("A", "D")) for key in keys]
        turns_mine = blocked = sent_nothing = 0
        for key, ok in zip(keys, has):
            f = fnum(key)
            if f % tags != t and all(owner != t for owner, _ in round_robin(f, tags, extra)):
                continue
            turns_mine += 1
            if not ok:
                blocked += 1
            elif (key, t) not in sent_frames:
                sent_nothing += 1
        miss = runs(has)
        schedule[t] = {"frames": len(keys), "schedule_rate": rate(sum(has), len(keys)),
                       "anchor1_poll_missed": sum(1 for key in keys if key not in log.frames
                                                  or log.frames[key].sched in ("-", None)),
                       "schedule_packet_missed": sum(1 for key in keys if key in log.frames and log.frames[key].sched == "N"),
                       "my_turns": turns_mine, "turns_blocked_no_schedule": blocked,
                       "blocked_rate": rate(blocked, turns_mine),
                       "turns_with_schedule_but_nothing_sent": sent_nothing,
                       "max_consecutive_without_schedule": max(miss, default=0),
                       "without_schedule_run_lengths": dict(sorted(Counter(miss).items()))}
    sched_packet = {}
    for (src, kind), idents in sorted(by_src.items()):
        if kind != "C":
            continue
        idents.sort()
        for dst in sorted(by_tag):
            if dst == src:
                continue
            seq = [ident in received[dst] for ident in idents if ident[0] in present_set[dst]]
            if seq:
                st = loss_stats([seq])
                sched_packet[f"{src}->{dst}"] = {"sent": st["sent"], "lost": st["lost"], "loss_rate": st["loss_rate"],
                                                 "mean_burst": st["mean_burst"], "max_burst": st["max_burst"]}
    out["schedule"] = schedule
    out["schedule_packet_links"] = sched_packet

    # ---- 10) 구간 시각
    out["slot_error_ns"] = {f"{d}{kind}": {**dist(v, 0), "abs_p99": dist([abs(x) for x in v], 0)["p99"],
                                          "abs_max": max(abs(x) for x in v)}
                            for (d, kind), v in sorted(dt.items())}
    out["late_transmissions"] = {f"tag{t} {kind}": n for (t, kind), n in sorted(late.items())}

    # ---- 11) 무선이 쓴 시간
    drone_by_frame = defaultdict(list)
    for (key, _, _, _, _), info in sent.items():
        drone_by_frame[key].append((info["sched_us"] - SHR_US, info["sched_us"] + tail_us(info["bytes"] + 2)))
    s_us, d1, d2 = cfg[5], cfg[6], cfg[7]
    ends, airs, planned = [], [], []
    for key in range(lo, hi + 1):
        recs = [log.frames[key] for log in by_tag.values() if key in log.frames]
        if not any(r.t0_hi for r in recs):
            continue
        seen = 0
        for r in recs:
            seen |= r.seen
        spans = []
        for a in (1, 2, 3):
            if seen & (1 << (a - 1)):
                spans.append(((a - 1) * s_us - SHR_US, (a - 1) * s_us + tail_us(POLL_PSDU)))
            if seen & (SEEN_FINAL << (a - 1)):
                at = 2 * s_us + d1 + d2 + (a - 1) * s_us
                spans.append((at - SHR_US, at + tail_us(FINAL_PSDU)))
        if seen & SEEN_RESPONSE or any(r.response == "R" for r in recs):
            at = 2 * s_us + d1
            spans.append((at - SHR_US, at + tail_us(RESPONSE_PSDU)))
        spans += drone_by_frame.get(key, [])
        ends.append(max(e for _, e in spans))
        airs.append(sum(e - b for b, e in spans))
        planned += [r.radio_end_us for r in recs if r.radio_end_us]
    out["radio_time"] = {
        "frame_us": frame_us, "limit_us": 0.6 * frame_us,
        "planned_end_us": dist(planned, 0), "actual_end_us": dist(ends, 1), "on_air_us": dist(airs, 1),
        "frames_over_60_percent": sum(1 for e in ends if e > 0.6 * frame_us), "frames": len(ends),
    }
    tx = {}
    for t, log in by_tag.items():
        keys = present[t]
        if not keys:
            continue
        responses = sum(1 for key in keys if key in log.frames and log.frames[key].response == "R")
        total_us = tx_air[t] + responses * airtime_us(RESPONSE_PSDU)
        seconds = len(keys) * frame_us * 1e-6
        tx[t] = {"seconds": round(seconds, 2), "tx_ms_per_s": round(total_us / 1000 / seconds, 3),
                 "duty_percent": round(total_us / (seconds * 1e6) * 100, 3)}
    out["tx_time_per_drone"] = tx

    # ---- 12) 시계: 받은 패킷마다의 시계 차이 (dwt_readclockoffset 값 / 2^26 = 비율)
    clock = {}
    for (rx, src), values in sorted(cko.items()):
        ppm = [v * 1e6 / (1 << 26) for _, v in values]
        row = {"n": len(ppm), "mean_ppm": round(statistics.fmean(ppm), 3), "std_ppm": round(statistics.pstdev(ppm), 3),
               "min_ppm": round(min(ppm), 3), "max_ppm": round(max(ppm), 3), "drift_ppm_per_min": None}
        if len(values) >= 20:
            xs_min = [(key - values[0][0]) * frame_us * 1e-6 / 60.0 for key, _ in values]
            mx, my = statistics.fmean(xs_min), statistics.fmean(ppm)
            den = sum((x - mx) ** 2 for x in xs_min)
            if den > 0:
                row["drift_ppm_per_min"] = round(sum((x - mx) * (y - my) for x, y in zip(xs_min, ppm)) / den, 4)
        clock[f"{rx}<-{src}"] = row
    out["clock_offset_between_tags"] = clock

    # ---- 13) 조용한 구간에 쓴 시간
    quiet = {}
    for t, log in by_tag.items():
        recs = [log.frames[key] for key in present[t] if key in log.frames and log.frames[key].work_us is not None]
        if recs:
            dropped = [r.dropped for r in recs]
            quiet[t] = {"work_us": dist([r.work_us for r in recs], 0), "work_max_us": dist([r.work_max_us for r in recs], 0),
                        "available_us": dist([frame_us - r.radio_end_us for r in recs if r.radio_end_us], 0),
                        "log_dropped_first": dropped[0], "log_dropped_last": dropped[-1],
                        "log_dropped_wraps": sum(1 for a, b in zip(dropped, dropped[1:]) if b < a)}
    out["quiet_work"] = quiet

    # ---- 13-2) 제어 계산: 걸린 시간, 그리고 받은 패킷이 계산에 쓰이기까지의 시간
    compute = {}
    all_compute, all_wait = [], []
    for t, log in by_tag.items():
        ran = {key: log.frames[key] for key in present[t]
               if key in log.frames and log.frames[key].ctl_end_us and log.frames[key].ctl_end_us >= log.frames[key].ctl_start_us}
        if not ran:
            continue
        took = [r.ctl_end_us - r.ctl_start_us for r in ran.values()]
        waits, next_frame, unused = [], 0, 0
        for p in received[t].values():
            if p.dt_ns is None:
                continue
            arrival = p.sched_us + p.dt_ns / 1000.0
            if p.key in ran and ran[p.key].ctl_start_us >= arrival:
                waits.append(ran[p.key].ctl_start_us - arrival)
            elif p.key + 1 in ran:                 # 그 프레임의 계산이 이미 시작됐거나 없었다: 다음 프레임의 계산에 쓰인다
                waits.append(frame_us - arrival + ran[p.key + 1].ctl_start_us)
                next_frame += 1
            else:
                unused += 1
        got = [log.frames[key].rx_broadcasts for key in present[t]
               if key in log.frames and log.frames[key].rx_broadcasts is not None]
        compute[t] = {"frames_computed": len(ran), "compute_us": dist(took, 0),
                      "start_us": dist([r.ctl_start_us for r in ran.values()], 0),
                      "end_us": dist([r.ctl_end_us for r in ran.values()], 0),
                      "packet_to_use_us": dist(waits, 0), "packets_used_next_frame": next_frame,
                      "packets_without_following_compute": unused,
                      "broadcasts_received_per_frame": dist(got, 2)}
        all_compute += took
        all_wait += waits
    out["control_compute"] = compute
    out["control_compute_all"] = {"compute_us": dist(all_compute, 0), "packet_to_use_us": dist(all_wait, 0)}

    # ---- 14) 이웃 정보가 오래된 정도 (방송을 조각 둘 다 받은 프레임 기준)
    ages, all_ages = {}, []
    for dst in sorted(by_tag):
        mine = []
        for src in range(tags):
            if src == dst:
                continue
            got = sorted({key for kind in "HB" for key, _, ok in msg_outcomes.get((src, dst, kind), []) if ok})
            if not got:
                continue
            i = 0
            for key in present[dst]:
                if key < got[0]:
                    continue
                while i + 1 < len(got) and got[i + 1] <= key:
                    i += 1
                mine.append(key - got[i])
        if mine:
            d = dist(mine, 2)
            ages[dst] = {**d, "seconds_p99": round(d["p99"] * frame_us * 1e-6, 3),
                         "seconds_max": round(d["max"] * frame_us * 1e-6, 3)}
            all_ages += mine
    out["neighbor_age_frames"] = ages
    out["neighbor_age_all"] = ({**dist(all_ages, 2),
                                "histogram(50 = 50 이상)": dict(sorted(Counter(min(a, 50) for a in all_ages).items()))}
                               if all_ages else None)

    # ---- 15) 전달 확률 추정식
    est = {}
    for label, kinds in (("heartbeat", "H"), ("all_broadcasts", "HB")):
        links = defaultdict(list)
        for (src, dst, kind), items in msg_outcomes.items():
            if kind in kinds:
                links[(src, dst)] += items
        hits = n = always = last = win = 0
        sq = 0.0
        per_link = []
        for items in links.values():
            seq = [ok for _, _, ok in sorted(items)]
            h, c, s2 = ewma_score(seq)
            a, la, w = baseline_scores(seq)
            hits, n, sq, always, last, win = hits + h, n + c, sq + s2, always + a, last + la, win + w
            if c:
                per_link.append(h / c)
        est[label] = {"predictions": n, "ewma_hit_rate": rate(hits, n), "ewma_brier": round(sq / n, 4) if n else None,
                      "ewma_hit_rate_worst_link": min(per_link) if per_link else None,
                      "always_success_hit_rate": rate(always, n), "last_outcome_hit_rate": rate(last, n),
                      "recent10_hit_rate": rate(win, n)}
    out["delivery_estimator"] = est
    out["_link_rows"] = link_rows
    return out


def command_response(log, frame_us, min_step_mm_s=50.0, window_s=5.0):
    """C 줄에서: 명령 속도가 바뀐 프레임부터 비행 제어기 속도가 변하기 시작할 때(변화 폭의 10 %)와 63 % 에 이를 때까지의 ms.
    명령과 비행 제어기 속도의 좌표축이 달라도 되게, 속도 변화의 크기 |v - v0| 를 명령 변화의 크기와 견준다.
    비행 제어기 값은 그 줄의 "위치 나이"만큼 앞선 때의 것으로 본다. [{frame, step_mm_s, start_ms, t63_ms}, ...]"""
    rows = sorted((key, v) for key, v, _ in log.fc)
    out, prev_cmd = [], None
    window = int(window_s * 1e6 / frame_us)
    for i, (key, v) in enumerate(rows):
        cmd = (v[15], v[16])
        if cmd[0] is None or cmd[1] is None:
            continue
        if prev_cmd is None or cmd == prev_cmd:
            prev_cmd = cmd
            continue
        step = math.hypot(cmd[0] - prev_cmd[0], cmd[1] - prev_cmd[1])
        prev_cmd = cmd
        if step < min_step_mm_s:
            continue
        base = next(((u[3], u[4]) for _, u in reversed(rows[:i + 1]) if u[3] is not None and u[4] is not None), None)
        if base is None:
            continue
        start = t63 = None
        for key2, u in rows[i + 1:]:
            if key2 - key > window or epoch_of(key2) != epoch_of(key):
                break
            if (u[15], u[16]) != cmd:
                break                               # 다음 명령이 나왔다
            if u[3] is None or u[4] is None:
                continue
            moved = math.hypot(u[3] - base[0], u[4] - base[1])
            when = (key2 - key) * frame_us / 1000.0 - (u[9] or 0)
            if start is None and moved >= 0.1 * step:
                start = max(0.0, when)
            if moved >= 0.63 * step:
                t63 = max(0.0, when)
                break
        out.append({"frame": fnum(key), "step_mm_s": round(step, 1), "start_ms": start, "t63_ms": t63})
    return out


def _sim_params(summary, guard_margin_us):
    """시뮬레이터에 넣을 값. 값마다 표본 수(n). 표본이 없으면 null"""
    segs = summary.get("segments", [])
    seg = max(segs, key=lambda s: s["frames"][1] - s["frames"][0]) if segs else None
    out = {"note": "값은 가장 긴 설정 구간에서 구했다. 구간이 여럿이면 frame_summary.json 의 segments 를 볼 것",
           "segment": {"epoch": seg["epoch"], "frames": seg["frames"], "config": seg["config"]} if seg else None}
    if seg:
        c, t = seg["config"], seg["slot_times_us"]
        frag_n = seg["broadcast_totals"]["all"]["fragment"]["sent"] or None
        out["packet_time_us"] = {
            "how": "프리앰블+SFD 138.4 µs + PHR + 데이터(리드솔로몬 포함). 길이로 계산한 값이지 잰 값이 아니다. "
                   "n 은 이 구간에서 오간 방송 조각의 수",
            "broadcast_fragment_112B": round(airtime_us(DATA_BYTES + 2), 1) if frag_n else None,
            "broadcast_two_fragments": round(c["frag_us"] + airtime_us(DATA_BYTES + 2), 1) if frag_n else None,
            "schedule_packet_25B": round(airtime_us(25), 1) if seg["schedule_packet_links"] else None,
            "n": frag_n}
        xn = seg["exchange_total"]["started"] or None
        out["exchange_time_us"] = {"how": "패킷 3개: 2 x xchg + 마지막 패킷의 시간 (설정과 길이로 계산)",
                                   "packet_62B": round(airtime_us(62), 1) if xn else None,
                                   "value": round(2 * c["xchg_us"] + airtime_us(62), 1) if xn else None, "n": xn}
        measured = summary.get("position_end_measured_us")
        out["position_time_us"] = {
            "how": "7패킷 한 묶음: 첫 Poll 의 프리앰블부터 마지막 Final 끝까지. 계산값 = 4S + D1 + D2 + 138.4 + Final 꼬리. "
                   "measured 는 R 줄에서 잰 '마지막 Final 이 온 때'(Poll 기준)에 같은 앞뒤를 더한 것",
            "value": round(t["position_end"] + SHR_US + tail_us(FINAL_PSDU), 1),
            "measured_mean": round(measured["mean"] + SHR_US + tail_us(FINAL_PSDU), 1) if measured else None,
            "measured_max": round(measured["max"] + SHR_US + tail_us(FINAL_PSDU), 1) if measured else None,
            "n": measured["n"] if measured else None}
        errors = seg["slot_error_ns"]
        worst = max((v["abs_p99"] for v in errors.values()), default=None)
        out["guard_time_us"] = {
            "how": "모든 종류의 패킷에서 |실제 시각 - 예약 시각| 의 99 % 값 가운데 가장 큰 것 + 여유(--guard-margin-us). "
                   "여유의 크기는 설계서에 정해져 있지 않아 기본 0",
            "abs_error_p99_ns": worst, "abs_error_max_ns": max((v["abs_max"] for v in errors.values()), default=None),
            "margin_us": guard_margin_us,
            "value": round(worst / 1000 + guard_margin_us, 3) if worst is not None else None,
            "n": sum(v["n"] for v in errors.values()) or None}
        totals = seg["broadcast_totals"]

        def model(st):
            return {"mean_loss": round(st["loss_rate"], 5) if st["loss_rate"] is not None else None,
                    "mean_burst": st["mean_burst"], "max_burst": st["max_burst"] if st["sent"] else None,
                    "n": st["sent"] or None}
        out["loss_model"] = {
            "how": "mean_loss = 잃은 수 / 보낸 수, mean_burst = 연달아 잃은 길이의 평균 (링크마다 차례대로 세어 모음)",
            "drone_to_drone_fragment": model(totals["all"]["fragment"]),
            "drone_to_drone_broadcast": model(totals["all"]["message"]),
            "heartbeat_broadcast": model(totals["H"]["message"]),
            "extra_slot_broadcast": model(totals["B"]["message"]),
            "exchange_complete": {"success_rate": seg["exchange_total"]["complete_rate"],
                                  "mean_burst": seg["exchange_total"]["mean_burst"],
                                  "n": seg["exchange_total"]["judged"] or None},
        }
        cc = seg["control_compute_all"]
        out["compute_time_us"] = {"how": "F 줄의 제어 계산 끝 - 시작 (계산을 한 프레임만)",
                                  **({k: cc["compute_us"][k] for k in ("mean", "p99", "max", "n")} if cc["compute_us"]
                                     else {"mean": None, "p99": None, "max": None, "n": None})}
        out["packet_to_use_us"] = {
            "how": "제어 계산 시작 - 패킷이 온 때(P 줄의 예약 µs + 어긋남). 계산 시작 뒤에 온 패킷은 다음 프레임의 계산까지로 셈",
            **({k: cc["packet_to_use_us"][k] for k in ("mean", "p99", "max", "n")} if cc["packet_to_use_us"]
               else {"mean": None, "p99": None, "max": None, "n": None})}
    else:
        for name in ("packet_time_us", "exchange_time_us", "position_time_us", "guard_time_us", "loss_model",
                     "compute_time_us", "packet_to_use_us"):
            out[name] = None
    starts = [x["start_ms"] for v in summary.get("command_response", {}).values() for x in v["each"] if x["start_ms"] is not None]
    t63s = [x["t63_ms"] for v in summary.get("command_response", {}).values() for x in v["each"] if x["t63_ms"] is not None]
    out["command_delay_ms"] = {
        "how": "C 줄: 명령 속도가 바뀐 프레임부터 비행 제어기 속도가 변화 폭의 10 % 를 넘을 때(start)와 63 % 에 이를 때(t63)까지. "
               "비행 제어기 값이 초당 몇 번만 오므로 그 간격만큼 거칠다",
        "start_mean": round(statistics.fmean(starts), 1) if starts else None, "start_max": max(starts) if starts else None,
        "t63_mean": round(statistics.fmean(t63s), 1) if t63s else None, "t63_max": max(t63s) if t63s else None,
        "n": len(starts) or None}
    tags = summary["tag"]
    poll = {}
    frames = sum(e["frames"] for e in tags.values())
    for a in (1, 2, 3):
        got = sum(round(e["poll_rate_by_anchor"][a] * e["frames"]) for e in tags.values())
        poll[f"anchor{a}"] = {"mean_loss": round(1 - got / frames, 5) if frames else None, "n": frames or None}
    a1 = summary.get("anchor1_poll_loss") or {}
    links = summary.get("anchor_links", {})
    up = [(v["response_heard"], v["response_sent"]) for name, v in links.items() if "->A" in name]
    down = [(v["final_heard"], v["final_sent"]) for name, v in links.items() if "final" in name]
    up_n, down_n = sum(n for _, n in up), sum(n for _, n in down)
    out["anchor_loss"] = {
        "anchor_to_drone_poll": poll,
        "anchor1_poll_mean_burst": a1.get("mean_burst"), "anchor1_poll_max_burst": a1.get("max_burst") if frames else None,
        "drone_to_anchor_response": {"mean_loss": round(1 - sum(x for x, _ in up) / up_n, 5) if up_n else None,
                                     "n": up_n or None, "how": "앵커 기록이 있어야 나온다"},
        "anchor_to_drone_final": {"mean_loss": round(1 - sum(x for x, _ in down) / down_n, 5) if down_n else None,
                                  "n": down_n or None, "how": "앵커 기록이 있어야 나온다"},
        "position_success_rate": {str(t): e["position"]["success_rate"] for t, e in tags.items()},
    }
    pair = seg["clock_offset_between_tags"] if seg else {}
    pair_means = [abs(v["mean_ppm"]) for v in pair.values()]
    drifts = [abs(v["drift_ppm_per_min"]) for v in pair.values() if v["drift_ppm_per_min"] is not None]
    out["clock_error"] = {
        "tag_vs_anchor1_ppm": {str(t): e["timing"]["clock_vs_anchor1_ppm"] for t, e in tags.items()},
        "tag_vs_anchor1_ppm_how": "프레임 간격(태그의 DW 시계로 잰 값) / 앵커 1 이 실제로 쓰는 프레임 길이 - 1. "
                                  "그 길이는 20000 µs 설정에서 20000.124 µs (펌웨어가 1 µs 를 63898 DTU 로 환산)",
        "frame_actual_us": round(nominal_frame_us(summary["frame_us"]), 4),
        "between_tags_abs_ppm_max": round(max(pair_means), 3) if pair_means else None,
        "between_tags_abs_ppm_mean": round(statistics.fmean(pair_means), 3) if pair_means else None,
        "between_tags_drift_abs_ppm_per_min_max": round(max(drifts), 4) if drifts else None,
        "n_pairs": len(pair_means) or None,
    }
    return out


def analyze(tag_logs, anchor_logs, frame_us=20000.0, guard_margin_us=0.0):
    by_tag, duplicates = {}, []
    for log in tag_logs:
        if log.tag is None or not log.frames:
            continue
        if log.tag in by_tag:
            duplicates.append(log.path)       # 같은 태그의 기록이 둘이면 프레임이 더 많은 것을 쓴다
            if len(log.frames) <= len(by_tag[log.tag].frames):
                continue
        by_tag[log.tag] = log
    by_tag = {t: by_tag[t] for t in sorted(by_tag)}
    logs = list(by_tag.values())
    tags = max((log.tags or 0) for log in logs) if logs else 0
    anchors, anchor_info = defaultdict(dict), {}
    for a in anchor_logs:
        for k, v in a.items():
            anchors[k].update(v)
        for k, v in getattr(a, "info", {}).items():
            anchor_info.setdefault(k, {}).update(v)
    summary = {"tags_configured": tags, "tags_logged": sorted(by_tag), "frame_us": frame_us, "tag": {}, "notes": {}}
    if duplicates:
        summary["notes"]["같은 태그의 기록이 여럿 (프레임이 많은 것만 씀)"] = duplicates
    epochs = {t: sorted({epoch_of(k) for k in log.frames}) for t, log in by_tag.items()}
    if len({tuple(v) for v in epochs.values()}) > 1:
        summary["notes"]["기록마다 본 앵커 1 재시작 횟수가 다름 (구간이 어긋날 수 있음)"] = {str(t): v for t, v in epochs.items()}

    for log in logs:
        summary["tag"][log.tag] = _legacy_tag_part(log, tags, frame_us)

    # 앵커 1 의 Poll: 연달아 잃은 길이 (손실 모형)
    st = loss_stats([[f in log.frames and log.frames[f].t0_hi != 0 for f in log.present()] for log in logs])
    summary["anchor1_poll_loss"] = st
    # 위치 측정 묶음이 끝나는 때(마지막 Final 의 RMARKER, 앵커 1 의 Poll 기준 µs): R 줄에서 잰 값
    ends = []
    for log in logs:
        for key, rs in log.ranges.items():
            fr = log.frames.get(key)
            r3 = next((r for r in rs if r.anchor == 3 and r.end == "F"), None)
            if fr is not None and fr.t0_hi and r3 is not None:
                ends.append(((r3.poll_hi - fr.t0_hi) % (1 << 32)) * m.HI_S * 1e6 + r3.end_ns / 1000.0)
    summary["position_end_measured_us"] = dist(ends, 2)

    # 태그 -> 태그: 차례인 태그 j 가 Response 를 보낸 프레임에서, 태그 i 가 그것을 들었는가
    matrix = {}
    for j, sender in by_tag.items():
        sent = [f for f, fr in sender.frames.items() if fr.turn == j and fr.response == "R"]
        for i, listener in by_tag.items():
            if i == j:
                continue
            in_range = [f for f in sent if listener.is_present(f)]
            heard = sum(1 for f in in_range if f in listener.frames and listener.frames[f].seen & SEEN_RESPONSE)
            matrix[f"{j}->{i}"] = {"sent": len(in_range), "heard": heard, "rate": rate(heard, len(in_range))}
    summary["tag_to_tag_response"] = matrix

    # 앵커 기록이 있으면: 태그 -> 앵커 (Response), 앵커 -> 태그 (Final)
    links = {}
    for a, frames in sorted(anchors.items()):
        for j, sender in by_tag.items():
            sent = [f for f, fr in sender.frames.items() if fr.turn == j and fr.response == "R" and f in frames]
            listed = [f for f in sent if any(x.anchor == a for x in sender.ranges.get(f, []))]   # 태그가 이 앵커의 Poll 을 받아 회차를 실음
            got = sum(1 for f in listed if frames[f][0] & 2)
            finals = [f for f in listed if frames[f][0] & 4]
            final_ok = sum(1 for f in finals if any(x.anchor == a and x.end == "F" for x in sender.ranges.get(f, [])))
            links[f"tag{j}->A{a}"] = {"response_sent": len(listed), "response_heard": got, "rate": rate(got, len(listed))}
            links[f"A{a}->tag{j} final"] = {"final_sent": len(finals), "final_heard": final_ok,
                                           "rate": rate(final_ok, len(finals))}
    summary["anchor_links"] = links
    summary["anchor_logs"] = {a: len(v) for a, v in anchors.items()}
    summary["anchor_info"] = {str(a): v for a, v in sorted(anchor_info.items())}

    # 2판: 설정 구간마다
    summary["segments"] = [_segment_part(seg, by_tag, tags, frame_us) for seg in (_segments(logs) if tags else [])]

    # 비행 기록, LED, 모르는 줄
    flight = {}
    for t, log in by_tag.items():
        if not (log.fc or log.events or log.leds):
            continue
        states = Counter(SCRIPT_STATES.get(v[13], str(v[13])) for _, v, _ in log.fc if v[13] is not None)
        speeds = [math.hypot(v[3], v[4]) / 1000.0 for _, v, _ in log.fc if v[3] is not None and v[4] is not None]
        flight[t] = {"c_lines": len(log.fc), "commands_sent": sum(1 for _, v, _ in log.fc if v[17]),
                     "script_states": dict(states), "speed_max_m_s": round(max(speeds), 3) if speeds else None,
                     "events": dict(Counter(name for _, name, _, _, _ in log.events)),
                     "led_changes": len(log.leds),
                     "led_on_frames": [fnum(key) for key, on, _ in log.leds if on][:20]}
    summary["flight"] = flight
    response = {}
    for t, log in by_tag.items():
        if log.fc:
            steps = command_response(log, frame_us)
            response[t] = {"steps": len(steps), "start_ms": dist([x["start_ms"] for x in steps if x["start_ms"] is not None], 0),
                           "t63_ms": dist([x["t63_ms"] for x in steps if x["t63_ms"] is not None], 0),
                           "no_response_seen": sum(1 for x in steps if x["start_ms"] is None), "each": steps[:50]}
    summary["command_response"] = response
    unknown = Counter()
    for log in logs:
        unknown.update(log.unknown)
    summary["unknown_lines"] = dict(unknown.most_common())
    summary["line_notes"] = {str(t): dict(log.notes) for t, log in by_tag.items() if log.notes}
    summary["sim_params"] = _sim_params(summary, guard_margin_us)
    return summary


# ---------------------------------------------------------------------------------------------- 보고서
def _matrix(L, title, cells, tags, value):
    L.append(title + " | " + " | ".join(f"→{i}" for i in tags))
    for j in tags:
        if any((j, i) in cells for i in tags):
            L.append(f"{j} | " + " | ".join("·" if (j, i) not in cells else value(cells[(j, i)]) for i in tags))


def _report_segment(L, n, seg, many):
    c = seg["config"]
    L.append("")
    L.append(f"######## 설정 구간 {n}" + (f" (앵커 1 을 켠 횟수로 {seg['epoch'] + 1}번째)" if many else "") +
             f": 프레임 {seg['frames'][0]}~{seg['frames'][1]} ########")
    L.append(f"추가 슬롯 {c['extra_slots']}개 ({'교환' if c['extra_type'] == 'exchange' else '방송'}), 일정을 보내는 쪽 "
             f"{'드론 0번' if c['coordinator'] == 'drone0' else '앵커 1'}, post {c['post_us']} frag {c['frag_us']} "
             f"xchg {c['xchg_us']} µs, S/D1/D2 {c['S_us']}/{c['D1_us']}/{c['D2_us']} µs, "
             f"대본 {c['script']} (명령 {c['script_command']}, 시작 프레임 {c['script_start_frame']}, "
             f"속도 {c['script_speed_cm_s']} cm/s, 한도 {c['script_limit_cm_s']} cm/s, 움직일 태그 "
             f"{'모두' if c['script_mover'] == 255 else c['script_mover']})")
    t = seg["slot_times_us"]
    L.append(f"구간 시각 µs: 위치 측정 끝 {t['position_end']}, heartbeat {t['heartbeat']}, 첫 추가 슬롯 {t['extra0']}, "
             f"추가 슬롯 폭 {t['width']}")
    if seg["senders_estimated_from_schedule"]:
        L.append(f"! 보낸 쪽 기록이 없어 일정으로 추정한 태그: {seg['senders_estimated_from_schedule']} "
                 "(이 태그가 앵커 1 의 Poll 을 놓쳐 안 보낸 것도 손실로 셈)")

    links = seg["broadcast_links"]
    L.append("")
    L.append("== 7. 드론 → 드론 방송 (보낸 태그 → 받은 태그) ==")
    if not links:
        L.append("방송 기록(P 줄) 없음")
    for kind in "HB":
        cells = {(v["from"], v["to"]): v for v in links.values() if v["kind"] == kind}
        if not cells:
            continue
        tags = sorted({x for pair in cells for x in pair})
        L.append(f"-- {KIND_NAMES[kind]}: 방송(조각 둘 다) 손실률 --")
        _matrix(L, "보낸 태그", cells, tags, lambda v: pct(v["message_loss"], 1))
        L.append(f"-- {KIND_NAMES[kind]}: 조각 손실률 / 방송을 연달아 잃은 최대 --")
        _matrix(L, "보낸 태그", cells, tags, lambda v: f"{pct(v['fragment_loss'], 1)} / {v['message_max_burst']}")
        tot = seg["broadcast_totals"][kind]
        L.append(f"   합계: 조각 {tot['fragment']['sent']}개 중 손실 {pct(tot['fragment']['loss_rate'])}, 방송 "
                 f"{tot['message']['sent']}개 중 손실 {pct(tot['message']['loss_rate'])}, 연달아 잃은 길이 평균 "
                 f"{tot['message']['mean_burst']} 최대 {tot['message']['max_burst']} 분포 {tot['message']['burst_lengths']}")

    if seg["exchange_pairs"]:
        L.append("")
        L.append("== 8. 교환 (패킷 3개: 시작한 태그 → 상대 → 시작한 태그) ==")
        L.append("시작한 태그 ↔ 상대 | 시작 | 0단계 도착 | 1단계까지 | 셋 다 | 셋 다 된 비율 | 연달아 실패 최대")
        for name, v in seg["exchange_pairs"].items():
            L.append(f"{name} | {v['started']} | {v['step0_delivered']} | {v['step1_delivered']} | {v['complete']} | "
                     f"{pct(v['complete_rate'])} | {v['max_consecutive_failures']}")
        e = seg["exchange_total"]
        L.append(f"   합계: 시작 {e['started']}, 0단계 {pct(e['step0_rate'])}, 1단계까지 {pct(e['step1_rate'])}, "
                 f"셋 다 {pct(e['complete_rate'])}, 연달아 실패한 길이 평균 {e['mean_burst']} 최대 {e['max_burst']}")

    L.append("")
    L.append("== 9. 일정 ==")
    L.append("태그 | 프레임 수 | 일정 수신율 | 앵커 1 의 Poll 을 놓침 | 일정 패킷만 놓침 | 내가 보낼 차례 | 일정이 없어 못 보냄 | "
             "그 비율 | 일정은 있는데 보낸 기록 없음 | 연달아 일정 없음 최대")
    for tag, v in seg["schedule"].items():
        L.append(f"{tag} | {v['frames']} | {pct(v['schedule_rate'])} | {v['anchor1_poll_missed']} | "
                 f"{v['schedule_packet_missed']} | {v['my_turns']} | {v['turns_blocked_no_schedule']} | "
                 f"{pct(v['blocked_rate'])} | {v['turns_with_schedule_but_nothing_sent']} | "
                 f"{v['max_consecutive_without_schedule']}")
    if seg["schedule_packet_links"]:
        L.append("일정 패킷 (보낸 태그 → 받은 태그) | 보냄 | 손실률 | 연달아 잃은 평균 | 최대")
        for name, v in seg["schedule_packet_links"].items():
            L.append(f"{name} | {v['sent']} | {pct(v['loss_rate'])} | {v['mean_burst']} | {v['max_burst']}")

    L.append("")
    L.append("== 10. 구간 시각: 실제 시각 - 예약 시각 (ns) ==")
    L.append("방향·종류 | 수 | 평균 | 흩어짐 | 최소 | 99 % | 최대 | 절댓값의 99 % | 절댓값의 최대")
    names = {"T": "보냄", "R": "받음"}
    for name, v in seg["slot_error_ns"].items():
        L.append(f"{names[name[0]]} {KIND_NAMES[name[1]]} | {v['n']} | {v['mean']} | {v['std']} | {v['min']} | {v['p99']} | "
                 f"{v['max']} | {v['abs_p99']} | {v['abs_max']}")
    L.append("예약이 늦어 못 보낸 것: " + (", ".join(f"{k} {v}번" for k, v in seg["late_transmissions"].items()) or "없음"))

    r = seg["radio_time"]
    L.append("")
    L.append(f"== 11. 무선이 쓴 시간 (프레임 {r['frame_us'] / 1000:.0f} ms, 한도 60 % = {r['limit_us']:.0f} µs) ==")
    if r["actual_end_us"]:
        a, o = r["actual_end_us"], r["on_air_us"]
        L.append(f"무선이 끝나는 때 µs (기록된 패킷으로 계산): 평균 {a['mean']}, 99 % {a['p99']}, 최대 {a['max']}  |  "
                 f"펌웨어가 계산한 값의 최대: {r['planned_end_us']['max'] if r['planned_end_us'] else '-'}")
        L.append(f"공중에 있던 시간의 합 µs: 평균 {o['mean']}, 99 % {o['p99']}, 최대 {o['max']}  |  "
                 f"60 % 를 넘은 프레임 {r['frames_over_60_percent']} / {r['frames']}")
    L.append("태그 | 기록된 시간 s | 1초에 쓴 송신 시간 ms | 비율 %")
    for tag, v in seg["tx_time_per_drone"].items():
        L.append(f"{tag} | {v['seconds']} | {v['tx_ms_per_s']} | {v['duty_percent']}")

    L.append("")
    L.append("== 12. 보드 사이 시계 차이 (ppm. 칩이 패킷마다 알려 준 값의 평균. 줄 = 받은 태그, 칸 = 보낸 태그) ==")
    clock = seg["clock_offset_between_tags"]
    if clock:
        cells = {tuple(int(x) for x in name.split("<-")): v for name, v in clock.items()}
        ids = sorted({x for pair in cells for x in pair})
        L.append("받은 태그 | " + " | ".join(f"←{j}" for j in ids))
        for i in ids:
            if any((i, j) in cells for j in ids):
                L.append(f"{i} | " + " | ".join("·" if (i, j) not in cells else f"{cells[(i, j)]['mean_ppm']:.2f}" for j in ids))
        drifts = [abs(v["drift_ppm_per_min"]) for v in clock.values() if v["drift_ppm_per_min"] is not None]
        L.append(f"짝 {len(clock)}개: 패킷마다의 흩어짐 최대 {max(v['std_ppm'] for v in clock.values())} ppm, "
                 f"시간에 따른 변화의 최대 {max(drifts) if drifts else '-'} ppm/분 (짝별 값은 frame_summary.json)")
    else:
        L.append("받은 패킷 없음")

    L.append("")
    L.append("== 13. 조용한 구간과 제어 계산 ==")
    L.append("태그 | 쓸 수 있는 시간 µs (프레임 - 무선 끝, 최소) | 쓴 시간 µs: 평균 / 99 % / 최대 | 한 번에 가장 길게 µs: 99 % / 최대 | "
             "기록 버림 (처음 → 끝)")
    for tag, v in seg["quiet_work"].items():
        w, x, av = v["work_us"], v["work_max_us"], v["available_us"]
        L.append(f"{tag} | {av['min'] if av else '-'} | {w['mean']} / {w['p99']} / {w['max']} | {x['p99']} / {x['max']} | "
                 f"{v['log_dropped_first']} → {v['log_dropped_last']}" +
                 (f" (65536 을 {v['log_dropped_wraps']}번 넘김)" if v["log_dropped_wraps"] else ""))

    if seg["control_compute"]:
        L.append("태그 | 제어 계산을 한 프레임 | 걸린 시간 µs: 평균 / 99 % / 최대 | 시작한 때 µs: 평균 / 최대 | 끝난 때 µs: 최대 | "
                 "받은 패킷이 쓰이기까지 µs: 평균 / 99 % / 최대 | 다음 프레임에 쓰인 패킷 | 프레임당 다 받은 방송 수 평균")
        for tag, v in seg["control_compute"].items():
            c2, s2, e2, w2, b2 = (v["compute_us"], v["start_us"], v["end_us"], v["packet_to_use_us"],
                                  v["broadcasts_received_per_frame"])
            L.append(f"{tag} | {v['frames_computed']} | {c2['mean']} / {c2['p99']} / {c2['max']} | {s2['mean']} / {s2['max']} | "
                     f"{e2['max']} | " + (f"{w2['mean']} / {w2['p99']} / {w2['max']}" if w2 else "-") +
                     f" | {v['packets_used_next_frame']} | {b2['mean'] if b2 else '-'}")
    else:
        L.append("제어 계산 기록 없음 (F 줄에 계산 시작·끝 칸이 없거나 모두 0)")

    L.append("")
    L.append("== 14. 이웃 정보가 오래된 정도 (다른 드론의 방송을 마지막으로 다 받은 뒤 지난 프레임 수) ==")
    L.append("받는 태그 | 표본 | 평균 | 중앙 | 95 % | 99 % | 최대 | 99 % 를 초로 | 최대를 초로")
    for tag, v in seg["neighbor_age_frames"].items():
        L.append(f"{tag} | {v['n']} | {v['mean']} | {v['median']} | {v['p95']} | {v['p99']} | {v['max']} | "
                 f"{v['seconds_p99']} | {v['seconds_max']}")

    L.append("")
    L.append("== 15. 전달 확률 추정식의 적중률 (시뮬레이터 mac.py: p <- 0.9 p + 0.1 x 받음, 처음 0.7. p >= 0.5 면 '받는다'로 예측) ==")
    L.append("대상 | 예측 수 | 추정식 | 가장 나쁜 링크 | 제곱 오차 평균 | 늘 받는다고 예측 | 직전 결과로 예측 | 최근 10개로 예측")
    for label, name in (("heartbeat", "heartbeat (시뮬레이터가 쓰는 대상)"), ("all_broadcasts", "방송 전부")):
        v = seg["delivery_estimator"][label]
        L.append(f"{name} | {v['predictions']} | {pct(v['ewma_hit_rate'])} | "
                 f"{pct(v['ewma_hit_rate_worst_link'])} | {v['ewma_brier']} | {pct(v['always_success_hit_rate'])} | "
                 f"{pct(v['last_outcome_hit_rate'])} | {pct(v['recent10_hit_rate'])}")

    L.append("")
    L.append("== 16. 손실 모형의 값 ==")
    L.append("링크 | 보낸 수 | 평균 손실률 | 연달아 잃는 평균 길이 | 최대")
    tot = seg["broadcast_totals"]
    for name, st in (("드론 → 드론 조각", tot["all"]["fragment"]), ("드론 → 드론 방송(조각 둘 다)", tot["all"]["message"]),
                     ("heartbeat 방송", tot["H"]["message"]), ("추가 슬롯 방송", tot["B"]["message"])):
        L.append(f"{name} | {st['sent']} | {pct(st['loss_rate'])} | {st['mean_burst']} | {st['max_burst']}")


def report(summary):
    L = []
    tags = summary["tag"]
    L.append(f"태그 {summary['tags_configured']}대 설정, 기록이 있는 태그 {summary['tags_logged']}, 프레임 {summary['frame_us'] / 1000:.0f} ms")
    for name, value in summary.get("notes", {}).items():
        L.append(f"! {name}: {value}")
    L.append("")
    L.append("== 1. 프레임 수신 (앵커 1 의 Poll = 프레임의 기준) ==")
    L.append("태그 | 프레임 범위 | 프레임 수 | 기준을 받은 비율 | 아무것도 못 들은 프레임 | 연달아 놓친 최대 | 기록 버림 | 수신 오류 | "
             "다른 방식 패킷 | 켠 횟수 | 앵커 1 재시작 | 깨진 줄")
    for t, e in tags.items():
        L.append(f"{t} | {e['frames_first']}~{e['frames_last']} | {e['frames']} | {pct(e['master_poll_rate'])} | "
                 f"{e['frames_without_any_packet']} | {e['max_consecutive_missed_frames']} | {e['log_dropped']} | "
                 f"{e['rx_errors']} | {e['other_mode_packets']} | {e.get('restarts', 0)} | "
                 f"{e.get('anchor1_restarts', 0)} | {e.get('bad_lines', 0)}")
    if any(e.get("sd") for e in tags.values()):
        L.append("태그 | SD 파일 | SD 상태(1 = 정상) | 쓴 섹터 | 버린 글자 | 쓰기 오류 | 가장 긴 쓰기 ms | 쓰기를 거는 데 걸린 최대 µs")
        for t, e in tags.items():
            d = e.get("sd")
            if d:
                L.append(f"{t} | {d['file']} | {d['state']} | {d['blocks_written']} | {d['dropped_bytes']} | "
                         f"{d['write_errors']} | {d['max_write_ms']} | {d['max_start_us']}")
    L.append("")
    L.append("== 2. 앵커 → 태그: Poll 수신율 (모든 프레임) ==")
    L.append("태그 | 앵커 1 | 앵커 2 | 앵커 3")
    for t, e in tags.items():
        L.append(f"{t} | " + " | ".join(pct(e["poll_rate_by_anchor"][a]) for a in (1, 2, 3)))
    a1 = summary.get("anchor1_poll_loss")
    if a1 and a1["sent"]:
        L.append(f"앵커 1 의 Poll: 손실 {pct(a1['loss_rate'])}, 연달아 잃은 길이 평균 {a1['mean_burst']} 최대 {a1['max_burst']} "
                 f"분포 {a1['burst_lengths']}")
    L.append("")
    L.append("== 3. 위치 측정 (내 차례인 프레임) ==")
    L.append("태그 | 앵커 | 차례 수 | 성공률 | 연달아 실패 최대 | 거리 평균 m | 흩어짐 cm | 실패 내역")
    for t, e in tags.items():
        for a, r in e["ranging"].items():
            fails = {k: v for k, v in r["outcomes"].items() if k != "ok"}
            L.append(f"{t} | A{a} | {r['turns']} | {pct(r['success_rate'])} | {r['max_consecutive_failures']} | "
                     + (f"{r['distance_mean_m']:.3f} | {r['distance_std_cm']:.1f}" if r["distances"] else "- | -")
                     + f" | {fails or ''}")
    L.append("태그 | 세 거리가 모두 나온 비율 | 연달아 실패 최대 | 그때 위치가 비는 시간 s")
    for t, e in tags.items():
        p = e["position"]
        L.append(f"{t} | {pct(p['success_rate'])} | {p['max_consecutive_failures']} | {p['max_gap_s']}")
    pe = summary.get("position_end_measured_us")
    if pe:
        L.append(f"마지막 Final 이 온 때 (앵커 1 의 Poll 기준 µs): 평균 {pe['mean']}, 99 % {pe['p99']}, 최대 {pe['max']} ({pe['n']}번)")
    L.append("")
    L.append("== 4. 태그 → 태그: 차례인 태그의 Response 를 들은 비율 (보낸 태그 → 들은 태그) ==")
    listeners = sorted(tags)
    L.append("보낸 태그 | " + " | ".join(f"→{i}" for i in listeners))
    for j in listeners:
        cells = []
        for i in listeners:
            cell = summary["tag_to_tag_response"].get(f"{j}->{i}")
            cells.append("·" if cell is None else pct(cell["rate"], 1))
        L.append(f"{j} | " + " | ".join(cells))
    if summary["anchor_links"]:
        L.append("")
        L.append("== 5. 앵커 기록으로 본 링크 (보낸 수가 있는 것만) ==")
        present = sorted({int(name.split("->A")[1]) for name in summary["anchor_links"] if "->A" in name})
        L.append("태그 → 앵커 (Response) | " + " | ".join(f"앵커 {a}" for a in present))
        for t in sorted(tags):
            L.append(f"{t} | " + " | ".join(pct(summary["anchor_links"].get(f"tag{t}->A{a}", {}).get("rate")) for a in present))
        L.append("앵커 → 태그 (Final) | " + " | ".join(f"앵커 {a}" for a in present))
        for t in sorted(tags):
            L.append(f"{t} | " + " | ".join(pct(summary["anchor_links"].get(f"A{a}->tag{t} final", {}).get("rate"))
                                           for a in present))
    L.append("")
    L.append("== 6. 시각 (태그 DW3000 시계) ==")
    L.append(f"앵커 1 이 실제로 쓰는 프레임 길이: {nominal_frame_us(summary['frame_us']):.3f} µs "
             "(펌웨어가 1 µs 를 63898 DTU 로 환산해서 설정값보다 6 ppm 길다. 시계 차이는 이 길이와 견준 값)")
    L.append("태그 | 프레임 간격 평균 µs | 흩어짐 µs | 앵커 1 과의 시계 차이 ppm | 10초씩 본 ppm 의 최소~최대 | 앵커 2 Poll 자리 µs | "
             "앵커 3 Poll 자리 µs | Response 예약 여유 최소 µs | 되돌림 최대 µs | Response (R 보냄, L 늦음)")
    for t, e in tags.items():
        tm = e["timing"]
        step, lead, turn = tm["frame_step_us"], tm["response_lead_us"], tm["turnaround_us"]
        p2, p3 = tm["poll_offset_us"][2], tm["poll_offset_us"][3]
        win = tm.get("clock_vs_anchor1_ppm_windows")
        L.append(f"{t} | " + (f"{step['mean']:.3f} | {step['std']:.3f}" if step else "- | -") +
                 f" | {tm['clock_vs_anchor1_ppm']} | " + (f"{win['min']}~{win['max']}" if win else "-") +
                 f" | {p2['mean'] if p2 else '-'} | {p3['mean'] if p3 else '-'} | "
                 f"{lead['min'] if lead else '-'} | {turn['max'] if turn else '-'} | {tm['responses']}")

    segments = summary.get("segments", [])
    if not segments:
        L.append("")
        L.append("(K 줄이 없어 7~16장은 계산하지 않았습니다: 1판 기록이거나 앵커 1 의 Poll 을 한 번도 못 받음)")
    many = len({s["epoch"] for s in segments}) > 1
    for n, seg in enumerate(segments, 1):
        _report_segment(L, n, seg, many)

    L.append("")
    L.append("== 17. 비행 기록, LED, 그 밖의 줄 ==")
    if summary.get("flight"):
        L.append("태그 | C 줄 | 명령을 보낸 프레임 | 대본 상태별 C 줄 수 | 가장 빠른 속도 m/s | 사건 | LED 가 바뀐 횟수 | LED 가 켜진 프레임(앞 20개)")
        for t, v in summary["flight"].items():
            L.append(f"{t} | {v['c_lines']} | {v['commands_sent']} | {v['script_states']} | {v['speed_max_m_s']} | "
                     f"{v['events']} | {v['led_changes']} | {v['led_on_frames']}")
    else:
        L.append("C, E, L 줄 없음")
    if summary.get("command_response"):
        L.append("명령이 반영되는 지연 (C 줄: 명령 속도가 바뀐 프레임부터. 비행 제어기 값이 오는 간격만큼 거칠다)")
        L.append("태그 | 명령이 바뀐 횟수 | 속도가 변하기 시작(10 %) ms: 평균 / 최대 | 63 % 에 이름 ms: 평균 / 최대 | 반응을 못 본 횟수")
        for t, v in summary["command_response"].items():
            a, b = v["start_ms"], v["t63_ms"]
            L.append(f"{t} | {v['steps']} | " + (f"{a['mean']} / {a['max']}" if a else "-") + " | " +
                     (f"{b['mean']} / {b['max']}" if b else "-") + f" | {v['no_response_seen']}")
    else:
        L.append("명령이 반영되는 지연: 기록 없음 (C 줄 없음)")
    L.append("모르는 줄: " + (", ".join(f"{k} {v}줄" for k, v in summary.get("unknown_lines", {}).items()) or "없음"))
    for t, notes in summary.get("line_notes", {}).items():
        extra = {k: v for k, v in notes.items() if k not in ("restarts", "other_lines")}
        if extra:
            L.append(f"태그 {t} 기록의 특이 사항: {extra}")
    return "\n".join(L)


def write_outputs(out, summary, logs):
    out.mkdir(parents=True, exist_ok=True)
    clean = json.loads(json.dumps(summary, ensure_ascii=False, default=str))
    for seg in clean.get("segments", []):
        seg.pop("_link_rows", None)
    (out / "frame_summary.json").write_text(json.dumps(clean, indent=2, ensure_ascii=False), encoding="utf-8")
    (out / "frame_summary.txt").write_text(report(summary) + "\n", encoding="utf-8")
    (out / "sim_params.json").write_text(json.dumps(clean.get("sim_params"), indent=2, ensure_ascii=False), encoding="utf-8")
    with open(out / "ranges.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["tag", "frame", "anchor", "distance_m", "valid", "end", "poll_time_hi32", "frame_time_hi32",
                    "rssi_dbm", "first_path_dbm", "host_time", "epoch"])
        for log in logs:
            for key in sorted(log.ranges):
                fr = log.frames.get(key)
                for r in log.ranges[key]:
                    w.writerow([log.tag, fnum(key), r.anchor, f"{r.mm / 1000:.3f}" if r.valid else "", r.valid, r.end,
                                f"{r.poll_hi:08X}", f"{fr.t0_hi:08X}" if fr else "", m.dbm(r.rssi), m.dbm(r.fp),
                                m.iso(fr.host) if fr else "", epoch_of(key)])
    with open(out / "frames.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["tag", "frame", "turn", "frame_time_hi32", "seen_hex", "poll1", "poll2", "poll3", "response",
                    "final1", "final2", "final3", "my_response", "response_lead_us", "turnaround_us", "rx_err",
                    "schedule", "extra_slots", "flags_hex", "radio_end_us", "quiet_work_us", "quiet_work_max_us",
                    "log_dropped", "host_time", "epoch", "control_start_us", "control_end_us", "broadcasts_received"])
        blank = lambda v: "" if v is None else v      # noqa: E731
        for log in logs:
            for key in sorted(log.frames):
                fr = log.frames[key]
                bits = [int(bool(fr.seen & (1 << i))) for i in range(7)]
                w.writerow([log.tag, fnum(key), fr.turn, f"{fr.t0_hi:08X}", f"{fr.seen:02X}", *bits, fr.response,
                            fr.lead_us, fr.turn_us, fr.rx_err, fr.sched or "", blank(fr.extra),
                            "" if fr.flags is None else f"{fr.flags:02X}", blank(fr.radio_end_us), blank(fr.work_us),
                            blank(fr.work_max_us), blank(fr.dropped), m.iso(fr.host), epoch_of(key),
                            blank(fr.ctl_start_us), blank(fr.ctl_end_us), blank(fr.rx_broadcasts)])
    with open(out / "packets.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["tag", "epoch", "frame", "kind", "direction", "slot", "part", "from", "to", "serial", "bytes", "ok",
                    "scheduled_us", "error_ns", "chip_time_hi32", "rssi_dbm", "first_path_dbm", "clock_offset_raw",
                    "clock_offset_ppm", "host_time"])
        for log in logs:
            for p in log.packets:
                w.writerow([log.tag, epoch_of(p.key), p.frame, p.type, p.dir, p.slot, p.part, p.src, p.dst, p.serial,
                            p.bytes, p.ok, p.sched_us, "" if p.dt_ns is None else p.dt_ns, f"{p.t_hi:08X}",
                            "" if abs(p.rssi) >= 32767 else f"{p.rssi / 256:.2f}",
                            "" if abs(p.fp) >= 32767 else f"{p.fp / 256:.2f}", p.cko if p.dir == "R" else "",
                            f"{p.cko * 1e6 / (1 << 26):.3f}" if p.dir == "R" else "", m.iso(p.host)])
    with open(out / "links.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["segment", "epoch", "frame_first", "frame_last", "link", "from", "to", "sent", "lost", "loss_rate",
                    "mean_burst", "max_burst", "burst_lengths", "note"])
        st = summary.get("anchor1_poll_loss")
        if st and st["sent"]:
            w.writerow(["all", "", "", "", "anchor 1 poll (all tags)", "A1", "tags", st["sent"], st["lost"],
                        f"{st['loss_rate']:.5f}", st["mean_burst"], st["max_burst"], json.dumps(st["burst_lengths"]), ""])
        for t, e in summary["tag"].items():
            for a in (1, 2, 3):
                r = e["poll_rate_by_anchor"][a]
                w.writerow(["all", "", e["frames_first"], e["frames_last"], "anchor poll", f"A{a}", t, e["frames"],
                            round((1 - r) * e["frames"]), f"{1 - r:.5f}", "", "", "", ""])
        for name, v in summary.get("tag_to_tag_response", {}).items():
            if v["sent"]:
                j, i = name.split("->")
                w.writerow(["all", "", "", "", "tag response (overheard)", j, i, v["sent"], v["sent"] - v["heard"],
                            f"{1 - v['rate']:.5f}", "", "", "", ""])
        for name, v in summary.get("anchor_links", {}).items():
            sent = v.get("response_sent", v.get("final_sent"))
            heard = v.get("response_heard", v.get("final_heard"))
            if sent:
                w.writerow(["all", "", "", "", "anchor log", name, "", sent, sent - heard, f"{1 - heard / sent:.5f}",
                            "", "", "", ""])
        for n, seg in enumerate(summary.get("segments", []), 1):
            base = [n, seg["epoch"], seg["frames"][0], seg["frames"][1]]
            for v in seg.get("_link_rows", []):
                note = "sender estimated from schedule" if v["sender_estimated"] else ""
                w.writerow(base + [f"{KIND_NAMES[v['kind']]} 조각", v["from"], v["to"], v["fragments_sent"],
                                   v["fragments_lost"], f"{v['fragment_loss']:.5f}", "", v["fragment_max_burst"], "", note])
                w.writerow(base + [f"{KIND_NAMES[v['kind']]} 방송", v["from"], v["to"], v["messages"], v["messages_lost"],
                                   f"{v['message_loss']:.5f}", v["message_mean_burst"], v["message_max_burst"],
                                   json.dumps(v["message_burst_lengths"]), note])
            for name, v in seg["schedule_packet_links"].items():
                j, i = name.split("->")
                w.writerow(base + ["일정 패킷", j, i, v["sent"], v["lost"], f"{v['loss_rate']:.5f}", v["mean_burst"],
                                   v["max_burst"], "", ""])
            for v in seg["exchange_pairs"].values():
                if v["judged"]:
                    w.writerow(base + ["교환 (셋 다)", v["owner"], v["partner"], v["judged"], v["judged"] - v["complete"],
                                       f"{1 - v['complete_rate']:.5f}", "", v["max_consecutive_failures"], "", ""])
    with open(out / "sync_led.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["tag", "epoch", "frame", "led_on", "host_time", "host_time_s"])
        for log in logs:
            for key, on, host in log.leds:
                w.writerow([log.tag, epoch_of(key), fnum(key), on, m.iso(host), f"{host:.6f}" if host else ""])
    with open(out / "flight.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["tag", "epoch", "frame", "x_mm", "y_mm", "z_mm", "vx_mm_s", "vy_mm_s", "vz_mm_s", "roll_cdeg",
                    "pitch_cdeg", "yaw_cdeg", "position_age_ms", "attitude_age_ms", "flight_mode", "armed",
                    "script_state", "script_row", "command_vx_mm_s", "command_vy_mm_s", "command_sent", "host_time",
                    "host_time_s"])
        for log in logs:
            for key, values, host in log.fc:
                w.writerow([log.tag, epoch_of(key), fnum(key)] + ["" if v is None else v for v in values] +
                           [m.iso(host), f"{host:.6f}" if host else ""])
    with open(out / "events.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["tag", "epoch", "frame", "event", "a", "b", "host_time"])
        for log in logs:
            for key, name, a, b, host in log.events:
                w.writerow([log.tag, epoch_of(key), fnum(key), name, a, b, m.iso(host)])


# ---------------------------------------------------------------------------------------------- 기록
def capture(port, baud, path, stop, on_line, errors):
    try:
        import serial
    except ImportError:
        errors.append("pyserial 이 없습니다: python -m pip install pyserial")
        stop.set()
        return
    try:
        with serial.Serial(port, baud, timeout=0.2) as s, open(path, "w", encoding="utf-8", newline="\n") as f:
            buf = b""
            while not stop.is_set():
                chunk = s.read(65536)
                if not chunk:
                    continue
                t = time.time()
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("ascii", errors="replace").strip()
                    if text:
                        f.write(f"{t:.6f}\t{text}\n")
                        on_line(text)
                f.flush()
    except Exception as error:    # noqa: BLE001 - 포트 오류는 그대로 사용자에게 보여 줌
        errors.append(f"{port}: {error}")
        stop.set()


def record(ports, seconds, out, anchor_ports=(), tag_baud=TAG_BAUD):
    """태그 포트는 port<i>.raw.log, 앵커 포트는 anchor<i>.raw.log 로 받는다. (태그 기록들, 앵커 기록들)"""
    out.mkdir(parents=True, exist_ok=True)
    stop, errors, lock = threading.Event(), [], threading.Lock()
    everything = [(p, tag_baud, out / f"port{i}.raw.log", True) for i, p in enumerate(ports)] + \
                 [(p, ANCHOR_BAUD, out / f"anchor{i}.raw.log", False) for i, p in enumerate(anchor_ports)]
    counts = [Counter() for _ in everything]

    def make(i):
        def on_line(text):
            with lock:
                counts[i][text[:1] if text[:2] in ("F,", "R,", "S,", "P,") else "A" if text[:1] == "A" else "other"] += 1
            if text.startswith(("TF_READY", "TF_INFO", "TF_INIT_FAILED", "TF_SD")) or "_BOOT," in text:
                print(f"  {everything[i][0]}: {text}")
        return on_line

    threads = [threading.Thread(target=capture, args=(port, baud, path, stop, make(i), errors), daemon=True)
               for i, (port, baud, path, _) in enumerate(everything)]
    for th in threads:
        th.start()
    print(f"기록 시작: 태그 {', '.join(ports)} ({tag_baud} bit/s)" +
          (f", 앵커 {', '.join(anchor_ports)} ({ANCHOR_BAUD} bit/s)" if anchor_ports else "") +
          f" → {out}  ({seconds:.0f} s, Ctrl+C 로 일찍 끝냄)")
    start = time.time()
    try:
        while not stop.is_set() and time.time() - start < seconds:
            time.sleep(max(0.1, min(5.0, seconds - (time.time() - start))))
            with lock:
                status = "  ".join(f"{e[0]}: 프레임 {c['F']} 거리 {c['R']} 패킷 {c['P']}" if e[3] else f"{e[0]}: 앵커 줄 {c['A']}"
                                   for e, c in zip(everything, counts))
            print(f"  {time.time() - start:5.0f} s  {status}")
    except KeyboardInterrupt:
        print("  멈춤")
    stop.set()
    for th in threads:
        th.join(timeout=2)
    if errors:
        sys.exit("포트 오류: " + "; ".join(errors))
    return [e[2] for e in everything if e[3]], [e[2] for e in everything if not e[3]]


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ports", nargs="+", help="기록: 태그 NUCLEO 들의 ST-LINK VCP (예: COM5 COM6 ...). 921600 bit/s")
    p.add_argument("--anchor-ports", nargs="*", default=[], help="기록: 앵커의 J-Link VCOM (있으면). 115200 bit/s")
    p.add_argument("--tag-baud", type=int, default=TAG_BAUD, help="기록: 태그 포트 속도. 1판 펌웨어(tagf-v1)는 115200")
    p.add_argument("--seconds", type=float, default=120.0, help="기록하는 시간 (기본 120)")
    p.add_argument("--tag-logs", nargs="+", help="분석: 저장한 태그 기록들 (와일드카드 가능. SD 에서 꺼낸 것도 됨)")
    p.add_argument("--anchor-logs", nargs="*", default=[], help="분석: 앵커 J-Link VCOM 기록들")
    p.add_argument("--frame-us", type=float, default=None,
                   help="프레임 길이 µs. 안 주면 TF_INFO 의 frame_us, 그것도 없으면 20000")
    p.add_argument("--guard-margin-us", type=float, default=0.0,
                   help="sim_params.json 의 보호 시간에 더할 여유 µs (기본 0: 어긋남의 99 %% 값만)")
    p.add_argument("--out", type=Path, help="결과 폴더 (기록할 때는 꼭 필요, 비어 있어야 함)")
    opt = p.parse_args()
    if bool(opt.ports) == bool(opt.tag_logs):
        p.error("--ports(기록) 또는 --tag-logs(분석) 중 하나만 주세요")
    anchor_paths = [Path(x) for pattern in opt.anchor_logs for x in (sorted(glob.glob(pattern)) or [pattern])]
    if opt.ports:
        if not opt.out:
            p.error("기록할 때는 --out 결과 폴더가 필요합니다")
        if opt.out.exists() and any(opt.out.iterdir()):
            p.error(f"{opt.out} 에 이미 파일이 있습니다. 새 폴더를 주세요")
        paths, recorded_anchors = record(opt.ports, opt.seconds, opt.out, opt.anchor_ports, opt.tag_baud)
        anchor_paths += recorded_anchors
    else:
        paths = [Path(x) for pattern in opt.tag_logs for x in (sorted(glob.glob(pattern)) or [pattern])]
    logs = [parse_tag_log(path) for path in paths]
    for log in logs:
        if log.tag is None:
            print(f"  ! {log.path}: TF_INFO 줄이 없어 어느 태그인지 모릅니다(기록을 태그 시작 전에 켜세요). 건너뜁니다.")
    anchor_logs = [parse_anchor_log(x) for x in anchor_paths]
    frame_us = opt.frame_us
    if frame_us is None:
        declared = {float(log.info["frame_us"]) for log in logs if log.info.get("frame_us", "").isdigit()}
        frame_us = declared.pop() if len(declared) == 1 else 20000.0
    summary = analyze(logs, anchor_logs, frame_us, opt.guard_margin_us)
    print(report(summary))
    if opt.out:
        write_outputs(opt.out, summary, [log for log in logs if log.tag is not None])
        print(f"\n결과 파일: {opt.out}")


if __name__ == "__main__":
    main()
