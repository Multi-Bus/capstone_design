"""태그(NUCLEO) VCP 로그로 3대1 측정을 계산합니다 (Windows / Linux 공용).
태그 펌웨어 tag3to1-v2(9패킷)와 tag7p-v1(7패킷) 로그를 모두 읽습니다. 어느 쪽인지는 로그에서 알아냅니다.

  1) 앵커별 성공률과 연속 실패
     앵커가 보낸 Poll 수(앵커가 Poll 마다 1씩 올리는 회차 번호)와 태그의 거리유효=1 수를 맞춰 봅니다.
  2) 묶음에 걸린 시간
     A1 Poll 시작부터 A3 교환 끝(Final 수신 또는 시간 초과)까지를 태그 DW3000 시계 하나로 잽니다.
     USB 로그가 도착한 시각은 쓰지 않습니다(시계 한 바퀴 17.2 s 를 넘는 공백을 풀 때만 씀).
     7패킷 로그는 TWR3B 줄(묶음마다 한 줄)로 묶음을 나누고, 패킷 7개의 시각을 시각표와 견줍니다.
     S, D1, D2 를 바꿔 가며 잰 로그(태그 --sweep 빌드)는 값마다 따로 정리합니다.
  3) 거리: 앵커별 평균과 표준편차 (거리유효=1 만)

기록하면서 분석 (pyserial 필요: pip install pyserial):
    python tools/twr3_measure.py --tag-port COM5 --seconds 120 --out results/P1 --point "P1 삼각형 안"
    python tools/twr3_measure.py --tag-port /dev/ttyACM0 --anchor-port /dev/ttyACM1 --seconds 120 --out results/P1

저장한 로그만 다시 분석:
    python tools/twr3_measure.py --tag-log results/P1/tag.raw.log [--anchor-log results/P1/anchor.raw.log] --out results/P1

두 결과 폴더 견주기 (같은 지점에서 잰 9패킷과 7패킷):
    python tools/twr3_measure.py --compare results/P1_9p results/P1_7p

여러 조건을 한 표로 (첫 폴더가 기준. 기수 방향, 기울기, 모터, 가림 같은 사전 시험용):
    python tools/twr3_measure.py --table results/base results/yaw90 results/yaw180 results/yaw270

--anchor-port/--anchor-log 는 앵커 한 대의 J-Link VCOM(J9) 로그입니다. 있으면 그 앵커의 Response 손실과 Final 손실을 나눕니다.
결과: summary.txt, summary.json, exchanges.csv(앵커 회차마다 한 줄), packets.csv(팀 형식), bundles.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import threading
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------- 시계와 프레임 길이
DTU_S = 1.0 / (499.2e6 * 128)      # DW3000 시각 단위 15.65 ps
HI_S = 256 * DTU_S                 # TWR3T 의 Poll 시각 1 = 4.006 ns
WRAP_S = (1 << 32) * HI_S          # Poll 시각이 한 바퀴 도는 시간 17.2 s
# 프레임 길이 추정(데이터시트 식): 채널 5, PRF 64 MHz(프리앰블 심볼 1017.63 ns), 프리앰블 128 + SFD 8,
# PHR 21비트 850 kb/s(1025.64 ns), 데이터 6.8 Mb/s(128.21 ns) + 리드-솔로몬 330비트마다 48비트.
# RMARKER(=타임스탬프)는 PHR 시작점이다. 이 값들은 측정이 아니라 계산이다.
SHR_S = (128 + 8) * 1017.63e-9


def _payload_s(psdu_bytes):
    bits = 8 * psdu_bytes
    return 21 * 1025.64e-9 + (bits + 48 * math.ceil(bits / 330)) * 128.21e-9


POLL_TAIL_S = _payload_s(18 + 2)   # RMARKER 뒤 Poll/Response 남은 길이
FINAL_TAIL_S = _payload_s(30 + 2)  # RMARKER 뒤 Final 남은 길이
RESPONSE_DELAY_US = 900

OUTCOMES = ("ok", "poll_lost", "tag_busy", "resp_not_sent", "resp_or_final_lost", "bad_distance")
OUTCOME_KO = {
    "ok": "성공(거리유효=1)",
    "poll_lost": "Poll 못 받음(회차 빠짐)",
    "tag_busy": "태그 출력 중이라 응답 못 함(측정 장치 탓)",
    "resp_not_sent": "태그가 Response 못 보냄",
    "resp_or_final_lost": "Response 또는 Final 손실",
    "bad_distance": "Final 받았으나 거리 계산 불가",
}
SIM_ANCHOR_LOSS = (0.0, 0.01, 0.05, 0.30)


@dataclass
class Rec:
    """태그가 받은 Poll 한 개 (TWR3 + TWR3T 줄)."""
    host: float | None
    seg: int
    session: int
    round: int
    anchor: int
    resp: int
    final: int
    valid: int
    mm: int
    prssi: int
    pfp: int
    frssi: int
    ffp: int
    pstat: int
    fstat: int
    n: int | None = None
    poll_hi: int | None = None
    prep_us: int | None = None
    end_ns: int | None = None
    end: str | None = None
    rx_err: int | None = None
    rx_other: int | None = None
    stale: int | None = None
    bundle: int | None = None  # 7패킷: 묶음 번호 (TWR3T 의 12번째 칸)
    t: float | None = None     # 펼친 태그 시각 (s, 같은 seg 안에서만 비교)


@dataclass
class AnchorLine:
    anchor: int
    session: int | None
    round: int
    stages: int
    rssi: int
    fp: int
    synced: int
    poll_lead: int | None = None    # 7패킷 앵커: Poll 예약 여유 µs
    final_lead: int | None = None   # 7패킷 앵커: Final 예약 여유 µs (-1 = 안 보냄)


@dataclass
class BundleLine:
    """7패킷 태그의 TWR3B 줄 (묶음 하나)."""
    seg: int
    no: int
    ref: int
    resp_hi: int
    s: int
    d1: int
    d2: int
    polls: str
    resp: str
    finals: str
    valid: str
    lead_us: int
    turn_us: int
    end_ns: int
    rx_err: int
    row: int


@dataclass
class Bundle7:
    line: BundleLine
    recs: dict                      # 앵커 -> Rec
    origin: float                   # 시각표의 0 (앵커 1 의 Poll 자리), 태그 시각 s
    t_resp: float | None            # Response RMARKER, 태그 시각 s


@dataclass
class Bundle:
    seg: int
    k: int
    recs: dict = field(default_factory=dict)   # 앵커 -> Rec


# ---------------------------------------------------------------- 로그 읽기
def parse_host_time(text):
    try:
        return float(text)
    except ValueError:
        try:
            return datetime.fromisoformat(text).timestamp()
        except ValueError:
            return None


def read_log(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            raw = raw.rstrip("\r\n")
            host = None
            if "\t" in raw:
                head, raw = raw.split("\t", 1)
                host = parse_host_time(head.strip())
            text = raw.strip()
            if text:
                yield host, text


def parse_tag_log(path):
    recs, info, notes, blines = [], {}, Counter(), []
    seg, last = 0, None
    for host, text in read_log(path):
        if text.startswith("TWR3_READY"):
            seg += 1
            last = None
            notes["tag_ready"] += 1
        elif text.startswith("TWR3_INFO,"):
            info.update(kv.split("=", 1) for kv in text.split(",")[1:] if "=" in kv)
        elif text.startswith("TWR3_INIT_FAILED"):
            notes["tag_init_failed"] += 1
        elif text.startswith("TWR3,"):
            c = text.split(",")
            last = None
            try:
                if len(c) != 14:
                    raise ValueError
                values = [int(x) for x in c[1:12]] + [int(c[12], 16), int(c[13], 16)]
            except ValueError:
                notes["bad_line"] += 1
                continue
            last = Rec(host, seg, *values)
            recs.append(last)
        elif text.startswith("TWR3T,"):
            c = text.split(",")
            try:
                if len(c) not in (11, 12) or last is None or last.n is not None:
                    raise ValueError
                if (int(c[2]), int(c[3])) != (last.anchor, last.round):
                    raise ValueError
                last.n, last.poll_hi, last.prep_us, last.end_ns = int(c[1]), int(c[4], 16), int(c[5]), int(c[6])
                last.end = c[7]
                last.rx_err, last.rx_other, last.stale = int(c[8]), int(c[9]), int(c[10])
                if len(c) == 12:
                    last.bundle = int(c[11])
            except ValueError:
                notes["orphan_timing_line"] += 1
        elif text.startswith("TWR3B,"):
            c = text.split(",")
            last = None
            try:
                if len(c) != 16 or c[8] not in ("R", "L", "N", "B") or any(len(x) != 3 for x in (c[7], c[9], c[10])):
                    raise ValueError
                blines.append(BundleLine(seg, int(c[1]), int(c[2]), int(c[3], 16), int(c[4]), int(c[5]), int(c[6]),
                                         c[7], c[8], c[9], c[10], int(c[11]), int(c[12]), int(c[13]), int(c[14]),
                                         int(c[15])))
            except ValueError:
                notes["bad_line"] += 1
        elif text.startswith("TWR3_WARN,"):
            notes["tag_warning"] += 1
            info["warning"] = text
        elif text.startswith("TWR3_ROW,"):
            notes["sweep_rows"] += 1
        else:
            notes["other_line"] += 1
    return recs, info, notes, blines


ANCHOR_RE = re.compile(r"^A([123]),(\d+),(\d+),(-?\d+),(-?\d+),([01]),([0-9A-Fa-f]{8})(?:,(-?\d+),(-?\d+))?$")
BOOT_RE = re.compile(r"^A([123])_BOOT,.*session=0x([0-9A-Fa-f]+)")


def parse_anchor_log(path):
    lines, session = [], {}
    for _, text in read_log(path):
        m = BOOT_RE.match(text)
        if m:
            session[int(m[1])] = int(m[2], 16)
            continue
        m = ANCHOR_RE.match(text)
        if m:
            a = int(m[1])
            lines.append(AnchorLine(a, session.get(a), int(m[2]), int(m[3]), int(m[4]), int(m[5]), int(m[6]),
                                    int(m[8]) if m[8] is not None else None,
                                    int(m[9]) if m[9] is not None else None))
    return lines


# ---------------------------------------------------------------- 시각 펼치기, USB 줄 빠짐
def unwrap_times(recs):
    prev = {}
    for r in recs:
        if r.poll_hi is None:
            continue
        p = prev.get(r.seg)
        if p is None:
            r.t = r.poll_hi * HI_S
        else:
            d = ((r.poll_hi - p.poll_hi) % (1 << 32)) * HI_S
            if r.host is not None and p.host is not None:
                d += max(0, round((r.host - p.host - d) / WRAP_S)) * WRAP_S
            r.t = p.t + d
        prev[r.seg] = r


def count_lost_lines(recs):
    lost, prev = 0, {}
    for r in recs:
        if r.n is None:
            continue
        p = prev.get(r.seg)
        if p is None:
            if r.seg > 0:            # TWR3_READY 뒤면 1부터 시작해야 함
                lost += r.n - 1
        elif r.n > p:
            lost += r.n - p - 1
        prev[r.seg] = r.n
    return lost


# ---------------------------------------------------------------- 1) 앵커별 성공률과 연속 실패
def outcome(r):
    if r is None:
        return "poll_lost"
    if r.end == "B":
        return "tag_busy"
    if r.final and r.valid:
        return "ok"
    if not r.resp:
        return "resp_not_sent"
    if r.final:
        return "bad_distance"
    return "resp_or_final_lost"


def runs_of_failure(flags):
    """flags: 성공 True/실패 False 목록들의 목록(세션마다 끊음) -> 실패 연속 길이 목록"""
    runs = []
    for seq in flags:
        run = 0
        for ok in seq:
            if ok:
                if run:
                    runs.append(run)
                run = 0
            else:
                run += 1
        if run:
            runs.append(run)
    return runs


def prob_run_at_least(n, p, k):
    """독립 실패(확률 p) n 번에서 k 번 이상 연속 실패가 한 번이라도 나올 확률"""
    if k <= 0:
        return 1.0
    if p <= 0 or n < k:
        return 0.0
    state = [1.0] + [0.0] * (k - 1)   # state[j] = 지금 j 번 연속 실패 중이고 아직 k 에 못 닿음
    for _ in range(n):
        total = sum(state)
        state = [(1 - p) * total] + [p * state[j - 1] for j in range(1, k)]
    return max(0.0, 1.0 - sum(state))


def expected_max_run(n, p):
    if p <= 0 or n <= 0:
        return 0.0
    total, k = 0.0, 1
    while k <= n:
        q = prob_run_at_least(n, p, k)
        total += q
        if q < 1e-4:
            break
        k += 1
    return total


def run_stats(flags):
    runs = runs_of_failure(flags)
    n = sum(len(s) for s in flags)
    ok = sum(sum(1 for x in s if x) for s in flags)
    p = 1 - ok / n if n else 0.0
    max_run = max(runs, default=0)
    return {
        "trials": n, "ok": ok, "success_rate": ok / n if n else None, "max_consecutive_failures": max_run,
        "failure_run_lengths": dict(sorted(Counter(runs).items())),
        "iid_expected_max_run": round(expected_max_run(n, p), 2) if n else None,
        "iid_prob_run_at_least_observed": round(prob_run_at_least(n, p, max_run), 4) if n and max_run else None,
    }


def analyze_anchors(recs, anchor_lines):
    table, order, dups = defaultdict(dict), [], 0
    for r in recs:
        if r.anchor not in (1, 2, 3) or r.round < 1:
            continue
        key = (r.anchor, r.session)
        if key not in table:
            order.append(key)
        if r.round in table[key]:
            dups += 1
            continue
        table[key][r.round] = r

    alog = defaultdict(dict)    # (앵커, 세션) -> {회차: AnchorLine}
    for line in anchor_lines:
        session = line.session
        if session is None:     # BOOT 줄 없이 시작한 앵커 로그: 회차가 가장 많이 겹치는 태그 세션에 붙임
            cands = [k for k in order if k[0] == line.anchor]
            session = max(cands, key=lambda k: len(table[k]) if line.round in table[k] else -1, default=(0, None))[1]
        alog[(line.anchor, session)][line.round] = line

    rows, result = [], {}
    for a in (1, 2, 3):
        keys = [k for k in order if k[0] == a]
        counts, flags, sent, heard, resp_sent = Counter(), [], 0, 0, 0
        pk = Counter()   # 앵커 로그가 있을 때 패킷별
        for key in keys:
            rounds = table[key]
            lo, hi = min(rounds), max(rounds)
            if hi - lo > 10_000_000:
                continue
            sent += hi - lo + 1
            heard += len(rounds)
            seq = []
            for rn in range(lo, hi + 1):
                r = rounds.get(rn)
                o = outcome(r)
                counts[o] += 1
                seq.append(o == "ok")
                if r is not None and r.resp and r.end != "B":
                    resp_sent += 1
                al = alog.get(key, {}).get(rn)
                if al is not None:
                    pk["anchor_lines"] += 1
                    if al.stages & 1:
                        pk["poll_sent"] += 1
                        pk["poll_lost"] += r is None
                    if r is not None and r.resp and r.end != "B":
                        pk["resp_sent"] += 1
                        pk["resp_lost"] += not (al.stages & 2)
                    if al.stages & 4:
                        pk["final_sent"] += 1
                        pk["final_lost"] += not (r is not None and r.final)
                    pk["final_late"] += bool(al.stages & 16)      # 7패킷: Final 예약이 늦어 못 보냄
                    pk["poll_late"] += bool(al.stages & 32)       # 7패킷: Poll 예약이 늦어 못 보냄
                rows.append((a, key[1], rn, r, o, al))
            flags.append(seq)
        if not sent:
            continue
        rs = run_stats(flags)
        entry = {
            "polls_sent_by_anchor": sent, "polls_heard_by_tag": heard,
            "outcomes": {o: counts[o] for o in OUTCOMES},
            "poll_loss_rate": counts["poll_lost"] / sent,
            "resp_or_final_loss_rate": counts["resp_or_final_lost"] / resp_sent if resp_sent else None,
            "tag_busy": counts["tag_busy"], "sessions": len(keys), **rs,
        }
        if pk["anchor_lines"]:
            entry["anchor_log"] = {
                **dict(pk),
                "poll_loss_rate": pk["poll_lost"] / pk["poll_sent"] if pk["poll_sent"] else None,
                "response_loss_rate": pk["resp_lost"] / pk["resp_sent"] if pk["resp_sent"] else None,
                "final_loss_rate": pk["final_lost"] / pk["final_sent"] if pk["final_sent"] else None,
            }
        result[a] = entry
    return result, rows, dups


# ---------------------------------------------------------------- 2) 묶음에 걸린 시간
def exchange_end(r):
    """Poll RMARKER 기준 교환이 끝난 태그 시각 (s). 모르면 None."""
    if r.t is None or r.end is None:
        return None
    if r.end == "F":
        return r.t + r.end_ns * 1e-9 + FINAL_TAIL_S
    if r.end in "TEXN":
        return r.t + r.end_ns * 1e-9
    if r.end == "L":
        return r.t + POLL_TAIL_S
    return None


def describe(values, scale=1.0, digits=1):
    if not values:
        return None
    v = sorted(x * scale for x in values)
    return {
        "n": len(v), "mean": round(statistics.fmean(v), digits), "median": round(statistics.median(v), digits),
        "std": round(statistics.pstdev(v), digits), "min": round(v[0], digits),
        "p95": round(v[min(len(v) - 1, int(0.95 * len(v)))], digits), "max": round(v[-1], digits),
    }


def analyze_bundles(recs, max_gap_s):
    """묶음을 나누고, 위치(세 거리 모두 유효) 성공/실패 목록을 만든다.
    아무 기록도 없는 공백이 max_gap_s 를 넘으면(앵커·태그를 껐다 켬 등) 위치 목록을 끊고 공백은 세지 않는다."""
    segs = defaultdict(list)
    for r in recs:
        if r.t is not None and r.anchor in (1, 2, 3):
            segs[r.seg].append(r)
    bundles, layout, flags, breaks = [], {}, [], 0
    for seg, rs in sorted(segs.items()):
        rs.sort(key=lambda r: r.t)
        spacing = [(b.t - a.t) / (b.anchor - a.anchor) for a, b in zip(rs, rs[1:])
                   if b.anchor > a.anchor and 0 < b.t - a.t < 1.0]
        last, cyc = {}, []
        for r in rs:
            p = last.get(r.anchor)
            if p is not None and p.session == r.session and r.round - p.round == 1 and r.t - p.t < 5.0:
                cyc.append(r.t - p.t)
            last[r.anchor] = r
        if not spacing or not cyc:
            continue
        slot, cycle = statistics.median(spacing), statistics.median(cyc)
        groups = []
        for r in rs:
            g = groups[-1] if groups else None
            if g and r.anchor > g[-1].anchor and \
                    abs(r.t - (g[0].t - (g[0].anchor - 1) * slot) - (r.anchor - 1) * slot) < slot / 2:
                g.append(r)
            else:
                groups.append([r])
        k, prev, seq = 0, None, []
        for g in groups:
            start = g[0].t - (g[0].anchor - 1) * slot
            if prev is not None:
                step = max(1, round((start - prev) / cycle))
                if start - prev > max_gap_s:
                    flags.append(seq)
                    seq = []
                    breaks += 1
                else:
                    seq.extend([False] * (step - 1))     # 세 앵커 모두 못 받은 바퀴
                k += step
            prev = start
            b = Bundle(seg, k, {r.anchor: r for r in g})
            bundles.append(b)
            seq.append(all(a in b.recs and outcome(b.recs[a]) == "ok" for a in (1, 2, 3)))
        flags.append(seq)
        layout[seg] = {"slot_ms": round(slot * 1e3, 4), "cycle_ms": round(cycle * 1e3, 4)}
    return bundles, layout, flags, breaks


def bundle_row(b):
    r1, r2, r3 = (b.recs.get(a) for a in (1, 2, 3))
    duration = None
    if r1 is not None and r3 is not None:
        end = exchange_end(r3)
        if end is not None:
            duration = end - (r1.t - SHR_S)
    return {
        "seg": b.seg, "k": b.k, "anchors": "".join(str(a) for a in sorted(b.recs)),
        "ok": "".join(str(a) for a in sorted(b.recs) if outcome(b.recs[a]) == "ok"),
        "complete": int(all(a in b.recs and outcome(b.recs[a]) == "ok" for a in (1, 2, 3))),
        "duration_us": round(duration * 1e6, 2) if duration is not None else "",
        "gap12_us": round((r2.t - r1.t) * 1e6, 2) if r1 and r2 else "",
        "gap23_us": round((r3.t - r2.t) * 1e6, 2) if r2 and r3 else "",
        **{f"a{a}_end": (b.recs[a].end or "") if a in b.recs else "" for a in (1, 2, 3)},
        **{f"a{a}_exchange_us": round(b.recs[a].end_ns / 1000, 3)
           if a in b.recs and b.recs[a].end == "F" else "" for a in (1, 2, 3)},
    }


def timing_summary(recs, bundles, layout, budget_ms):
    rows = [bundle_row(b) for b in bundles]
    with_ends = [r["duration_us"] for r in rows if r["duration_us"] != ""]
    complete = [r["duration_us"] for r in rows if r["duration_us"] != "" and r["complete"]]
    per_anchor, air_mean = {}, {}
    for a in (1, 2, 3):
        core = [r.end_ns / 1000 for r in recs if r.anchor == a and r.end == "F"]
        if core:
            per_anchor[a] = describe(core, digits=2)
            air_mean[a] = statistics.fmean(core) + (SHR_S + FINAL_TAIL_S) * 1e6
    prep = [r.prep_us for r in recs if r.prep_us is not None and r.end != "B"]
    packed = sum(air_mean.values()) if len(air_mean) == 3 else None
    out = {
        "layout": layout, "bundles": len(rows),
        "bundle_duration_us_all": describe(with_ends, digits=1),
        "bundle_duration_us_complete": describe(complete, digits=1),
        "exchange_poll_to_final_rmarker_us": per_anchor,
        "exchange_air_estimate_us": {a: round(v, 1) for a, v in air_mean.items()},
        "frame_constants_us": {"poll_shr": round(SHR_S * 1e6, 1), "poll_tail": round(POLL_TAIL_S * 1e6, 1),
                               "final_tail": round(FINAL_TAIL_S * 1e6, 1)},
        "gap12_us": describe([r["gap12_us"] for r in rows if r["gap12_us"] != ""], digits=1),
        "gap23_us": describe([r["gap23_us"] for r in rows if r["gap23_us"] != ""], digits=1),
        "tag_prep_us": describe(prep, digits=0),
        "packed_lower_bound_us": round(packed, 1) if packed else None,
        "budget_ms": budget_ms,
    }
    return out, rows


# ---------------------------------------------------------------- 2') 7패킷: 묶음과 시각표
SEVEN_SETTLE = 10          # S, D1, D2 가 바뀐 뒤 이만큼의 묶음은 통계에서 뺀다 (앵커가 새 값을 따라오는 동안)
GOAL_MS = 3.0              # 인계 문서의 목표: 묶음의 무선 점유 3 ms 이하
SIM_BUNDLE_MS = 2.75       # 시뮬레이터와 원고가 전제한 값


def schedule_us(s_us, d1_us, d2_us):
    """시각표대로일 때 묶음의 무선 점유: 첫 Poll 의 프리앰블 시작 ~ 마지막 Final 끝"""
    return 4 * s_us + d1_us + d2_us + (SHR_S + FINAL_TAIL_S) * 1e6


def analyze_bundles7(recs, blines, ant_dly, max_gap_s):
    """TWR3B 줄로 묶음을 만든다. 위치(세 거리 모두 유효) 성공/실패 목록도 만든다."""
    by = defaultdict(dict)
    for r in recs:
        if r.bundle is not None and r.t is not None and r.anchor in (1, 2, 3):
            by[(r.seg, r.bundle)][r.anchor] = r
    bundles = []
    for bl in blines:
        rs = by.get((bl.seg, bl.no))
        if not rs or bl.ref not in rs:
            continue
        ref = rs[bl.ref]
        origin = ref.t - (bl.ref - 1) * bl.s * 1e-6      # 시각표의 0: 기준 Poll 에서 거꾸로 센 앵커 1 의 Poll 자리
        t_resp = None
        if bl.resp in "RN":
            # 예약 시각에 송신 안테나 지연을 더한 때가 실제 RMARKER 다
            t_resp = ref.t + ((bl.resp_hi - ref.poll_hi) % (1 << 32)) * HI_S + ant_dly * DTU_S
        bundles.append(Bundle7(bl, rs, origin, t_resp))
    bundles.sort(key=lambda b: (b.line.seg, b.origin))
    steps = [b.origin - a.origin for a, b in zip(bundles, bundles[1:])
             if a.line.seg == b.line.seg and 0.005 < b.origin - a.origin < 1.0]
    cycle = statistics.median(steps) if steps else None
    flags, seq, breaks, prev = [], [], 0, None
    for b in bundles:
        if prev is not None and prev.line.seg == b.line.seg and cycle:
            gap = b.origin - prev.origin
            if gap > max_gap_s:
                flags.append(seq)
                seq = []
                breaks += 1
            else:
                seq.extend([False] * (max(1, round(gap / cycle)) - 1))   # 세 앵커 모두 못 받은 바퀴
        elif prev is not None:
            flags.append(seq)
            seq = []
        seq.append(complete7(b))
        prev = b
    flags.append(seq)
    return bundles, cycle, flags, breaks


def complete7(b):
    return all(a in b.recs and outcome(b.recs[a]) == "ok" for a in (1, 2, 3))


def bundle7_row(b):
    bl = b.line
    polls = {a: (r.t - b.origin) * 1e6 for a, r in b.recs.items()}
    finals = {a: (r.t + r.end_ns * 1e-9 - b.origin) * 1e6 for a, r in b.recs.items() if r.end == "F"}
    air = ""
    if complete7(b):
        air = round(max(finals.values()) + FINAL_TAIL_S * 1e6 - (min(polls.values()) - SHR_S * 1e6), 2)
    ref = b.recs[bl.ref]
    return {
        "host_time": iso(ref.host), "seg": bl.seg, "bundle": bl.no, "row": bl.row,
        "s_us": bl.s, "d1_us": bl.d1, "d2_us": bl.d2, "ref_anchor": bl.ref,
        "polls": bl.polls, "response": bl.resp, "finals": bl.finals, "valid": bl.valid,
        "complete": int(complete7(b)), "air_us": air, "schedule_us": round(schedule_us(bl.s, bl.d1, bl.d2), 2),
        **{f"poll{a}_us": round(polls[a], 3) if a in polls else "" for a in (1, 2, 3)},
        "response_us": round((b.t_resp - b.origin) * 1e6, 3) if b.t_resp is not None else "",
        **{f"final{a}_us": round(finals[a], 3) if a in finals else "" for a in (1, 2, 3)},
        "response_lead_us": bl.lead_us if bl.resp in "RN" else "", "turnaround_us": bl.turn_us,
        "end_us": round(bl.end_ns / 1000, 3), "rx_err": bl.rx_err,
    }


def timing_summary7(bundles, cycle):
    """S, D1, D2 조합마다 따로 정리한다. 조합이 여러 개면(훑기) 값이 바뀐 직후의 묶음은 뺀다."""
    rows = [bundle7_row(b) for b in bundles]
    keys = []
    for b in bundles:
        key = (b.line.row, b.line.s, b.line.d1, b.line.d2)
        if key not in keys:
            keys.append(key)
    settle = SEVEN_SETTLE if len(keys) > 1 else 0
    runs, prev = [], None                       # 같은 조합이 이어지는 구간
    for b, row in zip(bundles, rows):
        key = (b.line.seg, b.line.row, b.line.s, b.line.d1, b.line.d2)
        if key != prev:
            runs.append((key[1:], []))
            prev = key
        runs[-1][1].append((b, row))
    groups = []
    for key in keys:
        used, attempts = [], 0
        for k, items in runs:
            if k != key:
                continue
            items = items[settle:]
            if not items:
                continue
            span = items[-1][0].origin - items[0][0].origin
            attempts += max(len(items), round(span / cycle) + 1 if cycle else 0)
            used.extend(items)
        if not used:
            continue
        row_no, s_us, d1_us, d2_us = key
        sent = [(b, row) for b, row in used if b.line.resp in "RN"]
        g = {
            "row": row_no, "s_us": s_us, "d1_us": d1_us, "d2_us": d2_us,
            "schedule_us": round(schedule_us(s_us, d1_us, d2_us), 1),
            "bundles": len(used), "attempts": attempts,
            "complete": sum(row["complete"] for _, row in used),
            "position_success_rate": sum(row["complete"] for _, row in used) / attempts,
            "anchor_success_rate": {a: sum(1 for b, _ in used if a in b.recs and outcome(b.recs[a]) == "ok") / attempts
                                    for a in (1, 2, 3)},
            "response": dict(Counter(b.line.resp for b, _ in used)),
            "air_us": describe([row["air_us"] for _, row in used if row["air_us"] != ""], digits=1),
            "poll_offset_us": {a: describe([row[f"poll{a}_us"] for _, row in sent if row[f"poll{a}_us"] != ""],
                                           digits=2) for a in (1, 2, 3)},
            "final_offset_us": {a: describe([row[f"final{a}_us"] for _, row in sent if row[f"final{a}_us"] != ""],
                                            digits=2) for a in (1, 2, 3)},
            "response_offset_us": describe([row["response_us"] for _, row in sent if row["response_us"] != ""],
                                           digits=2),
            "response_lead_us": describe([b.line.lead_us for b, _ in sent], digits=0),
            "turnaround_us": describe([b.line.turn_us for b, _ in used if b.line.turn_us], digits=0),
            "rx_err": sum(b.line.rx_err for b, _ in used),
        }
        groups.append(g)
    out = {"packets": 7, "bundles": len(rows), "cycle_ms": round(cycle * 1e3, 4) if cycle else None,
           "settle_bundles_skipped": settle, "groups": groups,
           "frame_constants_us": {"poll_shr": round(SHR_S * 1e6, 1), "poll_tail": round(POLL_TAIL_S * 1e6, 1),
                                  "final_tail": round(FINAL_TAIL_S * 1e6, 1)},
           "goal_ms": GOAL_MS, "sim_bundle_ms": SIM_BUNDLE_MS}
    return out, rows


def best_group(groups, margin=0.01, min_attempts=30):
    """기준 줄(첫 줄)보다 위치 성공률이 margin 넘게 떨어지지 않는 조합 중 시각표가 가장 짧은 것"""
    if len(groups) < 2:
        return None
    base = groups[0]["position_success_rate"]
    good = [g for g in groups if g["attempts"] >= min_attempts and g["position_success_rate"] >= base - margin]
    return min(good, key=lambda g: g["schedule_us"]) if good else None


def distance_summary(recs):
    out = {}
    for a in (1, 2, 3):
        mm = [r.mm for r in recs if r.anchor == a and r.final and r.valid]
        if mm:
            out[a] = {"n": len(mm), "mean_m": round(statistics.fmean(mm) / 1000, 4),
                      "std_cm": round(statistics.pstdev(mm) / 10, 2), "min_m": min(mm) / 1000, "max_m": max(mm) / 1000}
    return out


def report_timing7(tim, L):
    L.append("== 2. 묶음에 걸린 시간 (7패킷, 태그 DW3000 시계 하나) ==")
    if not tim["groups"]:
        L.append("TWR3B 줄이 없어 계산하지 못했습니다.")
        return
    L.append(f"묶음 {tim['bundles']}개, 한 바퀴 {tim['cycle_ms']} ms (묶음 시작 간격에서 추정)")
    c = tim["frame_constants_us"]
    extra = c["poll_shr"] + c["final_tail"]
    many = len(tim["groups"]) > 1
    if many:
        L.append(f"S, D1, D2 조합 {len(tim['groups'])}개 (값이 바뀐 뒤 처음 {tim['settle_bundles_skipped']}묶음은 뺌)")
        L.append("줄 | S | D1 | D2 | 시각표 µs | 시도 | 위치 성공 | A1 | A2 | A3 | Response 늦음 | 무선 점유 평균 µs | "
                 "Response 여유 최소 µs | 되돌림 최대 µs")
        for g in tim["groups"]:
            air, lead, turn = g["air_us"], g["response_lead_us"], g["turnaround_us"]
            ar = g["anchor_success_rate"]
            L.append(" | ".join([
                str(g["row"]), str(g["s_us"]), str(g["d1_us"]), str(g["d2_us"]), f"{g['schedule_us']:.0f}",
                str(g["attempts"]), pct(g["position_success_rate"], 1), pct(ar[1], 1), pct(ar[2], 1), pct(ar[3], 1),
                str(g["response"].get("L", 0)), str(air["mean"]) if air else "-",
                f"{lead['min']:.0f}" if lead else "-", f"{turn['max']:.0f}" if turn else "-"]))
        best = best_group(tim["groups"])
        if best:
            L.append(f"기준(첫 줄)보다 위치 성공률이 1%p 넘게 떨어지지 않은 가장 짧은 조합: S {best['s_us']}, "
                     f"D1 {best['d1_us']}, D2 {best['d2_us']} µs → 시각표 {best['schedule_us'] / 1000:.2f} ms "
                     f"(위치 성공 {pct(best['position_success_rate'], 1)}, 시도 {best['attempts']}회)")
            L.append("  * 줄마다 시도 횟수가 적으면 1%p 차이는 우연일 수 있습니다. 고른 조합은 따로 길게 재서 확인하세요.")
    for g in tim["groups"][:1] if many else tim["groups"]:
        s_us, d1, d2 = g["s_us"], g["d1_us"], g["d2_us"]
        L.append(f"[S {s_us}, D1 {d1}, D2 {d2} µs] 묶음 {g['bundles']}개, 세 거리 모두 유효 {g['complete']}개")
        air = g["air_us"]
        if air:
            L.append(f"  무선 점유(첫 Poll 프리앰블 시작 ~ 마지막 Final 끝): 평균 {air['mean']} µs, 최소 {air['min']}, "
                     f"최대 {air['max']}")
            L.append(f"  시각표 4S + D1 + D2 + {extra:.1f} = {g['schedule_us']} µs → 차이 "
                     f"{air['mean'] - g['schedule_us']:+.1f} µs  ({extra:.1f} = 프리앰블·SFD + Final 뒷부분, 계산값)")
        po = [f"A{a} {e['mean']} (표준편차 {e['std']})" for a, e in g["poll_offset_us"].items() if e]
        if po:
            L.append(f"  Poll 시각 µs: {', '.join(po)}  ← 시각표 0 / {s_us} / {2 * s_us}")
        ro = g["response_offset_us"]
        if ro:
            L.append(f"  Response 시각 µs: {ro['mean']}  ← 시각표 {2 * s_us + d1} (예약 시각 + 송신 안테나 지연 0.26)")
        fo = [f"A{a} {e['mean']}" for a, e in g["final_offset_us"].items() if e]
        if fo:
            base = 2 * s_us + d1 + d2
            L.append(f"  Final 시각 µs: {', '.join(fo)}  ← 시각표 {base} / {base + s_us} / {base + 2 * s_us}")
        lead, turn = g["response_lead_us"], g["turnaround_us"]
        if lead:
            L.append(f"  Response 예약 여유: 평균 {lead['mean']:.0f} µs, 최소 {lead['min']:.0f} µs "
                     f"(예약 명령 바로 뒤에 잰 값. 0 에 가까우면 D1 을 더 줄일 수 없음). "
                     f"늦어서 못 보냄 {g['response'].get('L', 0)}회")
        if turn:
            room = s_us - c["poll_shr"]
            L.append(f"  패킷을 받고 다음 수신을 켜기까지: 평균 {turn['mean']:.0f} µs, 최대 {turn['max']:.0f} µs "
                     f"(S - {c['poll_shr']:.0f} = {room:.0f} µs 보다 작아야 함 → 여유 {room - turn['max']:.0f} µs)")
        ref = air["mean"] if air else g["schedule_us"]
        L.append(f"  판정: 무선 점유 {ref / 1000:.2f} ms → 목표 {tim['goal_ms']} ms 이하 "
                 f"{'예' if ref / 1000 <= tim['goal_ms'] else '아니오'}, 시뮬레이터 전제 {tim['sim_bundle_ms']} ms 이하 "
                 f"{'예' if ref / 1000 <= tim['sim_bundle_ms'] else '아니오'}")


def report_distances(dist, L):
    L.append("")
    L.append("== 3. 거리 (거리유효=1 만, 안테나 지연 보정 전이면 치우쳐 있음) ==")
    for a, e in dist.items():
        L.append(f"A{a}: 평균 {e['mean_m']:.3f} m, 표준편차 {e['std_cm']:.1f} cm, 최소 {e['min_m']:.3f}, "
                 f"최대 {e['max_m']:.3f}, {e['n']}개")
    if not dist:
        L.append("유효한 거리가 없습니다.")


def compare(dir_a, dir_b):
    """두 결과 폴더의 summary.json 을 견준다 (같은 지점에서 잰 9패킷과 7패킷)."""
    L, sums = [], []
    for d in (dir_a, dir_b):
        path = Path(d) / "summary.json"
        if not path.is_file():
            sys.exit(f"{path} 가 없습니다. 먼저 그 폴더를 분석하세요(--tag-log ... --out ...).")
        sums.append(json.loads(path.read_text(encoding="utf-8")))
    names = [f"{Path(d).name} ({s.get('packets', '?')}패킷)" for d, s in zip((dir_a, dir_b), sums)]
    L.append(f"견주기: [1] {names[0]}  [2] {names[1]}")
    L.append("앵커 | 거리 평균 [1] m | [2] m | 차이 cm | 2 cm 안 | 표준편차 [1] cm | [2] cm | 성공률 [1] | [2]")
    worst = None
    for a in ("1", "2", "3"):
        d1, d2 = (s.get("distances", {}).get(a) for s in sums)
        r1, r2 = (s.get("anchors", {}).get(a, {}).get("success_rate") for s in sums)
        if not d1 or not d2:
            L.append(f"A{a} | 한쪽에 유효한 거리가 없음 | | | | | | {pct(r1)} | {pct(r2)}")
            continue
        diff = (d2["mean_m"] - d1["mean_m"]) * 100
        worst = max(worst or 0.0, abs(diff))
        L.append(f"A{a} | {d1['mean_m']:.3f} | {d2['mean_m']:.3f} | {diff:+.1f} | {'예' if abs(diff) <= 2.0 else '아니오'} | "
                 f"{d1['std_cm']:.1f} | {d2['std_cm']:.1f} | {pct(r1)} | {pct(r2)}")
    p1, p2 = (s.get("positions", {}).get("success_rate") for s in sums)
    L.append(f"위치(세 거리 모두 유효) 성공률: [1] {pct(p1)}  [2] {pct(p2)}")
    for i, s in enumerate(sums, 1):
        tim = s.get("timing") or {}
        if tim.get("packets") == 7 and tim.get("groups"):
            g = tim["groups"][0]
            air = g.get("air_us")
            L.append(f"묶음 [{i}]: 7패킷 S {g['s_us']}, D1 {g['d1_us']}, D2 {g['d2_us']} µs, 무선 점유 평균 "
                     f"{air['mean'] / 1000:.2f} ms (시각표 {g['schedule_us'] / 1000:.2f} ms)" if air else
                     f"묶음 [{i}]: 7패킷, 완전한 묶음이 없음")
        elif tim.get("packed_lower_bound_us"):
            L.append(f"묶음 [{i}]: 9패킷, 세 교환의 무선 점유 합 {tim['packed_lower_bound_us'] / 1000:.2f} ms")
    if worst is not None:
        L.append(f"거리 평균의 차이는 가장 큰 앵커에서 {worst:.1f} cm → 기준 2 cm {'안' if worst <= 2.0 else '밖'}")
    return "\n".join(L)


def condition_table(dirs):
    """이미 분석한 결과 폴더 여러 개를 조건별 표로. 첫 폴더가 기준이고 거리 차이는 기준과의 차이다."""
    sums = []
    for d in dirs:
        path = Path(d) / "summary.json"
        if not path.is_file():
            sys.exit(f"{path} 가 없습니다. 먼저 그 폴더를 분석하세요(--tag-log ... --out ...).")
        sums.append((Path(d).name, json.loads(path.read_text(encoding="utf-8"))))
    base = sums[0][1]
    L = [f"조건별 표 (기준: {sums[0][0]}). 거리 차이 = 그 조건의 평균 - 기준의 평균",
         "조건 | 방식 | 앵커 | 보낸 Poll | 성공률 | Poll 손실 | Resp/Final 손실 | 최대 연속 실패 | 거리 평균 m | 흩어짐 cm | "
         "기준과의 차이 cm"]
    rows = []
    for name, s in sums:
        for a in ("1", "2", "3"):
            e, d = s.get("anchors", {}).get(a), s.get("distances", {}).get(a)
            if e is None:
                L.append(f"{name} | {s.get('packets', '?')}패킷 | A{a} | 기록 없음 | | | | | | |")
                continue
            b = base.get("distances", {}).get(a)
            diff = (d["mean_m"] - b["mean_m"]) * 100 if d and b else None
            rows.append({"condition": name, "packets": s.get("packets"), "anchor": int(a),
                         "polls_sent": e["polls_sent_by_anchor"], "success_rate": e["success_rate"],
                         "poll_loss_rate": e["poll_loss_rate"], "resp_or_final_loss_rate": e["resp_or_final_loss_rate"],
                         "max_consecutive_failures": e["max_consecutive_failures"],
                         "distance_mean_m": d["mean_m"] if d else None, "distance_std_cm": d["std_cm"] if d else None,
                         "difference_from_base_cm": round(diff, 2) if diff is not None else None})
            L.append(f"{name} | {s.get('packets', '?')}패킷 | A{a} | {e['polls_sent_by_anchor']} | {pct(e['success_rate'])} | "
                     f"{pct(e['poll_loss_rate'])} | {pct(e['resp_or_final_loss_rate'])} | {e['max_consecutive_failures']} | "
                     + (f"{d['mean_m']:.3f} | {d['std_cm']:.1f} | " if d else "- | - | ")
                     + (f"{diff:+.1f}" if diff is not None else "-"))
    L.append("조건 | 위치(세 거리 모두 유효) 성공률 | 최대 연속 실패")
    for name, s in sums:
        pos = s.get("positions", {})
        L.append(f"{name} | {pct(pos.get('success_rate'))} | {pos.get('max_consecutive_failures', '-')}")
    return "\n".join(L), rows


# ---------------------------------------------------------------- 출력
def pct(x, digits=2):
    return "-" if x is None else f"{100 * x:.{digits}f}%"


def dbm(v):
    return "" if v == 32767 else f"{v / 256:.2f}"     # 학생 capture.py 와 같은 환산


def iso(host):
    return datetime.fromtimestamp(host).astimezone().isoformat(timespec="milliseconds") if host else ""


def write_csvs(out, ex_rows, bundle_rows):
    with open(out / "exchanges.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["host_time", "anchor", "session", "round", "outcome", "tag_n", "tag_time_s", "resp_sent",
                    "final_rx", "distance_valid", "distance_m", "poll_rssi_dbm", "poll_fp_dbm", "final_rssi_dbm",
                    "final_fp_dbm", "prep_us", "end_us", "end_kind", "anchor_stages", "anchor_resp_rssi_dbm"])
        for a, session, rn, r, o, al in ex_rows:
            if r is None:
                w.writerow(["", a, session, rn, o] + [""] * 13 + ([al.stages, dbm(al.rssi)] if al else ["", ""]))
                continue
            w.writerow([iso(r.host), a, session, rn, o, r.n if r.n is not None else "",
                        f"{r.t:.9f}" if r.t is not None else "", r.resp, r.final, r.valid,
                        f"{r.mm / 1000:.3f}" if r.valid else "", dbm(r.prssi), dbm(r.pfp), dbm(r.frssi), dbm(r.ffp),
                        r.prep_us if r.prep_us is not None else "",
                        f"{r.end_ns / 1000:.3f}" if r.end_ns is not None else "", r.end or "",
                        al.stages if al else "", dbm(al.rssi) if al else ""])
    # 팀 문서 형식: 시각, 앵커ID, 패킷 종류, 성공 여부, 신호 세기, 첫 경로 세기, 거리값 (모르면 빈칸)
    with open(out / "packets.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["시각", "앵커ID", "패킷 종류", "성공 여부", "신호 세기", "첫 경로 세기", "거리값"])
        for a, session, rn, r, o, al in ex_rows:
            when = iso(r.host) if r else ""
            w.writerow([when, a, "poll", int(r is not None), dbm(r.prssi) if r else "", dbm(r.pfp) if r else "", ""])
            if r is None or not r.resp or r.end == "B":
                continue
            if al is not None:
                resp_ok = int(bool(al.stages & 2))
            else:
                resp_ok = 1 if r.final else ""           # Final 이 왔으면 Response 는 도착한 것
            w.writerow([when, a, "response", resp_ok, dbm(al.rssi) if al else "", dbm(al.fp) if al else "", ""])
            if resp_ok == 0:
                continue
            final_ok = int(bool(r.final)) if (resp_ok == 1 or r.final) else ""
            w.writerow([when, a, "final", final_ok, dbm(r.frssi) if r.final else "", dbm(r.ffp) if r.final else "",
                        f"{r.mm / 1000:.3f}" if r.valid else ""])
    if bundle_rows:
        with open(out / "bundles.csv", "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(bundle_rows[0]))
            w.writeheader()
            w.writerows(bundle_rows)


def nearest_sim(q):
    return min(SIM_ANCHOR_LOSS, key=lambda s: abs(s - q))


def report(summary, point):
    L = []
    s1, pos, tim = summary["anchors"], summary["positions"], summary["timing"]
    packets = summary.get("packets", 9)
    L.append(f"태그 펌웨어 {summary['tag_fw'] or '(TWR3_INFO 없음: 기록을 태그 시작 뒤에 켰거나 v1)'}, {packets}패킷 방식, "
             f"Poll 기록 {summary['tag_records']}개, "
             f"USB 줄 빠짐 {summary['usb_lines_lost']}개, 태그 재시작 {summary['notes'].get('tag_ready', 0)}회")
    if summary["notes"].get("tag_warning"):
        L.append(f"  ! 태그 경고 {summary['notes']['tag_warning']}회: {summary['tag_info'].get('warning', '')} "
                 "(9패킷 앵커와 7패킷 태그가 섞여 있으면 측정이 되지 않습니다)")
    L.append("")
    L.append("== 1. 앵커별 성공률과 연속 실패 (거리유효=1 기준, 앵커 회차 번호로 보낸 Poll 수를 셈) ==")
    L.append("앵커 | Poll 보냄 | 태그 받음 | 성공 | 성공률 | Poll 손실 | Resp/Final 손실 | 태그 출력 중 | "
             "최대 연속 실패 | 독립이면 기대 최대 (이만큼 나올 확률)")
    for a, e in s1.items():
        prob = e["iid_prob_run_at_least_observed"]
        L.append(f"A{a} | {e['polls_sent_by_anchor']} | {e['polls_heard_by_tag']} | {e['ok']} | "
                 f"{pct(e['success_rate'])} | {pct(e['poll_loss_rate'])} | {pct(e['resp_or_final_loss_rate'])} | "
                 f"{e['tag_busy']} | {e['max_consecutive_failures']} | {e['iid_expected_max_run']}"
                 + (f" ({prob:.3f})" if prob is not None else ""))
    for a, e in s1.items():
        L.append(f"  A{a} 실패 연속 길이 분포(길이: 횟수) {e['failure_run_lengths']}, 결과별 {e['outcomes']}")
        if "anchor_log" in e:
            g = e["anchor_log"]
            L.append(f"  A{a} 앵커 로그로 나눔: Poll 손실 {pct(g['poll_loss_rate'])}, Response 손실 "
                     f"{pct(g['response_loss_rate'])}, Final 손실 {pct(g['final_loss_rate'])} "
                     f"(앵커 로그 {g['anchor_lines']}줄, 앵커가 센 Poll {g.get('poll_sent', 0)})")
    if pos["positions"]:
        L.append(f"위치(세 거리 모두 유효) 성공: {pos['ok']}/{pos['positions']} = {pct(pos['success_rate'])}, "
                 f"최대 연속 실패 {pos['max_consecutive_failures']}, 분포 {pos['failure_run_lengths']}")
        L.append(f"  앵커가 서로 독립이라면 예상 성공률 {pct(pos['iid_product'])} "
                 "(실측이 이보다 많이 낮으면 손실이 몰려서 난 것)")
        if pos["long_gaps_skipped"]:
            L.append(f"  기록이 {pos['max_gap_s']} s 넘게 끊긴 곳 {pos['long_gaps_skipped']}군데는 위치 실패로 세지 않음")
        q = pos["per_message_loss_9"]
        if q is not None:
            L.append(f"  수신 한 번당 손실로 바꾸면 q = 1 - 성공률^(1/9) = {pct(q, 3)}. 시뮬레이터 앵커 손실 가정 1%"
                     f" (훑는 값 0/1/5/30% 중 가까운 값 {nearest_sim(q):.0%})")
            L.append("    위치 하나에 받아야 하는 횟수는 9패킷도 7패킷도 9번입니다(7패킷은 Response 하나를 세 앵커가 따로 받음)."
                     f" Response 손실이 세 앵커에 늘 함께 일어난다면 7번: (1 - q)^7 = {pct((1 - q) ** 7)}")
    poll = [e["poll_loss_rate"] for e in s1.values()]
    rf = [e["resp_or_final_loss_rate"] for e in s1.values() if e["resp_or_final_loss_rate"] is not None]
    worst = max(s1.items(), key=lambda kv: kv[1]["resp_or_final_loss_rate"] or 0)[0] if s1 else None
    if s1:
        rf_text = f"Response/Final 손실 {pct(statistics.fmean(rf), 1) if rf else '-'}(가장 큰 앵커 A{worst})"
        split = [f"A{a} Response {pct(e['anchor_log']['response_loss_rate'], 1)}·Final "
                 f"{pct(e['anchor_log']['final_loss_rate'], 1)}" for a, e in s1.items() if "anchor_log" in e]
        rf_text += f", 앵커 로그로 나눔 {', '.join(split)}" if split else " (둘을 나누려면 앵커 로그)"
        L.append(f"팀 한 줄: 0, 지점 {point or '?'}, {packets}패킷, Poll 손실 {pct(statistics.fmean(poll), 1)}, {rf_text}, "
                 f"위치 측정 성공 {pct(pos['success_rate'], 1)}, 최대 연속 실패 {pos['max_consecutive_failures']}")
    L.append("  * 처음 받은 회차 앞과 마지막 받은 회차 뒤는 셀 수 없습니다(앵커마다 받은 범위만 셈).")
    L.append("")
    if packets == 7:
        report_timing7(tim, L)
        report_distances(summary["distances"], L)
        return "\n".join(L)
    report_timing9(tim, L)
    report_distances(summary["distances"], L)
    return "\n".join(L)


def report_timing9(tim, L):
    L.append("== 2. 묶음에 걸린 시간 (태그 DW3000 시계 하나) ==")
    if not tim or not tim["bundles"]:
        L.append("TWR3T 줄이 없거나 앵커가 두 대 미만이라 계산하지 못했습니다(태그 v2 필요).")
        return
    for seg, lay in tim["layout"].items():
        L.append(f"태그 구간 {seg}: 칸 간격 {lay['slot_ms']} ms, 한 바퀴 {lay['cycle_ms']} ms (Poll 시각에서 추정)")
    d, dc = tim["bundle_duration_us_all"], tim["bundle_duration_us_complete"]
    if d:
        L.append(f"묶음(A1 Poll 시작 ~ A3 Final 끝 또는 시간 초과) {d['n']}개: 평균 {d['mean'] / 1000:.3f} ms, "
                 f"95% {d['p95'] / 1000:.3f} ms, 최대 {d['max'] / 1000:.3f} ms")
    if dc:
        L.append(f"  세 거리 모두 유효한 묶음 {dc['n']}개: 평균 {dc['mean'] / 1000:.3f} ms, 최대 {dc['max'] / 1000:.3f} ms")
    for a, e in tim["exchange_poll_to_final_rmarker_us"].items():
        L.append(f"교환 A{a}: Poll RMARKER→Final RMARKER 평균 {e['mean']} µs (최소 {e['min']}, 최대 {e['max']}), "
                 f"무선 점유 추정 {tim['exchange_air_estimate_us'][a]} µs")
    c = tim["frame_constants_us"]
    L.append(f"  (무선 점유 = 측정값 + 프리앰블·SFD {c['poll_shr']} µs + Final 뒷부분 {c['final_tail']} µs, 계산값)")
    for name, key in (("A1→A2", "gap12_us"), ("A2→A3", "gap23_us")):
        g = tim[key]
        if g:
            L.append(f"Poll 간격 {name}: 평균 {g['mean']} µs, 표준편차 {g['std']}, 최소 {g['min']}, 최대 {g['max']}")
    p = tim["tag_prep_us"]
    if p:
        L.append(f"태그가 Poll 을 알아채기까지(준비) 평균 {p['mean']:.0f} µs, 최대 {p['max']:.0f} µs "
                 f"→ 응답 지연 {RESPONSE_DELAY_US} µs 중 여유 {RESPONSE_DELAY_US - p['max']:.0f} µs")
    budget = tim["budget_ms"]
    if d:
        L.append(f"판정: 지금 묶음 평균 {d['mean'] / 1000:.2f} ms vs 예산 {budget} ms(20 ms 프레임 안 위치 측정)"
                 f" → {'들어감' if d['mean'] / 1000 <= budget else '안 들어감'}")
    if tim["packed_lower_bound_us"]:
        lb = tim["packed_lower_bound_us"] / 1000
        L.append(f"  세 교환을 간격 0으로 붙여도 {lb:.2f} ms (측정한 교환 시간의 합) → "
                 f"{'예산 안' if lb <= budget else '9패킷·지연 900/700 µs 로는 예산 밖'}")
        L.append("  지금 앵커는 칸 간격이 설정값(기본 20 ms)이라 묶음 시간은 대부분 그 간격입니다.")


def analyze(tag_log, anchor_log, out, point, budget_ms, max_gap_s=2.0):
    recs, info, notes, blines = parse_tag_log(tag_log)
    anchor_lines = parse_anchor_log(anchor_log) if anchor_log else []
    unwrap_times(recs)
    anchors, ex_rows, dups = analyze_anchors(recs, anchor_lines)
    seven = bool(blines) or info.get("packets") == "7"
    if seven:
        bundles7, cycle, pos_flags, gap_breaks = analyze_bundles7(recs, blines, int(info.get("ant_dly", 16385)),
                                                                  max_gap_s)
        timing, bundle_rows = timing_summary7(bundles7, cycle)
    else:
        bundles, layout, pos_flags, gap_breaks = analyze_bundles(recs, max_gap_s)
        timing, bundle_rows = timing_summary(recs, bundles, layout, budget_ms) if bundles else (None, [])
    pos = run_stats(pos_flags) if pos_flags else {"trials": 0, "ok": 0, "success_rate": None,
                                                  "max_consecutive_failures": 0, "failure_run_lengths": {}}
    iid = 1.0
    for e in anchors.values():
        iid *= e["success_rate"]
    pos = {"positions": pos["trials"], **pos, "long_gaps_skipped": gap_breaks, "max_gap_s": max_gap_s,
           "iid_product": iid if len(anchors) == 3 else None,
           "per_message_loss_9": 1 - pos["success_rate"] ** (1 / 9) if pos.get("success_rate") is not None else None}
    summary = {
        "tag_log": str(tag_log), "anchor_log": str(anchor_log) if anchor_log else None, "point": point,
        "packets": 7 if seven else 9, "distances": distance_summary(recs),
        "tag_fw": info.get("fw"), "tag_info": info, "notes": dict(notes), "tag_records": len(recs),
        "duplicate_records": dups, "usb_lines_lost": count_lost_lines(recs),
        "anchors": anchors, "positions": pos, "timing": timing,
        "sim_anchor_loss_assumption": {"current": 0.01, "sweep": list(SIM_ANCHOR_LOSS)},
    }
    text = report(summary, point)
    if out:
        out.mkdir(parents=True, exist_ok=True)
        write_csvs(out, ex_rows, bundle_rows)
        (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False, default=str),
                                          encoding="utf-8")
        (out / "summary.txt").write_text(text + "\n", encoding="utf-8")
    return summary, text


# ---------------------------------------------------------------- 기록
def capture(port, path, stop, on_line, errors):
    try:
        import serial
    except ImportError:
        errors.append("pyserial 이 없습니다: python -m pip install pyserial")
        stop.set()
        return
    try:
        with serial.Serial(port, 115200, timeout=0.2) as s, open(path, "w", encoding="utf-8", newline="\n") as f:
            buf = b""
            while not stop.is_set():
                chunk = s.read(4096)
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


def record(opt, out):
    out.mkdir(parents=True, exist_ok=True)
    stop, errors, lock = threading.Event(), [], threading.Lock()
    seen, ok, alines = Counter(), Counter(), [0]

    def on_tag(text):
        if text.startswith("TWR3,"):
            c = text.split(",")
            if len(c) == 14:
                with lock:
                    seen[c[3]] += 1
                    ok[c[3]] += c[5] == "1" and c[6] == "1"
        elif text.startswith(("TWR3_READY", "TWR3_INFO", "TWR3_INIT_FAILED", "TWR3_WARN")):
            print(f"  태그: {text}")

    def on_anchor(text):
        alines[0] += 1
        if "_BOOT" in text or "_READY" in text or "_INIT_FAILED" in text:
            print(f"  앵커: {text}")

    threads = [threading.Thread(target=capture, args=(opt.tag_port, out / "tag.raw.log", stop, on_tag, errors),
                                daemon=True)]
    if opt.anchor_port:
        threads.append(threading.Thread(target=capture, daemon=True, args=(
            opt.anchor_port, out / "anchor.raw.log", stop, on_anchor, errors)))
    for th in threads:
        th.start()
    print(f"기록 시작: 태그 {opt.tag_port}" + (f", 앵커 {opt.anchor_port}" if opt.anchor_port else "") +
          f" → {out}  ({opt.seconds:.0f} s, Ctrl+C 로 일찍 끝냄)")
    start = time.time()
    try:
        while not stop.is_set() and time.time() - start < opt.seconds:
            time.sleep(max(0.1, min(5.0, opt.seconds - (time.time() - start))))
            with lock:
                status = "  ".join(f"A{a} {ok[str(a)]}/{seen[str(a)]}" for a in (1, 2, 3))
            print(f"  {time.time() - start:5.0f} s  태그가 받은 Poll 중 성공: {status}"
                  + (f"  앵커 로그 {alines[0]}줄" if opt.anchor_port else ""))
    except KeyboardInterrupt:
        print("  멈춤")
    stop.set()
    for th in threads:
        th.join(timeout=2)
    if errors:
        sys.exit("포트 오류: " + "; ".join(errors))
    return out / "tag.raw.log", (out / "anchor.raw.log") if opt.anchor_port else None


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tag-port", help="태그 NUCLEO ST-LINK VCP (예: COM5, /dev/ttyACM0)")
    p.add_argument("--anchor-port", help="(선택) 앵커 한 대의 J-Link VCOM")
    p.add_argument("--seconds", type=float, default=120.0, help="기록 시간 (기본 120 s)")
    p.add_argument("--tag-log", type=Path, help="저장한 태그 로그 분석")
    p.add_argument("--anchor-log", type=Path, help="저장한 앵커 로그")
    p.add_argument("--out", type=Path, help="결과 폴더 (기록할 때는 꼭 필요)")
    p.add_argument("--point", default="", help="측정 지점 이름 (요약 한 줄에 들어감)")
    p.add_argument("--budget-ms", type=float, default=2.5, help="20 ms 프레임 안 위치 측정 예산 (기본 2.5 ms)")
    p.add_argument("--max-gap-s", type=float, default=2.0,
                   help="기록이 이보다 오래 끊기면 그 공백은 위치 실패로 세지 않음 (기본 2 s)")
    p.add_argument("--compare", nargs=2, type=Path, metavar=("폴더1", "폴더2"),
                   help="이미 분석한 두 결과 폴더를 견줌 (같은 지점의 9패킷과 7패킷: 거리 평균 차이, 성공률)")
    p.add_argument("--table", nargs="+", type=Path, metavar="폴더",
                   help="이미 분석한 결과 폴더 여러 개를 조건별 표로 (첫 폴더가 기준). --out 을 주면 conditions.csv 도 씀")
    opt = p.parse_args()
    if opt.compare:
        print(compare(*opt.compare))
        return
    if opt.table:
        text, rows = condition_table(opt.table)
        print(text)
        if opt.out and rows:
            opt.out.mkdir(parents=True, exist_ok=True)
            with open(opt.out / "conditions.csv", "w", newline="", encoding="utf-8-sig") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            (opt.out / "conditions.txt").write_text(text + "\n", encoding="utf-8")
            print(f"\n결과 파일: {opt.out}")
        return
    if bool(opt.tag_port) == bool(opt.tag_log):
        p.error("--tag-port(기록) 또는 --tag-log(분석) 중 하나만 주세요")
    if opt.tag_port:
        if not opt.out:
            p.error("기록할 때는 --out 결과 폴더가 필요합니다")
        if opt.out.exists() and any(opt.out.iterdir()):
            p.error(f"{opt.out} 에 이미 파일이 있습니다. 새 폴더를 주세요")
        tag_log, anchor_log = record(opt, opt.out)
    else:
        tag_log, anchor_log = opt.tag_log, opt.anchor_log
    _, text = analyze(tag_log, anchor_log, opt.out, opt.point, opt.budget_ms, opt.max_gap_s)
    print(text)
    if opt.out:
        print(f"\n결과 파일: {opt.out}")


if __name__ == "__main__":
    main()
