"""twr3_measure.py 오프라인 시험 (보드 없이 가짜 태그/앵커 로그로).

    python tools/test_twr3_measure.py

가짜 로그에 일부러 넣는 것: Poll 손실, Final 시간 초과, 태그 출력 중(B), 연속 실패,
DW 시계 한 바퀴(17.2 s) 넘김, 앵커 재시작(세션 바뀜), 40 s 공백, USB 줄 빠짐, v1 로그(TWR3T 없음), 앵커 로그.
7패킷 로그(TWR3B 줄, 묶음 번호): 일부 Poll 손실, Final 손실, 늦은 Response, 값을 바꿔 가며 잰 로그, 9패킷과 견주기.
(펌웨어 C 소스를 PC 에서 돌려 만든 로그로 하는 시험은 sim/run_sim.py 에 있다.)
"""
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import twr3_measure as m  # noqa: E402

FAILS = []


def check(name, got, want, tol=None):
    ok = (abs(got - want) <= tol) if tol is not None else got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got!r}" + ("" if ok else f" (기대 {want!r})"))
    if not ok:
        FAILS.append(name)


EXCH_NS = 1_605_000


def make_logs(path_tag, path_anchor=None, slot=0.020, cycle=0.060, cycles=600, v1=False,
              plan=None, reboot_after=None, gap_s=40.0, drop_line_at=None, anchor_split=None):
    """plan[(앵커, 바퀴)] = 'lost' | 'T' | 'B' (없으면 성공). 정답 사전을 돌려줌."""
    plan = plan or {}
    anchor_split = anchor_split or {}
    t0 = (1 << 32) * m.HI_S - 3.0          # 3 s 뒤 DW 시계 한 바퀴
    tag, anc = ["TWR3_READY,channel=5,tag=256"], []
    if not v1:
        tag.append("TWR3_INFO,fw=tag3to1-v2,ant_dly=16385,flush_idle_ms=3,stale_us=650")
    sessions = {1: 0x11111111, 2: 0x22222222, 3: 0x33333333}
    anc.append(f"A2_BOOT,fw=anchor3to1-v1,session=0x{sessions[2]:08X}")
    rounds = {1: 0, 2: 0, 3: 0}
    n, stale, offset = 0, 0, 0.0
    for c in range(cycles):
        if reboot_after is not None and c == reboot_after:
            offset += gap_s
            sessions = {a: s + 1 for a, s in sessions.items()}
            rounds = {1: 0, 2: 0, 3: 0}
            anc.append(f"A2_BOOT,fw=anchor3to1-v1,session=0x{sessions[2]:08X}")
        for a in (1, 2, 3):
            rounds[a] += 1
            rn = rounds[a]
            t = t0 + offset + c * cycle + (a - 1) * slot
            kind = plan.get((a, c), "ok")
            stages = {"ok": 7, "lost": 1, "B": 1, "T": anchor_split.get((a, c), 1)}[kind]
            if a == 2:
                anc.append(f"A2,{rn},{stages},-21000,-22000,1,00004000")
            if kind == "lost":
                continue
            n += 1
            host = 1.7e9 + t - t0 + 0.004
            hi = int(round(t / m.HI_S)) % (1 << 32)
            if kind == "ok":
                fields = (1, 1, 1, 3000 + a, "F", EXCH_NS + 10 * a, 180)
            elif kind == "T":
                fields = (1, 0, 0, 0, "T", 1_650_000, 180)
            else:
                stale += 1
                fields = (0, 0, 0, 0, "B", 900_000, 900)
            resp, final, valid, mm, end, end_ns, prep = fields
            tag.append(f"{host:.6f}\tTWR3,{sessions[a]},{rn},{a},{resp},{final},{valid},{mm},"
                       f"-20000,-21000,-20500,-21500,00004000,00024000")
            if not v1 and n != drop_line_at:
                tag.append(f"{host:.6f}\tTWR3T,{n},{a},{rn},{hi:08X},{prep},{end_ns},{end},0,0,{stale}")
    Path(path_tag).write_text("\n".join(tag) + "\n", encoding="utf-8")
    if path_anchor:
        Path(path_anchor).write_text("\n".join(anc) + "\n", encoding="utf-8")


def expected_runs(plan, anchor, cycles, segments):
    """세션(=segments 구간)마다 처음~마지막 받은 회차 사이의 실패 연속 최대 길이"""
    best = 0
    for lo, hi in segments:
        heard = [c for c in range(lo, hi) if plan.get((anchor, c), "ok") != "lost"]
        run = 0
        for c in range(heard[0], heard[-1] + 1):
            if plan.get((anchor, c), "ok") == "ok":
                run = 0
            else:
                run += 1
                best = max(best, run)
    return best


def scenario_normal(tmp):
    print("\n--- 1) 칸 20 ms, 손실·연속 실패·시계 한 바퀴·앵커 재시작(40 s 공백)·앵커 로그")
    plan = {(1, 10): "lost", (1, 11): "lost", (1, 12): "lost", (1, 50): "T",
            (2, 100): "T", (2, 101): "T", (2, 200): "B", (2, 201): "lost",
            (3, 300): "lost", (1, 450): "lost", (2, 450): "lost", (3, 450): "lost"}
    split = {(2, 100): 1, (2, 101): 7}       # 2번 앵커: 100은 Response 손실, 101은 Final 손실
    tag, anc, out = tmp / "t1.log", tmp / "a1.log", tmp / "out1"
    make_logs(tag, anc, plan=plan, reboot_after=400, anchor_split=split)
    s, text = m.analyze(tag, anc, out, "시험", 2.5)
    print(text)
    a = s["anchors"]
    check("A1 보낸 Poll(세션 두 개, 처음과 끝은 받은 회차)", a[1]["polls_sent_by_anchor"], 600)
    check("A1 성공", a[1]["ok"], 600 - 5)
    check("A1 Poll 손실", a[1]["outcomes"]["poll_lost"], 4)
    check("A1 최대 연속 실패", a[1]["max_consecutive_failures"], expected_runs(plan, 1, 600, [(0, 400), (400, 600)]))
    check("A1 연속 실패 분포", a[1]["failure_run_lengths"], {1: 2, 3: 1})
    check("A2 태그 출력 중(B)", a[2]["tag_busy"], 1)
    check("A2 최대 연속 실패 (T,T 와 B,lost)", a[2]["max_consecutive_failures"], 2)
    check("A2 세션 수", a[2]["sessions"], 2)
    g = a[2]["anchor_log"]
    check("A2 앵커 로그 Poll 손실", g["poll_lost"], 2)
    check("A2 앵커 로그 Response 손실", g["resp_lost"], 1)
    check("A2 앵커 로그 Final 손실", g["final_lost"], 1)
    check("A2 앵커가 센 Poll = 회차로 센 Poll", g["poll_sent"], a[2]["polls_sent_by_anchor"])
    pos = s["positions"]
    failed_cycles = {c for (_, c) in plan}
    check("위치 수(40 s 공백은 빼고)", pos["positions"], 600)
    check("위치 성공", pos["ok"], 600 - len(failed_cycles))
    check("위치 최대 연속 실패 (10,11,12 / 100,101 / 200,201)", pos["max_consecutive_failures"], 3)
    check("긴 공백 건너뜀", pos["long_gaps_skipped"], 1)
    tim = s["timing"]
    check("칸 간격 추정 ms", tim["layout"][1]["slot_ms"], 20.0, tol=0.001)
    check("한 바퀴 추정 ms", tim["layout"][1]["cycle_ms"], 60.0, tol=0.001)
    want = (2 * 0.020 + (EXCH_NS + 30) * 1e-9 + m.FINAL_TAIL_S + m.SHR_S) * 1e6
    check("완전한 묶음 시간 µs", tim["bundle_duration_us_complete"]["mean"], want, tol=0.05)
    check("A1→A2 Poll 간격 µs", tim["gap12_us"]["mean"], 20000.0, tol=0.05)
    check("교환 A3 µs", tim["exchange_poll_to_final_rmarker_us"][3]["mean"], (EXCH_NS + 30) / 1000, tol=0.01)
    check("USB 줄 빠짐", s["usb_lines_lost"], 0)
    for name in ("exchanges.csv", "packets.csv", "bundles.csv", "summary.json", "summary.txt"):
        check(f"파일 {name}", (out / name).is_file(), True)
    rows = (out / "packets.csv").read_text(encoding="utf-8-sig").splitlines()
    check("packets.csv 머리줄", rows[0], "시각,앵커ID,패킷 종류,성공 여부,신호 세기,첫 경로 세기,거리값")


def scenario_packed(tmp):
    print("\n--- 2) 붙인 묶음(칸 2.5 ms, 바퀴 60 ms): 예산 판정")
    tag = tmp / "t2.log"
    make_logs(tag, slot=0.0025, cycle=0.060, cycles=200)
    s, text = m.analyze(tag, None, None, "", 2.5)
    tim = s["timing"]
    want = (2 * 0.0025 + (EXCH_NS + 30) * 1e-9 + m.FINAL_TAIL_S + m.SHR_S) * 1e6
    check("묶음 시간 µs", tim["bundle_duration_us_all"]["mean"], want, tol=0.05)
    air = sum(EXCH_NS / 1000 + 10 * a / 1000 + (m.SHR_S + m.FINAL_TAIL_S) * 1e6 for a in (1, 2, 3))
    check("빈틈없이 붙인 하한 µs", tim["packed_lower_bound_us"], air, tol=0.2)
    check("판정 문구", "안 들어감" in text, True)
    check("위치 성공률", s["positions"]["success_rate"], 1.0)


def scenario_v1_and_usb(tmp):
    print("\n--- 3) v1 로그(TWR3T 없음)와 USB 줄 빠짐")
    tag = tmp / "t3.log"
    make_logs(tag, v1=True, cycles=100, plan={(3, 5): "lost"})
    s, text = m.analyze(tag, None, None, "", 2.5)
    check("v1: A3 Poll 손실", s["anchors"][3]["outcomes"]["poll_lost"], 1)
    check("v1: 묶음 시간 없음", s["timing"], None)
    check("v1: 안내 문구", "태그 v2 필요" in text, True)
    tag = tmp / "t4.log"
    make_logs(tag, cycles=50, drop_line_at=40)
    s, _ = m.analyze(tag, None, None, "", 2.5)
    check("TWR3T 한 줄 빠짐 → n 건너뜀 1", s["usb_lines_lost"], 1)


def make_logs7(path_tag, path_anchor=None, rows=((0, 600, 900, 700, 300),), cycle=0.060, plan=None,
               bundle_plan=None, mm_base=3010, with_info=True):
    """7패킷 태그 로그. rows = (표 줄 번호, S, D1, D2, 바퀴 수)들.
    plan[(앵커, 바퀴)] = 'lost'(Poll 손실) | 'T'(Final 안 옴), bundle_plan[바퀴] = 'L'(Response 늦음) | 'B'(늦게 알아챔)"""
    plan, bundle_plan = plan or {}, bundle_plan or {}
    t0 = (1 << 32) * m.HI_S - 3.0
    ant = 16385
    tag, anc = ["TWR3_READY,channel=5,tag=256"], ["A2_BOOT,fw=anchor7p-v1,session=0x22222222,packets=7"]
    if with_info:
        tag.append("TWR3_INFO,fw=tag7p-v1,packets=7,ant_dly=16385,s=600,d1=900,d2=700,sweep=0")
    sessions = {1: 0x11111111, 2: 0x22222222, 3: 0x33333333}
    n = bundle = stale = c = 0
    for row, s_us, d1, d2, count in rows:
        for _ in range(count):
            t = t0 + c * cycle
            got = [a for a in (1, 2, 3) if plan.get((a, c)) != "lost"]
            kind = bundle_plan.get(c, "R")
            for a in (1, 2, 3):
                if a == 2:
                    stages = 1 | (0 if a in got and kind == "R" else 8)
                    if a in got and kind == "R":
                        stages = 3 if plan.get((a, c)) == "T2" else 7      # T2: Response 를 받았으나 Final 을 못 보냄
                    anc.append(f"A2,{c + 1},{stages},-21000,-22000,1,00004000,230,{150 if stages & 4 else -1}")
            if not got:
                c += 1
                continue
            bundle += 1
            stale += kind == "B"
            ref = got[0]
            host = 1.7e9 + c * cycle + 0.045
            resp_hi = int(round((t + (2 * s_us + d1) * 1e-6 - ant * m.DTU_S) / m.HI_S)) % (1 << 32)
            finals = valid = ""
            for a in got:
                n += 1
                ok = kind == "R" and plan.get((a, c)) not in ("T", "T2")
                hi = int(round((t + (a - 1) * s_us * 1e-6) / m.HI_S)) % (1 << 32)
                end, end_ns = ("F", (2 * s_us + d1 + d2) * 1000) if ok else (kind if kind in "LB" else "T", 4_500_000)
                tag.append(f"{host:.6f}\tTWR3,{sessions[a]},{c + 1},{a},{int(kind == 'R')},{int(ok)},{int(ok)},"
                           f"{mm_base + a if ok else 0},-20000,-21000,-20500,-21500,00004000,00004080")
                tag.append(f"{host:.6f}\tTWR3T,{n},{a},{c + 1},{hi:08X},180,{end_ns},{end},0,0,{stale},{bundle}")
            polls = "".join("1" if a in got else "0" for a in (1, 2, 3))
            finals = "".join("1" if a in got and kind == "R" and plan.get((a, c)) not in ("T", "T2") else "0"
                             for a in (1, 2, 3))
            tag.append(f"{host:.6f}\tTWR3B,{bundle},{ref},{resp_hi:08X},{s_us},{d1},{d2},{polls},{kind},{finals},"
                       f"{finals},{-5 if kind == 'L' else 480},200,4000000,0,{row}")
            c += 1
    Path(path_tag).write_text("\n".join(tag) + "\n", encoding="utf-8")
    if path_anchor:
        Path(path_anchor).write_text("\n".join(anc) + "\n", encoding="utf-8")


def scenario_seven(tmp):
    print("\n--- 5) 7패킷: 묶음 번호와 TWR3B 줄, 일부 손실, 시각표, 앵커 로그")
    plan = {(1, 10): "lost", (2, 20): "lost", (2, 21): "lost", (3, 30): "T", (2, 40): "T2",
            (1, 50): "lost", (2, 50): "lost", (3, 50): "lost"}
    bundle_plan = {60: "L", 70: "B"}
    tag, anc, out = tmp / "t7.log", tmp / "a7.log", tmp / "out7"
    make_logs7(tag, anc, plan=plan, bundle_plan=bundle_plan)
    s, text = m.analyze(tag, anc, out, "시험7", 2.5)
    print(text)
    a = s["anchors"]
    check("7패킷으로 알아봄", s["packets"], 7)
    check("A1 Poll 손실(10, 50)", a[1]["outcomes"]["poll_lost"], 2)
    check("A1 성공 (300 - Poll 2 - 늦은 Response 1 - 늦게 알아챔 1)", a[1]["ok"], 296)
    check("A1 Response 를 못 보냄(L)", a[1]["outcomes"]["resp_not_sent"], 1)
    check("A1 태그 출력 중(B)", a[1]["tag_busy"], 1)
    check("A2 Poll 손실(20, 21, 50)", a[2]["outcomes"]["poll_lost"], 3)
    check("A2 최대 연속 실패(20, 21)", a[2]["max_consecutive_failures"], 2)
    check("A3 Final 안 옴", a[3]["outcomes"]["resp_or_final_lost"], 1)
    g = a[2]["anchor_log"]
    check("A2 앵커 로그 Poll 손실", g["poll_lost"], 3)
    check("A2 앵커 로그 Final 손실(보냈는데 못 받음)", g["final_lost"], 0)
    check("A2 앵커 로그: Final 을 보낸 횟수", g["final_sent"], 300 - 3 - 1 - 2)
    pos = s["positions"]
    check("위치 수 (세 Poll 을 다 놓친 바퀴 포함)", pos["positions"], 300)
    check("위치 성공", pos["ok"], 300 - 8)
    check("위치 최대 연속 실패(20, 21)", pos["max_consecutive_failures"], 2)
    tim = s["timing"]
    check("한 바퀴 ms", tim["cycle_ms"], 60.0, tol=0.001)
    g = tim["groups"][0]
    check("조합 (S, D1, D2)", (g["s_us"], g["d1_us"], g["d2_us"]), (600, 900, 700))
    check("시각표 µs = 4S + D1 + D2 + 프리앰블 + Final 뒷부분", g["schedule_us"],
          4000 + (m.SHR_S + m.FINAL_TAIL_S) * 1e6, tol=0.06)
    check("무선 점유 평균 µs", g["air_us"]["mean"], g["schedule_us"], tol=0.2)
    check("A2 Poll 자리 µs", g["poll_offset_us"][2]["mean"], 600.0, tol=0.02)
    check("Response 자리 µs", g["response_offset_us"]["mean"], 2100.0, tol=0.02)
    check("A3 Final 자리 µs", g["final_offset_us"][3]["mean"], 4000.0, tol=0.02)
    check("늦은 Response 횟수", g["response"].get("L"), 1)
    check("판정 문구(4.2 ms 는 목표 밖)", "목표 3.0 ms 이하 아니오" in text, True)
    check("거리 A2 평균 m", s["distances"][2]["mean_m"], 3.012, tol=1e-6)
    check("USB 줄 빠짐", s["usb_lines_lost"], 0)
    rows = (out / "bundles.csv").read_text(encoding="utf-8-sig").splitlines()
    check("bundles.csv 줄 수(머리줄 + 묶음 299)", len(rows), 300)
    check("bundles.csv 열", rows[0].split(",")[:8],
          ["host_time", "seg", "bundle", "row", "s_us", "d1_us", "d2_us", "ref_anchor"])

    print("\n--- 6) 7패킷: 값을 바꿔 가며 잰 로그 (줄마다 따로, 바뀐 직후 10묶음은 뺌)")
    tag = tmp / "t8.log"
    rows = ((0, 600, 900, 700, 60), (1, 400, 450, 450, 60), (2, 300, 350, 350, 60), (0, 600, 900, 700, 60))
    plan = {(3, c): "lost" for c in range(120, 180, 2)}        # 2번 줄에서 A3 Poll 을 절반 놓침
    make_logs7(tag, rows=rows, plan=plan)
    s, text = m.analyze(tag, None, None, "", 2.5)
    print(text[text.index("== 2."):text.index("== 3.")])
    groups = s["timing"]["groups"]
    check("조합 수", len(groups), 3)
    check("0 번 줄 시도 (두 번 나옴, 60 + 60 - 10 - 10)", groups[0]["attempts"], 100)
    check("1 번 줄 시각표 µs", groups[1]["schedule_us"], 2500 + (m.SHR_S + m.FINAL_TAIL_S) * 1e6, tol=0.06)
    check("1 번 줄 위치 성공률", groups[1]["position_success_rate"], 1.0)
    check("2 번 줄 위치 성공률 (A3 절반 손실)", groups[2]["position_success_rate"], 0.5, tol=0.001)
    check("2 번 줄 A3 성공률", groups[2]["anchor_success_rate"][3], 0.5, tol=0.001)
    best = m.best_group(groups)
    check("고른 조합 = 1 번 줄", (best["s_us"], best["d1_us"], best["d2_us"]), (400, 450, 450))
    check("고른 조합 문구", "가장 짧은 조합: S 400, D1 450, D2 450" in text, True)

    print("\n--- 7) 9패킷과 7패킷 견주기, 태그 시작 줄 없이 기록한 7패킷 로그")
    out9, out7, out7b = tmp / "c9", tmp / "c7", tmp / "c7b"
    make_logs(tmp / "t9.log", cycles=200)
    m.analyze(tmp / "t9.log", None, out9, "", 2.5)
    make_logs7(tmp / "t7a.log", rows=((0, 600, 900, 700, 200),), mm_base=3010)           # 9패킷보다 1 cm 김
    m.analyze(tmp / "t7a.log", None, out7, "", 2.5)
    make_logs7(tmp / "t7b.log", rows=((0, 600, 900, 700, 200),), mm_base=3030, with_info=False)   # 3 cm 김
    s, _ = m.analyze(tmp / "t7b.log", None, out7b, "", 2.5)
    check("TWR3_INFO 가 없어도 7패킷으로 알아봄", s["packets"], 7)
    text = m.compare(out9, out7)
    print(text)
    check("견주기: 1 cm 차이는 기준 안", "→ 기준 2 cm 안" in text, True)
    check("견주기: 차이 값", "| +1.0 | 예 |" in text, True)
    check("견주기: 3 cm 차이는 기준 밖", "→ 기준 2 cm 밖" in m.compare(out9, out7b), True)

    print("\n--- 8) 조건별 표 (사전 시험: 기준과 조건 두 개)")
    lossy = tmp / "c7_lossy"
    make_logs7(tmp / "t7c.log", rows=((0, 600, 900, 700, 200),), mm_base=3060,
               plan={(2, c): "lost" for c in range(50, 60)})                  # 앵커 2 를 10번 연달아 놓침, 5 cm 김
    m.analyze(tmp / "t7c.log", None, lossy, "", 2.5)
    text, rows = m.condition_table([out7, out7b, lossy])
    print(text)
    check("표의 줄 수 (조건 3 x 앵커 3)", len(rows), 9)
    check("기준 자신과의 차이 0", rows[0]["difference_from_base_cm"], 0.0)
    check("둘째 조건의 차이 +2 cm", rows[3]["difference_from_base_cm"], 2.0)
    worst = [r for r in rows if r["condition"] == "c7_lossy" and r["anchor"] == 2][0]
    check("셋째 조건 앵커 2 의 최대 연속 실패", worst["max_consecutive_failures"], 10)
    check("셋째 조건 앵커 2 의 성공률", worst["success_rate"], 190 / 200, tol=1e-9)
    check("셋째 조건의 차이 +5 cm", worst["difference_from_base_cm"], 5.0)


def scenario_stats():
    print("\n--- 4) 독립 손실 계산")
    check("p=0.5, n=2, 2연속", m.prob_run_at_least(2, 0.5, 2), 0.25, tol=1e-12)
    check("p=0.5, n=3, 2연속", m.prob_run_at_least(3, 0.5, 2), 3 / 8, tol=1e-12)
    check("p=0 기대 최대", m.expected_max_run(1000, 0.0), 0.0)
    e = m.expected_max_run(1000, 0.01)
    approx = math.log(1000 * 0.99) / math.log(100) + 0.5772 / math.log(100) - 0.5
    check("p=0.01, n=1000 기대 최대 연속(근사식과 비교)", e, approx, tol=0.3)


def main():
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        scenario_normal(tmp)
        scenario_packed(tmp)
        scenario_v1_and_usb(tmp)
        scenario_seven(tmp)
    scenario_stats()
    print(f"\n{'모두 통과' if not FAILS else '실패: ' + ', '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
