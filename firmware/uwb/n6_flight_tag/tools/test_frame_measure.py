"""frame_measure.py 시험: 가짜 기록(값을 아는 것)과 PC 모의시험 기록으로.

    python tools/test_frame_measure.py            # 전부 (모의시험기를 gcc 로 빌드한다. 2~3분)
    python tools/test_frame_measure.py --no-sim   # 가짜 기록만 (몇 초)

가짜 기록은 펌웨어(tag/tag_f.c 2판)가 내는 줄과 같은 형식으로 만든다: 손실, 줄 빠짐, 깨진 줄, 기체 재시작,
앵커 1 재시작, DW 시계 한 바퀴 넘김, 설정 변경, SD 에서 꺼낸 모양(PC 시각 없음), 비행 줄(C, E).
모의시험 빌드가 안 되면(펌웨어를 고치는 중일 때) 그 부분은 건너뛰고 그렇게 적는다.
"""
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / "sim"))
import frame_measure as fm  # noqa: E402

FAILS, COUNT, SKIPPED = [], [0], []
STEP_HI = (20000 * 63898) >> 8              # 프레임 하나가 DW 시각 상위 32비트로 얼마인가 (펌웨어와 같은 계산)
US_HI = 63898 / 256.0


def check(name, got, want=True, tol=None):
    COUNT[0] += 1
    if tol is None:
        ok = got == want
    else:
        ok = got is not None and abs(got - want) <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got!r}" + ("" if ok else f" (기대 {want!r}" + (f" ±{tol}" if tol else "") + ")"))
    if not ok:
        FAILS.append(name)


class Synth:
    """태그 N 대의 기록을 만든다. 설정은 (표시, 추가 슬롯 수), 손실은 함수로 준다."""

    def __init__(self, tags=6, host=True, f_fields=19):
        self.tags, self.host, self.f_fields = tags, host, f_fields
        self.lines = {t: [] for t in range(tags)}
        self.serial = {t: 0 for t in range(tags)}
        self.clock = 0.0
        self.base = {t: 0xFFF00000 + 7919 * t for t in range(tags)}   # 곧 한 바퀴 넘어가게 큰 값에서 시작
        self.started = set()
        self.logged_cfg = {}

    def emit(self, tag, text):
        self.lines[tag].append((f"{1700000000 + self.clock:.6f}\t" if self.host else "") + text)

    def boot(self, tag):
        self.emit(tag, "TF_READY,channel=5")
        self.emit(tag, f"TF_INFO,fw=tagf-v2,id={tag},tags={self.tags},addr={256 + tag},ant_dly=16385,records=512,sd=0,"
                       "flight=0,frame_us=20000,data_bytes=110,xchg_bytes=60,early_rearm=1,opt=test")
        self.logged_cfg.pop(tag, None)

    def frame(self, f, extra=1, flags=0, on=None, poll_ok=None, rx_ok=None, compute=True):
        """프레임 하나. on(t): 켜져 있나, poll_ok(t): 앵커 1 의 Poll 을 받았나, rx_ok(src, dst, slot, part): 받았나"""
        on = on or (lambda t: True)
        poll_ok = poll_ok or (lambda t: True)
        rx_ok = rx_ok or (lambda src, dst, slot, part: True)
        cfg = (flags, extra, 100, 60, 70, 600, 900, 700)
        times = fm.slot_times(cfg)
        self.clock = f * 0.02
        turn = f % self.tags
        radio_end = times["extra0"] + (extra - 1) * times["width"] + times["frag"] + 180 if extra else times["heartbeat"] + 780
        senders = [("H", 0, turn)] + [("B", j + 1, owner) for j, (owner, _) in enumerate(fm.round_robin(f, self.tags, extra))]
        complete = {t: 0 for t in range(self.tags)}
        for t in range(self.tags):
            if not on(t):
                continue
            if t not in self.started:
                self.started.add(t)
                self.boot(t)
            if poll_ok(t) and self.logged_cfg.get(t) != cfg:
                self.logged_cfg[t] = cfg
                self.emit(t, f"K,{f},0,{flags:02X},{extra},100,60,70,0,0,1500,255,30,50,600,900,700")
            if f % 250 in (0, 5):
                self.emit(t, f"L,{f},{int(f % 250 == 0)}")
            t0 = (self.base[t] + f * STEP_HI) & 0xFFFFFFFF
            if t == turn and poll_ok(t):
                for a in (1, 2, 3):
                    poll_hi = (t0 + int((a - 1) * 600 * US_HI)) & 0xFFFFFFFF
                    self.emit(t, f"R,{f},{a},{2000 + 100 * a + t},1,F,-13000,-13500,{poll_hi:08X},2800000")
        for kind, slot, src in senders:
            if not (on(src) and poll_ok(src)):
                continue
            serial = self.serial[src]
            self.serial[src] += 1
            for part in (0, 1):
                us = fm.expected_us(cfg, kind, slot, part)
                t_hi = (self.base[src] + f * STEP_HI + int(us * US_HI)) & 0xFFFFFFFF
                self.emit(src, f"P,{f},{kind},T,{slot},{part},{src},15,{serial},110,1,{us},-3,{t_hi:08X},32767,32767,0")
            for dst in range(self.tags):
                if dst == src or not on(dst):
                    continue
                got = [rx_ok(src, dst, slot, part) for part in (0, 1)]
                complete[dst] += all(got)
                for part in (0, 1):
                    if got[part]:
                        us = fm.expected_us(cfg, kind, slot, part)
                        t_hi = (self.base[dst] + f * STEP_HI + int(us * US_HI)) & 0xFFFFFFFF
                        known = poll_ok(dst)
                        self.emit(dst, f"P,{f},{kind},R,{slot},{part},{src},15,{serial},110,1,{us if known else 0},"
                                       f"{100 if known else 2147483647},{t_hi:08X},-12800,-13056,{67 * (src - dst)}")
        for t in range(self.tags):
            if not on(t):
                continue
            ok = poll_ok(t)
            t0 = (self.base[t] + f * STEP_HI) & 0xFFFFFFFF if ok else 0
            seen = 0x7F if ok else 0x08
            response = "R" if (t == turn and ok) else "-"
            text = f"F,{f},{t0:08X},{turn},{seen:02X},{response},{450 if response == 'R' else 0},{280 if response == 'R' else 0},0"
            if self.f_fields >= 16:
                text += f",{'A' if ok else '-'},{extra if ok else 0},{flags:02X},{radio_end if ok else 0},40,12,0"
            if self.f_fields >= 19:
                text += f",{9000 if ok and compute else 0},{9400 + t if ok and compute else 0},{complete[t]}"
            self.emit(t, text)

    def write(self, folder, only=None):
        folder.mkdir(parents=True, exist_ok=True)
        paths = []
        for t, lines in self.lines.items():
            if only is not None and t not in only:
                continue
            path = folder / f"tag{t}.raw.log"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            paths.append(path)
        return paths


def analyze(paths, **kw):
    logs = [fm.parse_tag_log(p) for p in paths]
    return fm.analyze(logs, [], **kw), logs


def fake_tests(tmp):
    print("--- 1) 손실 없는 가짜 기록 1000 프레임 (DW 시계가 한 바퀴 넘어감)")
    s = Synth()
    for f in range(1, 1001):
        s.frame(f)
    summary, logs = analyze(s.write(tmp / "clean"))
    seg = summary["segments"][0]
    check("기록이 있는 태그", summary["tags_logged"], [0, 1, 2, 3, 4, 5])
    check("설정 구간 수", len(summary["segments"]), 1)
    check("프레임 수", summary["tag"][2]["frames"], 1000)
    check("기준을 받은 비율", summary["tag"][2]["master_poll_rate"], 1.0)
    check("프레임 간격 µs (한 바퀴 넘김을 지나도 같음)", summary["tag"][0]["timing"]["frame_step_us"]["mean"], 20000.124, tol=0.002)
    check("프레임 간격의 흩어짐 µs", summary["tag"][0]["timing"]["frame_step_us"]["std"], 0.0, tol=0.001)
    check("앵커 1 과의 시계 차이 ppm", summary["tag"][0]["timing"]["clock_vs_anchor1_ppm"], 0.0, tol=0.02)
    check("위치 측정 성공률", summary["tag"][3]["position"]["success_rate"], 1.0)
    check("방송 조각 손실률", seg["broadcast_totals"]["all"]["fragment"]["loss_rate"], 0.0)
    check("heartbeat 조각 수 (1000 프레임 x 조각 2 x 받는 태그 5)", seg["broadcast_totals"]["H"]["fragment"]["sent"], 10000)
    check("추가 슬롯 방송 수 (1000 프레임 x 받는 태그 5)", seg["broadcast_totals"]["B"]["message"]["sent"], 5000)
    check("일정 수신율", seg["schedule"][1]["schedule_rate"], 1.0)
    check("내가 보낼 차례 (heartbeat 1/6 + 추가 슬롯 1/6)", seg["schedule"][1]["my_turns"], 334, tol=1)
    check("받은 것의 어긋남 평균 ns", seg["slot_error_ns"]["RH"]["mean"], 100.0)
    check("보낸 것의 어긋남 평균 ns", seg["slot_error_ns"]["TB"]["mean"], -3.0)
    check("제어 계산 시간 µs (태그 0)", seg["control_compute"][0]["compute_us"]["mean"], 400.0)
    check("제어 계산 시간 µs (태그 5)", seg["control_compute"][5]["compute_us"]["max"], 405.0)
    check("받은 패킷이 쓰이기까지 µs: 가장 긴 것 (heartbeat 첫 조각 5000 → 계산 시작 9000)",
          seg["control_compute"][0]["packet_to_use_us"]["max"], 4000.0, tol=1.0)
    check("받은 패킷이 쓰이기까지 µs: 가장 짧은 것 (추가 슬롯 둘째 조각 6800 → 9000)",
          seg["control_compute"][0]["packet_to_use_us"]["min"], 2200.0, tol=1.0)
    check("프레임당 다 받은 방송 수", seg["control_compute"][0]["broadcasts_received_per_frame"]["mean"], 1.67, tol=0.01)
    check("이웃 정보 나이의 최대 (heartbeat 와 추가 슬롯으로 6 프레임에 2번: 최대 4)", seg["neighbor_age_all"]["max"], 4)
    check("무선이 끝나는 때 µs (추가 슬롯 둘째 조각 6800 + 꼬리)", seg["radio_time"]["actual_end_us"]["max"],
          6800 + fm.tail_us(112), tol=0.1)
    tx = 2 * 2 * fm.airtime_us(112) / 6 + fm.airtime_us(36) / 6      # 프레임당: 방송 2개 x 조각 2 (6프레임에), Response (6프레임에 1)
    check("드론 한 대의 1초 송신 시간 ms", seg["tx_time_per_drone"][2]["tx_ms_per_s"], tx * 50 / 1000, tol=0.05)
    check("시계 차이 ppm (받은 태그 0 ← 보낸 태그 3: 67 x 3 / 2^26)", seg["clock_offset_between_tags"]["0<-3"]["mean_ppm"],
          67 * 3 * 1e6 / (1 << 26), tol=0.001)
    check("추정식 적중률 (손실 없음)", seg["delivery_estimator"]["heartbeat"]["ewma_hit_rate"], 1.0)
    check("LED 줄 수 (태그마다 8)", summary["flight"][0]["led_changes"], 8)
    check("깨진 줄 없음", summary["tag"][0]["bad_lines"], 0)
    fm.write_outputs(tmp / "clean" / "res", summary, logs)
    names = sorted(p.name for p in (tmp / "clean" / "res").iterdir())
    check("결과 파일", names, ["events.csv", "flight.csv", "frame_summary.json", "frame_summary.txt", "frames.csv",
                           "links.csv", "packets.csv", "ranges.csv", "sim_params.json", "sync_led.csv"])
    params = json.loads((tmp / "clean" / "res" / "sim_params.json").read_text(encoding="utf-8"))
    check("sim_params: 방송 조각 시간 µs", params["packet_time_us"]["broadcast_fragment_112B"], 293.3, tol=0.1)
    check("sim_params: 교환은 표본이 없으니 null", params["exchange_time_us"]["value"], None)
    check("sim_params: 위치 측정 시간 µs (4000 + 138.4 + Final 꼬리)", params["position_time_us"]["measured_mean"],
          4000 + fm.SHR_US + fm.tail_us(32), tol=0.5)
    check("sim_params: 계산 시간 평균 µs", params["compute_time_us"]["mean"], 402.5, tol=0.6)
    check("sim_params: 손실 모형 n", params["loss_model"]["drone_to_drone_fragment"]["n"], 20000)
    check("sim_params: 명령 지연은 C 줄이 없으니 null", params["command_delay_ms"]["n"], None)
    report = fm.report(summary)
    check("보고서에 7~17장이 있음", all(f"== {n}." in report for n in (7, 9, 10, 11, 12, 13, 14, 15, 16, 17)), True)

    print("--- 2) 정해 둔 손실: 태그 1 의 heartbeat 를 태그 2 가 프레임 7, 13, 19 (연달아 3번), 61, 121 에서 첫 조각을 잃음")
    lost = {7, 13, 19, 61, 121}
    s = Synth()
    for f in range(1, 601):
        s.frame(f, rx_ok=lambda src, dst, slot, part, f=f: not (src == 1 and dst == 2 and slot == 0 and part == 0 and f in lost))
    summary, _ = analyze(s.write(tmp / "loss"))
    seg = summary["segments"][0]
    link = seg["broadcast_links"]["1->2 H"]
    check("1→2 heartbeat 조각: 보낸 수", link["fragments_sent"], 200)
    check("1→2 heartbeat 조각: 잃은 수", link["fragments_lost"], 5)
    check("1→2 heartbeat 방송: 잃은 수", link["messages_lost"], 5)
    check("1→2 heartbeat 방송: 연달아 잃은 길이", link["message_burst_lengths"], {1: 2, 3: 1})
    check("1→2 heartbeat 방송: 연달아 잃은 최대", link["message_max_burst"], 3)
    check("다른 링크는 손실 없음", seg["broadcast_links"]["1->3 H"]["fragments_lost"], 0)
    check("손실 모형: 연달아 잃는 평균 길이 (5 / 3)", seg["broadcast_totals"]["H"]["message"]["mean_burst"], 1.667, tol=0.001)
    # 태그 1 은 프레임 ≡ 0 (추가 슬롯), ≡ 1 (heartbeat) 에 방송한다. heartbeat 를 놓치면 추가 슬롯에서 다음 추가 슬롯까지 6 프레임
    check("이웃 정보 나이의 최대 (태그 2: 보통 4, 태그 1 의 heartbeat 를 놓친 때 5)", seg["neighbor_age_frames"][2]["max"], 5)
    check("이웃 정보 나이의 최대 (다른 태그는 4)", seg["neighbor_age_frames"][3]["max"], 4)
    hits, n, _ = fm.ewma_score([f not in lost for f in range(1, 601) if f % 6 == 1])
    check("추정식: 손으로 돌린 값과 같음 (가장 나쁜 링크)", seg["delivery_estimator"]["heartbeat"]["ewma_hit_rate_worst_link"], hits / n)

    print("--- 3) 추정식 자체: p 0.7 에서 시작, 받음·못 받음·받음")
    hits, n, sq = fm.ewma_score([True, False, True])
    check("맞힌 수", (hits, n), (2, 3))
    check("제곱 오차의 합 (0.3² + 0.73² + 0.343²)", sq, 0.09 + 0.73 ** 2 + (1 - 0.657) ** 2, tol=1e-9)
    check("단순 기준 (늘 받는다, 직전 결과, 최근 10개)", fm.baseline_scores([True, False, True, True]), (3, 2, 3))

    print("--- 4) 줄 빠짐: 보낸 쪽 줄이 빠져도 받은 쪽 줄이 있으면 보낸 것으로 센다 / 받은 쪽 줄이 빠지면 손실로 보인다")
    s = Synth()
    for f in range(1, 301):
        s.frame(f)
    before = len(s.lines[5])
    s.lines[5] = [x for x in s.lines[5] if not ("\tP," in x and ",T," in x and ",H," in x)]      # 태그 5 가 보낸 heartbeat 줄을 모두 지움
    removed_tx = before - len(s.lines[5])
    victims = [i for i, x in enumerate(s.lines[4]) if "\tP,30,H,R,0," in x]                       # 태그 4 가 프레임 30 에서 받은 줄 2개
    for i in reversed(victims):
        del s.lines[4][i]
    summary, _ = analyze(s.write(tmp / "missing"))
    seg = summary["segments"][0]
    check("지운 송신 줄 수 (50번 x 조각 2)", removed_tx, 100)
    check("5→0 heartbeat 조각: 보낸 수는 그대로", seg["broadcast_links"]["5->0 H"]["fragments_sent"], 100)
    check("5→0 heartbeat 조각: 손실 없음", seg["broadcast_links"]["5->0 H"]["fragments_lost"], 0)
    check("0→4 heartbeat 조각: 지운 2줄이 손실로 보임", seg["broadcast_links"]["0->4 H"]["fragments_lost"], 2)
    check("태그 5: 일정은 있는데 보낸 기록 없음 (heartbeat 차례 50번)", seg["schedule"][5]["turns_with_schedule_but_nothing_sent"], 50)

    print("--- 5) 깨진 줄과 모르는 줄: 결과가 달라지지 않는다")
    s = Synth()
    for f in range(1, 301):
        s.frame(f)
    clean_summary, _ = analyze(s.write(tmp / "broken_ref"))
    stamp = "1700000001.000000\t"
    junk = [stamp + "P,12,H,R,0,0,1", stamp + "F,xx,00000000,1,7F", stamp + "\x00\x01garbage#$",
            stamp + "F,999999999,0BADF00D,1,7F,-,0,0,0,A,1,00,6980,40,12,0,0,0,0",             # 프레임 번호가 혼자 튄다
            stamp + "P,4000000,H,R,0,0,1,15,3,110,1,5000,100,00000000,-12800,-13056,0",
            stamp + "NEWLINE,1,2,3", stamp + "X9,hello"]
    for i, text in enumerate(junk):
        s.lines[3].insert(500 + 37 * i, text)
    at = next(i for i, x in enumerate(s.lines[3]) if "\tF,120," in x)
    s.lines[3][at + 1:at + 1] = [stamp + "E,120,abort_log,7,0", stamp + "E,121,weird_new_event,1,2"]
    summary, logs = analyze(s.write(tmp / "broken"))
    check("깨진 줄 수 (잘린 P, 숫자가 아닌 F, 튄 번호 2개)", summary["tag"][3]["bad_lines"], 4)
    check("모르는 줄", summary["unknown_lines"], {"\x00\x01garbage#$": 1, "NEWLINE": 1, "X9": 1})
    check("사건 줄 (모르는 이름도 센다)", summary["flight"][3]["events"], {"abort_log": 1, "weird_new_event": 1})
    check("프레임 수는 그대로", summary["tag"][3]["frames"], clean_summary["tag"][3]["frames"])
    check("손실률은 그대로", summary["segments"][0]["broadcast_totals"]["all"]["fragment"],
          clean_summary["segments"][0]["broadcast_totals"]["all"]["fragment"])
    check("설정 구간은 하나 그대로", len(summary["segments"]), 1)

    print("--- 6) 기체 재시작: 태그 3 이 프레임 300~399 동안 꺼져 있었다")
    s = Synth()
    for f in range(1, 601):
        if f == 400:
            s.started.discard(3)                 # 다시 켜짐: TF_READY, TF_INFO 가 또 나온다
        s.frame(f, on=lambda t, f=f: not (t == 3 and 300 <= f < 400))
    summary, logs = analyze(s.write(tmp / "restart"))
    seg = summary["segments"][0]
    check("태그 3 을 켠 횟수", summary["tag"][3]["restarts"], 2)
    check("태그 3 의 프레임 수 (꺼져 있던 100 은 뺌)", summary["tag"][3]["frames"], 500)
    check("태그 3 이 기준을 받은 비율", summary["tag"][3]["master_poll_rate"], 1.0)
    check("0→3 heartbeat: 꺼져 있던 동안은 손실로 세지 않음", seg["broadcast_links"]["0->3 H"]["fragments_lost"], 0)
    check("0→3 heartbeat: 보낸 수 (100번 중 켜져 있던 83번 x 2)", seg["broadcast_links"]["0->3 H"]["fragments_sent"], 166, tol=2)
    check("설정 구간은 하나 (K 줄이 다시 나와도 설정이 같음)", len(summary["segments"]), 1)

    print("--- 7) 앵커 1 재시작: 프레임 1~500 뒤에 다시 1~300")
    s = Synth()
    for f in list(range(1, 501)) + list(range(1, 301)):
        s.frame(f)
    summary, logs = analyze(s.write(tmp / "anchor_restart"))
    check("태그가 본 앵커 1 재시작", summary["tag"][0]["anchor1_restarts"], 1)
    check("프레임 수 (500 + 300)", summary["tag"][0]["frames"], 800)
    check("설정 구간 둘", [seg["frames"] for seg in summary["segments"]], [[1, 500], [1, 300]])
    check("두 번째 구간의 heartbeat 조각 수", summary["segments"][1]["broadcast_totals"]["H"]["fragment"]["sent"], 3000)
    check("손실 없음 (구간이 섞이지 않음)", summary["segments"][0]["broadcast_totals"]["all"]["fragment"]["lost"] +
          summary["segments"][1]["broadcast_totals"]["all"]["fragment"]["lost"], 0)
    check("깨진 줄 없음", summary["tag"][0]["bad_lines"], 0)

    print("--- 8) 설정 변경: 프레임 401 부터 추가 슬롯 3개")
    s = Synth()
    for f in range(1, 801):
        s.frame(f, extra=1 if f <= 400 else 3)
    summary, _ = analyze(s.write(tmp / "config"))
    check("설정 구간", [(seg["frames"], seg["config"]["extra_slots"]) for seg in summary["segments"]],
          [([1, 400], 1), ([401, 800], 3)])
    check("첫 구간의 추가 슬롯 방송 수", summary["segments"][0]["broadcast_totals"]["B"]["message"]["sent"], 2000)
    check("둘째 구간의 추가 슬롯 방송 수", summary["segments"][1]["broadcast_totals"]["B"]["message"]["sent"], 6000)

    print("--- 9) SD 에서 꺼낸 모양(PC 시각 없음), 1판의 짧은 F 줄, 칸 16개인 F 줄")
    a, b = Synth(host=True), Synth(host=False)
    for f in range(1, 201):
        a.frame(f)
        b.frame(f)
    sa, _ = analyze(a.write(tmp / "host"))
    sb, _ = analyze(b.write(tmp / "nohost"))
    check("PC 시각이 없어도 같은 값 (조각 손실, 무선 시간)",
          (sb["segments"][0]["broadcast_totals"], sb["segments"][0]["radio_time"]),
          (sa["segments"][0]["broadcast_totals"], sa["segments"][0]["radio_time"]))
    s16 = Synth(f_fields=16)
    for f in range(1, 201):
        s16.frame(f)
    summary, _ = analyze(s16.write(tmp / "f16"))
    check("칸 16개인 F 줄: 방송 손실 계산됨", summary["segments"][0]["broadcast_totals"]["all"]["fragment"]["sent"], 4000)
    check("칸 16개인 F 줄: 제어 계산 기록 없음", summary["segments"][0]["control_compute"], {})
    s9 = Synth(f_fields=9)
    for f in range(1, 201):
        s9.frame(f)
    for t in s9.lines:
        s9.lines[t] = [x for x in s9.lines[t] if "\tP," not in x and "\tK," not in x and "\tL," not in x]
    summary, _ = analyze(s9.write(tmp / "f9"))
    check("1판 기록: 프레임 수", summary["tag"][1]["frames"], 200)
    check("1판 기록: 7장부터는 없음", summary["segments"], [])
    check("1판 기록: sim_params 의 손실 모형은 null", summary["sim_params"]["loss_model"], None)

    print("--- 10) 보낸 쪽 기록이 없는 태그는 일정으로 추정")
    s = Synth()
    for f in range(1, 301):
        s.frame(f, rx_ok=lambda src, dst, slot, part, f=f: not (src == 5 and dst == 0 and f == 59))
    summary, _ = analyze(s.write(tmp / "nosender", only=[0, 1, 2, 3, 4]))
    seg = summary["segments"][0]
    check("일정으로 추정한 태그", seg["senders_estimated_from_schedule"], [5])
    check("5→0 heartbeat: 프레임 59 의 조각 2개가 손실", seg["broadcast_links"]["5->0 H"]["fragments_lost"], 2)
    check("5→1 heartbeat: 손실 없음", seg["broadcast_links"]["5->1 H"]["fragments_lost"], 0)
    check("추정했다는 표시", seg["broadcast_links"]["5->0 H"]["sender_estimated"], True)

    print("--- 11) 비행 줄: 명령이 프레임 1000 에 0 → 0.3 m/s. 비행 제어기 속도는 0.2 s 뒤부터 시정수 0.5 s 로 따라감")
    s = Synth(tags=2)
    for f in range(900, 1300):
        s.frame(f, extra=0)
        for t in (0, 1):
            command = 300 if f >= 1000 else 0
            fresh = f % 5 == 0                       # 비행 제어기 값은 5 프레임(0.1 s)마다, 20 ms 전의 것
            if not fresh and f != 1000:
                continue
            when = (f - 1000) * 0.02 - 0.02
            v = 300 * (1 - math.exp(-(when - 0.2) / 0.5)) if when > 0.2 else 0.0
            # 속도는 북·동으로 30도 돌아간 방향 (명령과 축이 달라도 크기로 견주는지 본다)
            vn, ve = v * math.cos(math.radians(30)), v * math.sin(math.radians(30))
            s.emit(t, f"C,{f},{int(v * when)},0,-1000,{vn:.0f},{ve:.0f},0,10,-20,3000,20,15,4,1,2,{max(0, f - 1000)},"
                      f"{command},0,{int(f == 1000 or fresh)}")
        if f == 1000:
            s.emit(0, "E,1000,start,1,0")
    summary, logs = analyze(s.write(tmp / "flight"))
    r = summary["command_response"][0]
    check("명령이 바뀐 횟수", r["steps"], 1)
    check("속도가 변하기 시작한 때 ms (10 %: 0.253 s 뒤의 첫 표본 0.28 s)", r["start_ms"]["mean"], 280.0)
    check("63 % 에 이른 때 ms (0.697 s 뒤의 첫 표본 0.78 s)", r["t63_ms"]["mean"], 780.0)
    check("C 줄 수", summary["flight"][0]["c_lines"], 80, tol=2)
    check("대본 상태", list(summary["flight"][0]["script_states"]), ["실행"])
    check("가장 빠른 속도 m/s", summary["flight"][0]["speed_max_m_s"], 0.30, tol=0.005)
    check("sim_params: 명령 지연", (summary["sim_params"]["command_delay_ms"]["start_mean"],
                                summary["sim_params"]["command_delay_ms"]["t63_mean"]), (280.0, 780.0))
    fm.write_outputs(tmp / "flight" / "res", summary, logs)
    rows = (tmp / "flight" / "res" / "flight.csv").read_text(encoding="utf-8-sig").splitlines()
    check("flight.csv 줄 수 (머리 + 태그 2대)", len(rows), 1 + 2 * summary["flight"][0]["c_lines"])
    check("sync_led.csv 에 프레임 1000 의 LED", any(x.startswith("0,0,1000,1,") for x in
          (tmp / "flight" / "res" / "sync_led.csv").read_text(encoding="utf-8-sig").splitlines()), True)

    print("--- 12) 공중 시간 계산식")
    check("방송 조각 112바이트 µs", fm.airtime_us(112), 138.4 + 21 * 1.02564 + (896 + 48 * 3) * 0.12821, tol=0.01)
    # 프레임 13: 차례 1, 주인 2, 3, 4. 상대 = 주인 + 1 + (13 // 6) % 5 = 주인 + 3
    check("차례대로 도는 일정 (프레임 13, 6대, 추가 3)", fm.round_robin(13, 6, 3), [(2, 5), (3, 0), (4, 1)])
    check("구간 시각 (일정을 드론이 보내면 frag 만큼 밀림)", fm.expected_us((0x02, 1, 100, 60, 70, 600, 900, 700), "B", 1, 1),
          4000 + 1000 + 600 + 1200 + 600)


def sim_tests(tmp):
    try:
        import run_sim
    except Exception as error:                      # noqa: BLE001
        SKIPPED.append(f"모의시험: run_sim 을 못 읽음 ({error})")
        return

    def build(name, defs):
        try:
            return run_sim.build_frame(tmp / "simbuild", name, anchor_defs=defs)
        except SystemExit as error:
            SKIPPED.append(f"모의시험 {name}: 빌드 실패 ({str(error)[:200]})")
            return None

    def run(binary, out, *args):
        out.mkdir(parents=True)
        done = subprocess.run([str(binary), "tags=6", f"out={out}", *args], capture_output=True, text=True)
        logs = [fm.parse_tag_log(p) for p in sorted(out.glob("tag*.raw.log"))]
        anchors = [fm.parse_anchor_log(p) for p in sorted(out.glob("a?.raw.log"))]
        return done, fm.analyze(logs, anchors), logs

    print("--- S1) 모의시험: 추가 슬롯 3개(방송), 손실 5 %, 60 s")
    binary = build("k3", ("-DANCHOR_EXTRA=3U",))
    if binary:
        done, summary, logs = run(binary, tmp / "s1", "seconds=60", "seed=5", "loss=0.05")
        seg = summary["segments"][0]
        check("모의시험이 끝남", "SIM_DONE" in done.stdout, True)
        check("설정: 추가 슬롯 3, 방송, 앵커 1", (seg["config"]["extra_slots"], seg["config"]["extra_type"],
                                          seg["config"]["coordinator"]), (3, "broadcast", "anchor1"))
        check("드론 → 드론 조각 손실률 (0.05)", seg["broadcast_totals"]["all"]["fragment"]["loss_rate"], 0.05, tol=0.004)
        check("방송(조각 둘 다) 손실률 (1 - 0.95² = 0.0975)", seg["broadcast_totals"]["all"]["message"]["loss_rate"], 0.0975, tol=0.006)
        check("연달아 잃는 평균 길이 (독립 손실이면 1 / 0.9025 = 1.108)", seg["broadcast_totals"]["all"]["message"]["mean_burst"],
              1.108, tol=0.03)
        check("앵커 1 의 Poll 손실률 (0.05)", summary["anchor1_poll_loss"]["loss_rate"], 0.05, tol=0.006)
        check("일정 수신율 (0.95)", seg["schedule"][2]["schedule_rate"], 0.95, tol=0.015)
        check("일정이 없어 못 보낸 차례의 비율 (0.05)", sum(v["turns_blocked_no_schedule"] for v in seg["schedule"].values()) /
              sum(v["my_turns"] for v in seg["schedule"].values()), 0.05, tol=0.01)
        check("받은 것의 어긋남: 절댓값의 99 % 가 0.3 µs 미만", max(v["abs_p99"] for v in seg["slot_error_ns"].values()) < 300, True)
        check("예약이 늦어 못 보낸 것 없음", seg["late_transmissions"], {})
        check("무선이 끝나는 때가 펌웨어 계산값 이하", seg["radio_time"]["actual_end_us"]["max"] <=
              seg["radio_time"]["planned_end_us"]["max"], True)
        check("60 % 를 넘은 프레임 없음", seg["radio_time"]["frames_over_60_percent"], 0)
        check("추정식 적중률 = 늘 받는다고 한 것과 비슷 (독립 손실)", seg["delivery_estimator"]["heartbeat"]["ewma_hit_rate"],
              seg["delivery_estimator"]["heartbeat"]["always_success_hit_rate"], tol=0.01)
        check("기록 버림 없음", max(summary["tag"][t]["log_dropped"] or 0 for t in range(6)), 0)
        check("깨진 줄 없음", sum(summary["tag"][t]["bad_lines"] for t in range(6)), 0)
        check("태그 → 앵커 Response 손실 (0.05)", summary["sim_params"]["anchor_loss"]["drone_to_anchor_response"]["mean_loss"],
              0.05, tol=0.01)
        ppm = summary["tag"][0]["timing"]["clock_vs_anchor1_ppm"]
        truth = [float(x) for x in done.stdout.split("ppm=")[1].split()[0].split("/")]
        check("앵커 1 과의 시계 차이 ppm (모의시험이 넣은 값: 태그 - 앵커 1)", ppm, truth[0] - truth[1], tol=0.05)
        pair = seg["clock_offset_between_tags"]
        check("태그 사이 시계 차이: 0←1 과 1←0 은 부호만 다름", pair["0<-1"]["mean_ppm"], -pair["1<-0"]["mean_ppm"], tol=0.02)
        # 공중 시간: 모의시험기가 적은 값과 같은가
        air = [x.split(",") for x in (tmp / "s1" / "air.csv").read_text().splitlines()[1:]]
        data = next(x for x in air if x[1] == "69")
        check("방송 조각의 공중 시간 µs (모의시험기의 값)", float(data[5]) - float(data[3]), fm.airtime_us(112), tol=0.1)
        # SD 에서 꺼낸 것처럼 PC 시각을 떼어도 같은 결과
        bare = tmp / "s1_bare"
        bare.mkdir()
        for p in sorted((tmp / "s1").glob("tag*.raw.log")):
            (bare / p.name).write_text("\n".join(x.split("\t", 1)[1] for x in p.read_text().splitlines()) + "\n")
        summary2 = fm.analyze([fm.parse_tag_log(p) for p in sorted(bare.glob("tag*.raw.log"))], [])
        check("PC 시각을 뗀 기록도 같은 손실률", summary2["segments"][0]["broadcast_totals"], seg["broadcast_totals"])

    print("--- S2) 모의시험: 추가 슬롯 2개를 교환으로, 손실 5 %, 60 s")
    binary = build("x2", ("-DANCHOR_EXTRA=2U", "-DANCHOR_EXCHANGE=1"))
    if binary:
        done, summary, logs = run(binary, tmp / "s2", "seconds=60", "seed=7", "loss=0.05")
        seg = summary["segments"][0]
        e = seg["exchange_total"]
        wrong = sum(1 for log in logs for p in log.packets
                    if p.type == "X" and p.dir == "T" and p.part == 0 and
                    fm.round_robin(p.frame, 6, 2)[p.slot - 1] != (p.src, p.dst))
        check("교환의 주인과 상대가 도구의 일정 계산(round_robin)과 같음", wrong, 0)
        check("설정: 교환", seg["config"]["extra_type"], "exchange")
        check("교환을 시작한 수 (프레임 3000 x 2 x 0.95 쯤)", e["started"], 5700, tol=250)
        check("0 단계 도착 (0.95)", e["step0_rate"], 0.95, tol=0.01)
        check("1 단계까지 (상대도 그 프레임의 기준을 받아야 답한다: 0.95³ = 0.857)", e["step1_rate"], 0.857, tol=0.015)
        check("셋 다 (0.95⁴ = 0.815)", e["complete_rate"], 0.815, tol=0.015)
        check("heartbeat 방송 손실 (0.0975)", seg["broadcast_totals"]["H"]["message"]["loss_rate"], 0.0975, tol=0.01)
        check("sim_params: 교환 시간 µs (2 x 700 + 62바이트 패킷)", summary["sim_params"]["exchange_time_us"]["value"],
              1400 + fm.airtime_us(62), tol=0.1)

    print("--- S3) 모의시험: 일정을 드론 0번이 보냄, 손실 5 %, 40 s")
    binary = build("d1", ("-DANCHOR_COORD_DRONE=1", "-DANCHOR_EXTRA=1U"))
    if binary:
        done, summary, _ = run(binary, tmp / "s3", "seconds=40", "seed=9", "loss=0.05")
        seg = summary["segments"][0]
        check("설정: 드론 0번", seg["config"]["coordinator"], "drone0")
        check("드론 0번의 일정 수신율 (스스로 만든다: 0.95)", seg["schedule"][0]["schedule_rate"], 0.95, tol=0.015)
        others = [seg["schedule"][t]["schedule_rate"] for t in range(1, 6)]
        check("다른 드론의 일정 수신율 (0번의 기준 x 내 기준 x 일정 패킷: 0.95³ = 0.857)", sum(others) / 5, 0.857, tol=0.02)
        check("일정 패킷의 손실률 (0.05)", seg["schedule_packet_links"]["0->3"]["loss_rate"], 0.05, tol=0.015)
        check("일정 패킷만 놓친 프레임이 있음", seg["schedule"][3]["schedule_packet_missed"] > 50, True)
        check("추가 슬롯 방송의 조각 손실률 (0.05)", seg["broadcast_totals"]["B"]["fragment"]["loss_rate"], 0.05, tol=0.006)
        print(f"   (참고) 이 설정에서 예약이 늦어 못 보낸 것: {seg['late_transmissions'] or '없음'}, "
              f"heartbeat 방송 손실 {fm.pct(seg['broadcast_totals']['H']['message']['loss_rate'])}")


def main():
    with tempfile.TemporaryDirectory(prefix="fm_test_") as tmp:
        tmp = Path(tmp)
        fake_tests(tmp)
        if "--no-sim" in sys.argv:
            SKIPPED.append("모의시험 (--no-sim)")
        else:
            sim_tests(tmp)
    for text in SKIPPED:
        print(f"건너뜀: {text}")
    print(f"\n확인 {COUNT[0]}개, " + ("모두 통과" if not FAILS else f"실패 {len(FAILS)}개: " + ", ".join(FAILS)))
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
