"""읽기 전용: ARM 구간별 요약. 실제 높이(거리센서)와 EKF 높이, 기압 높이의 차이를 본다.
사용: logwin.py <log.bin>"""
import sys

from pymavlink import DFReader

r = DFReader.DFReader_binary(sys.argv[1], zero_time_base=True)
arm = 0.0
cur = {}
win = []
rows = []
while True:
    m = r.recv_match(type=["EV", "RFND", "CTUN", "BAT", "RCOU"])
    if m is None:
        break
    t = m.TimeUS / 1e6
    ty = m.get_type()
    if ty == "EV" and m.Id in (10, 11):
        if m.Id == 10:
            arm = t; rows = []
        elif arm is not None:
            win.append((arm, t, rows)); arm = None
    elif ty == "RFND" and m.Instance == 0:
        cur["d"] = m.Dist
    elif ty == "BAT":
        cur["v"] = m.Volt
    elif ty == "RCOU":
        cur["pwm"] = max(m.C1, m.C2, m.C3, m.C4)
    elif ty == "CTUN" and arm is not None and "d" in cur:
        rows.append((t, cur["d"], m.Alt, m.BAlt, m.ThO, cur.get("v", 0), cur.get("pwm", 0)))
if arm is not None:
    win.append((arm, t, rows))
for a, b, rows in win:
    if len(rows) < 20:
        continue
    d = [x[1] for x in rows]; e = [x[2] for x in rows]; ba = [x[3] for x in rows]; th = [x[4] for x in rows]
    v = [x[5] for x in rows if x[5] > 1]; pw = [x[6] for x in rows]
    on = [x for x in rows if x[4] > 0.1]
    gap = max((x[2] - x[1] for x in on), default=0)
    print(f"ARM {a:6.1f}-{b:6.1f} ({b-a:4.0f}s)  실제(거리) max {max(d):.2f}  EKF max {max(e):.2f}  기압 max {max(ba):.2f}  EKF-실제 최대 {gap:.2f}  ThO max {max(th):.2f}  PWM max {max(pw)}  전압 min {min(v) if v else 0:.2f}")
