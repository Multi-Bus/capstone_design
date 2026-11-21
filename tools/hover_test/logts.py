"""읽기 전용: FC 로그 구간의 시계열. 사용: logts.py <log.bin> <t0> <t1> <step>
열: 거리센서(RFND), EKF 고도(CTUN.Alt), 기압 고도(CTUN.BAlt), 목표(DAlt), 스로틀 출력(ThO), 모터 PWM 평균, 전압(BAT)"""
import sys

from pymavlink import DFReader

path, t0, t1, step = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
r = DFReader.DFReader_binary(path, zero_time_base=True)
cur, nxt = {}, t0
print("t      dist  ekfAlt  baroAlt  tgt   ThO   pwm   volt   XKF3.IPD  XKF1.PD  XKFS?")
while True:
    m = r.recv_match(type=["RFND", "CTUN", "RCOU", "BAT", "XKF3", "XKF1", "XKFS"])
    if m is None:
        break
    t = m.TimeUS / 1e6
    if t > t1:
        break
    ty = m.get_type()
    if ty == "RFND" and m.Instance == 0:
        cur["dist"] = m.Dist
    elif ty == "CTUN":
        cur.update(alt=m.Alt, balt=m.BAlt, dalt=m.DAlt, tho=m.ThO)
    elif ty == "RCOU":
        cur["pwm"] = sum((m.C1, m.C2, m.C3, m.C4)) / 4
    elif ty == "BAT":
        cur["v"] = m.Volt
    elif ty == "XKF3" and m.C == 0:
        cur["ipd"] = m.IPD
    elif ty == "XKF1" and m.C == 0:
        cur["pd"] = m.PD
    elif ty == "XKFS" and m.C == 0:
        cur["src"] = (getattr(m, "SS", None), getattr(m, "OSS", None))
    if t >= nxt and t >= t0 and "alt" in cur and "dist" in cur:
        g = cur.get
        print(f"{t:6.1f} {g('dist',0):5.2f} {g('alt',0):6.2f} {g('balt',0):7.2f} {g('dalt',0):5.2f} {g('tho',0):5.2f} {g('pwm',0):6.0f} {g('v',0):5.2f}  {g('ipd',0):7.2f}  {g('pd',0):7.2f}  {g('src')}")
        nxt = t + step
