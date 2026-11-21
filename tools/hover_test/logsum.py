"""읽기 전용: FC 로그(.bin) 요약. 파라미터, ARM 구간, 구간별 고도(실제/EKF/기압)와 스로틀.
사용: logsum.py <log.bin>"""
import sys
from collections import defaultdict

from pymavlink import DFReader

r = DFReader.DFReader_binary(sys.argv[1], zero_time_base=True)
want = {"EK3_SRC1_POSZ", "EK3_RNG_USE_HGT", "EK3_RNG_USE_SPD", "RNGFND1_MAX_CM", "RNGFND1_MIN_CM", "RNGFND1_TYPE",
        "MOT_THST_HOVER", "MOT_SPIN_MAX", "EK3_SRC1_VELXY", "FLOW_ORIENT_YAW", "GND_EFFECT_COMP", "EK3_ALT_SOURCE", "EK3_ABIAS_P_NSE", "EK3_BCOEF_X", "EK3_BCOEF_Y", "BARO_ALT_OFFSET", "EK3_GND_EFF_DZ", "EK3_HGT_DELAY"}
params, arms, rows = {}, [], defaultdict(list)
armed = None
while True:
    m = r.recv_match()
    if m is None:
        break
    t = m.get_type()
    if t == "PARM" and m.Name in want:
        params[m.Name] = m.Value
    elif t == "EV" and m.Id in (10, 11):
        arms.append((round(m.TimeUS / 1e6, 1), "ARM" if m.Id == 10 else "DISARM"))
    elif t in ("CTUN", "RFND", "BARO", "XKF1", "XKF3", "XKF4", "MODE", "MSG", "XKFS", "RCOU"):
        rows[t].append(m)
print("params:", params)
print("events:", arms)
print("counts:", {k: len(v) for k, v in rows.items()})
for m in rows["MSG"][:40]:
    print(round(m.TimeUS / 1e6, 1), m.Message)
