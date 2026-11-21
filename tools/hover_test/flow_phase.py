"""Read-only per-phase optical-flow check. usage: flow_phase.py rec <port> <label> <secs>  |  flow_phase.py analyze
Sends only SET_MESSAGE_INTERVAL (runtime stream rate). No arm/mode/motor/param-write."""
import math
import sys
import time

import numpy as np

LEAD = 3.0  # seconds ignored at the start of each phase (time for the user to start moving)


def rec(port, label, secs):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(port, baud=115200)
    m.wait_heartbeat(timeout=10)
    for mid, hz in ((30, 25), (100, 25)):
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
    att, flo, t0 = [], [], time.time()
    while time.time() - t0 < secs:
        msg = m.recv_match(type=["ATTITUDE", "OPTICAL_FLOW"], blocking=True, timeout=1)
        if msg is None:
            continue
        t = time.time() - t0
        if msg.get_type() == "ATTITUDE":
            att.append((t, msg.rollspeed, msg.pitchspeed, msg.roll, msg.pitch))
        else:
            flo.append((t, msg.flow_x, msg.flow_y, getattr(msg, "flow_rate_x", 0.0), getattr(msg, "flow_rate_y", 0.0), msg.quality))
    np.savez(f"flowphase_{label}.npz", att=np.array(att), flo=np.array(flo))
    a = np.array(att)
    print(label, "saved; att", len(att), "flow", len(flo), "max tilt roll/pitch deg",
          round(math.degrees(np.abs(a[:, 3]).max()), 1), round(math.degrees(np.abs(a[:, 4]).max()), 1))


def corr(a, b):
    a, b = a - a.mean(), b - b.mean()
    d = math.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / d) if d > 1e-12 else float("nan")


def analyze():
    for label, ref in (("roll", 1), ("pitch", 2)):
        z = np.load(f"flowphase_{label}.npz")
        att, flo = z["att"], z["flo"]
        t = np.arange(LEAD, min(att[-1, 0], flo[-1, 0]) - 0.5, 0.04)
        g = np.interp(t, att[:, 0], att[:, ref])
        tilt = np.degrees(np.abs(np.interp(t, att[:, 0], att[:, 3 if ref == 1 else 4])).max())
        print(f"\n{label.upper()} phase: max tilt {tilt:.1f} deg, gyro std {g.std():.2f} rad/s, flow quality {flo[:, 5].mean():.0f}")
        if tilt < 8:
            print("  tilt too small -> not valid")
            continue
        for name, i in (("flow_x", 1), ("flow_y", 2), ("rate_x", 3), ("rate_y", 4)):
            f = np.interp(t, flo[:, 0], flo[:, i])
            print(f"  corr({name}, {'rollspd' if ref == 1 else 'pitchspd'}) = {corr(f, g):+.2f}  std {f.std():.3f}")


if sys.argv[1] == "rec":
    rec(sys.argv[2], sys.argv[3], float(sys.argv[4]))
else:
    analyze()
