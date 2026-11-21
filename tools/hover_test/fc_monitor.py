"""Read-only FC monitor over USB: logs arm/mode changes and STATUSTEXT with timestamps. No commands sent (except stream rate)."""
import sys
import time

from pymavlink import mavutil

port, secs = sys.argv[1], float(sys.argv[2])
out = open(sys.argv[3], "w", encoding="utf-8", buffering=1)
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 4: "GUIDED", 5: "LOITER", 9: "LAND", 16: "POSHOLD"}


def log(s):
    line = time.strftime("%H:%M:%S ") + s
    print(line, flush=True)
    out.write(line + "\n")


m = mavutil.mavlink_connection(port, baud=115200)
m.wait_heartbeat(timeout=10)
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
armed = mode = None
t0 = time.time()
last_hb = time.time()
log("monitor start")
while time.time() - t0 < secs:
    msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT", "SYS_STATUS"], blocking=True, timeout=1)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
        a = bool(msg.base_mode & 128)
        md = MODES.get(msg.custom_mode, msg.custom_mode)
        if a != armed or md != mode:
            log(f"ARMED={a} MODE={md}")
            armed, mode = a, md
    elif t == "STATUSTEXT":
        log("FC: " + msg.text)
log("monitor end")
