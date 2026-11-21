"""Read-only FC USB recorder: motor outputs, EKF height, rangefinder, attitude, battery. CSV at ~10 Hz.
Sends only stream-rate requests. usage: fc_rec.py <port> <secs> <out.csv>"""
import csv
import math
import sys
import time

from pymavlink import mavutil

port, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
m = mavutil.mavlink_connection(port, baud=115200)
hb = m.wait_heartbeat(timeout=10)
for mid, hz in ((36, 10), (32, 10), (132, 10), (30, 10), (1, 2)):  # SERVO_OUTPUT_RAW, LOCAL_POSITION_NED, DISTANCE_SENSOR, ATTITUDE, SYS_STATUS
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
cur = dict(armed=0, mode=0, m1=0, m2=0, m3=0, m4=0, ekf_h=0.0, dist=0.0, roll=0.0, pitch=0.0, volt=0.0)
w = csv.writer(open(out, "w", newline=""))
w.writerow(["t"] + list(cur))
t0, last = time.time(), 0.0
print("recording...", flush=True)
while time.time() - t0 < secs:
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg is not None:
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
            cur["armed"] = int(bool(msg.base_mode & 128)); cur["mode"] = msg.custom_mode
        elif t == "SERVO_OUTPUT_RAW":
            cur.update(m1=msg.servo1_raw, m2=msg.servo2_raw, m3=msg.servo3_raw, m4=msg.servo4_raw)
        elif t == "LOCAL_POSITION_NED":
            cur["ekf_h"] = round(-msg.z, 3)
        elif t == "DISTANCE_SENSOR":
            cur["dist"] = msg.current_distance / 100
        elif t == "ATTITUDE":
            cur["roll"] = round(math.degrees(msg.roll), 1); cur["pitch"] = round(math.degrees(msg.pitch), 1)
        elif t == "SYS_STATUS":
            cur["volt"] = msg.voltage_battery / 1000
    now = time.time() - t0
    if now - last >= 0.1:
        last = now
        w.writerow([round(now, 2)] + list(cur.values()))
print("done", flush=True)
