"""Read-only: FC USB. Reads a few params + samples sensor messages for N seconds. No arm/mode/motor/param-write."""
import sys
import time

from pymavlink import mavutil

port = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10
m = mavutil.mavlink_connection(port, baud=115200)
hb = m.wait_heartbeat(timeout=10)
print("heartbeat armed:", bool(hb.base_mode & 128), "custom_mode", hb.custom_mode)
for n in ("SERVO_BLH_RVMASK", "FLOW_ORIENT_YAW", "SYSID_THISMAV", "BATT_LOW_VOLT", "BATT_FS_LOW_ACT"):
    m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
    p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    print(n, p.param_value if p else None)
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
last, t0 = {}, time.time()
txt = []
while time.time() - t0 < secs:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "STATUSTEXT":
        txt.append(msg.text)
    elif t in ("DISTANCE_SENSOR", "OPTICAL_FLOW", "LOCAL_POSITION_NED", "ATTITUDE", "SYS_STATUS", "EKF_STATUS_REPORT", "VIBRATION"):
        last[t] = msg
d = last.get("DISTANCE_SENSOR")
print("DISTANCE_SENSOR m:", d.current_distance / 100 if d else None, "orient", d.orientation if d else None)
o = last.get("OPTICAL_FLOW")
print("OPTICAL_FLOW quality:", o.quality if o else None, "flow_x/y", (o.flow_x, o.flow_y) if o else None, "ground_distance", o.ground_distance if o else None)
l = last.get("LOCAL_POSITION_NED")
print("EKF height (-z) m:", -l.z if l else None)
a = last.get("ATTITUDE")
if a:
    import math
    print("roll/pitch deg:", round(math.degrees(a.roll), 1), round(math.degrees(a.pitch), 1))
s = last.get("SYS_STATUS")
print("battery V:", s.voltage_battery / 1000 if s else None)
e = last.get("EKF_STATUS_REPORT")
print("EKF flags:", e.flags if e else None)
v = last.get("VIBRATION")
print("vibe xyz:", (round(v.vibration_x, 1), round(v.vibration_y, 1), round(v.vibration_z, 1), v.clipping_0) if v else None)
print("STATUSTEXT:", txt[-8:])
