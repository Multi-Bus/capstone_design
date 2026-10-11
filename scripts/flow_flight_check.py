"""GPS 끈 광류 비행 점검: 이륙 -> 정지 -> 북/동 0.5m/s 이동. AP 추정 위치를 Gazebo 실제 위치와 비교한다."""
import sys, time
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pymavlink import mavutil
from get_truth import Truth

truth = Truth()
link = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=250)
assert link.wait_heartbeat(timeout=90), "heartbeat 없음"
link.mav.request_data_stream_send(link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
texts = []
last = {"pos": None, "ekf": None}
def pump(sec):
    end = time.time() + sec
    while time.time() < end:
        m = link.recv_match(blocking=True, timeout=0.3)
        if m is None: continue
        t = m.get_type()
        if t == "STATUSTEXT":
            texts.append(m.text); print("  [STATUSTEXT]", m.text, flush=True)
        elif t == "COMMAND_ACK" and m.command in (22, 400):
            print("  [ACK] cmd", m.command, "result", m.result, flush=True)
        elif t == "LOCAL_POSITION_NED": last["pos"] = m
        elif t == "EKF_STATUS_REPORT": last["ekf"] = m

def tr():
    d = truth.get(0, model_name="iris_flow_test")
    return d.get("position_enu_m")

print("sys", link.target_system, flush=True)
t0 = time.time()
while time.time() - t0 < 150 and not any("EKF3 IMU0 initial yaw alignment" in x or "yaw alignment complete" in x for x in texts):
    pump(2)
pump(20)   # 광류/거리센서가 EKF에 들어갈 시간
e = last["ekf"]
print("EKF flags:", None if e is None else hex(e.flags), flush=True)

t0 = time.time()
while time.time() - t0 < 30 and link.flightmode != "GUIDED":
    link.set_mode("GUIDED"); pump(1.5); link.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
print("모드:", link.flightmode, flush=True)
t0 = time.time()
while time.time() - t0 < 90 and not link.motors_armed():
    link.arducopter_arm(); pump(3)
print("armed:", bool(link.motors_armed()), flush=True)
if not link.motors_armed():
    sys.exit(1)

res = None
for attempt in range(5):
    link.mav.command_long_send(link.target_system, link.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 2)
    end = time.time() + 3
    while time.time() < end:
        m = link.recv_match(blocking=True, timeout=0.3)
        if m is None: continue
        if m.get_type() == "STATUSTEXT": print("  [STATUSTEXT]", m.text, flush=True)
        if m.get_type() == "COMMAND_ACK" and m.command == 22: res = m.result
    print("  takeoff 시도", attempt + 1, "결과", res, flush=True)
    if res == 0: break

def report(tag):
    p = last["pos"]; g = tr()
    if p and g:
        print(f"  [{tag}] AP(N,E,D)=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f})  truth ENU=({g[0]:+.2f},{g[1]:+.2f},{g[2]:+.2f})  "
              f"AP 속도(N,E)=({p.vx:+.2f},{p.vy:+.2f})", flush=True)

def vel_cmd(vn, ve, vd):
    link.mav.set_position_target_local_ned_send(0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)

pump(12); report("정지 12s")
# 북쪽(+N) 0.5 m/s로 8초: 실제로는 약 4m 이동해야 함
t0 = time.time(); k = 0
while time.time() - t0 < 8:
    vel_cmd(0.5, 0.0, 0.0); pump(0.2); k += 1
    if k % 10 == 0: report(f"북 0.5m/s {time.time()-t0:3.0f}s")
for _ in range(25):
    vel_cmd(0.0, 0.0, 0.0); pump(0.2)
report("정지 후 5s")
# 동쪽(+E) 0.5 m/s로 8초
t0 = time.time(); k = 0
while time.time() - t0 < 8:
    vel_cmd(0.0, 0.5, 0.0); pump(0.2); k += 1
    if k % 10 == 0: report(f"동 0.5m/s {time.time()-t0:3.0f}s")
for _ in range(25):
    vel_cmd(0.0, 0.0, 0.0); pump(0.2)
report("정지 후 5s")
link.set_mode("LAND")
pump(8)
