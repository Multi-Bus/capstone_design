from pymavlink import mavutil
import time
m = mavutil.mavlink_connection("COM37", baud=115200); m.wait_heartbeat(timeout=10)
for _ in range(3):
    m.mav.param_request_read_send(m.target_system, m.target_component, b"FLOW_ORIENT_YAW", -1)
    t0 = time.time()
    while time.time() - t0 < 2:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p is None: continue
        pid = p.param_id if isinstance(p.param_id, str) else p.param_id.decode()
        print(pid, p.param_value, p.param_type)
        if pid.startswith("FLOW_ORIENT_YAW"): break
