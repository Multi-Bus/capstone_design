from pymavlink import mavutil
m = mavutil.mavlink_connection("COM37", baud=115200); m.wait_heartbeat(timeout=10)
def rd():
    m.mav.param_request_read_send(m.target_system, m.target_component, b"FLOW_ORIENT_YAW", -1)
    return m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
p = rd(); print("before", p.param_value, "type", p.param_type)
assert p.param_value == 0.0, "unexpected current value"
m.mav.param_set_send(m.target_system, m.target_component, b"FLOW_ORIENT_YAW", 18000.0, p.param_type)
import time; time.sleep(1)
q = rd(); print("after", q.param_value)
