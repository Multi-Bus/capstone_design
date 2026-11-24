"""FC 파라미터 하나를 USB로 바꾸고 읽어서 확인한다. 시동이 걸려 있으면 거부한다.
사용: fc_setparam.py <port> <이름> <값>"""
import sys
import time

from pymavlink import mavutil

port, name, val = sys.argv[1], sys.argv[2], float(sys.argv[3])
m = mavutil.mavlink_connection(port, baud=115200)
hb = m.wait_heartbeat(timeout=10)
if hb.base_mode & 128:
    sys.exit("시동 상태: 거부")


def rd():
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 4:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p is not None and p.param_id == name:
            return p
    sys.exit("읽기 실패")


p = rd()
print("before", p.param_value)
m.mav.param_set_send(m.target_system, m.target_component, name.encode(), val, p.param_type)
time.sleep(1)
print("after", rd().param_value)
