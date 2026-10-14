"""센서 브리지: Gazebo 하향 센서 -> MAVLink (OPTICAL_FLOW, DISTANCE_SENSOR)

- /down_camera/image 연속 프레임을 cv2.phaseCorrelate로 비교해 광류(rad/s)를 만든다.
  축 규약(ArduPilot AP_OpticalFlow_SITL.cpp 기준): flow_rate_x = dx/f/dt, flow_rate_y = dy/f/dt
  (dx는 영상 오른쪽, dy는 영상 아래가 양수. 기체가 앞으로 가면 영상 내용이 아래로 이동)
- /down_range/scan 거리 -> DISTANCE_SENSOR (orientation 25 = 아래)
실행: ~/venv-ardupilot/bin/python scripts/sensor_bridge.py --mav tcp:127.0.0.1:5762
"""
import argparse, math, os, signal, threading, time
from capture import Recorder
os.environ["MAVLINK20"] = "1"  # OPTICAL_FLOW의 flow_rate_x/y는 MAVLink 2 확장 필드라서 필요
import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--mav", default="tcp:127.0.0.1:5762")
ap.add_argument("--cam", default="/down_camera/image")
ap.add_argument("--range", default="/down_range/scan")
ap.add_argument("--fov", type=float, default=1.0472, help="down_camera 수평 FOV(rad), model.sdf와 같게")
ap.add_argument("--min-resp", type=float, default=0.3, help="이 신뢰도 미만이면 quality=0")
ap.add_argument("--min-m", type=float, default=0.1)
ap.add_argument("--max-m", type=float, default=5.0)
ap.add_argument("--record", help="Optional diagnostic JSONL; images are not stored")
ap.add_argument("--drone-id", type=int, default=0)
a = ap.parse_args()
recorder = Recorder(a.record) if a.record else None

link = mavutil.mavlink_connection(a.mav, source_system=200, source_component=191)
lock = threading.Lock()
st = {"prev": None, "t_prev": None, "range_m": None, "win": None,
      "n_flow": 0, "n_range": 0, "last": (0.0, 0.0, 0)}

def on_image(msg):
    received = time.monotonic()
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    g = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).mean(axis=2).astype(np.float32)
    prev, t_prev = st["prev"], st["t_prev"]
    st["prev"], st["t_prev"] = g, t
    if prev is None:
        if recorder:
            recorder.emit('camera_skip',a.drone_id,dict(time_sim_s=t,reason='first_frame'),received)
        return
    dt = t - t_prev
    if not (0.0 < dt < 0.2):
        if recorder:
            recorder.emit('camera_skip',a.drone_id,dict(time_sim_s=t,dt_sim_s=dt,reason='frame_interval'),received)
        return
    if st["win"] is None:
        st["win"] = cv2.createHanningWindow((msg.width, msg.height), cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(prev, g, st["win"])
    f = (msg.width / 2) / math.tan(a.fov / 2)
    fx, fy = dx / f / dt, dy / f / dt
    q = int(min(1.0, max(0.0, resp)) * 255) if resp >= a.min_resp else 0
    rng = st["range_m"] if st["range_m"] is not None else -1.0
    with lock:
        sent = time.monotonic()
        link.mav.optical_flow_send(int(sent * 1e6), 0, 0, 0, 0.0, 0.0, q, rng, fx, fy)
    if recorder:
        recorder.emit('flow',a.drone_id,dict(time_sim_s=t,dt_sim_s=dt,
            flow_x_rad_s=fx,flow_y_rad_s=fy,response=resp,quality=q,range_m=rng,
            sent_host_s=sent,processing_host_s=sent-received),received)
    st["n_flow"] += 1
    st["last"] = (fx, fy, q)

def on_scan(msg):
    received = time.monotonic()
    r = msg.ranges[0] if len(msg.ranges) else float("inf")
    ok = math.isfinite(r) and a.min_m <= r <= a.max_m
    if ok:
        st["range_m"] = r
    cm = int(round(r * 100)) if ok else int(a.max_m * 100) + 1
    with lock:
        link.mav.distance_sensor_send(int(time.monotonic() * 1000) & 0xFFFFFFFF,
                                      int(a.min_m * 100), int(a.max_m * 100), cm, 0, 0, 25, 255)
    st["n_range"] += 1
    if recorder:
        recorder.emit('range',a.drone_id,dict(time_sim_s=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
            range_m=r if math.isfinite(r) else None,valid=ok),received)

node = Node()
assert node.subscribe(Image, a.cam, on_image), "camera subscribe failed"
assert node.subscribe(LaserScan, a.range, on_scan), "range subscribe failed"
print("bridge 시작:", a.mav, a.cam, a.range, flush=True)

def terminate(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, terminate)

try:
    n = 0
    while True:
        with lock:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        n += 1
        if n % 2 == 0:
            fx, fy, q = st["last"]
            print(f"  flow n={st['n_flow']} range n={st['n_range']} 최근 fx={fx:+.3f} fy={fy:+.3f} rad/s q={q} "
                  f"거리={st['range_m']}", flush=True)
        # udpin은 상대(MAVProxy) 주소를 수신한 패킷에서 배우므로, 읽지 않으면 송신이 전부 버려진다(TCP는 무관)
        end = time.time() + 1
        while time.time() < end:
            link.recv_match(blocking=True, timeout=0.2)
except KeyboardInterrupt:
    pass
finally:
    if recorder:
        recorder.close()
    link.close()
