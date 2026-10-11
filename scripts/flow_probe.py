"""광류 가능성 시험: down_camera를 알려진 거리만큼 옮기고, 영상에서 측정한 픽셀 이동량을 이론값과 비교한다.
실행(Gazebo에 down_camera가 있어야 함): ~/venv-ardupilot/bin/python scripts/flow_probe.py
"""
import math, subprocess, sys, time
import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

NAME, Z = "down_camera", 2.0
START = (1.0, 1.0)
STEP = 0.05                      # 한 번에 옮기는 거리(m)
FOV = 1.0472                     # model.sdf의 horizontal_fov
W = 320
F_PX = (W / 2) / math.tan(FOV / 2)   # 초점거리(픽셀)
EXPECT = STEP * F_PX / Z             # 5cm 이동 시 이론 픽셀 이동량

frames = {}
def cb(msg):
    frames["m"] = msg
node = Node()
assert node.subscribe(Image, "/down_camera/image", cb), "subscribe failed"

def set_pose(x, y):
    req = f'name: "{NAME}" position {{x: {x} y: {y} z: {Z}}}'
    out = subprocess.run(["gz", "service", "-s", "/world/room/set_pose", "--reqtype", "gz.msgs.Pose",
                          "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                         capture_output=True, text=True).stdout
    assert "true" in out, f"set_pose 실패: {out}"

def grab():
    frames.pop("m", None)
    t0 = time.time()
    while "m" not in frames and time.time() - t0 < 5:
        time.sleep(0.02)
    m = frames["m"]
    g = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).mean(axis=2)
    return g.astype(np.float32), m.header.stamp.sec + m.header.stamp.nsec * 1e-9

def shift(a, b):
    win = cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(a, b, win)
    return dx, dy, resp

print(f"초점거리 {F_PX:.1f}px, {STEP*100:.0f}cm 이동의 이론 이동량 {EXPECT:.2f}px")
set_pose(*START); time.sleep(1.0)

print("\n[정지 상태: 연속 프레임의 가짜 이동량 (0이어야 함)]")
prev, _ = grab()
for _ in range(3):
    time.sleep(0.2)
    cur, _ = grab()
    dx, dy, r = shift(prev, cur)
    print(f"  dx={dx:+.3f} dy={dy:+.3f} 신뢰도={r:.2f}")
    prev = cur

for axis in ("x", "y"):
    print(f"\n[세계 +{axis} 방향으로 {STEP*100:.0f}cm씩 이동]")
    x, y = START
    set_pose(x, y); time.sleep(0.6)
    prev, t_prev = grab()
    for i in range(5):
        if axis == "x": x += STEP
        else: y += STEP
        set_pose(x, y); time.sleep(0.6)
        cur, t_cur = grab()
        dx, dy, r = shift(prev, cur)
        print(f"  {i+1}: dx={dx:+.2f} dy={dy:+.2f} (이론 {EXPECT:.2f}) 신뢰도={r:.2f} 시뮬시간차={t_cur - t_prev:.2f}s")
        prev, t_prev = cur, t_cur

set_pose(*START)
print("\n끝")
