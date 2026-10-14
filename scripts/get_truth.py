"""get_truth: Gazebo 실제 위치(ENU). 시험: python3 get_truth.py [모델이름]"""
import math, sys, threading, time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

# 본체 기준 앞/왼/위(m). docs/front_camera_spec.md와 같은 값으로 유지한다.
CAMERA_OFFSET = (0.1, 0.0, 0.05)
STALE_S = 0.5  # 마지막 수신 후 이 시간(호스트 시계)을 넘으면 valid=False


class Truth:
    def __init__(self, world="room", model_fmt="drone_{}", recorder=None, ids=(0,1)):
        self._recorder = recorder
        self._ids = {model_fmt.format(i):i for i in ids}
        self._fmt = model_fmt
        self._poses = {}  # 모델 이름 -> (시뮬 시각, x, y, z, yaw, 수신 호스트 시각)
        self._lock = threading.Lock()
        self._node = Node()
        if not self._node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self._cb):
            raise RuntimeError("dynamic_pose/info 구독 실패")

    def _cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        now = time.monotonic()
        with self._lock:
            for p in msg.pose:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self._poses[p.name] = (t, p.position.x, p.position.y, p.position.z, yaw, now)
                if self._recorder and p.name in self._ids:
                    self._recorder.emit('truth',self._ids[p.name],dict(time_sim_s=t,
                        position=[p.position.x,p.position.y,p.position.z],
                        quaternion_wxyz=[q.w,q.x,q.y,q.z],yaw_enu_rad=yaw),now)

    def get(self, drone_id, model_name=None):
        name = model_name or self._fmt.format(drone_id)
        with self._lock:
            rec = self._poses.get(name)
        if rec is None:
            return {"valid": False}
        t, x, y, z, yaw, seen = rec
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy, oz = CAMERA_OFFSET
        # ponytail: 카메라 위치는 yaw만 반영(roll/pitch 무시). 기울기가 커지면 전체 회전으로 바꾼다.
        return {
            "time_sim_s": t,
            "position_enu_m": [x, y, z],
            "yaw_enu_rad": yaw,
            "camera_position_enu_m": [x + c * ox - s * oy, y + s * ox + c * oy, z + oz],
            "camera_yaw_enu_rad": yaw,
            "valid": time.monotonic() - seen <= STALE_S,
        }


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "iris_with_ardupilot"
    tr = Truth()
    while True:
        print(tr.get(0, model_name=name))
        time.sleep(0.5)
