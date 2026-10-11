"""2대 스침 시험: 평행한 두 차선에서 서로 반대 방향으로 이동해 스쳐 지나간다(GPS ON, 회피 아님).

첫 번째 id는 +x(동쪽), 두 번째 id는 -x(서쪽)로 간다. 스폰은 run_pass_check.sh 가 한다
(첫째 (-4,-0.75) yaw 0, 둘째 (4,+0.75) yaw pi). 흐름: EKF/GUIDED/arm -> 2m 이륙 -> 속도 명령으로 DIST m 이동 ->
정지 -> LAND. 이동 종료는 시간이 아니라 LOCAL_POSITION_NED 이동량(시작 대비)으로 판정한다.
사용: python pass_check.py [ids ...(기본 0 1)] [--speed 0.5] [--dist 8] [--alt 2]
SITL 은 TCP 5760+10*id 로 연결한다. 로거 시작 신호로 "LOG_START" 한 줄을 출력한다. 성공하면 종료코드 0.
"""
import argparse, sys, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("ids", type=int, nargs="*", default=[0, 1])
ap.add_argument("--speed", type=float, default=0.5, help="m/s (시험 코드 제한 0.5)")
ap.add_argument("--dist", type=float, default=8.0, help="각자 이동 거리 m")
ap.add_argument("--alt", type=float, default=2.0)
a = ap.parse_args()
assert len(a.ids) == 2 and a.speed <= 0.5


def log(*s):
    print(f"[{time.strftime('%H:%M:%S')}]", *s, flush=True)


class Drone:
    def __init__(self, i, direction):
        self.i, self.dir, self.pos, self.start, self.texts = i, direction, None, None, []
        t0 = time.time()
        while True:
            try:
                self.l = mavutil.mavlink_connection(f"tcp:127.0.0.1:{5760 + 10 * i}", source_system=250)
                break
            except OSError:
                if time.time() - t0 > 60:
                    raise
                time.sleep(1)
        assert self.l.wait_heartbeat(timeout=90), f"드론 {i} heartbeat 없음"
        self.l.mav.request_data_stream_send(self.l.target_system, self.l.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        log(f"드론 {i}: sysid {self.l.target_system}")

    def poll(self):
        while (m := self.l.recv_match(blocking=False)) is not None:
            t = m.get_type()
            if t == "STATUSTEXT":
                self.texts.append(m.text); log(f"  [{self.i}] {m.text}")
            elif t == "LOCAL_POSITION_NED":
                self.pos = (m.x, m.y, m.z)  # N, E, D (EKF 원점 기준)
            elif t == "COMMAND_ACK" and m.command == 22:
                self.takeoff_res = m.result

    def vel(self, vn, ve, vd):
        self.l.mav.set_position_target_local_ned_send(0, self.l.target_system, self.l.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)


def clip(v, m):
    return max(-m, min(m, v))


def wait(ds, cond, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        for d in ds:
            d.poll()
        if cond():
            return True
        time.sleep(0.05)
    if what:
        log(f"시간 초과: {what}")
    return False


ds = [Drone(a.ids[0], +1), Drone(a.ids[1], -1)]
fail = None
try:
    if not wait(ds, lambda: all(any("is using GPS" in t for t in d.texts) for d in ds), 240, "EKF GPS 준비"):
        sys.exit(2)
    for d in ds:
        t0 = time.time()
        while time.time() - t0 < 30 and d.l.flightmode != "GUIDED":
            d.l.set_mode("GUIDED"); wait([d], lambda: False, 1.5, "")
            d.l.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        log(f"드론 {d.i} 모드 {d.l.flightmode}")
        t0 = time.time()
        while time.time() - t0 < 150 and not d.l.motors_armed():
            d.l.arducopter_arm(); wait([d], lambda: False, 3, "")
        log(f"드론 {d.i} armed {bool(d.l.motors_armed())}")
        if not d.l.motors_armed():
            sys.exit(2)
    print("LOG_START", flush=True)
    for d in ds:
        d.takeoff_res = None
        for _ in range(5):
            d.l.mav.command_long_send(d.l.target_system, d.l.target_component,
                                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, a.alt)
            wait([d], lambda: d.takeoff_res is not None, 3, "takeoff ACK")
            if d.takeoff_res == 0:
                break
            d.takeoff_res = None
        log(f"드론 {d.i} takeoff ACK {d.takeoff_res}")
    if not wait(ds, lambda: all(d.pos and -d.pos[2] >= 0.9 * a.alt for d in ds), 90, "이륙 고도"):
        sys.exit(2)
    wait(ds, lambda: False, 4, "")  # 안정화(호스트 4초)
    for d in ds:
        d.start = d.pos
        log(f"드론 {d.i} 이륙 완료 z={-d.pos[2]:.2f} start(N,E)=({d.pos[0]:.2f},{d.pos[1]:.2f})")

    # ---- 반대 방향 이동 ----
    t0 = last_print = time.time()
    prog = lambda d: d.dir * (d.pos[1] - d.start[1])  # 가야 할 방향으로 간 거리
    while time.time() - t0 < 150 and not all(prog(d) >= a.dist for d in ds):
        for d in ds:
            d.poll()
            ve = d.dir * a.speed if prog(d) < a.dist else 0.0
            vn = clip(-0.5 * (d.pos[0] - d.start[0]), 0.2)  # 차선 유지
            vd = -clip(0.5 * (a.alt + d.pos[2]), 0.3)  # 고도 유지 (NED: d=-고도)
            d.vel(vn, ve, vd)
        if time.time() - last_print > 2:
            last_print = time.time()
            log("  이동량 " + "  ".join(f"드론 {d.i}: {prog(d):.2f}m" for d in ds))
        time.sleep(0.1)
    for _ in range(30):  # 정지 3초
        for d in ds:
            d.poll(); d.vel(0, 0, 0)
        time.sleep(0.1)
    ok = all(prog(d) >= a.dist for d in ds)
    log("이동 " + ("완료 " if ok else "실패 ") + "  ".join(f"드론 {d.i}: {prog(d):.2f}m" for d in ds))
    fail = None if ok else "이동 거리 미달"
finally:
    for d in ds:
        d.l.set_mode("LAND")
    wait(ds, lambda: False, 8, "")  # 착륙 시작 대기
    wait(ds, lambda: not any(d.l.motors_armed() for d in ds), 60, "착륙/disarm")
    for d in ds:
        d.l.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    log("착륙 후 armed:", {d.i: bool(d.l.motors_armed()) for d in ds})
print("DONE" if fail is None else f"FAIL: {fail}", flush=True)
sys.exit(0 if fail is None else 2)
