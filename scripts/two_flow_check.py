"""두 드론 GPS-off 광류 점검 (A 가이드 연결 경로: SITL -> MAVProxy --out -> 제어 14550/14560, 센서 브리지 15550/15560).

드론 0 = SITL I0 (sysid 1, udp 14550), 드론 1 = SITL I1 (sysid 2, udp 14560).
제어는 추정(LOCAL_POSITION_NED)만 쓴다. 실제 위치(get_truth)는 기록·판정 전용.
시간: 정착/정지/목표 보류 길이는 Gazebo 시뮬 시각(get_truth의 time_sim_s), 단계·이륙·착륙 제한시간은 호스트 시각.
단계: 정착 5s -> A 둘 다 정지 20s -> B 드론0만 +E 1.5m (드론1 정지) -> C 드론1만 +E 1.5m (드론0 정지) -> LAND.
사용: GZ_PARTITION=agentM python scripts/two_flow_check.py   (run_two_flow_check.sh 가 호출)
"""
import math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pymavlink import mavutil
from get_truth import Truth

LINKS = {0: ("udpin:127.0.0.1:14550", 1), 1: ("udpin:127.0.0.1:14560", 2)}  # 드론 id: (주소, 기대 sysid)
ALT = 2.0                         # 목표 고도 (추정 U, m)
SETTLE_S, HOVER_S = 5.0, 20.0     # sim s
MOVE_M, SPEED, KP = 1.5, 0.3, 1.0  # 이동량, 최대 수평속도(m/s), P 게인(1/s)
TOL, HOLD_S = 0.1, 3.0            # 목표 도달: 0.1m 이내가 3s(sim) 유지
DT = 0.1                          # 제어 주기 (호스트 s)
PHASE_HOST_S = 120                # 단계 호스트 시간 상한
HOVER_LIMIT_M = 0.3               # A: 둘 다 정지 판정 기준 (본 스크립트가 정한 값, 명세에 없음)
MOVE_TOL = 0.3                    # 이동 판정: 1.5 +- 0.3 m

truth = Truth(world="room")
rows = []     # dict(phase, id, sim, est(E,N,U), tr(E,N,U), cmd(수평 m/s), mode)
checks = []


class Drone:
    def __init__(self, i, addr, sysid):
        self.i, self.sysid = i, sysid
        self.link = mavutil.mavlink_connection(addr, source_system=250)
        self.est = None      # 추정 (E, N, U)
        self.ekf = 0         # EKF_STATUS_REPORT flags
        self.ack = None      # NAV_TAKEOFF 응답 결과
        self.wrong_src = 0   # 기대 sysid와 다른 송신자 메시지 수 (ID 점검)
        self.texts = []      # STATUSTEXT 모음
        self.failsafe = []

    def handle(self, m):
        t = m.get_type()
        if t == "LOCAL_POSITION_NED":
            if m.get_srcSystem() != self.sysid:
                self.wrong_src += 1
                return
            self.est = (m.y, m.x, -m.z)  # NED -> ENU
        elif t == "EKF_STATUS_REPORT":
            self.ekf = m.flags
        elif t == "STATUSTEXT":
            print(f"  [d{self.i}] {m.text}", flush=True)
            self.texts.append(m.text)
            if "ailsafe" in m.text:
                self.failsafe.append(m.text)
        elif t == "COMMAND_ACK" and m.command == 22:
            self.ack = m.result


drones = {i: Drone(i, addr, sysid) for i, (addr, sysid) in LINKS.items()}


def pump(sec):
    end = time.time() + sec
    for d in drones.values():  # GCS 하트비트 (MAVProxy 경유 명령 수신 유지)
        d.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    while time.time() < end:
        for d in drones.values():
            m = d.link.recv_match(blocking=True, timeout=0.01)
            if m is not None:
                d.handle(m)


def sim_t():
    return truth.get(0).get("time_sim_s", 0.0)


def command(d, ve, vn, vu):
    d.link.mav.set_position_target_local_ned_send(0, d.link.target_system, d.link.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, vn, ve, -vu, 0, 0, 0, 0, 0)


def steer(d, tgt):
    """추정 위치 P 제어 (수평 최대 SPEED) + 고도 ALT 유지."""
    ve, vn = KP * (tgt[0] - d.est[0]), KP * (tgt[1] - d.est[1])
    s = math.hypot(ve, vn)
    if s > SPEED:
        ve, vn = ve * SPEED / s, vn * SPEED / s
    return ve, vn, max(-0.3, min(0.3, 0.5 * (ALT - d.est[2])))


def run_phase(name, offsets, sim_s=None, mover=None):
    """단계 시작 추정 위치를 기준으로 목표 = 기준 + offsets[i]. sim_s 있으면 그 시간 동안, mover 있으면 그 드론이 목표 도달·보류 후 종료."""
    base = {i: d.est[:2] for i, d in drones.items()}
    tgt = {i: (base[i][0] + offsets[i][0], base[i][1] + offsets[i][1]) for i in drones}
    s0, reach, host0 = sim_t(), None, time.time()
    while True:
        if time.time() - host0 > PHASE_HOST_S:
            raise RuntimeError(f"단계 {name} 호스트 시간 초과")
        pump(DT)
        for i, d in drones.items():
            ve, vn, vu = steer(d, tgt[i])
            command(d, ve, vn, vu)
            tr = truth.get(i)
            if tr.get("valid"):
                rows.append(dict(phase=name, id=i, sim=tr["time_sim_s"], est=d.est, tr=tr["position_enu_m"],
                                 cmd=math.hypot(ve, vn), mode=d.link.flightmode))
        if mover is None:
            if sim_t() - s0 >= sim_s:
                return
        else:
            e = math.hypot(tgt[mover][0] - drones[mover].est[0], tgt[mover][1] - drones[mover].est[1])
            if e < TOL:
                reach = sim_t() if reach is None else reach
                if sim_t() - reach >= HOLD_S:
                    return
            else:
                reach = None


def stats(name, i):
    rs = [r for r in rows if r["phase"] == name and r["id"] == i]
    a, b = rs[0], rs[-1]
    tr = lambda r: r["tr"]
    ea, eb, ta, tb = a["est"], b["est"], tr(a), tr(b)
    return dict(
        n=len(rs), dur=b["sim"] - a["sim"],
        tnet=math.hypot(tb[0] - ta[0], tb[1] - ta[1]),
        tmax=max(math.hypot(tr(r)[0] - ta[0], tr(r)[1] - ta[1]) for r in rs),
        enet=math.hypot(eb[0] - ea[0], eb[1] - ea[1]),
        evt=math.hypot((eb[0] - ea[0]) - (tb[0] - ta[0]), (eb[1] - ea[1]) - (tb[1] - ta[1])),
        cmax=max(r["cmd"] for r in rs), modes=sorted({r["mode"] for r in rs}))


def check(name, ok, detail):
    checks.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)


def flight():
    for i, d in drones.items():
        assert d.link.wait_heartbeat(timeout=90), f"드론 {i} heartbeat 없음"
        d.link.mav.request_data_stream_send(d.link.target_system, d.link.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        print(f"드론 {i}: heartbeat sysid={d.link.target_system} (기대 {d.sysid})", flush=True)
    # 광류 융합은 EKF_STATUS_REPORT로 확인한다(0x08 = 상대 수평위치 융합, 광류 기반).
    # "fusing optical flow" STATUSTEXT는 EKF 초기화 때 한 번만 나가서 접속 전에 끝나므로 실시간으로는 못 잡는다.
    # 대신 shell 스크립트가 MAVProxy tlog에서 그 문장을 찾아 보여 준다.
    t0 = time.time()
    while time.time() - t0 < 150 and not all((d.ekf & 0x0B) == 0x0B and d.est for d in drones.values()):
        pump(1)
    print("EKF flags:", {i: hex(d.ekf) for i, d in drones.items()}, flush=True)
    t0 = time.time()
    while time.time() - t0 < 30 and not all(d.link.flightmode == "GUIDED" for d in drones.values()):
        for d in drones.values():
            d.link.set_mode("GUIDED")
        pump(1.5)
    print("모드:", {i: d.link.flightmode for i, d in drones.items()}, flush=True)
    t0 = time.time()
    while time.time() - t0 < 150 and not all(d.link.motors_armed() for d in drones.values()):
        for d in drones.values():
            if not d.link.motors_armed():
                d.link.arducopter_arm()
        pump(3)
    print("armed:", {i: bool(d.link.motors_armed()) for i, d in drones.items()}, flush=True)
    if not all(d.link.motors_armed() for d in drones.values()):
        raise RuntimeError("arm 실패")
    for _ in range(5):
        for d in drones.values():
            d.ack = None
        for d in drones.values():
            d.link.mav.command_long_send(d.link.target_system, d.link.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, ALT)
        pump(3)
        if all(d.ack == 0 for d in drones.values()):
            break
    print("takeoff ACK:", {i: d.ack for i, d in drones.items()}, flush=True)
    t0 = time.time()
    while time.time() - t0 < 60 and not all(d.est[2] >= ALT - 0.2 for d in drones.values()):
        pump(DT)
    print("추정 고도:", {i: round(d.est[2], 2) for i, d in drones.items()}, flush=True)

    run_phase("settle", {0: (0, 0), 1: (0, 0)}, sim_s=SETTLE_S)
    run_phase("A_hover", {0: (0, 0), 1: (0, 0)}, sim_s=HOVER_S)
    run_phase("B_d0_move", {0: (MOVE_M, 0), 1: (0, 0)}, mover=0)
    run_phase("C_d1_move", {0: (0, 0), 1: (MOVE_M, 0)}, mover=1)


def land():
    for d in drones.values():
        d.link.set_mode("LAND")
    t0 = time.time()
    while time.time() - t0 < 90 and any(d.link.motors_armed() for d in drones.values()):
        pump(0.5)
    print("착륙 후 armed:", {i: bool(d.link.motors_armed()) for i, d in drones.items()}, flush=True)
    return not any(d.link.motors_armed() for d in drones.values())


def main():
    landed = False
    err = None
    try:
        flight()
    except Exception as e:  # 실패해도 착륙 후 판정한다
        err = repr(e)
        print("중단:", err, flush=True)
    finally:
        landed = land()

    print("\n== 단계별 결과 (변위는 단계 시작 대비, truth=get_truth 평가 전용)")
    print(f"{'단계':11}{'드론':>4}{'샘플':>6}{'시뮬s':>7}{'truth끝':>9}{'truth최대':>10}{'est끝':>8}{'|est-truth|':>12}{'cmd최대':>9}  모드")
    S = {}
    for name in ["settle", "A_hover", "B_d0_move", "C_d1_move"]:
        for i in drones:
            if not any(r["phase"] == name and r["id"] == i for r in rows):
                continue
            s = S[(name, i)] = stats(name, i)
            print(f"{name:11}{i:>4}{s['n']:>6}{s['dur']:>7.1f}{s['tnet']:>9.3f}{s['tmax']:>10.3f}{s['enet']:>8.3f}"
                  f"{s['evt']:>12.3f}{s['cmax']:>9.3f}  {','.join(s['modes'])}")

    print("\n== 판정")
    if err:
        check("비행 완주", False, err)
    check("heartbeat sysid 1/2 (ID)", all(d.link.target_system == d.sysid for d in drones.values()),
          {i: d.link.target_system for i, d in drones.items()})
    check("추정 메시지 송신자 ID 일치 (ID)", all(d.wrong_src == 0 for d in drones.values()),
          {i: d.wrong_src for i, d in drones.items()})
    check("EKF 상대위치 융합(0x08, 광류) 두 드론", all(d.ekf & 0x08 for d in drones.values()),
          {i: hex(d.ekf) for i, d in drones.items()})
    if ("B_d0_move", 0) in S and ("C_d1_move", 1) in S:
        b0, b1 = S[("B_d0_move", 0)], S[("B_d0_move", 1)]
        c0, c1 = S[("C_d1_move", 0)], S[("C_d1_move", 1)]
        check("B: 드론1 truth 최대 이동 < 0.2m (드론1 정지)", b1["tmax"] < 0.2, f"{b1['tmax']:.3f} m")
        check("B: 드론0 truth 이동 1.5+-0.3m", abs(b0["tnet"] - MOVE_M) <= MOVE_TOL, f"{b0['tnet']:.3f} m")
        check("C: 드론0 truth 최대 이동 < 0.2m (드론0 정지)", c0["tmax"] < 0.2, f"{c0['tmax']:.3f} m")
        check("C: 드론1 truth 이동 1.5+-0.3m", abs(c1["tnet"] - MOVE_M) <= MOVE_TOL, f"{c1['tnet']:.3f} m")
        check("B/C: 움직인 드론의 추정-실제 변위 차 < 0.3m (ID/추정 일치)",
              b0["evt"] < 0.3 and c1["evt"] < 0.3, f"B {b0['evt']:.3f} C {c1['evt']:.3f} m")
    if ("A_hover", 0) in S:
        a = [S[("A_hover", i)]["tmax"] for i in drones]
        check("A: 둘 다 정지 (truth 최대 이동 < 0.3m, 본 스크립트 기준)", all(x < HOVER_LIMIT_M for x in a),
              f"드론0 {a[0]:.3f} 드론1 {a[1]:.3f} m")
    modes = {(r["phase"], r["mode"]) for r in rows}
    check("모드 GUIDED 유지 (정착~C)", all(m == "GUIDED" for _, m in modes), sorted(modes))
    fs = {i: d.failsafe for i, d in drones.items()}
    check("failsafe 없음", not any(fs.values()), fs)
    check("두 드론 착륙 후 disarm", landed, {i: bool(d.link.motors_armed()) for i, d in drones.items()})
    print("결과:", "PASS" if all(checks) and not err else "FAIL")
    sys.exit(0 if all(checks) and not err else 2)


if __name__ == "__main__":
    main()
