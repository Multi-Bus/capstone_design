"""GPS OFF 광류 정확도·드리프트 측정: 이륙 2m -> 60초 정지비행 -> A 가이드 14절 정사각형(추정 위치로 제어).

실제 위치(get_truth, 평가 전용)는 제어에 쓰지 않는다. AP LOCAL_POSITION_NED(NED)는 ENU(E,N,-D)로 바꿔 쓴다.
정렬: offset = 추정 - 실제(정지 상태 평균). 오차 = 추정 - 실제 - offset.
  정지비행 평가는 정지비행 첫 5초, 정사각형 평가는 정지비행 마지막 3초(= 정사각형 직전) 평균을 offset으로 쓴다.
사용: python scripts/flow_accuracy_check.py [--mav udpin:127.0.0.1:14550] [--out results/flow_accuracy_x.csv]
시계: 타이머(60초 정지, 2초 유지, 30초 timeout)는 Gazebo 시뮬레이션 시각 기준이다.
"""
import argparse, csv, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from pymavlink import mavutil
from get_truth import Truth

ap = argparse.ArgumentParser()
ap.add_argument("--mav", default="udpin:127.0.0.1:14550")
ap.add_argument("--model", default="iris_flow_test")
ap.add_argument("--hover-s", type=float, default=60.0)
ap.add_argument("--speed", type=float, default=0.3)
ap.add_argument("--out", default=None, help="CSV 경로(기본 results/flow_accuracy_<시각>.csv)")
a = ap.parse_args()
out_path = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results",
                                 time.strftime("flow_accuracy_%Y%m%d_%H%M%S.csv"))
CORNERS = [(2, 0), (2, 2), (0, 2), (0, 0)]  # 시작 추정 위치 기준 ENU 상대
TOL, HOLD_S, SEG_TIMEOUT_S = 0.2, 2.0, 30.0

truth = Truth()
link = mavutil.mavlink_connection(a.mav, source_system=250)
assert link.wait_heartbeat(timeout=90), "heartbeat 없음"
link.mav.request_data_stream_send(link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
print("sys", link.target_system, flush=True)

texts, rows = [], []
last = {"ekf": None}
cur = {"pos": None}


def handle(m):
    """메시지 처리. LOCAL_POSITION_NED면 True."""
    t = m.get_type()
    if t == "STATUSTEXT":
        texts.append(m.text); print("  [STATUSTEXT]", m.text, flush=True)
    elif t == "COMMAND_ACK" and m.command in (22, 400):
        print("  [ACK] cmd", m.command, "result", m.result, flush=True)
    elif t == "EKF_STATUS_REPORT":
        last["ekf"] = m
    elif t == "LOCAL_POSITION_NED":
        cur["pos"] = m
        return True
    return False


def pump(sec):
    end = time.time() + sec
    while time.time() < end:
        m = link.recv_match(blocking=True, timeout=0.3)
        if m is not None:
            handle(m)


def vel_cmd(ve, vn, vu):
    link.mav.set_position_target_local_ned_send(0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, vn, ve, -vu, 0, 0, 0, 0, 0)


def run(phase, ctrl, host_max):
    """LOCAL_POSITION_NED가 올 때마다(약 10Hz) 실제/추정을 기록하고 ctrl(est, simt)->(ve,vn,vu)|None|'done'을 실행한다.
    'done'이면 True, host_max 초 안에 못 끝내면 False."""
    t0 = time.time()
    while time.time() - t0 < host_max:
        m = link.recv_match(blocking=True, timeout=0.3)
        if m is None or not handle(m):
            continue
        d = truth.get(0, model_name=a.model)
        if not d.get("valid"):
            continue
        est = (m.y, m.x, -m.z)  # NED -> ENU (E, N, U)
        tr = d["position_enu_m"]
        r = dict(phase=phase, t_sim=d["time_sim_s"], t_host=time.time(), est=est, tr=tr, vel=(m.vy, m.vx))
        rows.append(r)
        v = ctrl(est, r["t_sim"], r)
        if v == "done":
            return True
        if v == "stop":
            return False
        if v is not None:
            vel_cmd(*v)
    return False


def clip2(ve, vn, vmax):
    s = math.hypot(ve, vn)
    return (ve, vn) if s <= vmax else (ve * vmax / s, vn * vmax / s)


def mean_off(rs):
    return np.mean([np.subtract(r["est"], r["tr"]) for r in rs], axis=0)


# ---- 준비: EKF, GUIDED, arm, 이륙 ----
t0 = time.time()
while time.time() - t0 < 150 and not (last["ekf"] and (last["ekf"].flags & 0x0B) == 0x0B):
    pump(1)
print("EKF flags:", None if last["ekf"] is None else hex(last["ekf"].flags), flush=True)
pump(10)
t0 = time.time()
while time.time() - t0 < 30 and link.flightmode != "GUIDED":
    link.set_mode("GUIDED"); pump(1.5); link.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
print("모드:", link.flightmode, flush=True)
t0 = time.time()
while time.time() - t0 < 120 and not link.motors_armed():
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
        if m is not None:
            handle(m)
            if m.get_type() == "COMMAND_ACK" and m.command == 22: res = m.result
    print("  takeoff 시도", attempt + 1, "결과", res, flush=True)
    if res == 0: break

ok, why = False, ""
alt_ref = [None]
off_h = off_s = None
corner_res = []
try:
    # ---- 이륙 완료 대기(추정 고도 1.8m 이상), 8초(sim) 안정화 ----
    if not run("takeoff", lambda e, t, r: "done" if e[2] >= 1.8 else None, 60):
        why = "이륙 고도 미도달"; raise RuntimeError(why)
    s0 = [None]

    def settle(e, t, r):
        s0[0] = s0[0] if s0[0] is not None else t
        return "done" if t - s0[0] >= 8 else (0.0, 0.0, 0.0)

    run("settle", settle, 60)
    alt_ref[0] = float(np.mean([r["est"][2] for r in rows if r["phase"] == "settle"][-30:]))  # 마지막 ~3초 평균 고도 유지

    def hold_z(e):
        return float(np.clip(0.5 * (alt_ref[0] - e[2]), -0.3, 0.3))

    # ---- ① 정지비행 ----
    h0 = [None]

    def hover(e, t, r):
        h0[0] = h0[0] if h0[0] is not None else t
        return "done" if t - h0[0] >= a.hover_s else (0.0, 0.0, hold_z(e))

    if not run("hover", hover, a.hover_s * 4 + 30):
        why = "정지비행 시간 초과"; raise RuntimeError(why)
    hv = [r for r in rows if r["phase"] == "hover"]
    off_h = mean_off([r for r in hv if r["t_sim"] - hv[0]["t_sim"] <= 5])
    off_s = mean_off([r for r in hv if hv[-1]["t_sim"] - r["t_sim"] <= 3])
    start = np.mean([r["est"] for r in hv if hv[-1]["t_sim"] - r["t_sim"] <= 3], axis=0)  # 정사각형 시작 추정 위치

    # ---- ② 정사각형 ----
    ok = True
    for k, (ce, cn) in enumerate(CORNERS, 1):
        tgt = (start[0] + ce, start[1] + cn)
        st = {"t0": None, "hold": None}

        def seg(e, t, r):
            st["t0"] = st["t0"] if st["t0"] is not None else t
            r["tgt"] = tgt
            de, dn = tgt[0] - e[0], tgt[1] - e[1]
            if math.hypot(de, dn) < TOL:
                st["hold"] = st["hold"] if st["hold"] is not None else t
                if t - st["hold"] >= HOLD_S:
                    return "done"
            else:
                st["hold"] = None
            if t - st["t0"] > SEG_TIMEOUT_S:
                return "stop"
            ve, vn = clip2(de, dn, a.speed)  # P 게인 1.0/s, 0.3m/s 제한
            return (ve, vn, hold_z(e))

        done = run(f"sq{k}", seg, SEG_TIMEOUT_S * 4 + 10)
        last_r = rows[-1]
        if not done:
            ok = False; why = f"꼭짓점 {k} {tgt[0] - start[0]:.0f},{tgt[1] - start[1]:.0f} timeout/미도달"
            break
        corner_res.append((k, (ce, cn), last_r))
        print(f"  꼭짓점 {k} 도달 simt={last_r['t_sim']:.1f}", flush=True)
    for _ in range(30):  # 정지
        vel_cmd(0, 0, 0); pump(0.1)
except RuntimeError:
    ok = False
finally:
    link.set_mode("LAND")
    t0 = time.time()
    while time.time() - t0 < 60 and (time.time() - t0 < 8 or link.motors_armed()):
        pump(1)
    print("착륙 후 armed:", bool(link.motors_armed()), flush=True)
    # ---- 저장 ----
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", newline="") as f:
        f.write(f"# offset_hover(est-truth E,N,U)={None if off_h is None else np.round(off_h, 4).tolist()} "
                f"offset_square={None if off_s is None else np.round(off_s, 4).tolist()} model={a.model} mav={a.mav}\n")
        w = csv.writer(f)
        w.writerow(["phase", "t_sim", "t_host", "est_e", "est_n", "est_u", "truth_e", "truth_n", "truth_u",
                    "err_e", "err_n", "err_u", "err_xy", "err_3d", "est_ve", "est_vn", "tgt_e", "tgt_n"])
        for r in rows:
            off = off_h if r["phase"] == "hover" else off_s if r["phase"].startswith("sq") else None
            if off is None or off_h is None:
                err = [""] * 5
            else:
                e = np.subtract(r["est"], r["tr"]) - off
                err = [*np.round(e, 4), round(math.hypot(e[0], e[1]), 4), round(float(np.linalg.norm(e)), 4)]
            tg = r.get("tgt", ("", ""))
            w.writerow([r["phase"], round(r["t_sim"], 3), round(r["t_host"], 3), *np.round(r["est"], 4), *np.round(r["tr"], 4),
                        *err, round(r["vel"][1], 3), round(r["vel"][0], 3), *tg])
    print("CSV:", os.path.abspath(out_path), "행", len(rows), flush=True)

# ---- 요약 ----
def xyz(rs, key):
    return np.array([r[key] for r in rs])


if off_h is None:
    print("결과: 실패 -", why)
    sys.exit(2)

print(f"\n== 정지비행 {a.hover_s:.0f}s (오차 = 추정-실제-첫5초 offset, offset={np.round(off_h, 3).tolist()}) ==")
t = xyz(hv, "t_sim") - hv[0]["t_sim"]
e = xyz(hv, "est") - xyz(hv, "tr") - off_h
rtf = (hv[-1]["t_sim"] - hv[0]["t_sim"]) / (hv[-1]["t_host"] - hv[0]["t_host"])
print(f"  표본 {len(hv)}개, 시뮬/실시간 배율 {rtf:.2f}")
for tt in range(0, int(a.hover_s) + 1, 10):
    i = int(np.argmin(abs(t - tt)))
    print(f"  t={tt:3d}s  오차 E={e[i,0]:+.3f} N={e[i,1]:+.3f} U={e[i,2]:+.3f}  |xy|={math.hypot(e[i,0], e[i,1]):.3f} m")
exy = np.hypot(e[:, 0], e[:, 1])
sl = [np.polyfit(t, e[:, j], 1)[0] * 60 for j in range(3)]
print(f"  최대 |xy| {exy.max():.3f} m, 마지막 |xy| {exy[-1]:.3f} m, 선형 추세(m/min) E={sl[0]:+.3f} N={sl[1]:+.3f} U={sl[2]:+.3f}")
tr_d = xyz(hv, "tr") - xyz(hv, "tr")[0]
es_d = xyz(hv, "est") - xyz(hv, "est")[0]
print(f"  실제 위치 이동(시작 대비) 최대 xy {np.hypot(tr_d[:,0], tr_d[:,1]).max():.3f} m / 추정 이동 최대 xy {np.hypot(es_d[:,0], es_d[:,1]).max():.3f} m")

sq = [r for r in rows if r["phase"].startswith("sq")]
if sq and off_s is not None:
    print(f"\n== 정사각형 (오차 = 추정-실제-정사각형 직전 offset, offset={np.round(off_s, 3).tolist()}) ==")
    for k, c, r in corner_res:
        er = np.subtract(r["est"], r["tr"]) - off_s
        print(f"  꼭짓점 {k} 목표 상대{c}  추정 상대=({r['est'][0]-start[0]:+.2f},{r['est'][1]-start[1]:+.2f})  "
              f"오차 E={er[0]:+.3f} N={er[1]:+.3f} U={er[2]:+.3f} |xy|={math.hypot(er[0], er[1]):.3f} m")
    es = xyz(sq, "est") - xyz(sq, "tr") - off_s
    exy = np.hypot(es[:, 0], es[:, 1]); e3 = np.linalg.norm(es, axis=1)
    print(f"  표본 {len(sq)}개  RMSE xy={math.sqrt(np.mean(exy**2)):.3f} m  RMSE 3D={math.sqrt(np.mean(e3**2)):.3f} m  "
          f"최대 xy={exy.max():.3f} m  최대 3D={e3.max():.3f} m")
    if len(corner_res) == len(CORNERS):
        fr = corner_res[-1][2]
        er = np.subtract(fr["est"], fr["tr"]) - off_s
        back = np.subtract(fr["tr"], [r for r in hv][-1]["tr"])
        print(f"  최종 복귀 오차(추정-실제 정렬) |xy|={math.hypot(er[0], er[1]):.3f} m, 실제 위치가 정사각형 시작점에서 떨어진 거리 xy={math.hypot(back[0], back[1]):.3f} m")
    allr = rows
    print(f"  비행 영역: 실제 x {min(r['tr'][0] for r in allr):.2f}~{max(r['tr'][0] for r in allr):.2f}, "
          f"y {min(r['tr'][1] for r in allr):.2f}~{max(r['tr'][1] for r in allr):.2f} (방 ±6m)")
fs = [x for x in texts if "ailsafe" in x or ("EKF" in x and "ariance" in x)]
print("  failsafe/EKF 이상 메시지:", fs if fs else "없음")
print("결과:", "정사각형 완료" if ok else f"실패 - {why}")
sys.exit(0 if ok else 2)
