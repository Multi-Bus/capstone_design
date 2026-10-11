"""csv_logger: A 가이드 §4 CSV 계약대로 10Hz(시뮬 시간) 로그를 쓴다. 한 행 = 한 시각·한 드론.

실행: python3 csv_logger.py --ids 0 1 --world room --out out.csv --duration 20 [--run-id X]

- 실제 위치/카메라 pose: get_truth.Truth. 시야 목록: visibility.visible_drones(카메라 위치·yaw).
- 추정값: 교체 가능한 소스 함수 estimate_source(drone_id) -> get_estimate 형식 dict 또는 None.
  기본 stub_estimate는 None(추정 없음) -> 추정 칸 빈칸, estimate_valid=false,
  uncertainty_valid=false, uncertainty_source=unavailable. A의 get_estimate가 오면 main()에 넘기면 된다.
- 값 없음은 빈칸. 불확실성은 0으로 채우지 않는다.
"""
import argparse, csv, json, os, sys, time
from get_truth import Truth
from visibility import visible_drones

COLUMNS = (
    "run_id,time_sim_s,drone_id,estimate_time_boot_ms,estimate_age_s,"
    "truth_x,truth_y,truth_z,estimate_x,estimate_y,estimate_z,"
    "estimate_vx,estimate_vy,estimate_vz,"
    "position_var_x,position_var_y,position_var_z,"
    "uncertainty_valid,uncertainty_source,estimate_valid,visible_ids"
).split(",")
PERIOD_S = 0.1  # 10Hz, 시뮬 시간 기준
NO_DATA_TIMEOUT_S = 10.0  # 호스트 시간 기준, 이 시간 동안 새 시뮬 시각이 없으면 종료


def stub_estimate(drone_id):
    return None


def _b(v):
    return "true" if v else "false"


def make_row(run_id, t, drone_id, truth, est, visible):
    row = dict.fromkeys(COLUMNS, "")
    row.update(run_id=run_id, time_sim_s=f"{t:.3f}", drone_id=drone_id,
               uncertainty_valid="false", uncertainty_source="unavailable", estimate_valid="false")
    if truth.get("valid"):
        row["truth_x"], row["truth_y"], row["truth_z"] = truth["position_enu_m"]
        row["visible_ids"] = json.dumps(visible)  # 관측자 truth가 없으면 판정 불가 -> 빈칸
    if est:
        row["estimate_valid"] = _b(est.get("valid") and not est.get("stale"))
        row["estimate_time_boot_ms"] = est.get("time_boot_ms", "")
        if est.get("time_sim_s") is not None:
            row["estimate_age_s"] = f"{t - est['time_sim_s']:.3f}"
        row["estimate_x"], row["estimate_y"], row["estimate_z"] = est["position_enu_m"]
        row["estimate_vx"], row["estimate_vy"], row["estimate_vz"] = est["velocity_enu_mps"]
        row["uncertainty_valid"] = _b(est.get("uncertainty_valid"))
        row["uncertainty_source"] = est.get("uncertainty_source", "unavailable")
        cov = est.get("position_cov_enu_m2")
        if est.get("uncertainty_valid") and cov is not None:
            row["position_var_x"], row["position_var_y"], row["position_var_z"] = (cov[i][i] for i in range(3))
    return row


def main(argv=None, estimate_source=stub_estimate):
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--world", default="room")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=20.0, help="시뮬 시간 초")
    ap.add_argument("--run-id", default=time.strftime("run_%Y%m%d_%H%M%S"))
    a = ap.parse_args(argv)

    truth = Truth(a.world)
    t0 = next_t = None
    last_new = time.monotonic()
    last_t = -1.0
    n = 0
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, COLUMNS)
        w.writeheader()
        while True:
            snap = {i: truth.get(i) for i in a.ids}
            ts = [s["time_sim_s"] for s in snap.values() if s.get("valid")]
            t = max(ts) if ts else None
            if t is not None and t > last_t:
                last_t, last_new = t, time.monotonic()
            elif time.monotonic() - last_new > NO_DATA_TIMEOUT_S:
                print("새 시뮬 시각 없음, 종료")
                break
            if t is None or (next_t is not None and t < next_t - 1e-9):
                time.sleep(0.005)
                continue
            if t0 is None:
                t0 = next_t = t
            if t - t0 > a.duration:
                break
            pos = {i: s["position_enu_m"] for i, s in snap.items() if s.get("valid")}
            for i in a.ids:
                s = snap[i]
                vis = visible_drones(i, s["camera_position_enu_m"], pos, s["camera_yaw_enu_rad"]) if s.get("valid") else []
                w.writerow(make_row(a.run_id, t, i, s, estimate_source(i), vis))
            n += 1
            while next_t <= t + 1e-9:  # 느려서 놓친 주기는 건너뛴다
                next_t += PERIOD_S
        f.flush()
    print(f"{n}개 시각 기록: {a.out}")


if __name__ == "__main__":
    main()
    # gz.transport 구독이 남은 채 인터프리터가 정리되면 종료 시 Segmentation fault가 난다(CSV는 이미 닫힘)
    sys.stdout.flush()
    os._exit(0)
