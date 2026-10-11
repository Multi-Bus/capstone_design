"""time_sync_check: Gazebo 시뮬 시각과 ArduPilot SITL time_boot_ms 의 관계를 잰다 (drone 5, 지상 대기, 비행 없음).

실행 (source ~/drone_project/env.sh; export GZ_PARTITION=agentT; spawn_drones.py 5 완료 후):
  python3 scripts/time_sync_check.py [--collect-s 60] [--pause-s 15] [--after-s 10] [--restart-collect-s 30]

측정 쌍: 같은 호스트 순간에 받은 ATTITUDE.time_boot_ms 와 dynamic_pose/info 의 최신 시뮬 스탬프.
  offset = sim_s - time_boot_ms/1000
  drift  = offset 의 호스트 시간에 대한 선형 기울기 (s/s)
  gz_age = 수신 시점 - 그 포즈를 받은 호스트 시점 (시뮬 시각이 이만큼 낡았을 수 있다)
단계: 1 수집 / 2 Gazebo 일시정지·재개 / 3 SITL 만 재시작 (Gazebo 는 계속 돈다).
샘플 원본: /tmp/agentT/time_sync_samples.csv
"""
import argparse, csv, os, signal, statistics, subprocess, sys, time

from pymavlink import mavutil
from get_truth import Truth

DRONE = 5
WORLD = "room"
SITL_PORT = 5810  # 5760 + 10*5 (sim_vehicle -I5)
SIM_VEHICLE = os.path.expanduser("~/ardupilot/Tools/autotest/sim_vehicle.py")
RUN_DIR = os.path.expanduser("~/drone_project/runs/agentT_5")
FRAME_PARM = os.path.expanduser("~/drone_project/config/frame.parm")
SITL_ARGS = ["-v", "ArduCopter", "-f", "gazebo-iris", "--model", "JSON", "-I5", "--sysid", "6",
             "--no-mavproxy", "--no-rebuild", "-w", f"--add-param-file={FRAME_PARM}"]
OUT_DIR = "/tmp/agentT"
PGID_FILE = os.path.join(OUT_DIR, "sitl.pgid")


def gz_rec(truth):
    # Truth.get() 은 수신 호스트 시각을 돌려주지 않으므로 내부 캐시를 직접 읽는다 (지연 측정용).
    with truth._lock:
        return truth._poses.get(f"drone_{DRONE}")


def gz_control(pause):
    r = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/control", "--reqtype", "gz.msgs.WorldControl",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                        "--req", f"pause: {'true' if pause else 'false'}"], capture_output=True, text=True)
    return f"rc={r.returncode} {r.stdout.strip()}"


def group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


class Sitl:
    def __init__(self):
        self.proc = None
        self.log = open(os.path.join(OUT_DIR, "sitl.log"), "a")

    def start(self):
        self.proc = subprocess.Popen([sys.executable, SIM_VEHICLE, *SITL_ARGS], cwd=RUN_DIR,
                                     stdin=subprocess.DEVNULL, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        with open(PGID_FILE, "a") as f:
            f.write(f"{self.proc.pid}\n")
        return time.monotonic()

    def stop(self):
        pg = self.proc.pid
        try:
            os.killpg(pg, signal.SIGTERM)
        except ProcessLookupError:
            pass
        t_end = time.monotonic() + 3
        while time.monotonic() < t_end:
            self.proc.poll()  # 리더 좀비 수거
            if not group_alive(pg):
                break
            time.sleep(0.2)
        if group_alive(pg):  # SIGTERM 을 무시한 경우
            os.killpg(pg, signal.SIGKILL)
            time.sleep(1)
        self.proc.wait()


def connect(timeout=120.0):
    t_end = time.monotonic() + timeout
    while True:
        try:
            m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{SITL_PORT}", source_system=255)
            break
        except OSError:
            if time.monotonic() > t_end:
                raise
            time.sleep(1)
    if m.wait_heartbeat(timeout=max(1.0, t_end - time.monotonic())) is None:
        raise RuntimeError("heartbeat 없음")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    return m


class Recorder:
    def __init__(self, truth, mav):
        self.truth, self.mav = truth, mav
        self.att, self.hb, self.gz_stamps = [], [], []

    def run(self, seconds, phase):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            msg = self.mav.recv_match(type=["ATTITUDE", "HEARTBEAT"], blocking=True, timeout=0.05)
            now = time.monotonic()
            rec = gz_rec(self.truth)
            if rec is not None:
                self.gz_stamps.append((now, rec[0]))
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT":
                self.hb.append(now)
            elif rec is not None:
                self.att.append({"phase": phase, "host": now, "boot_s": msg.time_boot_ms / 1e3,
                                 "gz": rec[0], "gz_age": now - rec[5]})


def fit_slope(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 0 else float("nan")


def summarize(label, rows):
    if len(rows) < 3:
        print(f"{label}: 샘플 {len(rows)}개 (부족)")
        return None
    off = [r["gz"] - r["boot_s"] for r in rows]
    host = [r["host"] for r in rows]
    gz = [r["gz"] for r in rows]
    boot = [r["boot_s"] for r in rows]
    age = [r["gz_age"] for r in rows]
    print(f"{label}: n={len(rows)} offset mean={statistics.fmean(off):.4f} min={min(off):.4f} "
          f"max={max(off):.4f} std={statistics.pstdev(off):.4f} s | drift={fit_slope(host, off):+.6f} s/s | "
          f"boot/gz slope={fit_slope(gz, boot):.6f} | gz/host slope={fit_slope(host, gz):.4f} | "
          f"gz_age mean={statistics.fmean(age) * 1e3:.1f} max={max(age) * 1e3:.1f} ms")
    return statistics.fmean(off)


def write_csv(rows):
    with open(os.path.join(OUT_DIR, "time_sync_samples.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase", "host_s", "boot_ms", "gz_sim_s", "gz_age_ms", "offset_s"])
        for r in rows:
            w.writerow([r["phase"], f"{r['host']:.3f}", f"{r['boot_s'] * 1e3:.0f}", f"{r['gz']:.3f}",
                        f"{r['gz_age'] * 1e3:.1f}", f"{r['gz'] - r['boot_s']:.4f}"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--collect-s", type=float, default=60.0)
    ap.add_argument("--pause-s", type=float, default=15.0)
    ap.add_argument("--after-s", type=float, default=10.0)
    ap.add_argument("--restart-collect-s", type=float, default=30.0)
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    truth = Truth(WORLD)
    sitl = Sitl()
    rec = None
    try:
        sitl.start()
        t_end = time.monotonic() + 30
        while gz_rec(truth) is None:
            if time.monotonic() > t_end:
                raise RuntimeError("drone_5 포즈 없음 (spawn_drones.py 5 확인)")
            time.sleep(0.2)
        mav = connect()
        rec = Recorder(truth, mav)

        print("== 1 수집: 시작 offset")
        rec.run(a.collect_s, "collect")
        for r in rec.att[:3]:
            print(f"  first sample: boot_ms={r['boot_s'] * 1e3:.0f} sim={r['gz']:.3f} offset={r['gz'] - r['boot_s']:.4f}")
        oc = summarize("collect", [r for r in rec.att if r["phase"] == "collect"])

        print("== 2 Gazebo 일시정지 / 재개")
        t_ps = time.monotonic()
        gz_before = gz_rec(truth)[0]
        print("pause:", gz_control(True))
        rec.run(a.pause_s, "pause")
        t_pr = time.monotonic()
        gz_at_resume = gz_rec(truth)[0]
        print("resume:", gz_control(False))
        rec.run(a.after_s, "after")

        before = [r for r in rec.att if t_ps - 5 <= r["host"] <= t_ps]
        during = [r for r in rec.att if r["phase"] == "pause"]
        after = [r for r in rec.att if r["phase"] == "after" and r["host"] >= t_pr + 2]
        hbs = [t for t in rec.hb if t_ps <= t <= t_pr]
        stamps = [g for (t, g) in rec.gz_stamps if t_ps <= t <= t_pr]
        print(f"pause window {t_pr - t_ps:.1f} s (host): ATTITUDE={len(during)} HEARTBEAT={len(hbs)}")
        print(f"  gz pose stamps during pause: distinct={len(set(stamps))} "
              f"sim {gz_before:.3f} -> {gz_at_resume:.3f} (delta {gz_at_resume - gz_before:.3f} s)")
        if during:
            print(f"  boot_ms during pause first={during[0]['boot_s'] * 1e3:.0f} last={during[-1]['boot_s'] * 1e3:.0f}")
        if before and after:
            b_last, a_first = before[-1], after[0]
            print(f"  across pause: boot_ms {b_last['boot_s'] * 1e3:.0f} -> {a_first['boot_s'] * 1e3:.0f} "
                  f"(delta {a_first['boot_s'] - b_last['boot_s']:.3f} s), sim delta {a_first['gz'] - b_last['gz']:.3f} s, "
                  f"host gap {a_first['host'] - b_last['host']:.3f} s")
        ob = summarize("before pause (last 5 s)", before)
        oa = summarize("after resume (>= 2 s)", after)
        if ob is not None and oa is not None:
            print(f"  offset jump at pause/resume: {oa - ob:+.4f} s")

        print("== 3 SITL 재시작 (Gazebo 계속 실행)")
        before_restart = [r for r in rec.att if r["host"] >= time.monotonic() - 5]
        t_stop = time.monotonic()
        sitl.stop()
        t_start = sitl.start()
        mav.close()
        rec.mav = connect()
        rec.run(a.restart_collect_s, "restart")
        post = [r for r in rec.att if r["phase"] == "restart"]
        if post:
            f0 = post[0]
            print(f"  first ATTITUDE after restart: {f0['host'] - t_start:.2f} s after SITL start, "
                  f"boot_ms={f0['boot_s'] * 1e3:.0f} sim={f0['gz']:.3f} offset={f0['gz'] - f0['boot_s']:.4f}")
            sim_in = [g for (t, g) in rec.gz_stamps if t_stop <= t <= f0["host"]]
            if sim_in:
                print(f"  sim stamps between SITL stop and first sample: distinct={len(set(sim_in))} "
                      f"sim delta={sim_in[-1] - sim_in[0]:.3f} s")
        obr = summarize("before restart (last 5 s)", before_restart)
        onr = summarize("after restart", post)
        if obr is not None and onr is not None:
            print(f"  offset change at SITL restart: {onr - obr:+.4f} s")

        print("== SUMMARY")
        print(f"  start offset mean (collect): {oc:.4f} s  (sim_s = boot_s + offset)")
    finally:
        if rec is not None:
            write_csv(rec.att)
            print(f"samples: {os.path.join(OUT_DIR, 'time_sync_samples.csv')} ({len(rec.att)} rows)")
        if sitl.proc is not None:
            sitl.stop()


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
