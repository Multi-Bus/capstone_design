"""바닥에 둔 상태 / 손으로 든 상태에서 FC 의 EKF 높이와 거리센서 값을 비교 기록 (시동 없음, 프롭 없이).

높이 관련 파라미터를 바꾸거나 재부팅한 뒤, FC 높이 추정이 거리센서를 따라가는지 확인하는 용도입니다.
기록 중에 기체를 바닥에 두었다가 → 손으로 0.3~0.5 m 들어 10 s 이상 유지 → 다시 바닥에 내려놓으세요.

    cd tools && source .venv/bin/activate
    python dev/height_check.py              # 80 s 기록
    python dev/height_check.py --seconds 60

정상 예 (2026-11-19 23:16, 재부팅 후): 바닥 EKF +0.04 / 거리센서 0.02, 들었을 때 차이 평균 -0.01 m.
n6_fly.py 가 실행 중이면 먼저 q 로 끄세요 (XIAO 는 BLE 연결 하나만 받음).
"""
import argparse
import asyncio
import importlib.util
import statistics
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("n6_fly", TOOLS / "n6_fly.py")
n6 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(n6)

PARAMS = ["EK3_SRC1_POSZ", "EK3_RNG_USE_HGT", "EK3_RNG_USE_SPD", "RNGFND1_GNDCLR", "RNGFND1_MIN", "RNGFND1_MAX"]


async def read_params(app, names, rounds=3):
    for n in names:
        app.params.pop(n, None)
    for _ in range(rounds):
        todo = [n for n in names if n not in app.params]
        if not todo:
            break
        for n in todo:
            await app.send(app.tx.param_request_read_encode(n6.FC_SYSID, n6.FC_COMPID, n.encode(), -1))
            await asyncio.sleep(0.3)
        end = time.monotonic() + 1.5
        while time.monotonic() < end and any(n not in app.params for n in names):
            await asyncio.sleep(0.1)
    return {n: app.params.get(n) for n in names}


async def main(opt):
    args = argparse.Namespace(name=opt.name, alt=0.8, hover=5.0, no_arm=True, scan_timeout=8.0,
                              log_dir=str(TOOLS / "logs"))
    con = n6.Console(Path(args.log_dir) / f"height_check_{time.strftime('%Y%m%d_%H%M%S')}.log")
    link = n6.BleLink(args.name, con, args.scan_timeout)
    app = n6.N6Fly(args, con, link)
    run = asyncio.get_running_loop().create_task(app.run(asyncio.Queue()))
    try:
        end = time.monotonic() + 30
        while time.monotonic() < end and not app.s.uplink_ok:
            await asyncio.sleep(0.1)
        if not app.s.uplink_ok:
            con.event(">>> 노트북→FC 확인 안 됨 (기체 전원·BLE 확인) — 중단")
            return
        con.event(f">>> 기록 시작 ({opt.seconds:.0f} s) — 바닥 → 손으로 들기 → 바닥")
        rows = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < opt.seconds:
            s = app.s
            e = -s.lpn_z if (s.lpn_z is not None and app._fresh(s.lpn_t)) else None
            r = s.rf_m if (s.rf_m is not None and app._fresh(s.rf_t)) else None
            rows.append((time.monotonic() - t0, e, r))
            await asyncio.sleep(0.2)
        con.event(">>> 기록 끝 — 시계열(2 s 간격): 시각 EKF높이/거리센서")
        line = []
        for t, e, r in rows[::10]:
            line.append(f"{t:4.0f}s {e:+.2f}/{r:.2f}" if (e is not None and r is not None) else f"{t:4.0f}s -")
        for i in range(0, len(line), 6):
            con.event(">>>   " + " | ".join(line[i:i + 6]))
        both = [(e, r) for _, e, r in rows if e is not None and r is not None]
        gnd = [(e, r) for e, r in both if r < 0.12]
        up = [(e, r) for e, r in both if r > 0.20]
        if gnd:
            con.event(f">>> 바닥 ({len(gnd)}개): EKF 평균 {statistics.mean(e for e, _ in gnd):+.2f}, "
                      f"거리센서 평균 {statistics.mean(r for _, r in gnd):.2f}")
        if up:
            d = [e - r for e, r in up]
            con.event(f">>> 들었을 때 ({len(up)}개): EKF 평균 {statistics.mean(e for e, _ in up):+.2f}, 거리센서 평균 "
                      f"{statistics.mean(r for _, r in up):.2f}, 차이(EKF−거리센서) 평균 {statistics.mean(d):+.2f} "
                      f"(범위 {min(d):+.2f}~{max(d):+.2f})")
        else:
            con.event(">>> 들어 올린 구간(거리센서 > 0.2 m)이 기록되지 않음")
        p = await read_params(app, PARAMS)
        con.event(">>> 파라미터: " + ", ".join(f"{k}={v:g}" if v is not None else f"{k}=?" for k, v in p.items()))
    finally:
        await app.request_quit("height check done")
        await asyncio.wait_for(run, 15)
        con.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="EKF 높이 vs 거리센서 기록 (시동 없음)")
    ap.add_argument("--name", default="N6-DRONE-1", help="XIAO BLE 이름")
    ap.add_argument("--seconds", type=float, default=80.0, help="기록 시간 s")
    asyncio.run(main(ap.parse_args()))
