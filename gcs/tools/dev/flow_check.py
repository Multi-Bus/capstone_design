"""광류 센서(MTF-01P) 방향 확인 — 손으로 들고 기울여 보는 시험 (시동·모터·파라미터 쓰기 없음).

ArduPilot 표준 시험과 같은 원리입니다: 기체를 제자리에서 기울이면(회전만, 이동 없이)
광류 센서가 보는 흐름(flowRate)과 FC 자이로의 회전 속도(bodyRate)가 **같은 부호로** 움직여야 합니다.
(FC 로그의 OF.flowX/bodyX 를 보는 것과 같음. FC 가 보내는 OPTICAL_FLOW 메시지 하나에 둘 다 들어 있음:
 ArduPilot GCS_MAVLINK::send_opticalflow() — flow_rate_x = flowRate.x, flow_comp_m_x = flowRate.x - bodyRate.x)
  - 같은 부호 (x↔롤, y↔피치)       → 방향 맞음
  - 둘 다 반대 부호                 → 센서가 180° 돌아가 있음 (FLOW_ORIENT_YAW 가 180° 틀림)
  - x 가 피치와, y 가 롤과 같이 움직임 → 90° 돌아가 있음

하는 법 (실행 후 약 60 s 동안):
  1. 프롭 없이, 기체를 바닥에서 30~50 cm 높이로 들어 바닥 쪽을 보게 수평으로 잡습니다.
  2. 제자리에서 **좌우로 10 s 정도** 천천히 기울였다 돌아오기를 반복 (±20° 정도, 1초에 한 번쯤)
  3. 이어서 **앞뒤로 10 s 정도** 같은 식으로 반복
  4. 남은 시간 동안 2~3 을 한 번 더. 기체를 옆으로 옮기지 말고 그 자리에서 기울이기만 하세요.
  (기울이는 순간만 계산에 쓰므로 시작 타이밍은 정확하지 않아도 됩니다)

    cd tools && source .venv/bin/activate
    python dev/flow_check.py
    python dev/flow_check.py --seconds 90

n6_fly.py 가 실행 중이면 먼저 q 로 끄세요 (XIAO 는 BLE 연결 하나만 받음).
"""
import argparse
import asyncio
import importlib.util
import math
import statistics
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("n6_fly", TOOLS / "n6_fly.py")
n6 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(n6)
m = n6.mavlink2

PARAMS = ["AHRS_ORIENTATION", "FLOW_TYPE", "FLOW_ORIENT_YAW", "FLOW_FXSCALER", "FLOW_FYSCALER",
          "FLOW_POS_X", "FLOW_POS_Y", "EK3_SRC1_VELXY", "EK3_FLOW_USE"]
MSG_ATTITUDE, MSG_OPTICAL_FLOW = 30, 100
FLOW_HZ = 10              # 너무 빠르면 XIAO(v1) 구간에서 대부분 빠짐 (20 Hz 에서 실측 3 %만 도착)
ATT_HZ = 10               # 자이로 교차 확인용 (OPTICAL_FLOW 의 bodyRate 가 0 이면 이걸 씀)
MSG_LOCAL_POSITION_NED = 32
LIFT_M = 0.20             # 거리센서가 이보다 높으면 '들어 올림'
MOVING_RAD_S = 0.3        # 이보다 빨리 돌 때만 계산에 사용


async def read_params(app, names, rounds=4):
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


def corr(a, b):
    if len(a) < 10:
        return float("nan")
    sa, sb = statistics.pstdev(a), statistics.pstdev(b)
    if sa < 1e-9 or sb < 1e-9:
        return float("nan")
    ma, mb = statistics.fmean(a), statistics.fmean(b)
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (len(a) * sa * sb)


def fit2(f, gx, gy):
    """f = a*gx + b*gy 최소제곱 (2x2)."""
    sxx = sum(x * x for x in gx); syy = sum(y * y for y in gy); sxy = sum(x * y for x, y in zip(gx, gy))
    sfx = sum(v * x for v, x in zip(f, gx)); sfy = sum(v * y for v, y in zip(f, gy))
    det = sxx * syy - sxy * sxy
    if abs(det) < 1e-12:
        return float("nan"), float("nan")
    return (sfx * syy - sfy * sxy) / det, (sfy * sxx - sfx * sxy) / det


async def main(opt):
    args = argparse.Namespace(name=opt.name, alt=0.8, hover=5.0, no_arm=True, scan_timeout=10.0,
                              log_dir=str(TOOLS / "logs"))
    con = n6.Console(Path(args.log_dir) / f"flow_check_{time.strftime('%Y%m%d_%H%M%S')}.log")
    link = n6.BleLink(args.name, con, args.scan_timeout)
    app = n6.N6Fly(args, con, link)

    rows = []          # (t, flowX, flowY, bodyX, bodyY, 높이, 품질)
    atts = []          # (t, rollspeed, pitchspeed) — 교차 확인용
    recording = {"on": False}
    orig_handle = app._handle

    def handle(msg, now):
        orig_handle(msg, now)
        if msg.get_srcSystem() != n6.FC_SYSID or not recording["on"]:
            return
        t = msg.get_type()
        if t == "ATTITUDE":
            atts.append((now, msg.rollspeed, msg.pitchspeed))
        elif t == "OPTICAL_FLOW" and app.s.rf_m is not None and app.s.rf_m > 0.15:
            frx, fry = getattr(msg, "flow_rate_x", 0.0), getattr(msg, "flow_rate_y", 0.0)
            rows.append((now, frx, fry, frx - msg.flow_comp_m_x, fry - msg.flow_comp_m_y,
                         msg.ground_distance, msg.quality))
    app._handle = handle

    run = asyncio.get_running_loop().create_task(app.run(asyncio.Queue()))
    try:
        end = time.monotonic() + 40
        while time.monotonic() < end and not (app.s.uplink_ok and app.fs_checked):
            await asyncio.sleep(0.1)
        if not app.s.uplink_ok:
            con.event(">>> 노트북→FC 확인 안 됨 (기체 전원·BLE 확인) — 중단")
            return
        p = await read_params(app, PARAMS)
        con.event(">>> 파라미터: " + ", ".join(f"{k}={v:g}" if v is not None else f"{k}=?" for k, v in p.items()))

        async def request_streams():
            for mid, hz in ((MSG_OPTICAL_FLOW, FLOW_HZ), (MSG_ATTITUDE, ATT_HZ)):
                await app.command(m.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz, quiet=True)
            # 대역 확보: 시험 중에는 LOCAL_POSITION_NED 끄기 (-1 = 보내지 않음). 끝나면 n6_fly 가 다시 켬
            await app.command(m.MAV_CMD_SET_MESSAGE_INTERVAL, MSG_LOCAL_POSITION_NED, -1, quiet=True)

        await request_streams()

        def lifted():
            st_ = app.s
            return st_.rf_m is not None and app._fresh(st_.rf_t) and st_.rf_m > LIFT_M

        con.event(f">>> 기체를 30~50 cm 높이로 들어 올리세요 — 거리센서가 {LIFT_M:.2f} m 를 넘으면 기록 시작 (최대 120 s 대기)")
        wait_end = time.monotonic() + 120
        while time.monotonic() < wait_end and not lifted():
            await asyncio.sleep(0.2)
        if not lifted():
            con.event(">>> 들어 올린 것이 확인되지 않음 — 중단")
            return
        con.event(f">>> 기록 시작 ({opt.seconds:.0f} s): 좌우로 10 s → 앞뒤로 10 s → 반복. "
                  "옮기지 말고 제자리에서 기울이기만")
        recording["on"] = True
        t0 = time.monotonic(); next_req = t0 + 5; next_note = t0 + 10
        while time.monotonic() - t0 < opt.seconds:
            now = time.monotonic()
            if now >= next_req:                 # 재연결로 간격 설정이 지워져도 다시 요청
                await request_streams(); next_req = now + 5
            if now >= next_note:
                con.event(f">>>   {now - t0:3.0f} s 경과, 광류 샘플 {len(rows)}개")
                next_note = now + 10
            await asyncio.sleep(0.2)
        recording["on"] = False

        if not rows:
            con.event(">>> OPTICAL_FLOW 를 하나도 못 받음 — FC 가 광류를 정상으로 보지 않음(FLOW_TYPE, 배선). "
                      "FC 로그의 OF 메시지로 확인 필요")
            return
        raw = TOOLS / "logs" / f"flow_check_raw_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        with open(raw, "w", encoding="utf-8") as f:
            f.write("t,flowX,flowY,bodyX,bodyY,hagl,quality\n")
            for r in rows:
                f.write(",".join(f"{v:.4f}" for v in r) + "\n")
        gd = [r[5] for r in rows]
        q = [r[6] for r in rows]
        con.event(f">>> 받은 광류 샘플 {len(rows)}개 (원자료 {raw.name}), 바닥 거리 평균 {statistics.fmean(gd):.2f} m, "
                  f"품질 평균 {statistics.fmean(q):.0f} (0~255)")
        # 교차 확인: 메시지에서 뽑은 bodyRate 가 정말 자이로인지 (ATTITUDE 와 가까운 시각끼리)
        pairs = []
        j = 0
        for r in rows:
            while j + 1 < len(atts) and atts[j + 1][0] <= r[0]:
                j += 1
            if atts and abs(atts[j][0] - r[0]) < 0.1:
                pairs.append((r[3], r[4], atts[j][1], atts[j][2]))
        if len(pairs) >= 10:
            con.event(f">>> (확인) bodyX vs ATTITUDE 롤속도 상관 {corr([p[0] for p in pairs], [p[2] for p in pairs]):+.2f}, "
                      f"bodyY vs 피치속도 {corr([p[1] for p in pairs], [p[3] for p in pairs]):+.2f} "
                      f"({len(pairs)}쌍, +1 에 가까워야 해석이 맞음)")
        body_sd = max(statistics.pstdev([r[3] for r in rows]), statistics.pstdev([r[4] for r in rows]))
        if body_sd < 0.02 and atts:
            con.event(">>> OPTICAL_FLOW 안의 bodyRate 가 거의 0 → ATTITUDE 자이로(가까운 시각)로 대신 계산")
            new_rows = []
            j = 0
            for r in rows:
                while j + 1 < len(atts) and atts[j + 1][0] <= r[0]:
                    j += 1
                if abs(atts[j][0] - r[0]) < 0.1:
                    new_rows.append((r[0], r[1], r[2], atts[j][1], atts[j][2], r[5], r[6]))
            rows = new_rows
        mv = [r for r in rows if math.hypot(r[3], r[4]) > MOVING_RAD_S]
        con.event(f">>> 기울이는 중(|회전| > {MOVING_RAD_S} rad/s)인 샘플 {len(mv)}개")
        if len(mv) < 10:
            con.event(">>> 기울인 구간이 너무 짧음 — 더 크게/오래 기울여서 다시")
            return
        fx = [r[1] for r in mv]; fy = [r[2] for r in mv]
        gx = [r[3] for r in mv]; gy = [r[4] for r in mv]
        con.event(">>> 상관계수 (+1 = 같이 움직임, -1 = 반대로 움직임)")
        con.event(f">>>   광류X vs 롤 {corr(fx, gx):+.2f} | 광류X vs 피치 {corr(fx, gy):+.2f}")
        con.event(f">>>   광류Y vs 롤 {corr(fy, gx):+.2f} | 광류Y vs 피치 {corr(fy, gy):+.2f}")
        a, b = fit2(fx, gx, gy)
        c, d = fit2(fy, gx, gy)
        con.event(f">>> 광류 = M x 회전,  M = [[{a:+.2f} {b:+.2f}] [{c:+.2f} {d:+.2f}]]  (맞으면 약 [[+1 0] [0 +1]])")
        ang = math.degrees(math.atan2(c - b, a + d))
        scale = math.hypot(a + d, c - b) / 2
        con.event(f">>> 회전 어긋남 약 {ang:+.0f}°, 크기 비 약 {scale:.2f}")
        if abs(ang) < 30:
            verdict = "방향 맞음 (FLOW_ORIENT_YAW 그대로 OK)"
        elif abs(ang) > 150:
            verdict = "센서가 약 180° 돌아가 있음 → FLOW_ORIENT_YAW 를 180° 바꿔야 함 (예: 0 → 18000)"
        else:
            verdict = (f"센서가 약 {ang:+.0f}° 돌아가 있음 → FLOW_ORIENT_YAW 를 그만큼 보정해야 함 "
                       "(부호는 바꾼 뒤 이 시험으로 다시 확인)")
        con.event(">>> 판정: " + verdict)
    finally:
        await app.request_quit("flow check done")
        await asyncio.wait_for(run, 15)
        con.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="광류 센서 방향 확인 (시동 없음)")
    ap.add_argument("--name", default="N6-DRONE-1", help="XIAO BLE 이름")
    ap.add_argument("--seconds", type=float, default=60.0, help="기록 시간 s")
    asyncio.run(main(ap.parse_args()))
