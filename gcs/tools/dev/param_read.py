"""FC 파라미터를 BLE 로 읽기만 하는 도구 (쓰기·시동 없음).

n6_fly.py 의 BLE/MAVLink 코드를 그대로 씁니다. XIAO 는 BLE 연결을 하나만 받으므로
n6_fly.py 가 실행 중이면 먼저 q 로 끄세요.

    cd tools && source .venv/bin/activate
    python dev/param_read.py EK3_SRC1_POSZ EK3_RNG_USE_HGT RNGFND1_GNDCLR MOT_THST_HOVER
    python dev/param_read.py --name N6-DRONE-2 FS_GCS_ENABLE

결과는 화면과 gcs/tools/logs/param_read_*.log 에 남습니다. 못 읽은 값은 "?" 로 나옵니다
(FC→노트북 방향이 XIAO 에서 일부 빠지므로 4번까지 다시 요청합니다).
"""
import argparse
import asyncio
import importlib.util
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("n6_fly", TOOLS / "n6_fly.py")
n6 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(n6)


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


async def main(opt):
    args = argparse.Namespace(name=opt.name, alt=0.8, hover=5.0, no_arm=True, scan_timeout=10.0,
                              log_dir=str(TOOLS / "logs"))
    con = n6.Console(Path(args.log_dir) / f"param_read_{time.strftime('%Y%m%d_%H%M%S')}.log")
    link = n6.BleLink(args.name, con, args.scan_timeout)
    app = n6.N6Fly(args, con, link)
    run = asyncio.get_running_loop().create_task(app.run(asyncio.Queue()))
    try:
        end = time.monotonic() + 40
        while time.monotonic() < end and not (app.s.uplink_ok and app.fs_checked):
            await asyncio.sleep(0.1)
        if not app.s.uplink_ok:
            con.event(">>> 노트북→FC 확인 안 됨 (기체 전원·BLE 확인) — 중단")
            return
        vals = await read_params(app, opt.params)
        for k, v in vals.items():
            con.event(f">>> {k} = {v:g}" if v is not None else f">>> {k} = ? (못 읽음 또는 없는 이름)")
    finally:
        await app.request_quit("param read done")
        await asyncio.wait_for(run, 15)
        con.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="FC 파라미터 읽기 전용 (BLE)")
    p.add_argument("--name", default="N6-DRONE-1", help="XIAO BLE 이름")
    p.add_argument("params", nargs="+", help="읽을 파라미터 이름들")
    asyncio.run(main(p.parse_args()))
