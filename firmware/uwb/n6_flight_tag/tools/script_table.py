"""대본(움직임 표)을 프레임 번호로 풀어 CSV 로 낸다. 같은 대본을 시뮬레이터에서 돌려 실물과 견줄 때 쓴다.

    python tools/script_table.py --script 1 --speed 30 --start 1500 --out script1_30.csv
    python tools/script_table.py --script 2 --speed 70 --start 1500 --mover 4 --tags 6 --out script2_70_tag4.csv

대본의 정의는 펌웨어(tag/fcl.c 의 script_1 ~ script_4)와 같아야 한다. tag/test_fcl.py 가 둘이 같은지 확인한다.
속도는 기체 기준이다: vx 는 앞(+), vy 는 오른쪽(+). 기수 방향은 바꾸지 않는다.

CSV 의 줄: frame, drone, vx_m_s, vy_m_s, row      (속도가 바뀌는 프레임만. 그 사이는 같은 값)
  - frame 은 앵커 1 을 켠 때부터 센 프레임 번호. 대본은 --start 프레임에 시작한다.
  - 대본이 끝난 뒤의 줄(속도 0, row = 줄 수)도 넣는다.
"""
import argparse
import csv
import math
from pathlib import Path

# (앞으로 cm, 오른쪽으로 cm, 멈춰 있는 ms)  -- tag/fcl.c 와 같은 값
SCRIPTS = {
    1: [(200, 0, 0), (0, 0, 5000), (-200, 0, 0), (0, 0, 5000), (0, 200, 0), (0, 0, 5000), (0, -200, 0), (0, 0, 5000)],
    2: [(400, 0, 0), (0, 0, 5000), (-400, 0, 0), (0, 0, 5000)],
    3: [(300, 0, 0), (0, 0, 10000), (-300, 0, 0), (0, 0, 5000)],
    4: [(0, 0, 60000)],
}
NAMES = {1: "앞 2 m, 원점, 오른쪽 2 m, 원점", 2: "직선 4 m 왕복", 3: "3 m 가다가 정지, 원점", 4: "제자리 60 s"}


def table(script, speed_cm, frame_us=20000):
    """[(줄 번호, 프레임 수, vx cm/s, vy cm/s), ...]"""
    rows = []
    for row, (forward, right, hold_ms) in enumerate(SCRIPTS[script]):
        if forward == 0 and right == 0:
            frames = hold_ms * 1000 // frame_us
        else:
            frames = math.ceil((abs(forward) + abs(right)) * 1_000_000 / (speed_cm * frame_us))
        sign = lambda v: speed_cm if v > 0 else -speed_cm if v < 0 else 0
        rows.append((row, frames, sign(forward), sign(right)))
    return rows


def extent_cm(script):
    forward = right = extent = 0
    for f, r, _ in SCRIPTS[script]:
        forward += f
        right += r
        extent = max(extent, abs(forward), abs(right))
    return extent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--script", type=int, required=True, choices=sorted(SCRIPTS))
    p.add_argument("--speed", type=int, default=30, help="cm/s (기본 30)")
    p.add_argument("--start", type=int, default=1500, help="대본이 시작하는 프레임 번호 (기본 1500)")
    p.add_argument("--mover", type=int, default=-1, help="움직일 기체 번호. -1 = 모두 (기본)")
    p.add_argument("--tags", type=int, default=6)
    p.add_argument("--frame-us", type=int, default=20000)
    p.add_argument("--out", type=Path)
    opt = p.parse_args()
    rows = table(opt.script, opt.speed, opt.frame_us)
    lines = []
    for drone in range(opt.tags):
        moving = opt.mover < 0 or opt.mover == drone
        frame = opt.start
        for row, frames, vx, vy in rows:
            lines.append((frame, drone, vx / 100 if moving else 0.0, vy / 100 if moving else 0.0, row))
            frame += frames
        lines.append((frame, drone, 0.0, 0.0, len(rows)))
    lines.sort()
    total = sum(r[1] for r in rows)
    print(f"대본 {opt.script} ({NAMES[opt.script]}), {opt.speed} cm/s: {total} 프레임 = {total * opt.frame_us / 1e6:.1f} s, "
          f"가장 멀리 {extent_cm(opt.script)} cm, 구역 한도 {extent_cm(opt.script) + 150} cm")
    if opt.out:
        with open(opt.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["frame", "drone", "vx_m_s", "vy_m_s", "row"])
            w.writerows(lines)
        print(f"저장: {opt.out} ({len(lines)}줄)")
    else:
        for line in lines:
            print(",".join(str(v) for v in line))


if __name__ == "__main__":
    main()
