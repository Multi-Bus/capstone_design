#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FC 장착 방향(AHRS_ORIENTATION) 확인.

기체를 세 자세(바로 놓기 / 기수 내리기 / 오른쪽 내리기)로 잡고 있으면 가속도계 값을 2초씩 평균해서
1) 지금 AHRS_ORIENTATION 으로 자세(ATTITUDE)가 맞게 나오는지 보여 주고
2) 측정값에 가장 잘 맞는 AHRS_ORIENTATION 값을 계산한다 (90° 단위 장착만).

안전: 읽기 전용. 파라미터를 쓰지 않고 재부팅도 하지 않는다. 시동/모터 명령 없음.
      값을 바꿔야 하면 화면에 나오는 apply_params.py 명령으로 바꾼다.

사용: .venv/bin/python orient_check.py --drone 3
"""
import argparse
import json
import math
import os
import re
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import apply_params as AP                      # noqa: E402
from apply_params import FC, ML, ArmedError    # noqa: E402
from sensor_check import port_holders, set_interval   # noqa: E402

HOLDS = [
    ('level', '기체를 평평한 바닥(책상)에 바로 놓고 손을 떼세요.'),
    ('nosedown', '기체 앞쪽(기수)을 아래로 30~60° 숙인 채 잡고 가만히 있으세요.'),
    ('rightdown', '기체 오른쪽을 아래로 30~60° 기울인 채 잡고 가만히 있으세요. (뒤에서 봤을 때 오른쪽)'),
]
MIN_TILT_MG = 350          # 기울인 자세에서 바뀌어야 하는 가속도 크기 (약 20° 이상)
MAX_SPREAD_MG = 200        # 2초 동안 흔들림 허용


# ---------------------------------------------------------------- 회전 (ArduPilot 과 같은 정의)
def from_euler(r, p, y):
    """ArduPilot Matrix3::from_euler (deg). body = M * board."""
    r, p, y = (math.radians(a) for a in (r, p, y))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
            [-sp, sr * cp, cr * cp]]


def mul(M, v):
    return [sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]


def mul_t(M, v):
    return [sum(M[j][i] * v[j] for j in range(3)) for i in range(3)]


def rotations():
    """MAV_SENSOR_ORIENTATION 이름에서 각도를 읽는다. 값 -> (이름, 행렬, 90°단위 여부)."""
    out = {}
    for v, e in sorted(ML.enums['MAV_SENSOR_ORIENTATION'].items()):
        if v >= 100:
            continue
        name = e.name.replace('MAV_SENSOR_ROTATION_', '')
        found = re.findall(r'(ROLL|PITCH|YAW)_(\d+)', name)
        if name != 'NONE' and not found:
            continue
        ang = {'ROLL': 0, 'PITCH': 0, 'YAW': 0}
        for axis, deg in found:
            ang[axis] = int(deg)
        right = all(a % 90 == 0 for a in ang.values()) and v != 38   # 38 은 실제로 68.8°/293.3°
        out[v] = (name, from_euler(ang['ROLL'], ang['PITCH'], ang['YAW']), right)
    return out


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def solve(holds, current):
    """holds: {'level','nosedown','rightdown'} -> 지금 AHRS_ORIENTATION 기준 가속도(mG).
    반환: 점수 순 [(점수, 값, 이름, (z, x, y))]"""
    rots = rotations()
    Mcur = rots[current][1] if current in rots else from_euler(0, 0, 0)
    b = {k: mul_t(Mcur, v) for k, v in holds.items()}        # 보드 좌표계로 되돌림
    d_nd = [b['nosedown'][i] - b['level'][i] for i in range(3)]
    d_rd = [b['rightdown'][i] - b['level'][i] for i in range(3)]
    seen, ranked = [], []
    for v, (name, M, right) in rots.items():
        if not right:
            continue
        key = tuple(round(x) for row in M for x in row)
        if key in seen:                       # 같은 회전의 다른 이름 (예: 10 과 27) -> 작은 번호만
            continue
        seen.append(key)
        ez = -mul(M, b['level'])[2] / norm(b['level'])      # 바로 놓으면 body z = -1 g
        ex = -mul(M, d_nd)[0] / norm(d_nd)                  # 기수 내리면 body x 가 - 쪽으로
        ey = -mul(M, d_rd)[1] / norm(d_rd)                  # 오른쪽 내리면 body y 가 - 쪽으로
        ranked.append((ez + ex + ey, v, name, (ez, ex, ey)))
    ranked.sort(reverse=True)
    return ranked


# ---------------------------------------------------------------- 측정
def drain(fc, seconds):
    fc.recv(None, seconds, cond=lambda m: False)


def measure(fc, seconds=2.0):
    acc, att = [], []
    end = time.time() + seconds
    while time.time() < end:
        m = fc.recv({'RAW_IMU', 'ATTITUDE'}, 0.5, cond=fc._from_fc)
        if m is None:
            continue
        if m.get_type() == 'RAW_IMU':
            acc.append((m.xacc, m.yacc, m.zacc))
        else:
            att.append((math.degrees(m.roll), math.degrees(m.pitch)))
    if not acc:
        return None
    avg = [sum(a[i] for a in acc) / len(acc) for i in range(3)]
    spread = max(max(a[i] for a in acc) - min(a[i] for a in acc) for i in range(3))
    roll = sum(a[0] for a in att) / len(att) if att else None
    pitch = sum(a[1] for a in att) / len(att) if att else None
    return {'acc': avg, 'spread': spread, 'n': len(acc), 'roll': roll, 'pitch': pitch}


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        raise SystemExit('입력이 없어 중단')


def main():
    ap = argparse.ArgumentParser(description='FC 장착 방향 확인 (읽기 전용)')
    ap.add_argument('--port', default=AP.DEFAULT_PORT_GLOB)
    ap.add_argument('--drone', default='', help='기체 번호 (기록 파일 이름에만 사용)')
    args = ap.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    tag = f'_drone{args.drone}' if args.drone else ''
    sys.stdout = AP.Tee(os.path.join(HERE, f'orient{tag}_{stamp}.txt'))

    held = port_holders(args.port)
    if held:
        print(f'!!! FC 포트를 다른 프로그램이 잡고 있음: {held} -> QGC 등을 꺼 주세요')
        return 1
    fc = FC(args.port)
    rec = {'stamp': stamp, 'drone': args.drone, 'holds': {}}
    code = 0
    try:
        fc.connect(30)
        r = fc.read_param('AHRS_ORIENTATION')
        if r is None:
            raise SystemExit('AHRS_ORIENTATION 을 읽지 못함')
        current = int(round(r[0]))
        rots = rotations()
        print(f'  연결됨: {fc.port}')
        print(f'  지금 AHRS_ORIENTATION = {current} ({rots.get(current, ("?",))[0]})')
        rec['current'] = current
        set_interval(fc, ML.MAVLINK_MSG_ID_RAW_IMU, 20)
        set_interval(fc, ML.MAVLINK_MSG_ID_ATTITUDE, 10)

        holds = {}
        for key, text in HOLDS:
            while True:
                print(f'\n[{key}] {text}')
                ask('  준비되면 Enter ... ')
                drain(fc, 1.0)                # 쌓인 옛 메시지 버리고 1초 안정
                m = measure(fc)
                if m is None:
                    print('  RAW_IMU 를 받지 못함, 다시')
                    continue
                print(f"  가속도 평균 (mG): x={m['acc'][0]:+.0f} y={m['acc'][1]:+.0f} z={m['acc'][2]:+.0f}"
                      f"  흔들림 {m['spread']:.0f}"
                      + ('' if m['roll'] is None else f"  | 자세 roll {m['roll']:+.1f}°, pitch {m['pitch']:+.1f}°"))
                if m['spread'] > MAX_SPREAD_MG:
                    print('  흔들림이 커요. 더 가만히 잡고 다시 해 주세요.')
                    continue
                if key != 'level':
                    d = norm([m['acc'][i] - holds['level']['acc'][i] for i in range(3)])
                    if d < MIN_TILT_MG:
                        print(f'  덜 기울었어요 (변화 {d:.0f} mG). 30° 이상 기울여서 다시 해 주세요.')
                        continue
                holds[key] = m
                break
        rec['holds'] = holds
        set_interval(fc, ML.MAVLINK_MSG_ID_RAW_IMU, 0)
        set_interval(fc, ML.MAVLINK_MSG_ID_ATTITUDE, 0)

        # 1) 지금 설정으로 자세가 맞는지 (FC 가 계산한 값 그대로)
        lv, nd, rd = holds['level'], holds['nosedown'], holds['rightdown']
        print('\n== 지금 설정으로 본 자세 (FC 계산값) ==')
        att_ok = None
        if None not in (lv['roll'], nd['pitch'], rd['roll']):
            c1 = abs(lv['roll']) < 10 and abs(lv['pitch']) < 10
            c2 = nd['pitch'] < -15
            c3 = rd['roll'] > 15
            print(f"  바로 놓기    : roll {lv['roll']:+.1f}°, pitch {lv['pitch']:+.1f}°  -> 둘 다 0 근처  {'OK' if c1 else '아님'}")
            print(f"  기수 내리기  : pitch {nd['pitch']:+.1f}°  -> 음수(-)여야 함  {'OK' if c2 else '아님'}")
            print(f"  오른쪽 내리기: roll {rd['roll']:+.1f}°  -> 양수(+)여야 함  {'OK' if c3 else '아님'}")
            att_ok = c1 and c2 and c3
        rec['attitude_ok'] = att_ok

        # 2) 가속도로 계산한 장착 방향
        ranked = solve({k: holds[k]['acc'] for k in holds}, current)
        best, second = ranked[0], ranked[1]
        confident = best[0] > 2.4 and min(best[3]) > 0.7 and best[0] - second[0] > 0.8
        rec['best'] = {'value': best[1], 'name': best[2], 'score': best[0]}
        rec['second'] = {'value': second[1], 'name': second[2], 'score': second[0]}
        print('\n== 가속도로 계산한 장착 방향 ==')
        print(f'  가장 잘 맞는 값: AHRS_ORIENTATION = {best[1]} ({best[2]}), 점수 {best[0]:.2f}/3')
        print(f'  두 번째        : {second[1]} ({second[2]}), 점수 {second[0]:.2f}/3')

        print('\n== 결론 ==')
        if not confident:
            print('  측정이 애매해요. 기수/오른쪽을 정확히 내렸는지 확인하고 다시 해 보세요.')
            code = 3
        elif best[1] == current and att_ok is not False:
            print(f'  지금 값 {current} 이 맞아요. 바꿀 필요 없음.')
        elif best[1] == current:
            print(f'  계산 결과는 지금 값 {current} 과 같은데 자세 표시가 이상해요. 담당자에게 알려 주세요.')
            code = 3
        else:
            print(f'  AHRS_ORIENTATION 을 {current} -> {best[1]} ({best[2]}) 로 바꿔야 해요.')
            print(f'  1) drones/drone<N>.param 에 "AHRS_ORIENTATION,{best[1]}" 을 적고')
            print('  2) .venv/bin/python apply_params.py --drone <N>   (적용 + 재부팅)')
            print('  3) 이 스크립트를 다시 돌려 "바꿀 필요 없음" 이 나오는지 확인')
            print('  4) QGC 에서 가속도계 6방향 + 수평 보정을 다시 하기')
            code = 4
        rec['exit_code'] = code
    except ArmedError as e:
        print(f'\n!!! {e}')
        code = 2
    except SystemExit as e:
        print(f'\n!!! 중단: {e}')
        code = 1
    except KeyboardInterrupt:
        print('\n중단 (Ctrl+C)')
        code = 1
    finally:
        try:
            if fc.m is not None:
                set_interval(fc, ML.MAVLINK_MSG_ID_RAW_IMU, 0)
                set_interval(fc, ML.MAVLINK_MSG_ID_ATTITUDE, 0)
        except Exception:
            pass
        fc.close()
        with open(os.path.join(HERE, f'orient{tag}_{stamp}.json'), 'w', encoding='utf-8') as f:
            json.dump(rec, f, ensure_ascii=False, indent=1, default=str)
        print('\n포트 닫음.')
        sys.stdout.flush()
    return code


if __name__ == '__main__':
    sys.exit(main())
