#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
최종 점검 + 저장.

FC 파라미터를 전부 읽어서
  1) n6_base.param (+ drones/drone<N>.param, --set) 값과 같은지
  2) 가속도계 보정 / 수평 보정이 됐는지
  3) 기체별 값 (모터 방향, 장착 방향, 전압 배율, MOT_SPIN_*)
를 보여 주고 drone<N>_final_<날짜>.param 으로 저장한다.
QGC 보정이 끝난 뒤에 돌린다 (보정 값까지 파일에 남기려고).

안전: 읽기 전용. 파라미터를 쓰지 않고 재부팅도 하지 않는다. 시동/모터 명령 없음.

사용: .venv/bin/python save_params.py --drone 3
"""
import argparse
import json
import math
import os
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import apply_params as AP                  # noqa: E402
from apply_params import FC, ArmedError    # noqa: E402
from sensor_check import port_holders     # noqa: E402

PER_DRONE = ['SERVO_BLH_RVMASK', 'AHRS_ORIENTATION', 'FLOW_ORIENT_YAW', 'BATT_VOLT_MULT',
             'MOT_SPIN_ARM', 'MOT_SPIN_MIN']


def calib_report(P):
    """가속도계 / 수평 보정 상태. 반환: (문제 목록, 설명 줄)"""
    g = lambda n: P[n][0] if n in P else None      # noqa: E731
    problems, lines = [], []
    for k, pre, temp in (('1', 'INS_ACC', 'INS_ACC1_CALTEMP'), ('2', 'INS_ACC2', 'INS_ACC2_CALTEMP'),
                         ('3', 'INS_ACC3', 'INS_ACC3_CALTEMP')):
        dev = g(f'{pre}_ID')
        if not dev:
            continue                               # 이 번호의 IMU 없음
        offs = [g(f'{pre}OFFS_{a}') for a in 'XYZ']
        scal = [g(f'{pre}SCAL_{a}') for a in 'XYZ']
        untouched = (all(o is not None and abs(o) < 1e-7 for o in offs)
                     and all(s is not None and abs(s - 1) < 1e-7 for s in scal))
        no_temp = g(temp) is not None and g(temp) <= -299
        ok = not untouched and not no_temp
        lines.append(f'  가속도계 {k}: offs ' + ', '.join(AP.fmt(o) for o in offs)
                     + ' / scale ' + ', '.join(AP.fmt(s) for s in scal)
                     + f'  -> {"보정됨" if ok else "보정 안 됨"}')
        if not ok:
            problems.append(f'가속도계 {k} 보정 안 됨 (QGC 센서 -> 가속도계)')
    tx, ty = g('AHRS_TRIM_X'), g('AHRS_TRIM_Y')
    if tx is not None and ty is not None:
        lvl = not (tx == 0 and ty == 0)
        lines.append(f'  수평 보정 AHRS_TRIM_X/Y: {AP.fmt(tx)}, {AP.fmt(ty)} rad '
                     f'({math.degrees(tx):+.2f}°, {math.degrees(ty):+.2f}°)  -> {"함" if lvl else "안 한 것 같음"}')
        if not lvl:
            problems.append('수평(Level) 보정 안 한 것 같음 (QGC 센서 -> 수평 맞추기)')
        if max(abs(math.degrees(tx)), abs(math.degrees(ty))) > 5:
            problems.append('수평 보정 값이 5° 넘게 큼 -> FC 장착 방향/기체 기울기 확인')
    return problems, lines


def main():
    ap = argparse.ArgumentParser(description='최종 점검 + 파라미터 저장 (읽기 전용)')
    ap.add_argument('--drone', required=True, help='기체 번호')
    ap.add_argument('--port', default=AP.DEFAULT_PORT_GLOB)
    ap.add_argument('--param-file', default=os.path.join(HERE, 'n6_base.param'))
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='추가로 확인할 값 (apply_params.py 에 준 --set 과 같게)')
    ap.add_argument('--note', default='', help='저장 파일 머리말에 남길 메모')
    ap.add_argument('--outdir', default=HERE)
    args = ap.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(args.outdir, exist_ok=True)
    sys.stdout = AP.Tee(os.path.join(args.outdir, f'save_log_drone{args.drone}_{stamp}.txt'))

    try:
        items, sources = AP.load_items(args.param_file, args.drone, args.set)
    except ValueError as e:
        ap.error(str(e))
    for line in sources:
        print(line)
    held = port_holders(args.port)
    if held:
        print(f'!!! FC 포트를 다른 프로그램이 잡고 있음: {held} -> QGC 등을 꺼 주세요')
        return 1

    fc = FC(args.port)
    summary = {'stamp': stamp, 'drone': args.drone, 'sources': sources}
    code = 0
    try:
        hb = fc.connect(30)
        av = fc.request_version()
        AP.print_info(fc, hb, av)
        print('\n== 전체 파라미터 읽기 ==')
        count, nmiss = fc.fetch_all()
        P = fc.params
        print(f'  {len(P)}개 읽음 (FC 보고 {count}개, 끝내 못 받은 것 {nmiss}개)')
        fw, board, _frame = AP.banner_info(fc.statustext)

        print('\n== 정해진 값과 비교 ==')
        mavn = AP.has_mavn_params(P)
        bad, missing = [], []
        for name, value in items:
            if name in AP.NEVER_TOUCH:
                continue
            r = AP.resolve(fc, name, value, mavn)
            if r is None:
                missing.append(name)
                print(f'  {name:<17} 이 펌웨어에 없음')
                continue
            ok = AP.same(r['prev'], r['target'])
            if not ok:
                bad.append(name)
            alias = f'  ({r["name"]} 사용)' if r['name'] != name else ''
            if not ok:
                print(f"  {name:<17} 지금 {AP.fmt(r['prev'], r['is_int'])}, 정해진 값 "
                      f"{AP.fmt(r['target'], r['is_int'])}  <- 다름!{alias}")
        print(f'  {len(items)}개 중 같음 {len(items) - len(bad) - len(missing)}, '
              f'다름 {len(bad)}, 없음 {len(missing)}')
        if bad:
            print('  -> 다른 값이 있으면 apply_params.py --drone <N> 을 다시 돌리세요.')

        print('\n== 기체별 값 ==')
        for n in PER_DRONE:
            v = P.get(n)
            print(f'  {n:<17} = {"없음" if v is None else AP.fmt(v[0], v[1] != AP.REAL32)}')

        print('\n== 보정 상태 ==')
        problems, lines = calib_report(P)
        for line in lines:
            print(line)

        path = AP.unique_path(os.path.join(
            args.outdir, f'drone{args.drone}_final_{datetime.now().strftime("%Y%m%d")}.param'))
        header = [f'drone{args.drone} final {stamp}' + (f' - {args.note}' if args.note else ''),
                  f'{fw} / {board}']
        AP.write_param_file(path, P, header)
        print(f'\n최종 파라미터 저장: {path}')

        print('\n== 결론 ==')
        todo = ([f'값이 다름: {", ".join(bad)}'] if bad else []) + \
               ([f'이름 없음: {", ".join(missing)}'] if missing else []) + problems
        for t in todo or ['문제 없음']:
            print(f'  - {t}')
        summary.update(final_file=path, mismatch=bad, missing=missing, problems=problems,
                       per_drone={n: (P[n][0] if n in P else None) for n in PER_DRONE})
        code = 0 if not todo else 5
    except ArmedError as e:
        print(f'\n!!! {e}')
        code = 2
    except SystemExit as e:
        print(f'\n!!! 중단: {e}')
        code = 1
    finally:
        fc.close()
        summary['exit_code'] = code
        with open(os.path.join(args.outdir, f'save_result_drone{args.drone}_{stamp}.json'), 'w',
                  encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1, default=str)
        print('\n포트 닫음.')
        sys.stdout.flush()
    return code


if __name__ == '__main__':
    sys.exit(main())
