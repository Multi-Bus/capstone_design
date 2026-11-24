#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FC 센서 확인 (광류 / 거리 센서 / SYS_STATUS / 배터리 전압).

안전: 재부팅(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=1), 메시지 주기 요청
      (MAV_CMD_SET_MESSAGE_INTERVAL, 저장 안 됨), GCS 하트비트만 보낸다.
      시동 / 모터 / 이륙 명령은 없다. 파라미터도 쓰지 않는다.

  reboot : 재부팅 -> 다시 연결 -> --listen 초 동안 STATUSTEXT 수집 -> SYS_STATUS 요약
  watch  : --seconds 초 동안 OPTICAL_FLOW / DISTANCE_SENSOR / RANGEFINDER 기록
"""
import argparse
import glob
import json
import os
import re
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import apply_params as AP          # noqa: E402
from apply_params import FC, ML, ArmedError, dedupe   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STREAMS = [  # (이름, 메시지 ID, Hz)
    ('SYS_STATUS', ML.MAVLINK_MSG_ID_SYS_STATUS, 2),
    ('BATTERY_STATUS', ML.MAVLINK_MSG_ID_BATTERY_STATUS, 1),
    ('OPTICAL_FLOW', ML.MAVLINK_MSG_ID_OPTICAL_FLOW, 10),
    ('DISTANCE_SENSOR', ML.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10),
    ('RANGEFINDER', ML.MAVLINK_MSG_ID_RANGEFINDER, 10),
]
SENSOR_ENUM = ML.enums['MAV_SYS_STATUS_SENSOR']


def port_holders(port_glob):
    """FC tty를 잡고 있는 다른 프로세스 (QGC 등)."""
    out = []
    for p in glob.glob(port_glob):
        dev = os.path.realpath(p)
        for fd in glob.glob('/proc/[0-9]*/fd/*'):
            try:
                if os.readlink(fd) == dev:
                    pid = fd.split('/')[2]
                    if int(pid) != os.getpid():
                        out.append((pid, open(f'/proc/{pid}/comm').read().strip()))
            except OSError:
                pass
    return sorted(set(out))


def set_interval(fc, msg_id, hz):
    us = 0 if hz == 0 else int(1e6 / hz)      # 0 = FC 기본 주기로 되돌림
    fc.m.mav.command_long_send(fc.sys, fc.comp, ML.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               msg_id, us, 0, 0, 0, 0, 0)
    ack = fc.recv({'COMMAND_ACK'}, 1.0,
                  cond=lambda m: fc._from_fc(m) and m.command == ML.MAV_CMD_SET_MESSAGE_INTERVAL)
    return 'ACK 없음' if ack is None else ML.enums['MAV_RESULT'][ack.result].name


def request_streams(fc, names):
    for name, mid, hz in STREAMS:
        if name in names:
            print(f'  {name} {hz}Hz 요청: {set_interval(fc, mid, hz)}')


def reset_streams(fc, names):
    for name, mid, _hz in STREAMS:
        if name in names:
            set_interval(fc, mid, 0)


SENSOR_ENUM_BY_NAME = {e.name: v for v, e in SENSOR_ENUM.items()}


def sys_status_report(ss, bs=None):
    if ss is None:
        print('  SYS_STATUS 를 받지 못함')
        return {}
    pr, en, he = (ss.onboard_control_sensors_present, ss.onboard_control_sensors_enabled,
                  ss.onboard_control_sensors_health)
    rep = {}
    print('  센서                         present  enabled  healthy')
    for label, name in (('광류 (OPTICAL_FLOW)', 'MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW'),
                        ('거리 (LASER_POSITION)', 'MAV_SYS_STATUS_SENSOR_LASER_POSITION'),
                        ('배터리 (BATTERY)', 'MAV_SYS_STATUS_SENSOR_BATTERY')):
        b = SENSOR_ENUM_BY_NAME[name]
        row = {'present': bool(pr & b), 'enabled': bool(en & b), 'healthy': bool(he & b)}
        rep[name] = row
        yn = lambda x: '예' if x else '아니오'   # noqa: E731
        print(f"  {label:<27}{yn(row['present']):>7}  {yn(row['enabled']):>7}  {yn(row['healthy']):>7}")
    bad = [e.name.replace('MAV_SYS_STATUS_SENSOR_', '') for v, e in sorted(SENSOR_ENUM.items())
           if v > 0 and not v & (v - 1) and pr & v and en & v and not he & v]
    rep['unhealthy'] = bad
    print(f"  present+enabled 인데 unhealthy: {', '.join(bad) or '없음'}")
    v = None if ss.voltage_battery in (0xFFFF, -1) else ss.voltage_battery / 1000.0
    i = None if ss.current_battery in (-1,) else ss.current_battery / 100.0
    rep.update(voltage_V=v, current_A=i, remaining_pct=ss.battery_remaining)
    print(f"  배터리 전압 (SYS_STATUS): {'-' if v is None else f'{v:.2f} V'}, "
          f"전류 {'-' if i is None else f'{i:.2f} A'}, 잔량 {ss.battery_remaining}%")
    if bs is not None:
        cells = [c for c in bs.voltages if c not in (0xFFFF, 0)]
        tot = sum(cells) / 1000.0 if cells else None
        rep['battery_status_V'] = tot
        print(f"  배터리 전압 (BATTERY_STATUS): {'-' if tot is None else f'{tot:.2f} V'}")
    return rep


def phase_reboot(fc, args, summary):
    hb = fc.connect(30)
    print(f'  연결됨: {fc.port} (sys {fc.sys}/{fc.comp}), 시동=DISARMED')
    fc.reboot()
    request_streams(fc, {'SYS_STATUS', 'BATTERY_STATUS'})
    end = fc.connected_at + args.listen
    left = end - time.time()
    print(f'\n== PreArm 수집을 위해 {left:.0f}초 대기 (PreArm은 약 30초마다 나옴) ==')
    last = {}
    while time.time() < end:
        msg = fc.recv(None, max(0.05, min(0.5, end - time.time())))
        if msg is not None and fc._from_fc(msg):
            last[msg.get_type()] = msg
    reset_streams(fc, {'SYS_STATUS', 'BATTERY_STATUS'})
    after = [s['text'] for s in fc.statustext[fc.mark:]]
    prearm = dedupe([t for t in after if re.search(r'prearm|config|arm:', t, re.I)])
    print('\n== 재부팅 후 STATUSTEXT 전부 ==')
    for t in after:
        print(f'  {t}')
    print('\n== 재부팅 후 PreArm / Config ==')
    for t in prearm or ['(없음)']:
        print(f'  {t}')
    print('\n== SYS_STATUS ==')
    summary['sys_status'] = sys_status_report(last.get('SYS_STATUS'), last.get('BATTERY_STATUS'))
    summary.update(statustext_after_reboot=after, prearm_config=prearm)


def phase_watch(fc, args, summary):
    fc.connect(30)
    print(f'  연결됨: {fc.port} (sys {fc.sys}/{fc.comp})')
    names = {n for n, _, _ in STREAMS}
    request_streams(fc, names)
    print(f'\n== {args.seconds:.0f}초 기록 시작 ==')
    print('   t(s) | DIST_SENSOR cm | RANGEFINDER m | flow qual | flow_rate x,y (rad/s) | OF hagl m')
    t0 = time.time()
    data = {n: [] for n in names}
    latest = {}
    next_print = 0.0
    while True:
        t = time.time() - t0
        if t >= args.seconds:
            break
        msg = fc.recv(None, max(0.05, min(0.1, args.seconds - t)))
        if msg is not None and fc._from_fc(msg) and msg.get_type() in data:
            mt = msg.get_type()
            t = time.time() - t0
            if mt == 'DISTANCE_SENSOR':
                row = {'t': t, 'cm': msg.current_distance, 'orient': msg.orientation,
                       'quality': getattr(msg, 'signal_quality', None)}
            elif mt == 'RANGEFINDER':
                row = {'t': t, 'm': msg.distance, 'voltage': msg.voltage}
            elif mt == 'OPTICAL_FLOW':
                row = {'t': t, 'quality': msg.quality, 'rx': getattr(msg, 'flow_rate_x', None),
                       'ry': getattr(msg, 'flow_rate_y', None), 'hagl': msg.ground_distance,
                       'fx': msg.flow_x, 'fy': msg.flow_y}
            else:
                row = {'t': t}
            data[mt].append(row)
            latest[mt] = msg
        if time.time() - t0 >= next_print:
            next_print += 0.5
            d = latest.get('DISTANCE_SENSOR')
            r = latest.get('RANGEFINDER')
            o = latest.get('OPTICAL_FLOW')
            dcm = '-' if d is None else f'{d.current_distance:5d}'
            bar = '' if d is None else '#' * min(40, int(d.current_distance / 5))
            rm = '-' if r is None else f'{r.distance:6.3f}'
            q = '-' if o is None else f'{o.quality:3d}'
            fr = '-' if o is None else f'{getattr(o, "flow_rate_x", 0):+.3f},{getattr(o, "flow_rate_y", 0):+.3f}'
            hg = '-' if o is None else f'{o.ground_distance:.3f}'
            print(f'  {time.time() - t0:5.1f} | {dcm:>14} | {rm:>13} | {q:>9} | {fr:>21} | {hg:>8}  {bar}')
    reset_streams(fc, names)

    print('\n== 요약 ==')
    dur = args.seconds
    for n in ('OPTICAL_FLOW', 'DISTANCE_SENSOR', 'RANGEFINDER', 'SYS_STATUS'):
        print(f'  {n:<16} {len(data[n]):4d}개 ({len(data[n]) / dur:.1f} Hz)')
    ds = [x['cm'] for x in data['DISTANCE_SENSOR']]
    rf = [x['m'] for x in data['RANGEFINDER']]
    fq = [x['quality'] for x in data['OPTICAL_FLOW']]
    if ds:
        print(f'  DISTANCE_SENSOR 거리: 최소 {min(ds)} cm, 최대 {max(ds)} cm, 변화폭 {max(ds) - min(ds)} cm '
              f'(orientation {data["DISTANCE_SENSOR"][-1]["orient"]})')
    if rf:
        print(f'  RANGEFINDER 거리: 최소 {min(rf):.3f} m, 최대 {max(rf):.3f} m')
    if fq:
        rx = [abs(x['rx'] or 0) for x in data['OPTICAL_FLOW']]
        ry = [abs(x['ry'] or 0) for x in data['OPTICAL_FLOW']]
        print(f'  광류 quality: 최소 {min(fq)}, 최대 {max(fq)}, 평균 {sum(fq) / len(fq):.0f} (0~255), '
              f'|flow_rate| 최대 x {max(rx):.3f}, y {max(ry):.3f} rad/s')
    print('\n== SYS_STATUS (기록 끝 시점) ==')
    summary['sys_status'] = sys_status_report(latest.get('SYS_STATUS'), latest.get('BATTERY_STATUS'))
    summary['samples'] = data
    summary['statustext'] = fc.statustext


def main():
    ap = argparse.ArgumentParser(description='FC 센서 확인')
    ap.add_argument('phase', choices=['reboot', 'watch'])
    ap.add_argument('--port', default=AP.DEFAULT_PORT_GLOB)
    ap.add_argument('--listen', type=float, default=45.0)
    ap.add_argument('--seconds', type=float, default=15.0)
    args = ap.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    sys.stdout = AP.Tee(os.path.join(HERE, f'sensor_{args.phase}_{stamp}.txt'))

    held = port_holders(args.port)
    if held:
        print(f'!!! FC 포트를 다른 프로그램이 잡고 있음: {held} -> QGC 등을 꺼 주세요')
        return 1
    fc = FC(args.port)
    summary = {'phase': args.phase, 'stamp': stamp}
    code = 0
    try:
        (phase_reboot if args.phase == 'reboot' else phase_watch)(fc, args, summary)
    except ArmedError as e:
        print(f'\n!!! {e}')
        code = 2
    except SystemExit as e:
        print(f'\n!!! 중단: {e}')
        code = 1
    finally:
        fc.close()
        with open(os.path.join(HERE, f'sensor_{args.phase}_{stamp}.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1, default=str)
        print('\n포트 닫음.')
        sys.stdout.flush()
    return code


if __name__ == '__main__':
    sys.exit(main())
