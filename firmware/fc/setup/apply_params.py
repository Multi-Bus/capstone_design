#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
N6 드론 FC(ArduPilot) 파라미터 일괄 설정.

순서: 연결 -> 펌웨어/보드 출력 -> 전체 백업
      -> 1단계(FRAME_CLASS/FRAME_TYPE) -> 재부팅
      -> 2단계(나머지) -> 재부팅
      -> (재부팅 뒤에야 생기는 파라미터가 있으면 다시 시도 -> 재부팅)
      -> 전체 다시 읽어서 최종 확인 -> 최종 파라미터 저장

안전: 이 스크립트가 FC에 보내는 것은 파라미터 읽기/쓰기, AUTOPILOT_VERSION·SYSTEM_TIME 요청,
      재부팅(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=1), GCS 하트비트뿐이다.
      ARM, 모터 테스트, 이륙 명령은 없다. FC가 시동(ARMED) 상태로 보이면 즉시 멈춘다.

값은 n6_base.param (공통) -> drones/drone<N>.param (기체별, 있으면) -> --set 순서로 합치고
뒤에 오는 값이 이긴다.

사용:
  .venv/bin/python apply_params.py --dry-run --drone 3              # 읽기만, 계획 표 출력
  .venv/bin/python apply_params.py --drone 3                         # 실제 적용
  .venv/bin/python apply_params.py --drone 3 --set BATT_VOLT_MULT=16.2   # 한 번만 다른 값
"""
import argparse
import glob
import json
import os
import re
import struct
import sys
import time
from datetime import datetime

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402

ML = mavutil.mavlink
HERE = os.path.dirname(os.path.abspath(__file__))
DRONE_DIR = os.path.join(HERE, 'drones')
DEFAULT_PORT_GLOB = '/dev/serial/by-id/usb-ArduPilot_DAKEFPVH743_*-if00'

STAGE1 = ('FRAME_CLASS', 'FRAME_TYPE')
NEVER_TOUCH = {'MOT_SPIN_ARM', 'MOT_SPIN_MIN'}
OR_BITMASK = {'ARMING_SKIPCHK', 'MAV2_OPTIONS'}          # 기존 값에 비트를 OR 해서 넣음
SCALED_ALIASES = {                                         # 새 이름 -> [(옛 이름, 배율)]
    'MAV_GCS_SYSID': [('SYSID_MYGCS', 1.0)],
    'RNGFND1_MIN': [('RNGFND1_MIN_CM', 100.0)],
    'RNGFND1_MAX': [('RNGFND1_MAX_CM', 100.0)],
    'RNGFND1_GNDCLR': [('RNGFND1_GNDCLEAR', 100.0)],
}
ARMING_CHECK_ALL = 1                                       # ARMING_CHECK 비트0 = 전체 검사
ARMING_CHECK_EVERY_BIT = (1 << 21) - 2                     # 비트1~20 (개별 검사 전부)
MAVOPT_TO_SERIALOPT = {2: 1024}                            # MAVn_OPTIONS 비트1 = SERIALn_OPTIONS 비트10 (포워딩 안 함)
REAL32 = ML.MAV_PARAM_TYPE_REAL32
FW_TYPES = {0: 'dev', 64: 'alpha', 128: 'beta', 192: 'rc', 255: 'official'}


# ---------------------------------------------------------------- 값 도우미
def f32(x):
    return struct.unpack('<f', struct.pack('<f', float(x)))[0]


def same(a, b):
    if a is None or b is None:
        return False
    a, b = f32(a), f32(b)
    return a == b or abs(a - b) <= 1e-6 * max(1.0, abs(b))


def fmt(v, is_int=False):
    if v is None:
        return '-'
    v = float(v)
    if is_int or (v.is_integer() and abs(v) < 1e9):
        return str(int(round(v)))
    v32 = f32(v)
    for p in range(1, 10):
        s = '%.*g' % (p, v32)
        if f32(float(s)) == v32:
            return s
    return repr(v32)


def pid(msg):
    s = msg.param_id
    if isinstance(s, bytes):
        s = s.decode('ascii', 'replace')
    return s.rstrip('\x00')


def has_mavn_params(params):
    return any(re.match(r'^MAV\d+_OPTIONS$', k) for k in params)


class Tee:
    def __init__(self, path):
        self.f = open(path, 'a', encoding='utf-8')
        self.out = sys.stdout

    def write(self, s):
        self.out.write(s)
        self.f.write(s)

    def flush(self):
        self.out.flush()
        self.f.flush()

    def isatty(self):
        return False


class ArmedError(RuntimeError):
    pass


# ---------------------------------------------------------------- FC 연결
class FC:
    def __init__(self, port_glob):
        self.port_glob = port_glob
        self.port = None
        self.m = None
        self.sys = None
        self.comp = None
        self.params = {}            # name -> (value, type)
        self.statustext = []        # 세션 전체
        self.mark = 0               # 마지막 (재)연결 시점의 statustext 위치
        self.connected_at = 0.0
        self.last_fc_hb = None
        self._last_hb_tx = 0.0

    # --- 연결 -------------------------------------------------------
    def find_port(self, timeout):
        t0 = time.time()
        while True:
            ports = sorted(glob.glob(self.port_glob))
            if ports:
                return ports[0]
            if time.time() - t0 >= timeout:
                return None
            time.sleep(0.3)

    def connect(self, timeout=30.0):
        t0 = time.time()
        port = self.find_port(timeout)
        if port is None:
            raise SystemExit(f'FC 포트를 찾지 못함: {self.port_glob}')
        last_err = None
        while True:
            try:
                self.m = mavutil.mavlink_connection(port, baud=115200, source_system=255,
                                                    source_component=190, autoreconnect=False)
                break
            except Exception as e:      # udev가 막 만든 장치는 잠깐 권한 오류가 날 수 있음
                last_err = e
                if time.time() - t0 > timeout:
                    msg = str(last_err)
                    if 'Permission' in msg or 'Errno 13' in msg:
                        raise SystemExit(f'포트 권한 오류: {msg}\n'
                                         '  -> sudo usermod -aG dialout $USER 후 로그아웃/로그인')
                    if 'busy' in msg.lower() or 'Errno 16' in msg:
                        raise SystemExit(f'포트 사용 중: {msg}\n  -> QGroundControl 등을 꺼 주세요')
                    raise SystemExit(f'포트 열기 실패: {msg}')
                time.sleep(0.5)
        self.port = port
        self.sys = self.comp = None
        hb = self.recv({'HEARTBEAT'}, max(5.0, timeout - (time.time() - t0)), cond=self._is_fc_hb)
        if hb is None:
            raise SystemExit('FC HEARTBEAT를 받지 못함')
        self.sys, self.comp = hb.get_srcSystem(), hb.get_srcComponent()
        self.last_fc_hb = hb
        if hb.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED:
            raise ArmedError('FC가 시동(ARMED) 상태입니다. 아무것도 하지 않고 중단합니다.')
        self.mark = len(self.statustext)
        self.connected_at = time.time()
        return hb

    def close(self):
        try:
            if self.m is not None:
                self.m.close()
        except Exception:
            pass
        self.m = None

    # --- 수신 -------------------------------------------------------
    def _is_fc_hb(self, msg):
        if msg.get_type() != 'HEARTBEAT' or msg.autopilot != ML.MAV_AUTOPILOT_ARDUPILOTMEGA:
            return False
        if msg.type in (ML.MAV_TYPE_GCS, ML.MAV_TYPE_ONBOARD_CONTROLLER):
            return False
        if self.sys is None:
            return msg.get_srcComponent() == ML.MAV_COMP_ID_AUTOPILOT1
        return (msg.get_srcSystem(), msg.get_srcComponent()) == (self.sys, self.comp)

    def _from_fc(self, msg):
        return (msg.get_srcSystem(), msg.get_srcComponent()) == (self.sys, self.comp)

    def _send_gcs_heartbeat(self):
        now = time.time()
        if now - self._last_hb_tx >= 1.0:
            self._last_hb_tx = now
            try:
                self.m.mav.heartbeat_send(ML.MAV_TYPE_GCS, ML.MAV_AUTOPILOT_INVALID, 0, 0,
                                          ML.MAV_STATE_ACTIVE)
            except Exception:
                pass

    def _observe(self, msg):
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode('utf-8', 'replace')
            text = text.rstrip('\x00').strip()
            src = f'{msg.get_srcSystem()}/{msg.get_srcComponent()}'
            self.statustext.append({'time': datetime.now().strftime('%H:%M:%S'), 'src': src,
                                    'severity': int(msg.severity), 'text': text})
            print(f'      [STATUSTEXT {src}] {text}')
        elif mtype == 'HEARTBEAT' and self.sys is not None and self._is_fc_hb(msg):
            self.last_fc_hb = msg
            if msg.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED:
                raise ArmedError('FC가 시동(ARMED) 상태가 됐습니다. 즉시 중단합니다.')

    def recv(self, types=None, timeout=1.0, cond=None):
        deadline = time.time() + timeout
        while True:
            self._send_gcs_heartbeat()
            left = deadline - time.time()
            if left <= 0:
                return None
            msg = self.m.recv_match(blocking=True, timeout=min(left, 0.25))
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == 'BAD_DATA':
                continue
            self._observe(msg)
            if types is not None and mtype not in types:
                continue
            if cond is not None and not cond(msg):
                continue
            return msg

    def wait(self, seconds):
        self.recv(None, seconds, cond=lambda m: False)

    # --- 파라미터 ---------------------------------------------------
    def read_param(self, name, retries=3, timeout=1.0):
        for _ in range(retries):
            self.m.mav.param_request_read_send(self.sys, self.comp, name.encode('ascii'), -1)
            msg = self.recv({'PARAM_VALUE'}, timeout,
                            cond=lambda m: self._from_fc(m) and pid(m) == name)
            if msg is not None:
                self.params[name] = (msg.param_value, msg.param_type)
                return self.params[name]
        return None

    def set_param(self, name, value, ptype, retries=3, timeout=1.5):
        got = None
        for _ in range(retries):
            self.m.mav.param_set_send(self.sys, self.comp, name.encode('ascii'), float(value), ptype)
            msg = self.recv({'PARAM_VALUE'}, timeout,
                            cond=lambda m: self._from_fc(m) and pid(m) == name)
            if msg is None:
                continue
            got = msg.param_value
            self.params[name] = (msg.param_value, msg.param_type)
            if same(got, value):
                return got, True
        r = self.read_param(name)
        if r is not None:
            return r[0], same(r[0], value)
        return got, False

    def fetch_all(self, attempts=4):
        for _attempt in range(attempts):
            got, count = {}, None
            self.m.mav.param_request_list_send(self.sys, self.comp)
            t0 = last = time.time()
            while True:
                msg = self.recv({'PARAM_VALUE'}, 0.5, cond=self._from_fc)
                now = time.time()
                if msg is not None:
                    last = now
                    count = msg.param_count
                    if msg.param_index < count:
                        got[msg.param_index] = (pid(msg), msg.param_value, msg.param_type)
                    if len(got) >= count:
                        break
                elif now - last > (3.0 if count else 6.0):
                    break
                if now - t0 > 180:
                    break
            if count is None:
                print('    파라미터 목록 응답 없음, 2초 뒤 다시 요청...')
                self.wait(2.0)
                continue
            for _ in range(5):
                missing = [i for i in range(count) if i not in got]
                if not missing:
                    break
                print(f'    빠진 {len(missing)}개 다시 요청')
                for i in missing:
                    self.m.mav.param_request_read_send(self.sys, self.comp, b'', i)
                    msg = self.recv({'PARAM_VALUE'}, 1.0,
                                    cond=lambda m, i=i: self._from_fc(m) and m.param_index == i)
                    if msg is not None:
                        got[i] = (pid(msg), msg.param_value, msg.param_type)
            missing = [i for i in range(count) if i not in got]
            if missing:
                print(f'    (경고) 끝내 받지 못한 파라미터 인덱스 {len(missing)}개: {missing[:20]}')
            self.params = {n: (v, t) for n, v, t in got.values()}
            return count, len(missing)
        raise SystemExit('파라미터 목록을 받지 못함')

    # --- 명령 -------------------------------------------------------
    def request_message(self, msg_id, msg_name, timeout=2.0):
        self.m.mav.command_long_send(self.sys, self.comp, ML.MAV_CMD_REQUEST_MESSAGE, 0,
                                     msg_id, 0, 0, 0, 0, 0, 0)
        return self.recv({msg_name}, timeout, cond=self._from_fc)

    def request_version(self):
        msg = self.request_message(ML.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 'AUTOPILOT_VERSION')
        if msg is None:
            self.m.mav.command_long_send(self.sys, self.comp,
                                         ML.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0,
                                         1, 0, 0, 0, 0, 0, 0)
            msg = self.recv({'AUTOPILOT_VERSION'}, 2.0, cond=self._from_fc)
        return msg

    def uptime_s(self):
        msg = self.request_message(ML.MAVLINK_MSG_ID_SYSTEM_TIME, 'SYSTEM_TIME')
        return None if msg is None else msg.time_boot_ms / 1000.0

    def reboot(self):
        if self.last_fc_hb is not None and self.last_fc_hb.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED:
            raise ArmedError('FC가 시동(ARMED) 상태라 재부팅하지 않고 중단합니다.')
        print('\n  플래시 저장 여유 2초...')
        self.wait(2.0)
        print('  재부팅 명령 전송 (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1)')
        ack = None
        for _ in range(3):
            try:
                self.m.mav.command_long_send(self.sys, self.comp,
                                             ML.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                             1, 0, 0, 0, 0, 0, 0)
                ack = self.recv({'COMMAND_ACK'}, 2.0,
                                cond=lambda m: self._from_fc(m)
                                and m.command == ML.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
            except ArmedError:
                raise
            except Exception:           # USB가 끊기면 읽기에서 예외가 날 수 있음 = 재부팅 시작
                break
            if ack is not None or not glob.glob(self.port_glob):
                break
        if ack is not None and ack.result != ML.MAV_RESULT_ACCEPTED:
            raise SystemExit(f'재부팅이 거부됨 (COMMAND_ACK result={ack.result})')
        self.close()
        t0 = time.time()
        while time.time() - t0 < 10 and glob.glob(self.port_glob):
            time.sleep(0.2)
        gone = not glob.glob(self.port_glob)
        print('  USB 장치 사라짐 -> 다시 나타나길 기다림' if gone
              else '  (참고) USB 장치가 사라지는 것은 못 봤음, 다시 연결 시도')
        time.sleep(1.0)
        self.connect(timeout=45)
        up = self.uptime_s()
        print(f'  다시 연결됨: {self.port} (sys {self.sys}/{self.comp}), '
              f'FC 부팅 후 {"?" if up is None else f"{up:.1f}"}초')
        if up is not None and up > 60:
            raise SystemExit('FC 가동 시간이 60초를 넘음 -> 재부팅이 안 된 것 같아 중단')


# ---------------------------------------------------------------- 파일
def read_param_file(path):
    items = []
    with open(path, encoding='utf-8') as f:
        for ln, line in enumerate(f, 1):
            s = line.split('#', 1)[0].strip()
            if not s:
                continue
            parts = [p for p in re.split(r'[,\s]+', s) if p]
            if len(parts) < 2:
                raise SystemExit(f'{path}:{ln} 형식 오류: {line.rstrip()}')
            items.append((parts[0].upper(), float(parts[1])))
    return items


def write_param_file(path, params, header):
    with open(path, 'w', encoding='utf-8') as f:
        for h in header:
            f.write(f'# {h}\n')
        for name in sorted(params):
            v, t = params[name]
            f.write(f'{name},{fmt(v, t != REAL32)}\n')


def load_items(param_file, drone=None, sets=()):
    """공통 값 파일 + drones/drone<N>.param(있으면) + --set 을 합친다. 뒤에 오는 값이 이긴다.
    반환: (items, 어디서 온 값인지 설명 줄 목록)"""
    items = read_param_file(param_file)
    notes = [f'공통 값: {param_file} ({len(items)}개)']

    def merge(name, value):
        nonlocal items
        items = [(a, b) for a, b in items if a != name] + [(name, value)]

    if drone:
        dp = os.path.join(DRONE_DIR, f'drone{drone}.param')
        if os.path.exists(dp):
            over = read_param_file(dp)
            for n, v in over:
                merge(n, v)
            notes.append(f'기체별 값: {dp} -> ' + (', '.join(f'{n}={fmt(v)}' for n, v in over) or '(비어 있음)'))
        else:
            notes.append(f'기체별 값 파일 없음: {dp}')
    for s in sets:
        if '=' not in s:
            raise ValueError(f'--set 형식은 NAME=VALUE: {s}')
        n, v = s.split('=', 1)
        try:
            value = float(v)
        except ValueError:
            raise ValueError(f'--set 값이 숫자가 아님: {s}')
        merge(n.strip().upper(), value)
        notes.append(f'--set {n.strip().upper()}={fmt(value)}')
    return items, notes


def unique_path(path):
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    return f'{root}_{datetime.now().strftime("%H%M%S")}{ext}'


# ---------------------------------------------------------------- 이름 해석
def resolve(fc, name, value, mavn_style):
    """파일의 (name, value)를 이 펌웨어에서 실제로 쓸 (이름, 현재값, 목표값)으로 바꾼다. 없으면 None."""
    def get(n):
        return fc.params.get(n) or fc.read_param(n, retries=2, timeout=0.8)

    def rec(used, cur, ptype, target, note=''):
        return {'file_name': name, 'file_value': value, 'name': used, 'prev': cur, 'ptype': ptype,
                'is_int': ptype != REAL32, 'target': target, 'new': None, 'status': None,
                'note': note, 'final': None, 'final_ok': None}

    r = get(name)
    if r is not None:
        cur, ptype = r
        if name in OR_BITMASK:
            target = int(round(cur)) | int(round(value))
            return rec(name, cur, ptype, target, f'기존 {fmt(cur, True)} OR {int(value)}')
        return rec(name, cur, ptype, value)

    if name == 'ARMING_SKIPCHK':
        r = get('ARMING_CHECK')
        if r is not None:
            cur = int(round(r[0]))
            base = ARMING_CHECK_EVERY_BIT if cur & ARMING_CHECK_ALL else cur
            target = base & ~int(round(value))
            note = f'ARMING_SKIPCHK 없음 -> ARMING_CHECK 에서 {int(value)} 비트 끔'
            if cur & ARMING_CHECK_ALL:
                note += ' (기존 1=전체 -> 개별 비트로 펼침)'
            return rec('ARMING_CHECK', r[0], r[1], target, note)

    if name == 'MAV2_OPTIONS' and not mavn_style:
        r = get('SERIAL1_OPTIONS')
        if r is not None:
            add = 0
            for bit, sbit in MAVOPT_TO_SERIALOPT.items():
                if int(value) & bit:
                    add |= sbit
            target = int(round(r[0])) | add
            return rec('SERIAL1_OPTIONS', r[0], r[1], target,
                       f'MAV2_OPTIONS 없음 -> SERIAL1_OPTIONS 기존 {fmt(r[0], True)} OR {add}')

    for alt, scale in SCALED_ALIASES.get(name, []):
        r = get(alt)
        if r is not None:
            target = value * scale
            return rec(alt, r[0], r[1], target,
                       f'{name} 없음 -> {alt}' + (f' (x{scale:g})' if scale != 1 else ''))
    return None


def print_row(r, final=False):
    new = r['new'] if r['new'] is not None else r['target']
    line = f"  {r['file_name']:<17}{fmt(r['prev'], r['is_int']):>10} -> {fmt(new, r['is_int']):>10}  [{r['status']}]"
    if final:
        line += f"  최종 {fmt(r['final'], r['is_int'])} {'OK' if r['final_ok'] else '불일치!'}"
    if r['name'] != r['file_name']:
        line += f"  ({r['name']} 사용)"
    if r['note'] and not final:
        line += f"  {r['note']}"
    print(line)


def apply_items(fc, items, results, mavn_style, dry_run, title):
    print(f'\n== {title} ==')
    print(f"  {'파라미터':<15}{'이전값':>9} -> {'새값':>9}  [상태]")
    deferred = []
    for name, value in items:
        r = resolve(fc, name, value, mavn_style)
        if r is None:
            deferred.append((name, value))
            print(f'  {name:<17}(지금은 없음' + (')' if dry_run else ' -> 재부팅 후 다시 시도)'))
            continue
        if same(r['prev'], r['target']):
            r['status'] = '같음'
            r['new'] = r['prev']
        elif dry_run:
            r['status'] = '변경예정'
        else:
            got, ok = fc.set_param(r['name'], r['target'], r['ptype'])
            r['new'] = got
            r['status'] = '변경' if ok else '실패'
        results[name] = r
        print_row(r)
    return deferred


# ---------------------------------------------------------------- 출력
def print_info(fc, hb, av):
    print(f'  포트: {fc.port}')
    try:
        tname = ML.enums['MAV_TYPE'][hb.type].name
    except Exception:
        tname = str(hb.type)
    armed = bool(hb.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f'  HEARTBEAT: sys {fc.sys}/{fc.comp}, 기체 {tname}, ArduPilot, mavlink v{hb.mavlink_version}, '
          f'시동={"ARMED" if armed else "DISARMED"}')
    m = re.search(r'usb-ArduPilot_([^_]+)_', fc.port)
    if m:
        print(f'  USB 장치 이름의 보드: {m.group(1)}')
    if av is None:
        print('  AUTOPILOT_VERSION: 응답 없음')
        return
    v = av.flight_sw_version
    fw = f'{(v >> 24) & 0xff}.{(v >> 16) & 0xff}.{(v >> 8) & 0xff} {FW_TYPES.get(v & 0xff, v & 0xff)}'
    try:
        git = bytes(av.flight_custom_version).rstrip(b'\x00').decode('ascii', 'replace')
    except Exception:
        git = str(av.flight_custom_version)
    print(f'  AUTOPILOT_VERSION: 펌웨어 {fw} (git {git}), board_version 0x{av.board_version:08x} '
          f'(보드 ID {av.board_version >> 16})')


def banner_info(statustext):
    fw = board = frame = None
    for s in statustext:
        t = s['text']
        if re.match(r'^Ardu\w+ V\d', t):
            fw = t
        elif re.match(r'^[A-Za-z0-9_\-]+ [0-9A-F]{8} [0-9A-F]{8} [0-9A-F]{8}$', t):
            board = t.split()[0]
        elif t.startswith('Frame:'):
            frame = t
    return fw, board, frame


def dedupe(texts):
    seen, out = set(), []
    for t in texts:
        if t not in seen:
            seen.add(t)
            out.append(t)
    return out


# ---------------------------------------------------------------- 메인
def run(fc, args, items, results, stamp, summary):
    print('== 연결 ==')
    hb = fc.connect(30)
    av = fc.request_version()
    print_info(fc, hb, av)

    print('\n== 전체 파라미터 읽기 (백업) ==')
    count, nmiss = fc.fetch_all()
    print(f'  {len(fc.params)}개 읽음 (FC 보고 {count}개)')
    fw, board, frame = banner_info(fc.statustext)
    print(f'  부팅 배너: 펌웨어 "{fw}", 보드 "{board}", {frame}')
    summary.update(firmware_banner=fw, board_banner=board, frame_banner=frame)
    shown = {}
    for n in [s.strip().upper() for s in args.show.split(',') if s.strip()]:
        v = fc.params.get(n)
        shown[n] = None if v is None else v[0]
        print(f'  현재 {n} = {"없음" if v is None else fmt(v[0], v[1] != REAL32)}')
    summary['shown'] = shown
    if not args.dry_run:
        path = unique_path(os.path.join(args.outdir, f'backup_{stamp}.param'))
        write_param_file(path, fc.params, [f'backup {stamp}', f'{fw} / {board}'])
        print(f'  백업 저장: {path}')
        summary['backup'] = path

    mavn = has_mavn_params(fc.params)
    stage1 = [(n, v) for n, v in items if n in STAGE1]
    stage2 = [(n, v) for n, v in items if n not in STAGE1]

    d1 = apply_items(fc, stage1, results, mavn, args.dry_run, '1단계: 프레임')
    if args.dry_run:
        d2 = apply_items(fc, stage2 + d1, results, mavn, True, '2단계 계획 (읽기만, 쓰지 않음)')
        summary['missing'] = [n for n, _ in d2]
        print(f'\n지금 없는 이름: {", ".join(summary["missing"]) or "없음"}')
        print('(MOT_* 등은 프레임 설정 + 재부팅 뒤에 생길 수 있음)')
        return

    if any(results.get(n, {}).get('status') == '실패' for n, _ in stage1) or d1:
        raise SystemExit('프레임 설정 실패 -> 중단')
    if any(results[n]['status'] == '변경' for n, _ in stage1):
        fc.reboot()
        fc.fetch_all()
        mavn = has_mavn_params(fc.params)
    else:
        print('  프레임 값이 이미 같아서 1단계 재부팅은 생략')

    d2 = apply_items(fc, stage2, results, mavn, False, '2단계: 나머지')
    fc.reboot()
    fc.fetch_all()
    mavn = has_mavn_params(fc.params)
    missing = d2
    if d2:
        d3 = apply_items(fc, d2, results, mavn, False, '재부팅 뒤 다시 시도 (보류 항목)')
        if any(results.get(n, {}).get('status') == '변경' for n, _ in d2):
            fc.reboot()
            fc.fetch_all()
        missing = d3

    left = args.listen - (time.time() - fc.connected_at)
    if left > 0:
        print(f'\n== PreArm 메시지 수집을 위해 {left:.0f}초 더 대기 (PreArm은 약 30초마다 나옴) ==')
        fc.wait(left)

    # 최종 확인 (마지막 재부팅 뒤 전체 목록 기준)
    for name, _ in items:
        r = results.get(name)
        if r is None:
            continue
        fv = fc.params.get(r['name'])
        r['final'] = None if fv is None else fv[0]
        r['final_ok'] = fv is not None and same(fv[0], r['target'])

    final_path = unique_path(os.path.join(
        args.outdir, f'drone{args.drone}_final_{datetime.now().strftime("%Y%m%d")}.param'))
    write_param_file(final_path, fc.params, [f'drone{args.drone} final {stamp}', f'{fw} / {board}'])
    summary['final_file'] = final_path

    print('\n' + '=' * 78)
    print('최종 결과 (이전값 -> 새값 [상태], 재부팅 후 다시 읽은 값)')
    print('=' * 78)
    for name, _ in items:
        if name in results:
            print_row(results[name], final=True)
    summary['missing'] = [n for n, _ in missing]
    bad = [n for n, _ in items if n in results and not results[n]['final_ok']]
    summary['mismatch'] = bad
    print(f'\n이름이 없어서 못 넣은 값: {", ".join(summary["missing"]) or "없음"}')
    print(f'최종 확인 불일치: {", ".join(bad) or "없음"}')
    print(f'최종 파라미터 저장: {final_path}')


def main():
    ap = argparse.ArgumentParser(description='N6 드론 FC 파라미터 일괄 설정')
    ap.add_argument('--port', default=DEFAULT_PORT_GLOB, help='포트 또는 glob')
    ap.add_argument('--param-file', default=os.path.join(HERE, 'n6_base.param'))
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='이번 실행에서만 값 추가/덮어쓰기 (예: BATT_VOLT_MULT=16.2)')
    ap.add_argument('--drone', help='기체 번호 (drones/drone<N>.param 을 읽고, 최종 파일 이름에 사용)')
    ap.add_argument('--dry-run', action='store_true', help='읽기만 하고 계획 표 출력')
    ap.add_argument('--listen', type=float, default=50.0,
                    help='마지막 재부팅 뒤 STATUSTEXT를 모으는 최소 시간(초)')
    ap.add_argument('--outdir', default=HERE)
    ap.add_argument('--show', default='SERVO_BLH_RVMASK,SERVO_BLH_AUTO,SERVO_BLH_MASK,'
                    'MOT_SPIN_ARM,MOT_SPIN_MIN', help='백업 직후 값을 출력할 파라미터 (쉼표 구분)')
    args = ap.parse_args()
    if not args.dry_run and not args.drone:
        ap.error('--drone <번호> 가 필요합니다')

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    tag = '_dryrun' if args.dry_run else ''
    sys.stdout = Tee(os.path.join(args.outdir, f'apply_log_{stamp}{tag}.txt'))

    try:
        items, sources = load_items(args.param_file, args.drone, args.set)
    except ValueError as e:
        ap.error(str(e))
    for line in sources:
        print(line)
    refused = [n for n, _ in items if n in NEVER_TOUCH]
    items = [(n, v) for n, v in items if n not in NEVER_TOUCH]
    if refused:
        print(f'(거부) 손대지 않기로 한 파라미터는 건너뜀: {", ".join(refused)}')
    if 'SERVO_BLH_RVMASK' not in [n for n, _ in items]:
        print('(참고) SERVO_BLH_RVMASK 가 drones/drone<N>.param 에도 --set 에도 없어서 이번에는 건드리지 않음')

    fc = FC(args.port)
    results, summary = {}, {'stamp': stamp, 'dry_run': args.dry_run, 'drone': args.drone,
                            'sources': sources}
    code = 0
    try:
        run(fc, args, items, results, stamp, summary)
    except ArmedError as e:
        print(f'\n!!! {e}')
        code = 2
    except SystemExit as e:
        print(f'\n!!! 중단: {e}')
        code = 1
    finally:
        fc.close()
        after = [s['text'] for s in fc.statustext[fc.mark:]]
        prearm = dedupe([s['text'] for s in fc.statustext
                         if re.search(r'prearm|config|arm:', s['text'], re.I)])
        print('\n== 마지막 (재)연결 이후 STATUSTEXT 전부 ==')
        for t in after:
            print(f'  {t}')
        print('\n== 세션 전체의 PreArm / Config 메시지 (중복 제거) ==')
        for t in prearm or ['(없음)']:
            print(f'  {t}')
        summary.update(results=list(results.values()), statustext=fc.statustext,
                       statustext_after_last_boot=after, prearm_config=prearm, exit_code=code)
        out = os.path.join(args.outdir, f'apply_result_{stamp}{tag}.json')
        with open(out, 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1, default=str)
        print(f'\n결과 JSON: {out}')
        sys.stdout.flush()
    return code


if __name__ == '__main__':
    sys.exit(main())
