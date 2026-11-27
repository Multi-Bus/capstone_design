"""비행 제어기 연결 모듈(tag/fcl.c)을 PC 에서 시험한다. 가짜 비행 제어기와 가짜 노트북은 pymavlink 로 만든다.

    ../tools/.venv/bin/python tag/test_fcl.py        (이 저장소의 tools/.venv 에 pymavlink 가 있다)

보드와 실제 비행 제어기 없이 확인하는 것: 메시지 중계가 바이트 그대로인지, 보드가 만드는 명령을 pymavlink 가
제대로 읽는지(속도, 기준, 무시 표시), 대본의 줄과 시각, 안전 규칙.
확인할 수 없는 것: 실제 ArduCopter 가 이 명령에 어떻게 움직이는지, UART 인터럽트(flight.c), BLE 의 손실.
"""
import ctypes
import math
import subprocess
import sys
import tempfile
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mav

HERE = Path(__file__).resolve().parent
FRAME_US = 20000
GUIDED, LAND, LOITER = 4, 9, 5
RUN = 1
FAILS = []
__test__ = False  # 스크립트로 실행한다 (python tag/test_fcl.py). pytest 가 test_* 함수를 모으지 않게 한다.


def check(name, got, want=True, tol=None):
    ok = abs(got - want) <= tol if tol is not None else got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f" (기대 {want})"))
    if not ok:
        FAILS.append(name)


def build():
    out = Path(tempfile.mkdtemp(prefix="fcl_")) / "libfcl.so"
    subprocess.run(["gcc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared", f"-I{HERE}",
                    "-o", str(out), str(HERE / "fcl.c"), str(HERE / "fcl_test_shim.c")], check=True)
    lib = ctypes.CDLL(str(out))
    lib.fcl_script_velocity.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint32,
                                        ctypes.POINTER(ctypes.c_int16), ctypes.POINTER(ctypes.c_int16),
                                        ctypes.POINTER(ctypes.c_uint16)]
    lib.fcl_script_velocity.restype = ctypes.c_bool
    return lib


class Bench:
    """보드 한 대 + 가짜 비행 제어기(sysid 1) + 가짜 노트북(sysid 250)"""

    def __init__(self, lib, tag=0):
        self.lib = lib
        lib.shim_init(tag, FRAME_US)
        self.fc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.gcs = mav.MAVLink(None, srcSystem=250, srcComponent=190)
        self.at_fc = mav.MAVLink(None)      # 비행 제어기 자리에서 받은 바이트를 푼다
        self.at_fc.robust_parsing = True
        self.at_gcs = mav.MAVLink(None)
        self.at_gcs.robust_parsing = True
        self.now = 1000
        self.mode, self.armed = GUIDED, True
        self.x = self.y = 0.0
        self.to_fc_raw = b""
        self.to_ble_raw = b""
        self.log = []                       # 보드가 기록하려고 낸 줄 (C, E)

    def fc_send(self, msg):
        data = msg.pack(self.fc)
        self.lib.fcl_from_fc(data, len(data), self.now)
        return data

    def gcs_send(self, msg):
        data = msg.pack(self.gcs)
        self.lib.fcl_from_ble(data, len(data), self.now)
        return data

    def heartbeat(self):
        base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
        self.fc_send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, base,
                                                   self.mode, mav.MAV_STATE_ACTIVE, 3))

    def position(self, vx=0.0, vy=0.0):
        self.fc_send(mav.MAVLink_local_position_ned_message(self.now, self.x, self.y, -1.0, vx, vy, 0.0))

    def take(self):
        """보드가 내보낸 것을 꺼낸다: (비행 제어기 쪽 메시지들, 노트북 쪽 메시지들)"""
        buf = ctypes.create_string_buffer(65536)
        n = self.lib.shim_take_fc(buf, len(buf))
        fc_raw = buf.raw[:n]
        n = self.lib.shim_take_ble(buf, len(buf))
        ble_raw = buf.raw[:n]
        self.to_fc_raw += fc_raw
        self.to_ble_raw += ble_raw
        return (self.at_fc.parse_buffer(fc_raw) or []), (self.at_gcs.parse_buffer(ble_raw) or [])

    def drain(self):
        buf = ctypes.create_string_buffer(200)
        while True:
            n = self.lib.fcl_take_line(buf, len(buf))
            if n == 0:
                return
            self.log.append(buf.raw[:n].decode().strip())

    def lines(self):
        """지금까지 쌓인 줄을 돌려주고 비운다"""
        self.drain()
        out, self.log = self.log, []
        return out

    def run(self, first, last, script=1, start=1500, mover=255, speed=30, limit=50, log_ok=True, feed=True,
            control=True, hook=None):
        """프레임 first..last 를 돌린다. 프레임마다 20 ms, 위치는 프레임마다, HEARTBEAT 는 1초마다.
        돌려주는 것: 프레임마다 (프레임, 비행 제어기 쪽으로 나간 메시지들)"""
        sent = []
        for frame in range(first, last + 1):
            self.now += 20
            if hook:
                hook(self, frame)
            if feed:
                if frame % 50 == 0:
                    self.heartbeat()
                self.position()
            if control:
                self.lib.shim_control(frame, script, RUN if script else 0, start, mover, speed, limit, int(log_ok), self.now)
            self.lib.fcl_tick(self.now)
            to_fc, _ = self.take()
            self.drain()                    # 펌웨어도 조용한 구간마다 줄을 꺼내 간다
            sent.append((frame, to_fc))
        return sent


def velocities(sent):
    return [(f, round(m.vx, 3), round(m.vy, 3)) for f, msgs in sent for m in msgs
            if m.get_type() == "SET_POSITION_TARGET_LOCAL_NED"]


def test_relay(lib):
    print("\n--- 1) 중계")
    b = Bench(lib)
    wanted = b""
    wanted += b.gcs_send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
    wanted += b.gcs_send(mav.MAVLink_command_long_message(1, 1, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0.5))
    wanted += b.gcs_send(mav.MAVLink_param_request_read_message(1, 1, b"FS_GCS_TIMEOUT", -1))
    b.take()
    check("노트북 -> 비행 제어기: 메시지 3개가 바이트 그대로", b.to_fc_raw == wanted, True)
    # 조각나서 들어와도, 사이에 쓰레기가 끼어도
    b = Bench(lib)
    one = mav.MAVLink_command_long_message(1, 1, mav.MAV_CMD_DO_SET_MODE, 0, 1, LAND, 0, 0, 0, 0, 0).pack(b.gcs)
    stream = b"\x00\x11" + one + b"\x7f" + one
    for i in range(0, len(stream), 7):
        chunk = stream[i:i + 7]
        lib.fcl_from_ble(chunk, len(chunk), b.now)
    b.take()
    check("7바이트씩 끊겨 오고 쓰레기가 끼어도 메시지 2개가 그대로", b.to_fc_raw == one + one, True)
    # MAVLink 1 도 넘긴다
    b = Bench(lib)
    v1 = mav.MAVLink(None, srcSystem=250, srcComponent=190)
    v1.WIRE_PROTOCOL_VERSION = "1.0"
    from pymavlink.dialects.v10 import ardupilotmega as mav1
    old = mav1.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3).pack(mav1.MAVLink(None, srcSystem=250, srcComponent=190))
    lib.fcl_from_ble(old, len(old), b.now)
    b.take()
    check("MAVLink 1 메시지도 그대로", b.to_fc_raw == old, True)

    b = Bench(lib)
    wanted = b""
    b.heartbeat()
    wanted_hb = b.to_ble_raw
    wanted += b.fc_send(mav.MAVLink_statustext_message(mav.MAV_SEVERITY_INFO, b"PreArm: test"))
    wanted += b.fc_send(mav.MAVLink_command_ack_message(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 250, 190))
    wanted += b.fc_send(mav.MAVLink_distance_sensor_message(b.now, 1, 800, 30, 0, 0, 25, 0))
    b.fc_send(mav.MAVLink_command_ack_message(511, 0, 0, 0, 251, 191))      # 보드가 보낸 명령의 답
    _, to_gcs = b.take()
    check("비행 제어기 -> 노트북: 종류", [m.get_type() for m in to_gcs],
          ["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK", "DISTANCE_SENSOR"])
    check("비행 제어기 -> 노트북: HEARTBEAT 뒤가 바이트 그대로", b.to_ble_raw.endswith(wanted), True)
    del wanted_hb
    # 위치 50 Hz, 자세 10 Hz 가 노트북 쪽으로는 5 Hz, 2 Hz
    count = {"LOCAL_POSITION_NED": 0, "ATTITUDE": 0}
    for i in range(500):
        b.now += 20
        b.position()
        if i % 5 == 0:
            b.fc_send(mav.MAVLink_attitude_message(b.now, 0.01, 0.02, 0.03, 0, 0, 0))
        _, to_gcs = b.take()
        for m in to_gcs:
            count[m.get_type()] += 1
    check("10초 동안 노트북으로 간 위치 (5 Hz)", count["LOCAL_POSITION_NED"], 50, tol=2)
    check("10초 동안 노트북으로 간 자세 (2 Hz)", count["ATTITUDE"], 20, tol=1)
    # 자리가 없으면 메시지를 통째로 버린다(반쪽을 넣지 않는다)
    b = Bench(lib)
    lib.shim_room(40)
    big = b.gcs_send(mav.MAVLink_command_long_message(1, 1, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0.5))
    small = b.gcs_send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
    lib.shim_room(65536)
    b.take()
    check(f"자리가 40바이트뿐일 때: 큰 메시지({len(big)}바이트)는 통째로 버리고 작은 것({len(small)}바이트)만 들어감",
          b.to_fc_raw == small, True)


def test_requests(lib):
    print("\n--- 2) 위치와 자세 요청")
    b = Bench(lib)
    b.heartbeat()
    lib.fcl_tick(b.now)
    to_fc, _ = b.take()
    req = {int(m.param1): m.param2 for m in to_fc if m.get_type() == "COMMAND_LONG" and m.command == 511}
    check("HEARTBEAT 를 보면 위치(32)와 자세(30)의 간격을 요청", req, {32: 20000.0, 30: 100000.0})
    check("보내는 쪽 번호 (sysid 251, compid 191)", {(m.get_srcSystem(), m.get_srcComponent()) for m in to_fc}, {(251, 191)})
    check("받는 쪽 번호 (비행 제어기 1/1)", {(m.target_system, m.target_component) for m in to_fc}, {(1, 1)})
    n = 0
    for _ in range(100):                       # 2초 동안 위치가 계속 오면 다시 요청하지 않는다
        b.now += 20
        b.position()
        b.fc_send(mav.MAVLink_attitude_message(b.now, 0, 0, 0, 0, 0, 0))
        if _ % 50 == 0:
            b.heartbeat()
        lib.fcl_tick(b.now)
        n += len(b.take()[0])
    check("값이 오는 동안 다시 요청한 횟수", n, 0)
    for _ in range(200):                       # 위치가 끊기면 2초마다 다시
        b.now += 20
        if _ % 50 == 0:
            b.heartbeat()
        lib.fcl_tick(b.now)
        n += sum(1 for m in b.take()[0] if int(m.param1) == 32)
    check("위치가 4초 끊긴 동안 다시 요청한 횟수", n, 2, tol=1)


def test_script(lib):
    print("\n--- 3) 대본 1 (앞 2 m, 원점, 오른쪽 2 m, 원점), 0.3 m/s")
    b = Bench(lib)
    b.heartbeat()
    sent = b.run(1300, 1499)
    check("시작 프레임 전에는 속도 명령 없음", velocities(sent), [])
    lines = b.lines()
    states = [line.split(",")[15] for line in lines if line.startswith("C,")]
    check("시작 2초 전부터 상태가 '준비됨'(1)", (states[0], states[-1]), ("0", "1"))
    move = math.ceil(200 / 30 * 50)            # 2 m 를 0.3 m/s 로: 334 프레임
    total = 4 * move + 4 * 250
    sent = b.run(1500, 1500 + total + 100)
    v = velocities(sent)
    check("첫 명령은 시작 프레임에, 앞으로 0.3 m/s", v[0], (1500, 0.3, 0.0))
    msg = next(m for _, msgs in sent for m in msgs if m.get_type() == "SET_POSITION_TARGET_LOCAL_NED")
    check("기준 = 기체 기준(MAV_FRAME_BODY_NED 8)", msg.coordinate_frame, 8)
    check("무시 표시 = 0x5C7 (속도와 회전 속도만 씀)", msg.type_mask, 0x5C7)
    check("vz, 회전 속도, 위치 칸은 0", (msg.vz, msg.yaw_rate, msg.x, msg.y, msg.z), (0.0, 0.0, 0.0, 0.0, 0.0))
    changes = [(f, vx, vy) for i, (f, vx, vy) in enumerate(v) if i == 0 or (vx, vy) != v[i - 1][1:]]
    want, at = [], 1500
    for vx, vy in ((0.3, 0.0), (0.0, 0.0), (-0.3, 0.0), (0.0, 0.0), (0.0, 0.3), (0.0, 0.0), (0.0, -0.3), (0.0, 0.0)):
        want.append((at, vx, vy))
        at += move if (vx, vy) != (0.0, 0.0) else 250
    check("속도가 바뀌는 프레임과 값", changes, want)
    gaps = [b_[0] - a_[0] for a_, b_ in zip(v, v[1:]) if b_[0] <= 1500 + total - 1]
    check("명령 사이 간격은 5 프레임 이하 (10 Hz)", max(gaps) <= 5, True)
    tail = [x for x in v if x[0] >= 1500 + total]
    check("끝난 뒤 속도 0 을 5번 보내고 그침", (len(tail), set(x[1:] for x in tail)), (5, {(0.0, 0.0)}))
    lines = b.lines()
    events = [line.split(",")[2] for line in lines if line.startswith("E,")]
    check("사건: start, done", [e for e in events if e in ("start", "done")], ["start", "done"])
    c = [line.split(",") for line in lines if line.startswith("C,")]
    check("C 줄: 칸 수 20", {len(x) for x in c}, {20})
    row = next(x for x in c if x[1] == "1600")
    check("C 줄(프레임 1600): 상태 2, 줄 0, 명령 300 mm/s", (row[15], row[16], row[17], row[18]), ("2", "0", "300", "0"))

    print("\n--- 4) 한 대만 움직이는 대본, 속도 한도")
    b = Bench(lib, tag=2)
    b.heartbeat()
    v = velocities(b.run(1500, 1700, script=2, mover=4, speed=70, limit=100))
    check("움직이는 기체가 아니면 속도 0 만 보냄(제자리)", set(x[1:] for x in v), {(0.0, 0.0)})
    b = Bench(lib, tag=4)
    b.heartbeat()
    v = velocities(b.run(1500, 1600, script=2, mover=4, speed=70, limit=100))
    check("움직이는 기체는 0.7 m/s", v[0][1:], (0.7, 0.0))
    b = Bench(lib)
    b.heartbeat()
    v = velocities(b.run(1500, 1600, script=2, speed=70, limit=50))
    check("Poll 의 한도(0.5)를 넘지 않음", v[0][1:], (0.5, 0.0))
    b = Bench(lib)
    b.heartbeat()
    v = velocities(b.run(1500, 1600, script=2, speed=200, limit=250))
    check("펌웨어의 절대 한도(1.0)를 넘지 않음", v[0][1:], (1.0, 0.0))


def test_safety(lib):
    print("\n--- 5) 안전")

    def started(**kw):
        b = Bench(lib)
        b.heartbeat()
        b.run(1400, 1499, **kw)
        return b

    b = Bench(lib)
    b.mode = LOITER
    b.heartbeat()
    v = velocities(b.run(1400, 1700))
    check("GUIDED 가 아니면 시작하지 않음", v, [])
    check("  사건 skip_notready", any(",skip_notready," in line for line in b.lines()), True)
    b.mode = GUIDED
    b.heartbeat()
    check("  나중에 GUIDED 가 돼도 그 대본에는 끼지 않음", velocities(b.run(1701, 1900)), [])

    b = Bench(lib)
    b.armed = False
    b.heartbeat()
    check("시동이 아니면 시작하지 않음", velocities(b.run(1400, 1700)), [])

    b = Bench(lib)
    b.heartbeat()
    check("기록이 안 되면 시작하지 않음", velocities(b.run(1400, 1700, log_ok=False)), [])

    b = Bench(lib)
    b.heartbeat()
    check("위치 값이 없으면 시작하지 않음", velocities(b.run(1400, 1700, feed=False)), [])

    b = Bench(lib)
    b.heartbeat()
    check("시작 프레임을 지나서 처음 들으면 끼지 않음", velocities(b.run(1600, 1800)), [])

    # 조종기가 모드를 바꿈
    b = started()
    def to_loiter(bench, frame):
        if frame == 1600:
            bench.mode = LOITER
            bench.heartbeat()
    sent = b.run(1500, 1800, hook=to_loiter)
    v = velocities(sent)
    check("모드가 바뀐 프레임(1600) 뒤로는 속도 명령 없음", max(f for f, _, _ in v) < 1600, True)
    others = [m.get_type() for f, msgs in sent if f >= 1600 for m in msgs
              if not (m.get_type() == "COMMAND_LONG" and m.command == 511)]     # 값 요청(자세가 안 와서)은 뺀다
    check("  그 뒤로 비행 제어기에 속도·모드 명령을 보내지 않음", others, [])
    check("  사건 abort_mode", any(",abort_mode,5," in line for line in b.lines()), True)

    # 구역 이탈
    b = started()
    def drift(bench, frame):
        if frame >= 1550:
            bench.x += 0.05
    sent = b.run(1500, 1800, hook=drift)
    lands = [(f, m.param2) for f, msgs in sent for m in msgs if m.get_type() == "COMMAND_LONG" and m.command == 176]
    check("구역(2 m + 1.5 m)을 벗어나면 LAND 모드 명령 3번", [p for _, p in lands], [9.0, 9.0, 9.0])
    first = lands[0][0]
    check("  벗어난 프레임 근처에서 (3.5 m / 0.05 m = 70 프레임 뒤)", first, 1550 + 70, tol=3)
    check("  그 뒤로 속도 명령 없음", max(f for f, _, _ in velocities(sent)) < first, True)
    check("  사건 abort_zone, land_sent", [e for e in (line.split(",")[2] for line in b.lines() if line.startswith("E,"))
                                          if e in ("abort_zone", "land_sent")], ["abort_zone", "land_sent"])

    # 기록이 멈춤
    b = started()
    sent = b.run(1500, 1560)
    sent += b.run(1561, 1800, log_ok=False)
    lands = [f for f, msgs in sent for m in msgs if m.get_type() == "COMMAND_LONG" and m.command == 176]
    check("기록이 2초 넘게 안 되면 LAND 명령 3번", len(lands), 3)
    check("  기록이 멈춘 뒤 2초쯤에 (프레임 1661)", lands[0], 1661, tol=3)

    # 기록이 잠깐(1초) 멈췄다 돌아오면 계속
    b = started()
    sent = b.run(1500, 1560) + b.run(1561, 1610, log_ok=False) + b.run(1611, 1700)
    check("기록이 1초 멈췄다 돌아오면 LAND 를 보내지 않음",
          sum(1 for _, msgs in sent for m in msgs if m.get_type() == "COMMAND_LONG" and m.command == 176), 0)

    # 앵커 1 의 Poll 이 끊김
    b = started()
    sent = b.run(1500, 1560) + b.run(1561, 1700, control=False)
    v = velocities(sent)
    tail = [x for x in v if x[0] > 1560]
    check("Poll 이 0.5초 끊기면 속도 0 을 5번 보내고 그침", (len(tail), set(x[1:] for x in tail)), (5, {(0.0, 0.0)}))
    check("  끊긴 뒤 0.5초쯤에 (프레임 1586)", tail[0][0], 1586, tol=3)
    check("  다시 들려도 이어서 하지 않음", velocities(b.run(1701, 1800)), [])

    # 비행 제어기의 위치가 끊김
    b = started()
    sent = b.run(1500, 1560)
    def only_heartbeat(bench, frame):
        if frame % 50 == 0:
            bench.heartbeat()
    sent = b.run(1561, 1700, feed=False, hook=only_heartbeat)
    tail = velocities(sent)
    check("위치가 0.5초 끊기면 속도 0 으로 멈춤", (set(x[1:] for x in tail if x[0] > 1590)), {(0.0, 0.0)})

    # 시동이 꺼짐
    b = started()
    def disarm(bench, frame):
        if frame == 1600:
            bench.armed = False
            bench.heartbeat()
    sent = b.run(1500, 1700, hook=disarm)
    check("시동이 꺼지면 더 보내지 않음", max(f for f, _, _ in velocities(sent)) < 1600, True)

    # 앵커 1 을 다시 켬 (프레임 번호가 처음부터): 새로 시작할 수 있다
    b = started()
    b.run(1500, 1600)
    sent = b.run(1, 60, start=50)
    check("앵커 1 을 다시 켜면(프레임 번호가 뒤로) 속도 0 을 보내고 새 대본을 기다림",
          set(x[1:] for x in velocities(sent) if x[0] < 50), {(0.0, 0.0)})
    check("  새 시작 프레임(50)에 다시 시작", [x for x in velocities(sent) if x[0] >= 50][0], (50, 0.3, 0.0))

    # 대본 없음
    b = Bench(lib)
    b.heartbeat()
    check("대본 번호 0 이면 아무 속도 명령도 없음", velocities(b.run(1400, 1700, script=0)), [])


def test_script_table(lib):
    print("\n--- 6) 대본 표 (tools/script_table.py 와 같은 값인지)")
    sys.path.insert(0, str(HERE.parent / "tools"))
    import script_table
    bad = 0
    for script in (1, 2, 3, 4):
        for speed in (30, 50, 70, 100):
            rows = script_table.table(script, speed, FRAME_US)
            step = 0
            for row, frames, vx, vy in rows:
                for s in (step, step + frames - 1):
                    a, b_, r = ctypes.c_int16(), ctypes.c_int16(), ctypes.c_uint16()
                    ok = lib.fcl_script_velocity(script, speed, FRAME_US, s, ctypes.byref(a), ctypes.byref(b_), ctypes.byref(r))
                    bad += not ok or (a.value, b_.value, r.value) != (vx, vy, row)
                step += frames
            a, b_, r = ctypes.c_int16(), ctypes.c_int16(), ctypes.c_uint16()
            bad += bool(lib.fcl_script_velocity(script, speed, FRAME_US, step, ctypes.byref(a), ctypes.byref(b_), ctypes.byref(r)))
            bad += lib.fcl_script_extent_cm(script) != script_table.extent_cm(script)
    check("대본 4개 x 속도 4개: 파이썬 표와 펌웨어가 다른 곳", bad, 0)


def test_linkfs(lib):
    print("\n--- 7) 링크 끊김 강제 DISARM (n6_bridge link_failsafe.h 와 같은 조건)")

    def play(mode, gcs_every_ms, seconds=20):
        """노트북 HEARTBEAT 를 처음 한 번(gcs_every_ms=0) 또는 주기마다, FC HEARTBEAT 는 1초마다 보내며 100 ms 씩 돌린다.
        돌려주는 것: (보낸 시각 ms(시작 기준), 메시지) 강제 DISARM 목록, 기록 줄"""
        b = Bench(lib)
        b.mode = mode
        t0 = b.now
        gcs = lambda: b.gcs_send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
        gcs()
        cuts = []
        for step in range(seconds * 10 + 1):
            b.now = t0 + step * 100
            if gcs_every_ms and step and (step * 100) % gcs_every_ms == 0:
                gcs()
            if step % 10 == 0:
                b.heartbeat()
            lib.fcl_tick(b.now)
            to_fc, _ = b.take()
            cuts += [(b.now - t0, m) for m in to_fc
                     if m.get_type() == "COMMAND_LONG" and m.command == mav.MAV_CMD_COMPONENT_ARM_DISARM]
        return cuts, b.lines()

    cuts, lines = play(LAND, 0)
    check("LAND + 노트북 끊김: 첫 강제 DISARM 시각 ms (LAND 9초)", cuts[0][0] if cuts else -1, 9000)
    m = cuts[0][1] if cuts else None
    check("명령 값 param1=0, param2=21196, 대상 1/1, 보낸 쪽 250/191",
          m is not None and (m.param1, m.param2, m.target_system, m.target_component,
                             m.get_srcSystem(), m.get_srcComponent()) == (0, 21196, 1, 1, 250, 191))
    check("되풀이 간격 1초 이상", all(b_[0] - a[0] >= 1000 for a, b_ in zip(cuts, cuts[1:])))
    check("E,linkfs_cut 줄", any(",linkfs_cut,1,9" in line for line in lines))
    cuts, _ = play(GUIDED, 0)
    check("GUIDED + 노트북 끊김: 강제 DISARM 수", len(cuts), 0)
    cuts, _ = play(LAND, 200)
    check("LAND + 노트북 하트비트 계속: 강제 DISARM 수", len(cuts), 0)


def main():
    lib = build()
    test_relay(lib)
    test_requests(lib)
    test_script(lib)
    test_safety(lib)
    test_script_table(lib)
    test_linkfs(lib)
    print(f"\n{'모두 통과' if not FAILS else '실패 ' + str(len(FAILS)) + '개: ' + ', '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
