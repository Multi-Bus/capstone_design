"""n6_fly.py 오프라인 시나리오 시험: 가짜 BLE 링크 + 가짜 ArduCopter."""
import argparse
import asyncio
import importlib.util
import math
import random
import sys
import time
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as m

SPEC = importlib.util.spec_from_file_location("n6_fly", Path(__file__).resolve().parents[1] / "n6_fly.py")
n6 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(n6)

OUT = Path(__file__).resolve().parent / "sim_out"
OUT.mkdir(exist_ok=True)


class Sink:
    def __init__(self):
        self.b = bytearray()

    def write(self, x):
        self.b += x


class FakeFC:
    """ArduCopter 흉내 (필요한 만큼만)."""

    def __init__(self, params=None, reject_arm=False, drop_param_once=(), hb_drop=0.0, baro_dip=0.0):
        self.hb_drop = hb_drop
        self.baro_dip = baro_dip
        self.drop_once = set(drop_param_once)
        self.rx = m.MAVLink(None, 1, 1)
        self.rx.robust_parsing = True
        self.sink = Sink()
        self.tx = m.MAVLink(self.sink, 1, 1)
        self.link = None
        self.mode = 0
        self.armed = False
        self.z = 0.0
        self.x = self.y = 0.0
        self.xy_goal = None
        self.pos_targets = []  # 받은 SET_POSITION_TARGET_LOCAL_NED (frame, mask, x, y, z, yaw)
        self.target = None
        self.yaw = 40.0        # 도
        self.yaw_goal = None
        self.yaw_rate = 0.0
        self.land_t = None
        self.gcs_last = 0.0
        self.gcs_seen = False
        self.intervals = {}
        self.streams_on = True
        self.log = []          # (t, type, detail)
        self.bad = 0
        self.params = params or {"MAV_GCS_SYSID": 250, "FS_GCS_ENABLE": 5, "FS_GCS_TIMEOUT": 3}
        self.reject_arm = reject_arm
        self.alive = True      # False = NUCLEO 리셋 흉내 (양방향 무응답)

    # GCS -> FC
    def feed(self, data):
        if not self.alive:
            return
        for msg in self.rx.parse_buffer(data) or []:
            self.on_msg(msg)

    def out(self, msg):
        self.tx.send(msg)
        data = bytes(self.sink.b)
        self.sink.b.clear()
        if self.link and self.link.connected and self.alive:
            i = 0
            while i < len(data):  # 알림 조각 크기 무작위
                n = random.randint(5, 40)
                self.link.on_bytes(data[i:i + n])
                i += n

    def ack(self, cmd, res):
        self.out(self.tx.command_ack_encode(cmd, res, 0, 0, 250, 190))

    def on_msg(self, msg):
        t = msg.get_type()
        now = time.monotonic()
        if t == "BAD_DATA":
            self.bad += 1
            return
        if t == "HEARTBEAT":
            assert msg.get_srcSystem() == 250 and msg.get_srcComponent() == 190
            self.gcs_last, self.gcs_seen = now, True
            self.log.append((now, "HB", None))
            return
        if t == "REQUEST_DATA_STREAM":
            self.log.append((now, "RDS", (msg.req_stream_id, msg.req_message_rate, msg.start_stop)))
            self.streams_on = False
            for k in (32, 1, 132, 30):  # ArduPilot: 스트림 바꾸면 그 스트림 메시지 간격 덮어씀
                self.intervals[k] = 0
            return
        if t == "SET_GPS_GLOBAL_ORIGIN":
            self.log.append((now, "ORIGIN", (msg.latitude, msg.longitude)))
            self.out(self.tx.gps_global_origin_encode(msg.latitude, msg.longitude, msg.altitude))
            return
        if t == "PARAM_REQUEST_READ":
            pid = msg.param_id
            if pid in self.drop_once:
                self.drop_once.discard(pid)
                return
            if pid in self.params:
                self.out(self.tx.param_value_encode(pid.encode(), float(self.params[pid]), 2, 1000, 0))
            return
        if t == "SET_POSITION_TARGET_LOCAL_NED":
            self.pos_targets.append((msg.coordinate_frame, msg.type_mask, msg.x, msg.y, msg.z, msg.yaw))
            if self.armed and self.mode == 4 and self.z < -0.05:
                self.xy_goal = (msg.x, msg.y)
            return
        if t != "COMMAND_LONG":
            self.log.append((now, t, None))
            return
        c = msg.command
        p = (msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7)
        self.log.append((now, "CMD", (c, p)))
        if c == m.MAV_CMD_SET_MESSAGE_INTERVAL:
            self.intervals[int(p[0])] = p[1]
            self.ack(c, 0)
        elif c == m.MAV_CMD_DO_SET_MODE:
            self.mode = int(p[1])
            if self.mode == 9:
                self.target = None
            self.ack(c, 0)
        elif c == m.MAV_CMD_COMPONENT_ARM_DISARM:
            if p[0] == 1:
                if self.reject_arm:
                    self.out(self.tx.statustext_encode(3, b"PreArm: Need Position Estimate"))
                    self.ack(c, 4)
                else:
                    self.armed = True
                    self.ack(c, 0)
            else:
                if p[1] == 21196 or self.z >= -0.01:
                    self.armed = False
                    self.target = None
                    self.ack(c, 0)
                else:
                    self.ack(c, 4)
        elif c == m.MAV_CMD_CONDITION_YAW:
            if self.armed and self.mode == 4 and self.z < -0.05:
                d = p[0] if p[2] >= 0 else -p[0]
                self.yaw_goal = (self.yaw + d) % 360.0 if p[3] else p[0]
                self.yaw_rate = p[1]
                self.ack(c, 0)
            else:
                self.ack(c, 4)
        elif c == m.MAV_CMD_NAV_TAKEOFF:
            if self.armed and self.mode == 4 and self.z >= -0.01:
                self.target = p[6]
                self.ack(c, 0)
            else:
                self.ack(c, 4)
        else:
            self.ack(c, 3)

    async def run(self):
        t_hb = t_lpn = t_ds = t_ss = t_att = 0.0
        dt = 0.02
        while True:
            await asyncio.sleep(dt)
            now = time.monotonic()
            # 물리
            if self.armed and self.mode == 4 and self.target is not None:
                self.z = max(self.z - 1.0 * dt, -self.target)
            if self.armed and self.mode == 9:
                self.z = min(self.z + 0.6 * dt, 0.0)
                if self.z >= 0.0:
                    if self.land_t is None:
                        self.land_t = now
                    elif now - self.land_t > 0.8:
                        self.armed = False
                        self.land_t = None
                        self.out(self.tx.statustext_encode(6, b"Land complete, disarming"))
            if self.xy_goal is not None and self.armed and self.mode == 4:
                dx, dy = self.xy_goal[0] - self.x, self.xy_goal[1] - self.y
                d = math.hypot(dx, dy)
                stepv = 0.5 * dt
                if d <= stepv:
                    self.x, self.y = self.xy_goal
                else:
                    self.x += dx / d * stepv
                    self.y += dy / d * stepv
            if self.yaw_goal is not None:
                err = (self.yaw_goal - self.yaw + 180.0) % 360.0 - 180.0
                stepv = self.yaw_rate * dt
                if abs(err) <= stepv:
                    self.yaw, self.yaw_goal = self.yaw_goal, None
                else:
                    self.yaw = (self.yaw + math.copysign(stepv, err)) % 360.0
            # GCS 페일세이프
            if self.gcs_seen and self.armed and self.mode != 9 and now - self.gcs_last > 3.0:
                self.mode = 9
                self.target = None
                self.log.append((now, "FAILSAFE", None))
                self.out(self.tx.statustext_encode(2, b"GCS Failsafe"))
            # 송신
            if now - t_hb >= 1.0:
                t_hb = now
                if random.random() >= self.hb_drop:
                    self.out(self.tx.heartbeat_encode(2, 3, 1 | (128 if self.armed else 0), self.mode, 4))
            if self.streams_on and now - t_att >= 0.05:  # 기본 스트림 흉내 (정지 전까지 소음)
                t_att = now
                self.out(self.tx.attitude_encode(0, 0, 0, 0, 0, 0, 0))
            iv = self.intervals
            if iv.get(30, 0) > 0 and now - t_att >= iv[30] / 1e6:
                t_att = now
                yaw = math.radians(self.yaw + random.gauss(0, 0.5))
                self.out(self.tx.attitude_encode(0, math.radians(random.gauss(0, 0.3)),
                                                 math.radians(random.gauss(1.0, 0.3)),
                                                 (yaw + math.pi) % (2 * math.pi) - math.pi, 0, 0, 0))
            if iv.get(32, 0) > 0 and now - t_lpn >= iv[32] / 1e6:
                t_lpn = now
                dip = self.baro_dip if (self.armed and self.z > -0.05) else 0.0   # 바닥에서 모터 바람 -> EKF 높이 떨어짐
                self.out(self.tx.local_position_ned_encode(0, self.x, self.y, self.z + dip, 0, 0, 0))
            if iv.get(132, 0) > 0 and now - t_ds >= iv[132] / 1e6:
                t_ds = now
                self.out(self.tx.distance_sensor_encode(0, 1, 800, int(-self.z * 100), 10, 1, 25, 0))
            if iv.get(1, 0) > 0 and now - t_ss >= iv[1] / 1e6:
                t_ss = now
                self.out(self.tx.sys_status_encode(0, 0, 0, 0, 11900, -1, -1, 0, 0, 0, 0, 0, 0))

    def cmds(self, since=0.0):
        return [(t, d) for (t, k, d) in self.log if k == "CMD" and t >= since]

    def modes_sent(self, since=0.0):
        return [int(d[1][1]) for (t, d) in self.cmds(since) if d[0] == m.MAV_CMD_DO_SET_MODE]


class FakeLink:
    def __init__(self, fc):
        self.fc = fc
        fc.link = self
        self.connected = False
        self.on_bytes = lambda b: None
        self.on_state = lambda up: None
        self._lock = asyncio.Lock()

    async def run(self):
        await asyncio.sleep(0.2)
        self.up()
        await asyncio.Event().wait()

    def up(self):
        self.connected = True
        self.on_state(True)

    def down(self):
        self.connected = False
        self.on_state(False)

    async def write(self, data):
        async with self._lock:
            if not self.connected:
                return False
            for i in range(0, len(data), 20):
                await asyncio.sleep(0.003)
                if not self.connected:
                    return False
                self.fc.feed(data[i:i + 20])
            return True

    async def close(self):
        pass


def make_args(**kw):
    a = argparse.Namespace(name="SIM", alt=0.8, hover=1.0, no_arm=False, scan_timeout=1, log_dir=str(OUT))
    for k, v in kw.items():
        setattr(a, k, v)
    return a


class Scenario:
    def __init__(self, name, fc=None, **argkw):
        self.name = name
        self.fc = fc or FakeFC()
        self.link = FakeLink(self.fc)
        self.console = n6.Console(OUT / f"{name}.log")
        self.app = n6.N6Fly(make_args(**argkw), self.console, self.link)
        self.q = asyncio.Queue()

    async def __aenter__(self):
        self.fc_task = asyncio.get_running_loop().create_task(self.fc.run())
        self.app_task = asyncio.get_running_loop().create_task(self.app.run(self.q))
        await self.wait(lambda: self.app.s.uplink_ok and "FS_GCS_ENABLE" in self.app.params and self.app.s.last_hb > 0 and self.app.fs_checked, 12)
        await asyncio.sleep(0.5)
        return self

    async def __aexit__(self, *exc):
        if not self.app_task.done():
            await self.app.request_quit("sim")
            await asyncio.wait_for(self.app_task, 10)
        self.fc_task.cancel()
        self.console.close()

    async def key(self, k, wait=0.05):
        self.q.put_nowait(k)
        await asyncio.sleep(wait)

    async def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            await asyncio.sleep(0.02)
        raise AssertionError(f"[{self.name}] 시간 초과: {cond}")

    def text(self):
        return (OUT / f"{self.name}.log").read_text(encoding="utf-8")


RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  — {detail}" if detail else ""), flush=True)


async def s1_normal():
    async with Scenario("s1_normal") as sc:
        fc = sc.fc
        # 설정 순서: 스트림 정지가 SET_MESSAGE_INTERVAL 보다 먼저
        rds_t = min(t for (t, k, d) in fc.log if k == "RDS")
        smi = [(t, d) for (t, d) in fc.cmds() if d[0] == m.MAV_CMD_SET_MESSAGE_INTERVAL]
        check("s1 스트림 정지 → 간격 설정 순서", all(t > rds_t for t, _ in smi) and len(smi) == 4)
        check("s1 메시지 간격 값", fc.intervals.get(32) == 200000 and fc.intervals.get(132) == 200000
              and fc.intervals.get(1) == 1000000 and fc.intervals.get(30) == 100000, str(fc.intervals))
        check("s1 상태줄 자세 표시", "피치 +1." in sc.app._status_text(),
              sc.app._status_text())
        rds = [d for (t, k, d) in fc.log if k == "RDS"][0]
        check("s1 REQUEST_DATA_STREAM(ALL,0,stop)", rds == (0, 0, 0), str(rds))
        # 하트비트 5 Hz
        t0 = time.monotonic()
        n0 = sum(1 for (_, k, _) in fc.log if k == "HB")
        await asyncio.sleep(2.0)
        n1 = sum(1 for (_, k, _) in fc.log if k == "HB")
        rate = (n1 - n0) / (time.monotonic() - t0)
        check("s1 하트비트 ~5 Hz", 4.5 <= rate <= 5.5, f"{rate:.2f} Hz")
        check("s1 높이 텔레메트리", sc.app.height() is not None and sc.app.s.vbat == 11.9)
        # 이륙 시험
        start = time.monotonic()
        await sc.key("t")
        check("s1 t 는 확인을 기다림 (아직 명령 없음)", not fc.cmds(start))
        await sc.key("y")
        await sc.wait(lambda: sc.app._test_task is not None and sc.app._test_task.done(), 30)
        seq = []
        for (t, k, d) in fc.log:
            if t < start:
                continue
            if k == "ORIGIN":
                seq.append("ORIGIN")
            elif k == "CMD":
                c, p = d
                if c == m.MAV_CMD_DO_SET_MODE:
                    seq.append(f"MODE{int(p[1])}")
                elif c == m.MAV_CMD_COMPONENT_ARM_DISARM:
                    seq.append(f"ARM{int(p[0])}")
                elif c == m.MAV_CMD_NAV_TAKEOFF:
                    seq.append(f"TAKEOFF{p[6]:.1f}")
        check("s1 명령 순서", seq == ["ORIGIN", "MODE4", "ARM1", "TAKEOFF0.8", "MODE9"], str(seq))
        check("s1 착지 후 시동 꺼짐", not fc.armed and fc.mode == 9)
        check("s1 '시험 완료' 출력", "시험 완료" in sc.text())
        check("s1 FC 쪽 깨진 프레임 0", fc.bad == 0, str(fc.bad))


async def _to_hover(sc):
    await sc.key("t")
    await sc.key("y")
    await sc.wait(lambda: sc.app._phase == "호버", 15)
    return time.monotonic()


async def s2_kill():
    async with Scenario("s2_kill", hover=5.0) as sc:
        t = await _to_hover(sc)
        await sc.key(" ", 0.6)
        modes = sc.fc.modes_sent(t)
        check("s2 킬 = LAND 3회", modes == [9, 9, 9], str(modes))
        check("s2 시험 취소됨", sc.app._test_task.done() and "시험 취소: 킬" in sc.text())
        await sc.wait(lambda: not sc.fc.armed, 10)
        check("s2 이후 GUIDED 재요청 없음", 4 not in sc.fc.modes_sent(t))


async def s3_mode_change():
    async with Scenario("s3_mode_change", hover=5.0) as sc:
        t = await _to_hover(sc)
        sc.fc.mode = 9  # FC 가 스스로 LAND (예: 페일세이프)
        await sc.wait(lambda: sc.app._test_task.done(), 3)
        await asyncio.sleep(1.0)
        check("s3 모드 변경 → 시험 중단", "시험 중단: 모드가 LAND" in sc.text())
        check("s3 아무 모드 명령도 안 보냄", sc.fc.modes_sent(t) == [], str(sc.fc.modes_sent(t)))


async def s4_short_drop():
    async with Scenario("s4_short_drop", hover=5.0) as sc:
        t = await _to_hover(sc)
        sc.link.down()
        await asyncio.sleep(1.0)
        check("s4 BLE 끊김 메시지", "BLE 끊김 — FC가 3초 뒤 스스로 착륙" in sc.text())
        check("s4 시험 취소", sc.app._test_task.done())
        sc.link.up()
        await sc.wait(lambda: 9 in sc.fc.modes_sent(t), 5)
        check("s4 짧은 끊김 후 GUIDED 상태 → LAND 전송", 4 not in sc.fc.modes_sent(t),
              str(sc.fc.modes_sent(t)))
        await sc.wait(lambda: not sc.fc.armed, 10)


async def s4b_long_drop():
    async with Scenario("s4b_long_drop", hover=8.0) as sc:
        t = await _to_hover(sc)
        sc.link.down()
        await sc.wait(lambda: any(k == "FAILSAFE" for (_, k, _) in sc.fc.log), 6)
        check("s4b 3초 뒤 FC 페일세이프 LAND", sc.fc.mode == 9)
        sc.link.up()
        await asyncio.sleep(3.0)
        check("s4b 재연결 후 아무 모드 명령 안 보냄", sc.fc.modes_sent(t) == [], str(sc.fc.modes_sent(t)))
        check("s4b 시험 재개 안 함", sc.app._test_task.done())


async def s4c_nucleo_reset():
    async with Scenario("s4c_nucleo_reset", hover=8.0) as sc:
        t = await _to_hover(sc)
        sc.fc.alive = False  # 양방향 무응답 (BLE 는 연결된 상태)
        await sc.wait(lambda: sc.app._test_task.done(), 5)
        check("s4c FC 무응답 → 시험 중단", "FC 응답 없음" in sc.text())
        await asyncio.sleep(1.0)
        sc.fc.alive = True
        await sc.wait(lambda: not sc.fc.armed, 15)
        check("s4c 재개 안 함, GUIDED 재요청 없음", 4 not in sc.fc.modes_sent(t), str(sc.fc.modes_sent(t)))


async def s5_double_x():
    async with Scenario("s5_double_x", hover=5.0) as sc:
        t = await _to_hover(sc)
        await sc.key("x", 1.3)
        n_single = [d for (_, d) in sc.fc.cmds(t) if d[0] == m.MAV_CMD_COMPONENT_ARM_DISARM]
        check("s5 x 한 번은 아무것도 안 보냄", not n_single and sc.fc.armed)
        await sc.key("x", 0.3)
        await sc.key("x", 0.6)
        dis = [d for (_, d) in sc.fc.cmds(t) if d[0] == m.MAV_CMD_COMPONENT_ARM_DISARM]
        check("s5 x x → 강제 DISARM(0,21196)", len(dis) == 3 and all(p[0] == 0 and p[1] == 21196 for _, p in dis),
              str(dis))
        check("s5 모터 정지", not sc.fc.armed)


async def s6_arm_reject():
    async with Scenario("s6_arm_reject", fc=FakeFC(reject_arm=True)) as sc:
        t = time.monotonic()
        await sc.key("t")
        await sc.key("y")
        await sc.wait(lambda: sc.app._test_task is not None and sc.app._test_task.done(), 10)
        txt = sc.text()
        check("s6 PreArm 사유 출력", "PreArm: Need Position Estimate" in txt)
        check("s6 시동 거부 → 시험 실패", "시동 거부" in txt)
        check("s6 시동 안 걸렸으니 LAND 안 보냄", sc.fc.modes_sent(t) == [4], str(sc.fc.modes_sent(t)))


async def s7_quit_armed():
    sc = Scenario("s7_quit_armed", hover=8.0)
    async with sc:
        t = await _to_hover(sc)
        await sc.key("q", 1.0)
        await asyncio.wait_for(sc.app_task, 10)
        check("s7 q: LAND 보내고 종료", sc.fc.modes_sent(t) == [9, 9, 9] and sc.app_task.done(),
              str(sc.fc.modes_sent(t)))


async def s8_bad_params():
    fc = FakeFC(params={"MAV_GCS_SYSID": 255, "FS_GCS_ENABLE": 5, "FS_GCS_TIMEOUT": 3})
    async with Scenario("s8_bad_params", fc=fc) as sc:
        t = time.monotonic()
        await sc.key("t", 0.3)
        check("s8 sysid 255 → 이륙 막음", "이륙 시험 못 함" in sc.text() and not sc.fc.cmds(t))


async def s8b_timeout_not_3():
    fc = FakeFC(params={"MAV_GCS_SYSID": 250, "FS_GCS_ENABLE": 5, "FS_GCS_TIMEOUT": 5})
    async with Scenario("s8b_timeout_not_3", fc=fc) as sc:
        check("s8b FS_GCS_TIMEOUT 5 → 기준 3 s 경고", "FS_GCS_TIMEOUT=5 s (이 스크립트 기준 3 s" in sc.text())


async def s9_no_arm():
    async with Scenario("s9_no_arm", no_arm=True) as sc:
        t = time.monotonic()
        await sc.key("t", 0.3)
        await sc.key("y", 0.3)
        check("s9 --no-arm → 이륙 막음", "--no-arm 으로 실행 중" in sc.text() and not sc.fc.cmds(t))


async def s10_ble_fragment_lock():
    """BleLink.write: 조각 + Lock. 동시에 많이 보내도 프레임이 섞이지 않는지, 취소돼도 프레임이 완결되는지."""
    con = n6.Console(OUT / "s10.log")
    link = n6.BleLink("X", con)
    stream = bytearray()

    class FakeClient:
        async def write_gatt_char(self, char, data, response=None):
            assert len(data) <= 20
            await asyncio.sleep(random.random() * 0.004)
            stream.extend(data)

    link._client, link._char, link.connected, link._chunk = FakeClient(), "c", True, 20
    app = n6.N6Fly(make_args(), con, link)
    msgs = []
    for i in range(40):
        if i % 2:
            msgs.append(app.tx.heartbeat_encode(6, 8, 0, 0, 4))
        else:
            msgs.append(app.tx.command_long_encode(1, 1, 511, 0, 32, 200000, 0, 0, 0, 0, i))
    await asyncio.gather(*(app.send(mm) for mm in msgs))
    rx = m.MAVLink(None, 1, 1)
    rx.robust_parsing = True
    got = rx.parse_buffer(bytes(stream)) or []
    bad = sum(1 for g in got if g.get_type() == "BAD_DATA")
    check("s10 40개 동시 전송, 섞임 없음", len(got) == 40 and bad == 0, f"{len(got)} 개, bad {bad}")

    stream.clear()

    async def victim():
        await app.send(app.tx.command_long_encode(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, 0.8))

    task = asyncio.get_running_loop().create_task(victim())
    await asyncio.sleep(0.002)
    task.cancel()
    await app.send(app.tx.command_long_encode(1, 1, 176, 0, 1, 9, 0, 0, 0, 0, 0))
    await asyncio.sleep(0.1)
    got = [g.get_type() for g in (rx.parse_buffer(bytes(stream)) or [])]
    check("s10 취소된 프레임도 완결 → 뒤 LAND 프레임 정상", got == ["COMMAND_LONG", "COMMAND_LONG"], str(got))
    con.close()


class _FakeChar:
    properties = ["write-without-response", "write"]
    max_write_without_response_size = 244


class _FakeServices:
    def get_characteristic(self, uuid):
        return _FakeChar()


class FakeBleakClient:
    instances = []

    def __init__(self, dev, disconnected_callback=None, timeout=20.0, **kw):
        self.cb = disconnected_callback
        self.is_connected = False
        self.services = _FakeServices()
        self.fail_write = []        # 다음 쓰기들에서 던질 예외 목록
        self.writes = []
        FakeBleakClient.instances.append(self)

    async def connect(self):
        await asyncio.sleep(0.05)
        self.is_connected = True

    async def start_notify(self, uuid, cb):
        self.notify_cb = cb

    async def write_gatt_char(self, char, data, response=None):
        if self.fail_write:
            raise self.fail_write.pop(0)
        self.writes.append(bytes(data))

    async def disconnect(self):
        if self.is_connected:
            self.is_connected = False
            self.cb(self)

    def drop(self):                  # 링크 끊김 흉내
        self.is_connected = False
        self.cb(self)


class _FakeDev:
    address = "FB:55:E2:D6:29:FB"


class FakeScanner:
    @staticmethod
    async def find_device_by_name(name, timeout=1.0):
        await asyncio.sleep(0.05)
        return _FakeDev()


async def s11_blelink_disconnect():
    """BleLink 끊김 처리 (실기 시험에서 발견한 버그 재현)."""
    real_client, real_scanner = n6.BleakClient, n6.BleakScanner
    n6.BleakClient, n6.BleakScanner = FakeBleakClient, FakeScanner
    FakeBleakClient.instances.clear()
    con = n6.Console(OUT / "s11_blelink.log")
    link = n6.BleLink("N6-DRONE-1", con, 0.1)
    app = n6.N6Fly(make_args(), con, link)
    states = []
    orig = link.on_state
    link.on_state = lambda up: (states.append(up), orig(up))
    run = asyncio.get_running_loop().create_task(link.run())

    async def until(cond, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not cond():
            await asyncio.sleep(0.01)
        return cond()

    def log():
        return (OUT / "s11_blelink.log").read_text(encoding="utf-8")

    try:
        await until(lambda: link.connected)
        a = FakeBleakClient.instances[-1]
        # (1) 진짜 끊김 -> on_state(False) + "BLE 끊김" + 진행 중 시험 취소
        dummy = asyncio.get_running_loop().create_task(asyncio.sleep(100))
        app._test_task = dummy
        a.drop()
        ok = await until(lambda: states == [True, False])
        await asyncio.sleep(0.05)
        check("s11 끊김 콜백 → on_state(False) 한 번", ok, str(states))
        check("s11 'BLE 끊김 — FC가 3초 뒤' 표시", "BLE 끊김 — FC가 3초 뒤 스스로 착륙" in log())
        check("s11 진행 중 시험 취소 + 재연결 시 LAND 확인 예약", dummy.cancelled() and app._land_after_relink)
        # (2) 재연결 후 이전 클라이언트의 늦은 끊김 콜백 -> 새 세션 유지
        await until(lambda: link.connected and len(FakeBleakClient.instances) == 2)
        b = FakeBleakClient.instances[-1]
        a.cb(a)
        await asyncio.sleep(0.3)
        check("s11 이전 연결의 늦은 콜백이 새 연결을 안 끊음", link.connected and states == [True, False, True], str(states))
        # (3) 연결은 살아 있고 크기 오류 -> 20 B 로 줄여 재시도 성공
        b.fail_write = [Exception("Invalid Length")]
        ok3 = await link.write(bytes(range(60)))
        check("s11 크기 오류 → 20 B 로 재시도 성공", ok3 and link._chunk == 20 and len(b.writes) == 3
              and "조각을 20 B 로 줄여" in log(), f"ok={ok3} chunk={link._chunk} writes={len(b.writes)}")
        # (4) 끊겨서 쓰기 실패 (콜백은 아직 안 옴, bleak 는 아직 연결됐다고 보고) -> 즉시 끊김 처리
        #     실기: 어댑터 OFF 직후 is_connected=True 인데 "Not connected" 가 옴
        link._chunk = 244
        n20 = log().count("조각을 20 B 로 줄여")
        b.fail_write = [Exception("org.bluez.Error.Failed: Not connected")]
        ok4 = await link.write(b"x" * 30)
        ok = await until(lambda: states[-1] is False)
        check("s11 끊김으로 쓰기 실패 → 바로 끊김 처리", (not ok4) and ok and log().count("조각을 20 B 로 줄여") == n20,
              f"ok4={ok4} states={states}")
    finally:
        link._closing = True
        run.cancel()
        await asyncio.gather(run, return_exceptions=True)
        n6.BleakClient, n6.BleakScanner = real_client, real_scanner
        con.close()


async def s12_param_lost_then_reread():
    """FS_GCS_ENABLE 응답이 3번 다 빠짐 -> t 막힘 -> t 다시 누르면 다시 읽고 통과."""
    fc = FakeFC(drop_param_once=())
    sc = Scenario("s12_param_lost", fc=fc)
    fc.drop_once = {"FS_GCS_ENABLE"}
    orig = fc.on_msg
    drops = {"n": 0}
    def on_msg(msg):  # 처음 3번의 FS_GCS_ENABLE 요청은 응답 안 함
        if msg.get_type() == "PARAM_REQUEST_READ" and msg.param_id == "FS_GCS_ENABLE" and drops["n"] < 3:
            drops["n"] += 1
            return
        orig(msg)
    fc.on_msg = on_msg
    sc.fc_task = asyncio.get_running_loop().create_task(fc.run())
    sc.app_task = asyncio.get_running_loop().create_task(sc.app.run(sc.q))
    try:
        await sc.wait(lambda: sc.app.fs_checked and sc.app.s.uplink_ok and sc.app.s.last_hb > 0, 15)
        t = time.monotonic()
        await sc.key("t", 0.3)
        check("s12 FS_GCS_ENABLE 못 읽음 → t 막힘", "FS_GCS_ENABLE 못 읽음" in sc.text() and not sc.fc.cmds(t))
        await sc.wait(lambda: sc.app.fs_checked and not sc.app.fs_problems, 8)
        check("s12 t 가 다시 읽기 시작 → 성공", "페일세이프 파라미터 OK" in sc.text())
        await sc.key("t", 0.2)
        check("s12 다시 읽은 뒤 t → 확인 문구", "확인했으면 y" in sc.text())
        await sc.key("n", 0.2)
    finally:
        await sc.app.request_quit("sim")
        await asyncio.wait_for(sc.app_task, 10)
        sc.fc_task.cancel()
        sc.console.close()


async def s13_heartbeat_loss():
    """XIAO 구간 실측처럼 FC HEARTBEAT 80% 손실 -> ACK 로 확인해서 시험이 끝까지 돌아야 함."""
    async with Scenario("s13_hb_loss", fc=FakeFC(hb_drop=0.8)) as sc:
        await sc.key("t")
        await sc.key("y")
        await sc.wait(lambda: sc.app._test_task is not None and sc.app._test_task.done(), 40)
        txt = sc.text()
        check("s13 HEARTBEAT 80% 손실에도 GUIDED·시동 확인(ACK)", "GUIDED 확인 (ACK)" in txt and "시동 확인 (ACK)" in txt)
        check("s13 시험 완료 + 시동 꺼짐", "시험 완료" in txt and not sc.fc.armed)


async def s14_baro_dip_at_arm():
    """실기 재현: 시동 순간 바닥에서 EKF 높이가 -0.21 m 로 떨어짐 -> 거리센서 기준으로 도달 판단해야 함."""
    fc = FakeFC(baro_dip=0.21)
    async with Scenario("s14_baro_dip", fc=fc, alt=0.3, hover=1.0) as sc:
        await sc.key("t")
        await sc.key("y")
        await sc.wait(lambda: " 도달 (" in sc.text() or "도달 못 함" in sc.text(), 15)
        true_h = -fc.z
        check("s14 도달 판단 시 실제 높이 ≥ 0.18 m (EKF 흔들림에 속지 않음)", true_h >= 0.18, f"실제 {true_h:.2f} m")
        await sc.wait(lambda: sc.app._test_task.done(), 30)
        check("s14 시험 완료", "시험 완료" in sc.text() and not fc.armed)


async def s15_yaw():
    """--yaw 90: 호버 뒤 90° 씩 4번 회전, 회전마다 통계, CSV 기록, 한 바퀴 돌아 원래 방위각."""
    (OUT / "s15_yaw_att.csv").unlink(missing_ok=True)
    async with Scenario("s15_yaw", yaw=90.0, yaw_hold=0.6, hover=0.3) as sc:
        fc = sc.fc
        start = time.monotonic()
        yaw0 = fc.yaw
        await sc.key("t")
        await sc.key("y")
        await sc.wait(lambda: sc.app._test_task is not None and sc.app._test_task.done(), 60)
        yc = [p for (_, (c, p)) in fc.cmds(start) if c == m.MAV_CMD_CONDITION_YAW]
        check("s15 CONDITION_YAW 4번 (90°, 30°/s, 시계, 상대)",
              len(yc) == 4 and all(p[:4] == (90.0, 30.0, 1.0, 1.0) for p in yc), str(yc))
        txt = sc.text()
        check("s15 회전마다 통계 4줄", sum(f"회전 {k}/4: 목표" in txt for k in range(1, 5)) == 4
              and txt.count("요 오차") == 4)
        check("s15 회전마다 도달", txt.count("만에 도달") == 4)
        check("s15 한 바퀴 뒤 원래 방위각", abs(((fc.yaw - yaw0) + 180) % 360 - 180) < 1.0, f"{fc.yaw:.1f}")
        check("s15 LAND 는 회전 뒤", fc.modes_sent(start)[-1] == 9 and "시험 완료" in txt)
        csv = OUT / "s15_yaw_att.csv"
        rows = csv.read_text(encoding="utf-8").splitlines()
        check("s15 자세 CSV (머리줄 + 회전 중 목표 방위각)", rows[0].startswith("time,phase,target_yaw_deg")
              and any(",회전2/4," in r for r in rows), f"{len(rows)} 줄")


async def s16_hold():
    """--hold: 시간 제한 없이 호버, 스페이스(킬)로만 LAND."""
    async with Scenario("s16_hold", alt=1.3, hover=float("inf")) as sc:
        t = await _to_hover(sc)
        await asyncio.sleep(4.0)
        check("s16 4초 지나도 호버 유지, LAND 안 보냄", sc.app._phase == "호버" and sc.fc.modes_sent(t) == []
              and abs(sc.fc.z + 1.3) < 0.01, f"{sc.app._phase} z={sc.fc.z:.2f}")
        check("s16 안내 문구", "착륙 명령(스페이스)까지" in sc.text())
        await sc.key(" ", 0.6)
        check("s16 스페이스 → LAND 3회", sc.fc.modes_sent(t) == [9, 9, 9], str(sc.fc.modes_sent(t)))
        await sc.wait(lambda: not sc.fc.armed, 10)


async def s17_move():
    """--move 0.3: 호버 뒤 앞·뒤·오른쪽·왼쪽 0.3 m 이동과 복귀, 방위각 고정, 시작점으로 돌아옴."""
    (OUT / "s17_move.log").unlink(missing_ok=True)
    async with Scenario("s17_move", move=0.3, move_hold=0.6, hover=0.3) as sc:
        fc = sc.fc
        fc.x, fc.y = 1.0, -2.0         # 원점이 아닌 곳에서 시작
        await sc.key("t")
        await sc.key("y")
        await sc.wait(lambda: sc.app._test_task is not None and sc.app._test_task.done(), 50)
        txt = sc.text()
        check("s17 이동 8번 모두 도달", txt.count("만에 도달 | 정지") == 8, str(txt.count("만에 도달 | 정지")))
        check("s17 시작점으로 복귀", math.hypot(fc.x - 1.0, fc.y + 2.0) < 0.05, f"{fc.x:.2f}, {fc.y:.2f}")
        psi = math.radians(40.0)
        far = max(fc.pos_targets, key=lambda p: math.hypot(p[2] - 1.0, p[3] + 2.0))
        check("s17 이동 거리 0.3 m", abs(math.hypot(far[2] - 1.0, far[3] + 2.0) - 0.3) < 1e-3)
        first = fc.pos_targets[0]   # 앞: 방위각 40° 방향
        check("s17 앞 = 기체 방위각 방향", abs(first[2] - (1.0 + 0.3 * math.cos(psi))) < 0.01
              and abs(first[3] - (-2.0 + 0.3 * math.sin(psi))) < 0.01, str(first))
        check("s17 프레임 LOCAL_NED, 방위각 고정",
              all(p[0] == m.MAV_FRAME_LOCAL_NED and p[1] == n6.MOVE_TYPE_MASK and abs(p[5] - psi) < 0.05
                  for p in fc.pos_targets), str(fc.pos_targets[:2]))
        check("s17 높이 유지 목표", all(abs(p[4] + 0.8) < 0.05 for p in fc.pos_targets), str(first[4]))
        check("s17 시험 완료", "시험 완료" in txt and not fc.armed)


async def s18_manual():
    """--manual: 키 하나에 한 칸, 이동 중 누른 키는 버림, 1 m 넘는 목표 거부, r = 시작점, 스페이스로 착륙."""
    (OUT / "s18_manual.log").unlink(missing_ok=True)
    async with Scenario("s18_manual", manual=True, step=0.3, hover=0.3) as sc:
        fc = sc.fc
        fc.x, fc.y = 1.0, -2.0
        psi = math.radians(40.0)
        await sc.key("t")
        await sc.key("y")
        await sc.wait(lambda: sc.app._phase == "수동대기", 20)
        await sc.key("UP")
        for k in ("UP", "w", "RIGHT"):          # 이동 중 연타 -> 모두 버려야 함
            await asyncio.sleep(0.05)
            await sc.key(k)
        await sc.wait(lambda: "앞 → 앞 +0.30" in sc.text(), 10)
        await sc.wait(lambda: sc.app._phase == "수동대기", 5)
        txt = sc.text()
        check("s18 이동 중 키 무시", txt.count("이동 중 — 입력 무시") == 3, str(txt.count("이동 중 — 입력 무시")))
        check("s18 한 칸만 감", math.hypot(fc.x - (1.0 + 0.3 * math.cos(psi)), fc.y - (-2.0 + 0.3 * math.sin(psi))) < 0.12,
              f"{fc.x:.2f}, {fc.y:.2f}")
        for _ in range(3):                       # 앞 0.6, 0.9, 1.2(거부)
            await sc.key("w")
            await sc.wait(lambda: sc.app._phase == "수동대기" and not sc.app._manual_busy, 10)
            await asyncio.sleep(0.1)
        check("s18 1 m 넘는 목표 거부", "1.20 m 라서 거부" in sc.text(), "")
        far = max(fc.pos_targets, key=lambda p: math.hypot(p[2] - 1.0, p[3] + 2.0))
        check("s18 최대 0.9 m", abs(math.hypot(far[2] - 1.0, far[3] + 2.0) - 0.9) < 1e-3)
        await sc.key("r")
        await sc.wait(lambda: "시작점 → 앞 +0.00" in sc.text(), 10)
        check("s18 r 시작점 복귀", math.hypot(fc.x - 1.0, fc.y + 2.0) < 0.12, f"{fc.x:.2f}, {fc.y:.2f}")
        check("s18 방위각·높이 고정", all(abs(p[5] - psi) < 0.05 and abs(p[4] + 0.8) < 0.05 for p in fc.pos_targets))
        await sc.key(" ")
        await sc.wait(lambda: not fc.armed, 20)
        check("s18 스페이스로 착륙", not fc.armed and not sc.app._manual_on)


async def main():
    random.seed(1)
    only = sys.argv[1:]
    for fn in (s11_blelink_disconnect, s10_ble_fragment_lock, s1_normal, s2_kill, s3_mode_change, s4_short_drop, s4b_long_drop,
               s4c_nucleo_reset, s5_double_x, s6_arm_reject, s7_quit_armed, s8_bad_params, s8b_timeout_not_3, s9_no_arm, s12_param_lost_then_reread, s13_heartbeat_loss, s14_baro_dip_at_arm, s15_yaw, s16_hold, s17_move, s18_manual):
        if only and fn.__name__ not in only:
            continue
        try:
            await asyncio.wait_for(fn(), 60)
        except Exception as e:
            check(fn.__name__, False, f"예외 {e!r}")
    fails = [r for r in RESULTS if not r[1]]
    print(f"\n{len(RESULTS) - len(fails)}/{len(RESULTS)} PASS")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    asyncio.run(main())
