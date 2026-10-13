"""A's ENU API. One owner/thread per MAVLink endpoint; truth never drives control.

Positions remain EKF-local unless set_alignment() explicitly sets a world offset.
No covariance is fabricated from EKF_STATUS_REPORT innovation diagnostics.
"""
import copy
import math
import os
import threading
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


def ned_to_enu(x, y, z):
    return [y, x, -z]


def position_covariance_enu(packed):
    """LOCAL_POSITION_NED_COV: upper triangular 9x9, first three states XYZ."""
    if len(packed) != 45:
        return None
    p = [[packed[0], packed[1], packed[2]],
         [packed[1], packed[9], packed[10]],
         [packed[2], packed[10], packed[17]]]
    if not all(math.isfinite(v) for row in p for v in row):
        return None
    if any(p[i][i] < 0 for i in range(3)):
        return None
    order, signs = (1, 0, 2), (1, 1, -1)
    return [[p[order[i]][order[j]] * signs[i] * signs[j]
             for j in range(3)] for i in range(3)]


class DroneLink:
    def __init__(self, drone_id, endpoint=None, stale_s=0.5, command_ttl=0.5):
        self.drone_id, self.sysid = drone_id, drone_id + 1
        self.stale_s, self.command_ttl = stale_s, command_ttl
        self.link = mavutil.mavlink_connection(
            endpoint or f'udpin:127.0.0.1:{14550 + 10 * drone_id}',
            source_system=250, source_component=190)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._pos = None
        self._seen = self._hb_seen = self._ekf_seen = self._cov_seen = 0.0
        self._armed, self._mode, self._ekf = False, '', 0
        self._cov = None
        self._offset = [0.0, 0.0, 0.0]
        self._clock_offset = None
        self._aligned = False
        self._velocity = None
        self._ttl_expired = False
        self._fault_land = False
        self._error = None
        self.texts, self.acks = [], []
        self.parameters = {}
        self.origin = None
        self.received_positions = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _handle(self, msg, now):
        # Sensor bridge heartbeats must never change our target or armed state.
        if msg.get_srcSystem() != self.sysid or msg.get_srcComponent() != 1:
            return
        kind = msg.get_type()
        if kind == 'HEARTBEAT':
            self._hb_seen = now
            self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self._mode = mavutil.mode_string_v10(msg)
        elif kind == 'LOCAL_POSITION_NED':
            if self._pos and msg.time_boot_ms < self._pos['time_boot_ms']:
                self._aligned = False
                self._offset = [0.0] * 3
                self._clock_offset = None
                self._velocity = None
            self._pos = dict(time_boot_ms=msg.time_boot_ms,
                             position=ned_to_enu(msg.x, msg.y, msg.z),
                             velocity=ned_to_enu(msg.vx, msg.vy, msg.vz))
            self._seen = now
            self.received_positions += 1
        elif kind == 'EKF_STATUS_REPORT':
            self._ekf, self._ekf_seen = msg.flags, now
        elif kind == 'LOCAL_POSITION_NED_COV':
            self._cov = position_covariance_enu(msg.covariance)
            self._cov_seen = now
        elif kind == 'STATUSTEXT':
            self.texts.append(msg.text)
            self.texts = self.texts[-500:]
        elif kind == 'COMMAND_ACK':
            self.acks.append(dict(command=msg.command, result=msg.result))
        elif kind == 'PARAM_VALUE':
            self.parameters[msg.param_id] = msg.param_value
        elif kind == 'GPS_GLOBAL_ORIGIN':
            self.origin = dict(latitude_deg=msg.latitude/1e7,
                               longitude_deg=msg.longitude/1e7, altitude_m=msg.altitude/1000)

    def _estimate(self, now):
        stale = self._pos is None or now - self._seen > self.stale_s
        # attitude, horizontal velocity, relative OR absolute horizontal position
        ekf_ok = (self._ekf & 3) == 3 and bool(self._ekf & 24)
        fresh_status = now - self._hb_seen < 3 and now - self._ekf_seen < 3
        raw = self._pos['position'][:] if self._pos else None
        cov_ok = self._cov is not None and now - self._cov_seen < self.stale_s
        return dict(
            time_boot_ms=self._pos['time_boot_ms'] if self._pos else None,
            time_sim_s=(self._pos['time_boot_ms'] / 1000 + self._clock_offset
                        if self._pos and self._clock_offset is not None else None),
            raw_position_enu_m=raw,
            position_enu_m=([v + o for v, o in zip(raw, self._offset)] if raw else [math.nan]*3),
            velocity_enu_mps=self._pos['velocity'][:] if self._pos else [math.nan]*3,
            position_cov_enu_m2=copy.deepcopy(self._cov) if cov_ok else None,
            uncertainty_valid=cov_ok,
            uncertainty_source='LOCAL_POSITION_NED_COV' if cov_ok else 'unavailable',
            valid=not stale and fresh_status and ekf_ok and self._error is None,
            stale=stale, age_host_s=now-self._seen if self._seen else None,
            mode=self._mode, armed=self._armed, heartbeat_fresh=now-self._hb_seen < 3,
            ekf_flags=self._ekf, aligned=self._aligned, error=self._error,
            heartbeat_age_host_s=now-self._hb_seen, ekf_age_host_s=now-self._ekf_seen,
            command_expired=self._ttl_expired)

    def get_estimate(self):
        with self._lock:
            return self._estimate(time.monotonic())

    def set_alignment(self, offset_enu, clock_offset_s):
        if len(offset_enu) != 3 or not all(math.isfinite(v) for v in [*offset_enu, clock_offset_s]):
            raise ValueError('finite XYZ and clock offset required')
        with self._lock:
            self._offset, self._clock_offset = list(offset_enu), clock_offset_s
            self._aligned = True

    def send_velocity(self, vx, vy, vz):
        if not all(math.isfinite(v) for v in (vx, vy, vz)):
            raise ValueError('velocity must be finite')
        if math.hypot(vx, vy) > 0.500001 or abs(vz) > 0.300001:
            raise ValueError('velocity exceeds limits: horizontal 0.5, vertical 0.3 m/s')
        with self._lock:
            est = self._estimate(time.monotonic())
            if not est['valid'] or not self._armed or self._mode != 'GUIDED':
                raise RuntimeError('velocity requires fresh valid estimate, armed GUIDED')
            self._velocity = (time.monotonic(), vx, vy, vz)
            self._ttl_expired = False
            self._fault_land = False

    def _send_velocity(self, ve, vn, vu):
        self.link.mav.set_position_target_local_ned_send(
            0, self.sysid, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            3527, 0, 0, 0, vn, ve, -vu, 0, 0, 0, 0, 0)

    def set_mode(self, mode):
        # Explicit target; pymavlink's discovered target may see bridge heartbeats.
        modes = {'GUIDED': 4, 'LAND': 9}
        with self._lock:
            if mode == 'LAND':
                self._velocity = None
            self.link.mav.set_mode_send(self.sysid, 1, modes[mode])

    def command(self, command, *params):
        with self._lock:
            self.link.mav.command_long_send(self.sysid, 1, command, 0,
                                           *(list(params) + [0]*(7-len(params))))

    def request_parameter(self, name):
        with self._lock:
            self.link.mav.param_request_read_send(self.sysid, 1, name.encode(), -1)

    def set_origin(self, latitude=37.0, longitude=127.0, altitude=0.0):
        if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
            raise ValueError('origin must be finite')
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError('invalid geographic origin')
        with self._lock:
            if self._armed:
                raise RuntimeError('set origin before arming')
            self.link.mav.set_gps_global_origin_send(
                self.sysid, round(latitude*1e7), round(longitude*1e7), round(altitude*1000))
            self.command(512, 49)

    def _control_step(self, now):
        if not self._velocity or not self._armed or self._mode != 'GUIDED':
            return
        if not self._estimate(now)['valid']:
            self._fault_land = True
            self.set_mode('LAND')
        elif now - self._velocity[0] > self.command_ttl:
            self._ttl_expired = True
            self._send_velocity(0, 0, 0)
        else:
            self._send_velocity(*self._velocity[1:])

    def _loop(self):
        next_hb = next_request = next_velocity = 0.0
        probed_covariance = False
        try:
            while not self._stop.is_set():
                with self._lock:
                    now = time.monotonic()
                    for _ in range(100):
                        msg = self.link.recv_match(blocking=False)
                        if msg is None:
                            break
                        self._handle(msg, now)
                    if now >= next_hb:
                        self.link.mav.heartbeat_send(6, 8, 0, 0, 0)
                        next_hb = now + 1
                    if self._hb_seen and now >= next_request:
                        for message, interval in ((32, 100000), (193, 500000)):
                            self.command(511, message, interval)
                        self.command(512, 49)
                        if not probed_covariance:
                            for message in (64, 230):
                                self.command(511, message, 200000)
                            probed_covariance = True
                        next_request = now + 10
                    if self._velocity and now >= next_velocity:
                        self._control_step(now)
                        next_velocity = now + 0.1
                self._stop.wait(0.01)
        except Exception as exc:
            with self._lock:
                self._error = repr(exc)
                if self._velocity and self._armed:
                    try:
                        self.set_mode('LAND')
                    except Exception:
                        pass

    def close(self):
        with self._lock:
            if self._velocity and self._armed:
                self.set_mode('LAND')
        self._stop.set()
        self._thread.join(timeout=2)
        self.link.close()


class Fleet:
    def __init__(self, ids, **kwargs):
        self.drones = {}
        try:
            for i in ids:
                self.drones[i] = DroneLink(i, **kwargs)
        except Exception:
            self.close()
            raise

    def get_estimate(self, drone_id):
        return self.drones[drone_id].get_estimate()

    def send_velocity(self, drone_id, vx, vy, vz):
        self.drones[drone_id].send_velocity(vx, vy, vz)

    def close(self):
        for drone in self.drones.values():
            drone.close()
