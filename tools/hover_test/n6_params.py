"""Read-only check: BLE connect, FC HEARTBEAT receive, PARAM_REQUEST_READ only.
Sends NO heartbeat, NO arm/mode/motor command, NO param write, NO EKF origin."""
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner
from pymavlink.dialects.v20 import ardupilotmega as mav

NAME = sys.argv[1] if len(sys.argv) > 1 else "N6-DRONE-4"
W = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
N = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
READ = ['FRAME_CLASS', 'FRAME_TYPE', 'AHRS_ORIENTATION', 'SERVO_BLH_RVMASK', 'SERIAL1_PROTOCOL', 'SERIAL1_BAUD', 'SERIAL4_PROTOCOL', 'SERIAL4_BAUD', 'FS_GCS_ENABLE', 'FS_GCS_TIMEOUT', 'MAV_GCS_SYSID', 'FLOW_TYPE', 'FLOW_ORIENT_YAW', 'RNGFND1_TYPE', 'RNGFND1_ORIENT', 'RNGFND1_MAX_CM', 'RNGFND1_MIN_CM', 'EK3_SRC1_POSXY', 'EK3_SRC1_VELXY', 'EK3_SRC1_POSZ', 'EK3_SRC1_VELZ', 'EK3_SRC1_YAW', 'EK3_RNG_USE_HGT', 'EK3_RNG_USE_SPD', 'EK3_ENABLE', 'EK2_ENABLE', 'AHRS_EKF_TYPE', 'ARMING_SKIPCHK', 'BATT_MONITOR', 'BATT_AMP_PERVLT', 'BATT_ARM_VOLT', 'BATT_FS_LOW_ACT', 'MOT_BAT_VOLT_MAX', 'MOT_BAT_VOLT_MIN', 'GUID_OPTIONS', 'WPNAV_SPEED', 'PILOT_TKOFF_ALT', 'RC_PROTOCOLS', 'FLTMODE1', 'INS_ACCEL_FILTER', 'BRD_SAFETY_DEFLT']
READ0 = ["MAV_GCS_SYSID", "FS_GCS_ENABLE", "FS_GCS_TIMEOUT", "ARMING_CHECK", "SERIAL4_BAUD"]

rx = mav.MAVLink(None, srcSystem=250, srcComponent=190)
rx.robust_parsing = True
tx = mav.MAVLink(open(__import__("os").devnull, "wb"), srcSystem=250, srcComponent=190)
hb = {"n": 0, "armed": None, "mode": None, "t": 0.0, "sys": None}
params, other = {}, {}


def on_notify(_s, data):
    for m in rx.parse_buffer(bytes(data)) or []:
        t = m.get_type()
        if t == "HEARTBEAT" and m.get_srcSystem() == 1:
            hb.update(n=hb["n"] + 1, armed=bool(m.base_mode & 128), mode=m.custom_mode, t=time.time(), sys=m.get_srcSystem())
        elif t == "PARAM_VALUE":
            params[m.param_id if isinstance(m.param_id, str) else m.param_id.decode()] = m.param_value
        elif t == "STATUSTEXT":
            other.setdefault("text", []).append(m.text)
        elif t != "BAD_DATA":
            other[t] = other.get(t, 0) + 1


async def main():
    dev = await BleakScanner.find_device_by_name(NAME, timeout=10)
    print("found:", dev)
    if dev is None:
        return
    async with BleakClient(dev, timeout=20) as c:
        ch = c.services.get_characteristic(W)
        resp = "write-without-response" not in ch.properties
        size = max(20, min(int(ch.max_write_without_response_size), 244))
        print("BLE connected, chunk", size, "response" if resp else "no-response")
        await c.start_notify(N, on_notify)
        async def hbloop():
            while True:
                f = tx.heartbeat_encode(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE).pack(tx)
                await c.write_gatt_char(ch, f, response=resp)
                await asyncio.sleep(0.2)
        ht = asyncio.create_task(hbloop()) if "--hb" in sys.argv else None
        await asyncio.sleep(3)
        print("HEARTBEAT count after 25s:", hb["n"], "armed:", hb["armed"], "custom_mode:", hb["mode"])
        for rnd in range(4):
            for name in READ:
                if name in params:
                    continue
                f = tx.param_request_read_encode(1, 1, name.encode(), -1).pack(tx)
                for i in range(0, len(f), size):
                    await c.write_gatt_char(ch, f[i:i + size], response=resp)
                await asyncio.sleep(0.25)
            await asyncio.sleep(1.5)
            pass
        [print(k, params.get(k)) for k in READ]
        print("missing:", [k for k in READ if k not in params])
        print("OTHER msgs:", other)
        print("HEARTBEAT total:", hb["n"], "armed:", hb["armed"])
        if ht: ht.cancel()


asyncio.run(main())
