"""Host test for FSBL/Inc/link_failsafe.h: builds the C driver, feeds pymavlink frames, checks the output.
Run: python firmware/nucleo/n6_bridge/test/test_link_failsafe.py"""
import os
import subprocess
import tempfile
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mav

HERE = Path(__file__).resolve().parent
exe = Path(tempfile.gettempdir()) / ("link_failsafe_host" + (".exe" if os.name == "nt" else ""))
subprocess.run(["gcc", "-Wall", "-Wextra", "-Werror", "-O2", "-o", str(exe), str(HERE / "link_failsafe_host.c")], check=True)


def enc(sysid, compid, msg):
    m = mav.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    return msg.pack(m).hex()


def gcs_hb():
    return enc(250, 190, mav.MAVLink_heartbeat_message(6, 8, 0, 0, 4, 3))


def fc_hb(armed, mode):
    return enc(1, 1, mav.MAVLink_heartbeat_message(2, 3, 0x80 if armed else 0, mode, 4, 3))


def run(lines):
    r = subprocess.run([str(exe)], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    return r.stdout.split("\n")[:-1]


LAND, GUIDED = 9, 4
base = [f"G 0 {gcs_hb()}", f"F 0 {fc_hb(True, GUIDED)}"]

# 1) link alive, LAND armed -> nothing
out = run(base + [f"G 9000 {gcs_hb()}", f"F 9500 {fc_hb(True, LAND)}", "P 9600"])
assert out == ["NONE"], out

# 2) link lost 12 s, FC armed LAND -> force disarm frame, parsed by pymavlink
out = run(base + [f"F 11900 {fc_hb(True, LAND)}", "P 11999", "P 12000", "P 12500", "P 13000"])
assert out[0] == "NONE" and out[1].startswith("OUT ") and out[2] == "NONE" and out[3].startswith("OUT "), out
p = mav.MAVLink(None)
msgs = p.parse_buffer(bytes.fromhex(out[1][4:]))
assert msgs and len(msgs) == 1, msgs
c = msgs[0]
assert c.get_type() == "COMMAND_LONG" and c.command == 400 and c.param1 == 0 and c.param2 == 21196, c
assert (c.target_system, c.target_component, c.get_srcSystem()) == (1, 1, 250), c

# 3) link lost but GUIDED -> never cut
out = run(base + [f"F 20000 {fc_hb(True, GUIDED)}", "P 20001"])
assert out == ["NONE"], out

# 4) link lost, LAND but disarmed -> nothing
out = run(base + [f"F 20000 {fc_hb(False, LAND)}", "P 20001"])
assert out == ["NONE"], out

# 5) FC heartbeat stale (>3 s) -> nothing
out = run(base + [f"F 12000 {fc_hb(True, LAND)}", "P 15500"])
assert out == ["NONE"], out

# 6) link returns -> stops
out = run(base + [f"F 12000 {fc_hb(True, LAND)}", "P 12001", f"G 12500 {gcs_hb()}", f"F 13500 {fc_hb(True, LAND)}", "P 13600"])
assert out[0].startswith("OUT ") and out[1] == "NONE", out

# 7) corrupted GCS heartbeat does not count as link alive
bad = bytearray.fromhex(gcs_hb()); bad[-1] ^= 0xFF
out = run(base + [f"G 9000 {bad.hex()}", f"F 12000 {fc_hb(True, LAND)}", "P 12001"])
assert out[0].startswith("OUT "), out

# 8) heartbeat split across noise / other messages still parsed
noise = "fe0102030405" + enc(1, 1, mav.MAVLink_attitude_message(1, 0.1, 0.2, 0.3, 0, 0, 0))
out = run([f"G 0 {gcs_hb()}", f"F 11000 {noise}{fc_hb(True, LAND)}{noise}", "P 12000"])
assert out[0].startswith("OUT "), out

# 9) never connected since boot -> nothing
out = run([f"F 20000 {fc_hb(True, LAND)}", "P 20001"])
assert out == ["NONE"], out

print("link_failsafe: all ok")
