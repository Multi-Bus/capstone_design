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


def fc_every_s(t0, t1, armed, mode):
    """FC heartbeat once per second from t0 to t1 ms (inclusive)."""
    return [f"F {t} {fc_hb(armed, mode)}" for t in range(t0, t1 + 1, 1000)]


def run(lines):
    """Lines are replayed in time order (G/F before P at the same ms), like the bridge sees them."""
    lines = sorted(lines, key=lambda l: (int(l.split()[1]), l[0] == "P"))
    r = subprocess.run([str(exe)], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    return r.stdout.split("\n")[:-1]


LAND, GUIDED = 9, 4
# Laptop heartbeat stops after t=0. FC: GUIDED until 2 s, failsafe LAND from 3 s (like the FC does).
lost = [f"G 0 {gcs_hb()}"] + fc_every_s(0, 2000, True, GUIDED)

# 1) 6호기 2026-11-25 case: 5 s after link loss, LAND only 2 s old -> must NOT cut
out = run(lost + fc_every_s(3000, 5000, True, LAND) + ["P 5000"])
assert out == ["NONE"], out

# 2) LAND 9 s old (link lost 12 s) -> force disarm, resent once per second
out = run(lost + fc_every_s(3000, 13000, True, LAND) + ["P 11999", "P 12000", "P 12500", "P 13000"])
assert out[0] == "NONE" and out[1].startswith("OUT ") and out[2] == "NONE" and out[3].startswith("OUT "), out
c = mav.MAVLink(None).parse_buffer(bytes.fromhex(out[1][4:]))
assert c and len(c) == 1, c
c = c[0]
assert c.get_type() == "COMMAND_LONG" and c.command == 400 and c.param1 == 0 and c.param2 == 21196, c
assert (c.target_system, c.target_component, c.get_srcSystem()) == (1, 1, 250), c

# 3) link alive, LAND long (pilot kill + slow landing) -> laptop handles it, bridge does nothing
alive = [f"G {t} {gcs_hb()}" for t in range(0, 20001, 200)]
out = run(alive + fc_every_s(0, 20000, True, LAND) + ["P 20000"])
assert out == ["NONE"], out

# 4) LAND started by the pilot before the link dropped: counts from the LAND start
out = run([f"G {t} {gcs_hb()}" for t in range(0, 4001, 200)] + fc_every_s(0, 9000, True, LAND) + ["P 8999", "P 9000"])
assert out == ["NONE", "OUT " + out[1][4:]] and out[1].startswith("OUT "), out

# 5) link lost but GUIDED -> never cut
out = run(lost + fc_every_s(3000, 30000, True, GUIDED) + ["P 30000"])
assert out == ["NONE"], out

# 6) LAND but disarmed -> nothing
out = run(lost + fc_every_s(3000, 30000, False, LAND) + ["P 30000"])
assert out == ["NONE"], out

# 7) FC heartbeat stale (>3 s) -> nothing
out = run(lost + fc_every_s(3000, 12000, True, LAND) + ["P 15500"])
assert out == ["NONE"], out

# 8) leaving LAND resets the LAND timer
out = run(lost + fc_every_s(3000, 8000, True, LAND) + fc_every_s(9000, 9000, True, GUIDED)
          + fc_every_s(10000, 17000, True, LAND) + ["P 17000", "P 19000"] + fc_every_s(18000, 19000, True, LAND))
assert out[0] == "NONE", out

# 9) link returns -> stops
out = run(lost + fc_every_s(3000, 12000, True, LAND) + ["P 12000", f"G 12500 {gcs_hb()}"]
          + fc_every_s(13000, 13000, True, LAND) + ["P 13600"])
assert out[0].startswith("OUT ") and out[1] == "NONE", out

# 10) corrupted GCS heartbeat does not count as link alive
bad = bytearray.fromhex(gcs_hb())
bad[-1] ^= 0xFF
out = run(lost + [f"G 6000 {bad.hex()}"] + fc_every_s(3000, 12000, True, LAND) + ["P 12000"])
assert out[0].startswith("OUT "), out

# 11) heartbeats mixed with noise and other messages still parsed
noise = "fe0102030405" + enc(1, 1, mav.MAVLink_attitude_message(1, 0.1, 0.2, 0.3, 0, 0, 0))
out = run(lost + [f"F {t} {noise}{fc_hb(True, LAND)}{noise}" for t in range(3000, 12001, 1000)] + ["P 12000"])
assert out[0].startswith("OUT "), out

# 12) never connected since boot -> nothing
out = run(fc_every_s(0, 20000, True, LAND) + ["P 20000"])
assert out == ["NONE"], out

print("link_failsafe: all ok")
