"""Read-only: list and download dataflash logs from FC over USB (MAVLink LOG_* protocol).
usage: fc_logdl.py <port> list | fc_logdl.py <port> get <id> <out.bin>"""
import sys
import time

from pymavlink import mavutil

port, cmd = sys.argv[1], sys.argv[2]
m = mavutil.mavlink_connection(port, baud=115200)
m.wait_heartbeat(timeout=10)
ts, tc = m.target_system, 1

if cmd == "list":
    m.mav.log_request_list_send(ts, tc, 0, 0xFFFF)
    ents, total, t0 = {}, None, time.time()
    while time.time() - t0 < 15:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=2)
        if e is None:
            if total is not None and len(ents) >= total:
                break
            continue
        total = e.num_logs
        ents[e.id] = e
        if len(ents) >= total:
            break
    for i in sorted(ents):
        e = ents[i]
        print(f"id {e.id:3d}  size {e.size:9d}  utc {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(e.time_utc)) if e.time_utc else '-'}  last_id {e.last_log_num}")
    print("total", total)
else:
    lid, out = int(sys.argv[3]), sys.argv[4]
    m.mav.log_request_list_send(ts, tc, lid, lid)
    e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=5)
    size = e.size
    buf = bytearray(size)
    got = set()
    pos = 0
    t0 = time.time()
    m.mav.log_request_data_send(ts, tc, lid, 0, size)
    last = time.time()
    while len(got) * 90 < size and time.time() - t0 < 300:
        d = m.recv_match(type="LOG_DATA", blocking=True, timeout=2)
        if d is None:
            # re-request the first missing chunk
            miss = next((o for o in range(0, size, 90) if o not in got), None)
            if miss is None:
                break
            m.mav.log_request_data_send(ts, tc, lid, miss, min(size - miss, 90 * 200))
            continue
        if d.id != lid:
            continue
        n = d.count
        buf[d.ofs:d.ofs + n] = bytes(d.data[:n])
        got.add(d.ofs)
        if len(got) % 2000 == 0:
            print("progress", len(got) * 90, "/", size, flush=True)
    miss = [o for o in range(0, size, 90) if o not in got]
    open(out, "wb").write(buf)
    m.mav.log_request_end_send(ts, tc)
    print("saved", out, "size", size, "missing chunks", len(miss))
