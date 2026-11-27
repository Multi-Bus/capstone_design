"""Join a single flight's --floor-height attitude CSV to its matching FC DataFlash log."""
import argparse
import csv
from bisect import bisect_right
from pathlib import Path

from pymavlink import mavutil


def join(attitude: Path, dataflash: Path, output: Path) -> None:
    fc = mavutil.mavlink_connection(str(dataflash))
    ticks, messages = [], []
    try:
        while (msg := fc.recv_match(type="CTUN")) is not None:
            ticks.append(msg.TimeUS / 1000)
            messages.append(msg)
    finally:
        fc.close()
    if not ticks:
        raise ValueError("FC 로그에 CTUN 없음")
    with attitude.open(encoding="utf-8-sig", newline="") as src:
        rows = csv.DictReader(src)
        if not {"fc_att_boot_ms", "decision_floor_m", "fc_takeoff_frame"} <= set(rows.fieldnames or ()):
            raise ValueError("--floor-height 시험 CSV 필요")
        extra = ["ctun_sample_age_ms", "fc_desired_origin_m", "fc_alt_m", "fc_baro_m", "fc_surface_m"]
        with output.open("x", encoding="utf-8", newline="") as dst:
            writer = csv.DictWriter(dst, [*rows.fieldnames, *extra])
            writer.writeheader()
            for row in rows:
                stamp = int(row["fc_att_boot_ms"])
                i = bisect_right(ticks, stamp) - 1
                if i >= 0 and stamp - ticks[i] <= 250:
                    msg = messages[i]
                    row.update(zip(extra, [round(stamp - ticks[i], 2), msg.DAlt, msg.Alt, msg.BAlt, msg.SAlt]))
                writer.writerow(row)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("attitude", "dataflash", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    join(args.attitude, args.dataflash, args.output)
