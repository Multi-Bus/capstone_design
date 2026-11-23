"""Compare tag clocks from HEAR3 lines (every anchor Poll each tag heard).

All tags hear the same anchor Poll at nearly the same instant (ns apart), so one Poll is a common event.
For each Poll a tag logs its DW3000 RX timestamp (40 bit, 1/(499.2 MHz x 128)), its MCU cycle counter
(DWT->CYCCNT, read right after the RX event was polled) and the DW clock offset to the anchor.

Outputs, in the capture folder:
  clock_tags.csv            per tag: MCU clock vs its own DW3000 clock (ppm)
  clock_anchor_offsets.csv  per tag and anchor: DW3000 clock offset reported by the receiver (ppm)
  clock_pairs.csv           per tag pair: relative drift (ppm), fit residual and next-event prediction error,
                            for the MCU clock (us) and the DW3000 clock (ns)
  clock.json                the same, plus settings
"""

import argparse
import csv
from datetime import datetime
from itertools import combinations
import json
import math
from pathlib import Path
import re

DW_HZ = 499.2e6 * 128
DW_WRAP = 1 << 40
CYC_WRAP = 1 << 32
OFFSET_SCALE = 1e6 / (1 << 26)  # dwt_readclockoffset() -> ppm; positive: this receiver is slower than the anchor
WINDOW = 10  # events used to predict the next one


def parse_log(path, session):
    """Return (cpu_hz, events) where events are in log order: dict with key, host, rx, cyc, offset."""
    cpu_hz = None
    events = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            host, _, text = line.rstrip("\n").partition("\t")
            ready = re.match(r"TWR3_READY,.*cpu_hz=(\d+)", text)
            if ready:
                cpu_hz = int(ready[1])
                continue
            if not text.startswith("HEAR3,"):
                continue
            columns = text.split(",")
            if len(columns) != 8:
                continue
            try:
                values = [int(column) for column in columns[1:5]]
                rx, cyc, offset = int(columns[5], 16), int(columns[6], 16), int(columns[7])
                host_s = datetime.fromisoformat(host).timestamp()
            except ValueError:
                continue
            if values[0] != session:
                continue
            events.append({"key": tuple(values[1:]), "anchor": values[3], "host": host_s,
                           "rx": rx, "cyc": cyc, "offset": offset})
    return cpu_hz, events


def unwrap(events, field, wrap, hz):
    """Turn a wrapping counter into seconds. The host receive time decides how many wraps passed,
    which works while host timing error stays far below the wrap period (7.2 s at 600 MHz, 17.2 s for DW)."""
    seconds = []
    total = None
    for index, event in enumerate(events):
        if total is None:
            total = event[field]
        else:
            delta = (event[field] - events[index - 1][field]) % wrap
            host_dt = event["host"] - events[index - 1]["host"]
            total += delta + max(0, round((host_dt * hz - delta) / wrap)) * wrap
        seconds.append(total / hz)
    return seconds


def fit(xs, ds, groups):
    """d = c_group + slope * x by least squares. Returns slope, per-group intercepts, residuals."""
    members = {}
    for x, d, group in zip(xs, ds, groups):
        members.setdefault(group, []).append((x, d))
    means = {group: (sum(x for x, _ in pairs) / len(pairs), sum(d for _, d in pairs) / len(pairs))
             for group, pairs in members.items()}
    sxx = sum((x - means[g][0]) ** 2 for x, g in zip(xs, groups))
    sxd = sum((x - means[g][0]) * (d - means[g][1]) for x, d, g in zip(xs, ds, groups))
    if sxx == 0:
        raise ValueError("not enough spread in time to fit a drift")
    slope = sxd / sxx
    intercepts = {group: mean_d - slope * mean_x for group, (mean_x, mean_d) in means.items()}
    residuals = [d - intercepts[g] - slope * x for x, d, g in zip(xs, ds, groups)]
    return slope, intercepts, residuals


def predict_errors(xs, ds):
    """Fit offset and drift on the previous WINDOW events, predict the next one. This is what a node that
    re-synchronizes on every received Poll could achieve with that clock."""
    errors = []
    for index in range(WINDOW, len(xs)):
        window_x, window_d = xs[index - WINDOW:index], ds[index - WINDOW:index]
        slope, intercepts, _ = fit(window_x, window_d, [0] * WINDOW)
        errors.append(ds[index] - intercepts[0] - slope * xs[index])
    return errors


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def compare(xs, ys, groups, scale):
    """Relative drift of clock y against clock x over common events (same Poll)."""
    ds = [y - x for x, y in zip(xs, ys)]
    slope, intercepts, residuals = fit(xs, ds, groups)
    flat = [d - intercepts[g] + intercepts[groups[0]] for d, g in zip(ds, groups)]  # remove per-anchor offsets
    predicted = predict_errors(xs, flat)
    return {"drift_ppm": slope * 1e6, "fit_rms": rms(residuals) * scale,
            "fit_max": max(abs(r) for r in residuals) * scale,
            "predict_rms": rms(predicted) * scale if predicted else None,
            "predict_max": max(abs(e) for e in predicted) * scale if predicted else None}


def analyze(out, session=None):
    out = Path(out)
    if session is None:
        session = json.loads((out / "run.json").read_text(encoding="utf-8"))["session"]
    tags = {}
    for path in sorted(out.glob("tag*.raw.log")):
        tag = int(re.fullmatch(r"tag(\d+)\.raw\.log", path.name)[1])
        cpu_hz, events = parse_log(path, session)
        if len(events) < WINDOW + 2 or not cpu_hz:
            continue
        dw = unwrap(events, "rx", DW_WRAP, DW_HZ)
        mcu = unwrap(events, "cyc", CYC_WRAP, cpu_hz)
        tags[tag] = {"cpu_hz": cpu_hz, "events": events,
                     "by_key": {event["key"]: (d, m) for event, d, m in zip(events, dw, mcu)}}
    if not tags:
        raise ValueError("no tag log with HEAR3 lines and a TWR3_READY cpu_hz line")

    tag_rows, offset_rows, pair_rows = [], [], []
    for tag, data in sorted(tags.items()):
        dw = [data["by_key"][event["key"]][0] for event in data["events"]]
        mcu = [data["by_key"][event["key"]][1] for event in data["events"]]
        own = compare(dw, mcu, [0] * len(dw), 1e6)
        tag_rows.append({"tag": tag, "cpu_hz": data["cpu_hz"], "events": len(dw),
                         "span_s": dw[-1] - dw[0], "mcu_vs_own_dw_ppm": own["drift_ppm"],
                         "fit_rms_us": own["fit_rms"]})
        for anchor in sorted({event["anchor"] for event in data["events"]}):
            values = [event["offset"] * OFFSET_SCALE for event in data["events"] if event["anchor"] == anchor]
            mean = sum(values) / len(values)
            offset_rows.append({"tag": tag, "anchor": anchor, "events": len(values), "dw_offset_ppm": mean,
                                "std_ppm": math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))})

    for left, right in combinations(sorted(tags), 2):
        keys = sorted(set(tags[left]["by_key"]) & set(tags[right]["by_key"]),
                      key=lambda key: tags[left]["by_key"][key][0])
        if len(keys) < WINDOW + 2:
            continue
        groups = [key[2] for key in keys]
        pick = lambda tag, index: [tags[tag]["by_key"][key][index] for key in keys]
        gaps = sorted(b - a for a, b in zip(pick(left, 0), pick(left, 0)[1:]))
        mcu = compare(pick(left, 1), pick(right, 1), groups, 1e6)
        dw = compare(pick(left, 0), pick(right, 0), groups, 1e9)
        pair_rows.append({"tag_a": left, "tag_b": right, "common_events": len(keys),
                          "median_gap_s": gaps[len(gaps) // 2],
                          "mcu_drift_ppm": mcu["drift_ppm"], "mcu_fit_rms_us": mcu["fit_rms"],
                          "mcu_fit_max_us": mcu["fit_max"], "mcu_predict_rms_us": mcu["predict_rms"],
                          "mcu_predict_max_us": mcu["predict_max"],
                          "dw_drift_ppm": dw["drift_ppm"], "dw_fit_rms_ns": dw["fit_rms"],
                          "dw_fit_max_ns": dw["fit_max"], "dw_predict_rms_ns": dw["predict_rms"],
                          "dw_predict_max_ns": dw["predict_max"]})

    for name, rows in (("clock_tags.csv", tag_rows), ("clock_anchor_offsets.csv", offset_rows),
                       ("clock_pairs.csv", pair_rows)):
        if rows:
            with (out / name).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    result = {"session": session, "window_events": WINDOW, "tags": tag_rows, "anchor_offsets": offset_rows,
              "pairs": pair_rows,
              "notes": ["drift_ppm > 0: tag_b clock runs fast relative to tag_a (mcu: MCU cycle counter, "
                        "dw: DW3000 RX timestamp)",
                        "offset-only sync error grows by |drift_ppm| microseconds per second without re-sync",
                        "predict_*: error of the next Poll time predicted from the previous window_events "
                        "(offset + drift re-sync on every Poll)"]}
    (out / "clock.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", type=Path, help="capture folder with run.json and tag*.raw.log")
    args = parser.parse_args()
    try:
        result = analyze(args.out)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"{error}\n")
    for row in result["pairs"]:
        print(f"tag {row['tag_a']}-{row['tag_b']}: MCU {row['mcu_drift_ppm']:+.2f} ppm, "
              f"predict rms {row['mcu_predict_rms_us']:.2f} us | DW {row['dw_drift_ppm']:+.3f} ppm, "
              f"predict rms {row['dw_predict_rms_ns']:.2f} ns")
