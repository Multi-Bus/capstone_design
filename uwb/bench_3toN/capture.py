"""Run serialized DS-TWR exchanges (3 anchors x N tags per round) and save packet CSV and schedule checks."""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from hashlib import sha256
import json
from pathlib import Path
import re
import secrets
import subprocess
from threading import Event, Thread
import time

import clock_sync


HERE = Path(__file__).resolve().parent
MAGIC = 0x55574233
ENTRY = 0x20004000
MAILBOX_WORDS = 16
PACKETS = ("poll", "response", "final")
ANCHORS = (1, 2, 3)
RX_TIMEOUT = 0x00220000  # DW3110 RXPTO | RXFTO
OTHER_RESPONSE = 8  # anchor.c: a valid Response for another tag, session or round


def now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def words_in_dump(text, address, count):
    words = {}
    for match in re.finditer(r"^(?:J-Link>\s*)?([0-9A-Fa-f]{8})\s*=\s*((?:[0-9A-Fa-f]{8}\s*)+)$", text, re.M):
        start = int(match[1], 16)
        for index, word in enumerate(match[2].split()):
            words[start + 4 * index] = int(word, 16)
    return [words[address + 4 * index] for index in range(count)]


@dataclass
class Probe:
    address: int
    serial: str
    jlink: str
    build: Path

    def __post_init__(self):
        folder = self.build / f"A{self.address}"
        self.image = (folder / "cdk-initiator.bin").resolve()
        self.manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8-sig"))
        if (self.manifest["anchor_id"] != self.address or self.manifest["channel"] != 5 or
                self.manifest["image_sha256"] != sha256(self.image.read_bytes()).hexdigest()):
            raise ValueError(f"A{self.address} image does not match its manifest")
        link_map = (folder / "cdk-initiator.map").read_text(encoding="utf-8", errors="replace")
        symbols = {name: int(value, 16) for value, name in re.findall(
            r"^\s*(0x[0-9a-fA-F]+)[ \t]+(start|mailbox)[ \t]*$", link_map, re.M
        )}
        self.mailbox = symbols["mailbox"]
        if (symbols["start"] != ENTRY or
                not ENTRY + self.image.stat().st_size <= self.mailbox <= 0x20020000 - 4 * MAILBOX_WORDS):
            raise ValueError(f"A{self.address} has unexpected RAM image addresses")

    def call(self, *commands):
        result = subprocess.run(
            [self.jlink, "-USB", self.serial, "-Device", "NRF52833_XXAA", "-If", "SWD",
             "-Speed", "4000", "-AutoConnect", "1", "-NoGui", "1"],
            input="\n".join(("ExitOnError 1", *commands, "exit", "")),
            capture_output=True, text=True, errors="replace", timeout=20,
        )
        if result.returncode != 0:
            raise RuntimeError(f"A{self.address} J-Link failed: {result.stdout[-1000:]}")
        return result.stdout

    def read(self, *commands):
        report = self.call(*commands, f"mem32 0x{self.mailbox:08X} 0x{MAILBOX_WORDS:X}")
        try:
            words = words_in_dump(report, self.mailbox, MAILBOX_WORDS)
        except KeyError as error:
            raise RuntimeError(f"A{self.address} mailbox could not be read: {report[-1000:]}") from error
        if words[:3] != [MAGIC, self.address, 1]:
            raise RuntimeError(f"A{self.address} is not ready or has the wrong image: {words[:3]}")
        return words

    def load(self):
        self.read("r", f'loadbin "{self.image}", 0x{ENTRY:X}', f'verifybin "{self.image}", 0x{ENTRY:X}',
                  f"SetPC 0x{ENTRY:X}", "g", "Sleep 100")

    def range_once(self, session, round_number, tag, guard_ms, token):
        timestamp = now()
        start = time.perf_counter()
        # The anchor starts when request_token (word 5) changes, so it is written last.
        # One anchor is called N times per round, so the token is a run-wide counter, not the round number.
        words = self.read(
            f"w4 0x{self.mailbox + 12:08X} 0x{session:08X}",
            f"w4 0x{self.mailbox + 16:08X} 0x{round_number:08X}",
            f"w4 0x{self.mailbox + 56:08X} 0x{tag:08X}",
            f"w4 0x{self.mailbox + 20:08X} 0x{token:08X}",
            f"Sleep {guard_ms}",
        )
        end = time.perf_counter()
        if words[6:9] != [token, session, round_number] or words[15] != tag:
            raise RuntimeError(f"A{self.address} did not acknowledge round {round_number} tag {tag}: "
                               f"{words[6:9]}, tag {words[15]}")
        return {"time": timestamp, "start_s": start, "end_s": end, "anchor": self.address, "tag": tag,
                "round": round_number, "stages": words[9], "response_status": words[10], "final_status": words[11],
                "response_rssi": signed32(words[12]), "response_fp": signed32(words[13])}


def signed32(word):
    return word - (1 << 32) if word & (1 << 31) else word


def tag_records(lines, session, port_tag):
    """Parse one tag port's TWR3 lines. Returns (records, anomalies). Anomalies are counted, not raised,
    so a schedule overlap still produces a summary."""
    records = {}
    anomalies = {"foreign_records": 0, "duplicate_records": 0, "invalid_records": 0}
    for line in lines:
        text = line.strip().split("\t", 1)[-1]
        if not text.startswith("TWR3,"):
            continue
        columns = text.split(",")
        try:
            if len(columns) != 15:
                raise ValueError
            values = [int(column) for column in columns[1:13]] + [int(column, 16) for column in columns[13:]]
        except ValueError:
            anomalies["invalid_records"] += 1
            continue
        tag_session, round_number, tag, anchor, *event = values
        if tag_session != session:
            continue
        if tag != port_tag:
            anomalies["foreign_records"] += 1
            continue
        key = (round_number, tag, anchor)
        if anchor not in ANCHORS or round_number < 1:
            anomalies["invalid_records"] += 1
        elif key in records:
            anomalies["duplicate_records"] += 1
        else:
            records[key] = event
    return records, anomalies


def dbm(value):
    return "" if value == 32767 else f"{value / 256:.2f}"


def packet_rows(attempt, tag):
    response_sent, final_rx, distance_valid, distance, poll_rssi, poll_fp, final_rssi, final_fp, poll_status, final_status = (
        tag if tag is not None else (0, 0, 0, 0, 32767, 32767, 32767, 32767, 0, 0)
    )
    stages = attempt["stages"]
    return (
        ("poll", bool(stages & 1), tag is not None, poll_rssi, poll_fp, "", poll_status),
        ("response", bool(response_sent), bool(stages & 2), attempt["response_rssi"], attempt["response_fp"], "",
         attempt["response_status"]),
        ("final", bool(stages & 4), bool(final_rx), final_rssi, final_fp,
         f"{distance / 1000:.3f}" if distance_valid else "", final_status),
    )


def max_run(flags):
    longest = run = 0
    for failed in flags:
        run = run + 1 if failed else 0
        longest = max(longest, run)
    return longest


def timing(attempts, tags):
    """Host-side schedule timing. Calls are serialized, so overlapping_calls must be zero."""
    overlaps = sum(1 for left, right in zip(attempts, attempts[1:]) if right["start_s"] < left["end_s"])
    durations = [attempt["end_s"] - attempt["start_s"] for attempt in attempts]
    update = {}
    for tag in tags:
        starts = [attempt["start_s"] for attempt in attempts if attempt["tag"] == tag and attempt["anchor"] == 1]
        gaps = [right - left for left, right in zip(starts, starts[1:])]
        update[str(tag)] = sum(gaps) / len(gaps) if gaps else None
    return {"overlapping_calls": overlaps,
            "mean_call_s": sum(durations) / len(durations) if durations else None,
            "max_call_s": max(durations) if durations else None,
            "mean_update_interval_s": update}


def write_results(out, session, attempts, records, tags, planned_rounds, anomalies, capture_error=None):
    counts = {str(tag): {str(anchor): {packet: {"expected": 0, "sent": 0, "received": 0, "valid": 0,
                                                 "lost_after_tx": 0} for packet in PACKETS} for anchor in ANCHORS}
              for tag in tags}
    successes = {}
    requested = set()
    other_responses = 0
    with (out / "packets.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("timestamp", "session", "round", "tag_id", "anchor_id", "packet", "tx_ok", "rx_ok",
                         "success", "status", "rssi_dbm", "first_path_dbm", "distance_m", "rx_sys_status"))
        for attempt in attempts:
            anchor, tag_id, round_number = attempt["anchor"], attempt["tag"], attempt["round"]
            requested.add((round_number, tag_id, anchor))
            other_responses += bool(attempt["stages"] & OTHER_RESPONSE)
            tag = records.get((round_number, tag_id, anchor))
            exchange_ok = tag is not None and bool(tag[2])
            for packet, sent, received, rssi, first_path, distance, status in packet_rows(attempt, tag):
                success = sent and received
                exchange_ok &= success
                counters = counts[str(tag_id)][str(anchor)][packet]
                counters["expected"] += 1
                counters["sent"] += sent
                counters["received"] += received
                counters["valid"] += success
                counters["lost_after_tx"] += sent and not received
                if success:
                    outcome = "ok"
                elif tag is None and packet != "response":
                    outcome = "no_tag_record"
                elif not sent:
                    outcome = "no_tx"
                else:
                    outcome = "rx_timeout" if status & RX_TIMEOUT else "not_received"
                writer.writerow((attempt["time"], session, round_number, tag_id, anchor, packet, int(sent),
                                 int(received), int(success), outcome, dbm(rssi), dbm(first_path), distance,
                                 f"0x{status:08X}"))
            successes.setdefault((round_number, tag_id), {})[anchor] = exchange_ok
    with (out / "rounds.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("session", "round", "tag_id", "a1_distance_m", "a2_distance_m", "a3_distance_m", "status"))
        for (round_number, tag_id), results in successes.items():
            distances = [f"{records[(round_number, tag_id, anchor)][3] / 1000:.3f}" if results.get(anchor) else ""
                         for anchor in ANCHORS]
            status = "partial" if len(results) != 3 else "ok" if all(results.values()) else "failed"
            writer.writerow((session, round_number, tag_id, *distances, status))
    per_tag = {}
    for tag_id in tags:
        complete = [all(row.values()) for (_, tag), row in successes.items() if tag == tag_id and len(row) == 3]
        per_tag[str(tag_id)] = {"complete_rounds": len(complete), "successful_rounds": sum(complete),
                                "all_three_ranges_rate": sum(complete) / len(complete) if complete else None,
                                "max_consecutive_failed_rounds": max_run(not ok for ok in complete),
                                "tag_records": sum(1 for key in records if key[1] == tag_id)}
        for packets in counts[str(tag_id)].values():
            for counters in packets.values():
                sent, expected = counters["sent"], counters["expected"]
                counters["loss_after_tx_rate"] = counters["lost_after_tx"] / sent if sent else None
                counters["miss_rate_per_expected_packet"] = 1 - counters["valid"] / expected if expected else None
    schedule = {"other_tag_responses_at_anchor": other_responses,
                "unrequested_tag_records": sum(1 for key in records if key not in requested),
                **anomalies, **timing(attempts, tags)}
    summary = {"mode": "9-packet x N", "channel": 5, "session": session, "tags": list(tags),
               "planned_rounds": planned_rounds, "schedule_checks": schedule, "per_tag": per_tag,
               "capture_error": capture_error, "packets": counts}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def capture(args):
    import serial  # pyserial is only required for the live bench.

    session = secrets.randbelow(0xFFFFFFFF) + 1
    probes = [Probe(index, serial_id, args.jlink, args.build) for index, serial_id in enumerate(args.probes, 1)]
    tag_shas = {str(tag): sha256(path.read_bytes()).hexdigest() for tag, path in zip(args.tags, args.tag_elfs or [])}
    args.out.mkdir(parents=True)
    (args.out / "run.json").write_text(json.dumps({
        "start_time": now(), "session": session, "mode": "9-packet x N", "channel": 5,
        "rounds": args.rounds, "guard_ms": args.guard_ms, "tags": args.tags,
        "tag_ports": dict(zip(map(str, args.tags), args.tag_ports)),
        "tag_elf_sha256": tag_shas, "anchors": [probe.manifest for probe in probes],
    }, indent=2) + "\n", encoding="utf-8")
    attempts = []
    errors = []
    failure = None
    stop = Event()
    ports = [serial.Serial(port, 115200, timeout=0.1) for port in args.tag_ports]
    raws = [(args.out / f"tag{tag}.raw.log").open("w", encoding="utf-8") for tag in args.tags]

    def read_tag(port, raw):
        try:
            buffer = bytearray()
            while not stop.is_set():
                buffer.extend(port.read(port.in_waiting or 1))
                while b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    raw.write(f"{now()}\t{line.rstrip().decode('utf-8', errors='replace')}\n")
                    raw.flush()
                if len(buffer) > 1024:
                    raise RuntimeError(f"{port.port} stream exceeded the expected line length")
        except (serial.SerialException, OSError, RuntimeError) as error:
            errors.append(error)

    readers = [Thread(target=read_tag, args=pair, daemon=True) for pair in zip(ports, raws)]
    try:
        with (args.out / "anchor.jsonl").open("w", encoding="utf-8") as anchors:
            for reader in readers:
                reader.start()
            for probe in probes:
                probe.load()
            token = 0
            for round_number in range(1, args.rounds + 1):
                for tag in args.tags:
                    for probe in probes:
                        if errors:
                            raise RuntimeError("Tag capture failed") from errors[0]
                        token += 1
                        attempt = probe.range_once(session, round_number, tag, args.guard_ms, token)
                        attempts.append(attempt)
                        anchors.write(json.dumps(attempt) + "\n")
                        anchors.flush()
                if round_number % 25 == 0:
                    print(f"round {round_number}/{args.rounds}", flush=True)
            time.sleep(0.25)
    except (OSError, RuntimeError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        failure = error
    finally:
        stop.set()
        for reader in readers:
            if reader.is_alive():
                reader.join()
        for handle in (*raws, *ports):
            handle.close()
    failure = failure or (errors[0] if errors else None)
    records = {}
    anomalies = {"foreign_records": 0, "duplicate_records": 0, "invalid_records": 0}
    for tag in args.tags:
        with (args.out / f"tag{tag}.raw.log").open(encoding="utf-8") as raw:
            found, problems = tag_records(raw, session, tag)
        records.update(found)
        for name, count in problems.items():
            anomalies[name] += count
    write_results(args.out, session, attempts, records, args.tags, args.rounds, anomalies,
                  f"{type(failure).__name__}: {failure}" if failure else None)
    try:
        clock_sync.analyze(args.out, session)
    except (OSError, ValueError) as error:
        print(f"clock analysis skipped: {error}")
    if failure:
        raise RuntimeError("Capture stopped early. Partial results were saved.") from failure
    if not records:
        raise RuntimeError("No matching tag records. Check the tag firmware, VCP and setup before using the RF totals.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probes", nargs=3, metavar=("A1", "A2", "A3"), required=True)
    parser.add_argument("--tags", nargs="+", type=int, required=True, help="tag numbers to poll, in order (1..6)")
    parser.add_argument("--tag-ports", nargs="+", required=True, help="VCP of each tag, same order as --tags")
    parser.add_argument("--tag-elfs", nargs="+", type=Path, help="record the SHA-256 of each tag ELF (same order)")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--rounds", type=int, default=1000)
    parser.add_argument("--guard-ms", type=int, default=50)
    parser.add_argument("--jlink", default="JLink.exe")
    parser.add_argument("--build", type=Path, default=HERE / "anchor_port" / "build")
    args = parser.parse_args()
    if (not 1 <= args.rounds <= 0xFFFFFFFF or not 25 <= args.guard_ms <= 1000 or
            len(set(args.probes)) != 3 or not all(re.fullmatch(r"[0-9]+", probe) for probe in args.probes)):
        parser.error("supply three distinct numeric J-Link serials, positive rounds, and guard-ms between 25 and 1000")
    if (len(set(args.tags)) != len(args.tags) or not all(1 <= tag <= 6 for tag in args.tags) or
            len(args.tag_ports) != len(args.tags) or len(set(args.tag_ports)) != len(args.tag_ports) or
            (args.tag_elfs and len(args.tag_elfs) != len(args.tags))):
        parser.error("--tags must be distinct numbers 1..6 with one distinct --tag-ports entry each")
    try:
        capture(args)
    except (OSError, KeyError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f"{error}\n")
