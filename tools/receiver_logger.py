#!/usr/bin/env python3
"""Capture receiver-side RX/RSSI/SNR evidence from LINK_TEST_V2 firmware."""

import argparse
import csv
import time
from pathlib import Path

import serial


FIELDS = (
    "host_wall_ns",
    "host_monotonic_ns",
    "event",
    "device_uptime_ms",
    "seq",
    "rssi_dbm",
    "snr_db",
    "tx_duration_us",
    "raw_line",
)


def wall_ns() -> int:
    # Raspberry Pi images with Python 3.6 do not provide time.time_ns().
    if hasattr(time, "time_ns"):
        return time.time_ns()
    return int(time.time() * 1_000_000_000)


def monotonic_ns() -> int:
    if hasattr(time, "monotonic_ns"):
        return time.monotonic_ns()
    return int(time.monotonic() * 1_000_000_000)


def parse_line(raw: str) -> dict:
    row = {name: "" for name in FIELDS}
    row.update({
        "host_wall_ns": wall_ns(),
        "host_monotonic_ns": monotonic_ns(),
        "raw_line": raw,
    })
    parts = [part.strip() for part in raw.split(",")]
    if not parts:
        return row
    row["event"] = parts[0].upper()
    try:
        if parts[0].upper() == "RX" and len(parts) == 5:
            row.update({
                "device_uptime_ms": int(parts[1]),
                "seq": int(parts[2]),
                "rssi_dbm": float(parts[3]),
                "snr_db": float(parts[4]),
            })
        elif parts[0].upper() == "ACK_TX" and len(parts) == 4:
            row.update({
                "device_uptime_ms": int(parts[1]),
                "seq": int(parts[2]),
                "tx_duration_us": int(parts[3]),
            })
    except ValueError:
        row["event"] = "PARSE_ERROR"
    return row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=Path, default=Path("receiver_serial_log.csv"))
    parser.add_argument("--duration", type=float, default=0,
                        help="seconds; zero records until Ctrl-C")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    with serial.Serial(args.port, args.baud, timeout=0.2) as port, args.output.open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        print(f"Recording receiver evidence from {args.port} to {args.output}")
        try:
            while deadline is None or time.monotonic() < deadline:
                data = port.readline()
                if not data:
                    continue
                raw = data.decode("ascii", errors="replace").rstrip("\r\n")
                row = parse_line(raw)
                writer.writerow(row)
                handle.flush()
                if row["event"] in {"RX", "ACK_TX"}:
                    print(raw, flush=True)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
