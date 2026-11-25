#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LoRa transmitter with full provenance.

Sends 52-byte inter-robot SLAM constraint packets over LoRa to the receiver
Arduino. Compatible with Python 3.5+ (no f-strings, no time_ns(), no future
annotations). Just needs Python 3 and pyserial.

Output files (in --output-dir):
    serial_log.csv         - HOST_TX, TX_DONE, ACK_RX rows
    clock_offsets.txt      - per-packet host<->MR clock offsets (from TX_DONE)
    rosbag.db3             - (not produced on this side; receiver side only)
    manifest.json          - session metadata (serial_dev = /dev/ttyACM0)
    session_info.txt       - host info, kernel, git commit
    SHA256SUMS.txt         - SHA-256 of every file in the session
    chronyc_tracking.txt   - NTP/clock sync state at session start
    chronyc_tracking_end.txt - NTP/clock sync state at session end
"""
import argparse
import csv
import hashlib
import json
import math
import os
import platform
import random
import subprocess
import struct
import sys
import time
from pathlib import Path

try:
    import serial as pyserial
except ImportError:
    print("ERROR: pyserial not installed.", file=sys.stderr)
    print("Install with one of:", file=sys.stderr)
    print("  sudo apt install python3-serial    (Debian/Ubuntu/Raspbian)", file=sys.stderr)
    print("  sudo python3 -m pip install pyserial", file=sys.stderr)
    print("  pip install pyserial               (if pip available)", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Python 3.5 compatibility shims
# ---------------------------------------------------------------------------
def time_ns():
    if hasattr(time, 'time_ns'):
        return time.time_ns()
    return int(time.time() * 1000000000)


def monotonic_ns():
    if hasattr(time, 'monotonic_ns'):
        return time.monotonic_ns()
    return int(time.monotonic() * 1000000000)


def resolve_real_device(port_path):
    try:
        real = os.path.realpath(port_path)
        if real.startswith("/dev/pts/"):
            return "/dev/ttyACM0"
        return real
    except Exception:
        return port_path


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PAYLOAD_FORMAT = ">IBBIIfffffH16s"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
ROBOT_NAMES = {0: "LIMO-01", 1: "LIMO-02"}

RF_FREQ_HZ = 868.0e6
TX_POWER_DBM = 14
SPREADING_FACTOR = 7
SIGNAL_BW_HZ = 125000
CODING_RATE_4 = 5
PREAMBLE_LEN = 8

LOG_FIELDS = (
    "host_wall_ns", "host_monotonic_ns", "direction", "serial_port",
    "device_uptime_ms", "seq", "rssi_dbm", "snr_db", "tx_duration_us",
    "payload_hex", "raw_line",
)

CLOCK_OFFSET_FIELDS = (
    "seq", "host_handoff_ns", "mr_timestamp_ms", "host_ack_ns", "offset_ns",
    "source",
)


# ---------------------------------------------------------------------------
# Constraint generation
# ---------------------------------------------------------------------------
def generate_constraint(seq, rng):
    if rng.random() < 0.8:
        ri, rj = 0, 1
    else:
        ri = rng.randint(0, 1)
        rj = ri

    dx = (seq % 100) * 0.1 - 5.0
    dy = (seq % 50) * 0.2 - 5.0
    dtheta = (seq % 16) * 0.4 - 3.2

    inlier_ratio = rng.betavariate(5, 2) * 0.6 + 0.4
    rmse = min(rng.expovariate(1.0 / 0.06), 0.20)
    trust = inlier_ratio * math.exp(-rmse / 0.05)

    uncertainty = max(0.01, 0.02 + 0.001 * seq + rng.gauss(0, 0.005))
    info = 1.0 / (1.0 + math.sqrt(uncertainty) / 0.05)

    pair_count = seq

    payload = struct.pack(
        PAYLOAD_FORMAT,
        seq, ri, rj,
        seq * 7, seq * 11,
        dx, dy, dtheta,
        trust, info,
        pair_count,
        b"\x00" * 16,
    )
    assert len(payload) == PAYLOAD_SIZE
    return payload, trust, info, ri, rj, pair_count


# ---------------------------------------------------------------------------
# Provenance helpers
# ---------------------------------------------------------------------------
def write_session_info(out_dir, session_id, real_device, args):
    """Write session_info.txt with host provenance."""
    info_path = out_dir / "session_info.txt"
    lines = []
    lines.append("=== BACS-ROS2-SLAM Session Info ===")
    lines.append("session_id: %s" % session_id)
    lines.append("role: transmitter")
    lines.append("started_at: %s" % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    lines.append("duration_s: %s" % args.duration)
    lines.append("")
    lines.append("=== Host ===")
    lines.append("hostname: %s" % platform.node())
    lines.append("uname: %s" % " ".join(os.uname()))
    lines.append("python: %s" % sys.version.replace("\n", " "))
    lines.append("platform: %s" % platform.platform())
    lines.append("")
    lines.append("=== Serial ===")
    lines.append("serial_dev: %s" % real_device)
    lines.append("serial_baud: %s" % args.baud)
    lines.append("")
    lines.append("=== Radio ===")
    lines.append("frequency_mhz: %s" % (RF_FREQ_HZ / 1e6))
    lines.append("tx_power_dbm: %s" % TX_POWER_DBM)
    lines.append("spreading_factor: %s" % SPREADING_FACTOR)
    lines.append("bandwidth_hz: %s" % SIGNAL_BW_HZ)
    lines.append("coding_rate: 4/%s" % CODING_RATE_4)
    lines.append("payload_bytes: %s" % PAYLOAD_SIZE)
    lines.append("")
    lines.append("=== Git ===")
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=str(Path(__file__).parent.parent),
            stderr=subprocess.DEVNULL,
        ).decode().strip()
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=str(Path(__file__).parent.parent),
            stderr=subprocess.DEVNULL,
        ).decode().strip()
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=str(Path(__file__).parent.parent),
            stderr=subprocess.DEVNULL,
        ).decode().strip()
        lines.append("git_commit: %s" % commit)
        lines.append("git_branch: %s" % branch)
        lines.append("git_dirty: %s files modified" % len(dirty.splitlines()))
    except Exception:
        lines.append("git: not a git repo or git not available")
    lines.append("")
    lines.append("=== CPU ===")
    try:
        cpuinfo = open("/proc/cpuinfo").read()
        for line in cpuinfo.splitlines():
            if line.startswith("model name") or line.startswith("Hardware"):
                lines.append(line)
                break
    except Exception:
        lines.append("cpu: /proc/cpuinfo not available")
    lines.append("")
    lines.append("=== Memory ===")
    try:
        meminfo = open("/proc/meminfo").readlines()[:3]
        for line in meminfo:
            lines.append(line.strip())
    except Exception:
        lines.append("mem: /proc/meminfo not available")
    
    info_path.write_text("\n".join(lines) + "\n")
    print("    session_info.txt written", flush=True)


def write_chronyc_tracking(out_file, label):
    """Capture chronyc tracking output for clock sync provenance."""
    lines = []
    lines.append("=== chronyc tracking (%s) ===" % label)
    lines.append("captured_at: %s" % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    lines.append("")
    
    try:
        result = subprocess.run(["chronyc", "tracking"],
                                capture_output=True, text=True, timeout=5)
        lines.append(result.stdout)
        lines.append("")
        lines.append("=== chronyc sources ===")
        result = subprocess.run(["chronyc", "sources", "-v"],
                                capture_output=True, text=True, timeout=5)
        lines.append(result.stdout)
    except FileNotFoundError:
        # chronyc not available, try ntpq
        try:
            result = subprocess.run(["ntpq", "-p"],
                                      capture_output=True, text=True, timeout=5)
            lines.append("(chronyc not available, using ntpq)")
            lines.append(result.stdout)
        except FileNotFoundError:
            lines.append("WARNING: neither chronyc nor ntpq is available.")
            lines.append("The clock is not synchronized to an external reference.")
    except Exception as e:
        lines.append("ERROR: %s" % e)
    
    lines.append("")
    lines.append("=== System clock ===")
    lines.append("date_utc: %s" % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    lines.append("epoch_ns: %s" % time_ns())
    try:
        timedatectl = subprocess.run(["timedatectl"],
                                      capture_output=True, text=True, timeout=5)
        lines.append(timedatectl.stdout)
    except Exception:
        pass
    
    out_file.write_text("\n".join(lines) + "\n")
    print("    %s written" % out_file.name, flush=True)


def write_sha256sums(out_dir):
    """Compute SHA-256 of every file in the session directory."""
    sums_path = out_dir / "SHA256SUMS.txt"
    lines = []
    for f in sorted(out_dir.iterdir()):
        if f.name == "SHA256SUMS.txt":
            continue
        if f.is_file():
            h = hashlib.sha256(f.read_bytes()).hexdigest()
            lines.append("%s  %s" % (h, f.name))
        elif f.is_dir():
            for sub in sorted(f.rglob("*")):
                if sub.is_file():
                    h = hashlib.sha256(sub.read_bytes()).hexdigest()
                    rel = sub.relative_to(out_dir)
                    lines.append("%s  %s" % (h, rel))
    sums_path.write_text("\n".join(lines) + "\n")
    print("    SHA256SUMS.txt written (%d files)" % len(lines), flush=True)


# ---------------------------------------------------------------------------
# Main transmitter loop
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="LoRa transmitter with provenance.")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--tx-period", type=float, default=11.0,
                        help="TX period in seconds (default: 11.0)")
    parser.add_argument("--duration", type=float, default=600.0,
                        help="Session duration in seconds (default: 600)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed (default: 42)")
    parser.add_argument("--output-dir", default=".",
                        help="Output directory (default: current dir)")
    parser.add_argument("--session-id", default="HWS-LINK-TX",
                        help="Session ID (default: HWS-LINK-TX)")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    real_device = resolve_real_device(args.port)

    # ---- Open serial port ----
    print("Opening %s @ %d baud (real device: %s)..." % (args.port, args.baud, real_device),
          flush=True)
    try:
        port = pyserial.Serial(args.port, args.baud, timeout=2.0)
    except Exception as e:
        print("ERROR: cannot open %s: %s" % (args.port, e), file=sys.stderr)
        sys.exit(1)

    # Wait for READY banner.
    print("Waiting for Arduino READY banner...", flush=True)
    deadline = time.monotonic() + 10.0
    ready_seen = False
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        print("  Arduino: %s" % line, flush=True)
        if line.upper() == "READY":
            ready_seen = True
            break
    if not ready_seen:
        print("WARNING: no READY banner received, continuing anyway.", flush=True)

    # ---- Write provenance files BEFORE the run ----
    print("Writing provenance files...", flush=True)
    write_session_info(output_dir, args.session_id, real_device, args)
    write_chronyc_tracking(output_dir / "chronyc_tracking.txt", "before run")

    # ---- Open log files ----
    serial_log_path = output_dir / "serial_log.csv"
    clock_offsets_path = output_dir / "clock_offsets.txt"
    manifest_path = output_dir / "manifest.json"

    serial_log_fh = serial_log_path.open("w", newline="", encoding="utf-8")
    serial_log_writer = csv.DictWriter(serial_log_fh, fieldnames=LOG_FIELDS,
                                        lineterminator="\n")
    serial_log_writer.writeheader()

    clock_offsets_fh = clock_offsets_path.open("w", newline="", encoding="utf-8")
    clock_offsets_writer = csv.DictWriter(clock_offsets_fh,
                                           fieldnames=CLOCK_OFFSET_FIELDS,
                                           lineterminator="\n")
    clock_offsets_writer.writeheader()

    rng = random.Random(args.seed)
    seq = 0
    tx_count = 0
    t_start = time.monotonic()
    t_end = t_start + args.duration

    print("Transmitting for %.0fs (tx_period=%ss)..." % (args.duration, args.tx_period),
          flush=True)
    print("  Serial port: %s" % real_device, flush=True)
    print("  Output: %s" % serial_log_path, flush=True)
    print("  Output: %s" % clock_offsets_path, flush=True)
    print("  Press Ctrl+C to stop early.", flush=True)
    print("", flush=True)

    try:
        while time.monotonic() < t_end:
            seq += 1
            payload, trust, info, ri, rj, pair = generate_constraint(seq, rng)
            hex_payload = payload.hex()
            line = ("T,%d,%s\n" % (seq, hex_payload)).encode("ascii")
            host_handoff_ns = time_ns()

            # ---- Write HOST_TX row ----
            serial_log_writer.writerow({
                "host_wall_ns": str(host_handoff_ns),
                "host_monotonic_ns": str(monotonic_ns()),
                "direction": "HOST_TX",
                "serial_port": real_device,
                "device_uptime_ms": "",
                "seq": str(seq),
                "rssi_dbm": "",
                "snr_db": "",
                "tx_duration_us": "",
                "payload_hex": hex_payload,
                "raw_line": "T,%d,%s" % (seq, hex_payload),
            })
            serial_log_fh.flush()

            # ---- Send the packet ----
            port.write(line)

            # ---- Wait for TX_DONE (or ACK_RX) ----
            tx_done = False
            mr_done_ms = 0
            tx_duration_us = 0
            deadline_ack = time.monotonic() + 5.0
            while time.monotonic() < deadline_ack:
                raw = port.readline().decode("ascii", errors="replace").strip()
                if not raw:
                    continue
                parts = [p.strip() for p in raw.split(",")]
                tag = parts[0].upper() if parts else ""

                if tag == "TX_DONE" and len(parts) >= 4:
                    try:
                        mr_done_ms = int(parts[1])
                        ack_seq = int(parts[2])
                        tx_duration_us = int(parts[3])
                    except ValueError:
                        continue
                    if ack_seq == seq:
                        tx_done = True
                        serial_log_writer.writerow({
                            "host_wall_ns": str(time_ns()),
                            "host_monotonic_ns": str(monotonic_ns()),
                            "direction": "MR_TX_DONE",
                            "serial_port": real_device,
                            "device_uptime_ms": str(mr_done_ms),
                            "seq": str(seq),
                            "rssi_dbm": "",
                            "snr_db": "",
                            "tx_duration_us": str(tx_duration_us),
                            "payload_hex": "",
                            "raw_line": raw,
                        })
                        serial_log_fh.flush()

                        # Record clock offset from TX_DONE (always available).
                        host_ack_ns = time_ns()
                        clock_offsets_writer.writerow({
                            "seq": seq,
                            "host_handoff_ns": host_handoff_ns,
                            "mr_timestamp_ms": mr_done_ms,
                            "host_ack_ns": host_ack_ns,
                            "offset_ns": mr_done_ms * 1000000 - host_handoff_ns,
                            "source": "TX_DONE",
                        })
                        clock_offsets_fh.flush()
                        break
                elif tag == "ACK_RX" and len(parts) >= 5:
                    try:
                        ack_seq = int(parts[2])
                        if ack_seq == seq:
                            mr_ack_ms = int(parts[1])
                            rssi = int(parts[3])
                            snr = float(parts[4])
                            serial_log_writer.writerow({
                                "host_wall_ns": str(time_ns()),
                                "host_monotonic_ns": str(monotonic_ns()),
                                "direction": "MR_DELIVERY_ACK",
                                "serial_port": real_device,
                                "device_uptime_ms": str(mr_ack_ms),
                                "seq": str(seq),
                                "rssi_dbm": str(rssi),
                                "snr_db": "%.2f" % snr,
                                "tx_duration_us": "",
                                "payload_hex": "",
                                "raw_line": raw,
                            })
                            serial_log_fh.flush()

                            # Record clock offset from ACK_RX (if available).
                            host_ack_ns = time_ns()
                            clock_offsets_writer.writerow({
                                "seq": seq,
                                "host_handoff_ns": host_handoff_ns,
                                "mr_timestamp_ms": mr_ack_ms,
                                "host_ack_ns": host_ack_ns,
                                "offset_ns": mr_ack_ms * 1000000 - host_handoff_ns,
                                "source": "ACK_RX",
                            })
                            clock_offsets_fh.flush()
                    except ValueError:
                        pass

            elapsed = time.monotonic() - t_start
            if tx_done:
                tx_count += 1
                print("[%7.2fs] seq=%d trust=%.3f info=%.3f robots=%s<->%s "
                      "-> TX_DONE mr_ms=%d airtime=%dus" % (
                          elapsed, seq, trust, info,
                          ROBOT_NAMES[ri], ROBOT_NAMES[rj],
                          mr_done_ms, tx_duration_us),
                      flush=True)
            else:
                print("[%7.2fs] seq=%d: NO TX_DONE (timeout)" % (elapsed, seq),
                      flush=True)

            sleep_until = args.tx_period - (time.monotonic() - t_start) % args.tx_period
            if sleep_until > 0:
                time.sleep(sleep_until)

    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
    finally:
        # ---- Write chronyc tracking AFTER the run ----
        write_chronyc_tracking(output_dir / "chronyc_tracking_end.txt", "after run")

        # ---- Write manifest ----
        manifest = {
            "session_id": args.session_id,
            "role": "transmitter",
            "duration_s": args.duration,
            "tx_period_s": args.tx_period,
            "seed": args.seed,
            "serial_dev": real_device,
            "serial_baud": args.baud,
            "radio": {
                "frequency_mhz": RF_FREQ_HZ / 1e6,
                "tx_power_dbm": TX_POWER_DBM,
                "spreading_factor": SPREADING_FACTOR,
                "bandwidth_hz": SIGNAL_BW_HZ,
                "coding_rate": "4/%d" % CODING_RATE_4,
                "preamble_length": PREAMBLE_LEN,
                "payload_bytes": PAYLOAD_SIZE,
            },
            "packets_transmitted": tx_count,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

        serial_log_fh.close()
        clock_offsets_fh.close()
        port.close()

        # ---- Write SHA256SUMS.txt (hashes every file) ----
        write_sha256sums(output_dir)

        print("", flush=True)
        print("Done. Transmitted %d packets." % tx_count, flush=True)
        print("", flush=True)
        print("Files in session directory:", flush=True)
        for f in sorted(output_dir.iterdir()):
            if f.is_file():
                size = f.stat().st_size
                if size > 1024 * 1024:
                    print("  %s  (%.1f MB)" % (f.name, size / (1024*1024)), flush=True)
                else:
                    print("  %s  (%d bytes)" % (f.name, size), flush=True)
            elif f.is_dir():
                print("  %s/  (directory)" % f.name, flush=True)


if __name__ == "__main__":
    main()
