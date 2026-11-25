#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LoRa transmitter with BACS scheduler policy selection.

Sends 52-byte inter-robot SLAM constraint packets over LoRa, using the BACS
scheduler to RANK and SELECT which constraint to transmit each tick. Supports
four policies: FIFO, BACS, BACS+, RANDOM. Compatible with Python 3.5+.

The scheduler maintains a pool of constraint candidates (generated every 2s).
On each TX tick (every 11s), it ranks the pool by the active policy and
transmits the highest-ranked constraint, then removes it from the pool.

Wire format (host -> Arduino):
    T,<seq>,<hex_payload>\n

Wire format (Arduino -> host):
    TX_DONE,<timestamp_ms>,<seq>,<tx_duration_us>\n
    ACK_RX,<timestamp_ms>,<seq>,<rssi>,<snr>\n
    RX,<timestamp_ms>,<seq>,<rssi>,<snr>,<hex_payload>\n

Usage:
    python3 pi_scheduler_tx.py --port /dev/ttyACM0 --policy FIFO --duration 600
    python3 pi_scheduler_tx.py --port /dev/ttyACM0 --policy BACS --seed 42
    python3 pi_scheduler_tx.py --port /dev/ttyACM0 --policy BACS+ --session-id HWS-BACS-003
    python3 pi_scheduler_tx.py --port /dev/ttyACM0 --policy RANDOM --seed 42

Output files (in --output-dir):
    serial_log.csv           - HOST_TX, TX_DONE, ACK_RX rows
    clock_offsets.txt        - per-packet host<->MR clock offsets
    scheduler_decisions.csv  - which constraint was selected and why
    manifest.json            - session metadata (includes policy)
    session_info.txt         - host info, kernel, git commit
    SHA256SUMS.txt           - SHA-256 of every file
    chronyc_tracking.txt     - NTP state before run
    chronyc_tracking_end.txt - NTP state after run
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
    print("Install with:  sudo apt install python3-serial", file=sys.stderr)
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
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)   # 52 bytes
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

SCHEDULER_DECISION_FIELDS = (
    "tx_seq", "policy", "constraint_seq", "robot_i", "robot_j",
    "trust", "info", "score", "pool_size_before", "pool_size_after",
    "selected_at_ns", "reason",
)


# ---------------------------------------------------------------------------
# LoRa airtime calculation (Semtech formula, matches scheduler.py)
# ---------------------------------------------------------------------------
def lora_airtime_s(payload_bytes, spreading_factor=7, bandwidth_hz=125000,
                   coding_rate_denominator=5, preamble_symbols=8):
    """Return LoRa packet time-on-air in seconds (Semtech explicit-header).

    payload_bytes can be an int (number of bytes) or a bytes object (in
    which case len() is used).
    """
    from math import ceil
    if isinstance(payload_bytes, (bytes, bytearray)):
        n_bytes = len(payload_bytes)
    else:
        n_bytes = int(payload_bytes)
    cr = coding_rate_denominator - 4
    ts = (2 ** spreading_factor) / bandwidth_hz
    low_data_rate = int(spreading_factor >= 11 and bandwidth_hz <= 125000)
    numerator = 8 * n_bytes - 4 * spreading_factor + 28 + 16
    denominator = 4 * (spreading_factor - 2 * low_data_rate)
    payload_symbols = 8 + max(ceil(numerator / denominator) * (cr + 4), 0)
    return (preamble_symbols + 4.25 + payload_symbols) * ts


# ---------------------------------------------------------------------------
# Constraint dataclass (matches scheduler.Constraint)
# ---------------------------------------------------------------------------
class Constraint(object):
    """A candidate inter-robot constraint available for transmission."""
    __slots__ = ('sequence', 'generated_ns', 'payload_bytes', 'predicted_trust',
                 'information_score', 'robot_i', 'robot_j', 'pair_constraints')

    def __init__(self, sequence, generated_ns, payload_bytes, predicted_trust,
                 information_score, robot_i, robot_j, pair_constraints=0):
        self.sequence = sequence
        self.generated_ns = generated_ns
        self.payload_bytes = payload_bytes
        self.predicted_trust = predicted_trust
        self.information_score = information_score
        self.robot_i = robot_i
        self.robot_j = robot_j
        self.pair_constraints = pair_constraints

    def observability(self, reference=6):
        from math import exp
        return exp(-self.pair_constraints / reference)


# ---------------------------------------------------------------------------
# Scheduler (matches scheduler.Scheduler, Python 3.5 compatible)
# ---------------------------------------------------------------------------
class Scheduler(object):
    """Select constraints using FIFO / BACS / BACS+ / RANDOM policy."""

    SUPPORTED_POLICIES = ("FIFO", "BACS", "BACS+", "RANDOM")

    def __init__(self, policy, trust_threshold=0.5, observability_weight=0.30,
                 observability_reference=6, defer_half_life_s=60.0, random_seed=0):
        if policy not in self.SUPPORTED_POLICIES:
            raise ValueError("unsupported policy: %s" % policy)
        self.policy = policy
        self.trust_threshold = trust_threshold
        self.observability_weight = observability_weight
        self.observability_reference = observability_reference
        from math import log
        self.gamma_defer = log(2) / defer_half_life_s
        self._rng = random.Random(random_seed)
        self._random_seed = random_seed

    def score(self, item, now_ns):
        from math import exp
        age_s = max(0, now_ns - item.generated_ns) / 1e9
        freshness = exp(-self.gamma_defer * age_s)
        base = item.information_score * item.predicted_trust * freshness
        if self.policy == "BACS+":
            base *= 1 + self.observability_weight * item.observability(self.observability_reference)
        return base / lora_airtime_s(item.payload_bytes)

    def rank(self, candidates, now_ns):
        """Return candidates ordered by policy (best first)."""
        eligible = [c for c in candidates if c.predicted_trust >= self.trust_threshold]
        if self.policy == "FIFO":
            return sorted(eligible, key=lambda c: c.sequence)
        if self.policy == "RANDOM":
            shuffled = list(eligible)
            self._rng.shuffle(shuffled)
            return shuffled
        return sorted(eligible, key=lambda c: (-self.score(c, now_ns), c.sequence))


# ---------------------------------------------------------------------------
# Constraint generation
# ---------------------------------------------------------------------------
def generate_constraint(seq, rng):
    """Generate one inter-robot constraint candidate."""
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
    return Constraint(
        sequence=seq, generated_ns=monotonic_ns(),
        payload_bytes=payload, predicted_trust=trust,
        information_score=info, robot_i=ROBOT_NAMES[ri],
        robot_j=ROBOT_NAMES[rj], pair_constraints=pair_count,
    )


# ---------------------------------------------------------------------------
# Provenance helpers
# ---------------------------------------------------------------------------
def write_session_info(out_dir, session_id, real_device, args):
    info_path = out_dir / "session_info.txt"
    lines = []
    lines.append("=== BACS-ROS2-SLAM Session Info ===")
    lines.append("session_id: %s" % session_id)
    lines.append("role: transmitter")
    lines.append("policy: %s" % args.policy)
    lines.append("started_at: %s" % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    lines.append("duration_s: %s" % args.duration)
    lines.append("tx_period_s: %s" % args.tx_period)
    lines.append("constraint_seed: %s" % args.seed)
    lines.append("constraint_period_s: 2.0")
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
        lines.append("git_commit: %s" % commit)
    except Exception:
        lines.append("git: not available")
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
    info_path.write_text("\n".join(lines) + "\n")
    print("    session_info.txt written", flush=True)


def write_chronyc_tracking(out_file, label):
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
        try:
            result = subprocess.run(["ntpq", "-p"],
                                      capture_output=True, text=True, timeout=5)
            lines.append("(chronyc not available, using ntpq)")
            lines.append(result.stdout)
        except FileNotFoundError:
            lines.append("WARNING: neither chronyc nor ntpq is available.")
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
    parser = argparse.ArgumentParser(description="LoRa transmitter with BACS scheduler.")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--policy", default="FIFO",
                        choices=["FIFO", "BACS", "BACS+", "RANDOM"],
                        help="Scheduler policy (default: FIFO)")
    parser.add_argument("--tx-period", type=float, default=11.0,
                        help="TX period in seconds (default: 11.0)")
    parser.add_argument("--duration", type=float, default=600.0,
                        help="Session duration in seconds (default: 600)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed (default: 42)")
    parser.add_argument("--trust-threshold", type=float, default=0.0,
                        help="Trust threshold (default: 0.0 = transmit all)")
    parser.add_argument("--output-dir", default=".",
                        help="Output directory (default: current dir)")
    parser.add_argument("--session-id", default=None,
                        help="Session ID (default: auto from policy)")
    args = parser.parse_args()

    if args.session_id is None:
        safe_policy = args.policy.replace("+", "plus")
        args.session_id = "HWS-BACS-%s-%s" % (safe_policy, time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))

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
    decisions_path = output_dir / "scheduler_decisions.csv"
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

    decisions_fh = decisions_path.open("w", newline="", encoding="utf-8")
    decisions_writer = csv.DictWriter(decisions_fh,
                                      fieldnames=SCHEDULER_DECISION_FIELDS,
                                      lineterminator="\n")
    decisions_writer.writeheader()

    # ---- Initialize scheduler ----
    scheduler = Scheduler(
        policy=args.policy,
        trust_threshold=args.trust_threshold,
        random_seed=args.seed,
    )
    rng = random.Random(args.seed)

    # ---- Constraint pool and generation ----
    pool = []
    gen_seq = 0
    tx_seq = 0
    gen_period_s = 2.0
    next_gen_ns = monotonic_ns()

    t_start = time.monotonic()
    t_end = t_start + args.duration

    print("", flush=True)
    print("Policy: %s" % args.policy, flush=True)
    print("Trust threshold: %s" % args.trust_threshold, flush=True)
    print("Constraint generation: every %ss" % gen_period_s, flush=True)
    print("TX period: every %ss" % args.tx_period, flush=True)
    print("Duration: %.0fs" % args.duration, flush=True)
    print("  Serial port: %s" % real_device, flush=True)
    print("  Output: %s" % serial_log_path, flush=True)
    print("  Output: %s" % decisions_path, flush=True)
    print("  Press Ctrl+C to stop early.", flush=True)
    print("", flush=True)

    try:
        while time.monotonic() < t_end:
            now_ns = monotonic_ns()

            # ---- Generate new constraints (every 2s) ----
            while now_ns >= next_gen_ns:
                gen_seq += 1
                c = generate_constraint(gen_seq, rng)
                pool.append(c)
                next_gen_ns += int(gen_period_s * 1e9)

            # ---- Check if it's time to transmit ----
            elapsed = time.monotonic() - t_start
            if elapsed >= (tx_seq + 1) * args.tx_period:
                # Time for the next TX tick.
                # Rank the pool by the active policy and pick the best.
                ranked = scheduler.rank(pool, now_ns)

                if not ranked:
                    # No eligible constraints (all below trust threshold).
                    tx_seq += 1
                    print("[%7.2fs] seq=%d: NO eligible constraint (pool=%d, all below threshold)" % (
                        elapsed, tx_seq, len(pool)), flush=True)
                    decisions_writer.writerow({
                        "tx_seq": tx_seq,
                        "policy": args.policy,
                        "constraint_seq": "",
                        "robot_i": "", "robot_j": "",
                        "trust": "", "info": "", "score": "",
                        "pool_size_before": len(pool),
                        "pool_size_after": len(pool),
                        "selected_at_ns": str(now_ns),
                        "reason": "no eligible constraint (all below trust threshold)",
                    })
                    decisions_fh.flush()
                    continue

                # Select the best constraint.
                chosen = ranked[0]
                tx_seq += 1

                # Build the TX line: T,<tx_seq>,<hex_payload>
                # Note: we use tx_seq as the packet sequence number (not
                # constraint.sequence) so the receiver sees 1,2,3,...
                # But the constraint data inside the payload retains the
                # original constraint.sequence for analysis.
                hex_payload = chosen.payload_bytes.hex()
                # Overwrite the first 4 bytes with tx_seq so the receiver
                # sees the TX sequence, not the constraint generation seq.
                payload_bytes = bytearray(chosen.payload_bytes)
                struct.pack_into(">I", payload_bytes, 0, tx_seq)
                hex_payload = bytes(payload_bytes).hex()

                line = ("T,%d,%s\n" % (tx_seq, hex_payload)).encode("ascii")
                host_handoff_ns = time_ns()

                # Record the scheduler decision.
                score = scheduler.score(chosen, now_ns)
                pool_size_before = len(pool)

                # Write HOST_TX row.
                serial_log_writer.writerow({
                    "host_wall_ns": str(host_handoff_ns),
                    "host_monotonic_ns": str(monotonic_ns()),
                    "direction": "HOST_TX",
                    "serial_port": real_device,
                    "device_uptime_ms": "",
                    "seq": str(tx_seq),
                    "rssi_dbm": "",
                    "snr_db": "",
                    "tx_duration_us": "",
                    "payload_hex": hex_payload,
                    "raw_line": "T,%d,%s" % (tx_seq, hex_payload[:32]) + "...",
                })
                serial_log_fh.flush()

                # Send the packet.
                port.write(line)

                # Wait for TX_DONE (or ACK_RX).
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

                    if tag in ("TX_DONE", "TX") and len(parts) >= 4:
                        try:
                            mr_done_ms = int(parts[1])
                            ack_seq = int(parts[2])
                            if tag == "TX_DONE":
                                tx_duration_us = int(parts[3])
                            else:
                                tx_duration_us = 0
                        except ValueError:
                            continue
                        if ack_seq == tx_seq:
                            tx_done = True
                            serial_log_writer.writerow({
                                "host_wall_ns": str(time_ns()),
                                "host_monotonic_ns": str(monotonic_ns()),
                                "direction": "MR_TX_DONE",
                                "serial_port": real_device,
                                "device_uptime_ms": str(mr_done_ms),
                                "seq": str(tx_seq),
                                "rssi_dbm": "",
                                "snr_db": "",
                                "tx_duration_us": str(tx_duration_us),
                                "payload_hex": "",
                                "raw_line": raw,
                            })
                            serial_log_fh.flush()

                            # Record clock offset.
                            host_ack_ns = time_ns()
                            clock_offsets_writer.writerow({
                                "seq": tx_seq,
                                "host_handoff_ns": host_handoff_ns,
                                "mr_timestamp_ms": mr_done_ms,
                                "host_ack_ns": host_ack_ns,
                                "offset_ns": mr_done_ms * 1000000 - host_handoff_ns,
                                "source": tag,
                            })
                            clock_offsets_fh.flush()
                            break
                    elif tag in ("ACK_RX", "RX") and len(parts) >= 5:
                        try:
                            ack_seq = int(parts[2])
                            if ack_seq == tx_seq:
                                mr_ack_ms = int(parts[1])
                                rssi = int(parts[3])
                                snr = float(parts[4])
                                serial_log_writer.writerow({
                                    "host_wall_ns": str(time_ns()),
                                    "host_monotonic_ns": str(monotonic_ns()),
                                    "direction": "MR_DELIVERY_ACK" if tag == "ACK_RX" else "MR_RX",
                                    "serial_port": real_device,
                                    "device_uptime_ms": str(mr_ack_ms),
                                    "seq": str(tx_seq),
                                    "rssi_dbm": str(rssi),
                                    "snr_db": "%.2f" % snr,
                                    "tx_duration_us": "",
                                    "payload_hex": "",
                                    "raw_line": raw,
                                })
                                serial_log_fh.flush()
                        except ValueError:
                            pass

                # Remove the transmitted constraint from the pool.
                pool = [c for c in pool if c.sequence != chosen.sequence]
                pool_size_after = len(pool)

                # Record the scheduler decision.
                decisions_writer.writerow({
                    "tx_seq": tx_seq,
                    "policy": args.policy,
                    "constraint_seq": chosen.sequence,
                    "robot_i": chosen.robot_i,
                    "robot_j": chosen.robot_j,
                    "trust": "%.4f" % chosen.predicted_trust,
                    "info": "%.4f" % chosen.information_score,
                    "score": "%.6f" % score,
                    "pool_size_before": pool_size_before,
                    "pool_size_after": pool_size_after,
                    "selected_at_ns": str(now_ns),
                    "reason": "selected by %s ranking" % args.policy,
                })
                decisions_fh.flush()

                if tx_done:
                    print("[%7.2fs] %s seq=%d constraint#%d trust=%.3f info=%.3f "
                          "score=%.6f pool=%d->%d -> TX_DONE mr_ms=%d airtime=%dus" % (
                              elapsed, args.policy, tx_seq, chosen.sequence,
                              chosen.predicted_trust, chosen.information_score,
                              score, pool_size_before, pool_size_after,
                              mr_done_ms, tx_duration_us),
                          flush=True)
                else:
                    print("[%7.2fs] %s seq=%d: NO TX_DONE (timeout)" % (
                        elapsed, args.policy, tx_seq), flush=True)

            # Sleep briefly to avoid busy-waiting.
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
    finally:
        # ---- Write chronyc tracking AFTER the run ----
        write_chronyc_tracking(output_dir / "chronyc_tracking_end.txt", "after run")

        # ---- Write manifest ----
        manifest = {
            "session_id": args.session_id,
            "role": "transmitter",
            "policy": args.policy,
            "duration_s": args.duration,
            "tx_period_s": args.tx_period,
            "seed": args.seed,
            "trust_threshold": args.trust_threshold,
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
            "packets_transmitted": tx_seq,
            "constraints_generated": gen_seq,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

        serial_log_fh.close()
        clock_offsets_fh.close()
        decisions_fh.close()
        port.close()

        # ---- Write SHA256SUMS.txt ----
        write_sha256sums(output_dir)

        print("", flush=True)
        print("Done. Policy=%s, transmitted %d packets, generated %d constraints." % (
            args.policy, tx_seq, gen_seq), flush=True)
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
