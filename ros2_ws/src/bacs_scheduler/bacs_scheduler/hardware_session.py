"""Run one measured RYLR998 BACS policy session on one experiment computer.

Run this command at the same scheduled UTC time on the server and both robots.
The server records the bag and runs reception/fusion; each robot runs candidate
generation and its policy sender.  Every process has a separate text log and
the run manifest is finalized as COMPLETE, FAILED or INTERRUPTED.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import IO

from .experiment_plan import POLICIES, expected_policy


DEFAULT_BAG_TOPICS = (
    "/tf", "/tf_static", "/scan/limo01", "/scan/limo02", "/odom/limo01", "/odom/limo02",
    "/vicon/limo01/pose", "/vicon/limo02/pose", "/map",
    "/bacs/candidates", "/bacs/scheduler", "/bacs/received", "/bacs/keyframes",
    "/bacs/kf_desc", "/bacs/kf_request", "/bacs/kf_points",
)
SESSION_RE = re.compile(r"^(?:HWS|PILOT)-[A-Za-z0-9+_.-]+$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def parse_start_at(text: str | None) -> float | None:
    if not text:
        return None
    value = text.strip()
    if value.isdigit():
        return float(value)
    if value.endswith("Z"):
        value = value[:-1] + "+00:00"
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        raise ValueError("--start-at must contain a timezone (normally Z)")
    return parsed.timestamp()


def chrony_offset_ms() -> tuple[float | None, str]:
    """Return absolute chrony last-offset in milliseconds and the full evidence."""
    try:
        result = subprocess.run(["chronyc", "tracking"], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError) as error:
        return None, f"UNAVAILABLE: {error}\n"
    evidence = (result.stdout or "") + (result.stderr or "")
    if result.returncode != 0:
        return None, evidence
    match = re.search(r"^Last offset\s*:\s*([+-]?[0-9.eE+-]+)\s+seconds", evidence, re.MULTILINE)
    return (abs(float(match.group(1))) * 1000 if match else None), evidence


def base_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--role", choices=("server", "robot"), required=True)
    parser.add_argument("--session", required=True, help="for example HWS-BACS-001")
    parser.add_argument("--run", type=int, required=True, help="paired run/block number")
    parser.add_argument("--policy", choices=POLICIES, required=True)
    parser.add_argument("--duration", type=float, default=600.0, help="recording seconds (default: 600)")
    parser.add_argument("--start-at", help="synchronized UTC start, e.g. 2026-11-19T09:00:00Z")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--log-dir", default="~/bacs_hw_logs")
    parser.add_argument("--ros-domain-id", type=int, default=42)
    parser.add_argument("--max-clock-offset-ms", type=float, default=1.0)
    parser.add_argument("--pilot", action="store_true", help="allow missing/out-of-limit chrony; session must start PILOT-")
    parser.add_argument("--network-id", type=int, default=18)
    parser.add_argument("--power-dbm", type=int, default=14)
    parser.add_argument("--robot", choices=("limo01", "limo02"), help="required for --role robot")
    parser.add_argument("--address", type=int, help="required for --role robot (normally 1 or 2)")
    parser.add_argument("--scan-topic", help="robot LaserScan topic; default /scan/<robot>")
    parser.add_argument("--local-frame", help="robot odometry frame; default <robot>/odom")
    parser.add_argument("--base-frame", help="robot base frame; default <robot>/base_link")
    parser.add_argument("--random-seed", type=int, default=0)
    parser.add_argument("--server-address", type=int, default=100)
    parser.add_argument("--duty-cycle", type=float, default=0.01)
    parser.add_argument("--duty-window-s", type=float, default=60.0)
    parser.add_argument("--trust-threshold", type=float, default=0.5)
    parser.add_argument("--observability-weight", type=float, default=0.30)
    parser.add_argument("--observability-reference", type=int, default=6)
    parser.add_argument("--defer-half-life-s", type=float, default=60.0)
    parser.add_argument("--max-queue-age-s", type=float, default=600.0)
    parser.add_argument("--keyframe-period-s", type=float, default=2.0)
    parser.add_argument("--min-motion-m", type=float, default=0.05)
    parser.add_argument("--min-rotation-deg", type=float, default=5.0)
    parser.add_argument("--sectors", type=int, default=60)
    parser.add_argument("--max-range", type=float, default=10.0)
    parser.add_argument("--shortlist", type=int, default=3)
    parser.add_argument("--place-threshold", type=float, default=0.12)
    parser.add_argument("--min-inlier-ratio", type=float, default=0.8)
    parser.add_argument("--max-rmse", type=float, default=0.05)
    parser.add_argument("--min-points", type=int, default=60)
    parser.add_argument("--trust-rmse-ref", type=float, default=0.05)
    parser.add_argument("--info-sigma-ref", type=float, default=0.05)
    parser.add_argument("--start-poses", nargs=6, type=float, metavar=("X1", "Y1", "YAW1", "X2", "Y2", "YAW2"),
                        help="server only: measured start poses; required for a real run")
    parser.add_argument("--extra-topic", action="append", default=[], help="server only: add a rosbag topic")
    parser.add_argument("--dry-run", action="store_true", help="validate and print commands without starting nodes")
    return parser


def validate(args: argparse.Namespace) -> None:
    if not SESSION_RE.match(args.session):
        raise ValueError("--session must start HWS- or PILOT- and contain only safe identifier characters")
    if args.run <= 0 or args.duration <= 0:
        raise ValueError("--run and --duration must be positive")
    if args.pilot and not args.session.startswith("PILOT-"):
        raise ValueError("--pilot sessions must start with PILOT-")
    if not args.pilot and not args.session.startswith("HWS-"):
        raise ValueError("measured sessions must start with HWS-")
    if args.role == "robot" and (not args.robot or args.address is None):
        raise ValueError("--role robot requires --robot and --address")
    if args.role == "server" and not args.start_poses:
        raise ValueError("--role server requires six measured --start-poses values")
    if args.session.startswith("HWS-"):
        expected_run, planned_policy = expected_policy(args.session)
        if args.policy != planned_policy or args.run != expected_run:
            raise ValueError(
                f"{args.session} is block/run {expected_run} policy {planned_policy}; "
                f"received run {args.run} policy {args.policy}"
            )
    if not 0 <= args.ros_domain_id <= 232:
        raise ValueError("--ros-domain-id must be between 0 and 232")
    if not 0 < args.power_dbm <= 22:
        raise ValueError("--power-dbm must be between 1 and 22")
    if not 0 < args.duty_cycle <= 0.01:
        raise ValueError("--duty-cycle must be greater than 0 and no more than 0.01")
    if args.duty_window_s <= 0 or args.defer_half_life_s <= 0 or args.max_queue_age_s <= 0:
        raise ValueError("duty window, defer half-life and maximum queue age must be positive")
    if not 0 <= args.trust_threshold <= 1 or args.observability_weight < 0 or args.observability_reference <= 0:
        raise ValueError("invalid scheduler scoring parameters")
    if (args.keyframe_period_s <= 0 or args.min_motion_m < 0 or args.min_rotation_deg < 0 or
            args.sectors <= 0 or args.max_range <= 0 or args.shortlist <= 0 or
            not 0 <= args.place_threshold or not 0 <= args.min_inlier_ratio <= 1 or
            args.max_rmse <= 0 or args.min_points <= 0 or args.trust_rmse_ref <= 0 or args.info_sigma_ref <= 0):
        raise ValueError("invalid candidate-generation parameters")


def ros_args(parameters: dict[str, object]) -> list[str]:
    result = ["--ros-args"]
    for name, value in parameters.items():
        if isinstance(value, list):
            rendered = "[" + ",".join(str(item).lower() if isinstance(item, bool) else str(item) for item in value) + "]"
        elif isinstance(value, bool):
            rendered = str(value).lower()
        else:
            rendered = str(value)
        result.extend(["-p", f"{name}:={rendered}"])
    return result


def commands(args: argparse.Namespace, run_dir: Path) -> list[tuple[str, list[str]]]:
    common = {"session": args.session, "run": args.run, "log_dir": str(Path(args.log_dir).expanduser())}
    radio = {"port": args.serial_port, "baud": args.baud, "network_id": args.network_id,
             "power_dbm": args.power_dbm}
    if args.role == "robot":
        robot = args.robot
        candidate = {**common, "robot": robot, "scan_topic": args.scan_topic or f"/scan/{robot}",
                     "local_frame": args.local_frame or f"{robot}/odom",
                     "base_frame": args.base_frame or f"{robot}/base_link",
                     "start_at_ns": int(getattr(args, "acquisition_start_ns", 0)),
                     "keyframe_period_s": args.keyframe_period_s, "min_motion_m": args.min_motion_m,
                     "min_rotation_deg": args.min_rotation_deg, "sectors": args.sectors,
                     "max_range": args.max_range, "shortlist": args.shortlist,
                     "place_threshold": args.place_threshold, "min_inlier_ratio": args.min_inlier_ratio,
                     "max_rmse": args.max_rmse, "min_points": args.min_points,
                     "trust_rmse_ref": args.trust_rmse_ref, "info_sigma_ref": args.info_sigma_ref}
        sender = {**common, **radio, "policy": args.policy, "robot": robot, "address": args.address,
                  "server_address": args.server_address, "random_seed": args.random_seed,
                  "window_s": args.duty_window_s, "duty_cycle": args.duty_cycle,
                  "trust_threshold": args.trust_threshold,
                  "observability_weight": args.observability_weight,
                  "observability_reference": args.observability_reference,
                  "defer_half_life_s": args.defer_half_life_s,
                  "max_queue_age_s": args.max_queue_age_s,
                  "robots": ["limo01", "limo02"]}
        return [
            ("candidate", ["ros2", "run", "bacs_scheduler", "bacs_candidates", *ros_args(candidate)]),
            ("sender", ["ros2", "run", "bacs_scheduler", "bacs_sender", *ros_args(sender)]),
        ]
    receiver = {**common, **radio, "address": args.server_address, "robots": ["limo01", "limo02"],
                "robot_addresses": [1, 2]}
    fusion = {**common, "robots": ["limo01", "limo02"], "start_poses": args.start_poses}
    bag_topics = list(dict.fromkeys((*DEFAULT_BAG_TOPICS, *args.extra_topic)))
    return [
        ("bag", ["ros2", "bag", "record", "-o", str(run_dir / "rosbag"), *bag_topics]),
        ("fusion", ["ros2", "run", "bacs_scheduler", "bacs_fusion", *ros_args(fusion)]),
        ("receiver", ["ros2", "run", "bacs_scheduler", "bacs_receiver", *ros_args(receiver)]),
    ]


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def stop_process(process: subprocess.Popen, timeout_s: float = 12.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def run(args: argparse.Namespace) -> int:
    validate(args)
    start_at = parse_start_at(args.start_at)
    if start_at is not None and not args.dry_run and start_at < time.time() + 10.0:
        raise RuntimeError("--start-at must be at least 10 seconds in the future so every node can initialize")
    acquisition_start = start_at if start_at is not None else time.time() + 10.0
    args.acquisition_start_ns = int(acquisition_start * 1e9)
    if shutil.which("ros2") is None:
        raise RuntimeError("ros2 is not on PATH; source /opt/ros/humble/setup.bash and the workspace install")
    if not Path(args.serial_port).exists():
        raise RuntimeError(f"serial device does not exist: {args.serial_port}")
    prefix = subprocess.run(["ros2", "pkg", "prefix", "bacs_scheduler"], capture_output=True, text=True)
    if prefix.returncode != 0:
        raise RuntimeError("bacs_scheduler is not installed in this shell; build and source ros2_ws/install/setup.bash")

    offset_ms, chrony = chrony_offset_ms()
    if not args.pilot and (offset_ms is None or offset_ms > args.max_clock_offset_ms):
        measured = "unavailable" if offset_ms is None else f"{offset_ms:.6f} ms"
        raise RuntimeError(f"chrony preflight failed: offset {measured}, limit {args.max_clock_offset_ms} ms")

    root = Path(args.log_dir).expanduser()
    run_dir = root / args.session / f"run_{args.run:02d}"
    manifest_stem = f"manifest_{args.role}{'_' + args.robot if args.robot else ''}"
    if args.dry_run:
        manifest_stem += "_dry_run"
    manifest_path = run_dir / f"{manifest_stem}.json"
    if manifest_path.exists():
        raise RuntimeError(f"run role already exists and will not be overwritten: {manifest_path}")
    run_dir.mkdir(parents=True, exist_ok=True)
    clock_name = f"clock_runner_{args.role}{'_' + args.robot if args.robot else ''}_start.txt"
    (run_dir / clock_name).write_text(f"time_ns={time.time_ns()}\n{chrony}", encoding="utf-8")
    planned = commands(args, run_dir)
    manifest = {
        "schema": 1, "status": "PREFLIGHT_OK", "session": args.session, "run": args.run,
        "policy": args.policy, "role": args.role, "robot": args.robot, "address": args.address,
        "server_address": args.server_address, "duration_s": args.duration, "start_at": args.start_at,
        "acquisition_start_ns": args.acquisition_start_ns,
        "created_utc": utc_now(), "ros_domain_id": args.ros_domain_id, "serial_port": args.serial_port,
        "radio": {"model": "RYLR998", "band_hz": 868000000, "sf": 7, "bandwidth_hz": 125000,
                  "coding_rate": "4/5", "payload_bytes": 52, "network_id": args.network_id,
                  "power_dbm": args.power_dbm},
        "scheduler": {"duty_cycle": args.duty_cycle, "window_s": args.duty_window_s,
                      "trust_threshold": args.trust_threshold,
                      "observability_weight": args.observability_weight,
                      "observability_reference": args.observability_reference,
                      "defer_half_life_s": args.defer_half_life_s,
                      "max_queue_age_s": args.max_queue_age_s, "random_seed": args.random_seed},
        "candidate_generator": {"keyframe_period_s": args.keyframe_period_s,
                                "min_motion_m": args.min_motion_m,
                                "min_rotation_deg": args.min_rotation_deg, "sectors": args.sectors,
                                "max_range": args.max_range, "shortlist": args.shortlist,
                                "place_threshold": args.place_threshold,
                                "min_inlier_ratio": args.min_inlier_ratio, "max_rmse": args.max_rmse,
                                "min_points": args.min_points, "trust_rmse_ref": args.trust_rmse_ref,
                                "info_sigma_ref": args.info_sigma_ref},
        "clock_offset_ms_start": offset_ms, "pilot": args.pilot,
        "commands": [{"name": name, "argv": command} for name, command in planned],
    }
    write_json(manifest_path, manifest)
    if args.dry_run:
        for name, command in planned:
            print(f"[{name}] " + " ".join(command))
        manifest["status"] = "DRY_RUN"
        write_json(manifest_path, manifest)
        return 0

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    children: list[tuple[str, subprocess.Popen, IO[str]]] = []
    interrupted = False
    try:
        manifest["status"] = "RUNNING"
        manifest["processes_started_utc"] = utc_now()
        manifest["processes_started_ns"] = time.time_ns()
        write_json(manifest_path, manifest)
        for name, command in planned:
            handle = (run_dir / f"process_{args.role}_{name}.log").open("w", encoding="utf-8")
            process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, env=env,
                                       start_new_session=True, text=True)
            children.append((name, process, handle))
            time.sleep(0.75)
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited during startup with code {process.returncode}")
        while time.time() < acquisition_start:
            failed = [(name, process.returncode) for name, process, _ in children
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"child process exited before synchronized start: {failed}")
            time.sleep(min(0.25, acquisition_start - time.time()))
        manifest["acquisition_started_utc"] = utc_now()
        manifest["acquisition_started_ns"] = time.time_ns()
        write_json(manifest_path, manifest)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            failed = [(name, process.returncode) for name, process, _ in children
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"child process exited before the session ended: {failed}")
            time.sleep(min(0.5, deadline - time.monotonic()))
    except KeyboardInterrupt:
        interrupted = True
    except Exception as error:
        manifest["error"] = str(error)
    finally:
        # Robot order is sender then candidate. Server order is receiver, fusion, then bag.
        for _, process, _ in reversed(children):
            stop_process(process)
        for _, _, handle in children:
            handle.close()
        end_offset_ms, end_chrony = chrony_offset_ms()
        suffix = f"{args.role}{'_' + args.robot if args.robot else ''}"
        (run_dir / f"clock_runner_{suffix}_end.txt").write_text(
            f"time_ns={time.time_ns()}\n{end_chrony}", encoding="utf-8")
        manifest["ended_utc"] = utc_now()
        manifest["ended_ns"] = time.time_ns()
        manifest["clock_offset_ms_end"] = end_offset_ms
        manifest["process_returncodes"] = {name: process.returncode for name, process, _ in children}
        if interrupted:
            manifest["status"] = "INTERRUPTED"
        elif "error" in manifest:
            manifest["status"] = "FAILED"
        elif not args.pilot and (end_offset_ms is None or end_offset_ms > args.max_clock_offset_ms):
            manifest["status"] = "FAILED"
            manifest["error"] = "chrony end offset missing or outside limit"
        else:
            manifest["status"] = "COMPLETE"
        write_json(manifest_path, manifest)
    return 0 if manifest["status"] == "COMPLETE" else 1


def main(argv: list[str] | None = None) -> None:
    args = base_parser().parse_args(argv)
    try:
        raise SystemExit(run(args))
    except (ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error


if __name__ == "__main__":
    main()
