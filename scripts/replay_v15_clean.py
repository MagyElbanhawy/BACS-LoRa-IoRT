"""Replay v15 in a new directory and compare all deterministic observations against the accepted run."""
import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
METRICS = ("align_rmse", "pose_rmse", "gamma")

def check_cell(reference, regenerated):
    def rows(path):
        with path.open(newline="") as stream:
            return sorted(csv.DictReader(stream), key=lambda row: row["arm"])
    previous, current = rows(reference), rows(regenerated)
    for a, b in zip(previous, current, strict=True):
        for key in a:
            if key in METRICS:
                if math.isclose(float(a[key]), float(b[key]), rel_tol=1e-9, abs_tol=1e-12):
                    continue
            if a[key] != b[key]:
                raise ValueError(f"{reference.name}, {a['arm']}, {key}: {a[key]} versus {b[key]}")

def replay(run, destination, workers):
    sources = json.loads((run/"sources.json").read_text())
    destination.mkdir(parents=True, exist_ok=False)
    for name, digest in sources.items():
        path = ROOT/name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError(f"Source mismatch: {name}")
        target = destination/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path,target)
    subprocess.run([sys.executable, "scripts/run_v15.py", "--workers", str(workers)],
                   cwd=destination, check=True)
    fresh = destination/"runs/v15"/run.name
    for experiment in ("confirmation","bridge"):
        expected = {p.name for p in (run/experiment).glob("*.csv")}
        actual = {p.name for p in (fresh/experiment).glob("*.csv")}
        if len(expected)!=480 or expected!=actual:
            raise ValueError(f"{experiment} grid mismatch")
        for name in sorted(expected):
            check_cell(run/experiment/name, fresh/experiment/name)
    print("960 regenerated cells match the archived v15 reference within the documented tolerances.")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("run",type=Path)
    p.add_argument("new_directory",type=Path)
    p.add_argument("--workers",type=int,default=12)
    args=p.parse_args()
    replay(args.run.resolve(),args.new_directory.resolve(),args.workers)
