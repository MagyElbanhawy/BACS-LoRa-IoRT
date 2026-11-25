"""Run fresh paired cells under an immutable source/config fingerprint. Resume by repeating the command."""
import argparse
import csv
import hashlib
import io
import itertools
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[key] = "1"
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bacs_sim.simulator import precompute, run
from scripts.revision.run_emrmf_bridge_v2 import CONDITIONS, configure_arm

CONFIG = ROOT / "config/sim/v15.json"
PLAN = json.loads(CONFIG.read_bytes())
PILOT = ROOT / "paper_results/revision/ranking_v2_c0c3/pilot_tdefer.csv"
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

if digest(PILOT) != PLAN["pilot_sha256"]:
    raise ValueError("pilot checksum mismatch")
with PILOT.open() as f:
    pilot = {r["condition"]: r for r in csv.DictReader(f)}
sources = [CONFIG, PILOT, Path(__file__).resolve(), ROOT / "scripts/revision/run_emrmf_bridge_v2.py",
           *sorted((ROOT / "bacs_sim").glob("*.py"))]
snapshot = {p.relative_to(ROOT).as_posix():digest(p) for p in sources}
revision = hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()
OUT = ROOT / "runs/v15" / revision

def cell(job):
    experiment, condition, seed, n = job
    target = OUT / experiment / f"{condition}_{seed}_{n}.csv"
    if target.exists():
        return
    p = pilot.get(condition, pilot["C0"])
    half = float(p["deferral_median_s"]) or float(p["deferral_mean_s"])
    link = next((c for c in CONDITIONS if c["name"] == condition), CONDITIONS[0])
    rows = []
    pre = None
    for arm in PLAN[experiment]["arms"]:
        cfg = configure_arm("fifo_defer" if arm=="fifo" else arm, link, seed, n, PLAN["session_s"])
        for field, value in json.loads(p["overrides"]).items():
            obj, attr = field.split(".")
            setattr(getattr(cfg,obj),attr,value)
        cfg.world.payload_jitter_bytes = 0
        assert cfg.world.payload_base_bytes + cfg.world.payload_descriptor_bytes == PLAN["payload_bytes"]
        if arm not in ("emrmf_published","emrmf_drift"):
            cfg.trust.gamma = math.log(2)/half
        if pre is None:
            pre = precompute(cfg)
        result = run(cfg, precomputed=pre, execution="v15")
        rows.append(dict(seed=seed,n_robots=n,condition=condition,arm=arm,
                         align_rmse=result.align_rmse,pose_rmse=result.pose_rmse,
                         n_sent=result.n_sent,n_delivered=result.n_delivered,gamma=result.gamma_final))
    buf = io.StringIO()
    writer = csv.DictWriter(buf,fieldnames=rows[0])
    writer.writeheader()
    writer.writerows(rows)
    tmp = target.with_suffix(f".{os.getpid()}.tmp")
    tmp.write_text(buf.getvalue())
    os.replace(tmp,target)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers",type=int,default=10)
    args = parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"sources.json").write_text(json.dumps(snapshot,indent=2))
    jobs = []
    for experiment in ("confirmation","bridge"):
        (OUT/experiment).mkdir(exist_ok=True)
        block = PLAN[experiment]
        start,end = block["seeds"]
        jobs.extend(itertools.product([experiment],block["conditions"],range(start,end+1),PLAN["team_sizes"]))
    print(f"Target directory: {OUT}", flush=True)
    print(f"Total cells to compute: {len(jobs)} with {args.workers} workers", flush=True)
    with ProcessPoolExecutor(args.workers) as pool:
        for idx, _ in enumerate(pool.map(cell, jobs), 1):
            if idx % 10 == 0 or idx == len(jobs):
                print(f"[{idx}/{len(jobs)}] cells completed", flush=True)
