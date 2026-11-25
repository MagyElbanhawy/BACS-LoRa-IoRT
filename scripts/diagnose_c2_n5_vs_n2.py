#!/usr/bin/env python3
"""Investigate why BACS+ outperforms Random under C2 at N=5 but not N=2."""

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import pandas as pd
from bacs_sim.simulator import precompute, run
from scripts.revision.run_emrmf_bridge_v2 import CONDITIONS, configure_arm

CONFIG = ROOT / "config/sim/v15.json"
PLAN = json.loads(CONFIG.read_bytes())
PILOT = ROOT / "paper_results/revision/ranking_v2_c0c3/pilot_tdefer.csv"

def inspect_pair_coverage(condition="C2", seeds=(70, 71, 72, 73, 74)):
    import csv
    with PILOT.open() as f:
        pilot = {r["condition"]: r for r in csv.DictReader(f)}

    p = pilot.get(condition, pilot["C0"])
    half = float(p["deferral_median_s"]) or float(p["deferral_mean_s"])
    link = next((c for c in CONDITIONS if c["name"] == condition), CONDITIONS[0])

    records = []
    for n in (2, 5):
        for s in seeds:
            pre = None
            for arm in ("random", "plus_0.30_6"):
                cfg = configure_arm("fifo_defer" if arm == "fifo" else arm, link, s, n, PLAN["session_s"])
                for field, value in json.loads(p["overrides"]).items():
                    obj, attr = field.split(".")
                    setattr(getattr(cfg, obj), attr, value)
                cfg.world.payload_jitter_bytes = 0
                cfg.trust.gamma = math.log(2) / half
                if pre is None:
                    pre = precompute(cfg)

                res = run(cfg, precomputed=pre, collect_graph=True, execution="v15")
                deliv = res.extras["delivered"]

                # Count delivered per robot pair
                pairs = {}
                for c in deliv:
                    pair_key = tuple(sorted((c.rid_from, c.rid_to)))
                    pairs[pair_key] = pairs.get(pair_key, 0) + 1

                total_possible_pairs = n * (n - 1) // 2
                connected_pairs = len(pairs)
                min_pair_count = min(pairs.values()) if connected_pairs == total_possible_pairs else 0

                # Mean age and loop span
                ages = [c.t_recv - c.t_created for c in deliv]
                spans = [abs(res.extras["truths"][c.rid_from].t[c.idx_from] - res.extras["truths"][c.rid_to].t[c.idx_to]) for c in deliv]

                records.append({
                    "n_robots": n,
                    "seed": s,
                    "arm": arm,
                    "align_rmse": res.align_rmse,
                    "n_delivered": len(deliv),
                    "connected_pairs": connected_pairs,
                    "total_possible_pairs": total_possible_pairs,
                    "pair_coverage_pct": 100.0 * connected_pairs / total_possible_pairs,
                    "min_pair_constraints": min_pair_count,
                    "mean_age_s": float(np.mean(ages)) if ages else 0.0,
                    "mean_loop_span_s": float(np.mean(spans)) if spans else 0.0,
                })

    df = pd.DataFrame(records)
    summary = df.groupby(["n_robots", "arm"]).agg(
        align_rmse=("align_rmse", "mean"),
        n_delivered=("n_delivered", "mean"),
        connected_pairs=("connected_pairs", "mean"),
        pair_coverage_pct=("pair_coverage_pct", "mean"),
        min_pair_constraints=("min_pair_constraints", "mean"),
        mean_age_s=("mean_age_s", "mean"),
        mean_loop_span_s=("mean_loop_span_s", "mean"),
    ).reset_index()

    print(f"\n--- PAIR COVERAGE & CONSTRAINT METRICS UNDER {condition} ---")
    print(summary.to_string(index=False, float_format=lambda x: f"{x:.4f}"))
    return df

if __name__ == "__main__":
    inspect_pair_coverage()
