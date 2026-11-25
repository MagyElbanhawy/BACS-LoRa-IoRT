"""
Experiment Execution Engine for Reviewer 1.3.

Runs full factorial evaluation across:
- Conditions: C4 (Geometry-dependent), C4-IID (Matched independent), C0 (Unobstructed baseline)
- Seeds: 130 to 159 (30 pristine seeds)
- Team sizes: 2, 3, 4, 5 robots
- Policies: FIFO, Random, LIFO, BACS (bacs_gated), BACS+ (bacs_plus)

Uses multiprocessing.Pool across available CPU cores for efficient parallel execution.
Saves raw per-run CSVs in reviewer_R1_3_geometry_outages/raw_results/.
"""
import os
import sys
import time
import argparse
import csv
from multiprocessing import Pool, cpu_count
from typing import List, Dict, Any, Tuple
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from bacs_sim.config import SimConfig
from bacs_sim.simulator import precompute, _clone

from reviewer_R1_3_geometry_outages.runner import (
    run_geometry_simulation,
    GeometryRunResult,
)
from reviewer_R1_3_geometry_outages.channel import MATCHED_C4_IID_LOSS_RATE

POLICIES = ["fifo", "random", "lifo", "bacs_gated", "bacs_plus"]
TEAM_SIZES = [2, 3, 4, 5]
SEEDS = list(range(130, 160))  # 30 seeds: 130 to 159 inclusive

RAW_RESULTS_DIR = os.path.join(os.path.dirname(__file__), "raw_results")


def run_seed_slice(args: Tuple[str, int, int]) -> List[Dict[str, Any]]:
    """
    Worker function: runs all 5 policies for a given (condition, seed, n_robots)
    on an identical precomputed candidate pool.
    """
    condition, seed, n_robots = args

    # Configure simulation
    cfg = SimConfig(seed=seed)
    cfg.world.n_robots = n_robots
    cfg.world.session_s = 600.0
    cfg.scheduler.window_s = 60.0
    cfg.lora.duty_cycle = 0.01

    # Precompute world trajectories and candidate pool ONCE for this seed and team size
    truths, base_cands = precompute(cfg)

    results = []
    for pol in POLICIES:
        pol_cfg = SimConfig(seed=seed)
        pol_cfg.world.n_robots = n_robots
        pol_cfg.world.session_s = 600.0
        pol_cfg.scheduler.window_s = 60.0
        pol_cfg.lora.duty_cycle = 0.01
        pol_cfg.scheduler.policy = pol

        # Deep clone candidates so each policy starts with identical candidate pool
        cloned_cands = {k: [_clone(c) for c in v] for k, v in base_cands.items()}

        res: GeometryRunResult = run_geometry_simulation(
            pol_cfg,
            condition=condition,
            precomputed=(truths, cloned_cands),
            matched_loss_rate=MATCHED_C4_IID_LOSS_RATE,
        )

        row = {
            "condition": res.condition,
            "seed": res.seed,
            "n_robots": res.n_robots,
            "policy": res.policy,
            "pose_rmse": res.pose_rmse,
            "align_rmse": res.align_rmse,
            "trust_yield": res.trust_yield,
            "accept_rate": res.accept_rate,
            "airtime_util": res.airtime_util,
            "n_candidates": res.n_candidates,
            "n_sent": res.n_sent,
            "n_delivered": res.n_delivered,
            "bytes_sent": res.bytes_sent,
            "sched_overhead_ms": res.sched_overhead_ms,
            "starvation": res.starvation,
            "outlier_share": res.outlier_share,
            "gamma_final": res.gamma_final,
            "dt_pred_bias": res.dt_pred_bias,
            "obstruction_fraction": res.obstruction_fraction,
            "n_outages": res.n_outages,
            "mean_outage_s": res.mean_outage_s,
            "max_outage_s": res.max_outage_s,
            "n_los_attempts": res.n_los_attempts,
            "n_los_delivered": res.n_los_delivered,
            "n_nlos_attempts": res.n_nlos_attempts,
            "n_nlos_delivered": res.n_nlos_delivered,
            "los_delivery_rate": res.los_delivery_rate,
            "nlos_delivery_rate": res.nlos_delivery_rate,
            "n_accepted_inliers": res.n_accepted_inliers,
            "n_rejected_inliers": res.n_rejected_inliers,
            "n_accepted_outliers": res.n_accepted_outliers,
            "n_rejected_outliers": res.n_rejected_outliers,
        }
        results.append(row)

    return results


def save_csv(rows: List[Dict[str, Any]], filename: str) -> None:
    if not rows:
        return
    filepath = os.path.join(RAW_RESULTS_DIR, filename)
    fieldnames = list(rows[0].keys())
    with open(filepath, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {len(rows)} records to {filepath}")


def main():
    parser = argparse.ArgumentParser(description="Run Reviewer 1.3 experiments.")
    parser.add_argument(
        "--workers", type=int, default=max(1, min(14, cpu_count() - 2)),
        help="Number of parallel workers"
    )
    parser.add_argument(
        "--conditions", nargs="+", default=["C4", "C4-IID", "C0"],
        help="Conditions to run"
    )
    args = parser.parse_args()

    os.makedirs(RAW_RESULTS_DIR, exist_ok=True)

    print("==================================================================")
    print(" Reviewer 1.3: Geometry-Dependent Outage Experiments (C4 / C4-IID / C0)")
    print(f" Seeds: {SEEDS[0]}..{SEEDS[-1]} ({len(SEEDS)} seeds)")
    print(f" Team sizes: {TEAM_SIZES}")
    print(f" Policies: {POLICIES}")
    print(f" Conditions: {args.conditions}")
    print(f" Parallel workers: {args.workers}")
    print("==================================================================")

    all_rows = []
    t_start = time.time()

    for cond in args.conditions:
        print(f"\n--- Starting Condition: {cond} ---")
        tasks = [
            (cond, s, n)
            for s in SEEDS
            for n in TEAM_SIZES
        ]
        print(f"Total tasks for {cond}: {len(tasks)} (each runs 5 policies)")

        cond_rows = []
        with Pool(processes=args.workers) as pool:
            for i, result_batch in enumerate(pool.imap_unordered(run_seed_slice, tasks)):
                cond_rows.extend(result_batch)
                if (i + 1) % 20 == 0 or (i + 1) == len(tasks):
                    elapsed = time.time() - t_start
                    print(f"  [{cond}] Completed {i + 1}/{len(tasks)} tasks ({len(cond_rows)} runs) [Elapsed: {elapsed:.1f}s]")

        # Sort condition rows by seed, n_robots, policy
        cond_rows.sort(key=lambda r: (r["seed"], r["n_robots"], r["policy"]))
        save_csv(cond_rows, f"raw_results_{cond.replace('-', '_')}.csv")
        all_rows.extend(cond_rows)

    # Save merged dataset
    all_rows.sort(key=lambda r: (r["condition"], r["seed"], r["n_robots"], r["policy"]))
    save_csv(all_rows, "all_runs_raw.csv")

    total_time = time.time() - t_start
    print(f"\nAll experiments complete in {total_time:.1f}s ({total_time / 60:.2f} min).")
    print(f"Total runs executed: {len(all_rows)}")


if __name__ == "__main__":
    main()
