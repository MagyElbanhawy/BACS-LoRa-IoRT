#!/usr/bin/env python3
"""Seed-level statistical analysis for BACS_v14.

The independent unit is the seed. Paired values are first averaged within
seed across team sizes and communication conditions, then tested across the 30
seed-level differences. Equivalence uses seed-level mean log ratios.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import t, wilcoxon


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def paired_seed_means(
    df, arm_column, treatment, reference, cell_columns=None,
):
    """Aggregate paired cell differences and log ratios within each seed.

    ``cell_columns`` must identify a unique experimental cell and include
    ``seed``. For example, confirmation uses seed, condition, and team size.
    """
    cell_columns = list(cell_columns or ["seed"])
    if "seed" not in cell_columns:
        raise ValueError("cell_columns must include seed")
    wide = df.pivot_table(
        index=cell_columns, columns=arm_column, values="align_rmse", aggfunc="mean"
    )
    paired_cells = wide[[treatment, reference]].dropna().copy()
    paired_cells["difference"] = paired_cells[treatment] - paired_cells[reference]
    paired_cells["log_ratio"] = np.log(
        paired_cells[treatment] / paired_cells[reference]
    )
    return paired_cells.reset_index().groupby("seed", as_index=False).agg({
        treatment: "mean",
        reference: "mean",
        "difference": "mean",
        "log_ratio": "mean",
    })


def difference_test(paired, treatment, reference):
    diff = paired["difference"].to_numpy(float)
    result = wilcoxon(diff, alternative="two-sided")
    se = diff.std(ddof=1) / np.sqrt(len(diff))
    q = t.ppf(0.975, len(diff) - 1)
    return {
        "n_seeds": len(diff),
        "treatment": treatment,
        "reference": reference,
        "treatment_mean": float(paired[treatment].mean()),
        "reference_mean": float(paired[reference].mean()),
        "mean_difference": float(diff.mean()),
        "difference_ci95_lo": float(diff.mean() - q * se),
        "difference_ci95_hi": float(diff.mean() + q * se),
        "wilcoxon_W": float(result.statistic),
        "wilcoxon_p": float(result.pvalue),
        "treatment_lower_seeds": int((diff < 0).sum()),
        "ties": int((diff == 0).sum()),
        "treatment_higher_seeds": int((diff > 0).sum()),
    }


def tost_log_ratio(paired, lower=0.90, upper=1.10):
    values = paired["log_ratio"].to_numpy(float)
    mean = values.mean()
    se = values.std(ddof=1) / np.sqrt(len(values))
    df = len(values) - 1
    q = t.ppf(0.95, df)
    p_lower = t.sf((mean - np.log(lower)) / se, df)
    p_upper = t.cdf((mean - np.log(upper)) / se, df)
    p_tost = max(p_lower, p_upper)
    return {
        "n_seeds": len(values),
        "geometric_mean_ratio": float(np.exp(mean)),
        "ratio_ci90_lo": float(np.exp(mean - q * se)),
        "ratio_ci90_hi": float(np.exp(mean + q * se)),
        "equivalence_bound_low": lower,
        "equivalence_bound_high": upper,
        "p_lower": float(p_lower),
        "p_upper": float(p_upper),
        "p_tost": float(p_tost),
        "equivalent": bool(p_tost < 0.05),
    }


def git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return "unavailable"


def write_experiment(name, raw, normalized, summary, source_commit, config_name):
    out = ROOT / "results" / "simulation" / name
    out.mkdir(parents=True, exist_ok=True)
    normalized.to_csv(out / "raw_per_seed.csv", index=False)
    pd.DataFrame(summary).to_csv(out / "summary.csv", index=False)
    config = ROOT / "config" / "sim" / config_name
    manifest = {
        "experiment": name,
        "source_commit": source_commit,
        "analysis_commit": git_commit(),
        "source_path": str(raw.relative_to(ROOT)),
        "source_sha256": sha256(raw),
        "config_path": str(config.relative_to(ROOT)),
        "config_sha256": sha256(config),
        "seeds": sorted(int(x) for x in normalized["seed"].unique()),
        "analysis_date_utc": datetime.now(timezone.utc).isoformat(),
        "runtime": "analysis-only; original simulator runtime is retained beside the source CSV where available",
    }
    (out / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )


def analyze():
    confirmation_path = ROOT / "paper_results/revision/final_confirm/raw.csv"
    bridge_path = ROOT / "paper_results/revision/emrmf_bridge_v2/raw.csv"
    decay_path = ROOT / "paper_results/revision/decay/raw.csv"

    confirmation = pd.read_csv(confirmation_path)
    cp = paired_seed_means(
        confirmation, "arm", "plus_0.30_6", "random",
        ["seed", "condition", "n_robots"],
    )
    c_diff = difference_test(cp, "plus_0.30_6", "random")
    c_eq = tost_log_ratio(cp)
    c_rows = [
        {"analysis": "superiority", **c_diff},
        {"analysis": "equivalence", **c_eq},
    ]
    c_norm = confirmation.rename(columns={"n_robots": "N", "arm": "policy"})[
        ["seed", "condition", "N", "policy", "align_rmse", "pose_rmse"]
    ]
    write_experiment(
        "confirmation", confirmation_path, c_norm, c_rows,
        "7b74df7743d58eed4923bce72d2ecb8383d55ced", "confirmation.yaml",
    )

    bridge = pd.read_csv(bridge_path)
    bp = paired_seed_means(
        bridge, "policy", "plus_0.30_6", "emrmf_published",
        ["seed", "condition", "n_robots"],
    )
    b_diff = difference_test(bp, "plus_0.30_6", "emrmf_published")
    raw_reference = bridge.loc[bridge.policy == "emrmf_published", "align_rmse"].mean()
    raw_treatment = bridge.loc[bridge.policy == "plus_0.30_6", "align_rmse"].mean()
    b_diff["raw_observation_reduction_pct"] = float(
        100 * (raw_reference - raw_treatment) / raw_reference
    )
    b_norm = bridge.rename(columns={"n_robots": "N"})[
        ["seed", "condition", "N", "policy", "align_rmse", "pose_rmse"]
    ]
    write_experiment(
        "bridge", bridge_path, b_norm, [{"analysis": "primary", **b_diff}],
        "4cd2c0e25c6e500a22eebbf30f9869f41eef53d1", "bridge.yaml",
    )

    decay = pd.read_csv(decay_path)
    dp = paired_seed_means(
        decay, "rule", "deferral derived", "fixed 0.003",
        ["seed", "n_robots"],
    )
    d_diff = difference_test(dp, "deferral derived", "fixed 0.003")
    d_eq = tost_log_ratio(dp)
    drift = paired_seed_means(
        decay, "rule", "deferral derived", "drift derived",
        ["seed", "n_robots"],
    )
    drift_diff = difference_test(drift, "deferral derived", "drift derived")
    d_rows = [
        {"analysis": "oracle_comparison", **d_diff},
        {"analysis": "oracle_equivalence", **d_eq},
        {"analysis": "drift_comparison", **drift_diff},
    ]
    d_norm = decay.rename(columns={"n_robots": "N", "rule": "policy"})[
        ["seed", "N", "policy", "align_rmse", "pose_rmse"]
    ]
    d_norm.insert(1, "condition", "C0")
    write_experiment(
        "decay", decay_path, d_norm, d_rows,
        "d3f4b20faae3728ccbadcfd3125885d4f56a12d3", "decay.yaml",
    )

    combined = []
    for experiment, rows in (("confirmation", c_rows), ("bridge", [{"analysis": "primary", **b_diff}]), ("decay", d_rows)):
        for row in rows:
            combined.append({"experiment": experiment, **row})
    out = ROOT / "results" / "simulation"
    pd.DataFrame(combined).to_csv(out / "seed_level_tests.csv", index=False)
    print(json.dumps({
        "confirmation": {**c_diff, **c_eq},
        "bridge": b_diff,
        "decay_oracle": {**d_diff, **d_eq},
    }, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    analyze()


if __name__ == "__main__":
    main()
