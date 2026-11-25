"""
Statistical Analysis and Table Generation Engine for Reviewer 1.3.

Reads raw CSVs from raw_results/ and computes:
- Seed-level paired comparisons (Wilcoxon signed-rank test, Cliff's delta, bootstrap 95% CIs)
- Holm-Bonferroni correction across team sizes
- Generation of Supplementary Tables S1, S2, S3, S4 in Markdown and CSV formats in tables/
- Processed summary statistics in processed_results/
"""
import os
import sys
import csv
import math
from typing import List, Dict, Any, Tuple
import numpy as np
from scipy import stats

RESULTS_DIR = os.path.dirname(__file__)
RAW_DIR = os.path.join(RESULTS_DIR, "raw_results")
PROCESSED_DIR = os.path.join(RESULTS_DIR, "processed_results")
TABLES_DIR = os.path.join(RESULTS_DIR, "tables")

POLICIES = ["bacs_plus", "bacs_gated", "random", "lifo", "fifo"]
POLICY_LABELS = {
    "bacs_plus": "BACS+",
    "bacs_gated": "BACS",
    "random": "Random",
    "lifo": "LIFO",
    "fifo": "FIFO",
}
TEAM_SIZES = [2, 3, 4, 5]


def load_raw_csv(filepath: str) -> List[Dict[str, Any]]:
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Missing {filepath}")
    rows = []
    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            parsed = {}
            for k, v in r.items():
                if k in ("condition", "policy"):
                    parsed[k] = v
                elif k in ("seed", "n_robots", "n_candidates", "n_sent", "n_delivered",
                          "bytes_sent", "starvation", "n_outages", "n_los_attempts",
                          "n_los_delivered", "n_nlos_attempts", "n_nlos_delivered",
                          "n_accepted_inliers", "n_rejected_inliers",
                          "n_accepted_outliers", "n_rejected_outliers"):
                    parsed[k] = int(v) if v != "" else 0
                else:
                    try:
                        parsed[k] = float(v)
                    except ValueError:
                        parsed[k] = float("nan")
            rows.append(parsed)
    return rows


def bootstrap_ci(diffs: np.ndarray, n_boot: int = 10000, alpha: float = 0.05, seed: int = 42) -> Tuple[float, float]:
    """Compute 95% percentile bootstrap confidence interval of the mean difference."""
    rng = np.random.default_rng(seed)
    n = len(diffs)
    if n == 0:
        return (float("nan"), float("nan"))
    boot_means = np.empty(n_boot)
    for i in range(n_boot):
        sample = rng.choice(diffs, size=n, replace=True)
        boot_means[i] = np.mean(sample)
    lo = float(np.percentile(boot_means, 100.0 * (alpha / 2.0)))
    hi = float(np.percentile(boot_means, 100.0 * (1.0 - alpha / 2.0)))
    return lo, hi


def cliffs_delta(x: np.ndarray, y: np.ndarray) -> float:
    """Compute Cliff's delta non-parametric effect size between two samples."""
    nx, ny = len(x), len(y)
    if nx == 0 or ny == 0:
        return float("nan")
    # Compare all pairs
    greater = 0
    less = 0
    for xi in x:
        for yj in y:
            if xi > yj:
                greater += 1
            elif xi < yj:
                less += 1
    return (greater - less) / (nx * ny)


def holm_bonferroni(p_vals: List[float]) -> List[float]:
    """Adjust a list of p-values using the Holm-Bonferroni method."""
    m = len(p_vals)
    sorted_indices = np.argsort(p_vals)
    adj_p = [0.0] * m
    cum_max = 0.0
    for rank, idx in enumerate(sorted_indices):
        p = p_vals[idx]
        adj = p * (m - rank)
        cum_max = max(cum_max, adj)
        adj_p[idx] = min(1.0, cum_max)
    return adj_p


def generate_supplementary_table_s1(c4_rows: List[Dict[str, Any]]) -> str:
    """Generate Supplementary Table S1: Factorial Performance under Condition C4."""
    # Metrics to report: align_rmse, pose_rmse, trust_yield, accept_rate, airtime_util, starvation
    metrics = [
        ("Map Alignment RMSE (m)", "align_rmse", "{:.3f}"),
        ("Trajectory ATE RMSE (m)", "pose_rmse", "{:.3f}"),
        ("Trust Yield (mean theta)", "trust_yield", "{:.3f}"),
        ("Inlier Accept Rate (%)", "accept_rate", "{:.1%}"),
        ("Airtime Utilization (%)", "airtime_util", "{:.1%}"),
        ("Max Deferrals (Starvation)", "starvation", "{:.1f}"),
    ]

    md = []
    md.append("# Supplementary Table S1: Factorial Multi-Robot SLAM Performance under Condition C4 (Geometry-Dependent Radio Outages)\n")
    md.append("Evaluated over 30 independent pristine seeds (130–159). Values report mean ± standard deviation across seeds.\n")
    md.append("| Metric | Policy | N = 2 Robots | N = 3 Robots | N = 4 Robots | N = 5 Robots | Mean Overall |")
    md.append("| :--- | :--- | :---: | :---: | :---: | :---: | :---: |")

    for metric_name, field_name, fmt in metrics:
        first = True
        for pol in POLICIES:
            label = POLICY_LABELS[pol]
            cell_strs = []
            vals_all = []
            for n in TEAM_SIZES:
                sub = [r[field_name] for r in c4_rows if r["policy"] == pol and r["n_robots"] == n and not np.isnan(r[field_name])]
                m_val = np.mean(sub) if sub else float("nan")
                s_val = np.std(sub, ddof=1) if len(sub) > 1 else 0.0
                vals_all.extend(sub)
                if "%" in fmt:
                    cell_strs.append(f"{m_val * 100:.1f} ± {s_val * 100:.1f}%")
                else:
                    cell_strs.append(f"{m_val:.3f} ± {s_val:.3f}")
            overall_m = np.mean(vals_all) if vals_all else float("nan")
            overall_s = np.std(vals_all, ddof=1) if len(vals_all) > 1 else 0.0
            if "%" in fmt:
                overall_str = f"{overall_m * 100:.1f} ± {overall_s * 100:.1f}%"
            else:
                overall_str = f"{overall_m:.3f} ± {overall_s:.3f}"

            prefix = f"**{metric_name}**" if first else ""
            first = False
            md.append(f"| {prefix} | {label} | {cell_strs[0]} | {cell_strs[1]} | {cell_strs[2]} | {cell_strs[3]} | {overall_str} |")

    return "\n".join(md)


def generate_supplementary_table_s2(c4_iid_rows: List[Dict[str, Any]]) -> str:
    """Generate Supplementary Table S2: Factorial Performance under Condition C4-IID (Matched Independent Loss)."""
    metrics = [
        ("Map Alignment RMSE (m)", "align_rmse", "{:.3f}"),
        ("Trajectory ATE RMSE (m)", "pose_rmse", "{:.3f}"),
        ("Trust Yield (mean theta)", "trust_yield", "{:.3f}"),
        ("Inlier Accept Rate (%)", "accept_rate", "{:.1%}"),
        ("Airtime Utilization (%)", "airtime_util", "{:.1%}"),
        ("Max Deferrals (Starvation)", "starvation", "{:.1f}"),
    ]

    md = []
    md.append("# Supplementary Table S2: Factorial Multi-Robot SLAM Performance under Condition C4-IID (Matched Independent Packet Loss)\n")
    md.append("Evaluated over 30 independent pristine seeds (130–159) with matched drop rate P_drop = 0.2615. Values report mean ± standard deviation across seeds.\n")
    md.append("| Metric | Policy | N = 2 Robots | N = 3 Robots | N = 4 Robots | N = 5 Robots | Mean Overall |")
    md.append("| :--- | :--- | :---: | :---: | :---: | :---: | :---: |")

    for metric_name, field_name, fmt in metrics:
        first = True
        for pol in POLICIES:
            label = POLICY_LABELS[pol]
            cell_strs = []
            vals_all = []
            for n in TEAM_SIZES:
                sub = [r[field_name] for r in c4_iid_rows if r["policy"] == pol and r["n_robots"] == n and not np.isnan(r[field_name])]
                m_val = np.mean(sub) if sub else float("nan")
                s_val = np.std(sub, ddof=1) if len(sub) > 1 else 0.0
                vals_all.extend(sub)
                if "%" in fmt:
                    cell_strs.append(f"{m_val * 100:.1f} ± {s_val * 100:.1f}%")
                else:
                    cell_strs.append(f"{m_val:.3f} ± {s_val:.3f}")
            overall_m = np.mean(vals_all) if vals_all else float("nan")
            overall_s = np.std(vals_all, ddof=1) if len(vals_all) > 1 else 0.0
            if "%" in fmt:
                overall_str = f"{overall_m * 100:.1f} ± {overall_s * 100:.1f}%"
            else:
                overall_str = f"{overall_m:.3f} ± {overall_s:.3f}"

            prefix = f"**{metric_name}**" if first else ""
            first = False
            md.append(f"| {prefix} | {label} | {cell_strs[0]} | {cell_strs[1]} | {cell_strs[2]} | {cell_strs[3]} | {overall_str} |")

    return "\n".join(md)


def generate_supplementary_table_s3(c4_rows: List[Dict[str, Any]]) -> str:
    """Generate Supplementary Table S3: Paired Statistical Comparisons under Condition C4."""
    comparisons = [
        ("BACS+ vs Random", "bacs_plus", "random"),
        ("BACS+ vs LIFO", "bacs_plus", "lifo"),
        ("BACS+ vs FIFO", "bacs_plus", "fifo"),
        ("BACS vs Random", "bacs_gated", "random"),
    ]

    md = []
    md.append("# Supplementary Table S3: Seed-Level Paired Statistical Comparisons of Map Alignment RMSE under Condition C4\n")
    md.append("Difference Delta = Metric(Policy A) - Metric(Policy B). Negative Delta indicates lower RMSE (superior alignment).\n")
    md.append("| Comparison | Team Size (N) | Mean Delta (m) | Median Delta (m) | 95% Bootstrap CI | Cliff's Delta | Wilcoxon W | Unadjusted p | Holm-Adjusted p |")
    md.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    for comp_label, pol_a, pol_b in comparisons:
        p_raw = []
        rows_data = []
        for n in TEAM_SIZES:
            # Pair by seed
            seeds = sorted(list(set(r["seed"] for r in c4_rows if r["n_robots"] == n)))
            diffs = []
            vals_a = []
            vals_b = []
            for s in seeds:
                row_a = next((r for r in c4_rows if r["n_robots"] == n and r["seed"] == s and r["policy"] == pol_a), None)
                row_b = next((r for r in c4_rows if r["n_robots"] == n and r["seed"] == s and r["policy"] == pol_b), None)
                if row_a and row_b and not np.isnan(row_a["align_rmse"]) and not np.isnan(row_b["align_rmse"]):
                    diff = row_a["align_rmse"] - row_b["align_rmse"]
                    diffs.append(diff)
                    vals_a.append(row_a["align_rmse"])
                    vals_b.append(row_b["align_rmse"])

            diffs = np.array(diffs)
            vals_a = np.array(vals_a)
            vals_b = np.array(vals_b)

            m_diff = float(np.mean(diffs))
            med_diff = float(np.median(diffs))
            ci_lo, ci_hi = bootstrap_ci(diffs)
            cd = cliffs_delta(vals_a, vals_b)

            # Wilcoxon signed rank test
            # Handle cases where all diffs are zero
            if np.all(diffs == 0):
                w_stat, p_val = 0.0, 1.0
            else:
                try:
                    w_res = stats.wilcoxon(diffs, alternative="two-sided")
                    w_stat, p_val = float(w_res.statistic), float(w_res.pvalue)
                except Exception:
                    w_stat, p_val = float("nan"), 1.0

            p_raw.append(p_val)
            rows_data.append((n, m_diff, med_diff, ci_lo, ci_hi, cd, w_stat, p_val))

        p_adj = holm_bonferroni(p_raw)

        for i, (n, m_diff, med_diff, ci_lo, ci_hi, cd, w_stat, p_val) in enumerate(rows_data):
            p_adj_val = p_adj[i]
            p_str = "< 0.001" if p_val < 0.001 else f"{p_val:.4f}"
            p_adj_str = "< 0.001" if p_adj_val < 0.001 else f"{p_adj_val:.4f}"
            md.append(f"| {comp_label} | {n} | {m_diff:+.3f} | {med_diff:+.3f} | [{ci_lo:+.3f}, {ci_hi:+.3f}] | {cd:+.3f} | {w_stat:.1f} | {p_str} | {p_adj_str} |")

    return "\n".join(md)


def generate_supplementary_table_s4(c4_rows: List[Dict[str, Any]]) -> str:
    """Generate Supplementary Table S4: Geometry Outage Telemetry and LoS/NLoS Statistics."""
    md = []
    md.append("# Supplementary Table S4: Physical Obstacle Outage Telemetry and LoS/NLoS Delivery Breakdown\n")
    md.append("Outage statistics computed over all 600 s robot trajectories in the 14.0 m x 10.5 m arena with 6 warehouse racks.\n")
    md.append("| Team Size (N) | Obstruction Fraction (%) | Mean Outages / Session | Mean Outage Duration (s) | Max Outage Duration (s) | LoS Delivery Rate (%) | NLoS Delivery Rate (%) | Overall Delivery Rate (%) |")
    md.append("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    for n in TEAM_SIZES:
        sub = [r for r in c4_rows if r["n_robots"] == n]
        obs_frac = np.mean([r["obstruction_fraction"] for r in sub]) * 100.0
        n_outages = np.mean([r["n_outages"] for r in sub])
        mean_dur = np.mean([r["mean_outage_s"] for r in sub])
        max_dur = np.max([r["max_outage_s"] for r in sub])

        los_att = sum(r["n_los_attempts"] for r in sub)
        los_del = sum(r["n_los_delivered"] for r in sub)
        los_rate = (los_del / los_att * 100.0) if los_att > 0 else float("nan")

        nlos_att = sum(r["n_nlos_attempts"] for r in sub)
        nlos_del = sum(r["n_nlos_delivered"] for r in sub)
        nlos_rate = (nlos_del / nlos_att * 100.0) if nlos_att > 0 else float("nan")

        tot_att = los_att + nlos_att
        tot_del = los_del + nlos_del
        tot_rate = (tot_del / tot_att * 100.0) if tot_att > 0 else float("nan")

        md.append(f"| {n} | {obs_frac:.1f}% | {n_outages:.1f} | {mean_dur:.1f} s | {max_dur:.1f} s | {los_rate:.1f}% | {nlos_rate:.1f}% | {tot_rate:.1f}% |")

    # Overall summary row
    obs_frac_all = np.mean([r["obstruction_fraction"] for r in c4_rows]) * 100.0
    n_outages_all = np.mean([r["n_outages"] for r in c4_rows])
    mean_dur_all = np.mean([r["mean_outage_s"] for r in c4_rows])
    max_dur_all = np.max([r["max_outage_s"] for r in c4_rows])
    los_att_all = sum(r["n_los_attempts"] for r in c4_rows)
    los_del_all = sum(r["n_los_delivered"] for r in c4_rows)
    los_rate_all = (los_del_all / los_att_all * 100.0) if los_att_all > 0 else float("nan")
    nlos_att_all = sum(r["n_nlos_attempts"] for r in c4_rows)
    nlos_del_all = sum(r["n_nlos_delivered"] for r in c4_rows)
    nlos_rate_all = (nlos_del_all / nlos_att_all * 100.0) if nlos_att_all > 0 else float("nan")
    tot_att_all = los_att_all + nlos_att_all
    tot_del_all = los_del_all + nlos_del_all
    tot_rate_all = (tot_del_all / tot_att_all * 100.0) if tot_att_all > 0 else float("nan")

    md.append(f"| **Factorial Average** | **{obs_frac_all:.1f}%** | **{n_outages_all:.1f}** | **{mean_dur_all:.1f} s** | **{max_dur_all:.1f} s** | **{los_rate_all:.1f}%** | **{nlos_rate_all:.1f}%** | **{tot_rate_all:.1f}%** |")

    return "\n".join(md)


def save_table(md_content: str, base_name: str) -> None:
    """Save table as both Markdown and CSV in tables/ directory."""
    md_path = os.path.join(TABLES_DIR, f"{base_name}.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(md_content)

    csv_path = os.path.join(TABLES_DIR, f"{base_name}.csv")
    lines = md_content.splitlines()
    table_lines = [l.strip() for l in lines if l.strip().startswith("|") and not l.strip().startswith("| :")]
    if table_lines:
        rows = []
        for l in table_lines:
            cells = [c.strip().replace("**", "") for c in l.split("|")[1:-1]]
            rows.append(cells)
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(rows)
    print(f"Saved {base_name}.md and {base_name}.csv")


def main():
    os.makedirs(PROCESSED_DIR, exist_ok=True)
    os.makedirs(TABLES_DIR, exist_ok=True)

    print("Loading raw experimental results...")
    c4_file = os.path.join(RAW_DIR, "raw_results_C4.csv")
    c4_iid_file = os.path.join(RAW_DIR, "raw_results_C4_IID.csv")
    c0_file = os.path.join(RAW_DIR, "raw_results_C0.csv")

    if not os.path.exists(c4_file) or not os.path.exists(c4_iid_file):
        print(f"Error: Required raw CSV files not found in {RAW_DIR}")
        sys.exit(1)

    c4_rows = load_raw_csv(c4_file)
    c4_iid_rows = load_raw_csv(c4_iid_file)
    print(f"Loaded {len(c4_rows)} C4 runs and {len(c4_iid_rows)} C4-IID runs.")

    # Generate Supplementary Tables
    print("Generating Supplementary Table S1...")
    t_s1 = generate_supplementary_table_s1(c4_rows)
    save_table(t_s1, "Supplementary_Table_S1")

    print("Generating Supplementary Table S2...")
    t_s2 = generate_supplementary_table_s2(c4_iid_rows)
    save_table(t_s2, "Supplementary_Table_S2")

    print("Generating Supplementary Table S3...")
    t_s3 = generate_supplementary_table_s3(c4_rows)
    save_table(t_s3, "Supplementary_Table_S3")

    print("Generating Supplementary Table S4...")
    t_s4 = generate_supplementary_table_s4(c4_rows)
    save_table(t_s4, "Supplementary_Table_S4")

    print("All supplementary tables generated successfully in tables/")


if __name__ == "__main__":
    main()
