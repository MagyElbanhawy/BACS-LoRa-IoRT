"""
Comprehensive Statistical Analysis and Multi-Condition Comparison Engine.

Evaluates and compares:
- Condition C4: Geometry-dependent outages (obstacle blockage)
- Condition C4-IID: Trajectory-average matched independent loss (P_drop = 0.2615)
- Condition C4-IID-AL: Attempt-level matched independent loss sensitivity (P_drop = 0.3739)

Computes:
- Factorial summary tables across seeds 130-159 (N=2,3,4,5, policies: FIFO, Random, LIFO, BACS, BACS+)
- Explicit renaming of "inlier acceptance rate" to "high-trust delivery fraction (%)" (theta > 0.10)
- Realized attempt-level delivery vs scheduled packet delivery vs unique packet delivery
- Seed-level paired comparisons with 95% bootstrap CIs, Cliff's delta, Wilcoxon signed-rank test
- Holm-Bonferroni correction within defined comparison families
- Outputs Markdown and CSV tables into tables/ and sensitivity_al/
"""
import os
import sys
import csv
import json
from typing import List, Dict, Any, Tuple
import numpy as np
from scipy import stats

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RAW_DIR = os.path.join(BASE_DIR, "raw_results")
SENSITIVITY_DIR = os.path.join(BASE_DIR, "sensitivity_al")
TABLES_DIR = os.path.join(BASE_DIR, "tables")

POLICIES = ["bacs_plus", "bacs_gated", "random", "lifo", "fifo"]
POLICY_LABELS = {
    "bacs_plus": "BACS+",
    "bacs_gated": "BACS",
    "random": "Random",
    "lifo": "LIFO",
    "fifo": "FIFO",
}
TEAM_SIZES = [2, 3, 4, 5]


def load_csv(filepath: str) -> List[Dict[str, Any]]:
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Missing file: {filepath}")
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
    nx, ny = len(x), len(y)
    if nx == 0 or ny == 0:
        return float("nan")
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


def calc_attempt_delivery_rate(row: Dict[str, Any]) -> float:
    tot_att = row["n_los_attempts"] + row["n_nlos_attempts"]
    tot_del = row["n_los_delivered"] + row["n_nlos_delivered"]
    return (tot_del / tot_att) if tot_att > 0 else float("nan")


def calc_packet_delivery_rate(row: Dict[str, Any]) -> float:
    sent = row["n_sent"]
    deliv = row["n_delivered"]
    return (deliv / sent) if sent > 0 else float("nan")


def format_table_condition(rows: List[Dict[str, Any]], condition_name: str, subtitle: str) -> str:
    metrics = [
        ("Map Alignment RMSE (m)", "align_rmse", "{:.3f}"),
        ("Trajectory ATE RMSE (m)", "pose_rmse", "{:.3f}"),
        ("Trust Yield (mean theta)", "trust_yield", "{:.3f}"),
        ("High-Trust Delivery Fraction (%)", "accept_rate", "{:.1%}"),
        ("Airtime Utilization (%)", "airtime_util", "{:.1%}"),
        ("Unique Packets Delivered", "n_delivered", "{:.1f}"),
        ("Attempt Delivery Rate (%)", "_attempt_deliv", "{:.1%}"),
        ("Max Deferrals (Starvation)", "starvation", "{:.1f}"),
    ]

    # Precalculate derived metrics
    augmented = []
    for r in rows:
        d = dict(r)
        d["_attempt_deliv"] = calc_attempt_delivery_rate(r)
        d["_packet_deliv"] = calc_packet_delivery_rate(r)
        augmented.append(d)

    md = []
    md.append(f"# Factorial Multi-Robot SLAM Performance under Condition {condition_name}\n")
    md.append(f"{subtitle}\n")
    md.append("| Metric | Policy | N = 2 Robots | N = 3 Robots | N = 4 Robots | N = 5 Robots | Mean Overall |")
    md.append("| :--- | :--- | :---: | :---: | :---: | :---: | :---: |")

    for metric_name, field_name, fmt in metrics:
        first = True
        for pol in POLICIES:
            label = POLICY_LABELS[pol]
            cell_strs = []
            vals_all = []
            for n in TEAM_SIZES:
                sub = [r[field_name] for r in augmented if r["policy"] == pol and r["n_robots"] == n and not np.isnan(r[field_name])]
                m_val = np.mean(sub) if sub else float("nan")
                s_val = np.std(sub, ddof=1) if len(sub) > 1 else 0.0
                vals_all.extend(sub)
                if "%" in fmt:
                    cell_strs.append(f"{m_val * 100:.1f} ± {s_val * 100:.1f}%")
                elif "{:.1f}" in fmt:
                    cell_strs.append(f"{m_val:.1f} ± {s_val:.1f}")
                else:
                    cell_strs.append(f"{m_val:.3f} ± {s_val:.3f}")
            overall_m = np.mean(vals_all) if vals_all else float("nan")
            overall_s = np.std(vals_all, ddof=1) if len(vals_all) > 1 else 0.0
            if "%" in fmt:
                overall_str = f"{overall_m * 100:.1f} ± {overall_s * 100:.1f}%"
            elif "{:.1f}" in fmt:
                overall_str = f"{overall_m:.1f} ± {overall_s:.1f}"
            else:
                overall_str = f"{overall_m:.3f} ± {overall_s:.3f}"

            prefix = f"**{metric_name}**" if first else ""
            first = False
            md.append(f"| {prefix} | {label} | {cell_strs[0]} | {cell_strs[1]} | {cell_strs[2]} | {cell_strs[3]} | {overall_str} |")

    return "\n".join(md)


def paired_policy_comparisons(rows: List[Dict[str, Any]], condition_name: str) -> str:
    comparisons = [
        ("BACS+ vs FIFO", "bacs_plus", "fifo"),
        ("BACS vs FIFO", "bacs_gated", "fifo"),
        ("BACS+ vs LIFO", "bacs_plus", "lifo"),
        ("BACS vs LIFO", "bacs_gated", "lifo"),
        ("BACS+ vs BACS", "bacs_plus", "bacs_gated"),
        ("BACS+ vs Random", "bacs_plus", "random"),
        ("BACS vs Random", "bacs_gated", "random"),
    ]

    md = []
    md.append(f"# Seed-Level Paired Statistical Comparisons of Map Alignment RMSE under Condition {condition_name}\n")
    md.append("Difference Delta = Metric(Policy A) - Metric(Policy B). Negative Delta indicates lower RMSE (superior alignment).\n")
    md.append("| Comparison | Team Size (N) | Mean Delta (m) | Median Delta (m) | 95% Bootstrap CI | Cliff's Delta | Wilcoxon W | Unadjusted p | Holm-Adjusted p |")
    md.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    for comp_label, pol_a, pol_b in comparisons:
        p_raw = []
        rows_data = []
        for n in TEAM_SIZES:
            seeds = sorted(list(set(r["seed"] for r in rows if r["n_robots"] == n)))
            diffs = []
            vals_a = []
            vals_b = []
            for s in seeds:
                row_a = next((r for r in rows if r["n_robots"] == n and r["seed"] == s and r["policy"] == pol_a), None)
                row_b = next((r for r in rows if r["n_robots"] == n and r["seed"] == s and r["policy"] == pol_b), None)
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


def cross_condition_comparisons(
    c4_rows: List[Dict[str, Any]],
    c4_iid_rows: List[Dict[str, Any]],
    c4_iid_al_rows: List[Dict[str, Any]],
) -> str:
    """Compare conditions across each policy."""
    condition_pairs = [
        ("C4 vs C4-IID", c4_rows, c4_iid_rows),
        ("C4 vs C4-IID-AL", c4_rows, c4_iid_al_rows),
        ("C4-IID vs C4-IID-AL", c4_iid_rows, c4_iid_al_rows),
    ]

    md = []
    md.append("# Seed-Level Paired Statistical Comparisons Across Communication Conditions\n")
    md.append("Difference Delta = Metric(Condition A) - Metric(Condition B). Evaluated on Map Alignment RMSE (m).\n")
    md.append("| Contrast | Policy | Team Size (N) | Mean Delta (m) | Median Delta (m) | 95% Bootstrap CI | Cliff's Delta | Wilcoxon W | Unadjusted p | Holm-Adjusted p |")
    md.append("| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    for cond_label, rows_a, rows_b in condition_pairs:
        for pol in ["bacs_plus", "bacs_gated", "lifo", "fifo", "random"]:
            pol_label = POLICY_LABELS[pol]
            p_raw = []
            rows_data = []
            for n in TEAM_SIZES:
                seeds = sorted(list(set(r["seed"] for r in rows_a if r["n_robots"] == n)))
                diffs = []
                vals_a = []
                vals_b = []
                for s in seeds:
                    ra = next((r for r in rows_a if r["n_robots"] == n and r["seed"] == s and r["policy"] == pol), None)
                    rb = next((r for r in rows_b if r["n_robots"] == n and r["seed"] == s and r["policy"] == pol), None)
                    if ra and rb and not np.isnan(ra["align_rmse"]) and not np.isnan(rb["align_rmse"]):
                        diff = ra["align_rmse"] - rb["align_rmse"]
                        diffs.append(diff)
                        vals_a.append(ra["align_rmse"])
                        vals_b.append(rb["align_rmse"])

                diffs = np.array(diffs)
                vals_a = np.array(vals_a)
                vals_b = np.array(vals_b)

                m_diff = float(np.mean(diffs))
                med_diff = float(np.median(diffs))
                ci_lo, ci_hi = bootstrap_ci(diffs)
                cd = cliffs_delta(vals_a, vals_b)

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
                md.append(f"| {cond_label} | {pol_label} | {n} | {m_diff:+.3f} | {med_diff:+.3f} | [{ci_lo:+.3f}, {ci_hi:+.3f}] | {cd:+.3f} | {w_stat:.1f} | {p_str} | {p_adj_str} |")

    return "\n".join(md)


def three_way_summary_table(
    c4_rows: List[Dict[str, Any]],
    c4_iid_rows: List[Dict[str, Any]],
    c4_iid_al_rows: List[Dict[str, Any]],
) -> str:
    """Generate comprehensive three-way summary table comparing C4, C4-IID, and C4-IID-AL."""
    md = []
    md.append("# Three-Way Factorial Comparison: C4 vs. C4-IID vs. C4-IID-AL Sensitivity\n")
    md.append("Evaluated over 30 pristine seeds (130–159). Mean across all team sizes (N = 2, 3, 4, 5).\n")
    md.append("| Policy | Condition | Attempt Delivery (%) | Packet Delivery (%) | Unique Packets Delivered | Mean Trust (theta) | High-Trust Delivery (%) | Map Alignment RMSE (m) | Trajectory ATE RMSE (m) | Airtime Util (%) |")
    md.append("| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    cond_map = [
        ("C4 (Obstacle Blockage)", c4_rows),
        ("C4-IID (Matched Traj P_drop=0.2615)", c4_iid_rows),
        ("C4-IID-AL (Sensitivity P_drop=0.3739)", c4_iid_al_rows),
    ]

    for pol in POLICIES:
        pol_label = POLICY_LABELS[pol]
        for cond_name, c_rows in cond_map:
            sub = [r for r in c_rows if r["policy"] == pol]

            # Attempt delivery rate
            tot_att = sum(r["n_los_attempts"] + r["n_nlos_attempts"] for r in sub)
            tot_del = sum(r["n_los_delivered"] + r["n_nlos_delivered"] for r in sub)
            att_rate = (tot_del / tot_att * 100.0) if tot_att > 0 else float("nan")

            # Packet delivery rate
            tot_sent = sum(r["n_sent"] for r in sub)
            tot_pdel = sum(r["n_delivered"] for r in sub)
            pkt_rate = (tot_pdel / tot_sent * 100.0) if tot_sent > 0 else float("nan")

            # Unique delivered packets
            mean_uniq = np.mean([r["n_delivered"] for r in sub])

            # Trust yield
            mean_trust = np.mean([r["trust_yield"] for r in sub])

            # High-trust delivery fraction (accept_rate)
            mean_high_trust = np.mean([r["accept_rate"] for r in sub]) * 100.0

            # Map alignment RMSE
            mean_align = np.mean([r["align_rmse"] for r in sub if not np.isnan(r["align_rmse"])])

            # Trajectory ATE RMSE
            mean_pose = np.mean([r["pose_rmse"] for r in sub if not np.isnan(r["pose_rmse"])])

            # Airtime util
            mean_air = np.mean([r["airtime_util"] for r in sub]) * 100.0

            md.append(f"| {pol_label} | {cond_name} | {att_rate:.1f}% | {pkt_rate:.1f}% | {mean_uniq:.1f} | {mean_trust:.3f} | {mean_high_trust:.1f}% | {mean_align:.3f} | {mean_pose:.3f} | {mean_air:.1f}% |")

    return "\n".join(md)


def save_markdown_and_csv(md_text: str, filepath_base: str):
    with open(f"{filepath_base}.md", "w", encoding="utf-8") as f:
        f.write(md_text)

    # Convert to CSV
    lines = md_text.splitlines()
    table_lines = [l.strip() for l in lines if l.strip().startswith("|") and not l.strip().startswith("| :")]
    if table_lines:
        rows = []
        for l in table_lines:
            cells = [c.strip().replace("**", "") for c in l.split("|")[1:-1]]
            rows.append(cells)
        with open(f"{filepath_base}.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(rows)
    print(f"Saved {filepath_base}.md and {filepath_base}.csv")


def main():
    os.makedirs(SENSITIVITY_DIR, exist_ok=True)
    os.makedirs(TABLES_DIR, exist_ok=True)

    c4_file = os.path.join(RAW_DIR, "raw_results_C4.csv")
    c4_iid_file = os.path.join(RAW_DIR, "raw_results_C4_IID.csv")
    c4_iid_al_file = os.path.join(SENSITIVITY_DIR, "raw_results_C4_IID_AL.csv")

    print(f"Loading C4: {c4_file}")
    c4_rows = load_csv(c4_file)
    print(f"Loading C4-IID: {c4_iid_file}")
    c4_iid_rows = load_csv(c4_iid_file)
    print(f"Loading C4-IID-AL: {c4_iid_al_file}")
    c4_iid_al_rows = load_csv(c4_iid_al_file)

    print(f"Loaded: C4={len(c4_rows)}, C4-IID={len(c4_iid_rows)}, C4-IID-AL={len(c4_iid_al_rows)}")

    # 1. Condition C4 table
    t_c4 = format_table_condition(
        c4_rows, "C4 (Geometry-Dependent Radio Outages)",
        "Evaluated over 30 pristine seeds (130–159). Mean ± standard deviation across seeds."
    )
    save_markdown_and_csv(t_c4, os.path.join(TABLES_DIR, "Table_S1_Condition_C4"))

    # 2. Condition C4-IID table
    t_c4_iid = format_table_condition(
        c4_iid_rows, "C4-IID (Trajectory-Average Matched Independent Loss, P_drop = 0.2615)",
        "Evaluated over 30 pristine seeds (130–159). Mean ± standard deviation across seeds."
    )
    save_markdown_and_csv(t_c4_iid, os.path.join(TABLES_DIR, "Table_S2_Condition_C4_IID"))

    # 3. Condition C4-IID-AL table
    t_c4_iid_al = format_table_condition(
        c4_iid_al_rows, "C4-IID-AL (Attempt-Level Matched Independent Loss Sensitivity, P_drop = 0.3739)",
        "Evaluated over 30 pristine seeds (130–159). Mean ± standard deviation across seeds."
    )
    save_markdown_and_csv(t_c4_iid_al, os.path.join(SENSITIVITY_DIR, "Table_S5_Condition_C4_IID_AL"))
    save_markdown_and_csv(t_c4_iid_al, os.path.join(TABLES_DIR, "Table_S5_Condition_C4_IID_AL"))

    # 4. Three-way summary table
    t_3way = three_way_summary_table(c4_rows, c4_iid_rows, c4_iid_al_rows)
    save_markdown_and_csv(t_3way, os.path.join(SENSITIVITY_DIR, "Table_Three_Way_Comparison"))
    save_markdown_and_csv(t_3way, os.path.join(TABLES_DIR, "Table_Three_Way_Comparison"))

    # 5. Paired policy comparisons under C4, C4-IID, C4-IID-AL
    p_c4 = paired_policy_comparisons(c4_rows, "C4")
    save_markdown_and_csv(p_c4, os.path.join(TABLES_DIR, "Table_S3_Paired_Comparisons_C4"))

    p_c4_iid = paired_policy_comparisons(c4_iid_rows, "C4-IID")
    save_markdown_and_csv(p_c4_iid, os.path.join(TABLES_DIR, "Table_Paired_Comparisons_C4_IID"))

    p_c4_iid_al = paired_policy_comparisons(c4_iid_al_rows, "C4-IID-AL")
    save_markdown_and_csv(p_c4_iid_al, os.path.join(SENSITIVITY_DIR, "Table_S6_Paired_Comparisons_C4_IID_AL"))
    save_markdown_and_csv(p_c4_iid_al, os.path.join(TABLES_DIR, "Table_S6_Paired_Comparisons_C4_IID_AL"))

    # 6. Cross-condition comparisons
    cross_cond = cross_condition_comparisons(c4_rows, c4_iid_rows, c4_iid_al_rows)
    save_markdown_and_csv(cross_cond, os.path.join(SENSITIVITY_DIR, "Table_Cross_Condition_Comparisons"))
    save_markdown_and_csv(cross_cond, os.path.join(TABLES_DIR, "Table_Cross_Condition_Comparisons"))

    print("All statistical analyses and comparison tables successfully generated.")


if __name__ == "__main__":
    main()
