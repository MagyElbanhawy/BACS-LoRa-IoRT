#!/usr/bin/env python3
"""Regenerate the numbers, tables and data figures of Sections 5 and 6 of manuscript v15.

Reads only files in this repository:
  runs/v15/<run>/confirmation/*.csv and bridge/*.csv   (seeds 70-99 and 100-129)
  reviewer_R1_3_geometry_outages/raw_results/*.csv      (seeds 130-159)
  results/ablation_study_dev0_9.csv                     (development seeds 0-9)
  hardware/attached_hardware_evidence/session_hws_bacs_001/  (session HWS-BACS-001)

Writes to results/manuscript_v15/ (stats.json, table_*.csv, figures/*.png) and the
receiver segmentation table to
  hardware/attached_hardware_evidence/session_hws_bacs_001/derived/receiver_segmentation.csv

Usage:  python scripts/make_manuscript_tables_figures.py [--run runs/v15/<hash>]
The seed is the unit of every test: paired differences are averaged within each seed
over team sizes and conditions, then a two-sided Wilcoxon signed-rank test is applied to
the 30 seed-level values. Holm correction is applied within each pre-specified family.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import t as tdist, wilcoxon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from stats import difference_test, paired_seed_means, tost_log_ratio  # noqa: E402

RUN = "runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213"
OUT = ROOT / "results" / "manuscript_v15"
HW = ROOT / "hardware" / "attached_hardware_evidence" / "session_hws_bacs_001"

# Pre-specified comparison families (treatment, reference)
CONF_FAMILY = [("plus_0.30_6", "random"), ("plus_0.30_6", "lifo"), ("plus_0.30_6", "fifo"),
               ("bacs_gated", "random"), ("bacs_gated", "lifo"), ("bacs_gated", "fifo"),
               ("plus_0.30_6", "bacs_gated"), ("lifo", "random"), ("random", "fifo"), ("lifo", "fifo")]
BRIDGE_FAMILY = [("fifo_defer", "emrmf_published"), ("emrmf_drift", "emrmf_published"),
                 ("random", "fifo_defer"), ("lifo", "fifo_defer"), ("bacs_gated", "random"),
                 ("plus_0.30_6", "bacs_gated"), ("plus_0.30_6", "random"), ("plus_0.30_6", "lifo"),
                 ("bacs_gated", "lifo"), ("plus_0.30_6", "emrmf_published"),
                 ("bacs_gated", "emrmf_published"), ("random", "emrmf_published"),
                 ("lifo", "emrmf_published"), ("lifo", "random")]
LABEL = {"fifo": "FIFO", "lifo": "LIFO", "random": "Random", "bacs_gated": "BACS",
         "plus_0.30_6": "BACS+", "emrmf_published": "EMRMF ref.", "emrmf_drift": "EMRMF drift-γ",
         "fifo_defer": "FIFO, γ_defer"}
MARK = {"fifo": "s", "lifo": "^", "random": "D", "bacs_gated": "o", "plus_0.30_6": "P",
        "emrmf_published": "X", "emrmf_drift": "v", "fifo_defer": "<"}
COLOR = {"fifo": "#7f7f7f", "lifo": "#1f77b4", "random": "#2ca02c", "bacs_gated": "#ff7f0e",
         "plus_0.30_6": "#d62728", "emrmf_published": "#000000", "emrmf_drift": "#8c564b",
         "fifo_defer": "#9467bd"}


def holm(ps):
    order = np.argsort(ps)
    m = len(ps)
    adj = np.empty(m)
    running = 0.0
    for k, i in enumerate(order):
        running = max(running, min(1.0, (m - k) * ps[i]))
        adj[i] = running
    return adj


def load_cells(run, block):
    files = sorted((run / block).glob("*.csv"))
    if not files:
        raise SystemExit(f"no cell files in {run / block}")
    return pd.concat([pd.read_csv(f) for f in files], ignore_index=True)


def test(df, a, b, metric="align_rmse"):
    sub = df.drop(columns="align_rmse").rename(columns={metric: "align_rmse"}) if metric != "align_rmse" else df
    paired = paired_seed_means(sub, "arm", a, b, ["seed", "condition", "n_robots"])
    d = difference_test(paired, a, b)
    e = tost_log_ratio(paired)
    return dict(a=a, b=b, mean_a=d["treatment_mean"], mean_b=d["reference_mean"],
                change_pct=100 * (d["treatment_mean"] / d["reference_mean"] - 1),
                W=d["wilcoxon_W"], p=d["wilcoxon_p"], lower=d["treatment_lower_seeds"],
                n=d["n_seeds"], diff=d["mean_difference"], diff_lo=d["difference_ci95_lo"],
                diff_hi=d["difference_ci95_hi"], gmr=e["geometric_mean_ratio"],
                gmr_lo=e["ratio_ci90_lo"], gmr_hi=e["ratio_ci90_hi"], p_tost=e["p_tost"])


def block_stats(df, family, conds):
    s = {}
    s["bycond"] = {c: df[df.condition == c].groupby("arm").align_rmse.mean().round(4).to_dict() for c in conds}
    s["byN"] = {str(n): df[df.n_robots == n].groupby("arm").align_rmse.mean().round(4).to_dict() for n in (2, 3, 4, 5)}
    s["mean"] = {m: df.groupby("arm")[m].mean().round(4).to_dict() for m in ("align_rmse", "pose_rmse", "n_sent", "n_delivered")}
    s["gt05"] = (df.groupby("arm").align_rmse.apply(lambda x: (x > 0.5).mean() * 100).round(1)).to_dict()
    tests = [test(df, a, b) for a, b in family]
    for r, h in zip(tests, holm([r["p"] for r in tests])):
        r["p_holm"] = float(h)
    s["tests"] = tests
    s["posetests"] = [test(df, a, b, "pose_rmse") for a, b in family]
    s["percond"] = {c: [test(df[df.condition == c], a, b) for a, b in family] for c in conds}
    s["perN"] = {str(n): [test(df[df.n_robots == n], a, b) for a, b in family] for n in (2, 3, 4, 5)}
    return s


def seed_ci(df, arm, metric="align_rmse"):
    s = df[df.arm == arm].groupby("seed")[metric].mean()
    return s.mean(), tdist.ppf(0.975, len(s) - 1) * s.std(ddof=1) / np.sqrt(len(s))


def subgroups(conf, bridge):
    rows = []
    for block, df in (("Confirmation", conf), ("Comparison", bridge)):
        pv = df.pivot_table(index=["seed", "condition", "n_robots"], columns="arm", values="align_rmse").reset_index()
        for c in sorted(df.condition.unique()):
            for n in (2, 3, 4, 5):
                s = pv[(pv.condition == c) & (pv.n_robots == n)]
                d = s["plus_0.30_6"] - s["random"]
                rows.append(dict(block=block, condition=c, N=n,
                                 change_pct=100 * (s["plus_0.30_6"].mean() / s["random"].mean() - 1),
                                 seeds_lower=int((d < 0).sum()), p=float(wilcoxon(d).pvalue)))
    for r, h in zip(rows, holm([r["p"] for r in rows])):
        r["p_holm"] = float(h)
    return rows


def c4_stats():
    base = ROOT / "reviewer_R1_3_geometry_outages" / "raw_results"
    out = {}
    for name, f in (("C4", "raw_results_C4.csv"), ("C4-IID", "raw_results_C4_IID.csv"), ("C4-IID-AL", "raw_results_C4_IID_AL.csv")):
        df = pd.read_csv(base / f)
        table = df.groupby("policy").agg(align=("align_rmse", "mean"), pose=("pose_rmse", "mean"),
                                         delivered=("n_delivered", "mean"), sent=("n_sent", "mean"),
                                         airtime_util=("airtime_util", "mean"),
                                         share_over_budget=("airtime_util", lambda s: (s > 1).mean()))
        byN = df.groupby(["n_robots", "policy"]).align_rmse.mean().unstack().round(3)
        pv = df.pivot_table(index=["seed", "n_robots"], columns="policy", values="align_rmse")
        tests = {}
        for a, b in (("bacs_plus", "fifo"), ("bacs_plus", "random"), ("bacs_plus", "lifo"),
                     ("bacs_plus", "bacs_gated"), ("random", "fifo"), ("lifo", "random")):
            d = (pv[a] - pv[b]).groupby("seed").mean()
            tests[f"{a}-{b}"] = dict(change_pct=100 * (pv[a].mean() / pv[b].mean() - 1),
                                     p=float(wilcoxon(d).pvalue), seeds_lower=int((d < 0).sum()))
        out[name] = dict(table=table.round(4).to_dict(), byN=byN.to_dict(), tests=tests,
                         obstruction=float(df.obstruction_fraction.mean()),
                         mean_outage_s=float(df[df.n_outages > 0].mean_outage_s.mean()),
                         max_outage_s=float(df.max_outage_s.max()),
                         attempt_delivery=float((df.n_los_delivered + df.n_nlos_delivered).sum()
                                                / (df.n_los_attempts + df.n_nlos_attempts).sum()))
        table.round(4).to_csv(OUT / f"table_c4_{name}.csv")
        byN.to_csv(OUT / f"table_c4_{name}_byN.csv")
    return out


def hardware():
    tx = pd.read_csv(HW / "transmitter" / "serial_log.csv")
    rx = pd.read_csv(HW / "receiver" / "receiver_serial_log.csv")
    sd = pd.read_csv(HW / "transmitter" / "scheduler_decisions.csv")
    h = tx[tx.direction == "HOST_TX"]
    dn = tx[tx.direction == "MR_TX_DONE"]
    m = h[["seq", "host_monotonic_ns"]].merge(dn[["seq", "host_monotonic_ns", "device_uptime_ms", "tx_duration_us"]],
                                               on="seq", suffixes=("_h", "_d"))
    handoff = (m.host_monotonic_ns_d - m.host_monotonic_ns_h) / 1e6
    r = rx[rx.event == "RX"].reset_index(drop=True)
    start = r.index[r.seq == 1][-1]          # the session starts at the last reception of sequence 1
    seg = r.loc[start:]
    first = seg.drop_duplicates("seq")
    dup = seg[seg.duplicated("seq")]
    pre = r.loc[:start - 1]
    other = rx[rx.event != "RX"]
    txi = m.set_index("seq").device_uptime_ms.diff()
    rxi = first.set_index("seq").device_uptime_ms.diff()
    dif = (txi - rxi).dropna()
    tsec = (sd.selected_at_ns - sd.selected_at_ns.iloc[0]) / 1e9
    n_gen = sd.pool_size_before + sd.tx_seq - 1
    a = np.polyfit(tsec, n_gen, 1)
    age = tsec - (sd.constraint_seq - a[1]) / a[0]
    # segmentation table (row-level), written beside the session as a derived file
    seg_rows = []
    for idx, row in rx.iterrows():
        if row.event != "RX":
            label = "excluded: noise/corrupt frame"
        elif idx < rx.index[rx.event == "RX"][start]:
            label = "excluded: earlier test transmission"
        elif row.name in dup.index.map(lambda i: rx.index[rx.event == "RX"][i]):
            label = "excluded: receiver double report"
        else:
            label = "counted: first reception in session"
        seg_rows.append(dict(row_index=idx, event=row.event, seq=row.get("seq"), device_uptime_ms=row.get("device_uptime_ms"),
                             rssi_dbm=row.get("rssi_dbm"), treatment=label))
    seg_df = pd.DataFrame(seg_rows)
    (HW / "derived").mkdir(exist_ok=True)
    seg_df.to_csv(HW / "derived" / "receiver_segmentation.csv", index=False)
    stats = dict(packets_tx=int(len(m)), packets_rx_unique=int(first.seq.nunique()),
                 duplicates=int(len(dup)), pre_session_rows=int(len(pre)), other_rows=int(len(other)),
                 airtime_ms_min=float(m.tx_duration_us.min() / 1000), airtime_ms_max=float(m.tx_duration_us.max() / 1000),
                 airtime_ms_mean=float(m.tx_duration_us.mean() / 1000),
                 handoff_ms_min=float(handoff.min()), handoff_ms_median=float(handoff.median()), handoff_ms_max=float(handoff.max()),
                 interval_diff_ms_min=float(dif.min()), interval_diff_ms_max=float(dif.max()), interval_diff_ms_mean=float(dif.mean()),
                 queue_first=int(sd.pool_size_before.iloc[0]), queue_last=int(sd.pool_size_before.iloc[-1]),
                 age_s_first=float(age.iloc[0]), age_s_median=float(age.median()), age_s_last=float(age.iloc[-1]),
                 rssi_min=float(first.rssi_dbm.min()), rssi_max=float(first.rssi_dbm.max()),
                 snr_min=float(first.snr_db.min()), snr_max=float(first.snr_db.max()),
                 segmentation_counts=seg_df.treatment.value_counts().to_dict())
    return stats, m, handoff, dif, sd, age


def figures(conf, bridge, c4, hwdata):
    fig_dir = OUT / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "serif", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    arms = ["fifo", "lifo", "random", "bacs_gated", "plus_0.30_6"]
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.8))
    for k, (key, cats, xl) in enumerate((("condition", ["C0", "C1", "C2", "C3"], "Communication condition"),
                                         ("n_robots", [2, 3, 4, 5], "Team size N"))):
        for j, a in enumerate(arms):
            ms, hs = zip(*[seed_ci(conf[conf[key] == c], a) for c in cats])
            ax[k].errorbar(np.arange(len(cats)) + (j - 2) * 0.09, ms, yerr=hs, marker=MARK[a], color=COLOR[a],
                           label=LABEL[a], ms=5, capsize=2, lw=1)
        ax[k].set_xticks(range(len(cats))); ax[k].set_xticklabels(cats); ax[k].set_xlabel(xl)
        ax[k].set_ylabel("Map-alignment RMSE (m)"); ax[k].set_title("(a)" if k == 0 else "(b)", loc="left")
    h, l = ax[0].get_legend_handles_labels()
    fig.legend(h, l, fontsize=7, ncol=5, frameon=False, loc="lower center")
    fig.tight_layout(rect=(0, 0.08, 1, 1)); fig.savefig(fig_dir / "fig_confirmation.png", dpi=300); plt.close(fig)

    pv = conf.pivot_table(index=["seed", "condition", "n_robots"], columns="arm", values="align_rmse")
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.6), sharey=True)
    for k, b in enumerate(["random", "lifo"]):
        d = (pv["plus_0.30_6"] - pv[b]).groupby("seed").mean().sort_values()
        ax[k].scatter(range(len(d)), d.values, c=["#d62728" if v > 0 else "#2ca02c" for v in d], s=14)
        ax[k].axhline(0, color="k", lw=0.7); ax[k].set_xlabel("Seed (sorted by effect)")
        ax[k].set_title(f"({'ab'[k]}) BACS+ − {LABEL[b]}", loc="left")
    ax[0].set_ylabel("Paired difference in\nalignment RMSE (m)")
    fig.tight_layout(); fig.savefig(fig_dir / "fig_seed_effects.png", dpi=300); plt.close(fig)

    barms = ["emrmf_published", "emrmf_drift", "fifo_defer", "random", "lifo", "bacs_gated", "plus_0.30_6"]
    fig, ax = plt.subplots(figsize=(7.0, 2.7))
    for j, a in enumerate(barms):
        ms, hs = zip(*[seed_ci(bridge[bridge.condition == c], a) for c in ["L0", "L1", "L2", "L3"]])
        ax.errorbar(np.arange(4) + (j - 3) * 0.08, ms, yerr=hs, marker=MARK[a], color=COLOR[a], label=LABEL[a], ms=5, capsize=2, lw=1)
    ax.set_xticks(range(4)); ax.set_xticklabels(["L0 (no loss)", "L1 (10%)", "L2 (30%)", "L3 (burst + 0.2 s)"])
    ax.set_ylabel("Map-alignment RMSE (m)")
    ax.legend(fontsize=7, ncol=4, frameon=False, loc="center", bbox_to_anchor=(0.5, 0.55))
    fig.tight_layout(); fig.savefig(fig_dir / "fig_comparison.png", dpi=300); plt.close(fig)

    base = ROOT / "reviewer_R1_3_geometry_outages" / "raw_results"
    dfs = [pd.read_csv(base / f) for f in ("raw_results_C4.csv", "raw_results_C4_IID.csv", "raw_results_C4_IID_AL.csv")]
    pol = {"fifo": "fifo", "lifo": "lifo", "random": "random", "bacs_gated": "bacs_gated", "bacs_plus": "plus_0.30_6"}
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
    for j, (p, key) in enumerate(pol.items()):
        ms, hs = [], []
        for df in dfs:
            s = df[df.policy == p].groupby("seed").align_rmse.mean()
            ms.append(s.mean()); hs.append(tdist.ppf(0.975, len(s) - 1) * s.std(ddof=1) / np.sqrt(len(s)))
        ax.errorbar(np.arange(3) + (j - 2) * 0.09, ms, yerr=hs, marker=MARK[key], color=COLOR[key], label=LABEL[key], ms=5, capsize=2, lw=1)
    ax.set_xticks(range(3)); ax.set_xticklabels(["C4: geometry-dependent blockage", "C4-IID: matched mean loss", "C4-IID-AL: matched attempt loss"])
    ax.set_ylabel("Map-alignment RMSE (m)"); ax.legend(fontsize=7, ncol=5, frameon=False, loc="center", bbox_to_anchor=(0.5, 0.6))
    fig.tight_layout(); fig.savefig(fig_dir / "fig_geometry.png", dpi=300); plt.close(fig)

    m, handoff, dif, sd, age = hwdata
    fig, ax = plt.subplots(1, 3, figsize=(7.0, 2.3))
    ax[0].plot(m.seq, m.tx_duration_us / 1000, "o", ms=3, label="On-air time (board)")
    ax[0].plot(m.seq, handoff, "^", ms=3, label="Hand-off → TX_DONE")
    ax[0].axhline(102.7, ls="--", color="k", lw=0.7, label="Model 102.7 ms")
    ax[0].set_xlabel("Packet"); ax[0].set_ylabel("ms"); ax[0].set_ylim(95, 135); ax[0].legend(fontsize=6, frameon=False, loc="upper center")
    ax[0].set_title("(a)", loc="left")
    ax[1].plot(dif.index, dif.values, "s", ms=3); ax[1].axhline(0, color="k", lw=0.7)
    ax[1].set_xlabel("Packet"); ax[1].set_ylabel("TX − RX interval (ms)"); ax[1].set_title("(b)", loc="left")
    ax[2].plot(sd.tx_seq, age, "D", ms=3, label="Age at send (s)")
    ax[2].plot(sd.tx_seq, sd.pool_size_before, "v", ms=3, label="FIFO queue length")
    ax[2].set_xlabel("Packet"); ax[2].legend(fontsize=6, frameon=False); ax[2].set_title("(c)", loc="left")
    fig.tight_layout(); fig.savefig(fig_dir / "fig_hardware.png", dpi=300); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", default=RUN)
    args = ap.parse_args()
    run = ROOT / args.run
    OUT.mkdir(parents=True, exist_ok=True)
    conf = load_cells(run, "confirmation")
    bridge = load_cells(run, "bridge")
    out = {}
    for name, df, fam, conds in (("confirmation", conf, CONF_FAMILY, ["C0", "C1", "C2", "C3"]),
                                 ("bridge", bridge, BRIDGE_FAMILY, ["L0", "L1", "L2", "L3"])):
        s = block_stats(df, fam, conds)
        for k, v in s.items():
            out[f"{name}_{k}"] = v
        pd.DataFrame(s["tests"]).to_csv(OUT / f"tests_{name}.csv", index=False)
        pd.DataFrame(s["bycond"]).T.to_csv(OUT / f"table_{name}_by_condition.csv")
        pd.DataFrame(s["byN"]).T.to_csv(OUT / f"table_{name}_by_team_size.csv")
    out["subgroups"] = subgroups(conf, bridge)
    pd.DataFrame(out["subgroups"]).to_csv(OUT / "table_subgroups.csv", index=False)
    out["c4"] = c4_stats()
    hw, *hwdata = hardware()
    out["hw"] = hw
    dev = pd.read_csv(ROOT / "results" / "ablation_study_dev0_9.csv")
    out["dev"] = dev.groupby("variant")[["align_rmse", "outliers_delivered", "accepted_outliers", "median_age_s",
                                          "mean_age_s", "n_delivered"]].mean().round(3).to_dict()
    pd.DataFrame(out["dev"]).to_csv(OUT / "table_dev_diagnostic.csv")
    (OUT / "stats.json").write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")
    figures(conf, bridge, out["c4"], hwdata)
    t = {(r["a"], r["b"]): r for r in out["bridge_tests"]}[("plus_0.30_6", "emrmf_published")]
    print(f"comparison: BACS+ {t['mean_a']:.3f} m vs EMRMF {t['mean_b']:.3f} m ({t['change_pct']:+.1f}%), "
          f"{t['lower']}/30 seeds lower, p = {t['p']:.2g}, Holm p = {t['p_holm']:.2g}")
    print(f"hardware: {hw['packets_rx_unique']}/{hw['packets_tx']} received, handoff median {hw['handoff_ms_median']:.0f} ms")
    print(f"outputs in {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
