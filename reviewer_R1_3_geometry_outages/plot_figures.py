"""
Publication Figure Generation Script for Reviewer 1.3.

Produces:
- Figure S1: Environment geometry, 6 racks, server antenna, robot trajectories, LoS/NLoS rays.
- Figure S2: Outage timeline and link state dynamics over 600 s.
- Figure S3: Map alignment RMSE comparison across C0, C4-IID, C4 for all 5 policies.
- Figure S4: Seed-level paired performance difference plots (BACS+ vs baselines).
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Ensure project root in sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from bacs_sim.config import SimConfig
from bacs_sim.simulator import precompute

from reviewer_R1_3_geometry_outages.geometry_model import (
    DEFAULT_OBSTACLES,
    DEFAULT_SERVER_POS,
    check_link_blocked,
    line_intersects_rect,
)
from reviewer_R1_3_geometry_outages.analyze_results import (
    load_raw_csv,
    bootstrap_ci,
    POLICIES,
    POLICY_LABELS,
    TEAM_SIZES,
)

FIGURES_DIR = os.path.join(os.path.dirname(__file__), "figures")
RAW_DIR = os.path.join(os.path.dirname(__file__), "raw_results")


def set_paper_style():
    """Configure matplotlib for clean publication styling."""
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 12,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 13,
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linestyle": "--",
    })


def plot_figure_s1_geometry():
    """Figure S1: Arena geometry, 6 racks, server antenna, robot trajectories, LoS/NLoS rays."""
    set_paper_style()
    fig, ax = plt.subplots(figsize=(8.5, 6.5))

    # Arena boundary 14.0 x 10.5 m
    W, H = 14.0, 10.5
    arena_rect = patches.Rectangle((0, 0), W, H, linewidth=2, edgecolor="#2c3e50", facecolor="#f8f9fa")
    ax.add_patch(arena_rect)

    # Plot obstacles (6 warehouse storage racks)
    for obs in DEFAULT_OBSTACLES:
        w = obs.x_max - obs.x_min
        h = obs.y_max - obs.y_min
        rack = patches.Rectangle(
            (obs.x_min, obs.y_min), w, h,
            linewidth=1.5, edgecolor="#34495e", facecolor="#7f8c8d", alpha=0.75, hatch="//"
        )
        ax.add_patch(rack)
        # Label racks
        ax.text(
            obs.x_min + w / 2, obs.y_min + h / 2, obs.name.replace("Rack_", "").replace("_", " "),
            ha="center", va="center", color="white", fontsize=8, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2", facecolor="#2c3e50", alpha=0.8, edgecolor="none")
        )

    # Plot fusion server antenna
    sx, sy = DEFAULT_SERVER_POS
    ax.plot(sx, sy, marker="^", markersize=14, color="#e74c3c", markeredgecolor="#962d22", markeredgewidth=2, zorder=10)
    ax.text(sx, sy + 0.3, "Fusion Server Antenna\n(7.0, 10.5) m", ha="center", va="bottom", fontsize=9, fontweight="bold", color="#c0392b")

    # Generate sample robot trajectories (N=4, seed 130)
    cfg = SimConfig(seed=130)
    cfg.world.n_robots = 4
    truths, _ = precompute(cfg)
    colors = ["#2980b9", "#27ae60", "#8e44ad", "#d35400"]
    for rid, truth in enumerate(truths):
        traj = truth.gt[:, :2]
        ax.plot(traj[:, 0], traj[:, 1], color=colors[rid], linewidth=1.8, alpha=0.8, label=f"Robot {rid} Trajectory")
        ax.plot(traj[0, 0], traj[0, 1], marker="o", markersize=6, color=colors[rid])

    # Illustrate LoS ray (clear aisle path)
    p_los = (5.0, 5.0)  # Central aisle
    ax.plot([p_los[0], sx], [p_los[1], sy], color="#27ae60", linewidth=2.5, linestyle="--", label="Unobstructed LoS ($P_{deliv} = 0.95$)", zorder=9)
    ax.plot(p_los[0], p_los[1], marker="s", markersize=8, color="#27ae60", zorder=10)

    # Illustrate NLoS ray (obstructed by Rack_North_West)
    p_nlos = (2.5, 4.5)  # Behind rack
    ax.plot([p_nlos[0], sx], [p_nlos[1], sy], color="#e74c3c", linewidth=2.5, linestyle="-", label="Obstructed NLoS ($P_{deliv} = 0.10$)", zorder=9)
    ax.plot(p_nlos[0], p_nlos[1], marker="X", markersize=9, color="#e74c3c", zorder=10)

    ax.set_xlim(-0.5, W + 0.5)
    ax.set_ylim(-0.5, H + 1.2)
    ax.set_xlabel("X Coordinate (m)")
    ax.set_ylabel("Y Coordinate (m)")
    ax.set_title("Supplementary Figure S1: Industrial Warehouse Environment with 6 Racks and Server Antenna", pad=15)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.22), ncol=3, frameon=True)
    plt.tight_layout()

    out_png = os.path.join(FIGURES_DIR, "Figure_S1_Geometry_Arena.png")
    out_pdf = os.path.join(FIGURES_DIR, "Figure_S1_Geometry_Arena.pdf")
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.savefig(out_pdf, bbox_inches="tight")
    plt.close()
    print(f"Generated Figure S1: {out_png}")


def plot_figure_s2_timeline():
    """Figure S2: Outage timeline and link state dynamics over 600 s."""
    set_paper_style()
    cfg = SimConfig(seed=130)
    cfg.world.n_robots = 4
    truths, _ = precompute(cfg)

    t = truths[0].t
    dt = cfg.world.dt

    fig, axes = plt.subplots(4, 1, figsize=(9.5, 7.0), sharex=True)

    colors = ["#2980b9", "#27ae60", "#8e44ad", "#d35400"]
    for rid in range(4):
        ax = axes[rid]
        pts = truths[rid].gt[:, :2]
        blocked = np.array([check_link_blocked(tuple(p), DEFAULT_SERVER_POS, DEFAULT_OBSTACLES) for p in pts])

        # Fill LoS vs NLoS
        ax.fill_between(t, 0, 1, where=~blocked, color="#d5f5e3", alpha=0.8, label="LoS ($P_{deliv}=0.95$)" if rid == 0 else "")
        ax.fill_between(t, 0, 1, where=blocked, color="#fadbd8", alpha=0.8, label="NLoS Shadow ($P_{deliv}=0.10$)" if rid == 0 else "")

        # Draw window boundaries
        for w_idx in range(1, 10):
            ax.axvline(w_idx * 60.0, color="#7f8c8d", linestyle=":", linewidth=0.8)

        # Plot blockage state curve
        ax.plot(t, blocked.astype(int), color=colors[rid], linewidth=1.5, drawstyle="steps-post")

        frac = np.mean(blocked) * 100.0
        ax.set_ylabel(f"Robot {rid}\nBlockage", fontsize=9)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["LoS", "NLoS"])
        ax.set_ylim(-0.1, 1.1)
        ax.text(595, 0.5, f"Blocked: {frac:.1f}%", ha="right", va="center", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.8))

    axes[0].legend(loc="upper left", bbox_to_anchor=(0.0, 1.35), ncol=2, frameon=True)
    axes[0].set_title("Supplementary Figure S2: Temporal Radio Blockage Dynamics across 10 Scheduling Windows (600 s)", pad=20)
    axes[-1].set_xlabel("Session Elapsed Time (s)")
    axes[-1].set_xlim(0, 600)
    plt.tight_layout()

    out_png = os.path.join(FIGURES_DIR, "Figure_S2_Outage_Timeline.png")
    out_pdf = os.path.join(FIGURES_DIR, "Figure_S2_Outage_Timeline.pdf")
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.savefig(out_pdf, bbox_inches="tight")
    plt.close()
    print(f"Generated Figure S2: {out_png}")


def plot_figure_s3_alignment_rmse(c4_rows, c4_iid_rows, c0_rows=None):
    """Figure S3: Map alignment RMSE comparison across communication conditions for all 5 policies."""
    set_paper_style()
    fig, axes = plt.subplots(1, 4, figsize=(12.0, 4.5), sharey=True)

    palette = {
        "bacs_plus": "#2980b9",
        "bacs_gated": "#3498db",
        "random": "#e67e22",
        "lifo": "#9b59b6",
        "fifo": "#e74c3c",
    }

    cond_specs = []
    if c0_rows is not None and len(c0_rows) > 0:
        cond_specs.append(("C0\nBaseline", c0_rows))
    cond_specs.append(("C4-IID\nMatched", c4_iid_rows))
    cond_specs.append(("C4\nGeometry", c4_rows))

    n_conds = len(cond_specs)
    n_pols = len(POLICIES)
    stride = n_pols + 1

    for idx, n in enumerate(TEAM_SIZES):
        ax = axes[idx]
        x_positions = []
        box_data = []
        box_colors = []

        pos_cursor = 0
        group_centers = []
        group_labels = []

        for c_idx, (cond_label, rows) in enumerate(cond_specs):
            start_pos = pos_cursor
            for p_idx, pol in enumerate(POLICIES):
                sub = [r["align_rmse"] for r in rows if r["n_robots"] == n and r["policy"] == pol and not np.isnan(r["align_rmse"])]
                box_data.append(sub if sub else [0])
                x_positions.append(pos_cursor)
                box_colors.append(palette[pol])
                pos_cursor += 1
            group_centers.append(start_pos + (n_pols - 1) / 2.0)
            group_labels.append(cond_label)
            pos_cursor += 1  # Gap between conditions

        bp = ax.boxplot(
            box_data, positions=x_positions, widths=0.75, patch_artist=True,
            showmeans=True, meanline=True,
            medianprops=dict(color="black", linewidth=1.5),
            meanprops=dict(color="blue", linewidth=1.2, linestyle="--"),
            flierprops=dict(marker="o", markersize=3, alpha=0.5)
        )

        for patch, col in zip(bp["boxes"], box_colors):
            patch.set_facecolor(col)
            patch.set_alpha(0.75)

        ax.set_xticks(group_centers)
        ax.set_xticklabels(group_labels, fontsize=8)
        ax.set_title(f"Team Size N = {n}", fontsize=11, fontweight="bold")
        if idx == 0:
            ax.set_ylabel("Map Alignment RMSE (m)", fontsize=10)

    # Custom legend for policies
    handles = [patches.Patch(facecolor=palette[p], alpha=0.75, label=POLICY_LABELS[p]) for p in POLICIES]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 1.05), ncol=5, frameon=True)
    fig.suptitle("Supplementary Figure S3: Map Alignment RMSE Across Communication Conditions", y=1.12, fontsize=12, fontweight="bold")
    plt.tight_layout()

    out_png = os.path.join(FIGURES_DIR, "Figure_S3_Alignment_RMSE_Comparison.png")
    out_pdf = os.path.join(FIGURES_DIR, "Figure_S3_Alignment_RMSE_Comparison.pdf")
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.savefig(out_pdf, bbox_inches="tight")
    plt.close()
    print(f"Generated Figure S3: {out_png}")


def plot_figure_s4_seed_differences(c4_rows):
    """Figure S4: Seed-level paired performance difference plots (BACS+ vs baselines)."""
    set_paper_style()
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 4.0), sharey=True)

    baselines = [("random", "Random", "#e67e22"), ("lifo", "LIFO", "#9b59b6"), ("fifo", "FIFO", "#e74c3c")]

    for b_idx, (base_pol, base_label, color) in enumerate(baselines):
        ax = axes[b_idx]
        diffs_by_team = []
        ci_los = []
        ci_his = []
        means = []

        for n in TEAM_SIZES:
            seeds = sorted(list(set(r["seed"] for r in c4_rows if r["n_robots"] == n)))
            diffs = []
            for s in seeds:
                row_bp = next((r for r in c4_rows if r["n_robots"] == n and r["seed"] == s and r["policy"] == "bacs_plus"), None)
                row_base = next((r for r in c4_rows if r["n_robots"] == n and r["seed"] == s and r["policy"] == base_pol), None)
                if row_bp and row_base and not np.isnan(row_bp["align_rmse"]) and not np.isnan(row_base["align_rmse"]):
                    diffs.append(row_bp["align_rmse"] - row_base["align_rmse"])

            diffs = np.array(diffs)
            diffs_by_team.append(diffs)
            m = np.mean(diffs) if len(diffs) > 0 else 0.0
            lo, hi = bootstrap_ci(diffs)
            means.append(m)
            ci_los.append(m - lo)
            ci_his.append(hi - m)

            # Scatter individual seed points
            jitter = np.random.default_rng(s).uniform(-0.15, 0.15, size=len(diffs))
            ax.scatter(n + jitter, diffs, color=color, alpha=0.45, s=20, edgecolors="none")

        # Plot error bars for mean and 95% bootstrap CI
        ax.errorbar(
            TEAM_SIZES, means, yerr=[ci_los, ci_his], fmt="o-",
            color="black", ecolor="#2c3e50", elinewidth=2, capsize=4, capthick=1.5,
            markersize=6, label="Mean ± 95% Bootstrap CI"
        )

        ax.axhline(0.0, color="#c0392b", linestyle="--", linewidth=1.2, label="Parity ($\Delta = 0$)")
        ax.set_title(f"$\Delta$(BACS+ − {base_label}) under C4", fontsize=11, fontweight="bold")
        ax.set_xlabel("Robot Team Size (N)")
        ax.set_xticks(TEAM_SIZES)
        if b_idx == 0:
            ax.set_ylabel("Pairwise Alignment Difference $\Delta e_{align}$ (m)", fontsize=10)
        ax.legend(loc="upper left" if b_idx == 0 else "best", fontsize=8)

    fig.suptitle("Supplementary Figure S4: Seed-Level Paired Differences (Negative indicates BACS+ Superiority)", y=1.05, fontsize=12, fontweight="bold")
    plt.tight_layout()

    out_png = os.path.join(FIGURES_DIR, "Figure_S4_Seed_Level_Differences.png")
    out_pdf = os.path.join(FIGURES_DIR, "Figure_S4_Seed_Level_Differences.pdf")
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.savefig(out_pdf, bbox_inches="tight")
    plt.close()
    print(f"Generated Figure S4: {out_png}")


def main():
    os.makedirs(FIGURES_DIR, exist_ok=True)
    print("Generating Figure S1 (Environment Geometry)...")
    plot_figure_s1_geometry()

    print("Generating Figure S2 (Outage Timeline)...")
    plot_figure_s2_timeline()

    # Load results if ready
    c4_file = os.path.join(RAW_DIR, "raw_results_C4.csv")
    c4_iid_file = os.path.join(RAW_DIR, "raw_results_C4_IID.csv")
    c0_file = os.path.join(RAW_DIR, "raw_results_C0.csv")

    c4_rows = None
    if os.path.exists(c4_file):
        print("Loading C4 data for Figure S4...")
        c4_rows = load_raw_csv(c4_file)
        print("Generating Figure S4 (Seed-Level Differences)...")
        plot_figure_s4_seed_differences(c4_rows)

    if c4_rows is not None and os.path.exists(c4_iid_file):
        print("Loading C4-IID data for Figure S3...")
        c4_iid_rows = load_raw_csv(c4_iid_file)
        c0_rows = load_raw_csv(c0_file) if os.path.exists(c0_file) else None
        print("Generating Figure S3 (Alignment RMSE Comparison)...")
        plot_figure_s3_alignment_rmse(c4_rows, c4_iid_rows, c0_rows)


if __name__ == "__main__":
    main()
