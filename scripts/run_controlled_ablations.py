#!/usr/bin/env python3
"""Controlled ablation study on development seeds 0-9.

Isolates individual mechanisms between historical and v15 implementations:
1. Freshness weighting in ranking density
2. Premature queue drop on unblended predicted trust floor
3. Confidence blending in predicted spatial trust
4. Transmission cursor timing (t0 vs t1)
5. Temporal decay calibration (T_defer = 155s vs 72s)
6. Pairwise observability term (BACS vs BACS+)
7. Baselines: FIFO, Random, LIFO

Independent unit is the seed. Condition is nominal C0 (session 480s, team sizes 2-5).
"""

from __future__ import annotations

import argparse
import copy
import io
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import t, wilcoxon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bacs_sim.config import SimConfig, TrustConfig, InfoGainConfig, LoRaConfig
from bacs_sim.simulator import (
    precompute, _clone, _odometry_information, _align_first
)
from bacs_sim.lora import Channel, time_on_air, airtime_budget
from bacs_sim.trust import GammaController, server_trust
from bacs_sim.infogain import CoverageMap
from bacs_sim.observability import PairObservability
from bacs_sim.posegraph import PoseGraph
from bacs_sim.agents import make_agent
from bacs_sim.world import relative
from bacs_sim.manuscript_algorithm import Candidate, select_window


def run_ablation_cell(
    seed: int,
    n_robots: int,
    variant_name: str,
    precomputed,
    *,
    policy: str = "bacs_plus",
    freshness_rate_mode: str = "gamma",  # "zero" or "gamma"
    use_conf_blend: bool = False,
    allow_low_trust: bool = False,       # True: don't drop on trust < 1.5*floor
    t_cursor_mode: str = "t1",           # "t0" or "t1"
    t_defer: float = 155.0,              # half-life prior (155s or 72s)
    ranking_rule: str = "bacs",          # "bacs", "fifo", "random", "lifo"
):
    cfg = SimConfig(seed=seed)
    cfg.world.session_s = 480
    cfg.world.n_robots = n_robots
    cfg.scheduler.policy = policy
    cfg.scheduler.use_observability = (policy == "bacs_plus")
    cfg.trust.gamma = math.log(2) / t_defer
    w = cfg.world
    rng = np.random.default_rng(seed)

    truths, cands = precomputed
    cands = {k: [_clone(c) for c in v] for k, v in cands.items()}

    agents = {r: make_agent(r, cfg.agent(r), rng) for r in range(w.n_robots)}
    for rid, lst in cands.items():
        for c in lst:
            agents[rid].on_report(c)

    gamma_ctl = GammaController(cfg.trust, w)
    channel = Channel(cfg.channel, cfg.lora, rng)
    coverage = {r: CoverageMap(cfg.infogain) for r in range(w.n_robots)}
    pair_obs = PairObservability(cfg.infogain.obs_ref)

    budget_per_window = airtime_budget(cfg.scheduler.window_s, cfg.lora, w.n_robots)
    n_windows = int(np.ceil(w.session_s / cfg.scheduler.window_s))

    pending = {r: [] for r in range(w.n_robots)}
    delivered = []
    degree = {r: np.zeros(len(truths[r].gt), int) for r in range(w.n_robots)}
    last_map_time = 0.0
    airtime_used = 0.0
    airtime_avail = 0.0
    n_sent = 0
    n_expired_trust = 0
    n_expired_deferral = 0

    om = _odometry_information(w)
    graph = PoseGraph()
    for rid in range(w.n_robots):
        odo = truths[rid].odom
        for k in range(len(odo)):
            graph.add_node((rid, k), odo[k])
        for k in range(1, len(odo)):
            graph.add_edge(graph.idx((rid, k - 1)), graph.idx((rid, k)),
                           relative(odo[k - 1], odo[k]), om, 1.0)
    fused = {rid: truths[rid].odom.copy() for rid in range(w.n_robots)}

    for wi in range(n_windows):
        t0 = wi * cfg.scheduler.window_s
        t1 = t0 + cfg.scheduler.window_s
        gamma = gamma_ctl.update()

        for rid in range(w.n_robots):
            fresh = [c for c in cands[rid] if t0 <= c.t_created < t1]
            pending[rid].extend(fresh)
            if not pending[rid]:
                continue

            map_age = t1 - last_map_time
            candidates = pending[rid]
            airtime_avail += budget_per_window

            if ranking_rule == "fifo":
                order = sorted(candidates, key=lambda c: c.t_created)
                chosen = []
                used = 0.0
                for c in order:
                    t_air = time_on_air(c.payload_bytes, cfg.lora)
                    if used + t_air <= budget_per_window + 1e-12:
                        chosen.append(c)
                        used += t_air
                retained_candidates = [c for c in candidates if id(c) not in set(id(x) for x in chosen)]

            elif ranking_rule == "lifo":
                order = sorted(candidates, key=lambda c: -c.t_created)
                chosen = []
                used = 0.0
                for c in order:
                    t_air = time_on_air(c.payload_bytes, cfg.lora)
                    if used + t_air <= budget_per_window + 1e-12:
                        chosen.append(c)
                        used += t_air
                retained_candidates = [c for c in candidates if id(c) not in set(id(x) for x in chosen)]

            elif ranking_rule == "random":
                shuffled = list(candidates)
                rng.shuffle(shuffled)
                chosen = []
                used = 0.0
                for c in shuffled:
                    t_air = time_on_air(c.payload_bytes, cfg.lora)
                    if used + t_air <= budget_per_window + 1e-12:
                        chosen.append(c)
                        used += t_air
                retained_candidates = [c for c in candidates if id(c) not in set(id(x) for x in chosen)]

            else:  # BACS / BACS+
                features = []
                for i, c in enumerate(candidates):
                    pred = relative(fused[rid][c.idx_from], fused[c.rid_to][c.idx_to])
                    raw_res = float(np.linalg.norm((c.z - pred)[:2]))
                    span = abs(truths[rid].t[c.idx_from] - truths[c.rid_to].t[c.idx_to])

                    if use_conf_blend:
                        conf = 1.0 / (1.0 + (w.speed * max(map_age, 0.0)) / cfg.trust.tau_e)
                        spatial = max(0.0, 1.0 - (raw_res / cfg.trust.tau_e) ** cfg.trust.p)
                        spatial_b = conf * spatial + (1.0 - conf) * 1.0
                        res_eff = cfg.trust.tau_e * max(0.0, 1.0 - spatial_b) ** (1.0 / cfg.trust.p)
                    else:
                        res_eff = raw_res

                    features.append(Candidate(
                        sequence=i,
                        generated_s=c.t_created,
                        provisional_residual_m=res_eff,
                        novelty=coverage[rid].novelty(truths[rid].odom[c.idx_from][:2]),
                        normalized_degree=min(degree[rid][c.idx_from] / 8, 1.0),
                        normalized_loop_span=min(span / 60, 1.0),
                        robot_i=rid,
                        robot_j=c.rid_to,
                        delivered_pair_count=pair_obs.count(rid, c.rid_to),
                        payload_bytes=c.payload_bytes,
                        deferrals=c.deferrals,
                    ))

                f_rate = 0.0 if freshness_rate_mode == "zero" else gamma

                t_cfg = copy.copy(cfg.trust)
                if allow_low_trust:
                    t_cfg.floor = 0.0  # prevents drop in manuscript_algorithm line 162

                picked, retained = select_window(
                    features,
                    now_s=t1,
                    budget_s=budget_per_window,
                    window_s=cfg.scheduler.window_s,
                    gamma=gamma,
                    freshness_rate=f_rate,
                    trust_cfg=t_cfg,
                    info_cfg=cfg.infogain,
                    lora=cfg.lora,
                    bacs_plus=(policy == "bacs_plus"),
                    trust_gate=cfg.scheduler.trust_gate,
                    loss_probability=gamma_ctl.p_loss,
                )

                chosen = [candidates[f.sequence] for f in picked]
                for c, f in zip(chosen, picked):
                    c.dt_hat = f.predicted_age_s
                    c.theta_hat = f.predicted_trust

                retained_indices = set(f.sequence for f in retained)
                chosen_indices = set(f.sequence for f in picked)
                # Count dropped due to trust vs kept
                for idx, c in enumerate(candidates):
                    if idx not in chosen_indices and idx not in retained_indices:
                        if c.deferrals >= 12:
                            n_expired_deferral += 1
                        else:
                            n_expired_trust += 1

                retained_candidates = [candidates[f.sequence] for f in retained]

            pending[rid] = chosen + retained_candidates
            chosen_ids = set(id(c) for c in chosen)
            t_cursor = t0 if t_cursor_mode == "t0" else t1

            for c in chosen:
                c.t_sent = max(t_cursor, c.t_created)
                ok, t_recv, used, attempts = channel.transmit(c.payload_bytes, c.t_sent)
                t_cursor = t_recv if ok else t_cursor + used
                airtime_used += used
                n_sent += 1
                c.attempts = attempts
                c.delivered = ok
                c.t_recv = t_recv
                dt_actual = t_recv - c.t_created
                gamma_ctl.observe(dt_actual, not ok)
                if ok:
                    delivered.append(c)
                    coverage[rid].mark(truths[rid].odom[c.idx_from][:2])
                    degree[rid][c.idx_from] += 1
                    pair_obs.mark(c.rid_from, c.rid_to)

            rest = []
            for c in pending[rid]:
                if id(c) in chosen_ids:
                    continue
                c.deferrals += 1
                if c.deferrals < 12:
                    rest.append(c)
                else:
                    n_expired_deferral += 1
            pending[rid] = rest

        new_edges = [c for c in delivered if not getattr(c, "_ingested", False)]
        for c in new_edges:
            c._ingested = True
            dt = c.t_recv - c.t_created
            pred = relative(fused[c.rid_from][c.idx_from], fused[c.rid_to][c.idx_to])
            c.theta = server_trust(float(np.linalg.norm((c.z - pred)[:2])),
                                   dt, cfg.trust, gamma_ctl.value)
            graph.add_edge(graph.idx((c.rid_from, c.idx_from)),
                           graph.idx((c.rid_to, c.idx_to)), c.z, om, c.theta)

        if new_edges and (wi % max(w.fusion_every_windows, 1) == 0):
            Xk, _ = graph.optimize(iterations=6)
            for rid in range(w.n_robots):
                fused[rid] = np.array([Xk[graph.idx((rid, k))]
                                       for k in range(len(truths[rid].gt))])
            last_map_time = t1

    X, H_final = graph.optimize()

    # Pose RMSE
    errs = []
    for rid in range(w.n_robots):
        gt = truths[rid].gt
        est = np.array([X[graph.idx((rid, k))] for k in range(len(gt))])
        est_a = _align_first(est, gt)
        errs.append(np.linalg.norm(est_a[:, :2] - gt[:, :2], axis=1))
    pose_rmse = float(np.sqrt(np.mean(np.concatenate(errs) ** 2)))

    # Map-alignment RMSE
    align = []
    for a in range(w.n_robots):
        for b in range(a + 1, w.n_robots):
            Pa, Pb = truths[a].gt[:, :2], truths[b].gt[:, :2]
            stride = max(len(Pb) // 200, 1)
            jb = np.arange(0, len(Pb), stride)
            d = np.linalg.norm(Pa[:, None, :] - Pb[None, jb, :], axis=2)
            ii, jj = np.where(d < w.obs_radius)
            if len(ii) == 0:
                continue
            sel = np.linspace(0, len(ii) - 1, min(len(ii), 400)).astype(int)
            for k in sel:
                ka, kb = int(ii[k]), int(jb[jj[k]])
                ev = X[graph.idx((a, ka))][:2] - X[graph.idx((b, kb))][:2]
                gv = truths[a].gt[ka, :2] - truths[b].gt[kb, :2]
                align.append(np.linalg.norm(ev - gv))
    align_rmse = float(np.sqrt(np.mean(np.square(align)))) if align else float("nan")

    ages = [c.t_recv - c.t_created for c in delivered]
    mean_age = float(np.mean(ages)) if ages else float("nan")
    median_age = float(np.median(ages)) if ages else float("nan")
    outliers = sum(c.is_outlier for c in delivered)
    accepted_outliers = sum(c.is_outlier and c.theta > 0.10 for c in delivered)
    util = float(airtime_used / airtime_avail) if airtime_avail > 0 else 0.0

    return {
        "variant": variant_name,
        "seed": seed,
        "n_robots": n_robots,
        "align_rmse": align_rmse,
        "pose_rmse": pose_rmse,
        "n_delivered": len(delivered),
        "n_sent": n_sent,
        "outliers_delivered": outliers,
        "accepted_outliers": accepted_outliers,
        "expired_trust": n_expired_trust,
        "expired_deferral": n_expired_deferral,
        "mean_age_s": mean_age,
        "median_age_s": median_age,
        "airtime_util": util,
    }


def execute_job(args):
    seed, n, var_name, var_kwargs, truths_cands = args
    return run_ablation_cell(seed, n, var_name, truths_cands, **var_kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--seeds", type=int, nargs=2, default=[0, 9])
    parser.add_argument("--out", type=Path, default=ROOT / "results/ablation_study_dev0_9.csv")
    args = parser.parse_args()

    variants = {
        # 1. Historical baseline:
        "historical_bacs_plus": dict(
            policy="bacs_plus", freshness_rate_mode="zero", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t0", t_defer=155.0, ranking_rule="bacs"
        ),
        # 2. Corrected v15 as run in fadd...:
        "v15_current_flawed": dict(
            policy="bacs_plus", freshness_rate_mode="gamma", use_conf_blend=False,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=72.0, ranking_rule="bacs"
        ),
        # 3. Factor: freshness rate zero (keep all other v15 current):
        "v15_freshness_zero": dict(
            policy="bacs_plus", freshness_rate_mode="zero", use_conf_blend=False,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=72.0, ranking_rule="bacs"
        ),
        # 4. Factor: confidence blending (keep all other v15 current):
        "v15_conf_blend": dict(
            policy="bacs_plus", freshness_rate_mode="gamma", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=72.0, ranking_rule="bacs"
        ),
        # 5. Factor: allow low trust (no drop on trust floor):
        "v15_no_floor_drop": dict(
            policy="bacs_plus", freshness_rate_mode="gamma", use_conf_blend=False,
            allow_low_trust=True, t_cursor_mode="t1", t_defer=72.0, ranking_rule="bacs"
        ),
        # 6. Factor: t_cursor = t0 (start of window):
        "v15_t_cursor_t0": dict(
            policy="bacs_plus", freshness_rate_mode="gamma", use_conf_blend=False,
            allow_low_trust=False, t_cursor_mode="t0", t_defer=72.0, ranking_rule="bacs"
        ),
        # 7. Factor: calibration prior 155s:
        "v15_prior_155s": dict(
            policy="bacs_plus", freshness_rate_mode="gamma", use_conf_blend=False,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=155.0, ranking_rule="bacs"
        ),
        # 8. Combined Corrected Variant: Pure Info Density + Conf Blend + Prior 155s + t_cursor t1:
        "v15_corrected_clean_t1": dict(
            policy="bacs_plus", freshness_rate_mode="zero", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=155.0, ranking_rule="bacs"
        ),
        # 9. BACS (without pairwise term):
        "v15_bacs_no_pair": dict(
            policy="bacs_gated", freshness_rate_mode="zero", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=155.0, ranking_rule="bacs"
        ),
        # 10. Baselines under identical environment:
        "baseline_random": dict(
            policy="random", freshness_rate_mode="zero", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=155.0, ranking_rule="random"
        ),
        "baseline_fifo": dict(
            policy="fifo", freshness_rate_mode="zero", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=155.0, ranking_rule="fifo"
        ),
        "baseline_lifo": dict(
            policy="lifo", freshness_rate_mode="zero", use_conf_blend=True,
            allow_low_trust=False, t_cursor_mode="t1", t_defer=155.0, ranking_rule="lifo"
        ),
    }

    start_seed, end_seed = args.seeds
    seeds = list(range(start_seed, end_seed + 1))
    team_sizes = [2, 3, 4, 5]

    print(f"Precomputing worlds for seeds {start_seed}..{end_seed} and N in {team_sizes}...")
    precomputed_worlds = {}
    for s in seeds:
        for n in team_sizes:
            cfg = SimConfig(seed=s)
            cfg.world.session_s = 480
            cfg.world.n_robots = n
            precomputed_worlds[(s, n)] = precompute(cfg)

    jobs = []
    for var_name, var_kwargs in variants.items():
        for s in seeds:
            for n in team_sizes:
                jobs.append((s, n, var_name, var_kwargs, precomputed_worlds[(s, n)]))

    print(f"Total jobs: {len(jobs)} ({len(variants)} variants x {len(seeds)} seeds x {len(team_sizes)} team sizes). Running with {args.workers} workers...")
    t_start = time.time()
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for res in pool.map(execute_job, jobs):
            results.append(res)

    df = pd.DataFrame(results)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out, index=False)
    elapsed = time.time() - t_start
    print(f"Completed in {elapsed:.1f} s. Saved to {args.out}")

    # Summary table per variant averaged over seeds and team sizes:
    summary = df.groupby("variant").agg({
        "align_rmse": "mean",
        "pose_rmse": "mean",
        "n_delivered": "mean",
        "outliers_delivered": "mean",
        "accepted_outliers": "mean",
        "expired_trust": "mean",
        "expired_deferral": "mean",
        "mean_age_s": "mean",
        "airtime_util": "mean",
    }).reset_index()

    print("\n--- Summary of Ablations (Seeds 0-9, N=2-5, Condition C0) ---")
    print(summary.sort_values("align_rmse").to_string(index=False))


if __name__ == "__main__":
    main()
