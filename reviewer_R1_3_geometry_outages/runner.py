"""
Simulation Execution Engine for Reviewer 1.3 (Conditions C4 and C4-IID).

Executes full multi-robot SLAM sessions under physical obstacle blockage (C4)
and matched independent packet loss (C4-IID).
Maintains strict candidate pool isolation via precomputed ground truth and candidates,
ensuring all policies (FIFO, Random, LIFO, BACS, BACS+) compete on identical candidate sets.
"""
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

from bacs_sim.config import SimConfig
from bacs_sim.world import relative, wrap
from bacs_sim.lora import airtime_budget, time_on_air
from bacs_sim.trust import GammaController, server_trust
from bacs_sim.infogain import CoverageMap
from bacs_sim.observability import PairObservability
from bacs_sim.schedulers import schedule
from bacs_sim.posegraph import PoseGraph
from bacs_sim.agents import make_agent
from bacs_sim.manuscript_algorithm import Candidate, select_window
from bacs_sim.simulator import (
    RunResult,
    _align_first,
    _clone,
    precompute,
    _odometry_information,
)

from .geometry_model import (
    DEFAULT_OBSTACLES,
    DEFAULT_SERVER_POS,
    check_link_blocked,
    compute_outage_statistics,
)
from .channel import GeometryLoRaChannel, MatchedIIDChannel, MATCHED_C4_IID_LOSS_RATE


@dataclass
class GeometryRunResult:
    policy: str
    condition: str
    seed: int
    n_robots: int
    pose_rmse: float = float("nan")
    align_rmse: float = float("nan")
    trust_yield: float = float("nan")
    accept_rate: float = float("nan")
    airtime_util: float = float("nan")
    n_candidates: int = 0
    n_sent: int = 0
    n_delivered: int = 0
    bytes_sent: int = 0
    sched_overhead_ms: float = 0.0
    starvation: int = 0
    outlier_share: float = float("nan")
    gamma_final: float = float("nan")
    dt_pred_bias: float = float("nan")

    # Geometry and outage metrics
    obstruction_fraction: float = 0.0
    n_outages: int = 0
    mean_outage_s: float = 0.0
    max_outage_s: float = 0.0
    n_los_attempts: int = 0
    n_los_delivered: int = 0
    n_nlos_attempts: int = 0
    n_nlos_delivered: int = 0
    los_delivery_rate: float = float("nan")
    nlos_delivery_rate: float = float("nan")

    # Inlier/outlier filtering metrics
    n_accepted_inliers: int = 0
    n_rejected_inliers: int = 0
    n_accepted_outliers: int = 0
    n_rejected_outliers: int = 0

    extras: Dict[str, Any] = field(default_factory=dict)


def run_geometry_simulation(
    cfg: SimConfig,
    condition: str = "C4",
    precomputed=None,
    obstacles=DEFAULT_OBSTACLES,
    server_pos=DEFAULT_SERVER_POS,
    matched_loss_rate: float = MATCHED_C4_IID_LOSS_RATE,
) -> GeometryRunResult:
    """
    Run one full simulation under condition C4 (Geometry-dependent) or C4-IID (Matched IID).
    """
    rng = np.random.default_rng(cfg.seed)
    w = cfg.world

    use_obs = (cfg.scheduler.use_observability or cfg.scheduler.policy == "bacs_plus")
    pair_obs = PairObservability(cfg.infogain.obs_ref)

    if precomputed is None:
        truths = precompute(cfg)[0]
        cands = precompute(cfg)[1]
    else:
        truths, cands = precomputed
        cands = {k: [_clone(c) for c in v] for k, v in cands.items()}

    agents = {r: make_agent(r, cfg.agent(r), rng) for r in range(w.n_robots)}
    for rid, lst in cands.items():
        for c in lst:
            agents[rid].on_report(c)

    gamma_ctl = GammaController(cfg.trust, w)

    # Initialize physical channel
    if condition == "C4":
        channel = GeometryLoRaChannel(
            cfg.channel, cfg.lora, rng, truths=truths,
            obstacles=obstacles, server_pos=server_pos, dt=w.dt
        )
    elif condition == "C4-IID":
        channel = MatchedIIDChannel(
            cfg.channel, cfg.lora, rng, loss_rate=matched_loss_rate
        )
    elif condition == "C0":
        # Unobstructed / baseline channel (no loss)
        channel = MatchedIIDChannel(
            cfg.channel, cfg.lora, rng, loss_rate=0.0
        )
    else:
        raise ValueError(f"Unknown condition: {condition}")

    coverage = {r: CoverageMap(cfg.infogain) for r in range(w.n_robots)}
    budget_per_window = airtime_budget(cfg.scheduler.window_s, cfg.lora, w.n_robots)
    n_windows = int(np.ceil(w.session_s / cfg.scheduler.window_s))

    pending: Dict[int, List] = {r: [] for r in range(w.n_robots)}
    delivered: List = []
    airtime_used = 0.0
    airtime_avail = 0.0
    bytes_sent = 0
    n_sent = 0
    sched_time = 0.0
    max_deferral = 0
    dt_errors = []

    degree = {r: np.zeros(len(truths[r].gt), int) for r in range(w.n_robots)}
    last_map_time = 0.0

    om = _odometry_information(w)
    graph = PoseGraph()
    for rid in range(w.n_robots):
        odo = truths[rid].odom
        for k in range(len(odo)):
            graph.add_node((rid, k), odo[k])
        for k in range(1, len(odo)):
            graph.add_edge(
                graph.idx((rid, k - 1)), graph.idx((rid, k)),
                relative(odo[k - 1], odo[k]), om, 1.0
            )

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

            # Schedule selection
            if cfg.scheduler.policy in ("bacs_gated", "bacs_plus"):
                ts = time.perf_counter()
                candidates = pending[rid]
                features = []
                for i, c in enumerate(candidates):
                    pred = relative(fused[rid][c.idx_from], fused[c.rid_to][c.idx_to])
                    span = abs(truths[rid].t[c.idx_from] - truths[c.rid_to].t[c.idx_to])
                    features.append(
                        Candidate(
                            i, c.t_created, float(np.linalg.norm((c.z - pred)[:2])),
                            coverage[rid].novelty(truths[rid].odom[c.idx_from][:2]),
                            min(degree[rid][c.idx_from] / 8, 1), min(span / 60, 1),
                            rid, c.rid_to, pair_obs.count(rid, c.rid_to),
                            c.payload_bytes, c.deferrals
                        )
                    )
                picked, retained = select_window(
                    features, now_s=t1, budget_s=budget_per_window,
                    window_s=cfg.scheduler.window_s, gamma=gamma, freshness_rate=gamma,
                    trust_cfg=cfg.trust, info_cfg=cfg.infogain, lora=cfg.lora,
                    bacs_plus=use_obs, trust_gate=cfg.scheduler.trust_gate,
                    loss_probability=gamma_ctl.p_loss,
                    map_age_s=map_age, robot_speed=w.speed,
                    use_confidence_blend=True,
                )
                chosen = [candidates[f.sequence] for f in picked]
                for c, f in zip(chosen, picked):
                    c.dt_hat, c.theta_hat = f.predicted_age_s, f.predicted_trust
                pending[rid] = chosen + [candidates[f.sequence] for f in retained]
                sched_time += time.perf_counter() - ts
            else:
                ts = time.perf_counter()
                chosen = schedule(pending[rid], budget_per_window, cfg.scheduler, cfg.lora, rng)
                sched_time += time.perf_counter() - ts

            airtime_avail += budget_per_window
            chosen_ids = set(id(c) for c in chosen)

            t_cursor = t1
            for c in chosen:
                c.t_sent = max(t_cursor, c.t_created)
                if isinstance(channel, GeometryLoRaChannel):
                    ok, t_recv, used, attempts = channel.transmit(
                        c.payload_bytes, c.t_sent, rid=c.rid_from
                    )
                else:
                    ok, t_recv, used, attempts = channel.transmit(
                        c.payload_bytes, c.t_sent
                    )

                t_cursor = t_recv if ok else t_cursor + used
                airtime_used += used
                bytes_sent += c.payload_bytes * attempts
                n_sent += 1
                c.attempts = attempts
                c.delivered = ok
                c.t_recv = t_recv
                dt_actual = t_recv - c.t_created
                gamma_ctl.observe(dt_actual, not ok)

                if ok:
                    dt_errors.append(c.dt_hat - dt_actual)
                    delivered.append(c)
                    coverage[rid].mark(truths[rid].odom[c.idx_from][:2])
                    degree[rid][c.idx_from] += 1
                    pair_obs.mark(c.rid_from, c.rid_to)

            rest = []
            for c in pending[rid]:
                if id(c) in chosen_ids:
                    continue
                c.deferrals += 1
                max_deferral = max(max_deferral, c.deferrals)
                if c.deferrals < 12:
                    rest.append(c)
            pending[rid] = rest

        # Ingest arrivals and optimize server pose graph
        new_edges = [c for c in delivered if not getattr(c, "_ingested", False)]
        for c in new_edges:
            c._ingested = True
            dt = c.t_recv - c.t_created
            pred = relative(fused[c.rid_from][c.idx_from], fused[c.rid_to][c.idx_to])
            c.theta = server_trust(
                float(np.linalg.norm((c.z - pred)[:2])),
                dt, cfg.trust, gamma_ctl.value
            )
            graph.add_edge(
                graph.idx((c.rid_from, c.idx_from)),
                graph.idx((c.rid_to, c.idx_to)),
                c.z, om, c.theta
            )

        if new_edges and (wi % max(w.fusion_every_windows, 1) == 0):
            Xk, _ = graph.optimize(iterations=6)
            for rid in range(w.n_robots):
                fused[rid] = np.array([Xk[graph.idx((rid, k))] for k in range(len(truths[rid].gt))])
            last_map_time = t1

    # Final global optimization
    thetas = [c.theta for c in delivered]
    X, H_final = graph.optimize()

    # Trajectory ATE RMSE
    errs = []
    for rid in range(w.n_robots):
        gt = truths[rid].gt
        est = np.array([X[graph.idx((rid, k))] for k in range(len(gt))])
        est_a = _align_first(est, gt)
        errs.append(np.linalg.norm(est_a[:, :2] - gt[:, :2], axis=1))
    pose_rmse = float(np.sqrt(np.mean(np.concatenate(errs) ** 2)))

    # Map alignment RMSE on ground-truth co-location pairs
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

    n_cand = sum(len(v) for v in cands.values())

    # Trajectory-wide outage stats across all robots
    all_blocked_states = []
    for rid in range(w.n_robots):
        pts = truths[rid].gt[:, :2]
        b_series = np.array([check_link_blocked(tuple(p), server_pos, obstacles) for p in pts])
        all_blocked_states.append(b_series)
    combined_blocked = np.concatenate(all_blocked_states)
    outage_stats = compute_outage_statistics(combined_blocked, w.dt)

    # Inlier/outlier filtering breakdown
    accepted = [c for c in delivered if c.theta > cfg.trust.accept_threshold]
    transmitted_outliers = [c for c in delivered if c.is_outlier]
    rejected_candidates = [
        c for lst in cands.values() for c in lst
        if c.attempts == 0 or (c.delivered and c.theta <= cfg.trust.accept_threshold)
    ]
    n_acc_inliers = sum(1 for c in accepted if not c.is_outlier)
    n_rej_inliers = sum(1 for c in rejected_candidates if not c.is_outlier)
    n_acc_outliers = sum(1 for c in accepted if c.is_outlier)
    n_rej_outliers = sum(1 for c in rejected_candidates if c.is_outlier)

    # Channel stats
    if isinstance(channel, GeometryLoRaChannel):
        n_los_att = channel.n_los_attempts
        n_los_del = channel.n_los_delivered
        n_nlos_att = channel.n_nlos_attempts
        n_nlos_del = channel.n_nlos_delivered
        los_del_rate = float(n_los_del / n_los_att) if n_los_att > 0 else float("nan")
        nlos_del_rate = float(n_nlos_del / n_nlos_att) if n_nlos_att > 0 else float("nan")
    else:
        n_los_att = channel.n_attempts_total
        n_los_del = channel.n_delivered
        n_nlos_att = 0
        n_nlos_del = 0
        los_del_rate = float(n_los_del / n_los_att) if n_los_att > 0 else float("nan")
        nlos_del_rate = float("nan")

    return GeometryRunResult(
        policy=cfg.scheduler.policy,
        condition=condition,
        seed=cfg.seed,
        n_robots=w.n_robots,
        pose_rmse=pose_rmse,
        align_rmse=align_rmse,
        trust_yield=float(np.mean(thetas)) if thetas else 0.0,
        accept_rate=float(np.mean([t > cfg.trust.accept_threshold for t in thetas])) if thetas else 0.0,
        airtime_util=float(airtime_used / airtime_avail) if airtime_avail > 0 else 0.0,
        n_candidates=n_cand,
        n_sent=n_sent,
        n_delivered=len(delivered),
        bytes_sent=bytes_sent,
        sched_overhead_ms=float(1000.0 * sched_time / max(n_windows, 1)),
        starvation=int(max_deferral),
        outlier_share=float(np.mean([c.is_outlier for c in delivered])) if delivered else 0.0,
        gamma_final=float(gamma_ctl.value),
        dt_pred_bias=float(np.mean(dt_errors)) if dt_errors else float("nan"),
        obstruction_fraction=outage_stats["obstruction_fraction"],
        n_outages=outage_stats["n_outages"],
        mean_outage_s=outage_stats["mean_outage_s"],
        max_outage_s=outage_stats["max_outage_s"],
        n_los_attempts=n_los_att,
        n_los_delivered=n_los_del,
        n_nlos_attempts=n_nlos_att,
        n_nlos_delivered=n_nlos_del,
        los_delivery_rate=los_del_rate,
        nlos_delivery_rate=nlos_del_rate,
        n_accepted_inliers=n_acc_inliers,
        n_rejected_inliers=n_rej_inliers,
        n_accepted_outliers=n_acc_outliers,
        n_rejected_outliers=n_rej_outliers,
    )
