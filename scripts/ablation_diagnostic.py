import sys
import copy
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from bacs_sim.config import SimConfig
from bacs_sim.simulator import run, precompute, _odometry_information, _clone, _align_first
from bacs_sim.lora import Channel, time_on_air, airtime_budget
from bacs_sim.trust import GammaController, predicted_delay, predicted_trust, server_trust
from bacs_sim.infogain import CoverageMap, surrogate_info
from bacs_sim.observability import PairObservability
from bacs_sim.posegraph import PoseGraph
from bacs_sim.agents import make_agent
from bacs_sim.world import relative
from bacs_sim.manuscript_algorithm import Candidate, select_window

def run_custom(cfg, precomputed, *, 
               freshness_rate=None, # None means use gamma
               t_cursor_mode="t1", # "t0" or "t1"
               use_confidence_blend=False,
               relax_floor_drop=False):
    rng = np.random.default_rng(cfg.seed)
    w = cfg.world
    use_obs = (cfg.scheduler.use_observability or cfg.scheduler.policy == "bacs_plus")
    pair_obs = PairObservability(cfg.infogain.obs_ref)

    truths, cands = precomputed
    cands = {k: [_clone(c) for c in v] for k, v in cands.items()}

    agents = {r: make_agent(r, cfg.agent(r), rng) for r in range(w.n_robots)}
    for rid, lst in cands.items():
        for c in lst:
            agents[rid].on_report(c)

    gamma_ctl = GammaController(cfg.trust, w)
    channel = Channel(cfg.channel, cfg.lora, rng)
    coverage = {r: CoverageMap(cfg.infogain) for r in range(w.n_robots)}

    budget_per_window = airtime_budget(cfg.scheduler.window_s, cfg.lora, w.n_robots)
    n_windows = int(np.ceil(w.session_s / cfg.scheduler.window_s))

    pending = {r: [] for r in range(w.n_robots)}
    delivered = []
    degree = {r: np.zeros(len(truths[r].gt), int) for r in range(w.n_robots)}
    last_map_time = 0.0

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
            features = []
            for i, c in enumerate(candidates):
                pred = relative(fused[rid][c.idx_from], fused[c.rid_to][c.idx_to])
                res = float(np.linalg.norm((c.z - pred)[:2]))
                span = abs(truths[rid].t[c.idx_from] - truths[c.rid_to].t[c.idx_to])
                
                # Confidence blend test:
                if use_confidence_blend:
                    conf = 1.0 / (1.0 + (w.speed * max(map_age, 0.0)) / cfg.trust.tau_e)
                    spatial = max(0.0, 1.0 - (res / cfg.trust.tau_e) ** cfg.trust.p)
                    spatial_blended = conf * spatial + (1.0 - conf) * 1.0
                    # compute provisional residual that would give spatial_blended:
                    # Or pass directly:
                    # If spatial_blended > 0, res_eff = tau_e * (1 - spatial_blended)**(1/p)
                    res_eff = cfg.trust.tau_e * max(0.0, (1.0 - spatial_blended)) ** (1.0 / cfg.trust.p)
                else:
                    res_eff = res

                features.append(Candidate(
                    i, c.t_created, res_eff,
                    coverage[rid].novelty(truths[rid].odom[c.idx_from][:2]),
                    min(degree[rid][c.idx_from] / 8, 1), min(span / 60, 1),
                    rid, c.rid_to, pair_obs.count(rid, c.rid_to),
                    c.payload_bytes, c.deferrals))

            f_rate = gamma if freshness_rate is None else freshness_rate

            # If relax_floor_drop is True, temporarily set floor to 0 so live doesn't drop
            t_cfg = copy.copy(cfg.trust)
            if relax_floor_drop:
                t_cfg.floor = 0.0

            picked, retained = select_window(
                features, now_s=t1, budget_s=budget_per_window,
                window_s=cfg.scheduler.window_s, gamma=gamma, freshness_rate=f_rate,
                trust_cfg=t_cfg, info_cfg=cfg.infogain, lora=cfg.lora,
                bacs_plus=use_obs, trust_gate=cfg.scheduler.trust_gate,
                loss_probability=gamma_ctl.p_loss)

            chosen = [candidates[f.sequence] for f in picked]
            for c, f in zip(chosen, picked):
                c.dt_hat, c.theta_hat = f.predicted_age_s, f.predicted_trust
            pending[rid] = chosen + [candidates[f.sequence] for f in retained]

            chosen_ids = set(id(c) for c in chosen)
            t_cursor = t0 if t_cursor_mode == "t0" else t1

            for c in chosen:
                c.t_sent = max(t_cursor, c.t_created)
                ok, t_recv, used, attempts = channel.transmit(c.payload_bytes, c.t_sent)
                t_cursor = t_recv if ok else t_cursor + used
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

    # Map alignment:
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
    return align_rmse, len(delivered), sum(c.is_outlier for c in delivered)


def main():
    cfg = SimConfig(seed=0)
    cfg.world.session_s = 480
    cfg.world.n_robots = 3
    cfg.scheduler.policy = "bacs_plus"
    cfg.scheduler.use_observability = True
    cfg.trust.gamma = np.log(2) / 155.0
    pre = precompute(cfg)

    print("Running factor ablations on Seed 0 (N=3, C0):")
    
    # 0. Legacy
    res_leg = run(cfg, precomputed=pre, execution="legacy")
    print(f"0. Legacy baseline: align_rmse={res_leg.align_rmse:.4f} m, deliv={res_leg.n_delivered}")

    # 1. v15 current
    res_v15 = run(cfg, precomputed=pre, execution="v15")
    print(f"1. v15 current:     align_rmse={res_v15.align_rmse:.4f} m, deliv={res_v15.n_delivered}")

    # 2. v15 with freshness_rate = 0.0 (Pure information density, no freshness weighting)
    a, d, o = run_custom(cfg, pre, freshness_rate=0.0, t_cursor_mode="t1", use_confidence_blend=False, relax_floor_drop=False)
    print(f"2. v15, freshness=0.0: align_rmse={a:.4f} m, deliv={d}")

    # 3. v15 with t_cursor = "t0" (start of window)
    a, d, o = run_custom(cfg, pre, freshness_rate=None, t_cursor_mode="t0", use_confidence_blend=False, relax_floor_drop=False)
    print(f"3. v15, t_cursor=t0:   align_rmse={a:.4f} m, deliv={d}")

    # 4. v15 with confidence blend
    a, d, o = run_custom(cfg, pre, freshness_rate=None, t_cursor_mode="t1", use_confidence_blend=True, relax_floor_drop=False)
    print(f"4. v15, conf_blend:   align_rmse={a:.4f} m, deliv={d}")

    # 5. v15 with relax_floor_drop
    a, d, o = run_custom(cfg, pre, freshness_rate=None, t_cursor_mode="t1", use_confidence_blend=False, relax_floor_drop=True)
    print(f"5. v15, no_floor_drop:align_rmse={a:.4f} m, deliv={d}")

    # 6. Combinations:
    # 6a. freshness=0.0 + t_cursor=t0
    a, d, o = run_custom(cfg, pre, freshness_rate=0.0, t_cursor_mode="t0", use_confidence_blend=False, relax_floor_drop=False)
    print(f"6a. freshness=0 + t0: align_rmse={a:.4f} m, deliv={d}")

    # 6b. freshness=0.0 + conf_blend
    a, d, o = run_custom(cfg, pre, freshness_rate=0.0, t_cursor_mode="t1", use_confidence_blend=True, relax_floor_drop=False)
    print(f"6b. freshness=0 + conf_blend: align_rmse={a:.4f} m, deliv={d}")

    # 6c. freshness=0.0 + conf_blend + t0
    a, d, o = run_custom(cfg, pre, freshness_rate=0.0, t_cursor_mode="t0", use_confidence_blend=True, relax_floor_drop=False)
    print(f"6c. freshness=0 + conf_blend + t0: align_rmse={a:.4f} m, deliv={d}")

    # 6d. freshness=0.0 + relax_floor_drop
    a, d, o = run_custom(cfg, pre, freshness_rate=0.0, t_cursor_mode="t1", use_confidence_blend=False, relax_floor_drop=True)
    print(f"6d. freshness=0 + no_floor_drop: align_rmse={a:.4f} m, deliv={d}")

    # 6e. freshness=0.0 + relax_floor_drop + t0
    a, d, o = run_custom(cfg, pre, freshness_rate=0.0, t_cursor_mode="t0", use_confidence_blend=False, relax_floor_drop=True)
    print(f"6e. freshness=0 + no_floor_drop + t0: align_rmse={a:.4f} m, deliv={d}")

if __name__ == "__main__":
    main()
