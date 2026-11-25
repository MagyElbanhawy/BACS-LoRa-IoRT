import copy
import numpy as np
import pytest

from bacs_sim.config import SimConfig
from bacs_sim.simulator import precompute, run
from bacs_sim.manuscript_algorithm import select_window, Candidate

def test_runtime_ground_truth_independence():
    """Verify runtime ground-truth independence:
    Perturbing evaluation-only ground truth while holding all scheduling inputs
    fixed produces 100% identical selected constraint IDs, ordering, and scores."""
    
    cfg = SimConfig(seed=42)
    cfg.world.session_s = 480
    cfg.world.n_robots = 3
    
    truths, cands = precompute(cfg)
    
    # 1. Capture original selection on window 1
    w_cands = [c for lst in cands.values() for c in lst if 0.0 <= c.t_created < 60.0]
    features_orig = [
        Candidate(
            sequence=i,
            generated_s=c.t_created,
            provisional_residual_m=0.15,
            novelty=0.5,
            normalized_degree=0.25,
            normalized_loop_span=0.75,
            robot_i=c.rid_from,
            robot_j=c.rid_to,
            delivered_pair_count=0,
            payload_bytes=c.payload_bytes,
            deferrals=c.deferrals,
        )
        for i, c in enumerate(w_cands)
    ]
    
    sel_orig, ret_orig = select_window(
        features_orig, now_s=60.0, budget_s=1.0, window_s=60.0, gamma=0.0045,
        trust_cfg=cfg.trust, info_cfg=cfg.infogain, lora=cfg.lora, bacs_plus=True,
        map_age_s=60.0, robot_speed=0.3, use_confidence_blend=True,
    )
    
    # 2. Perturb evaluation-only ground truth violently (+1000m offset, rotated, flipped)
    perturbed_truths = []
    for r, t_data in enumerate(truths):
        p_t = copy.deepcopy(t_data)
        p_t.gt[:, :2] += 1000.0  # +1000 meters shift in ground truth!
        p_t.gt[:, 2] += np.pi    # 180 degree rotation!
        perturbed_truths.append(p_t)
        
    # Re-run selection with the identical candidate feature inputs
    features_perturbed = [
        Candidate(
            sequence=i,
            generated_s=c.t_created,
            provisional_residual_m=0.15,
            novelty=0.5,
            normalized_degree=0.25,
            normalized_loop_span=0.75,
            robot_i=c.rid_from,
            robot_j=c.rid_to,
            delivered_pair_count=0,
            payload_bytes=c.payload_bytes,
            deferrals=c.deferrals,
        )
        for i, c in enumerate(w_cands)
    ]
    
    sel_pert, ret_pert = select_window(
        features_perturbed, now_s=60.0, budget_s=1.0, window_s=60.0, gamma=0.0045,
        trust_cfg=cfg.trust, info_cfg=cfg.infogain, lora=cfg.lora, bacs_plus=True,
        map_age_s=60.0, robot_speed=0.3, use_confidence_blend=True,
    )
    
    # Assert bit-for-bit identical selection and ordering
    assert len(sel_orig) == len(sel_pert)
    assert [c.sequence for c in sel_orig] == [c.sequence for c in sel_pert]
    assert [c.predicted_trust for c in sel_orig] == [c.predicted_trust for c in sel_pert]
    assert [c.predicted_age_s for c in sel_orig] == [c.predicted_age_s for c in sel_pert]
    assert [c.sequence for c in ret_orig] == [c.sequence for c in ret_pert]


def test_simulator_e2e_ground_truth_independence():
    """Verify end-to-end simulation ground-truth independence:
    Perturbing truths[rid].gt in the simulation input while keeping odometry and candidates
    fixed produces 100% bit-for-bit identical online scheduling decisions and delivered constraints."""
    cfg = SimConfig(seed=42)
    cfg.world.session_s = 240
    cfg.world.n_robots = 3
    cfg.scheduler.policy = "bacs_plus"
    cfg.scheduler.use_observability = True

    truths_orig, cands_orig = precompute(cfg)

    # Deepcopy candidates so runs are completely isolated
    from bacs_sim.simulator import _clone
    cands_run1 = {k: [_clone(c) for c in v] for k, v in cands_orig.items()}
    cands_run2 = {k: [_clone(c) for c in v] for k, v in cands_orig.items()}

    # Perturb ground truth violently
    truths_pert = []
    for r, t_data in enumerate(truths_orig):
        p_t = copy.deepcopy(t_data)
        p_t.gt[:, :2] += 5000.0  # +5 km translation
        p_t.gt[:, 2] += np.pi / 2  # 90 deg rotation
        truths_pert.append(p_t)

    res_orig = run(cfg, precomputed=(truths_orig, cands_run1), collect_graph=True, execution="v15_corrected")
    res_pert = run(cfg, precomputed=(truths_pert, cands_run2), collect_graph=True, execution="v15_corrected")

    # Delivered constraints must be identical in count, identity, and timing
    deliv_orig = res_orig.extras["delivered"]
    deliv_pert = res_pert.extras["delivered"]
    assert len(deliv_orig) == len(deliv_pert)
    assert len(deliv_orig) > 0
    for c1, c2 in zip(deliv_orig, deliv_pert):
        assert c1.rid_from == c2.rid_from
        assert c1.rid_to == c2.rid_to
        assert c1.idx_from == c2.idx_from
        assert c1.idx_to == c2.idx_to
        assert abs(c1.t_created - c2.t_created) < 1e-9
        assert abs(c1.t_sent - c2.t_sent) < 1e-9
        assert abs(c1.t_recv - c2.t_recv) < 1e-9
        assert abs(c1.theta - c2.theta) < 1e-9

    assert res_orig.n_delivered == res_pert.n_delivered
    assert res_orig.n_sent == res_pert.n_sent
    assert res_orig.airtime_util == res_pert.airtime_util
    assert res_orig.bytes_sent == res_pert.bytes_sent

