from math import exp, floor

import pytest

from bacs_sim.config import InfoGainConfig, LoRaConfig, TrustConfig
from bacs_sim.lora import time_on_air
from bacs_sim.manuscript_algorithm import (
    Candidate,
    arrival_age_s,
    information_surrogate,
    pairwise_score,
    predicted_trust,
    select_window,
)


def candidate(sequence, **overrides):
    values = dict(
        sequence=sequence,
        generated_s=0.0,
        provisional_residual_m=0.0,
        novelty=0.5,
        normalized_degree=0.5,
        normalized_loop_span=0.5,
        robot_i=0,
        robot_j=1,
    )
    values.update(overrides)
    return Candidate(**values)


def test_arrival_age_includes_accumulated_age_queue_and_retry():
    age = arrival_age_s(
        now_s=100.0, generated_s=40.0, airtime_ahead_s=0.2,
        own_airtime_s=0.1, budget_s=0.6, window_s=60.0,
        loss_probability=0.2, ack_timeout_s=0.5,
    )
    assert age == pytest.approx(60.2 + 0.1 + 0.2 * 0.6 / 0.8)


def test_surrogate_weights_and_pairwise_term_match_equations():
    cfg = InfoGainConfig()
    c = candidate(1, novelty=1.0, normalized_degree=0.0, normalized_loop_span=1.0)
    assert information_surrogate(c, cfg) == pytest.approx(1.0)
    assert pairwise_score(0, 6) == pytest.approx(1.0)
    assert pairwise_score(6, 6) == pytest.approx(exp(-1))


def test_predicted_trust_matches_spatial_temporal_product():
    cfg = TrustConfig(tau_e=0.5, p=3, floor=0.01)
    expected = (1 - (0.25 / 0.5) ** 3) * exp(-0.0045 * 155)
    assert predicted_trust(0.25, 155, cfg, 0.0045) == pytest.approx(expected)


def test_gate_relaxes_when_no_candidate_passes():
    lora, trust, info = LoRaConfig(), TrustConfig(), InfoGainConfig()
    one = time_on_air(52, lora)
    selected, _ = select_window(
        [candidate(2), candidate(1)], now_s=0.0, budget_s=one + 1e-9,
        window_s=60.0, gamma=30.0,
        trust_cfg=trust, info_cfg=info, lora=lora, trust_gate=0.05,
    )
    assert [c.sequence for c in selected] == [1]
    assert selected[0].predicted_trust < 0.05


def test_equal_size_budget_and_deterministic_tie_breaking():
    lora, trust, info = LoRaConfig(), TrustConfig(), InfoGainConfig()
    one = time_on_air(52, lora)
    selected, retained = select_window(
        [candidate(3), candidate(1), candidate(2)], now_s=0.0,
        budget_s=2 * one + 1e-9, window_s=60.0, gamma=0.0,
        trust_cfg=trust, info_cfg=info, lora=lora,
    )
    assert [c.sequence for c in selected] == [1, 2]
    assert [c.sequence for c in retained] == [3]
    assert retained[0].deferrals == 1


def test_bacs_plus_uses_delivered_pair_count_additively():
    lora, trust, info = LoRaConfig(), TrustConfig(), InfoGainConfig()
    one = time_on_air(52, lora)
    sparse = candidate(2, delivered_pair_count=0)
    served = candidate(1, delivered_pair_count=12)
    selected, _ = select_window(
        [served, sparse], now_s=0.0, budget_s=one,
        window_s=60.0, gamma=0.0, trust_cfg=trust, info_cfg=info,
        lora=lora, bacs_plus=True,
    )
    assert selected[0].sequence == 2


def test_expiration_removes_low_trust_and_twelve_window_candidates():
    lora, trust, info = LoRaConfig(), TrustConfig(), InfoGainConfig()
    one = time_on_air(52, lora)
    expired_age = candidate(1, provisional_residual_m=2.0)
    expired_count = candidate(2, deferrals=12)
    selected, retained = select_window(
        [expired_age, expired_count], now_s=0.0, budget_s=one,
        window_s=60.0, gamma=0.0, trust_cfg=trust, info_cfg=info,
        lora=lora,
    )
    assert selected == []
    assert retained == []


def test_manuscript_packet_airtime_is_52_bytes_with_crc():
    assert time_on_air(52, LoRaConfig(crc=True)) == pytest.approx(0.102656)


def test_v15_freshness_changes_equal_cost_selection():
    radio = LoRaConfig()
    common = dict(now_s=40, budget_s=time_on_air(52, radio)+1e-6,
                  window_s=60, gamma=0, trust_cfg=TrustConfig(),
                  info_cfg=InfoGainConfig(), lora=radio)
    items = [candidate(1, generated_s=0), candidate(2, generated_s=40)]
    assert select_window(items, **common)[0][0].sequence == 1
    assert select_window(items, freshness_rate=0.01, **common)[0][0].sequence == 2


def test_confidence_blending_allows_drifted_inlier_survival():
    """Verify that a valid loop closure with high initial residual caused by odometry
    drift survives admission under confidence blending, but is dropped under unblended trust."""
    lora, trust, info = LoRaConfig(), TrustConfig(tau_e=0.3, p=6, floor=0.01), InfoGainConfig()
    one = time_on_air(52, lora)
    
    # 0.824 m residual (observed mean drift residual) on a candidate
    drifted_cand = candidate(1, provisional_residual_m=0.824, novelty=0.8, normalized_loop_span=0.9)
    
    # Case A: Flawed unblended evaluation (use_confidence_blend=False)
    # Predicted trust falls to 0.0000 < 1.5 * floor (0.015), causing hard queue deletion
    sel_unblended, ret_unblended = select_window(
        [drifted_cand], now_s=60.0, budget_s=one, window_s=60.0, gamma=0.0045,
        trust_cfg=trust, info_cfg=info, lora=lora,
        map_age_s=60.0, robot_speed=0.3, use_confidence_blend=False,
    )
    assert sel_unblended == []
    assert ret_unblended == []  # Dropped from queue!

    # Case B: Corrected confidence-blended evaluation (use_confidence_blend=True, map_age_s=60.0s)
    # Map confidence c = 1 / (1 + 0.3*60/0.3) = 1/61 = 0.01639, blending trust towards 1.0 (approx 0.98)
    sel_blended, ret_blended = select_window(
        [drifted_cand], now_s=60.0, budget_s=one, window_s=60.0, gamma=0.0045,
        trust_cfg=trust, info_cfg=info, lora=lora,
        map_age_s=60.0, robot_speed=0.3, use_confidence_blend=True,
    )
    assert len(sel_blended) == 1
    assert sel_blended[0].sequence == 1
    assert sel_blended[0].predicted_trust > 0.50


def test_outlier_filtering_when_map_is_converged():
    """Verify that when the map has converged (map_age_s=0.0), spatial filtering
    is sharp and rejects gross outliers."""
    lora, trust, info = LoRaConfig(crc=True), TrustConfig(tau_e=0.3, p=6, floor=0.01), InfoGainConfig()
    one = time_on_air(52, lora)

    # When map is aligned (map_age_s = 0.0), a gross outlier (res = 2.0m) must be filtered
    gross_outlier = candidate(1, provisional_residual_m=2.0)
    sel, ret = select_window(
        [gross_outlier], now_s=60.0, budget_s=one, window_s=60.0, gamma=0.0045,
        trust_cfg=trust, info_cfg=info, lora=lora,
        map_age_s=0.0, robot_speed=0.3, use_confidence_blend=True,
    )
    assert sel == []
    assert ret == []


def test_bacs_plus_pairwise_observability_prioritizes_neglected_pairs():
    """Verify that BACS+ pairwise coverage term correctly prioritizes unserved robot pairs."""
    lora, trust, info = LoRaConfig(), TrustConfig(), InfoGainConfig(w_obs=0.30, obs_ref=6.0)
    one = time_on_air(52, lora)
    
    # Candidate 1: pair (0, 1) has 10 delivered constraints
    cand_served = candidate(1, robot_i=0, robot_j=1, delivered_pair_count=10, novelty=0.5, normalized_loop_span=0.5)
    # Candidate 2: pair (0, 2) has 0 delivered constraints (identical other features)
    cand_neglected = candidate(2, robot_i=0, robot_j=2, delivered_pair_count=0, novelty=0.5, normalized_loop_span=0.5)

    # Standard BACS (without observability): sequence 1 is picked by tie-break
    sel_bacs, _ = select_window(
        [cand_served, cand_neglected], now_s=60.0, budget_s=one, window_s=60.0, gamma=0.0,
        trust_cfg=trust, info_cfg=info, lora=lora, bacs_plus=False,
    )
    assert sel_bacs[0].sequence == 1

    # BACS+ (with observability): neglected pair receives +w_obs * exp(0) = +0.30, ranking first
    sel_bacs_plus, _ = select_window(
        [cand_served, cand_neglected], now_s=60.0, budget_s=one, window_s=60.0, gamma=0.0,
        trust_cfg=trust, info_cfg=info, lora=lora, bacs_plus=True,
    )
    assert sel_bacs_plus[0].sequence == 2


def corrected_arrival_age_s(
    *, now_s: float, generated_s: float, airtime_ahead_s: float,
    own_airtime_s: float, budget_s: float, window_s: float,
    loss_probability: float = 0.0, ack_timeout_s: float = 0.5,
    queue_airtimes: list[float] | None = None,
) -> float:
    """Corrected event-based arrival age predictor for Equation (5).

    Accurately accounts for accumulated age, discrete window boundaries,
    unused budget bypass, and retransmission retry expectation.
    """
    if budget_s <= 0 or window_s <= 0 or own_airtime_s < 0 or airtime_ahead_s < 0:
        raise ValueError("airtime, budget, and window values must be valid")
    loss = min(max(loss_probability, 0.0), 0.95)
    retry = loss * (own_airtime_s + ack_timeout_s) / (1.0 - loss)

    if queue_airtimes is not None:
        win = 0
        used = 0.0
        for a in queue_airtimes:
            if used + a <= budget_s + 1e-12:
                used += a
            else:
                win += 1
                used = a
        if used + own_airtime_s <= budget_s + 1e-12:
            send_win = win
            start_in_win = used
        else:
            send_win = win + 1
            start_in_win = 0.0
        future_delay = send_win * window_s + start_in_win + own_airtime_s + retry
    else:
        whole_windows = max(0, floor((airtime_ahead_s + own_airtime_s - 1e-12) / budget_s))
        within_send_window = max(0.0, airtime_ahead_s - whole_windows * budget_s)
        future_delay = whole_windows * window_s + within_send_window + own_airtime_s + retry

    return max(now_s - generated_s, 0.0) + future_delay


def test_arrival_age_exact_budget_boundary():
    """Verify the exact-budget boundary flaw in the legacy Eq. (5) formula,
    and validate the corrected event-based predictor."""
    # In legacy Eq. (5), budget is 0.60 s. Packet is 0.10 s. Ahead is 0.50 s.
    # floor((0.50 + 0.10) / 0.60) = floor(1.0) = 1, so legacy formula jumped to window 1 (80.0 s!)
    age_legacy = arrival_age_s(
        now_s=100.0, generated_s=80.0, airtime_ahead_s=0.50,
        own_airtime_s=0.10, budget_s=0.60, window_s=60.0,
    )
    assert age_legacy == pytest.approx(80.0)  # Demonstrates Defect 2 in legacy code

    # Corrected predictor: fits in window 0 at 0.60 s
    age_exact = corrected_arrival_age_s(
        now_s=100.0, generated_s=80.0, airtime_ahead_s=0.50,
        own_airtime_s=0.10, budget_s=0.60, window_s=60.0,
    )
    assert age_exact == pytest.approx(20.0 + 0.60)

    # Just below budget: 0.499999 + 0.10 = 0.599999 <= 0.60 -> window 0
    age_below = corrected_arrival_age_s(
        now_s=100.0, generated_s=80.0, airtime_ahead_s=0.499999,
        own_airtime_s=0.10, budget_s=0.60, window_s=60.0,
    )
    assert age_below == pytest.approx(20.0 + 0.599999)

    # Just above budget: 0.500001 + 0.10 = 0.600001 > 0.60 -> spills into window 1
    age_above = corrected_arrival_age_s(
        now_s=100.0, generated_s=80.0, airtime_ahead_s=0.500001,
        own_airtime_s=0.10, budget_s=0.60, window_s=60.0,
    )
    assert age_above >= 80.0

    # Event-based queue simulation
    q_airtimes = [0.10] * 5  # 5 preceding packets of 0.10 s (total 0.50 s)
    age_event = corrected_arrival_age_s(
        now_s=100.0, generated_s=80.0, airtime_ahead_s=0.50,
        own_airtime_s=0.10, budget_s=0.60, window_s=60.0,
        queue_airtimes=q_airtimes,
    )
    assert age_event == pytest.approx(20.0 + 0.60)


def test_n2_selection_rank_inversion_under_freshness_decay():
    """Verify that for N=2, adding the constant pairwise term C = w_o * O_12 can flip
    candidate rankings when candidates in the queue have different generation ages."""
    lora, trust, info = LoRaConfig(), TrustConfig(), InfoGainConfig(w_obs=0.30, obs_ref=6.0)
    one = time_on_air(52, lora)
    
    # Candidate 1: Older (generated at t=10.0), higher novelty (1.0 -> I1 = 0.90)
    cand_older_high_info = candidate(1, generated_s=10.0, novelty=1.0, normalized_degree=0.0, normalized_loop_span=2/3)
    # Candidate 2: Fresh (generated at t=50.0), moderate novelty (0.6 -> I2 = 0.70)
    cand_fresh_mod_info = candidate(2, generated_s=50.0, novelty=0.6, normalized_degree=0.0, normalized_loop_span=2/3)

    # Under BACS (pairwise bonus disabled, gamma=0.005 s^-1, now_s=60):
    # Candidate 1 has higher freshness-discounted score (0.7006 > 0.6655), so Candidate 1 is chosen!
    sel_bacs, _ = select_window(
        [cand_older_high_info, cand_fresh_mod_info], now_s=60.0, budget_s=one,
        window_s=60.0, gamma=0.005, freshness_rate=0.005, trust_cfg=trust, info_cfg=info,
        lora=lora, bacs_plus=False,
    )
    assert sel_bacs[0].sequence == 1

    # Under BACS+ (pairwise bonus C = 0.30 enabled):
    # The constant +0.30 bonus is multiplied by exp(-gamma*dt). Because Candidate 2 is fresher,
    # it receives a larger absolute freshness boost (0.30*exp(-0.005*10.1) vs 0.30*exp(-0.005*50.1)),
    # flipping the selection order from Candidate 1 to Candidate 2 (0.9507 > 0.9341)!
    sel_bacs_plus, _ = select_window(
        [cand_older_high_info, cand_fresh_mod_info], now_s=60.0, budget_s=one,
        window_s=60.0, gamma=0.005, freshness_rate=0.005, trust_cfg=trust, info_cfg=info,
        lora=lora, bacs_plus=True,
    )
    assert sel_bacs_plus[0].sequence == 2


