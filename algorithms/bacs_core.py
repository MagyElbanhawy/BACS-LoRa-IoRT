"""BACS Core Algorithm Reference Implementation.

Contains the standalone mathematical reference for:
- BACS (Bandwidth-Aware Constraint Scheduling)
- BACS+ (Adaptive Geometric Gating and Spatial Diversity)
- Baseline Schedulers: FIFO, LIFO, Uniform Random

Equations referenced correspond directly to the submitted manuscript:
"BACS: Bandwidth-Aware Constraint Scheduling for Scalable Multi-Robot SLAM
over Constrained LoRa Networks", Journal of King Saud University - CIS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple, Optional


class SchedulingPolicy(str, Enum):
    BACS = "BACS"
    BACS_PLUS = "BACS_PLUS"
    FIFO = "FIFO"
    LIFO = "LIFO"
    RANDOM = "RANDOM"


@dataclass
class LoRaPhyConfig:
    """LoRa physical layer parameters for airtime calculation."""
    spreading_factor: int = 7       # SF7 to SF12
    bandwidth_hz: float = 125000.0   # 125 kHz
    coding_rate: int = 1            # 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
    preamble_symbols: int = 8
    header_explicit: bool = True
    low_data_rate_optimize: bool = False
    crc_enabled: bool = True
    ack_timeout_s: float = 0.5


def calculate_lora_airtime(payload_bytes: int, cfg: LoRaPhyConfig) -> float:
    """Calculate LoRa packet airtime (in seconds) according to Semtech SX1262 specifications.
    
    Formula:
        T_sym = 2^SF / BW
        T_preamble = (N_preamble + 4.25) * T_sym
        N_payload = 8 + max( ceil((8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE))) * (CR + 4), 0 )
        T_payload = N_payload * T_sym
        T_packet = T_preamble + T_payload
    """
    sf = cfg.spreading_factor
    bw = cfg.bandwidth_hz
    t_sym = (2.0 ** sf) / bw

    t_preamble = (cfg.preamble_symbols + 4.25) * t_sym

    ih = 0 if cfg.header_explicit else 1
    crc = 1 if cfg.crc_enabled else 0
    de = 1 if cfg.low_data_rate_optimize else 0
    cr = cfg.coding_rate

    numerator = 8 * payload_bytes - 4 * sf + 28 + 16 * crc - 20 * ih
    denominator = 4 * (sf - 2 * de)

    bits_ceil = math.ceil(max(numerator, 0) / float(denominator))
    n_payload = 8 + max(bits_ceil * (cr + 4), 0)

    t_payload = n_payload * t_sym
    return t_preamble + t_payload


@dataclass
class ConstraintCandidate:
    """Represents an inter-robot loop closure constraint candidate."""
    sequence_id: int
    robot_i: int
    robot_j: int
    timestamp_gen_s: float
    residual_m: float
    novelty: float = 1.0
    normalized_degree: float = 0.0
    normalized_loop_span: float = 1.0
    delivered_pair_count: int = 0
    payload_bytes: int = 52
    deferral_count: int = 0

    # Computed attributes
    predicted_arrival_age_s: float = 0.0
    predicted_trust: float = 0.0
    information_score: float = 0.0
    priority_score: float = 0.0


@dataclass
class SchedulerConfig:
    """Configuration hyperparameters for BACS/BACS+."""
    window_duration_s: float = 1.0
    airtime_budget_s: float = 0.100  # 100 ms per 1 s window (10% local, or 1% regulatory)
    max_queue_capacity: int = 50
    max_deferrals: int = 12
    aoi_decay_rate: float = 0.05     # Lambda for exponential age discount
    trust_gate_threshold: float = 0.05
    trust_floor: float = 0.02
    tau_error: float = 0.40          # Spatial tolerance parameter (meters)
    error_power: float = 2.0         # Spatial penalty exponent
    w_novelty: float = 0.50
    w_degree: float = 0.25
    w_loop_span: float = 0.25
    w_pairwise_obs: float = 0.30     # BACS+ pairwise coverage weight
    pairwise_ref_count: float = 6.0


def compute_information_surrogate(candidate: ConstraintCandidate, cfg: SchedulerConfig, is_bacs_plus: bool) -> float:
    """Equation (7) & (8): Topological and observational surrogate metric."""
    info = (
        cfg.w_novelty * candidate.novelty
        + cfg.w_degree * (1.0 - candidate.normalized_degree)
        + cfg.w_loop_span * candidate.normalized_loop_span
    )
    if is_bacs_plus:
        coverage = math.exp(-max(candidate.delivered_pair_count, 0) / cfg.pairwise_ref_count)
        info += cfg.w_pairwise_obs * coverage
    return info


def predict_arrival_age(
    now_s: float,
    generated_s: float,
    airtime_ahead_s: float,
    own_airtime_s: float,
    budget_s: float,
    window_s: float,
    loss_prob: float = 0.0,
    ack_timeout_s: float = 0.5,
) -> float:
    """Equation (5): Predict arrival age including queueing, transmission, and expected retries."""
    p_loss = min(max(loss_prob, 0.0), 0.95)
    expected_retries_delay = p_loss * (own_airtime_s + ack_timeout_s) / (1.0 - p_loss) if p_loss > 0 else 0.0
    whole_windows = math.floor((airtime_ahead_s + own_airtime_s) / budget_s)
    intra_window = airtime_ahead_s - (whole_windows * budget_s)
    
    return (
        max(now_s - generated_s, 0.0)
        + whole_windows * window_s
        + intra_window
        + own_airtime_s
        + expected_retries_delay
    )


def predict_trust_metric(
    residual_m: float,
    age_s: float,
    cfg: SchedulerConfig,
    gamma: float = 0.05,
) -> float:
    """Equation (6): Spatial alignment trust combined with temporal decay."""
    spatial_trust = max(0.0, 1.0 - (max(residual_m, 0.0) / cfg.tau_error) ** cfg.error_power)
    temporal_decay = math.exp(-gamma * max(age_s, 0.0))
    return spatial_trust * temporal_decay


def schedule_epoch(
    candidates: List[ConstraintCandidate],
    policy: SchedulingPolicy,
    now_s: float,
    sched_cfg: SchedulerConfig,
    phy_cfg: LoRaPhyConfig,
    channel_loss_prob: float = 0.0,
) -> Tuple[List[ConstraintCandidate], List[ConstraintCandidate]]:
    """Executes a single scheduling epoch under the specified policy.
    
    Returns:
        (admitted_for_transmission, retained_in_queue)
    """
    if not candidates:
        return [], []

    # Filter out candidates exceeding maximum deferrals
    active_pool = [c for c in candidates if c.deferral_count < sched_cfg.max_deferrals]
    if not active_pool:
        return [], []

    if policy == SchedulingPolicy.FIFO:
        # Standard FIFO: arrival/generation order
        sorted_pool = sorted(active_pool, key=lambda c: (c.timestamp_gen_s, c.sequence_id))
    elif policy == SchedulingPolicy.LIFO:
        # LIFO: newest first
        sorted_pool = sorted(active_pool, key=lambda c: (-c.timestamp_gen_s, c.sequence_id))
    elif policy == SchedulingPolicy.RANDOM:
        # Deterministic sequence hash for reproducible uniform random sampling
        sorted_pool = sorted(active_pool, key=lambda c: (hash((c.sequence_id, int(now_s))), c.sequence_id))
    elif policy in (SchedulingPolicy.BACS, SchedulingPolicy.BACS_PLUS):
        is_plus = (policy == SchedulingPolicy.BACS_PLUS)

        # Pass 1: Provisional placement at middle of budget
        provisional_ahead = sched_cfg.airtime_budget_s * 0.5
        for c in active_pool:
            own_airtime = calculate_lora_airtime(c.payload_bytes, phy_cfg)
            c.predicted_arrival_age_s = predict_arrival_age(
                now_s=now_s,
                generated_s=c.timestamp_gen_s,
                airtime_ahead_s=provisional_ahead,
                own_airtime_s=own_airtime,
                budget_s=sched_cfg.airtime_budget_s,
                window_s=sched_cfg.window_duration_s,
                loss_prob=channel_loss_prob,
                ack_timeout_s=phy_cfg.ack_timeout_s,
            )
            c.information_score = compute_information_surrogate(c, sched_cfg, is_plus)
            c.predicted_trust = predict_trust_metric(c.residual_m, c.predicted_arrival_age_s, sched_cfg)
            
            density = c.information_score * math.exp(-sched_cfg.aoi_decay_rate * c.predicted_arrival_age_s)
            c.priority_score = density / own_airtime

        pass1_sorted = sorted(active_pool, key=lambda c: (-c.priority_score, c.sequence_id))

        # Pass 2: Exact arrival age based on first-pass schedule order
        current_ahead = 0.0
        for c in pass1_sorted:
            own_airtime = calculate_lora_airtime(c.payload_bytes, phy_cfg)
            c.predicted_arrival_age_s = predict_arrival_age(
                now_s=now_s,
                generated_s=c.timestamp_gen_s,
                airtime_ahead_s=current_ahead,
                own_airtime_s=own_airtime,
                budget_s=sched_cfg.airtime_budget_s,
                window_s=sched_cfg.window_duration_s,
                loss_prob=channel_loss_prob,
                ack_timeout_s=phy_cfg.ack_timeout_s,
            )
            c.predicted_trust = predict_trust_metric(c.residual_m, c.predicted_arrival_age_s, sched_cfg)
            density = c.information_score * math.exp(-sched_cfg.aoi_decay_rate * c.predicted_arrival_age_s)
            c.priority_score = density / own_airtime
            current_ahead += own_airtime

        # Trust gating
        gated_pool = [
            c for c in pass1_sorted
            if c.predicted_trust >= 1.5 * sched_cfg.trust_floor
        ]
        admitted = [c for c in gated_pool if c.predicted_trust >= sched_cfg.trust_gate_threshold]
        if not admitted:  # Relaxation if no candidate meets upper gate
            admitted = gated_pool

        sorted_pool = sorted(admitted, key=lambda c: (-c.priority_score, c.sequence_id))
    else:
        raise ValueError(f"Unknown scheduling policy: {policy}")

    # Greedy knapsack admission under airtime budget
    selected: List[ConstraintCandidate] = []
    used_airtime = 0.0

    for c in sorted_pool:
        airtime = calculate_lora_airtime(c.payload_bytes, phy_cfg)
        if used_airtime + airtime <= sched_cfg.airtime_budget_s + 1e-9:
            selected.append(c)
            used_airtime += airtime

    selected_ids = {c.sequence_id for c in selected}
    retained: List[ConstraintCandidate] = []
    for c in active_pool:
        if c.sequence_id not in selected_ids:
            c.deferral_count += 1
            if c.deferral_count < sched_cfg.max_deferrals:
                retained.append(c)

    return selected, retained


if __name__ == "__main__":
    print("Executing BACS core unit test...")
    phy = LoRaPhyConfig()
    cfg = SchedulerConfig(airtime_budget_s=0.150)

    airtime_52b = calculate_lora_airtime(52, phy)
    print(f"Airtime for 52-byte packet (SF7, BW125): {airtime_52b*1000.0:.2f} ms")
    assert 0.080 < airtime_52b < 0.120, "Airtime check failed"

    # Create synthetic candidates
    sample_candidates = [
        ConstraintCandidate(sequence_id=1, robot_i=0, robot_j=1, timestamp_gen_s=0.0, residual_m=0.12, novelty=0.9),
        ConstraintCandidate(sequence_id=2, robot_i=0, robot_j=1, timestamp_gen_s=0.2, residual_m=0.35, novelty=0.3),
        ConstraintCandidate(sequence_id=3, robot_i=1, robot_j=2, timestamp_gen_s=0.5, residual_m=0.08, novelty=0.8),
    ]

    sel, ret = schedule_epoch(sample_candidates, SchedulingPolicy.BACS_PLUS, now_s=1.0, sched_cfg=cfg, phy_cfg=phy)
    print(f"Selected {len(sel)} constraints, retained {len(ret)} constraints.")
    assert len(sel) >= 1, "Admission check failed"
    print("BACS core reference test passed successfully.")
