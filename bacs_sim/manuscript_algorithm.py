"""Executable reference for BACS/BACS+ Equations (5)-(9) and Algorithm 1.

This module is intentionally separate from the legacy simulator implementation.
It provides a small, deterministic specification that can be unit tested without
changing the historical code path that produced the manuscript CSV files.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import exp, floor
from typing import Iterable

from .config import InfoGainConfig, TrustConfig
from .lora import time_on_air


@dataclass(frozen=True)
class Candidate:
    sequence: int
    generated_s: float
    provisional_residual_m: float
    novelty: float
    normalized_degree: float
    normalized_loop_span: float
    robot_i: int
    robot_j: int
    delivered_pair_count: int = 0
    payload_bytes: int = 52
    deferrals: int = 0
    predicted_age_s: float = 0.0
    predicted_trust: float = 0.0
    information_score: float = 0.0


def information_surrogate(candidate: Candidate, cfg: InfoGainConfig) -> float:
    """Equation (7), with all inputs expected in [0, 1]."""
    return (
        cfg.w_novelty * candidate.novelty
        + cfg.w_degree * (1.0 - candidate.normalized_degree)
        + cfg.w_loop * candidate.normalized_loop_span
    )


def pairwise_score(delivered_pair_count: int, n_ref: float = 6.0) -> float:
    """Pairwise coverage term from Equation (8)."""
    if n_ref <= 0:
        raise ValueError("n_ref must be positive")
    return exp(-max(delivered_pair_count, 0) / n_ref)


def arrival_age_s(
    *, now_s: float, generated_s: float, airtime_ahead_s: float,
    own_airtime_s: float, budget_s: float, window_s: float,
    loss_probability: float = 0.0, ack_timeout_s: float = 0.5,
) -> float:
    """Equation (5), including accumulated age and the retry expectation."""
    if budget_s <= 0 or window_s <= 0 or own_airtime_s < 0 or airtime_ahead_s < 0:
        raise ValueError("airtime, budget, and window values must be valid")
    loss = min(max(loss_probability, 0.0), 0.95)
    retry = loss * (own_airtime_s + ack_timeout_s) / (1.0 - loss)
    whole_windows = floor((airtime_ahead_s + own_airtime_s) / budget_s)
    within_send_window = airtime_ahead_s - whole_windows * budget_s
    return (
        max(now_s - generated_s, 0.0)
        + whole_windows * window_s
        + within_send_window
        + own_airtime_s
        + retry
    )


def predicted_trust(
    residual_m: float, age_s: float, cfg: TrustConfig, gamma: float,
    *, map_age_s: float = 0.0, robot_speed: float = 0.3, use_confidence_blend: bool = True,
) -> float:
    """Equation (6); when use_confidence_blend is True, applies the legacy
    map-confidence blending from Eq. (10) / bacs_sim.trust.predicted_trust.
    Unlike server trust, this expression is not floored.
    """
    spatial = max(0.0, 1.0 - (max(residual_m, 0.0) / cfg.tau_e) ** cfg.p)
    if use_confidence_blend:
        conf = 1.0 / (1.0 + (robot_speed * max(map_age_s, 0.0)) / cfg.tau_e)
        spatial = conf * spatial + (1.0 - conf) * 1.0
    return spatial * exp(-gamma * max(age_s, 0.0))


def _score(
    candidate: Candidate, *, now_s: float, airtime_ahead_s: float,
    budget_s: float, window_s: float, loss_probability: float,
    ack_timeout_s: float, gamma: float, trust_cfg: TrustConfig,
    info_cfg: InfoGainConfig, bacs_plus: bool, lora,
    map_age_s: float = 0.0, robot_speed: float = 0.3, use_confidence_blend: bool = True,
) -> Candidate:
    own = time_on_air(candidate.payload_bytes, lora)
    age = arrival_age_s(
        now_s=now_s, generated_s=candidate.generated_s,
        airtime_ahead_s=airtime_ahead_s, own_airtime_s=own,
        budget_s=budget_s, window_s=window_s,
        loss_probability=loss_probability, ack_timeout_s=ack_timeout_s,
    )
    info = information_surrogate(candidate, info_cfg)
    if bacs_plus:
        info += info_cfg.w_obs * pairwise_score(
            candidate.delivered_pair_count, info_cfg.obs_ref
        )
    return replace(
        candidate,
        predicted_age_s=age,
        predicted_trust=predicted_trust(
            candidate.provisional_residual_m, age, trust_cfg, gamma,
            map_age_s=map_age_s, robot_speed=robot_speed, use_confidence_blend=use_confidence_blend,
        ),
        information_score=info,
    )


def select_window(
    candidates: Iterable[Candidate], *, now_s: float, budget_s: float,
    window_s: float, gamma: float, trust_cfg: TrustConfig,
    info_cfg: InfoGainConfig, lora, bacs_plus: bool = False,
    trust_gate: float = 0.05, max_deferrals: int = 12,
    loss_probability: float = 0.0, freshness_rate: float = 0.0,
    map_age_s: float = 0.0, robot_speed: float = 0.3, use_confidence_blend: bool = True,
) -> tuple[list[Candidate], list[Candidate]]:
    """Two-pass Algorithm 1 selection.

    Returns ``(selected, retained)``. ``retained`` excludes selected and expired
    candidates and increments the deferral count of every eligible non-selection.
    Ties are resolved by ascending sequence number.
    """
    if freshness_rate < 0:
        raise ValueError("freshness_rate must be nonnegative")

    def priority(c):
        density = c.information_score * exp(-freshness_rate * c.predicted_age_s)
        return (-density / time_on_air(c.payload_bytes, lora), c.sequence)

    pool = list(candidates)
    if not pool:
        return [], []

    # Pass 1: every candidate is provisionally placed at the middle of a budget.
    first = [
        _score(
            c, now_s=now_s, airtime_ahead_s=budget_s * 0.5,
            budget_s=budget_s, window_s=window_s,
            loss_probability=loss_probability,
            ack_timeout_s=getattr(lora, "ack_timeout_s", 0.5), gamma=gamma,
            trust_cfg=trust_cfg, info_cfg=info_cfg,
            bacs_plus=bacs_plus, lora=lora,
            map_age_s=map_age_s, robot_speed=robot_speed, use_confidence_blend=use_confidence_blend,
        )
        for c in pool
    ]
    first.sort(key=priority)

    # Pass 2: recompute arrival age from the first-pass order.
    second: list[Candidate] = []
    ahead = 0.0
    for c in first:
        scored = _score(
            c, now_s=now_s, airtime_ahead_s=ahead,
            budget_s=budget_s, window_s=window_s,
            loss_probability=loss_probability,
            ack_timeout_s=getattr(lora, "ack_timeout_s", 0.5), gamma=gamma,
            trust_cfg=trust_cfg, info_cfg=info_cfg,
            bacs_plus=bacs_plus, lora=lora,
            map_age_s=map_age_s, robot_speed=robot_speed, use_confidence_blend=use_confidence_blend,
        )
        second.append(scored)
        ahead += time_on_air(c.payload_bytes, lora)

    live = [
        c for c in second
        if c.predicted_trust >= 1.5 * trust_cfg.floor
        and c.deferrals < max_deferrals
    ]
    admitted = [c for c in live if c.predicted_trust >= trust_gate]
    if not admitted:  # Algorithm 1 gate relaxation.
        admitted = live

    admitted.sort(key=priority)
    selected: list[Candidate] = []
    used = 0.0
    for c in admitted:
        airtime = time_on_air(c.payload_bytes, lora)
        if used + airtime <= budget_s + 1e-12:
            selected.append(c)
            used += airtime

    selected_ids = {c.sequence for c in selected}
    retained = [
        replace(c, deferrals=c.deferrals + 1)
        for c in live
        if c.sequence not in selected_ids
        and c.deferrals + 1 < max_deferrals
    ]
    return selected, retained
