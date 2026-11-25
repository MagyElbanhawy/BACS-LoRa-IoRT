"""
Physical Channel Emulators for Reviewer 1.3.

Provides:
- GeometryLoRaChannel (Condition C4): Dynamic LoS / NLoS channel state driven
  by physical robot position in the 14.0 m x 10.5 m arena with 6 warehouse racks.
  LoS delivery probability: 0.95 (5% drop).
  NLoS delivery probability: 0.10 (90% drop).
- MatchedIIDChannel (Condition C4-IID): Independent Bernoulli channel with drop
  probability exactly matched to the trajectory-average loss of C4 (~26.15%).

Strictly adheres to LoRa airtime charging: failed attempts occupy the radio
channel and consume time on air and duty cycle budget identically to successful attempts.
"""
from typing import List, Tuple, Dict, Any, Optional
import numpy as np

from bacs_sim.config import ChannelConfig, LoRaConfig
from bacs_sim.lora import time_on_air
from bacs_sim.world import RobotTruth

from .geometry_model import (
    RectObstacle,
    DEFAULT_OBSTACLES,
    DEFAULT_SERVER_POS,
    P_DELIV_UNOBSTRUCTED,
    P_DELIV_OBSTRUCTED,
    check_link_blocked,
)

# Factored trajectory-average matched loss rate calculated across seeds 130-159:
# 24.89% blockage * 0.90 loss + 75.11% LoS * 0.05 loss = 26.15% drop rate
MATCHED_C4_IID_LOSS_RATE = 0.2615


class GeometryLoRaChannel:
    """
    Geometry-dependent LoRa channel emulator (Condition C4).
    Evaluates radio propagation path against physical obstacles at the exact
    transmission instant.
    """

    def __init__(
        self,
        cfg: ChannelConfig,
        lora: LoRaConfig,
        rng: np.random.Generator,
        truths: Optional[List[RobotTruth]] = None,
        obstacles: List[RectObstacle] = DEFAULT_OBSTACLES,
        server_pos: Tuple[float, float] = DEFAULT_SERVER_POS,
        p_los: float = P_DELIV_UNOBSTRUCTED,
        p_nlos: float = P_DELIV_OBSTRUCTED,
        dt: float = 0.1,
    ):
        self.cfg = cfg
        self.lora = lora
        self.rng = rng
        self.truths = truths
        self.obstacles = obstacles
        self.server_pos = server_pos
        self.p_los = p_los
        self.p_nlos = p_nlos
        self.dt = dt

        # Outage and transmission telemetry
        self.history: List[Dict[str, Any]] = []
        self.n_transmissions = 0
        self.n_attempts_total = 0
        self.n_delivered = 0
        self.n_los_attempts = 0
        self.n_los_delivered = 0
        self.n_nlos_attempts = 0
        self.n_nlos_delivered = 0

    def set_truths(self, truths: List[RobotTruth]) -> None:
        """Attach robot ground-truth trajectories for position lookups."""
        self.truths = truths

    def get_robot_position(self, rid: int, t: float) -> Tuple[float, float]:
        """Look up 2D ground-truth position (x, y) of robot rid at time t."""
        if self.truths is None or rid >= len(self.truths):
            # Fallback if truths not bound
            return (0.0, 0.0)
        truth = self.truths[rid]
        idx = int(np.clip(t / self.dt, 0, len(truth.gt) - 1))
        return float(truth.gt[idx, 0]), float(truth.gt[idx, 1])

    def transmit(
        self,
        payload_bytes: int,
        t_ready: float,
        rid: Optional[int] = None,
    ) -> Tuple[bool, float, float, int]:
        """
        Attempt delivery of one packet offered to the radio at t_ready.

        Returns: (delivered, t_recv, airtime_consumed, n_attempts).
        Airtime is charged for every attempt including failures.
        """
        t_air = time_on_air(payload_bytes, self.lora)
        t = t_ready
        used = 0.0
        self.n_transmissions += 1

        for attempt in range(self.cfg.max_retries + 1):
            t += t_air
            used += t_air
            self.n_attempts_total += 1

            # Determine spatial blockage state at transmission instant
            if rid is not None and self.truths is not None:
                pos = self.get_robot_position(rid, t)
                blocked = check_link_blocked(pos, self.server_pos, self.obstacles)
            else:
                pos = (0.0, 0.0)
                blocked = False

            # Delivery probability depends on geometry
            p_deliv = self.p_nlos if blocked else self.p_los
            if blocked:
                self.n_nlos_attempts += 1
            else:
                self.n_los_attempts += 1

            success = bool(self.rng.random() < p_deliv)

            if success:
                if blocked:
                    self.n_nlos_delivered += 1
                else:
                    self.n_los_delivered += 1
                self.n_delivered += 1

                self.history.append({
                    "rid": rid,
                    "t_sent": t_ready,
                    "t_recv": t + self.cfg.extra_delay_s,
                    "payload_bytes": payload_bytes,
                    "attempts": attempt + 1,
                    "blocked": blocked,
                    "delivered": True,
                    "pos": pos,
                })
                return True, t + self.cfg.extra_delay_s, used, attempt + 1

            t += self.lora.ack_timeout_s

        self.history.append({
            "rid": rid,
            "t_sent": t_ready,
            "t_recv": t,
            "payload_bytes": payload_bytes,
            "attempts": self.cfg.max_retries + 1,
            "blocked": blocked,
            "delivered": False,
            "pos": pos,
        })
        return False, t, used, self.cfg.max_retries + 1


class MatchedIIDChannel:
    """
    Matched-rate independent packet loss channel emulator (Condition C4-IID).
    Applies identical average drop rate as C4, but uniformly and independently across time,
    without spatial or temporal correlation.
    """

    def __init__(
        self,
        cfg: ChannelConfig,
        lora: LoRaConfig,
        rng: np.random.Generator,
        loss_rate: float = MATCHED_C4_IID_LOSS_RATE,
    ):
        self.cfg = cfg
        self.lora = lora
        self.rng = rng
        self.loss_rate = loss_rate

        self.n_transmissions = 0
        self.n_attempts_total = 0
        self.n_delivered = 0

    def transmit(
        self,
        payload_bytes: int,
        t_ready: float,
        rid: Optional[int] = None,
    ) -> Tuple[bool, float, float, int]:
        """
        Attempt delivery of one packet offered to the radio at t_ready.
        Returns: (delivered, t_recv, airtime_consumed, n_attempts).
        """
        t_air = time_on_air(payload_bytes, self.lora)
        t = t_ready
        used = 0.0
        self.n_transmissions += 1

        for attempt in range(self.cfg.max_retries + 1):
            t += t_air
            used += t_air
            self.n_attempts_total += 1

            success = bool(self.rng.random() >= self.loss_rate)
            if success:
                self.n_delivered += 1
                return True, t + self.cfg.extra_delay_s, used, attempt + 1

            t += self.lora.ack_timeout_s

        return False, t, used, self.cfg.max_retries + 1
