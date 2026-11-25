"""Inter-robot constraint candidate source for BACS sessions.

Generates realistic SLAM constraint candidates that the scheduler can rank and
select for transmission. Each constraint represents an inter-robot pose-graph
edge produced by ICP scan matching between two AgileX LIMO robots.

Constraint fields (matching scheduler.Constraint):
    sequence          - monotonic generation counter
    generated_ns      - host monotonic clock at generation time
    payload_bytes     - always 52 (LoRa packet size, matches config/lora.yaml)
    predicted_trust   - ICP quality surrogate: inlier_ratio * exp(-rmse / ref)
    information_score  - information-theoretic: 1 / (1 + sqrt(trace(Sigma_xy)) / ref)
    robot_i           - source robot name (e.g. "LIMO-01")
    robot_j           - target robot name (e.g. "LIMO-02")
    pair_constraints  - count of prior constraints between this robot pair
                        (drives the BACS+ observability weighting)

The generator is seeded for reproducibility: identical seeds produce identical
constraint streams, so FIFO / BACS / BACS+ / RANDOM sessions can be compared
on the same candidate set.
"""
from __future__ import annotations

import math
import random as _random
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

from .scheduler import Constraint


# Two-robot testbed from the paper. Hardcoded here so the constraint source
# doesn't need ROS 2 parameter plumbing for a standalone capture session.
ROBOTS = ("LIMO-01", "LIMO-02")

# 52-byte payload layout for a serialized constraint:
#   seq        uint32   4
#   robot_i    uint8    1   (0=LIMO-01, 1=LIMO-02)
#   robot_j    uint8    1
#   kf_i       uint32   4   (keyframe index on robot_i)
#   kf_j       uint32   4   (keyframe index on robot_j)
#   dx         float32  4   (relative x, metres)
#   dy         float32  4   (relative y, metres)
#   dtheta     float32  4   (relative yaw, radians)
#   trust      float32  4
#   info       float32  4
#   pair_cnt   uint16   2
#   reserved   bytes   16
# Total = 52
PAYLOAD_FORMAT = ">IBBIIfff ffH16s"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
assert PAYLOAD_SIZE == 52, f"payload layout is {PAYLOAD_SIZE} bytes, expected 52"

ROBOT_ID = {name: idx for idx, name in enumerate(ROBOTS)}


@dataclass
class ConstraintSource:
    """Generates inter-robot SLAM constraint candidates.

    Parameters
    ----------
    seed:
        RNG seed for reproducible constraint streams.
    generation_period_s:
        How often a new constraint candidate appears in the queue.
    trust_ref:
        RMSE reference for the trust formula (lower = stricter).
    info_ref:
        Uncertainty reference for the information formula.
    """

    seed: int = 42
    generation_period_s: float = 2.0
    trust_ref: float = 0.05
    info_ref: float = 0.05

    _rng: _random.Random = field(init=False, repr=False)
    _seq: int = field(init=False, default=0, repr=False)
    _pair_counts: dict[tuple[str, str], int] = field(init=False, repr=False, default_factory=dict)
    _next_gen_ns: int = field(init=False, default=0, repr=False)

    def __post_init__(self) -> None:
        self._rng = _random.Random(self.seed)

    def maybe_generate(self, now_ns: int) -> list[Constraint]:
        """Return a list of 0 or more new constraints generated since last call."""
        out: list[Constraint] = []
        if self._next_gen_ns == 0:
            self._next_gen_ns = now_ns + int(self.generation_period_s * 1e9)
            return out
        while now_ns >= self._next_gen_ns:
            out.append(self._generate_one(self._next_gen_ns))
            self._next_gen_ns += int(self.generation_period_s * 1e9)
        return out

    def _generate_one(self, generated_ns: int) -> Constraint:
        self._seq += 1

        # Pick a robot pair. 80 % are inter-robot (LIMO-01 <-> LIMO-02),
        # 20 % are self-loopback (rare but realistic for loop-closure).
        if self._rng.random() < 0.8:
            ri, rj = 0, 1
        else:
            ri = self._rng.randint(0, 1)
            rj = ri  # self constraint (loop closure)
        robot_i = ROBOTS[ri]
        robot_j = ROBOTS[rj]

        key = (robot_i, robot_j)
        pair_count = self._pair_counts.get(key, 0)
        self._pair_counts[key] = pair_count + 1

        # ICP quality surrogates.
        # Inlier ratio: 0.4 - 0.98 (beta-distributed, centred ~0.75)
        inlier_ratio = self._rng.betavariate(5, 2) * 0.6 + 0.4
        # RMSE: 0.01 - 0.20 m (exponential, mean ~0.06)
        rmse = self._rng.expovariate(1 / 0.06)
        rmse = min(rmse, 0.20)

        # Trust = inlier_ratio * exp(-rmse / trust_ref)
        predicted_trust = inlier_ratio * math.exp(-rmse / self.trust_ref)

        # Pose uncertainty (grows as robots explore further).
        # Model: trace(Sigma_xy) grows with sequence number.
        uncertainty = 0.02 + 0.001 * self._seq + self._rng.gauss(0, 0.005)
        uncertainty = max(0.01, uncertainty)
        # Information = 1 / (1 + sqrt(uncertainty) / info_ref)
        information_score = 1.0 / (1.0 + math.sqrt(uncertainty) / self.info_ref)

        return Constraint(
            sequence=self._seq,
            generated_ns=generated_ns,
            payload_bytes=52,
            predicted_trust=predicted_trust,
            information_score=information_score,
            robot_i=robot_i,
            robot_j=robot_j,
            pair_constraints=pair_count,
        )

    @staticmethod
    def serialize(constraint: Constraint) -> bytes:
        """Pack a constraint into the 52-byte LoRa payload."""
        ri = ROBOT_ID.get(constraint.robot_i, 0)
        rj = ROBOT_ID.get(constraint.robot_j, 0)
        # Deterministic-ish dx/dy/dtheta for reproducibility (based on seq).
        dx = (constraint.sequence % 100) * 0.1 - 5.0   # -5.0 .. +4.9
        dy = (constraint.sequence % 50) * 0.2 - 5.0    # -5.0 .. +4.8
        dtheta = (constraint.sequence % 16) * 0.4 - 3.2  # -3.2 .. +2.8
        return struct.pack(
            PAYLOAD_FORMAT,
            constraint.sequence,
            ri, rj,
            constraint.sequence * 7,        # kf_i (arbitrary but deterministic)
            constraint.sequence * 11,       # kf_j
            dx, dy, dtheta,
            constraint.predicted_trust,
            constraint.information_score,
            constraint.pair_constraints,
            b"\x00" * 16,
        )


__all__ = ["ConstraintSource", "ROBOTS", "PAYLOAD_FORMAT"]
