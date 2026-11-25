"""
Geometry-Dependent Radio Outage Model for Reviewer 1.3 (Condition C4).

Implements 2D obstacle intersection, Line-of-Sight (LoS) / Non-Line-of-Sight (NLoS)
classification, spatial outage tracking, and matched-rate loss calibration.
"""
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Any
import numpy as np


@dataclass(frozen=True)
class RectObstacle:
    """Axis-aligned rectangular obstacle [x_min, x_max, y_min, y_max]."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    name: str = "obstacle"

    def bounds(self) -> Tuple[float, float, float, float]:
        return (self.x_min, self.x_max, self.y_min, self.y_max)


# Fixed, pre-registered obstacle environment in 14.0 m x 10.5 m arena:
# Six warehouse storage racks arranged in two tiers of three aisles
DEFAULT_OBSTACLES = [
    # North tier (y in [6.5, 8.0])
    RectObstacle(1.5, 4.5, 6.5, 8.0, name="Rack_North_West"),
    RectObstacle(5.5, 8.5, 6.5, 8.0, name="Rack_North_Center"),
    RectObstacle(9.5, 12.5, 6.5, 8.0, name="Rack_North_East"),
    # South tier (y in [2.5, 4.0])
    RectObstacle(1.5, 4.5, 2.5, 4.0, name="Rack_South_West"),
    RectObstacle(5.5, 8.5, 2.5, 4.0, name="Rack_South_Center"),
    RectObstacle(9.5, 12.5, 2.5, 4.0, name="Rack_South_East"),
]

# Fusion server radio endpoint fixed at the northern ceiling / entry boundary
DEFAULT_SERVER_POS = (7.0, 10.5)

# Stress-test transmission probabilities specified by Reviewer 1.3 / prompt:
P_DELIV_UNOBSTRUCTED = 0.95
P_DELIV_OBSTRUCTED = 0.10


def line_intersects_rect(p1: Tuple[float, float], p2: Tuple[float, float],
                         rect: RectObstacle) -> bool:
    """
    Check whether line segment connecting p1 and p2 intersects the axis-aligned rect.
    Uses the Liang-Barsky parametric clipping algorithm.
    """
    x0, x1, y0, y1 = rect.bounds()
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    p = [-dx, dx, -dy, dy]
    q = [p1[0] - x0, x1 - p1[0], p1[1] - y0, y1 - p1[1]]

    u1 = 0.0
    u2 = 1.0

    for pi, qi in zip(p, q):
        if pi == 0.0:
            if qi < 0.0:
                return False  # Parallel and outside
        else:
            t = qi / pi
            if pi < 0.0:
                if t > u2:
                    return False
                if t > u1:
                    u1 = t
            else:
                if t < u1:
                    return False
                if t < u2:
                    u2 = t

    return u1 <= u2


def check_link_blocked(robot_pos: Tuple[float, float],
                       server_pos: Tuple[float, float] = DEFAULT_SERVER_POS,
                       obstacles: List[RectObstacle] = DEFAULT_OBSTACLES) -> bool:
    """Return True if propagation path from robot_pos to server_pos intersects any obstacle."""
    for obs in obstacles:
        if line_intersects_rect(robot_pos, server_pos, obs):
            return True
    return False


def get_delivery_probability(is_blocked: bool,
                             p_los: float = P_DELIV_UNOBSTRUCTED,
                             p_nlos: float = P_DELIV_OBSTRUCTED) -> float:
    """Return packet delivery probability based on blockage state."""
    return p_nlos if is_blocked else p_los


def compute_outage_statistics(blocked_series: np.ndarray, dt: float) -> Dict[str, float]:
    """
    Compute outage metrics from a boolean array of blockage states over time.
    blocked_series[k] == True indicates obstructed link at step k.
    """
    total_time = len(blocked_series) * dt
    if total_time == 0:
        return {
            "obstruction_fraction": 0.0,
            "n_outages": 0,
            "mean_outage_s": 0.0,
            "max_outage_s": 0.0,
        }

    fraction_blocked = float(np.mean(blocked_series))

    # Identify contiguous runs of blockage
    runs = []
    current_run = 0
    for b in blocked_series:
        if b:
            current_run += 1
        else:
            if current_run > 0:
                runs.append(current_run * dt)
                current_run = 0
    if current_run > 0:
        runs.append(current_run * dt)

    n_outages = len(runs)
    mean_outage = float(np.mean(runs)) if runs else 0.0
    max_outage = float(np.max(runs)) if runs else 0.0

    return {
        "obstruction_fraction": fraction_blocked,
        "n_outages": n_outages,
        "mean_outage_s": mean_outage,
        "max_outage_s": max_outage,
    }
