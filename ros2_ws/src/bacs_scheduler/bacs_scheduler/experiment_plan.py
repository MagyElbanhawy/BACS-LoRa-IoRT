"""Balanced four-policy plans for the physical BACS experiment.

One block contains FIFO, BACS, BACS+ and RANDOM exactly once.  The starting
policy rotates between blocks, so policy and run order are not confounded.
The functions in this module are ROS-free and are also used by the command-line
session runner.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass


POLICIES = ("FIFO", "BACS", "BACS+", "RANDOM")


@dataclass(frozen=True)
class PlannedRun:
    block: int
    position: int
    run: int
    session: str
    policy: str
    duration_s: int


def policy_order(block: int) -> tuple[str, ...]:
    """Return the cyclic, balanced policy order for a one-based block number."""
    if block < 1:
        raise ValueError("block must be at least 1")
    shift = (block - 1) % len(POLICIES)
    return POLICIES[shift:] + POLICIES[:shift]


def block_plan(block: int, duration_s: int = 600, first_session: int = 1) -> list[PlannedRun]:
    """Build four immutable run records for ``block``.

    Session numbers are globally unique as long as every block uses the same
    ``first_session`` value.  The per-policy run number equals the block number,
    which makes paired analysis straightforward.
    """
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if first_session < 1:
        raise ValueError("first_session must be positive")
    base = first_session + (block - 1) * len(POLICIES)
    result = []
    for position, policy in enumerate(policy_order(block), start=1):
        result.append(PlannedRun(block, position, block, f"HWS-BACS-{base + position - 1:03d}",
                                 policy, duration_s))
    return result


def plan_as_dicts(block: int, duration_s: int = 600, first_session: int = 1) -> list[dict]:
    return [asdict(item) for item in block_plan(block, duration_s, first_session)]


def expected_policy(session: str, first_session: int = 1) -> tuple[int, str]:
    """Return ``(block, policy)`` encoded by a numbered ``HWS-BACS-NNN`` ID."""
    prefix = "HWS-BACS-"
    number_text = session[len(prefix):] if session.startswith(prefix) else ""
    if len(number_text) != 3 or not number_text.isdigit():
        raise ValueError("measured session IDs must have the form HWS-BACS-NNN")
    number = int(number_text)
    offset = number - first_session
    if offset < 0:
        raise ValueError(f"session number must be at least {first_session:03d}")
    block, position = divmod(offset, len(POLICIES))
    return block + 1, policy_order(block + 1)[position]
