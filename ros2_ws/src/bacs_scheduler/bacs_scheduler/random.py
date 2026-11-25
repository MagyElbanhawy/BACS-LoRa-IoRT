"""RANDOM baseline policy factory.

The RANDOM policy is an ablation baseline: it picks eligible constraints in a
seeded, reproducible shuffle. It is not meant to be competitive with FIFO /
BACS / BACS+ ; it exists so that the physical evidence pipeline can quantify
the value of an *informed* scheduler against an *un-informed* one on the same
hardware, same duty-cycle budget, and same radio configuration.
"""
from .scheduler import DutyCycleBudget, Scheduler


def create(
    budget: DutyCycleBudget,
    trust_threshold: float = 0.5,
    random_seed: int = 0,
) -> Scheduler:
    return Scheduler(
        "RANDOM",
        budget,
        trust_threshold=trust_threshold,
        random_seed=random_seed,
    )
