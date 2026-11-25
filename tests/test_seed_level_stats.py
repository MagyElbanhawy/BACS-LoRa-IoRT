import numpy as np
import pandas as pd

from scripts.stats import paired_seed_means, tost_log_ratio


def test_seed_aggregation_precedes_inference():
    df = pd.DataFrame([
        {"seed": 1, "condition": "C0", "n": 2, "arm": "a", "align_rmse": 1.0},
        {"seed": 1, "condition": "C0", "n": 2, "arm": "b", "align_rmse": 2.0},
        {"seed": 1, "condition": "C1", "n": 3, "arm": "a", "align_rmse": 3.0},
        {"seed": 1, "condition": "C1", "n": 3, "arm": "b", "align_rmse": 2.0},
        {"seed": 2, "condition": "C0", "n": 2, "arm": "a", "align_rmse": 2.0},
        {"seed": 2, "condition": "C0", "n": 2, "arm": "b", "align_rmse": 1.0},
    ])
    paired = paired_seed_means(df, "arm", "a", "b", ["seed", "condition", "n"])
    assert len(paired) == 2
    assert paired.loc[paired.seed == 1, "difference"].item() == 0.0


def test_tost_uses_seed_level_log_ratios():
    ratios = np.array([0.97, 0.98, 1.00, 1.01, 0.99, 0.96])
    paired = pd.DataFrame({"log_ratio": np.log(ratios)})
    result = tost_log_ratio(paired, 0.90, 1.10)
    assert 0.90 < result["geometric_mean_ratio"] < 1.10
    assert result["ratio_ci90_lo"] < result["geometric_mean_ratio"] < result["ratio_ci90_hi"]
