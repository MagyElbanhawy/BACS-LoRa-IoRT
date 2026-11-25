# Seed-level reanalysis amendment

Date: 2026-11-22

This document is a transparent retrospective amendment, not a publicly registered preregistration. Earlier analyses sometimes pooled condition and team-size rows. Because these are repeated observations generated from the same seed, the seed is the independent unit.

For confirmation (seeds 70–99), match BACS+ and Random within seed, condition, and team size. Average paired log alignment-RMSE ratios within seed. Report their exponentiated mean and 90% t confidence interval, then perform TOST on the 30 mean log-ratios with ratio bounds 0.90 and 1.10. Also report the two-sided paired Wilcoxon test on 30 seed-level mean differences.

For the EMRMF bridge (seeds 100–129), match BACS+ and published EMRMF+FIFO within seed, condition, and team size. Average paired alignment-RMSE differences within seed, then perform a two-sided paired Wilcoxon test on the 30 seed-level differences. The descriptive percentage reduction remains based on the corresponding raw-observation means.

For decay (seeds 10–39), use the same within-seed reduction across team sizes for the rule comparisons and log-ratio equivalence analysis.

The implementation is `scripts/stats.py`; tests are in `tests/test_seed_level_stats.py`. This amendment does not alter raw results.
