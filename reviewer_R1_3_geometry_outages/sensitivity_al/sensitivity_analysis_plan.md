# Sensitivity Analysis Plan: Attempt-Level Matched Independent Loss (Condition C4-IID-AL)

**Document Version:** 1.0 (Pre-Execution Protocol Freeze)  
**Date:** 2026-11-24  
**Context:** Reviewer 1 Comment R1.3 — BACS / BACS+ Multi-Robot SLAM Evaluation  
**Status:** Frozen Prior to Execution  

---

## 1. Objective and Scientific Rationale

Condition C4 evaluates multi-robot scheduling under physical obstacle blockage (six metallic warehouse racks in a 14.0 m x 10.5 m arena), producing spatio-temporally correlated radio outages with Line-of-Sight (LoS, $P_{\text{deliv}} = 0.95$) and Non-Line-of-Sight (NLoS, $P_{\text{deliv}} = 0.10$) propagation states.
Condition C4-IID matched the trajectory-average packet loss ($P_{\text{drop}} = 0.2615$) as an independent Bernoulli process.

To test sensitivity against potential discrepancies between trajectory-average loss and realized attempt-level transmission loss, this protocol implements **Condition C4-IID-AL** (Attempt-Level Matched Independent Loss):
- Per-attempt packet drop probability: $P_{\text{drop}} = 0.3739$.
- Target nominal per-attempt delivery probability: $P_{\text{deliv}} = 0.6261$.

This is a post hoc sensitivity experiment designed to probe the boundary between correlated obstacle outages and elevated independent packet drop, rather than an independently preregistered confirmation experiment. The drop probability $P_{\text{drop}} = 0.3739$ is strictly frozen prior to execution and will not be tuned post hoc based on observed map-alignment outcomes.

---

## 2. Experimental Design and Factorial Matrix

The sensitivity experiment uses the exact pristine seed block and parameters as C4 and C4-IID:

- **Seeds:** 130 to 159 inclusive ($N_{\text{seeds}} = 30$).
- **Team Sizes:** $N \in \{2, 3, 4, 5\}$ robots ($4$ team sizes).
- **Scheduling Policies:**
  1. `fifo` (First-In, First-Out)
  2. `random` (Uniform Random Selection)
  3. `lifo` (Last-In, First-Out)
  4. `bacs_gated` (BACS with Trust Gating)
  5. `bacs_plus` (BACS+ with Trust Gating and Pairwise Observability Balancing)
- **Total Sensitivity Runs:** $30 \times 4 \times 5 = 600$ simulation runs.
- **Parallel Workers:** 12 workers via `multiprocessing.Pool`.
- **Preserved Invariants:**
  - Identical robot waypoint trajectories and ground truth (`RobotTruth`).
  - Identical loop-closure candidate pools precomputed via `precompute(cfg)` with deep-cloning per policy.
  - Identical LoRa PHY/MAC constraints: SF7, BW 125 kHz, CR 4/5, 52-byte payload, 1% duty-cycle ceiling, max 3 retries (4 total attempts), 0.5 s ACK timeout.
  - Identical session duration (600 s) and scheduling window (60 s).
  - Identical Levenberg-Marquardt pose-graph optimization backend.

---

## 3. Storage and Directory Isolation

To ensure that existing experimental artifacts are strictly preserved:
- Existing C4 runs (`raw_results/raw_results_C4.csv`) and C4-IID runs (`raw_results/raw_results_C4_IID.csv`) remain unchanged.
- All new sensitivity results will be written to:
  `reviewer_R1_3_geometry_outages/sensitivity_al/raw_results_C4_IID_AL.csv`.
- Processed comparisons and summary statistical tables will be written to:
  `reviewer_R1_3_geometry_outages/sensitivity_al/`.

---

## 4. Evaluation Metrics and Reporting

1. **Map Alignment RMSE (m):** Evaluated over ground-truth co-location pairs ($d < 3.0$ m).
2. **Trajectory ATE RMSE (m):** Absolute trajectory error after first-pose SE(2) gauge alignment.
3. **Realized Delivery Rate (%):** Delivered packets divided by total transmission attempts ($N_{\text{deliv}} / N_{\text{att}}$).
4. **Unique Packet Delivery ($N_{\text{deliv}}$):** Total successfully delivered inter-robot loop closures.
5. **Mean Trust Yield ($\bar{\theta}$):** Average trust metric of delivered constraints.
6. **High-Trust Delivery Fraction (%):** Fraction of delivered constraints satisfying $\theta > 0.10$ (formerly referred to as inlier acceptance rate).
7. **Airtime Utilization (%):** Consumed channel airtime relative to regulatory duty-cycle budget.
8. **Constraint Expiry / Starvation:** Maximum queue deferrals before candidate expiration ($\text{deferrals} \ge 12$).

---

## 5. Statistical Analysis Protocol

- **Seed-Level Paired Contrasts:**
  - BACS vs. FIFO
  - BACS+ vs. FIFO
  - BACS vs. LIFO
  - BACS+ vs. LIFO
  - BACS vs. BACS+
  - BACS vs. Random
  - BACS+ vs. Random
  - Cross-condition paired contrasts: C4 vs. C4-IID, C4 vs. C4-IID-AL, C4-IID vs. C4-IID-AL for each policy.
- **Estimators:**
  - Mean paired difference $\bar{\Delta} = \text{Metric}(A) - \text{Metric}(B)$ (for RMSE, negative indicates superiority).
  - 95% Percentile Bootstrap Confidence Interval (10,000 resamples).
  - Non-parametric effect size: Cliff's delta ($\delta$) and paired Cohen's $d_z$.
  - Two-sided Wilcoxon signed-rank test.
  - Multiplicity adjustment: Holm-Bonferroni step-down procedure within clearly specified comparison families.
- **Retention Rule:** Complete retention of all outcomes, including non-significant and negative findings.
