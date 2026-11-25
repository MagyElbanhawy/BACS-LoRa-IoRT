# Experimental Analysis Plan: Reviewer 1.3 — Geometry-Dependent Radio Outages

**Document Version:** 1.0 (Pre-Registration Frozen)  
**Target Manuscript:** BACS / BACS+ (Submission ID `ff2f82e9-99ff-4e72-8582-384551789f9b`)  
**Journal:** *Journal of King Saud University – Computer and Information Sciences*  
**Date:** November 2026  
**Status:** Pre-registered, Immutable Protocol  

---

## 1. Scientific Context & Reviewer Inquiry

Reviewer 1.3 highlighted that while the controlled loss conditions in the original manuscript (Conditions C1: Independent Loss, C2: Bursty Gilbert–Elliott Loss, and C3: Asymmetric Downlink Loss) successfully isolate scheduling effects, they do not establish scheduler robustness to **geometry-dependent radio availability** caused by physical obstacles during robot motion:

> *"The controlled loss conditions isolate scheduling effects but do not establish robustness to geometry-dependent radio availability during robot motion. Test correlated blockage or contact outages separately from the fixed loss processes, with the same candidate pools where possible. A broader discussion of spatial radio adaptation could clarify this boundary, without proposing RIS hardware as necessary for the present LoRa experiment."*  
> *(Reviewer also requested discussion of Oubbati et al., "A Comprehensive Analysis of RIS and Movable Antenna Placement in Future Wireless Networks," IEEE OJ-COMS, 2026).*

This analysis plan defines an isolated, rigorously controlled experimental investigation to evaluate multi-robot SLAM scheduling under physical obstacle blockage, compared directly against a matched-loss independent control.

---

## 2. Hypotheses & Pre-Registered Questions

### Hypothesis 1 ($H_1$): Spatial Outage Clustering vs. Matched Independent Loss
*When packet delivery failures are driven by spatial obstacle blockage (creating contiguous shadow outages as robots navigate warehouse aisles), does temporal clustering of outages degrade map-alignment RMSE relative to a memoryless independent channel with identical average packet delivery rate ($\bar{P}_{\text{deliv}} \approx 73.85\%$)?*
- **Mechanism:** In NLoS shadow zones, transmission attempts fail and retransmission delays consume available LoRa airtime, forcing constraint deferrals. Under matched independent loss (C4-IID), losses are dispersed across time, allowing intermittent successful transmissions even during queue congestion.

### Hypothesis 2 ($H_2$): Scheduling Policy Robustness Under Geometry Outages
*How do BACS and BACS+ compare against naive baseline policies (FIFO, Random, LIFO) under geometry-induced outages?*
- In prior degraded conditions (C1 and C2), Random scheduling exhibited competitive or superior alignment RMSE due to uniform temporal constraint sampling across trajectories, despite lower trust yield.
- We pre-register to report all empirical outcomes transparently: if Random or LIFO outperforms BACS+ in alignment RMSE under C4, we will document the exact effect size, report the causal trade-offs (temporal dispersion vs. trust yield and outlier filtering), and avoid asserting false superiority.

### Hypothesis 3 ($H_3$): Demarcation of Spatial Radio Adaptation vs. Estimator Scheduling
*Clarifying the architectural boundary between physical-layer spatial adaptation (e.g., Reconfigurable Intelligent Surfaces [RIS] or Movable Antennas [MA]) and application-layer scheduling (BACS).*
- Physical-layer adaptation (analyzed in Oubbati et al., 2026) dynamically optimizes reflection matrices or antenna positions to restore line-of-sight propagation or enhance channel gains in obstructed environments.
- In contrast, BACS operates at the application/estimator layer, taking physical channel constraints as an unalterable operational reality and optimizing the selection of transmitted loop closures within regulatory duty-cycle budgets.
- BACS does not require physical RIS hardware, but is complementary: physical adaptation improves raw link availability, while BACS ensures optimal information density per transmitted byte.

---

## 3. Experimental Design & Environment Specification

### 3.1 Arena Geometry and Obstacle Placement
- **Arena Dimensions:** $14.0\,\text{m} \times 10.5\,\text{m}$ rectangular industrial environment.
- **Fixed Infrastructure Antenna:** Fusion server radio antenna placed at northern ceiling boundary $(x = 7.0\,\text{m}, y = 10.5\,\text{m})$.
- **Obstacle Configuration:** Six warehouse storage racks arranged in two tiers of three aisles:
  - **North Tier ($y \in [6.5, 8.0]\,\text{m}$):**
    - Rack NW: $x \in [1.5, 4.5]\,\text{m}$
    - Rack NC: $x \in [5.5, 8.5]\,\text{m}$
    - Rack NE: $x \in [9.5, 12.5]\,\text{m}$
  - **South Tier ($y \in [2.5, 4.0]\,\text{m}$):**
    - Rack SW: $x \in [1.5, 4.5]\,\text{m}$
    - Rack SC: $x \in [5.5, 8.5]\,\text{m}$
    - Rack SE: $x \in [9.5, 12.5]\,\text{m}$
  - Aisles of $1.0\,\text{m}$ width ($x \in [4.5, 5.5]\,\text{m}$ and $x \in [8.5, 9.5]\,\text{m}$) provide realistic corridor traversal paths.

### 3.2 Evaluated Communication Conditions
1. **Condition C4 (Geometry-Dependent Radio Outages):**
   - Ray-obstacle intersection computed via 2D parametric Liang-Barsky clipping between transmitting robot $(x_r, y_r)$ and server antenna $(7.0, 10.5)$.
   - **Line-of-Sight (LoS / Unobstructed):** Delivery probability $P_{\text{deliv}}^{\text{LoS}} = 0.95$ ($P_{\text{drop}} = 0.05$).
   - **Non-Line-of-Sight (NLoS / Obstructed):** Delivery probability $P_{\text{deliv}}^{\text{NLoS}} = 0.10$ ($P_{\text{drop}} = 0.90$).
2. **Condition C4-IID (Matched-Rate Independent Packet Loss):**
   - Independent Bernoulli packet loss applied to every transmission attempt, independent of robot location.
   - Drop rate calibrated to the exact factorial trajectory-average loss of C4:
     $$\bar{P}_{\text{drop}}^{\text{matched}} = 0.2615 \quad (\bar{P}_{\text{deliv}} = 0.7385)$$
3. **Condition C0 (Unobstructed Reference):**
   - Baseline performance under unobstructed propagation ($P_{\text{drop}} = 0.00$).

### 3.3 Candidate Pool Isolation & Factorial Grid
- **Target Seed Block:** 30 pristine seeds: **130 through 159** (completely disjoint from frozen evaluation blocks 0–29 and 30–59).
- **Robot Team Sizes:** $N \in \{2, 3, 4, 5\}$.
- **Evaluated Policies:** 5 policies:
  1. `fifo`: First-In-First-Out queue order.
  2. `random`: Uniform random candidate selection within available airtime.
  3. `lifo`: Last-In-First-Out (freshest candidates first).
  4. `bacs` (`bacs_gated`): Corrected two-pass BACS algorithm with trust gate relaxation and map confidence blending.
  5. `bacs_plus`: BACS with pair-observability scoring.
- **Runs per Condition:** $30 \times 4 \times 5 = 600$ runs.
- **Total Factorial Runs:** 1,200 runs across C4 and C4-IID (plus C0 baseline reference).
- **Candidate Pool Isolation:** For each seed and team size, `precompute(cfg)` generates the ground truth trajectories and candidate pool once. All 5 policies execute on deep-cloned identical candidate sets.

### 3.4 Physical Radio & Session Constraints
- LoRa Modulation: SF7, Bandwidth 125 kHz, Coding Rate 4/5, Preamble 8 symbols.
- Regulatory Duty Cycle: 1% enforced on transmission airtime.
- Transmission Window: $W = 60.0\,\text{s}$ ($T_{\text{budget}} = 0.60\,\text{s}$ per robot per window).
- Session Duration: $T_{\text{session}} = 600.0\,\text{s}$ (10 full scheduling windows).
- Candidate Payload: 52 bytes ($T_{\text{air}} \approx 103\,\text{ms}$).
- Retransmission Policy: Up to 3 retries with ACK timeout ($0.5\,\text{s}$). Airtime charged for all attempts.

---

## 4. Evaluation Metrics & Statistical Protocol

### 4.1 Primary Metrics
- **Inter-Robot Map Alignment RMSE ($e_{\text{align}}$, m):** Frame discrepancy evaluated on co-location ground-truth pairs in overlapping coverage zones.
- **Trajectory ATE RMSE ($e_{\text{pose}}$, m):** Absolute Trajectory Error averaged across all robots.

### 4.2 Secondary Telemetry
- **Trust Yield ($\bar{\theta}$):** Mean server-side trust score across delivered inter-robot loop closures.
- **Inlier Retention & Outlier Rejection:** Inliers accepted, inliers rejected, outliers accepted, outliers filtered.
- **Radio Outage Statistics:** Time fraction in NLoS ($\bar{F}_{\text{NLoS}}$), number of discrete outage episodes, mean and maximum outage durations (s).
- **LoS vs. NLoS Delivery Rates:** Empirical delivery percentages for transmissions initiated in LoS versus NLoS.
- **Queue Starvation:** Maximum deferral count across candidate pool.

### 4.3 Statistical Testing Protocol
1. **Paired Seed-Level Comparisons:** Within each seed and team size, pairwise differences ($\Delta = \text{Metric}_A - \text{Metric}_B$) are computed on identical candidate pools.
2. **Non-Parametric Significance:** Two-tailed Wilcoxon signed-rank test.
3. **Effect Size:** Non-parametric Cliff's delta ($\delta$) with standard interpretation thresholds ($|\delta| < 0.147$ negligible, $< 0.33$ small, $< 0.474$ medium, $\ge 0.474$ large).
4. **Bootstrap Confidence Intervals:** 95% bias-corrected and accelerated (BCa) or percentile bootstrap with 10,000 resamples.
5. **Multiple Comparison Correction:** Holm-Bonferroni step-down procedure across the 4 team sizes.
