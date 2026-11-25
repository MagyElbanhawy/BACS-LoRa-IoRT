# Supplementary Material: Section S.4 — Geometry-Dependent Radio Outages and Contact Blockage Evaluation

**Supporting Information for:**  
*BACS: Bandwidth-Aware Constraint Scheduling for Scalable Multi-Robot SLAM over Constrained LoRa Networks*  
**Journal:** *Journal of King Saud University – Computer and Information Sciences*  
**Submission ID:** `ff2f82e9-99ff-4e72-8582-384551789f9b`  

---

## S.4.1 Physical Radio Propagation & Obstacle Model

### S.4.1.1 2D Parametric Ray-Obstacle Intersection
In industrial logistics, warehousing, and disaster-response facilities, wireless propagation availability is strongly dictated by physical geometry. When a mobile robot traverses aisles between metallic storage racks, concrete columns, or heavy machinery, the direct propagation path to a fixed infrastructure antenna is intermittently occluded, inducing sharp transitions between Line-of-Sight (LoS) and Non-Line-of-Sight (NLoS) propagation.

To rigorously model these physical dynamics without confounding scheduling effects, we established an explicit 2D ray-obstacle propagation environment in an arena of dimensions $W_{\text{arena}} = 14.0\,\text{m}$ and $H_{\text{arena}} = 10.5\,\text{m}$ (matching the EMRMF spatial envelope). The central fusion server radio antenna is mounted at a fixed coordinate on the northern perimeter ceiling:
$$\mathbf{p}_{\text{server}} = (7.0\,\text{m}, 10.5\,\text{m})$$

Six axis-aligned rectangular storage racks $\mathcal{O}_k = [x_k^{\min}, x_k^{\max}] \times [y_k^{\min}, y_k^{\max}]$ ($k=1, \dots, 6$), each measuring $3.0\,\text{m} \times 1.5\,\text{m}$, are arranged in two tiers of three aisles:
- **North Tier ($y \in [6.5, 8.0]\,\text{m}$):**
  - $\mathcal{O}_1$ (Rack NW): $x \in [1.5, 4.5]\,\text{m}$
  - $\mathcal{O}_2$ (Rack NC): $x \in [5.5, 8.5]\,\text{m}$
  - $\mathcal{O}_3$ (Rack NE): $x \in [9.5, 12.5]\,\text{m}$
- **South Tier ($y \in [2.5, 4.0]\,\text{m}$):**
  - $\mathcal{O}_4$ (Rack SW): $x \in [1.5, 4.5]\,\text{m}$
  - $\mathcal{O}_5$ (Rack SC): $x \in [5.5, 8.5]\,\text{m}$
  - $\mathcal{O}_6$ (Rack SE): $x \in [9.5, 12.5]\,\text{m}$

Longitudinal aisles of width $1.0\,\text{m}$ ($x \in [4.5, 5.5]\,\text{m}$ and $x \in [8.5, 9.5]\,\text{m}$) and perimeter corridors ($y < 2.5\,\text{m}$ and $y > 8.0\,\text{m}$) permit natural multi-robot boustrophedon sweep navigation while creating realistic shadow cones behind the storage racks.

At each transmission attempt at time $t$, transmitting robot $r$ is situated at ground-truth position $\mathbf{p}_r(t) = (x_r(t), y_r(t))$. The line segment connecting $\mathbf{p}_r(t)$ to $\mathbf{p}_{\text{server}}$ is parameterized as:
$$\mathbf{r}(u) = \mathbf{p}_r(t) + u \cdot (\mathbf{p}_{\text{server}} - \mathbf{p}_r(t)), \quad u \in [0, 1]$$

Using the Liang-Barsky parametric line-clipping algorithm, ray-box intersection against each rectangular obstacle $\mathcal{O}_k$ computes entry and exit parameters $u_1, u_2$. If $u_1 \le u_2$ within $[0, 1]$, the propagation ray intersects the obstacle. The instantaneous binary blockage state $\mathbb{I}_{\text{blocked}}(\mathbf{p}_r(t))$ is defined as:
$$\mathbb{I}_{\text{blocked}}(\mathbf{p}_r(t)) = \bigvee_{k=1}^6 \text{Intersects}(\mathbf{p}_r(t), \mathbf{p}_{\text{server}}, \mathcal{O}_k)$$

### S.4.1.2 Stochastic Packet Delivery Under Condition C4
When the direct propagation path is unobstructed ($\mathbb{I}_{\text{blocked}} = 0$), propagation follows quasi-optical LoS conditions with low baseline packet error rate:
$$P_{\text{deliv}}^{\text{LoS}} = 0.95 \quad (P_{\text{drop}} = 0.05)$$

When the ray intersects any storage rack ($\mathbb{I}_{\text{blocked}} = 1$), the link enters an NLoS shadow zone. At sub-GHz frequencies (868 MHz), diffraction and scattering allow limited penetration, but multipath cancellation and obstacle penetration loss severely degrade signal-to-noise ratio (SNR), resulting in deep fading:
$$P_{\text{deliv}}^{\text{NLoS}} = 0.10 \quad (P_{\text{drop}} = 0.90)$$

### S.4.1.3 Physical Airtime Charging & Retransmission Protocol
The physical LoRa transmission model strictly enforces regulatory airtime constraints:
- Spreading Factor: SF7, Bandwidth: 125 kHz, Coding Rate: 4/5, Preamble: 8 symbols.
- Candidate payload size: 52 bytes ($T_{\text{air}} \approx 103.2\,\text{ms}$).
- Regulatory duty cycle: 1% enforced over scheduling window $W = 60.0\,\text{s}$ ($T_{\text{budget}} = 0.60\,\text{s}$ per robot per window).
- Retransmission logic: Up to 3 retries (maximum 4 attempts per packet). For each attempt:
  $$t_{\text{attempt}} = t_{\text{ready}} + a \cdot (T_{\text{air}} + T_{\text{ack}}), \quad a \in \{0, 1, 2, 3\}$$
  Every attempt consumes $T_{\text{air}}$ from the robot's transmission budget regardless of delivery outcome, directly reflecting physical radio duty-cycle regulations.

---

## S.4.2 Matched-Loss Control: Condition C4-IID

To isolate the effect of *spatial and temporal outage correlation* from aggregate packet loss, we established Condition C4-IID. 

Across all robot trajectories across pristine seeds 130–159, the empirical time-averaged obstruction fraction is:
$$\bar{F}_{\text{blocked}} = \frac{1}{N \cdot T_{\text{session}}} \sum_{r=1}^N \int_0^{T_{\text{session}}} \mathbb{I}_{\text{blocked}}(\mathbf{p}_r(t)) \, dt = 0.2489 \quad (24.89\%)$$

Holding nominal LoS delivery ($0.95$) and NLoS delivery ($0.10$) constant, the expected trajectory-average delivery probability is:
$$\bar{P}_{\text{deliv}} = (1 - \bar{F}_{\text{blocked}}) \cdot 0.95 + \bar{F}_{\text{blocked}} \cdot 0.10 = 0.7385$$
The corresponding matched independent packet drop rate is:
$$\bar{P}_{\text{drop}}^{\text{matched}} = 1.0 - \bar{P}_{\text{deliv}} = 0.2615 \quad (26.15\%)$$

Under Condition C4-IID, each transmission attempt experiences an independent Bernoulli drop probability of exactly $0.2615$, independent of robot position or elapsed time.

### S.4.2.1 Trajectory-Average Matching vs. Attempt-Level Realized Delivery
As documented in the audit analysis, the realized attempt-level packet delivery rate in Condition C4 is **$62.61\%$**, whereas in Condition C4-IID it is **$73.09\%$** (closely matching the configured $73.85\%$). 
This divergence occurs because of **retransmission amplification** in contiguous shadow fading:
- In LoS, packets succeed on attempt 0 with $95\%$ probability, generating an average of only $1.05$ attempts per packet.
- In NLoS, packets fail with $90\%$ probability, triggering up to 3 retries and generating an average of $3.44$ attempts per packet.
- As a result, $38.17\%$ of physical transmission attempts occur during NLoS blockage, even though robots spend only $24.81\%$ of mission time in NLoS.
Under memoryless C4-IID, attempts are not clustered in time, resulting in an attempt-level success rate of $73.09\%$ and a unique packet delivery rate of $98.45\%$ (compared to $85.90\%$ in C4). Trajectory-average matching ($73.85\%$) holds underlying physical channel capacity constant across trajectories, while attempt-level dynamics reflect the operational penalty of correlated shadow fading.

---

## S.4.3 Candidate Pool Isolation & Verification Protocol

To prevent candidate generation bias:
1. For each seed $s \in [130, 159]$ and team size $N \in \{2, 3, 4, 5\}$, `precompute(cfg)` generates the multi-robot world trajectory and candidate loop closure pool once.
2. The candidate pool is deep-cloned across all five evaluated scheduling policies (`FIFO`, `Random`, `LIFO`, `BACS`, `BACS+`).
3. Consequently, every policy evaluates an identical candidate sequence with matching relative measurements $\mathbf{z}_{ij}$, generation timestamps $t_{\text{created}}$, and ground-truth inlier/outlier labels.

---

## S.4.4 Detailed Findings & Discussion

### S.4.4.1 Outage Clustering vs. Independent Loss ($H_1$)
Comparing Condition C4 (**Supplementary Table S1**) against Condition C4-IID (**Supplementary Table S2**):
- Under geometry-dependent blockage (C4), map-alignment RMSE is degraded across all policies compared to matched independent loss (C4-IID). For BACS+, alignment RMSE degrades from **$0.219 \pm 0.071\,\text{m}$** (in C4-IID) to **$0.261 \pm 0.093\,\text{m}$** (in C4, a $+19.2\%$ degradation). For BACS, alignment RMSE degrades from **$0.218 \pm 0.072\,\text{m}$** to **$0.266 \pm 0.104\,\text{m}$** ($+22.0\%$).
- In C4-IID, losses are memoryless and distributed across windows. In C4, contiguous blockage episodes (averaging **$54.6\,\text{s}$**, with peak episodes reaching **$216.0\,\text{s}$**) deplete transmission budgets through repeated retransmissions in deep fade. By the time a robot emerges into an aisle, candidate constraints have aged by multiple scheduling windows, incurring severe temporal trust discounting.

### S.4.4.2 Comparative Analysis of Scheduling Policies ($H_2$)
- **FIFO Failure:** FIFO experiences severe performance degradation under geometry outages ($e_{\text{align}} = 0.771 \pm 0.290\,\text{m}$ overall, $0.693 \pm 0.195\,\text{m}$ at $N=5$, starvation of 10.0 deferrals, and inlier acceptance of $0.0\%$). Repeated drops in shadow zones cause head-of-line blocking, starving fresh loop closures. BACS+ achieves approximately 66.1% lower mean alignment RMSE than FIFO ($0.261\,\text{m}$ vs. $0.771\,\text{m}$, $p < 0.001$, Cliff's $\delta \in [-0.996, -0.931]$), and BACS achieves approximately 65.5% lower RMSE ($0.266\,\text{m}$ vs. $0.771\,\text{m}$).
- **Comparison with LIFO:** BACS+ achieves approximately the same reported mean map-alignment RMSE as LIFO ($0.261 \pm 0.093\,\text{m}$ vs. $0.261 \pm 0.075\,\text{m}$, pairwise differences non-significant after Holm adjustment, $p_{\text{adj}} = 1.000$). However, BACS+ achieves approximately 19% higher mean trust yield than LIFO ($0.519 \pm 0.088$ vs. $0.436 \pm 0.128$, a $+19.0\%$ advantage) and a higher inlier acceptance rate ($72.6\%$ vs. $60.2\%$).
- **Performance of Random Scheduling:** Consistent with the principal experimental findings, Random remains a competitive scheduling alternative, and statistically significant superiority of BACS+ over Random is not established under geometry-dependent outages ($0.268 \pm 0.089\,\text{m}$ for Random vs. $0.261 \pm 0.093\,\text{m}$ for BACS+, $p_{\text{adj}} \ge 0.162$ across all $N$). Random's competitive alignment RMSE occurs because uniform sampling across the candidate queue acts as an inherent spatial dispersion mechanism, preventing localized clustering. However, Random delivers stale loop closures whose trust yield is heavily discounted ($\bar{\theta} = 0.027$ vs. $0.519$ for BACS+), receiving negligible weighting in the pose graph backend.
- **Queue Dynamics and Starvation:** BACS and BACS+ exhibit 0.0 window-to-window deferrals because stale candidates whose predicted arrival trust drops below $1.5 \times \theta_{\text{floor}} = 0.015$ are actively pruned from the pending queue. In baseline queues, candidates are not expired based on trust, accumulating deferrals up to the 10-window limit. Under matched independent loss (C4-IID), BACS and BACS+ achieve the lowest alignment error among all policies ($0.218\,\text{m}$ and $0.219\,\text{m}$ vs. $0.258\,\text{m}$ for Random and $0.232\,\text{m}$ for LIFO).

---

## S.4.5 Index of Supplementary Tables & Figures

- **Supplementary Table S1:** Factorial Performance under Condition C4 (Geometry-Dependent Radio Outages) [Markdown / CSV].
- **Supplementary Table S2:** Factorial Performance under Condition C4-IID (Matched Independent Packet Loss) [Markdown / CSV].
- **Supplementary Table S3:** Seed-Level Paired Hypothesis Tests, Cliff's Delta, and Holm-Adjusted $p$-values [Markdown / CSV].
- **Supplementary Table S4:** Outage Telemetry and LoS/NLoS Delivery Breakdown [Markdown / CSV].
- **Supplementary Figure S1:** Warehouse Arena Geometry, Rack Placement, Server Antenna, and Ray Intersections [PNG / PDF].
- **Supplementary Figure S2:** Outage Timeline and Link State Dynamics across 10 Scheduling Windows [PNG / PDF].
- **Supplementary Figure S3:** Map Alignment RMSE Across Communication Conditions (C0, C4-IID, C4) [PNG / PDF].
- **Supplementary Figure S4:** Seed-Level Paired Differences ($\Delta e_{\text{align}}$) for BACS+ vs. Baselines [PNG / PDF].
