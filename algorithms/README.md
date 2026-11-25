# BACS and Baseline Scheduling Algorithms

This directory provides the formal mathematical formulations, algorithmic pseudocode, complexity analysis, and executable reference implementations for **BACS**, **BACS+**, and the comparative baseline policies evaluated in the manuscript.

---

## 1. Problem Formulation

Multi-robot SLAM in communication-degraded environments requires robot agents to identify, prioritize, and transmit loop-closure constraints over severely bandwidth-limited radio links (such as LoRa 868/915 MHz). Under European Telecommunications Standards Institute (ETSI) regulations, transmissions in the sub-GHz unlicensed ISM bands are constrained by a statutory duty cycle:
$$\delta \le 1.0\% \quad (\text{i.e., } 36\text{ seconds of airtime per hour})$$

Within any sliding window of duration $W$ (typically $W = 1.0\text{ s}$ to $10.0\text{ s}$), the usable transmission airtime budget $B_i$ allocated to robot $i \in \{1, \dots, N\}$ is bounded by:
$$\sum_{z \in \mathcal{S}_i} \tau(z) \le B_i \le \alpha_i \cdot \delta \cdot W$$
where $\tau(z)$ is the physical transmission airtime of constraint packet $z$, and $\alpha_i \in (0, 1]$ represents the sub-band sharing allocation across $N$ contending robots ($\sum_{i=1}^N \alpha_i \le 1$).

---

## 2. Mathematical Formulations

### 2.1 BACS Priority Metric

BACS formulates constraint selection as a constrained knapsack problem over an active candidate buffer $\mathcal{C}_i$. The priority $P(z)$ of prospective constraint $z$ is defined as the marginal Fisher information surrogate $\Delta \mathcal{I}(z)$ per unit transmission airtime $\tau(z)$, discounted by an exponential age-of-information (AoI) penalty:

$$P_{\text{BACS}}(z) = \frac{\Delta \mathcal{I}(z)}{\tau(z)} \cdot \exp\left(-\lambda \cdot \hat{\Delta t}_{\text{arr}}(z)\right)$$

where:
1. **Marginal Information Surrogate $\Delta \mathcal{I}(z)$**:
   $$\Delta \mathcal{I}(z) = w_{\text{nov}} \cdot \text{Nov}(z) + w_{\text{deg}} \cdot (1 - \bar{d}(z)) + w_{\text{span}} \cdot \bar{L}(z)$$
   - $\text{Nov}(z) \in [0, 1]$: Spatial novelty of the loop closure relative to existing keyframes.
   - $\bar{d}(z) \in [0, 1]$: Normalized local factor graph node degree (prioritizing sparser graph regions).
   - $\bar{L}(z) \in [0, 1]$: Normalized loop span (temporal trajectory separation between constrained poses).
2. **Predicted Arrival Age $\hat{\Delta t}_{\text{arr}}(z)$**:
   $$\hat{\Delta t}_{\text{arr}}(z) = (t_{\text{now}} - t_{\text{gen}}) + \lfloor \frac{\tau_{\text{ahead}} + \tau(z)}{B_i} \rfloor \cdot W + \Delta \tau_{\text{intra}} + \tau(z) + \frac{P_{\text{drop}}}{1 - P_{\text{drop}}} (\tau(z) + T_{\text{ACK}})$$
3. **Transmission Airtime $\tau(z)$**: Calculated directly via Semtech SX1262 LoRa physical layer equations (see `algorithms/lora_timing.py`).

---

### 2.2 BACS+ Adaptive Enhancements

BACS+ extends standard BACS with two mechanisms designed for multi-robot scalability and severe channel shadowing:

1. **Pairwise Observational Diversity Scoring**:
   $$\Delta \mathcal{I}_{\text{BACS+}}(z) = \Delta \mathcal{I}(z) + w_{\text{obs}} \cdot \exp\left(-\frac{N_{\text{deliv}}(i, j)}{N_{\text{ref}}}\right)$$
   where $N_{\text{deliv}}(i, j)$ represents the cumulative number of constraints already successfully delivered between robot pair $(i, j)$, and $N_{\text{ref}} = 6.0$. This prevents bandwidth monopolization between a single pair of close robots.
2. **Dynamic Geometric and Trust Gating**:
   Prospective constraints are evaluated against an expected spatial alignment trust metric:
   $$\hat{T}(z) = \max\left(0, 1 - \left(\frac{r(z)}{\tau_e}\right)^p\right) \cdot \exp\left(-\gamma \cdot \hat{\Delta t}_{\text{arr}}(z)\right)$$
   Candidates are admitted to the transmission candidate pool only if:
   $$\hat{T}(z) \ge \gamma_{\text{gate}}$$
   where $\gamma_{\text{gate}}$ adapts dynamically to trajectory stability and channel outage episodes.

---

### 2.3 Baseline Schedulers

| Algorithm | Selection Criterion | Knapsack Admission Rule |
| :--- | :--- | :--- |
| **FIFO** | $\arg\min_{z} t_{\text{gen}}(z)$ | First-In First-Out by creation timestamp until $\sum \tau(z) > B_i$. |
| **LIFO** | $\arg\max_{z} t_{\text{gen}}(z)$ | Last-In First-Out (newest measurements first) until $\sum \tau(z) > B_i$. |
| **Random** | Uniform random shuffle | Random selection without replacement until $\sum \tau(z) > B_i$. |
| **BACS** | $\arg\max_{z} P_{\text{BACS}}(z)$ | Descending information density until $\sum \tau(z) > B_i$. |
| **BACS+** | $\arg\max_{z} P_{\text{BACS+}}(z)$ subject to $\hat{T}(z) \ge \gamma_{\text{gate}}$ | Descending diverse information density until $\sum \tau(z) > B_i$. |

---

## 3. Algorithmic Complexity

For a candidate queue of size $K$ ($K \le 50$ in typical embedded robot configurations):
- **Candidate Evaluation:** Computing surrogate metrics and arrival age is $O(1)$ per candidate $\implies O(K)$ total.
- **Sorting / Priority Queue:** Two-pass priority queue ranking requires $O(K \log K)$ comparisons.
- **Knapsack Admission:** Linear scan over ranked candidates is $O(K)$ in the worst case.
- **Total Time Complexity:** $\mathcal{O}(K \log K)$ per scheduling window.
- **Space Complexity:** $\mathcal{O}(K)$ for the candidate buffer.

Per-window run time on robot hardware has not been benchmarked in the current release.

---

## 4. Manuscript Equation Traceability Matrix

| Manuscript Section / Equation | Mathematical Concept | Source Code Location |
| :--- | :--- | :--- |
| **Section 3.1, Eq. (1)** | Regulatory duty-cycle airtime budget | `bacs_sim/lora.py:regulatory_budget` |
| **Section 3.1, Eq. (2)** | Shared vs per-device channel budget | `bacs_sim/lora.py:airtime_budget` |
| **Section 3.2, Eqs. (3)–(4)** | LoRa Time-on-Air (ToA) calculation | `algorithms/lora_timing.py:lora_time_on_air` |
| **Section 3.3, Eq. (5)** | Arrival age prediction with ARQ | `algorithms/bacs_core.py:predict_arrival_age` |
| **Section 3.3, Eq. (6)** | Spatial-temporal trust metric | `algorithms/bacs_core.py:predict_trust_metric` |
| **Section 3.4, Eq. (7)** | Topological information surrogate | `algorithms/bacs_core.py:compute_information_surrogate` |
| **Section 3.4, Eq. (8)** | Pairwise inter-robot coverage term | `bacs_sim/manuscript_algorithm.py:pairwise_score` |
| **Section 4.1, Algorithm 1** | Two-pass BACS/BACS+ knapsack scheduler | `bacs_sim/manuscript_algorithm.py:select_window` |
| **ROS 2 Scheduler Node** | Embedded real-time scheduler node | `ros2_ws/src/bacs_scheduler/bacs_scheduler/scheduler_node_bacs.py` |
