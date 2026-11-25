# Results summary

Each result is labelled **simulation** or **hardware**. Simulation results come from run directory `runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213` (configuration `config/sim/v15.json`) unless stated otherwise; tests use the seed as the unit (30 seeds per block, two-sided Wilcoxon signed-rank, Holm correction within families).

## 1. Comparison with the EMRMF reference (simulation, seeds 100–129, L0–L3, N = 2–5)

| Arm | Mean map-alignment RMSE (m) | Runs > 0.5 m | Pose RMSE (m) |
|---|---|---|---|
| EMRMF reference (FIFO, γ = 0.10 s⁻¹) | 0.757 | 85.4% | 0.526 |
| EMRMF, drift-derived γ (FIFO) | 0.733 | 82.3% | 0.515 |
| FIFO, γ_defer | 0.692 | 77.5% | 0.496 |
| Random, γ_defer | 0.212 | 1.7% | 0.435 |
| LIFO, γ_defer | 0.235 | 1.0% | 0.461 |
| BACS | 0.252 | 3.3% | 0.440 |
| BACS+ | 0.242 | 2.7% | 0.431 |

BACS+ vs EMRMF reference: −68.0%, lower in 30 of 30 seeds, p = 1.9 × 10⁻⁹. Decomposition: decay rule under FIFO −8.6%; FIFO → random −69.3%; random → BACS +18.4%; BACS → BACS+ −3.7%; LIFO → BACS+ +3.2% (n.s.).

## 2. Pre-specified confirmation block (simulation, seeds 70–99, C0–C3, N = 2–5)

| Ordering | Mean map-alignment RMSE (m) |
|---|---|
| FIFO | 0.720 |
| LIFO | 0.289 |
| Random | 0.254 |
| BACS | 0.301 |
| BACS+ | 0.292 |

BACS+ vs LIFO: +0.9%, equivalent within ±10% (TOST p = 5.1 × 10⁻⁶). Random ordering: 13.0% lower error than BACS+ (p = 1.8 × 10⁻⁵). BACS+ vs BACS with five robots: −11.8% (p = 0.011).

## 3. Geometry-dependent blockage (simulation, separate harness, seeds 130–159)

C4 rack blockage: FIFO 0.771 m; LIFO 0.261 m; BACS+ 0.261 m; BACS 0.266 m; random 0.268 m (BACS+ vs random −2.9%, p = 0.641). Matched independent loss: BACS+ vs random −15.2% (C4-IID) and −16.6% (C4-IID-AL). The harness uses γ = 0.10 s⁻¹, 600 s sessions and three retries. Source: `reviewer_R1_3_geometry_outages/raw_results/`.

## 4. Link-level LoRa session HWS-BACS-001 (hardware)

54 of 54 packets delivered; 103–104 ms board-reported airtime (model 102.7 ms); 0.94% duty cycle; TX/RX interval agreement −1 to +4 ms; FIFO queue 6 → 245 candidates and constraint age 10 → 487 s while the radio path took about 0.12 s. Source: `hardware/attached_hardware_evidence/session_hws_bacs_001/`.
