# BACS: Scheduling Inter-Robot Constraints for Multi-Robot Map Fusion over Duty-Cycle-Limited LoRa Links

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-green.svg)](https://www.python.org/)

This repository contains the implementation, simulation datasets, analysis scripts and hardware experimental materials for **BACS (Bandwidth-Aware Constraint Scheduling)** and **BACS+**, described in the manuscript *"Scheduling Inter-Robot Constraints for Multi-Robot Map Fusion over Duty-Cycle-Limited LoRa Links"* (under review).

---

## 1. Research overview

Collaborative SLAM lets several robots build one shared map. Beyond Wi-Fi coverage, the inter-robot constraints that align their maps must travel over low-power links such as LoRa, whose regulatory duty cycle (1% in the EU 868 MHz band) allows only a few packets per minute. Candidate constraints therefore wait in a queue for tens of seconds to minutes, and the order in which a robot transmits them becomes part of the estimator.

BACS is a transmitter-side scheduler that predicts each candidate's arrival age from the airtime queue, uses predicted server-side trust as an admission gate, and ranks admitted candidates by surrogate information weighted by predicted freshness per unit airtime. BACS+ adds a pairwise coverage term. The receiving side is EMRMF trust-weighted map fusion.

The research framework combines the following components:

| Component | What it provides | Where |
|---|---|---|
| **Simulation-based multi-robot SLAM evaluation** | Two to five robots, trust-weighted pose-graph fusion, LoRa duty-cycle budget and loss models, pre-specified seed blocks | `bacs_sim/`, `config/sim/`, `runs/v15/`, `reviewer_R1_3_geometry_outages/` |
| **Physical two-robot ROS 2 experimental platform** | Two robot endpoints (labelled LIMO-01 and LIMO-02) running ROS 2, with photographs and videos of the setup | `ros2_ws/`, `hardware/` |
| **LoRa-based inter-robot communication** | MKR WAN 1310 boards (Murata CMWX1ZZABZ, SX1276), 868 MHz, SF7, 125 kHz, CR 4/5, 52-byte payload | `hardware/firmware/` |
| **Hardware communication and scheduling experiments** | Link-level session HWS-BACS-001: airtime, delivery, clock agreement and FIFO backlog growth | `hardware/attached_hardware_evidence/session_hws_bacs_001/` |
| **BACS and BACS+ constraint scheduling** | Algorithm 1 of the manuscript and the ROS 2 scheduler node | `bacs_sim/manuscript_algorithm.py`, `algorithms/`, `ros2_ws/src/bacs_scheduler/` |
| **EMRMF trust-weighted map fusion** | Server-side trust factor and pose-graph optimization | `bacs_sim/trust.py`, `bacs_sim/posegraph.py` |

Simulation and hardware are complementary: the simulation measures map accuracy across team sizes and radio conditions, and the hardware platform measures real LoRa packet timing and delivery. **Every result below states whether it was obtained in simulation or on hardware. Map-alignment accuracy is a simulation result; no hardware map-accuracy result is reported.**

---

## 2. Main results

### 2.1 Simulation results (run directory `runs/v15/409d1b1d…`)

Pre-specified tests on unused seeds, 1% duty cycle, two to five robots, seed-level paired Wilcoxon tests (30 seeds per block).

| Result (simulation) | Value | Source |
|---|---|---|
| BACS+ vs EMRMF reference with FIFO transmission (seeds 100–129, L0–L3) | **68.0% lower map-alignment RMSE** (0.757 m → 0.242 m), lower in 30 of 30 seeds, p = 1.9 × 10⁻⁹ | `runs/v15/409d…/comparison.csv`, `bridge/` |
| Runs with alignment RMSE above 0.5 m | 85.4% (EMRMF reference) → 2.7% (BACS+) | `runs/v15/409d…/bridge/` |
| BACS+ vs newest-first (LIFO), seeds 70–99, C0–C3 | Equivalent within ±10% (ratio 1.008, TOST p = 5.1 × 10⁻⁶) | `runs/v15/409d…/confirmation/` |
| Pairwise term (BACS+ vs BACS) with five robots | 11.8% and 16.2% lower error on the two blocks | `runs/v15/409d…/` |
| Random ordering (reference) | 13.0% (seeds 70–99) and 12.4% (seeds 100–129) lower error than BACS+ | `runs/v15/409d…/comparison.csv` |
| Geometry-dependent blockage (seeds 130–159, separate harness) | FIFO 0.771 m; LIFO, BACS+, BACS and random 0.261–0.268 m, statistically indistinguishable | `reviewer_R1_3_geometry_outages/raw_results/` |

The decomposition shows that most of the gain over the EMRMF reference comes from avoiding oldest-first transmission. Full tables are in `RESULTS_SUMMARY.md` and in the Supplementary Material.

### 2.2 Hardware results (session HWS-BACS-001)

Two stationary endpoints, FIFO policy, synthetic candidate stream (one candidate every 2 s), 600 s session.

| Result (hardware) | Value | Evidence |
|---|---|---|
| Packets transmitted / received | 54 / 54 (unique sequence numbers), payloads byte-identical | `transmitter/serial_log.csv`, `receiver/receiver_serial_log.csv` |
| On-air time per packet (board-reported) | 103–104 ms, mean 103.6 ms (model 102.7 ms) | `transmitter/serial_log.csv` |
| Duty cycle used | 0.94% (one packet every 11.0 s) | `transmitter/serial_log.csv`, `session_info.txt` |
| TX/RX inter-packet interval agreement | −1 to +4 ms per packet (mean 1.5 ms) | both serial logs |
| Received signal strength / SNR | −36 to −40 dBm / 9.75–10.75 dB | `receiver/receiver_serial_log.csv` |
| FIFO backlog | Queue 6 → 245 candidates; age of transmitted constraint 10 → 487 s (median 249 s), while the radio path took about 0.12 s | `transmitter/scheduler_decisions.csv` |

See [`hardware/README.md`](hardware/README.md) for the platform, workflow, photographs and videos.

---

## 3. Repository structure

```
.
├── README.md                  # This overview
├── RESULTS_SUMMARY.md         # Verified simulation and hardware results with sources
├── DATA_AVAILABILITY.md       # Data availability statement
├── bacs_sim/                  # Simulator (world, candidates, LoRa, trust, pose graph, BACS/BACS+)
├── config/sim/v15.json        # Configuration of the reported simulation results
├── analysis_plans/            # Pre-specified analysis plans for seeds 70–99 and 100–129
├── runs/v15/409d1b1d…/        # Per-cell results with source hashes (reported results)
├── reviewer_R1_3_geometry_outages/  # Geometry-dependent blockage study (seeds 130–159)
├── paper_results/revision/ranking_v2_c0c3/pilot_tdefer.csv  # Development-seed T_defer calibration
├── results/ablation_study_dev0_9.csv  # Development-seed diagnostic (Supplementary Table)
├── analysis/                  # Hardware log analysis and statistics helpers
├── scripts/                   # Run and analysis scripts
├── tests/                     # Automated tests
├── algorithms/                # Standalone reference implementation of the scheduler
├── ros2_ws/                   # ROS 2 Humble workspace (scheduler, sender/receiver, LIMO and LiDAR drivers)
├── hardware/                  # Hardware platform documentation, logs, firmware, photographs and videos
├── demo/five_robots/          # Five-robot ROS 2 Gazebo demonstration (illustrative)
├── tools/                     # Serial loggers and hardware capture helpers
└── docs/                      # Technical documentation (metrics, hardware experiment design, historical datasets)
```

---

## 4. Reproducing the results

```bash
python -m pip install -r environment/v15-requirements.txt pytest
python -m pytest -q tests

# Simulation statistics from the stored per-cell results
run=runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213
python scripts/confirmation_v15.py "$run"
python scripts/analyze_v15.py "$run"

# All tables, statistics and data figures of Sections 5-6 of the manuscript
python scripts/make_manuscript_tables_figures.py   # -> results/manuscript_v15/

# Full rerun of both pre-specified blocks (about 2.5 h on 12 workers)
python3 scripts/run_v15.py --workers 12

# Geometry-dependent blockage study
python3 reviewer_R1_3_geometry_outages/run_experiment.py --workers 12
python3 reviewer_R1_3_geometry_outages/analyze_results.py
```

`runs/v15/409d…/sources.json` records the SHA-256 of every simulator file used for the reported runs; the files in this repository match those hashes.

---

## 5. Scope

The simulation uses a small planar environment, SE(2) constraints, a single outlier model and an idealized, collision-free channel; the hardware session uses two stationary endpoints and FIFO only and does not measure map accuracy. These and further limitations are listed in Section 7.2 of the manuscript.

## 6. License and citation

MIT License (see `LICENSE`). Citation metadata will be added to `CITATION.cff` after publication.
