# Data availability

The current public reproducibility package provides the simulation datasets, analysis scripts and available hardware experimental materials.

## Simulation (simulation-based multi-robot SLAM evaluation)

- **Pre-specified confirmation and comparison blocks** (seeds 70–99 and 100–129, N = 2–5, 960 cells): `runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213/`, with `sources.json` (SHA-256 of every simulator file), manifests and one CSV per seed × condition × team size. Configuration: `config/sim/v15.json`. Analysis plans: `analysis_plans/`.
- **Geometry-dependent blockage study** (seeds 130–159, 1,800 runs): `reviewer_R1_3_geometry_outages/` (harness, raw results, tests, tables, figures).
- **Development-seed calibration and diagnostic** (seeds 0–9): `paper_results/revision/ranking_v2_c0c3/pilot_tdefer.csv`, `results/ablation_study_dev0_9.csv`.

## Hardware (physical two-robot ROS 2 platform with LoRa communication)

- **Session HWS-BACS-001**: `hardware/attached_hardware_evidence/session_hws_bacs_001/` — transmitter and receiver serial logs, scheduler decisions, host clock records, manifests and SHA-256 checksums; the receiver segmentation table is in `derived/receiver_segmentation.csv`.
- **Photographs and videos**: `hardware/hardware_media/` (audio-free public copies of the videos; originals are preserved by the authors). Captions are in `hardware/README.md`.
- **Historical hardware datasets**: historical hardware raw records are not included in this release; see `docs/HISTORICAL_DATASETS.md`.

## Manuscript tables and figures

`scripts/make_manuscript_tables_figures.py` regenerates every number, table and data figure of Sections 5 and 6 from the files above and writes them to `results/manuscript_v15/` (`stats.json`, `table_*.csv`, `figures/`).

## Software

Simulator `bacs_sim/`, ROS 2 workspace `ros2_ws/`, firmware `hardware/firmware/`, analysis scripts `scripts/`, tests `tests/`.
