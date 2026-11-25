# Reviewer 1.3: Geometry-Dependent Radio Outages & Spatial Radio Adaptation

This directory contains the complete, self-contained implementation, experimental data, analysis scripts, figures, tables, and reviewer response addressing **Reviewer 1.3** for the *Journal of King Saud University – Computer and Information Sciences* (Submission ID: `ff2f82e9-99ff-4e72-8582-384551789f9b`).

---

## 1. Directory Structure

```text
reviewer_R1_3_geometry_outages/
├── README.md                          # Replication and directory documentation
├── analysis_plan.md                   # Pre-registered immutable experimental protocol
├── experiment_config.yaml             # Experiment and arena parameters
├── source_changes.md                  # Non-interference & hash verification
├── geometry_model.py                  # 2D obstacle ray-clipping & outage statistics
├── channel.py                         # Physical LoRa channels (C4 Geometry & C4-IID Matched)
├── runner.py                          # Simulation execution engine
├── run_experiment.py                  # Parallel experiment runner (multiprocessing)
├── analyze_results.py                 # Statistical analysis & Supplementary Tables S1-S4
├── plot_figures.py                    # Publication figure generator (Figures S1-S4)
├── response_R1_3.md                   # Point-by-point formal response to Reviewer 1.3
├── supplementary_material.md          # Supplementary Section S.4 text & methodology
├── manuscript_additions.md            # Exact text diffs for main manuscript
├── verification_report.md             # Scientific compliance & reproducibility report
├── manifest_sha256.txt                # Cryptographic checksum manifest
├── tests/
│   └── test_geometry_outages.py       # 12 unit tests for ray clipping & channel physics
├── raw_results/                       # Per-run CSV datasets (C4, C4-IID, C0, all_runs)
├── processed_results/                 # Intermediate statistical summaries
├── tables/                            # Supplementary Tables S1, S2, S3, S4 (Markdown & CSV)
└── figures/                           # Publication Figures S1, S2, S3, S4 (PNG & PDF)
```

---

## 2. Experimental Parameters & Conditions

- **Arena:** $14.0\,\text{m} \times 10.5\,\text{m}$ rectangular warehouse.
- **Server Antenna:** $(7.0\,\text{m}, 10.5\,\text{m})$ (northern perimeter ceiling).
- **Obstacles:** Six rectangular storage racks ($3.0\,\text{m} \times 1.5\,\text{m}$ each) in two tiers of three aisles.
- **Conditions:**
  - **C4 (Geometry-Dependent Outages):** 2D Liang-Barsky ray clipping.
    - LoS (Unobstructed): $P_{\text{deliv}}^{\text{LoS}} = 0.95$ ($5\%$ drop).
    - NLoS (Obstructed): $P_{\text{deliv}}^{\text{NLoS}} = 0.10$ ($90\%$ drop).
  - **C4-IID (Matched Independent Loss):** Uniform Bernoulli loss matched to the trajectory-average loss of C4 ($P_{\text{drop}} = 0.2615$).
  - **C0 (Unobstructed Reference):** Baseline with zero packet loss ($P_{\text{drop}} = 0.00$).
- **Seed Block:** Pristine seeds **130–159** (30 seeds).
- **Robot Team Sizes:** $N \in \{2, 3, 4, 5\}$.
- **Evaluated Policies:** `FIFO`, `Random`, `LIFO`, `BACS` (`bacs_gated`), `BACS+` (`bacs_plus`).
- **Physical Radio Constraints:** LoRa SF7/BW 125 kHz, 52-byte payload, 1% duty cycle enforced on transmission airtime ($T_{\text{air}} \approx 103\,\text{ms}$), up to 3 retries with ACK timeouts ($0.5\,\text{s}$).

---

## 3. Quick Replication Instructions

### Run Unit Tests
```bash
python3 -m pytest reviewer_R1_3_geometry_outages/tests/test_geometry_outages.py -v
```

### Run Factorial Simulation Sweep
```bash
python3 reviewer_R1_3_geometry_outages/run_experiment.py --workers 12
```

### Generate Statistical Analysis & Supplementary Tables
```bash
python3 reviewer_R1_3_geometry_outages/analyze_results.py
```

### Generate Publication Figures (S1–S4)
```bash
python3 reviewer_R1_3_geometry_outages/plot_figures.py
```

---

## 4. Double-Anonymous Review Integrity
In compliance with journal policies, all identifying author, laboratory, institution, or project metadata has been omitted.
