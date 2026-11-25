# Source Changes & Non-Interference Verification: Reviewer 1.3

**Target Revision:** Journal of King Saud University – Computer and Information Sciences  
**Submission ID:** `ff2f82e9-99ff-4e72-8582-384551789f9b`  
**Date:** November 2026  

---

## 1. Absolute Non-Interference Mandate

To preserve scientific reproducibility and comply with the journal audit requirements:
1. **Zero Modifications to Frozen Code:** None of the core algorithm or simulation files in `bacs_sim/` or configuration files in `config/sim/` have been modified, edited, or deleted.
2. **Untouched Baseline Results:** The 960 canonical evaluation cells and existing ablations remain completely untouched.
3. **Pristine Seed Block:** All experiments for Reviewer 1.3 are restricted to pristine seeds **130–159**, which are completely disjoint from the main manuscript seeds (0–29 and 30–59) and other reviewer evaluations.
4. **Isolated Directory Architecture:** All new models, simulation execution scripts, analysis routines, tables, and figures reside strictly within `reviewer_R1_3_geometry_outages/`.

---

## 2. Frozen File Hash Audit

Every tracked core file in `bacs_sim/` matches its baseline SHA256 checksum byte-for-byte:

| File Path | SHA256 Checksum | Baseline Status |
| :--- | :--- | :--- |
| `bacs_sim/world.py` | `05b191c1fffa9b47e834ebcb52eaee58739d4e5d62551ecb6a957fe1a8a0a9ff` | UNTOUCHED / VERIFIED |
| `bacs_sim/simulator.py` | `6ae1eecaeaa6560bb236714d5e94ea4e8574c86ecf1304543eb3fa6dcbf7cf71` | UNTOUCHED / VERIFIED |
| `bacs_sim/manuscript_algorithm.py` | `5c6ebca4640d2e8be668a6230f2f354c0b48455648ea92a95f9c4ea8074d2843` | UNTOUCHED / VERIFIED |
| `bacs_sim/schedulers.py` | `cf6dc4254bb4e334a1796cf9387431c3bf79bfca592bb37fbef7b9ba5e1d7108` | UNTOUCHED / VERIFIED |
| `bacs_sim/lora.py` | `da43c49a0a03bb6c0c29f2dcce112c33df86a7d6ff5be9154f85e3cbdf0cba33` | UNTOUCHED / VERIFIED |
| `bacs_sim/config.py` | `432db5df36c568f635f11e9f1a21a525996e3fc9ee0c63fa5cb78e47f9f6e6fb` | UNTOUCHED / VERIFIED |
| `config/sim/v15.json` | `d6e1564f33190be2d6a56fbc66ba84ff1355d0458dfabeb42ff00366ebce1b25` | UNTOUCHED / VERIFIED |

---

## 3. New Modules Created in `reviewer_R1_3_geometry_outages/`

All implementation artifacts for Reviewer 1.3 are self-contained in this directory:

- `geometry_model.py`:
  - Definition of `RectObstacle` and warehouse storage rack layouts.
  - Parametric Liang-Barsky 2D line segment clipping against axis-aligned rectangles.
  - `check_link_blocked`: Ray-tracing from robot position to server antenna $(7.0, 10.5)$.
  - `compute_outage_statistics`: Exact tracking of blockage fractions, discrete outage episode counts, and mean/max outage durations.
- `channel.py`:
  - `GeometryLoRaChannel`: Dynamic physical LoS/NLoS channel driven by robot position at the exact transmission instant. Emulates physical LoRa airtime charging for every transmission attempt including failures.
  - `MatchedIIDChannel`: Memoryless Bernoulli channel matched to the exact trajectory-average loss rate ($\bar{P}_{\text{drop}} = 0.2615$).
- `runner.py`:
  - Clean simulation wrapper integrating `bacs_sim` modular components (`build_world`, `generate_candidates`, `select_window`, `PoseGraph`, `precompute`) without modifying any frozen files.
- `tests/test_geometry_outages.py`:
  - 12 comprehensive unit tests covering ray-obstacle intersections, boundary conditions, outage statistics, channel delivery rates, airtime charging on retries, candidate pool isolation, and end-to-end execution.
- `run_experiment.py`:
  - Multiprocessing execution engine running the full factorial experiment grid (1,200 runs across C4 and C4-IID, plus C0 baseline).
- `analyze_results.py`:
  - Statistical analysis engine computing paired Wilcoxon tests, bootstrap 95% CIs, Cliff's delta, and Holm-adjusted $p$-values, generating Supplementary Tables S1–S4.
