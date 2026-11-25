# EMRMF to BACS plus bridge on seeds 100 to 129

## Status and scope

This plan covers the controlled bridge evaluation over fresh seeds 100 through 129, robot counts N = 2, 3, 4, and 5, link conditions L0 through L3, and 480 second sessions. It was retained in the repository before analysis but was not registered in a public registry. All arms must use identical precomputed worlds, trajectories, drift realizations, candidate pools, and channel realizations.

The conditions are L0 with no loss, L1 with 10 percent independent loss, L2 with 30 percent independent loss, and L3 with 30 percent Gilbert-Elliott burst loss plus 0.2 seconds of delay. The radio model is 868 MHz LoRa at SF7, 125 kHz bandwidth, coding rate 4/5, a 52 byte payload, CRC enabled, and a 1 percent duty-cycle ceiling.

## Arms

- `emrmf_published`: FIFO transmission, gamma = 0.10, p = 3, tau_e = 0.5 m, trust floor 0.01, no trust gate, and no observability term.
- `emrmf_drift`: FIFO transmission with drift-derived gamma = 0.0333.
- `fifo_defer`: FIFO transmission with deferral-derived gamma = 0.0045.
- `random`: random ordering with gamma = 0.0045.
- `bacs_gated`: gated information-density ranking with gamma = 0.0045 and trust gate 0.05.
- `plus_0.30_6`: BACS+ with gamma = 0.0045, trust gate 0.05, observability weight 0.30, and observability reference 6.

## Primary endpoint and test

The primary endpoint is map-alignment RMSE. Compare `plus_0.30_6` with `emrmf_published` using a two-sided paired Wilcoxon signed-rank test pooled over all 480 paired cases. Use alpha = 0.05.

## Secondary analyses

- Run paired tests separately for L0 through L3 and apply Holm adjustment across the four condition-specific tests.
- Report the stepwise decomposition from `emrmf_published` to `emrmf_drift`, from `emrmf_drift` to `fifo_defer`, and from `fifo_defer` to `plus_0.30_6`.
- Compare `plus_0.30_6` with `random` as a control comparison.
- Report means, paired differences, confidence intervals, effect sizes, and win, tie, and loss counts without post-hoc exclusions or retuning.

Before execution, record UTC time and SHA-256 hashes for the simulator, scheduler, and experiment-definition files. The run must stop if seeds 100 through 129 occur in any earlier raw result table.
