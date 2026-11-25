# Final confirmation on seeds 70 to 99

## Status and scope

This plan covers the fresh-seed confirmation run over seeds 70 through 99, robot counts N = 2, 3, 4, and 5, conditions C0 through C3, and 480 second sessions. It was retained in the repository before analysis but was not registered in a public registry.

The final treatment arm is selected mechanically from the preceding residual-gate test: use BACS plus residual gating only if the residual-gate success rule is met; otherwise use the frozen BACS+ configuration `plus_0.30_6`. The comparison arms are FIFO, random, BACS, and frozen BACS+, with the selected final arm added if distinct.

## Primary endpoint and test

The primary endpoint is map-alignment RMSE. Compare the selected final arm with random selection using a two-sided paired Wilcoxon signed-rank test pooled over C0 through C3, N = 2 through 5, and seeds 70 through 99, for 480 paired cases. The result is confirmed only when p < 0.05 and the final arm has lower mean RMSE. Report the sign of the mean paired difference separately for each condition.

## Secondary and equivalence analyses

- Compare the final arm with FIFO and BACS on map-alignment RMSE by condition, using Holm adjustment within the secondary alignment family.
- If the residual-gated arm is selected, also compare it with frozen BACS+ in that family.
- Compare the final arm with random selection on pose RMSE by condition and pooled, using Holm adjustment within the secondary pose family.
- Report paired equivalence of final versus random on the log RMSE ratio using two one-sided t-tests at alpha 0.05, a 10 percent equivalence margin, and a 90 percent confidence interval for the geometric-mean ratio.

No seed or run may be excluded after inspection of the outcome. The run must stop if seeds 70 through 99 already occur in an earlier result table or if the frozen-input hashes do not match the values recorded before execution.
