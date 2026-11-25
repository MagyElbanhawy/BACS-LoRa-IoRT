# Three-Way Factorial Comparison: C4 vs. C4-IID vs. C4-IID-AL Sensitivity

Evaluated over 30 pristine seeds (130–159). Mean across all team sizes (N = 2, 3, 4, 5).

| Policy | Condition | Attempt Delivery (%) | Packet Delivery (%) | Unique Packets Delivered | Mean Trust (theta) | High-Trust Delivery (%) | Map Alignment RMSE (m) | Trajectory ATE RMSE (m) | Airtime Util (%) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| BACS+ | C4 (Obstacle Blockage) | 60.6% | 84.8% | 29.8 | 0.519 | 72.6% | 0.261 | 0.466 | 99.5% |
| BACS+ | C4-IID (Matched Traj P_drop=0.2615) | 72.9% | 98.4% | 34.6 | 0.483 | 70.8% | 0.219 | 0.440 | 93.8% |
| BACS+ | C4-IID-AL (Sensitivity P_drop=0.3739) | 61.3% | 94.9% | 33.4 | 0.474 | 70.1% | 0.224 | 0.441 | 107.4% |
| BACS | C4 (Obstacle Blockage) | 60.6% | 84.8% | 29.8 | 0.521 | 72.8% | 0.266 | 0.466 | 99.7% |
| BACS | C4-IID (Matched Traj P_drop=0.2615) | 72.9% | 98.4% | 34.6 | 0.488 | 71.5% | 0.218 | 0.448 | 93.8% |
| BACS | C4-IID-AL (Sensitivity P_drop=0.3739) | 61.3% | 94.9% | 33.3 | 0.481 | 70.9% | 0.224 | 0.449 | 107.5% |
| Random | C4 (Obstacle Blockage) | 63.9% | 86.5% | 34.6 | 0.027 | 5.2% | 0.268 | 0.489 | 101.2% |
| Random | C4-IID (Matched Traj P_drop=0.2615) | 74.1% | 98.3% | 39.3 | 0.033 | 6.5% | 0.258 | 0.466 | 99.2% |
| Random | C4-IID-AL (Sensitivity P_drop=0.3739) | 62.9% | 94.7% | 37.9 | 0.033 | 6.6% | 0.268 | 0.459 | 112.6% |
| LIFO | C4 (Obstacle Blockage) | 63.8% | 86.6% | 34.6 | 0.436 | 60.2% | 0.261 | 0.475 | 101.6% |
| LIFO | C4-IID (Matched Traj P_drop=0.2615) | 72.7% | 98.5% | 39.4 | 0.426 | 61.6% | 0.232 | 0.432 | 101.3% |
| LIFO | C4-IID-AL (Sensitivity P_drop=0.3739) | 61.3% | 94.8% | 37.9 | 0.420 | 61.3% | 0.237 | 0.433 | 115.5% |
| FIFO | C4 (Obstacle Blockage) | 63.8% | 86.6% | 34.6 | 0.010 | 0.0% | 0.771 | 0.524 | 101.6% |
| FIFO | C4-IID (Matched Traj P_drop=0.2615) | 72.7% | 98.5% | 39.4 | 0.010 | 0.0% | 0.760 | 0.521 | 101.3% |
| FIFO | C4-IID-AL (Sensitivity P_drop=0.3739) | 61.3% | 94.8% | 37.9 | 0.010 | 0.0% | 0.773 | 0.525 | 115.6% |