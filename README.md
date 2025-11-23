## 🚀 Multi-Agent ADMM Performance

The Riccati warm start feature provides significant performance gains for multi-agent planning scenarios using the TurboADMM solver. The following tables compare the performance with and without this feature for 2, 4, and 6-agent collision avoidance problems.

### Standard ADMM (without Riccati Warm Start)

| Scenario | ADMM Iterations | Total QP Iterations | Solve Time | Converged |
|:---------|:---------------:|:-------------------:|:-----------|:---------:|
| 2-agent  | 2               | 86                  | 23.22 ms   | YES       |
| 4-agent  | 6               | 184                 | 52.04 ms   | YES       |
| 6-agent  | 24              | 344                 | 99.40 ms   | YES       |

*All tests converged with primal and dual residuals at 0.000000.*

### ADMM with Riccati Warm Start

| Scenario | ADMM Iterations | Total QP Iterations | Solve Time | Converged |
|:---------|:---------------:|:-------------------:|:-----------|:---------:|
| 2-agent  | 2               | 4                   | 6.37 ms    | YES       |
| 4-agent  | 6               | 24                  | 15.75 ms   | YES       |
| 6-agent  | 24              | 104                 | 38.49 ms   | YES       |

*All tests converged with primal and dual residuals at 0.000000.*

### Performance Gains

The Riccati warm start dramatically reduces the number of QP iterations required for convergence, leading to a significant reduction in total solve time.

| Scenario | QP Iteration Reduction | Solve Time Reduction |
|:---------|:----------------------:|:--------------------:|
| 2-agent  | **95.3%** (86 → 4)     | **72.6%** (23ms → 6ms) |
| 4-agent  | **87.0%** (184 → 24)   | **69.7%** (52ms → 16ms)|
| 6-agent  | **69.8%** (344 → 104)  | **61.3%** (99ms → 38ms)  |

This demonstrates the effectiveness of using a high-quality primal-dual warm start for complex, coupled multi-agent MPC problems.
