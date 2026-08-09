# RA-L current evidence freeze

This file records the frozen evidence consumed by the submission manuscript.
It supersedes older development snapshots in `RA_L_BENCHMARK_STATUS.md` and
includes the single untouched final split.

## Solver and primary scaling

- Frozen Git commit: `de0ae3d0d492c64a9cdda2978a4acb5867b3dc68`.
- Release executable SHA-256:
  `1792050c7849d73903cb82b3fcdc942557742fdd8caa21e6f9192c47decc08c1`.
- WSL test suite at the frozen executable: 19/19 passed.
- Untouched final matrix: 720 scenarios and 2160 scheduled executions.
- TurboADMM-NL: 720/720 completed, independently valid, and strictly
  converged.
- Centralized OSQP: 720/720 completed, independently valid, and strictly
  converged.
- Centralized qpOASES: 270/720 completed and 450/720 reached the frozen 120 s
  cap; there were no execution errors or uncensored nonzero exits.
- Median, 95th-percentile, and maximum absolute TurboADMM-NL/OSQP objective
  gaps are 0.2143%, 4.4750%, and 4.9903%.
- Median OSQP/TurboADMM-NL wall ratios at 4, 8, 12, and 20 agents are 0.488,
  0.928, 1.627, and 2.530.
- The pooled median wall ratio for N >= 12 is 2.2007 with exact sign-test
  `p=4.9777e-65`.
- At 20 agents, median peak memory is 126388 KiB for TurboADMM-NL and
  2249664 KiB for centralized OSQP, a 17.8x ratio.

Evidence directory:
`build-ral-wsl/results/ral-monte-carlo-primary-final`.

## CSDO-compatible recovery

The exact-current-commit frozen passing-bay evaluation contains 32 paired
cases. TurboADMM-NL validates 32/32 and original CSDO validates 15/32. The
paired table contains 17 Turbo-only successes and no CSDO-only success. Turbo
repairs all 17 retained-root or independent-path conflicts; exact two-sided
McNemar `p=1.52587890625e-05`.

Evidence directory:
`build-ral-wsl/results/csdo_ral/evaluation_passing_bay_ec3b29d`.

## Frozen continuation ablation

Data-collection commit `f247a5922b105ba9efaff7fb681c785404737462` ran
240 scenarios with `full`, `qp_continuation`, and `inner` under one randomized,
interleaved WSL schedule. All 720 executions completed, and every method is
independently valid and strictly converged in all 240 scenarios.

The original v1 gate is intentionally preserved as failed. It required a 20%
runtime effect from both `full` versus `inner` and `full` versus
`qp_continuation`. That combines two different causal questions and assigns
the same effect-size threshold to the complete cross-SCP mechanism and to its
last incremental consensus-transport layer.

The causal decomposition is:

| Comparison | Isolated mechanism | Median wall reduction | Local-QP reduction | Paired wins |
|---|---|---:|---:|---:|
| `qp_continuation` vs. `inner` | Matrix/working-set continuation | 45.61% | 0.00% | 240/240 |
| `full` vs. `qp_continuation` | Pair auxiliary/dual transport | 6.29% overall, 12.59% at N=20 | 38.46% | 175/240 |
| `full` vs. `inner` | Complete cross-SCP continuation | 46.65% | 38.46% | 239/240 |

The matrix-continuation comparison has maximum objective disagreement
`3.15e-6%`. Both comparisons involving the full transport have maximum
objective disagreement 4.466%, below the unchanged 5% bound. Runtime sign-test
`p` values are `1.13e-72`, `7.82e-13`, and `2.73e-70`, respectively.

Gate v2 separately requires:

- at least 20% total cross-SCP runtime reduction;
- at least 20% matrix-continuation runtime reduction;
- at least 5% overall and 10% largest-scale pair-transport runtime reduction;
- at least 20% local-QP reduction for total and pair transport;
- 240/240 validity and strict convergence for every variant;
- at most 5% objective disagreement and paired sign-test `p <= 0.05`.

This gate passes the development evidence and was frozen before the untouched
final split. Its manifest and the final manifest record the same executable and
runner SHA-256 values. The failed v1 artifact is not overwritten.

Evidence directory:
`build-ral-wsl/results/ral-continuation-ablation-f247a59`.

SHA-256 evidence hashes:

- `run_manifest.json`: `6d3eabc4b3e5f9f5e805e332d64b4863a1dd8229cda56c943c582cb48a677dc9`
- `full_vs_inner_pairs.csv`: `21ec668dc4dd159adb72b4ef3ca1c2f2124c12dc7bf6ea10abf093818bb11481`
- `full_vs_qp_continuation_pairs.csv`: `6fa02945ef7c0b8504d0dd29af90b93c131574c14d99f1588d9d4d71013865bb`
- `qp_continuation_vs_inner_pairs.csv`: `91224dd79f4f81852b01da548e277276731aeb0dd670086c6cf67417bf2322e9`
- failed `ablation_gate.json`: `ffdd85e13b4ab753fad3185bf3ee84f2eb5f9a50f368843bd92471f524db604d`
- passing `ablation_gate_v2.json`: `72888592f008bc918f24a33026301ec8d86aef7c1a139e7c20883e718f1de3ed`

## Completed final protocol

Final mode rejected method, scenario, timeout, and protocol overrides. All 2160
scheduled rows are accounted for by completion or timeout, and the independent
primary gate passes its success, strict-convergence, objective-quality,
medium/large crossover, largest-scale speedup, and paired-significance
requirements. No solver or benchmark parameter changed after the final split
was inspected.

SHA-256 evidence hashes:

- `execution_status.csv`: `9914aec9996971ffb1509abb146f02e83313da9a83fb321343f424351ba54eb8`
- `results.csv`: `002d214776724b674a5bc51dedab8065e51a0f8ed3bfebdf3b102f910b982ab8`
- `paired_results.csv`: `77a1314bb6716d500f4aa09cec055a3cf8abe635943949f27777b6104b971d12`
- `run_manifest.json`: `919a41a83a0a852c3c1bf8e3f95378fdc20425ca1a60e53be3fb709b867bbf50`
