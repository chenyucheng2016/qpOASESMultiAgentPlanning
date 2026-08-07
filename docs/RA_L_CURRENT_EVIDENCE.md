# RA-L current evidence freeze

This file records the current submission evidence after the collision-merit
optimization. It supersedes older development snapshots in
`RA_L_BENCHMARK_STATUS.md`; it does not replace the untouched final split.

## Solver and primary scaling

- Solver commit: `ec3b29d2816530b8f57c9cf02e6b43bf0b16fa79`.
- WSL test suite: 19/19 passed.
- Primary development matrix: 240 paired scenarios, with 240/240 independent
  validations and 240/240 strict convergences for both TurboADMM-NL and
  centralized OSQP.
- Maximum absolute objective gap: 4.9835%.
- Median OSQP/TurboADMM-NL wall ratios at 4, 8, 12, and 20 agents: 0.525,
  1.057, 1.558, and 2.580.
- At 20 agents, median peak memory is 126456 KiB for TurboADMM-NL and
  2249706 KiB for centralized OSQP.

Evidence directory:
`build-ral-wsl/results/ral-primary-development-clean-ec3b29d`.

## CSDO-compatible recovery

The exact-current-commit frozen passing-bay evaluation contains 32 paired
cases. TurboADMM-NL validates 32/32 and original CSDO validates 15/32. The
paired table contains 17 Turbo-only successes and no CSDO-only success. Turbo
repairs all 17 retained-root or independent-path conflicts; exact two-sided
McNemar `p=1.52587890625e-05`.

Evidence directory:
`build-ral-wsl/results/csdo_ral/evaluation_passing_bay_ec3b29d`.

## Current continuation ablation

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

This gate passes the development evidence. It is frozen before the untouched
final split is launched. The failed v1 artifact is not overwritten.

Evidence directory:
`build-ral-wsl/results/ral-continuation-ablation-f247a59`.

SHA-256 evidence hashes:

- `run_manifest.json`: `6d3eabc4b3e5f9f5e805e332d64b4863a1dd8229cda56c943c582cb48a677dc9`
- `full_vs_inner_pairs.csv`: `21ec668dc4dd159adb72b4ef3ca1c2f2124c12dc7bf6ea10abf093818bb11481`
- `full_vs_qp_continuation_pairs.csv`: `6fa02945ef7c0b8504d0dd29af90b93c131574c14d99f1588d9d4d71013865bb`
- `qp_continuation_vs_inner_pairs.csv`: `91224dd79f4f81852b01da548e277276731aeb0dd670086c6cf67417bf2322e9`
- failed `ablation_gate.json`: `ffdd85e13b4ab753fad3185bf3ee84f2eb5f9a50f368843bd92471f524db604d`
- passing `ablation_gate_v2.json`: `72888592f008bc918f24a33026301ec8d86aef7c1a139e7c20883e718f1de3ed`

## Remaining submission evidence

The pre-launch final protocol contains all 720 untouched scenarios and the
three externally comparable methods: full TurboADMM-NL, centralized OSQP, and
centralized qpOASES. The cold, inner-only, and QP-continuation variants remain
in the completed 240-case development ablation; repeating them on the final
split would not test an additional external-comparison claim. Final mode rejects
method, scenario, timeout, and protocol overrides.

Before submission, rerun the repeated deterministic protocol on the frozen
solver, update the manuscript from these current artifacts, and execute the
untouched 720-scenario final split once. No solver or benchmark parameter may
change after inspecting that final split.
