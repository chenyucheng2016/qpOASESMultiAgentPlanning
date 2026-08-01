# TurboADMM-NL development benchmark status

This is development evidence collected on 2026-08-01. It is not the locked
30-seed final-paper experiment. The WSL build used GCC 9.4, OpenMP, and OSQP
1.x from `/usr/local/lib/libosqp.so`. Every scenario-method process had the
same 120 second wall-time cap.

## Revision 3 current verification

The current OSQP-enabled WSL Release build passes all 11 registered tests.
The final post-tuning rerun of the seven-case `full` deterministic gate passed
in 42.17 seconds. All seven solver trajectories pass the independent validator;
five terminate with strict SCP convergence and two terminate at the SCP limit
with feasible trajectories. The largest terminal error is 1.9365 m against
the frozen 2.5 m limit. Minimum reported obstacle clearance remains above the
frozen -0.01 m tolerance in every case.

The frozen automatic OpenMP policy is
`min(n, max(4, ceil(n/2)), OpenMP maximum)`. It selects 2, 4, 4, 7, and 10
threads for `n={2,4,8,14,20}`. Residual-balanced rho adaptation is enabled
only for `n>=8`; the two eight-agent deterministic cases use 4 threads and
complete in 9.24 s and 20.67 s.

The following matched WSL measurements are single-run development evidence,
not the required ten-repetition final timing table:

| Scenario | TurboADMM-NL | Centralized OSQP | OSQP / Turbo |
|---|---:|---:|---:|
| easy open | 0.349 s | 0.224 s | 0.64x |
| easy blocker | 0.424 s | 0.596 s | 1.40x |
| medium doorway | 1.164 s | 15.875 s | 13.64x |
| medium heterogeneous open | 4.233 s | 2.507 s | 0.59x |
| hard heterogeneous doorway | 6.051 s | 47.599 s | 7.87x |
| hard warehouse | 9.239 s | timeout (>120 s) | >12.99x |
| very hard maze | 20.672 s | timeout (>120 s) | >5.81x |

The performance crossover is now visible, but objective quality is not yet
uniform. In particular, the medium doorway objective is 670.0 for the
50-round TurboADMM-NL gate versus 325.8 for OSQP, because every inner ADMM
subproblem reaches its iteration cap. A 100-round development ablation closes
that gap (329.6) and remains 6.3x faster, but it fails the heterogeneous hard
doorway and is therefore rejected from revision 3. This quality limitation
must be reported or resolved before a final paper claim.

Revision-3 matched OSQP rows and timeout records are under
`build-ral-wsl/results/frozen_osqp/`. Repeated-run protocol smoke evidence is
under `build-ral-wsl/results/protocol_smoke/`.

## Revision 2 archived evidence

The current OSQP-enabled WSL build passes all 11 registered tests in 142.43
seconds. This includes the seven-case deterministic `full` gate (118.18 s),
terminal-accuracy regression, static polygons, heterogeneous models, and the
centralized qpOASES regression. The native build also passes 11/11 (81.66 s).

The WSL constant-density crossover pilot uses seed 1000 and the same generated
scenario, tolerances, compiler/runtime, and timeout for every method:

| Method | n=8, m=8 | n=20, m=32 |
|---|---:|---:|
| cold | 8.84 s | 10.67 s |
| inner | 2.41 s | 2.95 s |
| qp_continuation | 2.19 s | 3.14 s |
| full | 1.41 s | 2.87 s |
| centralized_qpoases | 91.94 s | timeout (>120 s) |
| centralized_osqp | 2.48 s | 16.11 s |

All 11 completed runs passed both the solver and independent validator. The
active graph stayed local: 4/28 pairs at n=8 and 10/190 pairs at n=20, with
maximum degree 1. Each agent saw at most 2/8 or 4/32 obstacles. Maximum local
QP variables stayed at 373, while the centralized QP grew from 1,865 variables
and 12,478 constraints to 4,849 variables and 35,802 constraints. Thus `full`
is 1.76x and 5.62x faster than centralized OSQP at the two scales.

These are development results, not the locked 30-seed paper experiment. Raw
rows, timeout records, logs, and generated summaries are under

## Verification

Both final builds passed all ten registered tests:

- native MinGW: 10/10, 129.95 seconds total; seven-case gate 122.73 seconds;
- WSL with OSQP: 10/10, 285.60 seconds total; seven-case gate 268.95 seconds.

The suite includes a scenario-index regression and an easy-case centralized
qpOASES regression that prevents reuse of an unverified coupled primal guess.

## Deterministic matrix

All 42 combinations of seven scenarios and six methods were attempted. There
were 39 completed processes and three timeouts.

| Method | Attempted | Completed | Timeouts | Protocol success |
|---|---:|---:|---:|---:|
| cold | 7 | 6 | 1 | 2/7 |
| inner | 7 | 7 | 0 | 6/7 |
| qp_continuation | 7 | 7 | 0 | 7/7 |
| full | 7 | 7 | 0 | 7/7 |
| centralized_qpoases | 7 | 7 | 0 | 3/7 |
| centralized_osqp | 7 | 5 | 2 | 5/7 |

The timeouts were `medium_heterogeneous_open/cold`,
`hard_warehouse/centralized_osqp`, and
`very_hard_maze/centralized_osqp`. After removing an infeasible Riccati primal
guess from the coupled qpOASES initialization, that baseline passes the three
unicycle cases through `medium_doorway`; its four harder completed runs remain
QP failures.

## Monte Carlo smoke matrix

The smoke suite contains 24 fixed-seed scenarios spanning `n={2,4}`,
`m={0,4}`, unicycle and balanced heterogeneous compositions, and seeds 0-2.
All 144 scenario-method combinations were attempted: 141 completed and three
cold runs timed out.

| Method | Attempted | Completed | Timeouts | Protocol success |
|---|---:|---:|---:|---:|
| cold | 24 | 21 | 3 | 9/24 |
| inner | 24 | 24 | 0 | 23/24 |
| qp_continuation | 24 | 24 | 0 | 24/24 |
| full | 24 | 24 | 0 | 23/24 |
| centralized_qpoases | 24 | 24 | 0 | 15/24 |
| centralized_osqp | 24 | 24 | 0 | 24/24 |

The full-method miss was balanced `n=4`, `m=4`, seed 2. Collision checks
passed (minimum pairwise clearance 3.182 m and obstacle clearance -0.00111 m
under a -0.01 m tolerance), but terminal error was 2.925 m under the frozen
2.5 m limit. This is recorded as a terminal failure. QP continuation passed
that run, so it currently has the strongest smoke success rate.

On common successful smoke cases, paired medians show that `full` reduces
working-set recalculations by 87.9 percent versus `inner` and 22.6 percent
versus `qp_continuation`. Median wall time is 6.6 percent slower than `inner`
and 2.4 percent faster than `qp_continuation`; median objective changes are
within 0.4 percent. The work-reduction target is met, but the novelty gate is
not yet passed because `full` has one more smoke failure than
`qp_continuation`.

Raw run rows, timeout records, per-run logs, and generated summaries are in:

- `build-ral-wsl/results/manual_matrix/`;
- `build-ral-wsl/results/smoke_matrix/`.

Use `results_corrected.csv`, `execution_status_corrected.csv`, and the
`*_corrected.csv` summaries in those directories. The corrected qpOASES rows
and their individual logs are preserved under `manual_qpoases_fixed/` and
`smoke_qpoases_fixed/`; the original pre-fix rows remain available for audit.

These development results do not yet satisfy the paper novelty gate because
`full` has a worse smoke success rate than `qp_continuation`. Terminal
robustness must be improved on the development split. The locked development
and final matrices must not be launched until success parity and the measured
work reduction are both confirmed.
