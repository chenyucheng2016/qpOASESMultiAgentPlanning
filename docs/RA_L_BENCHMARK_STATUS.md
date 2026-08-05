# TurboADMM-NL development benchmark status

This is development evidence collected through 2026-08-05. It is not the locked
30-seed final-paper experiment. The WSL build used GCC 9.4, OpenMP, and OSQP
1.x from `/usr/local/lib/libosqp.so`. The archived repeated timing matrices
below used the same 120 second wall-time cap per scenario-method process.

## Revision 11 representative all-method gate

Eight preselected paper-development instances exercise both compositions at
the smallest and largest scales and the obstacle-dense intermediate scales.
All six methods use the same inventory rows, interleaved schedule, WSL build,
and 120-second process cap. The gate attempted 48 tasks: 43 completed and five
were timeout-censored, with no execution errors.

| Method | Attempted | Completed | Timeouts | Protocol success | Successful median time |
|---|---:|---:|---:|---:|---:|
| TurboADMM-NL (`full`) | 8 | 8 | 0 | 8/8 | 2.750 s |
| centralized OSQP | 8 | 8 | 0 | 8/8 | 6.649 s |
| centralized qpOASES | 8 | 4 | 4 | 4/8 | 9.606 s |
| QP continuation | 8 | 8 | 0 | 8/8 | 3.188 s |
| inner hotstart only | 8 | 8 | 0 | 8/8 | 3.127 s |
| cold ADMM | 8 | 7 | 1 | 7/8 | 13.938 s |

All four qpOASES timeouts occur at `n>=12`; cold ADMM times out on the balanced
`n=20, m=40` case. Turbo and OSQP independently validate on every paired case.
Turbo is slower on two `n=4` cases, but the OSQP/Turbo ratio is 2.23--3.25x on
the sampled `n={12,20}` cases. The largest absolute Turbo objective gap is
`4.162%`. This is a bounded applicability and ablation gate, not a substitute
for the complete 240-instance development matrix.

The manifest records exact commit `ff1bdc66e40d1026ca3029648734021b1a99c454`,
executable SHA-256
`aeb33c65d2af5999663dda2fdb20a5aa61b99059d080d5efba22e0680d6a989d`,
and inventory SHA-256
`4a1945370b6e68455133e6dd293c943c6699642fdb10e1285d162ae88fdd266e`.

## Revision 10 paper-primary protocol gate

The submission-facing primary Monte Carlo protocol is frozen separately from
the legacy breadth matrix. It contains 24 cells from `n={4,8,12,20}`,
`m={0,n,2n}`, homogeneous unicycles and balanced heterogeneous agents, with 10
development or 30 untouched final seeds per cell. Dry-run emits 240 development
and 720 final inventory rows, and the runner hashes the inventory into its
immutable manifest.

The exact-commit one-seed-per-cell WSL pilot at `772e4d5` completed all 48
TurboADMM-NL/centralized-OSQP processes without timeout or execution error. Both
methods independently validate on all 24 cases. The maximum absolute Turbo
objective gap to OSQP is `4.162%`, below the frozen 5% quality gate. Median
OSQP/Turbo solver-time ratios by scale are:

| Agents | Four paired cells plus obstacle variants | Median OSQP / Turbo |
|---:|---:|---:|
| 4 | 6 | 0.73x |
| 8 | 6 | 1.61x |
| 12 | 6 | 1.92x |
| 20 | 6 | 2.41x |

Across valid Turbo rows, maximum terminal error is `0.0506` m, minimum pair
clearance is `-0.000203` m, minimum obstacle clearance is `0.0237` m, and the
dynamics defect is numerical zero. The active interaction degree remains one
and maximum local QP size remains 373 variables, while the centralized problem
reaches 4,849 variables and 44,762 constraints. This is a protocol/crossover
gate, not the final 30-seed statistical result.

## Revision 9 terminal-restoration globalization and CSDO sweep

The candidate shrinks the control trust region after a heavily damped step
only when pair and obstacle constraints are already feasible and terminal or
dynamics restoration remains. The global merit and collision acceptance rules
are unchanged, preserving the established collision-repair behavior. The CSDO
adapter uses a 1 mm pair buffer, a 12-step line search, and a 0.01 minimum
control trust region with the unchanged independent validator.

All 13 WSL tests pass (`387.04` seconds in the recorded single run). The six
headline deterministic cases also strictly converge and validate in a fresh
one-repetition run:

| Scenario | Candidate time (s) | Revision 8 time (s) | OSQP objective gap |
|---|---:|---:|---:|
| easy open | 0.727 | 0.571 | 0.013% |
| easy blocker | 0.980 | 0.573 | -0.026% |
| medium doorway | 3.740 | 3.585 | 0.494% |
| hard heterogeneous doorway | 9.259 | 8.351 | 2.319% |
| hard warehouse | 55.094 | 47.357 | OSQP failed |
| very hard maze | 71.166 | 61.525 | OSQP failed |

The four overlapping OSQP objective gaps remain at most `2.319%`. Turbo
remains much faster on medium doorway (`3.74` versus `28.67` seconds) and hard
heterogeneous doorway (`9.26` versus `106.97` seconds). The two centralized
OSQP large-scale rows remain failures.

On the exact-commit v2 rerun of the frozen 24-case passing-bay development
family, Turbo validates all 24 cases. Outcomes are 12 shared successes and 12
Turbo-only successes, with no CSDO-only or joint failures. Turbo validates all
11 cases where the common PBS/root search fails; CSDO validates none. The
fixed-corridor primary solve retains its previous 20 successes, and a
three-stage temporal corridor fallback is invoked only for and repairs the four
remaining primary failures.

Maximum Turbo terminal error is `0.910` mm; minimum pair and exact-obstacle
clearances are `0.664` mm and `44.14` mm; dynamics defect is numerical zero.
Median total Turbo wall time, including both attempts when recovery is used, is
`8.34` seconds versus CSDO's `0.245` seconds, so runtime is explicitly not the
CSDO-comparison claim. On the 12 shared successes, Turbo reduces the arrival
sum in eight and ties four (median `8.95%` reduction), and shortens paths in
eight (median `1.57%` reduction), while its median smoothness objective is
worse (`0.891` versus `0.334`).

The result is manifest-locked to commit `133dec2`, schedule seed `20260804`, a
120-second per-attempt cap, and Turbo executable SHA-256
`14e246c033c0087505ddf5fdca80f3927c16583d182f28f2a0e3821e8038c035`. The
three-stage window was selected on this development family; the disjoint
evaluation below tests the frozen policy without further tuning.

## CSDO passing-bay evaluation

The predeclared 32-case evaluation passes all four gates. Turbo validates
30/32 cases (`93.75%`) versus CSDO's 15/32 (`46.88%`), a `46.88` percentage-
point advantage. Among 17 cases where the common PBS/root fails, Turbo validates
15 (`88.24%`) and CSDO validates none. Paired outcomes are 15 shared, 15
Turbo-only, zero CSDO-only, and two neither-valid.

Every valid Turbo output passes the unchanged validator: maximum terminal error
is `0.946` mm, minimum pair and exact-obstacle clearances are `0.872` mm and
`50.17` mm, and dynamics defect is numerical zero. The two failures are the
10.2 m / `+1.5` m cases under both priorities; both remain obstacle-invalid by
`41.29` mm. The fallback is invoked for only these cases and recovers neither,
so all 30 successes come from the primary solver.

On the 15 shared successes, Turbo lowers the arrival sum in nine and ties six
with a median `3.03%` reduction, and shortens paths in ten with a median
`0.655%` reduction. Its median smoothness objective remains worse (`0.989`
versus `0.295`). Median wall time is `8.58` seconds for Turbo versus `0.099`
seconds for CSDO, so runtime is not claimed as an advantage.

The result is locked to exact commit
`b81785388be4d3eafcb480a4c7cbf06e8c8fd6de`, seed `20260805`, executable
SHA-256 `14e246c033c0087505ddf5fdca80f3927c16583d182f28f2a0e3821e8038c035`, and
manifest SHA-256
`7db22f99dc0fd4cf5c3af17d32a7d9914614331b87e9b32e68756ea138f1792f`.

## Revision 8 manifest-locked deterministic pilot

The frozen `ral-deterministic-v1-pilot` protocol records commit `faa8c18`, the
executable SHA-256, fixed `rho=35`, exact early ADMM, the automatic OpenMP team
policy, a 300-second cap, and all 36 interleaved tasks in `run_manifest.json`.
Changing any recorded configuration or rebuilding the executable prevents that
directory from being resumed.

The complete six-scenario by six-method WSL pilot records 28 completed solver
rows and eight censored timeouts. Full TurboADMM-NL strictly converges and
validates on all six headline scenarios. Centralized OSQP returns four valid
trajectories, but three stop at the SCP limit; its warehouse and maze rows time
out. Centralized qpOASES has two strict successes, two QP failures with
validator-feasible iterates, and two timeouts.

| Scenario | Full | OSQP | qpOASES | QP continuation | Inner | Cold |
|---|---:|---:|---:|---:|---:|---:|
| easy open | 0.541 | 0.250 | 0.642 | 0.468 | 0.721 | 10.734 |
| easy blocker | 0.545 | 0.992* | 1.368 | 0.578 | 0.866 | 19.632 |
| medium doorway | 3.550 | 28.632* | F (179.886) | 9.338 | 14.255 | T |
| hard heterogeneous doorway | 8.314 | 106.903* | F (99.403) | 19.085* | 26.862* | T |
| hard warehouse | 47.314 | T | T | 97.175* | 263.538* | T |
| very hard maze | 61.454 | T | T | 103.656* | 278.336* | T |

Values are solver time in seconds. `T` is a 300-second process timeout; `F` is
a solver failure. An asterisk marks an independently valid trajectory returned
without strict SCP convergence. On the four cases where OSQP returns a row,
Turbo's objective gaps are `0.013%`, `-0.026%`, `0.494%`, and `2.319%`,
respectively.

This is a one-repetition configuration gate, not the paper timing table. The
ten-repetition deterministic run remains locked until core solver parameters
and CSDO comparison behavior are frozen.

## Revision 7 delayed pair-feasibility polishing

Strict consensus polishing is now enabled only in the final half of SCP when
dynamics, terminal constraints, and static-obstacle constraints are already
feasible and pairwise clearance is the sole remaining defect. This preserves
the fast inexact path early in SCP and avoids scenario-specific parameter
exceptions.

The previously invalid `packed_within_flow_order04` development gate now
returns an independently valid trajectory with zero pairwise clearance,
`1.03e-7` m obstacle clearance, `1.42e-14` dynamics defect, and `1.26e-4`
terminal-state error. It takes 370.45 s, 31 SCP iterations, and 1,286 ADMM
iterations with two WSL threads. The trajectory is valid, but strict solver
convergence remains false because the line-search recovery budget is exhausted
after feasibility has been reached.

The order00 and order01 controls remain strictly converged and valid at 35.80 s
and 36.05 s, respectively. The new phase therefore repairs the deep-conflict
case without slowing the two existing successful controls in their retained
single-run measurements.

## Revision 6 terminal correctness and restoration hotstarts

A fixed-warm-start permutation diagnostic now separates Turbo solver ordering
from CSDO/PBS ordering. The successful `packed_within_flow_order00` guess and
corridors were held byte-identical while the Turbo agent vector was permuted as
`0-1-2-3-4`, `0-1-2-4-3`, and `1-0-2-3-4`. All three runs converged and
validated. After mapping schedules by physical name, their maximum state
difference was `6.38e-11` and their objective was `3.719340847186` to displayed
precision. The prior within-flow sensitivity therefore originates in different
PBS/Hybrid-A* warm starts and horizons, not Turbo's agent-vector ordering.

The diagnostic also exposed a correctness bug: elastic restoration disabled
the masked terminal-state equality, while solver convergence and the bridge
validator checked terminal position only. One completed order-01 trajectory
was incorrectly labeled valid with `0.0685` rad terminal-yaw error. Masked
terminal bounds now remain active in every restoration formulation, nonlinear
convergence includes masked state error, and the independent validator has a
separate terminal-state tolerance. The CSDO bridge freezes both terminal
position and terminal state tolerance at `1e-3`.

The main order-01 runtime pathology was repeated destruction of qpOASES state:
43 restoration attempts caused 353 cold starts and 450.56 s solver time.
Normal, reliable-retry, and elastic QPs now retain separate solver pools, so
each fixed formulation can use matrix/vector hotstarts across SCP iterations.
With the same 66-stage warm start, corridors, two WSL threads, and tolerances,
order-01 now converges and validates in 37.74 s with 4 restoration attempts,
28 cold starts, 75 matrix hotstarts, 456 vector hotstarts, `3.52e-5` maximum
terminal-state error, and zero final restoration slack. The order-00 control
remains at 37.37 s and validates with `7.23e-5` maximum terminal-state error.

The complete manual Turbo gate still passes 7/7 strict convergence and
independent validation. Single-run WSL solver times are 0.659, 0.847, 3.318,
13.678, 8.828, 50.068, and 67.010 s in scenario order. The six headline
cases therefore pass; `medium_heterogeneous_open` remains regression-only.
Objectives, minimum clearances, terminal-position errors, and zero final
restoration slack match the revision-5 solutions.

The deep-conflict `packed_within_flow_order04` case is not fixed. Its frozen
warm start has `-0.375` m pairwise clearance. A completion diagnostic exits
invalid after 281.03 s with 744 ADMM iterations, 0.279 dynamics defect,
`-0.353` m pairwise clearance, `-0.245` m exact obstacle clearance, and
maximum restoration slack 1.07. A direct rollout of the interpolated controls
is not a viable initializer: it misses goals by 5.30 m and reaches `-3.80` m
obstacle clearance. This row remains negative evidence; the CSDO publication
gate is not yet satisfied.

## Revision 5 restoration and CSDO milestone

The canonical WSL policy is fixed rho 35 with exact early ADMM and a strict
polishing phase. Rejected SCP directions now trigger a bounded restoration
solve and trust-region contraction. Practical stationarity requires a
converged strict ADMM solve plus three consecutive accepted relative-merit
decreases below `1e-3`; feasibility tolerances are unchanged.

Elastic obstacle slacks are shared by the collision samples in one time
interval. This removes the dense-Hessian memory blow-up in `very_hard_maze`:
the former process was killed at 15.48 GiB RSS, while the passing run peaks at
279 MiB. A global 2 m obstacle working set is now used by every benchmark case,
with the full obstacle set retained for nonlinear merit and independent
validation.

The final Release CTest run passes 13/13 in 257.86 seconds. All seven manual
regressions strictly converge and validate; the six headline cases exclude
`medium_heterogeneous_open`, which remains as an overhead-control regression.
Single-run solver evidence from this gate is:

| Scenario | n | m | Turbo time | Objective | Terminal error | Min pair / obstacle clearance |
|---|---:|---:|---:|---:|---:|---:|
| easy open | 2 | 0 | 0.537 s | 38.085 | 0.00061 m | -0.000003 / N/A m |
| easy blocker | 2 | 1 | 0.633 s | 48.560 | 0.00246 m | 1.055 / 0.000007 m |
| medium doorway | 4 | 2 | 3.041 s | 326.838 | 0.0528 m | 0.296 / -0.000206 m |
| medium heterogeneous open (regression only) | 4 | 0 | 12.368 s | 133.766 | 0.00779 m | -0.000059 / N/A m |
| hard heterogeneous doorway | 4 | 2 | 7.563 s | 357.889 | 0.2818 m | 1.416 / -0.000385 m |
| hard warehouse | 8 | 8 | 45.429 s | 1380.119 | 0.3011 m | 0.544 / -0.000869 m |
| very hard maze | 8 | 16 | 59.651 s | 1260.539 | 0.2618 m | 0.360 / -0.001434 m |

The previously measured centralized-OSQP warehouse reference is objective
1461.034 in 238.49 seconds, so the current Turbo solution is 5.54% lower in
objective and 5.25x faster. This reference was not rerun in revision 5 and is
not a statistical paper result.

The CSDO option-3 bridge now has two distinct, instrumented comparisons. The
strict backend ablation keeps CSDO's SQP loop and replaces only OSQP: on the
five-agent empty pilot, OSQP takes 0.0210 s and Turbo qpOASES takes 1.1815 s;
Riccati initializes all five agents and 14 matrix hotstarts are recorded. On
the 25-obstacle ex0 pilot, both methods are inaccurate (status 2 versus 3), at
0.0778 s and 3.4608 s. This ablation does not expose agent-level ADMM structure.

The full-engine bridge gives both methods the same PBS/Hybrid-A* guess and CSDO
corridors. Turbo validates on the empty pilot in 1.449 s, versus CSDO's 0.032 s
optimizer. The obstacle bridge is still blocked: on ex1, original CSDO succeeds
in 0.130 s with 0.111 m exact obstacle clearance, while Turbo stops after four
SCP iterations in 163.79 s with an interpolated obstacle collision. Restoration
now reaches ADMM and reports QP status, failed agent, slack, and recovery fields,
but it does not yet project the dynamics-defective Hybrid-A* guess into the
tight corridor efficiently. No runtime-advantage claim should be made from the
CSDO comparison until this blocker is resolved on the frozen seed matrix.

## Revision 4 convergence-quality milestone

Revision 4 removes the agent-count guard from residual-balanced rho adaptation,
adds per-SCP machine-readable traces, and introduces a cold reliable-mode
qpOASES restoration retry. If both the normal and reliable hard QPs fail, a
second retry adds exact-penalty terminal and obstacle slacks while shrinking
the control trust region. Convergence is forbidden until restoration slack is
below `1e-6`. The medium doorway failure was qpOASES return code 36
(`RET_INIT_FAILED_HOTSTART`) after a TQ factorization error, not geometric
infeasibility; its reliable hard retry succeeds without using elastic slack.

After the ordinary nonlinear convergence test first passes, every distributed
case enters a fixed-rho polish with `rho=5`, zero relative ADMM tolerance,
`1e-3` primal and dual absolute tolerances, and at most 200 ADMM rounds per SCP
subproblem. CTest now requires both feasibility and strict convergence.

The current seven-case WSL Release gate passes 7/7 strict convergence. Ten
fresh-process repetitions of TurboADMM-NL pass 70/70; the five centralized
OSQP cases that complete under the cap pass 50/50. Solver-time statistics are:

| Scenario | n | m | Turbo median [IQR], p95 | OSQP median [IQR], p95 | OSQP / Turbo | Objective gap |
|---|---:|---:|---:|---:|---:|---:|
| easy open | 2 | 0 | 0.329 [0.317, 0.342], 0.358 s | 0.227 [0.219, 0.234], 0.243 s | 0.69x | +0.05% |
| easy blocker | 2 | 1 | 0.545 [0.526, 0.566], 0.582 s | 0.624 [0.608, 0.636], 0.658 s | 1.14x | +0.06% |
| medium doorway | 4 | 2 | 2.432 [2.322, 2.442], 2.505 s | 14.348 [14.213, 15.063], 15.874 s | 5.90x | +0.32% |
| medium heterogeneous open | 4 | 0 | 4.113 [4.007, 4.234], 4.461 s | 2.171 [2.149, 2.212], 2.234 s | 0.53x | +1.23% |
| hard heterogeneous doorway | 4 | 2 | 4.599 [4.524, 4.717], 4.773 s | 39.451 [39.235, 39.564], 39.840 s | 8.58x | -0.69% |
| hard warehouse | 8 | 8 | 11.446 [11.341, 11.674], 11.944 s | timeout (>120 s) | >10.48x | N/A |
| very hard maze | 8 | 16 | 26.354 [26.164, 26.797], 27.680 s | timeout (>120 s) | >4.55x | N/A |

The two timeout rows use one censored OSQP execution from the same frozen WSL
protocol; repeating a known 120 second censor ten times was intentionally
avoided. On the five fully repeated comparisons, exact two-sided rank-sum tests
give `p <= 7.58e-5`. Bootstrap 95% intervals for the OSQP/Turbo median-time
ratio are `[5.81, 6.42]` for medium doorway and `[8.32, 8.75]` for hard
heterogeneous doorway. The five tests remain significant after Holm correction.

Every completed objective comparison is within the frozen 5% quality gate. In
the 70 TurboADMM-NL repetitions, the worst terminal error is 1.9365 m under the
2.5 m limit, minimum pairwise clearance is -0.000134 m, minimum obstacle
clearance is -0.001381 m under the -0.01 m validator tolerance, dynamics defect
is zero, and final restoration slack is zero. The repeated evidence supports a
structural crossover claim, not a universal small-problem speed claim: OSQP is
faster on easy open and four-agent heterogeneous open.

Raw rows and per-process logs are under
`build-ral-wsl/results/revision4_{turbo,osqp}_10/`; the combined summary is under
`build-ral-wsl/results/revision4_comparison/`. These are development results;
publication timing still requires the locked, preferably interleaved execution
schedule and the full Monte Carlo matrix.

## Revision 3 archived verification

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
