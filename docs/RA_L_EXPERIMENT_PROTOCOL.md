# TurboADMM-NL RA-L experiment protocol

This document freezes the scientific claim, algorithm variants, benchmark
design, and reporting rules before paper-scale experiments. Development runs
may use the smoke and development seeds. Final seeds must not be inspected or
used for parameter tuning.

## Positioning

TurboADMM-NL extends TurboADMM from one sequence of linearly constrained QPs
to a nested nonlinear optimization. At a fixed sequential convex programming
(SCP) linearization, ADMM changes only consensus-dependent QP vectors. Between
SCP iterations, dynamics Jacobians, collision normals, obstacle half-spaces,
and QP matrices change gradually. TurboADMM-NL exploits both forms of
parametric similarity while preserving per-agent dynamics and QP dimensions.
For spatially local problems, proximity screening keeps the active interaction
graph and per-agent obstacle set sparse; every omitted interaction is still
checked by the global nonlinear validator and is activated by a later SCP
linearization if it enters the configured margin.

The primary claim is:

> Nested parametric continuation transfers local active-set state and
> geometry-aligned consensus state both within ADMM and across SCP
> linearizations, reducing nonlinear multi-agent planning work without
> sacrificing solution feasibility or quality.

The implementation is an algorithmically distributed, shared-memory solver.
It is not a network-distributed multi-robot system.

## Required algorithm variants

All distributed variants must use the same model, convexification, costs,
bounds, initialization, penalty, trust region, and stopping tolerances.

1. `cold`: every local QP is initialized independently; no QP or consensus
   state crosses an ADMM or SCP boundary.
2. `inner`: Riccati initializes the first local QP of each SCP iteration and
   qpOASES vector hotstarts are used only inside that ADMM solve.
3. `qp_continuation`: `inner` plus matrix/working-set hotstarts between SCP
   iterations; pairwise consensus and dual state are reset.
4. `full`: `qp_continuation` plus geometry-aware transport of consensus and
   scaled dual state between compatible SCP collision half-spaces.
5. `centralized_qpoases`: the same SCP subproblem assembled as one QP and
   solved by qpOASES.
6. `centralized_osqp`: the identical centralized SCP QP solved by official
   OSQP. This variant is enabled only when the OSQP dependency is available.

Published external planners must use their official implementations. CSDO is
evaluated only on compatible ground-vehicle scenarios. An official aerial
planner is evaluated only on its compatible quadrotor subset. Unsupported
models are reported as not applicable, not as failures.

## Geometry-aware state transport

For pair `(i,j)` and stage `k`, let `n_old` and `n_new` be consecutive unit
collision normals. Transport is permitted when

`dot(n_old, n_new) >= continuation_minimum_normal_dot`.

The previous auxiliary positions are projected onto the new half-space

`n_new' (v_i - v_j) >= r_i + r_j`.

Only the normal component of each previous scaled dual is retained and aligned
with `n_new`. Incompatible stages are initialized from the new nominal
positions. Transport and reset counts are recorded.

## Success predicate

A run is successful only when the solver returns without a QP/backend failure
and an independent post-solve validator confirms:

- minimum pairwise clearance is at least `-collision_tolerance`;
- minimum obstacle clearance is at least `-collision_tolerance`;
- maximum nonlinear dynamics defect is at most `dynamics_tolerance`;
- each terminal position error is at most the scenario tolerance;
- every masked terminal-state equality satisfies its frozen tolerance; and
- the solver terminates before the scenario timeout.

Solver-reported success alone is not sufficient. Pairwise and obstacle safety
are checked at trajectory knots and on interpolated substeps.

Experiment revision 7 uses 20 collision samples per interval, at most 90 SCP
iterations, and at most 50 ADMM iterations per ordinary SCP subproblem. The
canonical penalty is `rho=35`; primal, dual, and relative tolerances are
`1e-3`, `1e-2`, and `1e-3`; over-relaxation is `1.6`; merit penalty is `1e7`;
validator interpolation count is 20; and terminal position tolerance is 2.5.
Inexact early ADMM and residual-balanced penalty adaptation are disabled in
the frozen deterministic comparison.

After the ordinary convergence test first passes, distributed methods retain
canonical `rho=35` and use zero relative tolerance, primal and dual absolute
tolerances `1e-3`, and at most 200 ADMM rounds per SCP subproblem. Failed
convex solves are rebuilt cold with qpOASES reliable options. A final fallback
permits two restoration attempts, shrinks the control trust region by 0.5 down
to 0.1, and assigns exact-penalty weight `1e4` to nonnegative obstacle,
corridor, and terminal slacks. Strict convergence requires both maximum and
final slack at most `1e-6`.

In the final half of SCP, exact consensus polishing is also activated when
dynamics, terminal constraints, and static-obstacle constraints are feasible
and pairwise clearance is the sole remaining defect. This gate is uniform
across scenarios and does not change physical tolerances.

Constant-density scenarios use 2.0 m pair and obstacle activation margins;
fixed-workspace scenarios retain every constraint. All global safety checks
retain every pair and obstacle. Benchmark matrices use a 300 second wall-time
limit per scenario-method run. Results from different experiment revisions may
not be mixed.

Before Monte Carlo evaluation, every method is run on a fixed difficulty
ladder with explicit geometry and an outer-ring feasibility witness:

1. `easy_open`: two unicycles, no obstacles;
2. `easy_single_blocker`: two unicycles and one central polygon;
3. `medium_doorway`: four unicycles and a two-polygon doorway;
4. `medium_heterogeneous_open`: four mixed-model agents without obstacles;
5. `hard_heterogeneous_doorway`: the mixed-model agents plus the doorway;
6. `hard_warehouse`: eight heterogeneous agents and eight shelf polygons; and
7. `very_hard_maze`: eight heterogeneous agents and sixteen maze polygons.

The six headline cases exclude `medium_heterogeneous_open`, which remains a
regression-only overhead control because it does not exercise collision or
obstacle structure. It is never deleted from CTest.

All seven cases are correctness gates for `full` TurboADMM-NL and are executed
by CTest with `--require-success --require-convergence`. The six headline cases
are timed for every competitor. Competitor failures remain in the results and
define their measured boundary; they are not grounds for weakening or deleting
a headline scenario. Monte Carlo evaluation starts after the full-method gate
passes and retains every valid failure.

## Benchmark matrix

The submission-facing primary Monte Carlo grid is intentionally nonredundant:

- agents `n = {4, 8, 12, 20}`;
- convex polygon obstacles `m in {0, n, 2n}`;
- homogeneous unicycles and balanced heterogeneous agents;
- constant-density local exchanges, so the intended interaction degree remains
  bounded while centralized QP dimensions grow with `n`; and
- 10 development seeds or 30 untouched final seeds per cell.

This produces exactly 240 `paper_development` scenarios and 720 `paper_final`
scenarios. `--track primary` selects this grid. Dry-run writes the complete
scenario inventory, including its index, seed, dimensions, feasibility witness,
horizon, and potential pair count. The runner hashes that inventory into the run
manifest before any optimizer is launched.

The earlier breadth grid remains available for exploratory coverage:

- agents `n = {2, 4, 8, 14, 20}`;
- convex polygon obstacles `m = {0, 4, 8, 16, 32}`;
- at least 30 locked seeds per reported cell; and
- fixed-workspace and constant-density regimes.

The experiment is split into nonredundant tracks rather than taking the full
Cartesian product of every axis:

- `scaling`: balanced heterogeneous agents over the complete `n` by `m` grid;
  the fixed-workspace regime uses dense antipodal or random-field geometry,
  while the constant-density regime uses separated local exchanges with one
  intended interaction per agent and spatially local obstacle groups;
- `models`: all four model compositions at `n in {4,8}` and `m in {0,8}` in a
  fixed workspace; and
- `families`: the balanced composition at `n = 8`, `m in {8,16}` on all
  obstacle families in a fixed workspace.

`--track all` is the union of these tracks. Overlapping cases are executed
once. This preserves all claimed axes without an uninformative full Cartesian
product.

Scenario families:

- antipodal open crossing;
- spatially local pair exchanges with local polygon groups;
- random convex-obstacle field;
- doorway or bottleneck;
- warehouse aisles;
- multiple-homotopy maze;
- heterogeneous 2D/3D crossing.

Agent compositions include homogeneous unicycle, homogeneous bicycle,
homogeneous reduced-order quadcopter, a balanced mixture, and ground- or
aerial-dominant mixtures. The reduced-order quadcopter limitation must be
stated explicitly in the paper.

Every random scenario records its seed. The seed perturbs the common formation
phase, individual angular spacing, and random polygon placement. The horizon
grows with workspace radius so the bounded-speed agents can complete the
analytic outer-ring witness. Random polygons that intersect the witness
clearance tube are rejected before any compared optimizer is run.

The smoke suite contains 24 unique scenarios. The nonredundant development and
locked-final legacy breadth-track unions contain 680 and 2,040 scenarios,
respectively. They are supplementary stress suites, not the submission-facing
primary matrix.

## Data splits

- Smoke seeds: `0-2`; used by CI and quick local checks.
- Legacy breadth development seeds: `1000-1009`.
- Legacy breadth final seeds: `10000-10029`.
- Paper-primary development seeds: `2000-2009`; parameter tuning is allowed.
- Paper-primary final seeds: `20000-20029`; locked until algorithms and
  parameters freeze.

Changing an algorithm or a tuned parameter after inspecting final results
invalidates affected final runs.

## Timing protocol

All publication timings use the OSQP-enabled WSL Release build. The runner
sets `OMP_DYNAMIC=FALSE`, `OMP_PROC_BIND=close`, and `OMP_PLACES=cores`.
TurboADMM-NL uses the frozen automatic local-QP team size

`T = min(n, max(4, ceil(n/2)), OpenMP maximum)`.

An explicit positive `--threads` value is permitted only for the declared
thread ablation. Deterministic timing tables use ten fresh process executions
per scenario-method pair. Method order is rotated by scenario and repetition
using schedule seed `20260804`; this prevents one method from always occupying
the same thermal/order position. Report the median, interquartile range, and
95th percentile. Monte Carlo cells use one execution for each of 30 locked
seeds and paired seed-wise statistics. Solver-internal time and process wall
time are both retained. Deterministic runs retain their 300-second cap; the
paper-primary Monte Carlo pilot, development, and final matrices use a frozen
120-second cap. Timeouts remain censored failures.

Every run directory contains `run_manifest.json`, including the Git commit,
executable and runner SHA-256 values, exact solver flags, OpenMP policy,
timeout, and complete task order. The runner rejects an attempt to resume that
directory with a different binary, runner, or configuration. Each process
attempt writes to a unique temporary result path before atomic promotion, so a
solver orphaned by host termination cannot corrupt a resumed runner's result.
The paired analyzer emits per-instance, per-cell, and aggregate-by-scale CSVs.

The one-repetition protocol check and locked ten-repetition run are launched
from WSL with:

```bash
TURBOADMM_GIT_COMMIT=<commit> ./benchmarks/run_ral_deterministic_wsl.sh pilot
TURBOADMM_GIT_COMMIT=<commit> ./benchmarks/run_ral_deterministic_wsl.sh final
```

The primary Monte Carlo pilot, development matrix, and immutable final matrix
are launched with:

```bash
./benchmarks/run_ral_monte_carlo_wsl.sh pilot
./benchmarks/run_ral_monte_carlo_wsl.sh development
./benchmarks/run_ral_monte_carlo_wsl.sh final
```

## Recorded metrics

Each run emits one machine-readable row containing:

- scenario family, density regime, seed, `n`, `m`, and model composition;
- method, solver configuration, and actual thread count;
- the run manifest separately records the Git commit, executable hash,
  environment policy, and task order;
- solver success, validator success, strict convergence, status, and failure
  category, with feasible-planning success and strict convergence reported
  separately;
- total wall time and maximum per-agent local-QP time;
- objective and objective gap to the best feasible centralized solution;
- minimum pairwise and obstacle clearances;
- maximum nonlinear dynamics defect and terminal errors;
- SCP and ADMM iterations, QP solves, working-set recalculations;
- active and potential pairs, maximum graph degree, and local obstacle counts;
- maximum local and centralized QP dimensions;
- QP build, pair build, repeated assembly, parallel local-QP batch, consensus,
  and globalization timings;
- cold, vector, and matrix hotstarts;
- transported and reset pair-stage states;
- dimension-aware primal and dual stopping thresholds;
- per-SCP objective, merit, residual, rho, trust-region, clearance, terminal,
  QP return-code, restoration, and polishing traces;
- restoration attempts and maximum/final elastic slack; and
- timeout and peak memory when available.

Report medians and quantiles for time and work, confidence intervals for
success rates, and paired comparisons on identical scenarios. Failures are
classified as invalid scenario, initialization, QP failure, SCP stagnation,
ADMM timeout, safety violation, dynamics violation, or terminal violation.

## Paper decision gates

1. Correctness: `full` strictly converges on all seven deterministic gates in
   the OSQP-enabled WSL Release build. Monte Carlo feasible-planning success and
   strict convergence are distinct statistics; solver and validator failures
   are reported in the success distribution and are never discarded. A valid
   SCP-limit iterate is not relabeled as strictly converged.
2. Novelty: `full` produces a consistent and practically meaningful reduction
   in time or working-set recalculations relative to `inner` and
   `qp_continuation`, without a worse success rate or objective distribution.
   A target effect size of at least 20 percent median reduction is fixed before
   final experiments.
3. Scaling: fixed-workspace and constant-density results are reported
   separately. In the local regime, active degree must remain bounded while
   centralized QP dimensions grow with total problem size. Solver crossover
   claims are based on locked paired results, not selected successful examples.
4. External comparison: on a frozen CSDO-compatible congested family, compare
   both methods with the same PBS/Hybrid-A* front end and independent validator.
   A recovery or trajectory-quality claim requires a family-level paired effect,
   not one hand-selected failure. Runtime is retained as a secondary metric even
   when recovery success is the primary comparison.
   The 24-case passing-bay grid is development data. After its parameters are
   fixed, generate a distinct evaluation grid, record all rows, and do not tune
   the solver, initializer, horizon, tolerance, or time cap on those evaluation
   outcomes.
5. Reproducibility: a clean build can run tests, smoke benchmarks, analysis,
   and figure generation from documented commands.

If the novelty gate fails, the paper claim must be revised before running the
final benchmark. Benchmark breadth cannot substitute for an ineffective
continuation mechanism.
