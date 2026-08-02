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
- each terminal position error is at most the scenario tolerance; and
- the solver terminates before the scenario timeout.

Solver-reported success alone is not sufficient. Pairwise and obstacle safety
are checked at trajectory knots and on interpolated substeps.

Experiment revision 4 uses 20 collision samples per interval, 30 SCP
iterations, at most 50 ADMM iterations per ordinary SCP subproblem, canonical
penalty `rho=35`, absolute primal and dual tolerances of `1e-3` and `1e-2`,
relative tolerance `1e-3`, over-relaxation `1.6`, merit penalty `1e7`, validator
interpolation count 20, and terminal position tolerance 2.5. The first two SCP
iterations use ADMM tolerance multipliers 5 and 3. Residual-balanced penalty
adaptation is active at every agent count, is checked every five rounds, uses
imbalance 10 and scale factor 2, and keeps `rho` in `[5,140]`. Scaled duals are
rescaled whenever `rho` changes.

After the ordinary convergence test first passes, distributed methods use a
fixed `rho=5` polishing phase with zero relative tolerance, primal and dual
absolute tolerances `1e-3`, and at most 200 ADMM rounds per SCP subproblem.
Failed convex solves are rebuilt cold with qpOASES reliable options. A final
fallback permits two restoration attempts, shrinks the control trust region by
0.5 down to 0.1, and assigns exact-penalty weight `1e4` to nonnegative obstacle
and terminal slacks. Strict convergence requires maximum slack at most `1e-6`.

Constant-density scenarios use 2.0 m pair and obstacle activation margins;
fixed-workspace scenarios retain every constraint. All global safety checks
retain every pair and obstacle. Benchmark matrices use a 120 second wall-time
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

All seven cases are correctness gates for `full` TurboADMM-NL and are executed
by CTest with `--require-success --require-convergence`. Competitor failures
remain in the results and
define their measured boundary; they are not grounds for weakening or deleting
a deterministic scenario. Monte Carlo evaluation starts after the full-method
gate passes and retains every valid failure.

## Benchmark matrix

Primary scaling grid:

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
locked-final track unions contain 680 and 2,040 scenarios, respectively.

## Data splits

- Smoke seeds: `0-2`; used by CI and quick local checks.
- Development seeds: `1000-1009`; parameter tuning is allowed.
- Final seeds: `10000-10029`; locked until algorithms and parameters freeze.

Changing an algorithm or a tuned parameter after inspecting final results
invalidates affected final runs.

## Timing protocol

All publication timings use the OSQP-enabled WSL Release build. The runner
sets `OMP_DYNAMIC=FALSE`, `OMP_PROC_BIND=close`, and `OMP_PLACES=cores`.
TurboADMM-NL uses the frozen automatic local-QP team size

`T = min(n, max(4, ceil(n/2)), OpenMP maximum)`.

An explicit positive `--threads` value is permitted only for the declared
thread ablation. Deterministic timing tables use ten independent process
executions per scenario-method pair and report median, interquartile range,
and 95th percentile. Monte Carlo cells use one execution for each of 30 locked
seeds and paired seed-wise statistics. Solver-internal time and process wall
time are both retained. Timeouts remain censored failures at 120 seconds.

## Recorded metrics

Each run emits one machine-readable row containing:

- scenario family, density regime, seed, `n`, `m`, and model composition;
- method, configuration, Git commit, compiler, CPU, and thread count;
- solver success, validator success, status, and failure category;
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

1. Correctness: `full` strictly converges on all seven deterministic cases in
   the OSQP-enabled WSL Release build. Monte Carlo solver and validator failures are
   reported in the success distribution and are never discarded.
2. Novelty: `full` produces a consistent and practically meaningful reduction
   in time or working-set recalculations relative to `inner` and
   `qp_continuation`, without a worse success rate or objective distribution.
   A target effect size of at least 20 percent median reduction is fixed before
   final experiments.
3. Scaling: fixed-workspace and constant-density results are reported
   separately. In the local regime, active degree must remain bounded while
   centralized QP dimensions grow with total problem size. Solver crossover
   claims are based on locked paired results, not selected successful examples.
4. Reproducibility: a clean build can run tests, smoke benchmarks, analysis,
   and figure generation from documented commands.

If the novelty gate fails, the paper claim must be revised before running the
final benchmark. Benchmark breadth cannot substitute for an ineffective
continuation mechanism.
