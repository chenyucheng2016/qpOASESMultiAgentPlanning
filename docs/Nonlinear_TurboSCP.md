# TurboSCP nonlinear extension

This branch adds an end-to-end nonlinear path without changing the existing
linear `TurboADMM` API.

## Implemented

- `NonlinearModel` separates discrete dynamics, analytic Jacobians, position
  output, and position Jacobians.
- `UnicycleModel` uses state `[px, py, heading, speed]` and input
  `[acceleration, yaw_rate]`.
- `BicycleModel` uses state
  `[px, py, heading, speed, steering_angle]` and input
  `[acceleration, steering_rate]`.
- `QuadcopterModel` is a reduced-order 3D model with position, velocity, and
  roll-pitch-yaw states, collective thrust, and commanded attitude rates.
- A single problem may mix state, control, and position dimensions. Collision
  geometry is evaluated in the maximum world dimension; 2D positions are
  embedded at `z = 0` when a 3D model is present.
- `problem.collisionRadius` defines each agent's circular/spherical footprint.
  Pairwise separation is the sum of the two radii. A negative radius uses
  `options.safetyDistance / 2`, preserving the previous global-distance API.
- Preflight validation rejects colliding fixed starts and requested goal
  references with agent, obstacle or pair, and stage identifiers. Failed SCP
  geometry reports the worst conflict and detects overlapping corridor margins.
- `ConvexPolygonObstacle` supplies ordered x-y vertices to the nonlinear
  solver. If a reference intersects an obstacle, the solver deterministically
  selects the lower-cost bypass side and uses supporting planes along the
  clearance-offset polygon boundary. A collision-free initial trajectory fixes
  the homotopy and bypass rows are accepted only when they retain the current
  nominal point. If the initial trajectory crosses an obstacle, the selected
  deterministic bypass remains authoritative. This permits both explicit
  feasible witnesses and zero-control initialization for the tested
  single- and multiple-obstacle routes.
- Each agent may override `options.obstacleSafetyDistance` with
  `problem.obstacleSafetyDistance`; a negative value keeps the global default.
- Polygon obstacles are treated as vertical prisms for 3D agents.
- `solveStageVaryingLqr` solves affine, time-varying finite-horizon LQR and
  supplies a dynamics-feasible primal warm start.
- Four continuation modes isolate cold QPs, inner-ADMM vector hotstarts,
  cross-SCP matrix/active-set continuation, and full geometry-aware transport
  of consensus auxiliaries and scaled duals.
- `NCM_DISTRIBUTED_ADMM` uses one `SQProblem` per agent, vector hotstarts
  inside ADMM, matrix/working-set hotstarts between SCP iterations, and
  optional OpenMP-parallel local solves.
- `NCM_CENTRALIZED_SCP` assembles the same linearizations and collision
  half-spaces in one QP for an algorithmic baseline.
- `NCM_CENTRALIZED_OSQP` sends that identical assembled QP to OSQP when the
  optional backend is enabled. Both OSQP 0.6 and 1.x public APIs are supported.
- Pairwise and polygon constraints are imposed at configurable interpolated
  samples between trajectory knots, not only at the knots.
- Merit evaluation and solver success use the same inter-knot samples as the
  convexified collision rows, preventing a knot-only convergence decision from
  hiding a between-knot collision.
- `validateNonlinearTrajectories` independently reruns nonlinear dynamics and
  densely samples pairwise and polygon clearance. Benchmark success requires
  both a non-failing solver result and this validator.
- Nonlinear rollout plus merit line search globalizes every SCP step.

At stage `k`, collision convexification fixes the nominal relative-position
normal `n_k` and enforces `n_k' (p_i - p_j) >= d_safe`. The distributed
coordinator projects auxiliary positions onto this half-space. Its dual
residual is `rho * ||v - v_previous||`, not a copy of the primal residual.

Static obstacles are passed through the overload:

```cpp
ConvexPolygonObstacle obstacle;
obstacle.vertices = {-0.8, 0.0, 0.0, -0.5, 0.8, 0.0, 0.0, 0.5};
std::vector<ConvexPolygonObstacle> obstacles(1, obstacle);
NonlinearTurboResult result = solver.solve(agents, obstacles, options);
```

Clockwise and counter-clockwise vertex order are accepted; concave, degenerate,
or unordered polygons are rejected. Automatic bypass selection is local and
deterministic. Complicated obstacle fields with competing homotopy classes may
still benefit from a collision-free reference or initial controls.

## Build and verify

The OSQP comparison is optional. On WSL with OSQP installed under
`/usr/local`, run the complete reproducible workflow with:

```bash
bash benchmarks/run_nonlinear_wsl.sh ci all
```

The equivalent manual configuration is:

```bash
cmake -S . -B build-nonlinear \
  -DQPOASES_BUILD_EXAMPLES=OFF \
  -DQPOASES_BUILD_OSQP_BACKEND=ON -DOSQP_ROOT=/usr/local
cmake --build build-nonlinear -j
(cd build-nonlinear && ctest --output-on-failure)
python3 benchmarks/run_nonlinear_matrix.py \
  build-nonlinear/bin/nonlinear_benchmark --suite smoke \
  --output-dir build-nonlinear/results/smoke --timeout-seconds 120
python3 benchmarks/analyze_nonlinear_benchmark.py \
  build-nonlinear/results/smoke/results.csv \
  --execution-status build-nonlinear/results/smoke/execution_status.csv \
  --summary build-nonlinear/smoke_summary.csv \
  --enriched build-nonlinear/smoke_enriched.csv \
  --method-summary build-nonlinear/smoke_method_summary.csv
```

The benchmark first provides a seven-case deterministic ladder from a
two-unicycle open crossing through heterogeneous doorway, warehouse, and maze
stress cases. It then provides smoke, development, and locked final seed splits;
agent counts 2, 4, 8, 14, and 20; obstacle counts 0, 4, 8, 16, and 32;
fixed-workspace and constant-density scaling; unicycle, bicycle, reduced-order
quadcopter, and balanced heterogeneous compositions; and antipodal, random
convex-field, bottleneck, warehouse-aisle, and multiple-homotopy families.
Monte Carlo seeds randomize formation phase, angular spacing, and polygon
placement. An adaptive horizon and obstacle rejection preserve an analytic
outer-ring feasibility witness.
Every run records solver, validator, and combined protocol success separately,
wall time, objective, clearances, dynamics and terminal errors, iterations,
backend work, hotstarts, transported/reset collision state, and parallel QP
batches, residuals, and the frozen collision/SCP/ADMM/merit settings. The
analysis script reports Wilson success intervals, median and interquartile
times, work, run-level objective gaps, and timeout-aware method summaries.
The matrix runner executes one scenario-method pair per process, preserves
completed rows across restarts, records timeouts and logs separately, and hashes
the generated scenario inventory into its immutable run manifest.

The submission-primary matrix uses `n={4,8,12,20}`, `m={0,n,2n}`, homogeneous
unicycles and balanced heterogeneous agents, and constant-density local
exchanges. It contains 240 development scenarios and 720 untouched final
scenarios. The complete primary matrix can be audited without solving it:

`./build-nonlinear/bin/nonlinear_benchmark --suite paper_final --track primary
--dry-run --output paper_final_inventory.csv`

The broader legacy matrix remains auditable with:

`./build-nonlinear/bin/nonlinear_benchmark --suite development --track all
--dry-run`


The deterministic tests verify:

1. analytic unicycle Jacobians against finite differences;
2. affine Riccati dynamics and control stationarity;
3. distributed and centralized nonlinear two-agent radius-sum safety,
   backward-compatible global-distance fallback, invalid-start diagnostics,
   tracking, nonlinear dynamics defects, solver work, and hotstart counters;
4. analytic bicycle and reduced-order quadcopter Jacobians against finite
   differences;
5. one heterogeneous problem containing a unicycle, bicycle, and 3D
   quadcopter for both distributed ADMM and centralized SCP;
6. convex-polygon avoidance for distributed and centralized SCP, invalid
   polygon rejection, endpoint and insufficient-passage diagnostics, per-agent
   clearance in mixed unicycle-quadcopter avoidance of a vertically extruded
   polygon, zero-seed avoidance of two intersecting polygons, and a 0.70 m
   polygon corridor with 0.10 m residual width after clearance inflation.
7. continuation-mode accounting and state-transport behavior; and
8. all seven deterministic benchmark scenarios under the independent
   validator through the `nonlinear_manual_full` CTest gate.

Current development evidence is recorded in `RA_L_BENCHMARK_STATUS.md`.

## RA-L benchmark completion gates

Before submission, integrate the selected recent planner through its official
implementation on the subset of models it supports, freeze parameters on the
development split, and run the final 30-seed matrix without tuning. The current
quadcopter is reduced-order; a rigid-body model with angular velocity and torque
inputs remains a separate fidelity study. The novelty gate in
`RA_L_EXPERIMENT_PROTOCOL.md` must pass before claiming that cross-SCP
continuation improves TurboADMM.
