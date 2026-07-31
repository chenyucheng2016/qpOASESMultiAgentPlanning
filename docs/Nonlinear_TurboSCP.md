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
- `solveStageVaryingLqr` solves affine, time-varying finite-horizon LQR and
  supplies a dynamics-feasible primal warm start.
- `NCM_DISTRIBUTED_ADMM` uses one `SQProblem` per agent, vector hotstarts
  inside ADMM, and matrix/working-set hotstarts between SCP iterations.
- `NCM_CENTRALIZED_SCP` assembles the same linearizations and collision
  half-spaces in one QP for an algorithmic baseline.
- Nonlinear rollout plus merit line search globalizes every SCP step.

At stage `k`, collision convexification fixes the nominal relative-position
normal `n_k` and enforces `n_k' (p_i - p_j) >= d_safe`. The distributed
coordinator projects auxiliary positions onto this half-space. Its dual
residual is `rho * ||v - v_previous||`, not a copy of the primal residual.

## Build and verify

External comparison dependencies are optional:

```text
cmake -S . -B build-nonlinear -DQPOASES_BUILD_EXAMPLES=OFF
cmake --build build-nonlinear --config Release
ctest --test-dir build-nonlinear --output-on-failure
```

Set `QPOASES_BUILD_EXTERNAL_BENCHMARKS=ON` after installing the official OSQP,
MOSEK, HPIPM, and BLASFEO dependencies.

Generate one CSV row per implemented coordination method for the mixed
unicycle-bicycle-quadcopter scenario:

```text
cmake --build build-nonlinear --target nonlinear_heterogeneous_benchmark
```

The output is `build-nonlinear/heterogeneous_benchmark.csv`. It records
convergence and feasibility separately, wall time, objective, safety margin,
nonlinear dynamics defect, iteration and QP work counts, hotstart counts, and
terminal tracking errors.

The deterministic tests verify:

1. analytic unicycle Jacobians against finite differences;
2. affine Riccati dynamics and control stationarity;
3. distributed and centralized nonlinear two-agent safety, tracking,
   nonlinear dynamics defects, solver work, and hotstart counters;
4. analytic bicycle and reduced-order quadcopter Jacobians against finite
   differences;
5. one heterogeneous problem containing a unicycle, bicycle, and 3D
   quadcopter for both distributed ADMM and centralized SCP.

## RA-L benchmark completion gates

Before submission, add the official OSQP backend for centralized SCP, an
independent decentralized iLQR plus dual-consensus ADMM baseline, and official
acados/HPIPM and CasADi/IPOPT integrations. Run 2, 4, 8, 14, and 20 agents over
at least 30 deterministic seeds, including several heterogeneous mixtures.
Report safety failures and nonlinear dynamics defects alongside time,
objective, SCP/ADMM iterations, QP work, and cold/primal/working-set/full
hotstart ablations. The current quadcopter is reduced-order; a rigid-body model
with angular velocity and torque inputs is a separate fidelity study.
