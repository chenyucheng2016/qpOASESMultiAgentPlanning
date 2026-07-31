# TurboSCP nonlinear extension

This branch adds an end-to-end nonlinear path without changing the existing
linear `TurboADMM` API.

## Implemented

- `NonlinearModel` separates discrete dynamics, analytic Jacobians, position
  output, and position Jacobians.
- `UnicycleModel` uses state `[px, py, heading, speed]` and input
  `[acceleration, yaw_rate]`.
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

The deterministic tests verify:

1. analytic unicycle Jacobians against finite differences;
2. affine Riccati dynamics and control stationarity;
3. distributed and centralized nonlinear two-agent safety, tracking,
   nonlinear dynamics defects, solver work, and hotstart counters.

## RA-L benchmark completion gates

Before submission, add the official OSQP backend for centralized SCP, an
independent decentralized iLQR plus dual-consensus ADMM baseline, and official
acados/HPIPM and CasADi/IPOPT integrations. Extend models to a bicycle and a
quadrotor, then run 2, 4, 8, 14, and 20 agents over at least 30 deterministic
seeds. Report safety failures and nonlinear dynamics defects alongside time,
objective, SCP/ADMM iterations, QP work, and cold/primal/working-set/full
hotstart ablations.
