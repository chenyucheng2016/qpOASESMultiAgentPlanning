# CSDO paired comparison protocol

This comparison isolates the nonlinear optimization stage while preserving
CSDO's published centralized front end:

    CSDO instance
      -> PBS and spatiotemporal Hybrid A*
      -> CSDO interpolated warm start
         -> original CSDO decentralized SQP/OSQP
         -> TurboADMM-NL

The official CSDO source remains an external, unmodified checkout. The bridge
reads its generated guesses YAML file; it does not replace CSDO's internal
solveOSQP function, because that function is downstream of CSDO's fixed
separating-plane decomposition and no longer represents the multi-agent
consensus problem.

## Matched formulation

- State: x, y, yaw, steering.
- Control: velocity, steering rate.
- Time step: 3 * 0.706 / (3 * 0.8) = 0.8825 seconds.
- Wheelbase: 1.0 metre.
- Control bounds: absolute velocity at most 1.0 and absolute steering rate at
  most 0.07.
- Steering bound: absolute steering at most atan(1/3).
- Vehicle geometry: circles of radius 1.25 at longitudinal offsets 1.25 and
  -0.25 metres.
- Objective: sum of squared successive velocity differences plus squared
  steering rates.
- Initial x, y, yaw, steering and terminal x, y, yaw are fixed as in CSDO.
  Terminal steering is not fixed.
- CSDO circular obstacles are represented in the Turbo QPs by 32-sided
  circumscribed polygons. Both outputs are independently checked against the
  exact circles and map boundary.

The bridge uses one collision sample per interval to match CSDO's knot-wise
constraints. The independent validator uses ten interpolation substeps, so
between-knot collisions are still reported.

## Build

TurboADMM-NL:

    cmake -S . -B build-ral-wsl \
      -DQPOASES_BUILD_CSDO_COMPARISON=ON
    cmake --build build-ral-wsl -j --target csdo_turbo_comparison

CSDO needs its documented Boost Program Options, OMPL, yaml-cpp, Eigen, and
OSQP 0.6.3 dependencies. Keep OSQP 0.6.3 in a private prefix and pass that
prefix to CSDO with CMAKE_PREFIX_PATH; do not replace the OSQP 1.x
installation used by this repository.

## Run one paired instance

    python3 scripts/run_csdo_turbo_comparison.py \
      --instance CSDOTrajectoryPlanning/benchmark/map50by50/agents5/obstacle/map_50by50_obst25_agents5_ex0.yaml \
      --csdo-executable CSDOTrajectoryPlanning/build-codex/csdo \
      --turbo-executable build-ral-wsl/bin/csdo_turbo_comparison \
      --work-dir build-ral-wsl/csdo-pilot \
      --output-csv build-ral-wsl/csdo-pilot/results.csv

The official executable returns code 1 even after a successful run. The
runner therefore judges completion by the presence of both CSDO output files
and records the return code rather than treating it as an automatic failure.

## Timing fields

The CSV deliberately keeps these clocks separate:

- CSDO process wall time.
- CSDO reported sequential optimization time.
- CSDO paper-style ideal parallel time, the maximum per-agent optimizer time.
- Turbo process wall time.
- Turbo solver-internal wall time.
- A shared-front-end Turbo end-to-end estimate equal to CSDO search plus CSDO
  preprocess plus Turbo solver.

The shared-front-end estimate is not labeled as directly measured end-to-end
time. This prevents CSDO's sequential implementation and the bridge process
startup from being hidden in the comparison.

## Evaluation gate

Start with ten frozen instances in each cell of:

- 5, 10, 15, and 20 agents.
- Empty maps and maps with 25 obstacles.

Scale to all 60 instances per cell only after both methods pass the exact
validator and the CSV contains no missing timing or quality fields.

## Local smoke test

The repository includes a one-agent CSDO-format parser and formulation smoke
fixture:

    build-ral-wsl/bin/csdo_turbo_comparison \
      --input tests/data/csdo_smoke_input.yaml \
      --guess tests/data/csdo_smoke_guesses.yaml \
      --output build-ral-wsl/csdo_smoke_result.yaml
