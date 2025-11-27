This repository demonstrates the significant performance gains achieved by using a Riccati-based primal-dual warm start for multi-agent motion planning. The evidence provided by the 2, 4, and 6-agent test scenarios shows that this feature is critical for enabling real-time, complex multi-agent control.

## Purpose

The goal of this project is real-time, complex, heterogeneous multi‑agent control. Rather than a full planner, it provides a fast multi‑agent trajectory optimization engine that can be embedded inside a planner. Built on qpOASES, it exploits the hotstart mechanism to reuse KKT factorizations across iterations. Multi‑agent coordination is formulated in a distributed fashion via ADMM, with coupling handled by Lagrangian multipliers. This framework, although solving problems iteratively, is more scalable than the centralized approaches that require monolithic Schur‑complement solves of arrowhead‑structured Hessians.

## Methodology

This work achieves real-time multi-agent trajectory optimization through two key innovations:

### 1. Riccati Warm Start for MPC-Aware QP Solving

Each agent's trajectory optimization is formulated as a Model Predictive Control (MPC) problem solved via quadratic programming (QP). We exploit MPC structure using **Riccati recursion**:

- **Backward Riccati recursion** solves the unconstrained LQR problem analytically in O(N) time
- Provides optimal primal (trajectory) and dual (costates) warm start for qpOASES
- **Reduces QP iterations by 70-95%** (e.g., 86 → 4 iterations for 2-agent case)
- Enables fast convergence even for long horizons

### 2. OpenMP Parallelization for Distributed ADMM

Multi-agent coordination uses the Alternating Direction Method of Multipliers (ADMM), where each agent solves its own QP independently. We parallelize this using **OpenMP**:

- Agent subproblems solved in parallel: `#pragma omp parallel for`
- Automatic thread management to avoid oversubscription
- **2-3× additional speedup** from parallelization
- Scales naturally with number of agents and CPU cores

This enables real-time multi-agent MPC at 10-60 Hz control frequencies.

## Visualizations

### 4-agent trajectories visualization

![agents_trajectory](https://github.com/user-attachments/assets/a81ab824-7238-467f-98c5-b2c92fd42a8a)

### You can interact with the 6-agent scenario here:
https://claude.ai/public/artifacts/ca114b27-1b36-4d00-8cde-eb6a9f8d66c7


## Performance Evidence

The following tables compare the performance of the TurboADMM solver with and without the Riccati warm start feature across three increasingly complex collision avoidance scenarios.

### Standard ADMM with OpenMP (without Riccati Warm Start)

| Scenario | ADMM Iterations | Total QP Iterations | Solve Time | Converged |
|:---------|:---------------:|:-------------------:|:-----------|:---------:|
| 2-agent  | 2               | 86                  | 11.58 ms   | YES       |
| 4-agent  | 6               | 184                 | 23.57 ms   | YES       |
| 6-agent  | 24              | 344                 | 29.65 ms   | YES       |

### ADMM with Riccati Warm Start + OpenMP Parallelization (This Feature)

| Scenario | ADMM Iterations | Total QP Iterations | Solve Time | Converged |
|:---------|:---------------:|:-------------------:|:-----------|:---------:|
| 2-agent  | 2               | 4                   | 4.89 ms    | YES       |
| 4-agent  | 6               | 24                  | 7.31 ms    | YES       |
| 6-agent  | 24              | 104                 | 16.17 ms   | YES       |

All processing is conducted on Intel(R) Core(TM) Ultra 7 155H (22 cores).

### OSQP-based SQP (for comparison)

For reference, we also implemented a centralized approach using OSQP solver with Sequential Quadratic Programming (SQP) for collision constraint linearization:

| Scenario | SQP Iterations | Solve Time | Notes |
|:---------|:--------------:|:-----------|:------|
| 2-agent  | 3              | 3.32 ms    | ✅ Fastest for small problems |
| 4-agent  | 3              | 6.11 ms    | ✅ Competitive |
| 6-agent  | 18             | 87.99 ms   | ❌ **5.4× slower** than ADMM |

**Key Observations:**
- **Small-scale (2-4 agents):** OSQP centralized approach is competitive or slightly faster
- **Large-scale (6+ agents):** ADMM distributed approach shows clear superiority
  - 6-agent: OSQP 87.99ms vs ADMM 16.17ms (**5.4× faster with ADMM**)
- **Scalability:** ADMM scales better due to distributed nature (each agent solves independently in parallel)
- **Robustness:** ADMM with Riccati warm start provides more consistent QP iteration counts

**Why ADMM Wins at Scale:**
1. **Parallelization:** Agent subproblems solved independently (OpenMP)
2. **Smaller QPs:** Each agent solves its own small QP (124 vars) vs one large QP (744 vars for 6 agents)
3. **Warm starting:** Riccati provides excellent initialization for each agent's QP
4. **Distributed architecture:** Natural fit for multi-agent systems


## How to Verify the Results

You can download this code and run the tests on your own to validate these results.

### 1. Build the Tests

Navigate to the `tests/` directory. If you have `make` installed, you can build all tests:

```bash
cd tests
make
```

### 2. Run the Scenarios

Execute the compiled test files from the `tests/` directory:

```bash
# Run the 2-agent test
./bin/test_2agent_rho25

# Run the 4-agent test
./bin/test_4agent_rho25

# Run the 6-agent test
./bin/test_6agent
```

Each test will print a detailed analysis of the trajectory, collision avoidance, and performance statistics, allowing you to verify the results documented above.

### 3. Run OSQP-based SQP Tests (for comparison)

To reproduce the OSQP centralized SQP results:

```bash
# Navigate to build directory and configure with CMake
cd build
cmake ..
make

# Run the OSQP tests
./test_2agent_rho25_osqp
./test_4agent_rho25_osqp
./test_6agent_osqp
```

These tests demonstrate the centralized approach using OSQP solver with SQP for collision constraint linearization, allowing direct comparison with the distributed ADMM approach.