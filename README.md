# TurboADMM
### Riccati-Accelerated Distributed Multi-Agent Inference Engine

> Real-time trajectory optimization for multi-agent systems achieving over **5x speedup** compared to top-tier centralized methods

[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![OpenMP](https://img.shields.io/badge/OpenMP-Parallel-green.svg)](https://www.openmp.org/)
[![qpOASES](https://img.shields.io/badge/qpOASES-MPC--Aware-orange.svg)](https://github.com/coin-or/qpOASES)

---
## What is TurboADMM?

TurboADMM is a high-performance inference engine designed for **real-time multi-agent control**. Built on top of **qpOASES** (a parametric active-set QP solver), it leverages MPC-aware hotstart and distributed optimization. Instead of solving one massive centralized optimization problem, each agent solves its own small problem in parallel, coordinated through the Alternating Direction Method of Multipliers (ADMM).

**Key Features:**
- **MPC-Aware QP Solving:** Exploits Model Predictive Control structure using Riccati recursion
- **Distributed Architecture:** Each agent solves independently, enabling parallelization
- **OpenMP Parallelization:** Automatic multi-threading for agent subproblems
- **Embeddable Engine:** Designed to be integrated into larger planning systems

## Methodology

### 1. Riccati Warm Start for MPC-Aware QP Solving

Each agent's trajectory optimization is formulated as a Model Predictive Control (MPC) problem solved via quadratic programming (QP). We exploit MPC structure using **Riccati recursion** to solve a pure affine LQR as auxiliary QP for qpOASES initialization.


### 2. OpenMP Parallelization for Distributed ADMM

Multi-agent coordination uses the Alternating Direction Method of Multipliers (ADMM), where each agent solves its own QP independently. Agent subproblems are solved in parallel using OpenMP with automatic thread management.

### 3. Exploit Sparsity of Active Constraints in QR Factorization

Skip Givens rotations in QR factorization when the corresponding entry is zero.


## Performance Evidence

The following tables compare the performance of TurboADMM across different optimization configurations, from baseline to fully accelerated.

### BaseADMM: qpOASES(No Hotstart) + ADMM + OpenMP 

| Scenario | ADMM Iters | Total QP Iters | Solve Time  | Converged | Tracking Error (m) |
|:---------|:----------:|:--------------:|:------------|:---------:|:------------------:|
| 2-agent  | 2          | 170            | 24.27 ms    |    YES    | 0.010 ± 0.000      |
| 4-agent  | 6          | 1023           | 135.21 ms   |    YES    | 0.010 ± 0.000      |
| 6-agent  | 24         | 6843           | 792.38 ms   |    YES    | 0.171 ± 0.128      |
| 10-agent | 55         | 32191          | 3523.45 ms  |    YES    | 0.490 ± 0.350      |
| 14-agent | 144        | 109512         | 14275.94 ms |    YES    | 0.508 ± 0.786      |

### HotstartADMM: qpOASES + Hotstart + ADMM + OpenMP 

| Scenario | ADMM Iters | Total QP Iters | Solve Time | Converged | Tracking Error (m) |
|:---------|:----------:|:--------------:|:-----------|:---------:|:------------------:|
| 2-agent  | 2          | 86             | 11.58 ms   |    YES    | 0.010 ± 0.000      |
| 4-agent  | 6          | 184            | 23.57 ms   |    YES    | 0.010 ± 0.000      |
| 6-agent  | 24         | 344            | 29.65 ms   |    YES    | 0.171 ± 0.128      |
| 10-agent | 53         | 923            | 78.14 ms   |    YES    | 0.490 ± 0.350      |
| 14-agent | 144        | 1014           | 120.87 ms  |    YES    | 0.508 ± 0.786      |

### TurboADMM: Riccati-Accelerated qpOASES + Hotstart + ADMM + OpenMP 

| Scenario | ADMM Iters | Total QP Iters | Solve Time | Converged | Tracking Error (m) |
|:---------|:----------:|:--------------:|:-----------|:---------:|:------------------:|
| 2-agent  | 2          | 4              | 4.89 ms    |    YES    | 0.010 ± 0.000      |
| 4-agent  | 6          | 24             | 7.31 ms    |    YES    | 0.010 ± 0.000      |
| 6-agent  | 24         | 104            | 16.17 ms   |    YES    | 0.171 ± 0.128      |
| 10-agent | 39         | 368            | 39.46 ms   |    YES    | 0.490 ± 0.350      |
| 14-agent | 144        | 476            | 96.12 ms   |    YES    | 0.508 ± 0.786      |

### Comparison with Industry-Leading Generic QP Solvers

To demonstrate TurboADMM's domain-specific advantages, we compare against two top-tier generic sparse QP solvers using centralized SQP formulations:

#### OSQP-based SQP

[**OSQP**](https://osqp.org/) is a widely-used open-source operator splitting solver, known for its robustness and efficiency on large-scale sparse problems:

| Scenario | SQP Iters | Solve Time  | Converged | Tracking Error (m) | Notes                           |
|:---------|:---------:|:------------|:---------:|:------------------:|:--------------------------------|
| 2-agent  | 3         | 3.32 ms     |    YES    | 0.123 ± 0.000      | Faster than TurboADMM           |
| 4-agent  | 3         | 6.11 ms     |    YES    | 0.141 ± 0.020      | Competitive with TurboADMM      |
| 6-agent  | 18        | 81.45 ms    |    YES    | 0.454 ± 0.154      | **5.0× slower** than TurboADMM  |
| 10-agent | 35        | 390.79 ms   |    YES    | 0.738 ± 0.392      | **6.7× slower** than TurboADMM  |
| 14-agent | 54        | 1420.75 ms  |    YES    | 0.979 ± 1.783      | **14.8× slower** than TurboADMM |

#### MOSEK-based SQP

[**MOSEK**](https://www.mosek.com/) is a state-of-the-art commercial interior-point solver, widely regarded as one of the fastest and most reliable solvers for convex optimization:

| Scenario | SQP Iters | Solve Time | Converged | Tracking Error (m) | Notes                            |
|:---------|:---------:|:-----------|:---------:|:------------------:|:---------------------------------|
| 2-agent  | 2         | 55.89 ms   |    YES    | 0.121 ± 0.000      | **11.4× slower** than TurboADMM |
| 4-agent  | 2         | 95.41 ms   |    YES    | 0.126 ± 0.020      | **13.1× slower** than TurboADMM |
| 6-agent  | 8         | 320.23 ms  |    YES    | 0.451 ± 0.165      | **19.8× slower** than TurboADMM  |
| 10-agent | 15        | 755.72 ms  |    YES    | 0.561 ± 0.364      | **13.2× slower** than TurboADMM  |
| 14-agent | 28        | 2218.32 ms |    YES    | 0.168 ± 0.091      | **23.1× slower** than TurboADMM  |

#### HPIPM-based SQP

[**HPIPM**](https://github.com/giaf/hpipm) is a high-performance interior-point method solver specifically designed for optimal control problems with MPC structure:

| Scenario            | SQP Iters | Total QP Iters | Solve Time | Converged | Tracking Error (m) | Notes                                     |
|:--------------------|:---------:|:--------------:|:-----------|:---------:|:------------------:|:------------------------------------------|
| 2-agent (SPEED_ABS) | 2         | 30             | 3.65 ms    |    YES    | 0.31 ± 0.000       | Fast mode, converges                      |
| 2-agent (BALANCE)   | 2         | 31             | 4.77 ms    |    YES    | 0.31 ± 0.000       | Robust mode, +31% slower                  |
| 4-agent             | 0         | 11             | -          |  **NO**   | -                  | **Fails: Minimum step length (status 2)** |
| 6-agent             | 0         | 11             | -          |  **NO**   | -                  | **Fails: Minimum step length (status 2)** |

**Configuration:** Hard constraints (no slack variables), matching OSQP setup for fair comparison  
**Initialization:** Staggered start (2-agent) or manually designed feasible curves (4+ agents)

**Key Findings:**
- ✅ **2 agents:** HPIPM converges successfully with hard constraints, competitive performance (3.65-4.77ms)
- ❌ **4+ agents:** HPIPM fails even with carefully designed feasible initialization (manually designed collision avoidance curves)
- **Failure mode:** Interior point method encounters minimum step length (alpha < alpha_min), returns nonsensical solution
- **Root cause:** Lack of feasibility restoration mechanisms and homogeneous embedding makes HPIPM sensitive to the number and dynamics of active collision constraints
- **Residuals at failure:** Complementarity: 417, Stationarity: 159, Equality/Inequality: 14.8 (all exceed tolerance by 2-4 orders of magnitude)

**Conclusion:** HPIPM is unsuitable for multi-agent collision avoidance beyond 2-3 agents. ADMM-based solvers (TurboADMM, OSQP) handle large-scale problems robustly through progressive feasibility enforcement.

All processing is conducted on Intel(R) Core(TM) i7-155H (22 cores).
**Note:** All solvers (ADMM and SQP outer loops) use identical convergence criterion: relative objective change < 1e-4, and collision free.
**Note:** Tracking error standards for mean and standard dev of final state tracking errorover all agents.

### Problem Dimensions and Scalability

The following table shows how the problem size scales with the number of agents (N=20 horizon, nx=4 states, nu=2 controls):

| Scenario | Variables | Dynamics Constraints | Collision Constraints | Total Constraints | Pairs |
|:---------|----------:|---------------------:|----------------------:|------------------:|------:|
| 2-agent  | 248       | 160                  | 21                    | 181               | 1     |
| 4-agent  | 496       | 320                  | 126                   | 446               | 6     |
| 6-agent  | 744       | 480                  | 315                   | 795               | 15    |
| 10-agent | 1,240     | 800                  | 945                   | 1,745             | 45    |
| 14-agent | 1,736     | 1,120                | 1,911                 | 3,031             | 91    |

**Key Insights:**
- **Variables scale linearly:** Each agent adds 124 variables ((N+1)×nx + N×nu = 21×4 + 20×2 = 124)
- **Dynamics constraints scale linearly:** Each agent adds 80 constraints (N×nx = 20×4 = 80)
- **Collision constraints scale quadratically:** C(n,2) = n(n-1)/2 pairs, each with (N+1) = 21 timesteps
- **14-agent problem:** 1,736 variables, 3,031 constraints - solved in **96.12 ms** by TurboADMM vs **1,420.75 ms** by OSQP vs **2,218.32 ms** by MOSEK

## Visualizations

### 4-Agent Collision Avoidance Trajectories

![agents_trajectory](https://github.com/user-attachments/assets/a81ab824-7238-467f-98c5-b2c92fd42a8a)

*Four agents successfully navigate to their targets while maintaining safe distances. The distributed ADMM approach coordinates their trajectories in real-time.*

### Interactive 6-Agent Demo

Experience the 6-agent scenario in action: [**Interactive Demo**](https://claude.ai/public/artifacts/ca114b27-1b36-4d00-8cde-eb6a9f8d66c7)

*Click to explore the full 6-agent collision avoidance scenario with real-time trajectory visualization.*

## Quick Start

### Prerequisites

- C++17 compiler (GCC, Clang, or MSVC)
- CMake 3.10+
- OpenMP support
- Make (for tests directory)
- OSQP installed (for comparison tests)
- MOSEK installed (for comparison tests)

### Building and Running Tests

**Run TurboADMM Tests**

```bash
cd tests
make

# Run the test scenarios
./bin/test_2agent   # 2-agent collision avoidance
./bin/test_4agent   # 4-agent collision avoidance
./bin/test_6agent   # 6-agent collision avoidance
./bin/test_10agent  # 10-agent collision avoidance
./bin/test_14agent  # 14-agent collision avoidance
```

Each test outputs detailed performance metrics, QP iteration counts, and collision avoidance verification.

**Run OSQP Comparison Tests**

```bash
mkdir build && cd build
cmake ..
make

# Run OSQP centralized SQP tests
./bin/test_2agent_osqp
./bin/test_4agent_osqp
./bin/test_6agent_osqp
./bin/test_10agent_osqp
./bin/test_14agent_osqp
```

**Run MOSEK Comparison Tests**

```bash
# Run MOSEK centralized SQP tests
./bin/test_2agent_mosek
./bin/test_4agent_mosek
./bin/test_6agent_mosek
./bin/test_10agent_mosek
./bin/test_14agent_mosek
```

**Run Time Comparsion Tests**
```bash
./benchmark_tests.sh
```

**HPIPM Convergence Analysis/GLPK feasibility check**
```bash
./hpipmSovlerData/lp_feasibility_test_glpk.m
```

These demonstrate the centralized approach for direct performance comparison.

## You are more than welcomed to contribute via:

- Improving TurboADMM's performance and scalability
- Adding advanced SQP tricks (e.g. metric function, filters, line search, etc.) to current basic implementation
- Testing on larger scenarios (10+, 20+, 50+ agents)
- Extending to different robot dynamics (quadrotors, car-like, manipulators)
- Implementing algorithmic improvements (adaptive ADMM, robust MPC, etc.)

