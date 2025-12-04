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

The following tables compare the performance of the TurboADMM solver with and without the Riccati warm start feature across three increasingly complex collision avoidance scenarios.

### BaseADMM: Standard qpOASES + Distributed ADMM + OpenMP Parallelization

| Scenario | ADMM Iters | Total QP Iters | Solve Time | Converged |
|:---------|:----------:|:--------------:|:-----------|:---------:|
| 2-agent  | 2          | 86             | 11.58 ms   |    YES    |
| 4-agent  | 6          | 184            | 23.57 ms   |    YES    |
| 6-agent  | 24         | 344            | 29.65 ms   |    YES    |
| 8-agent  | 46         | 550            | 56.52 ms   |    YES    |

### TurboADMM: Riccati-Accelerated qpOASES + Distributed ADMM + OpenMP Parallelization

| Scenario | ADMM Iters | Total QP Iters | Solve Time | Converged |
|:---------|:----------:|:--------------:|:-----------|:---------:|
| 2-agent  | 2          | 4              | 4.89 ms    |    YES    |
| 4-agent  | 6          | 24             | 7.31 ms    |    YES    |
| 6-agent  | 24         | 104            | 16.17 ms   |    YES    |
| 8-agent  | 46         | 230            | 33.53 ms   |    YES    |

### Comparison with Industry-Leading Generic QP Solvers

To demonstrate TurboADMM's domain-specific advantages, we compare against two top-tier generic sparse QP solvers using centralized SQP formulations:

#### OSQP-based SQP

[**OSQP**](https://osqp.org/) is a widely-used open-source operator splitting solver, known for its robustness and efficiency on large-scale sparse problems:

| Scenario | SQP Iters | Solve Time  | Converged | Notes                           |
|:---------|:---------:|:------------|:---------:|:--------------------------------|
| 2-agent  | 3         | 3.32 ms     |    YES    | Faster than TurboADMM           |
| 4-agent  | 3         | 6.11 ms     |    YES    | Competitive with TurboADMM      |
| 6-agent  | 18        | 81.45 ms    |    YES    | **5.0× slower** than TurboADMM  |
| 8-agent  | 26        | 209.07 ms   |    YES    | **6.2× slower** than TurboADMM  |

#### MOSEK-based SQP

[**MOSEK**](https://www.mosek.com/) is a state-of-the-art commercial interior-point solver, widely regarded as one of the fastest and most reliable solvers for convex optimization:

| Scenario | SQP Iters | Solve Time | Converged | Notes                            |
|:---------|:---------:|:-----------|:---------:|:---------------------------------|
| 2-agent  | 2         | 55.89 ms   |    YES    | **11.42× slower** than TurboADMM |
| 4-agent  | 2         | 95.41 ms   |    YES    | **13.05× slower** than TurboADMM |
| 6-agent  | 8         | 320.23 ms  |    YES    | **19.8× slower** than TurboADMM  |
| 8-agent  | 7         | 388.27 ms  |    YES    | **11.6× slower** than TurboADMM  |

All processing is conducted on Intel(R) Core(TM) i7-155H (22 cores).

**Key Observations:**
- **Small-scale (2-4 agents):** OSQP centralized approach is competitive or slightly faster
- **Large-scale (6+ agents):** ADMM distributed approach shows clear superiority
  - 6-agent: OSQP 87.99ms vs TurboADMM 16.17ms
  - 6-agent: MOSEK 332.35ms vs TurboADMM 16.17ms
- **Scalability:** ADMM scales better due to distributed nature (each agent solves independently in parallel)
- **Solver Choice:** Interior-point methods (MOSEK) have higher per-iteration cost than active-set methods (OSQP, qpOASES)

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
./bin/test_2agent_rho25   # 2-agent collision avoidance
./bin/test_4agent_rho25   # 4-agent collision avoidance
./bin/test_6agent         # 6-agent collision avoidance
```

Each test outputs detailed performance metrics, QP iteration counts, and collision avoidance verification.

**Run OSQP Comparison Tests**

```bash
mkdir build && cd build
cmake ..
make

# Run OSQP centralized SQP tests
./bin/test_2agent_rho25_osqp
./bin/test_4agent_rho25_osqp
./bin/test_6agent_osqp
```

**Run MOSEK Comparison Tests**

```bash
# Run MOSEK centralized SQP tests
./bin/test_2agent_rho25_mosek
./bin/test_4agent_rho25_mosek
./bin/test_6agent_mosek
```

These demonstrate the centralized approach for direct performance comparison.

## You are more than welcomed to contribute via:

- Improving TurboADMM's performance and scalability
- Adding advanced SQP tricks (e.g. metric function, filters, line search, etc.) to current basic implementation
- Testing on larger scenarios (10+, 20+, 50+ agents)
- Extending to different robot dynamics (quadrotors, car-like, manipulators)
- Implementing algorithmic improvements (adaptive ADMM, robust MPC, etc.)

