# TurboADMM
### Riccati-Accelerated Distributed Multi-Agent Planning

> Real-time trajectory optimization for multi-agent systems achieving over **5x speedup** over OSQP based centralized methods

[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![OpenMP](https://img.shields.io/badge/OpenMP-Parallel-green.svg)](https://www.openmp.org/)
[![qpOASES](https://img.shields.io/badge/qpOASES-MPC--Aware-orange.svg)](https://github.com/coin-or/qpOASES)

---
## What is TurboADMM?

TurboADMM is a high-performance trajectory optimization engine designed for **real-time multi-agent control**. Built on top of **qpOASES** (a parametric active-set QP solver), it leverages MPC-aware hotstart and distributed optimization. Instead of solving one massive centralized optimization problem, each agent solves its own small problem in parallel, coordinated through the Alternating Direction Method of Multipliers (ADMM).

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
./test_2agent_rho25_osqp
./test_4agent_rho25_osqp
./test_6agent_osqp
```

These demonstrate the centralized approach for direct performance comparison.

### Expected Output

Each test will display:
- ADMM convergence status and iteration count
- Total solve time and QP iterations
- Trajectory tracking accuracy
- Collision avoidance verification (minimum distances)
- Performance breakdown by agent