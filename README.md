![agents_trajectory](https://github.com/user-attachments/assets/f505176c-88c3-4e9e-b2e6-7d356d6bc1ca)# The Impact of Riccati Warm Start on Multi-Agent MPC

This repository demonstrates the significant performance gains achieved by using a Riccati-based primal-dual warm start for multi-agent motion planning. The evidence provided by the 2, 4, and 6-agent test scenarios shows that this feature is critical for enabling real-time, complex multi-agent control.

## 🚀 The Impact of This Work

The primary impact of this feature is that it makes **real-time, complex multi-agent MPC feasible** for a new class of applications. By reducing the core QP solver iterations by **70-95%**, this work enables:

- **Real-Time Performance:** Solving complex scenarios (e.g., 6 agents in a crossing pattern) at speeds suitable for robotics and autonomous systems.
- **Enhanced Scalability:** Managing the computational burden as the number of agents and their interactions grow, which is critical for applications like drone swarms or warehouse automation.
- **More Complex Scenarios:** Allowing users to tackle problems with longer planning horizons or a higher number of agents that were previously computationally prohibitive.
## Visualizations

### 4-agent trajectories visualization

![agents_trajectory](https://github.com/user-attachments/assets/a81ab824-7238-467f-98c5-b2c92fd42a8a)

### You can interact with the 6-agent scenario here:
https://claude.ai/public/artifacts/ca114b27-1b36-4d00-8cde-eb6a9f8d66c7


## 📊 Performance Evidence

The following tables compare the performance of the TurboADMM solver with and without the Riccati warm start feature across three increasingly complex collision avoidance scenarios.

### Standard ADMM (without Riccati Warm Start)

| Scenario | ADMM Iterations | Total QP Iterations | Solve Time | Converged |
|:---------|:---------------:|:-------------------:|:-----------|:---------:|
| 2-agent  | 2               | 86                  | 23.22 ms   | YES       |
| 4-agent  | 6               | 184                 | 52.04 ms   | YES       |
| 6-agent  | 24              | 344                 | 99.40 ms   | YES       |

### ADMM with Riccati Warm Start (This Feature)

| Scenario | ADMM Iterations | Total QP Iterations | Solve Time | Converged |
|:---------|:---------------:|:-------------------:|:-----------|:---------:|
| 2-agent  | 2               | 4                   | 6.37 ms    | YES       |
| 4-agent  | 6               | 24                  | 15.75 ms   | YES       |
| 6-agent  | 24              | 104                 | 38.49 ms   | YES       |

### Summary of Performance Gains

| Scenario | QP Iteration Reduction | Solve Time Reduction |
|:---------|:----------------------:|:--------------------:|
| 2-agent  | **95.3%** (86 → 4)     | **72.6%** (23ms → 6ms) |
| 4-agent  | **87.0%** (184 → 24)   | **69.7%** (52ms → 16ms)|
| 6-agent  | **69.8%** (344 → 104)  | **61.3%** (99ms → 38ms)  |

## 🔧 How to Verify the Results

You can download this code and run the tests on your own to validate these results.

### 1. Build the Tests

Navigate to the `tests/` directory. If you have `make` installed, you can build all tests:

```bash
cd tests
make
```

If you do not have `make`, you can compile the tests manually using `g++`. For example, to build the 6-agent test:

```bash
g++ -g -O2 -Wall -std=c++11 -fno-omit-frame-pointer -I../include -I. -o bin/test_6agent test_6agent.cpp ../src/TurboADMM.cpp ../src/QProblem.cpp ../src/QProblemB.cpp ../src/Bounds.cpp ../src/Constraints.cpp ../src/SubjectTo.cpp ../src/Indexlist.cpp ../src/Utils.cpp ../src/Options.cpp ../src/Matrices.cpp ../src/MessageHandling.cpp ../src/Flipper.cpp ../src/BLASReplacement.cpp ../src/LAPACKReplacement.cpp -lm
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
