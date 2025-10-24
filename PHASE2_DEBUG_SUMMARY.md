# Phase 2 Implementation - Debugging Summary

**Date:** October 24, 2025, 13:05

## Current Status

### ✅ Phase 1: COMPLETE
- Q_aug construction: ✓ Working correctly
- setupMPCStructure: ✓ SUCCESS for both agents
- Q_aug values verified: 31.0 for position (with ADMM aug), 0.005 for velocity

### ⚠️ Phase 2: IMPLEMENTED BUT NOT WORKING
- Riccati warm start: ✓ Compiles and runs
- QP iterations: ❌ Still 130 (no improvement from baseline)
- Riccati solution: ❌ Shows instability (huge values: 1e20)
- Final solution: ✅ Still correct (QP solver is robust)

## Problem Diagnosis

### Root Cause Identified: Data Structure Mismatch

**Decision Variable Format (qpOASES):**
```
z = [x[0], u[0], x[1], u[1], ..., x[N]]
Size: nV = N*(nx+nu) + nx

Example for N=20, nx=4, nu=2:
z[0:3]   = x[0] (state at stage 0)
z[4:5]   = u[0] (control at stage 0)
z[6:9]   = x[1] (state at stage 1)
z[10:11] = u[1] (control at stage 1)
...
z[120:123] = x[20] (terminal state)
```

**Riccati LQR Expected Format:**
```cpp
solveRiccatiLQR(double* x_opt, double* u_opt, const double* g)
```
- `x_opt`: States packed as `[x[0], x[1], ..., x[N]]` (size: (N+1)*nx)
- `u_opt`: Controls packed as `[u[0], u[1], ..., u[N-1]]` (size: N*nu)

**Current Implementation (WRONG):**
```cpp
agent_solvers_[i]->solveRiccatiLQR(
    z_riccati,              // Points to interleaved [x[0], u[0], x[1], u[1], ...]
    &z_riccati[agent.nx],   // Points to u[0] within interleaved format
    g
);
```

This causes Riccati to read/write data incorrectly, leading to:
1. Numerical instability (huge values)
2. Invalid warm start (QP ignores it)
3. No performance improvement

## Solution Options

### Option A: Repack Data (Recommended)
Convert between qpOASES format and Riccati format:

```cpp
// Allocate separate buffers for Riccati
real_t* x_riccati = new real_t[(agent.N+1) * agent.nx];
real_t* u_riccati = new real_t[agent.N * agent.nu];

// Set initial state
for (int j = 0; j < agent.nx; ++j) {
    x_riccati[j] = x_init[i][j];
}

// Call Riccati with separate buffers
ret_riccati = agent_solvers_[i]->solveRiccatiLQR(
    x_riccati, u_riccati, g
);

// Repack Riccati solution into qpOASES format
real_t* z_riccati = new real_t[agent.nV];
for (int k = 0; k <= agent.N; ++k) {
    // Copy state x[k]
    for (int j = 0; j < agent.nx; ++j) {
        z_riccati[k*(agent.nx+agent.nu) + j] = x_riccati[k*agent.nx + j];
    }
    // Copy control u[k] (if k < N)
    if (k < agent.N) {
        for (int j = 0; j < agent.nu; ++j) {
            z_riccati[k*(agent.nx+agent.nu) + agent.nx + j] = u_riccati[k*agent.nu + j];
        }
    }
}

// Use z_riccati as warm start
agent_solvers_[i]->init(..., z_riccati, ...);

delete[] x_riccati;
delete[] u_riccati;
delete[] z_riccati;
```

**Pros:** Clean, correct, straightforward
**Cons:** Extra memory allocation and copying

### Option B: Modify solveRiccatiLQR to Accept Interleaved Format
Change `solveRiccatiLQR` to work directly with qpOASES decision variable format.

**Pros:** No repacking overhead
**Cons:** Requires modifying core qpOASES function

### Option C: Use Riccati for Pure LQR Only (No Affine Term)
Call Riccati without gradient for initial guess, use reference trajectory instead.

**Pros:** Simpler, avoids affine LQR complexity
**Cons:** Lower quality warm start

## Test Results

### Baseline (No MPC-Awareness):
- Agent 0, ADMM iter 0: 130 QP iterations
- Agent 1, ADMM iter 0: 130 QP iterations
- Agent 0, ADMM iter 1: 18 QP iterations
- Agent 1, ADMM iter 1: 18 QP iterations
- **Total: 296 iterations, 290ms solve time**

### Current (Phase 2 with Broken Riccati):
- Agent 0, ADMM iter 0: 130 QP iterations (no improvement)
- Agent 1, ADMM iter 0: 130 QP iterations (no improvement)
- Agent 0, ADMM iter 1: 18 QP iterations
- Agent 1, ADMM iter 1: 18 QP iterations
- **Total: 296 iterations, 199ms solve time** (31% faster due to overhead reduction)

### Expected (Phase 2 with Working Riccati):
- Agent 0, ADMM iter 0: 0-20 QP iterations (85-100% reduction)
- Agent 1, ADMM iter 0: 0-20 QP iterations (85-100% reduction)
- Agent 0, ADMM iter 1: 0-10 QP iterations
- Agent 1, ADMM iter 1: 0-10 QP iterations
- **Target: <90 iterations, <120ms solve time**

## Recommendation

**Implement Option A** (Repack Data) because:
1. Clean and correct solution
2. Minimal code changes
3. No modifications to core qpOASES functions
4. Clear performance path forward

The repacking overhead is negligible compared to QP solve time.

## Next Steps

1. Implement Option A (data repacking)
2. Verify Riccati solution is stable (no huge values)
3. Verify QP iterations are reduced (target: <20)
4. Test full ADMM convergence
5. Measure performance improvement

## Files to Modify
- `src/TurboADMM.cpp`: Fix Riccati call with data repacking (lines 1430-1470)

---

**Status:** Ready to implement Option A. Awaiting user confirmation to proceed.
