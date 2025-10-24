# MPC-Aware ADMM Implementation Review & Validation Plan

**Date:** October 23, 2025  
**Baseline Performance:** 290ms, 296 QP iterations (130+130+18+18)  
**Target Performance:** <100ms, <90 QP iterations

---

## 1. FUNCTION REVIEW: setupMPCStructure()

### Implementation Summary:
- **Location:** `src/QProblem.cpp` lines 6810-6860
- **Purpose:** Store MPC structure (A, B, Q, R) and allocate working memory
- **Does NOT compute TQ factorization** - only stores matrices

### Key Observations:
```cpp
returnValue QProblem::setupMPCStructure(int_t _N, int_t _nx, int_t _nu,
                                        const real_t* _A, const real_t* _B,
                                        const real_t* _Q, const real_t* _R)
{
    // 1. Validates inputs
    // 2. Frees existing MPC memory
    // 3. Copies A, B, Q, R to mpcData
    // 4. Sets mpcData.isInitialized = BT_TRUE
    // 5. Allocates working memory
    // 6. Does NOT call setupMPCTQfactorisation()!
}
```

### CRITICAL ISSUES IDENTIFIED:

#### ❌ Issue 1: Does NOT Call setupMPCTQfactorisation()
**Problem:** `setupMPCStructure()` only stores matrices, doesn't compute T, Q factorization.

**From Memory:** The TQ factorization must be computed AFTER `setupAuxiliaryWorkingSet()` marks dynamics constraints as active.

**Solution:** We must call `setupMPCTQfactorisation()` separately, AFTER calling `init()` which sets up the auxiliary working set.

**Correct Sequence:**
```cpp
// Step 1: Setup MPC structure (stores A, B, Q, R)
agent_solvers_[i]->setupMPCStructure(N, nx, nu, A, B, Q_aug, R);

// Step 2: Call init() - this calls setupAuxiliaryWorkingSet() internally
agent_solvers_[i]->init(H, g, A_constraint, lb, ub, lbA, ubA, nWSR);

// Step 3: Now TQ factorization is computed during init()
// (init() calls setupAuxiliaryWorkingSet() then setupMPCTQfactorisation())
```

#### ✅ Issue 2: Q_aug Must Match Hessian Exactly
**Requirement:** The Q matrix passed to `setupMPCStructure` must match the diagonal of the Hessian.

**For ADMM:**
```cpp
Q_aug[j,j] = stage_weight * Q[j,j] + ρ * num_neighbors  (for position states)
Q_aug[j,j] = stage_weight * Q[j,j]                       (for velocity states)
```

**Validation Check:**
```cpp
// After building Q_aug and Hessian
for (int j = 0; j < nx; ++j) {
    real_t q_aug_val = stage_weight * Q[j*nx+j];
    if (j < 2 && num_neighbors > 0) {
        q_aug_val += rho * num_neighbors;
    }
    assert(fabs(Q_aug[j*nx+j] - q_aug_val) < 1e-10);
}
```

---

## 2. FUNCTION REVIEW: setupMPCTQfactorisation()

### Implementation Summary:
- **Location:** `src/QProblem.cpp` lines 7357-7543 (truncated in view)
- **Purpose:** Compute O(N) TQ factorization for MPC structure
- **Requires:** Active constraints must be set up first

### Key Observations:

#### ✅ Feature 1: Horizon Size Check
```cpp
if (N > 40) {
    return RET_MPC_TQ_FACTORIZATION_FAILED;  // Fallback to standard
}
```
**Good:** Prevents memory issues for large horizons.  
**Our case:** N=20, so this won't trigger.

#### ✅ Feature 2: Requires Active Constraints
```cpp
if (constraints.getNC() <= 0)
    return SUCCESSFUL_RETURN;  // No constraints to factorize

int nAC = getNAC();
if (nAC == 0)
    return SUCCESSFUL_RETURN;  // No active constraints yet
```
**Critical:** This is why we must call `setupAuxiliaryWorkingSet()` FIRST!

#### ⚠️ Issue 3: Incomplete Implementation
The function appears to be truncated in the view. Need to verify:
- Does it complete the TQ factorization?
- Does it handle all N stages correctly?
- Does it store T and Q matrices properly?

**Action:** Review complete implementation tomorrow.

---

## 3. FUNCTION REVIEW: solveRiccatiLQR()

### Implementation Summary:
- **Location:** `src/QProblem.cpp` lines 6888-7250+
- **Purpose:** Solve affine LQR using backward Riccati recursion
- **Supports:** Linear cost term (gradient g) for affine LQR

### Key Observations:

#### ✅ Feature 1: Affine LQR Support
```cpp
real_t* k_affine = 0;
if (g != 0) {
    k_affine = new real_t[(N-1) * nu];
    // Compute affine feedback term
}
```
**Excellent:** Handles ADMM gradient terms correctly!

#### ✅ Feature 2: Backward Riccati Recursion
```cpp
// Terminal condition: P_N = Q
P[(N-1)*nx*nx + i] = mpcData.Q[i];

// Backward pass: k = N-2, ..., 0
for (int k = N-2; k >= 0; --k) {
    // Compute P_k = Q + A^T P_{k+1} A - A^T P_{k+1} B K_k
    // Compute K_k = (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A
    // Compute k_k = Sinv × (B^T v_{k+1} + g_u_k)  [affine term]
}
```
**Correct:** Standard Riccati recursion with affine feedback.

#### ✅ Feature 3: Forward Pass with Initial State Preservation
```cpp
/* x_opt[0:nx-1] already contains the initial state from the caller - don't overwrite it! */
// Forward rollout: x_{k+1} = A x_k + B u_k
for (int k = 0; k < N-1; ++k) {
    u_opt[k*nu + i] = -K_k[i*nx + j] * x_opt[k*nx + j];
    if (k_affine) u_opt[k*nu + i] -= k_k[i];  // Add affine term
    // Propagate dynamics
}
```
**Good:** Preserves initial state from caller (fixed in previous session).

#### ⚠️ Issue 4: Gradient Indexing
```cpp
const real_t* g_x_k = &g[k * (nx + nu)];      // State gradient at stage k
const real_t* g_u_k = &g[k * (nx + nu) + nx]; // Control gradient at stage k
```
**Critical:** Assumes gradient is packed as `[g_x[0], g_u[0], g_x[1], g_u[1], ...]`

**Our ADMM gradient structure:** Need to verify this matches!
- We build gradient as: `g[k*(nx+nu) + j]` for states
- And: `g[k*(nx+nu) + nx + j]` for controls

**Validation:** Check gradient construction in TurboADMM.cpp matches this format.

#### ⚠️ Issue 5: Cholesky Decomposition for S Inversion
```cpp
// Invert S = R + B^T P_{k+1} B using Cholesky
if (sum <= 0.0) {
    return THROWERROR(RET_HESSIAN_NOT_SPD);
}
```
**Potential Issue:** If S is not positive definite, Riccati fails.

**For ADMM:** S should always be SPD because:
- R > 0 (control cost)
- B^T P B ≥ 0 (positive semi-definite)
- S = R + B^T P B > 0

**Validation:** Monitor for RET_HESSIAN_NOT_SPD errors.

---

## 4. CRITICAL CONSISTENCY REQUIREMENTS

### Requirement 1: Q_aug Construction
```cpp
// Must match EXACTLY between setupMPCStructure and Hessian
real_t stage_weight = 0.5;
real_t rho_aug = params_.rho * num_neighbors;

for (int j = 0; j < nx; ++j) {
    Q_aug[j*nx + j] = stage_weight * Q[j*nx + j];
    if (j < 2 && num_neighbors > 0) {  // Position states only
        Q_aug[j*nx + j] += rho_aug;
    }
}
```

### Requirement 2: Gradient Format for Riccati
```cpp
// Gradient must be packed as: [g_x[0], g_u[0], g_x[1], g_u[1], ..., g_x[N], g_u[N-1]]
// Total size: N*(nx+nu) + nx

// For ADMM:
// g = q - ρP^T v + P^T λ
// Where q is the tracking gradient
```

### Requirement 3: Initial State in Riccati
```cpp
// Before calling solveRiccatiLQR:
real_t* z_riccati = new real_t[nV];
// Set initial state
for (int j = 0; j < nx; ++j) {
    z_riccati[j] = x_init[i][j];
}

// Call Riccati (will preserve z_riccati[0:nx-1])
solveRiccatiLQR(z_riccati, &z_riccati[nx], g);
```

### Requirement 4: Initialization Sequence
```cpp
// CORRECT ORDER (from memory):
// 1. setupMPCStructure(A, B, Q_aug, R)  - stores matrices
// 2. init(H, g, A, lb, ub, lbA, ubA)    - calls setupAuxiliaryWorkingSet() then setupMPCTQfactorisation()
// 3. hotstart(g, lb, ub, lbA, ubA)      - reuses TQ factorization

// WRONG ORDER (causes error 68):
// 1. setupMPCStructure(A, B, Q_aug, R)
// 2. setupMPCTQfactorisation()          - NO ACTIVE CONSTRAINTS YET!
// 3. init(...)                          - TQ factorization is wrong
```

---

## 5. IMPLEMENTATION PLAN (REVISED)

### Phase 1: Setup MPC Structure in setupAgents()

```cpp
// In TurboADMM::setupAgents()
for (int i = 0; i < num_agents_; ++i) {
    AgentData& agent = agents_[i];
    int num_neighbors_i = num_neighbors_[i];
    
    // Build Q_aug
    real_t* Q_aug = new real_t[agent.nx * agent.nx];
    memset(Q_aug, 0, agent.nx * agent.nx * sizeof(real_t));
    
    real_t stage_weight = 0.5;
    for (int j = 0; j < agent.nx; ++j) {
        Q_aug[j * agent.nx + j] = stage_weight * agent.Q[j * agent.nx + j];
    }
    
    if (num_neighbors_i > 0 && params_.enable_collision_avoidance) {
        real_t rho_aug = params_.rho * num_neighbors_i;
        for (int j = 0; j < 2 && j < agent.nx; ++j) {
            Q_aug[j * agent.nx + j] += rho_aug;
        }
    }
    
    // Store Q_aug (need to keep allocated)
    agent_Q_aug_[i] = Q_aug;
    
    // Enable MPC-aware features
    Options opts;
    opts.setToDefault();
    opts.enableMPCRiccati = BT_TRUE;
    opts.enableEqualities = BT_TRUE;
    opts.printLevel = PL_NONE;
    agent_solvers_[i]->setOptions(opts);
    
    // Setup MPC structure (stores A, B, Q_aug, R)
    returnValue ret = agent_solvers_[i]->setupMPCStructure(
        agent.N, agent.nx, agent.nu,
        agent.A, agent.B, Q_aug, agent.R
    );
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("[ERROR] Agent %d: setupMPCStructure failed with code %d\n", i, ret);
        // Disable MPC features and fall back
        opts.enableMPCRiccati = BT_FALSE;
        agent_solvers_[i]->setOptions(opts);
    }
    
    printf("[DEBUG] Agent %d: setupMPCStructure complete, Q_aug[0,0]=%.2f\n", i, Q_aug[0]);
}
```

### Phase 2: Use Riccati Warm Start in init() (ADMM iter 0)

```cpp
// In solveColdStart(), ADMM iter 0
// Build gradient g (already done)

// Solve affine LQR with Riccati
real_t* z_riccati = new real_t[agent.nV];

// Set initial state in z_riccati
for (int j = 0; j < agent.nx; ++j) {
    z_riccati[j] = x_init[i][j];
}

// Call Riccati (will preserve initial state)
returnValue ret_riccati = agent_solvers_[i]->solveRiccatiLQR(
    z_riccati,              // x_opt (input: x[0], output: full trajectory)
    &z_riccati[agent.nx],   // u_opt (output: controls)
    g                       // Gradient for affine LQR
);

if (ret_riccati != SUCCESSFUL_RETURN) {
    printf("[WARNING] Agent %d: Riccati failed with code %d\n", i, ret_riccati);
    // Use zero initialization
    memset(z_riccati, 0, agent.nV * sizeof(real_t));
    for (int j = 0; j < agent.nx; ++j) {
        z_riccati[j] = x_init[i][j];  // Preserve initial state
    }
}

// Call init() with Riccati warm start
// NOTE: init() will call setupAuxiliaryWorkingSet() then setupMPCTQfactorisation()
ret = agent_solvers_[i]->init(
    H, g, A_constraint,
    lb_local, ub_local,
    lbA_combined, ubA_combined,
    nWSR,
    nullptr,      // cputime
    z_riccati,    // x0: Riccati solution as warm start
    nullptr,      // y0: Dual variables
    nullptr,      // guessedBounds
    nullptr       // guessedConstraints
);

delete[] z_riccati;
```

### Phase 3: Verify Hotstart Consistency (ADMM iter ≥ 1)

```cpp
// In solveColdStart(), ADMM iter ≥ 1
// H and A_constraint are still valid (not deleted)
// TQ factorization is still valid (computed in init())

// Just call hotstart with updated gradient
ret = agent_solvers_[i]->hotstart(
    g,  // Updated gradient
    lb_local, ub_local,
    lbA_combined, ubA_combined,
    nWSR
);
```

---

## 6. VALIDATION CHECKLIST

### Pre-Implementation Validation:
- [ ] Review complete `setupMPCTQfactorisation()` implementation
- [ ] Verify gradient format matches Riccati expectations
- [ ] Check that stage_weight is consistent everywhere
- [ ] Confirm ADMM augmentation formula is correct

### During Implementation:
- [ ] Log Q_aug values and compare with Hessian diagonal
- [ ] Log Riccati solution and check for NaN/Inf
- [ ] Log QP iteration counts (expect 0-20 for init)
- [ ] Monitor for error codes (68, RET_HESSIAN_NOT_SPD, etc.)

### Post-Implementation:
- [ ] Run 5 test iterations and verify consistency
- [ ] Compare QP iterations: baseline vs MPC-aware
- [ ] Verify solution quality (tracking + collision avoidance)
- [ ] Measure solve time improvement

---

## 7. EXPECTED ISSUES & DEBUGGING STRATEGY

### Issue 1: Error 68 Returns
**Symptom:** Division by zero in TQ factorization  
**Cause:** Q_aug doesn't match Hessian  
**Debug:**
```cpp
printf("[DEBUG] Q_aug[0,0] = %.10f\n", Q_aug[0]);
printf("[DEBUG] H[0,0] = %.10f\n", H[0]);
printf("[DEBUG] Difference = %.10e\n", fabs(Q_aug[0] - H[0]));
```

### Issue 2: Riccati Returns RET_HESSIAN_NOT_SPD
**Symptom:** S matrix not positive definite  
**Cause:** R or Q_aug has negative eigenvalues  
**Debug:**
```cpp
printf("[DEBUG] R[0,0] = %.6f (should be > 0)\n", agent.R[0]);
printf("[DEBUG] Q_aug[0,0] = %.6f (should be > 0)\n", Q_aug[0]);
```

### Issue 3: Worse Performance Than Baseline
**Symptom:** More QP iterations with MPC-aware  
**Cause:** Riccati warm start is poor quality  
**Debug:**
```cpp
// Check Riccati solution quality
real_t riccati_cost = 0.0;
for (int k = 0; k <= N; ++k) {
    // Compute cost
}
printf("[DEBUG] Riccati cost = %.6f\n", riccati_cost);
```

### Issue 4: Wrong Solution
**Symptom:** Tracking or collision violations  
**Cause:** Gradient format mismatch or initial state issue  
**Debug:**
```cpp
// Verify gradient format
printf("[DEBUG] g[0] (x state) = %.6f\n", g[0]);
printf("[DEBUG] g[nx] (u control) = %.6f\n", g[agent.nx]);
printf("[DEBUG] g[nx+nu] (next x state) = %.6f\n", g[agent.nx + agent.nu]);
```

---

## 8. ROLLBACK PLAN

If MPC-aware features cause issues:

```cpp
// Disable MPC-aware features
Options opts;
opts.setToDefault();
opts.enableMPCRiccati = BT_FALSE;  // Disable
opts.enableEqualities = BT_TRUE;
opts.printLevel = PL_NONE;
agent_solvers_[i]->setOptions(opts);

// Don't call setupMPCStructure
// Use standard qpOASES (baseline)
```

**Rollback Triggers:**
- Error 68 persists
- Wrong solution (tracking or collision violations)
- Worse performance than baseline
- Any crashes or memory corruption

---

## 9. SUCCESS CRITERIA

### Minimum Success:
- ✅ No errors (no error 68, no crashes)
- ✅ Correct solution (perfect tracking + collision avoidance)
- ✅ QP iterations reduced by ≥ 50% (296 → <150)

### Target Success:
- ✅ QP iterations reduced by ≥ 70% (296 → <90)
- ✅ Solve time reduced by ≥ 60% (290ms → <120ms)
- ✅ ADMM still converges in 2 iterations

### Stretch Goal:
- ✅ QP iterations reduced by ≥ 85% (296 → <45)
- ✅ Solve time reduced by ≥ 75% (290ms → <75ms)
- ✅ Riccati provides near-optimal solution (0-5 QP iterations)

---

**Ready for tomorrow's implementation! Good luck! 🚀**
