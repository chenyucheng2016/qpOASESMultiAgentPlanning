# Control Input Gradient Mismatch Analysis

## Problem Statement

After fixing the gradient for **fixed variables** (initial state x[0]), we still observe mismatches for **free variables**:

```
Index | g_original    | g_recovered   | Difference    | Fixed?
------+---------------+---------------+---------------+--------
   4  |      0.000000 |     -6.520107 |      6.520107 | NO  ← u[0] control
   5  |      0.000000 |     -0.017104 |      0.017104 | NO  ← u[0] control
   6  |    -23.250000 |      0.000000 |    -23.250000 | NO  ← x[1] state
   7  |   -148.800000 |   -292.800000 |    144.000000 | NO  ← x[1] state
```

## Mathematical Analysis

### 1. Variable Ordering

**qpOASES format (interleaved):**
```
z = [x[0], u[0], x[1], u[1], ..., x[N]]
  = [x0_px, x0_py, x0_vx, x0_vy, u0_ax, u0_ay, x1_px, x1_py, ...]
```

**Indices:**
- `0-3`: x[0] (FIXED)
- `4-5`: u[0] (FREE) ← Control input
- `6-7`: x[1] position (FREE)
- `8-9`: x[1] velocity (FREE)

### 2. Gradient Components

The original gradient `g_original` comes from the ADMM objective:

```
J = (1/2) Σ_k [x[k]'Qx[k] + u[k]'Ru[k]] + Σ_k [g_x[k]'x[k] + g_u[k]'u[k]]
```

Where the linear terms include:
- **Stage cost gradient:** From Q, R matrices
- **ADMM consensus terms:** `ρ(z - z_neighbor + dual)`
- **Reference tracking:** Linear terms for tracking

### 3. KKT Recovery Formula

The auxiliary gradient is recovered via:

```
g_aux = yB - H*x + A'*yC
```

Where:
- `yB` = bound duals (should be zero for inactive bounds)
- `H*x` = Hessian times primal solution
- `A'*yC` = constraint Jacobian transpose times constraint duals (costates)

### 4. Why Control Gradients Don't Match

#### 4.1 The `A'*yC` Term

The constraint matrix `A` for MPC dynamics has the structure:

```
Constraint 0: -x[1] + A*x[0] + B*u[0] = 0
Constraint 1: -x[2] + A*x[1] + B*u[1] = 0
...
```

In matrix form:
```
A = [A_dyn, 0, 0, ...]  where A_dyn for constraint k is:
    [A, B, -I, 0, 0, ...]
     ↑  ↑   ↑
    x[k] u[k] x[k+1]
```

The transpose `A'` maps constraint duals to variable gradients:

```
(A'*yC)[u[0]] = B' * λ[0]
(A'*yC)[x[1]] = -λ[0] + A' * λ[1]
```

#### 4.2 Costate Sign Convention

**Riccati costates** satisfy:
```
λ[k] = Q*x[k] + g_x[k] + A'*λ[k+1]  (for k=0..N-1)
λ[N] = Q*x[N] + g_x[N]              (terminal)
```

**qpOASES constraint duals** for dynamics constraint `A*x = b`:
```
∇L = H*x + g + A'*μ = 0
```

**Key Question:** What is the relationship between Riccati `λ[k]` and qpOASES `μ[k]`?

### 5. Constraint Formulation Difference

#### 5.1 Riccati Formulation

Riccati solves the **unconstrained** affine LQR:
```
minimize  Σ_k [(1/2)x[k]'Qx[k] + (1/2)u[k]'Ru[k] + g_x[k]'x[k] + g_u[k]'u[k]]
subject to  x[k+1] = A*x[k] + B*u[k]  (substituted via dynamics)
```

The costates are:
```
λ[k] = ∂J/∂x[k] = Q*x[k] + g_x[k] + A'*λ[k+1]
```

#### 5.2 qpOASES Formulation

qpOASES solves the **constrained** QP:
```
minimize  (1/2)z'Hz + g'z
subject to  A_constraint * z = 0
```

Where the constraint is:
```
-x[k+1] + A*x[k] + B*u[k] = 0
```

The KKT stationarity is:
```
H*z + g + A_constraint'*μ = 0
```

### 6. The Mismatch Source

#### 6.1 Different Gradient Definitions

**Riccati gradient `g_riccati`:**
- Includes the **linear cost terms** from the objective
- Computed as: `g_x[k]` and `g_u[k]` from the affine LQR

**qpOASES gradient `g_qpoases`:**
- Should represent the **same linear cost terms**
- But may have **different conventions** or **additional terms**

#### 6.2 Constraint Dual Sign

The constraint `-x[k+1] + A*x[k] + B*u[k] = 0` has dual `μ[k]`.

In KKT:
```
∂L/∂u[k] = R*u[k] + g_u[k] + B'*μ[k] = 0
```

Solving for the gradient:
```
g_u[k] = -R*u[k] - B'*μ[k]
```

But from Riccati:
```
g_u[k] = -R*u[k] - B'*λ[k]
```

**This suggests:** `μ[k] = λ[k]` (same sign!)

#### 6.3 Why the Mismatch?

The mismatch occurs because:

1. **Different constraint numbering:** 
   - Riccati: λ[0] corresponds to x[1] = A*x[0] + B*u[0]
   - qpOASES: μ[0] corresponds to constraint 0

2. **Constraint count mismatch:**
   - We have **N dynamics constraints** (k=0 to N-1)
   - But we're only mapping **(N-1) costates** (skipping terminal λ[N])
   - **This is the bug!**

### 7. The Root Cause: Missing Terminal Constraint

Looking at the code:
```cpp
// Map first (N-1) stages of costates
for (int k = 0; k < agent.N - 1; ++k) {
    y_riccati[agent.nV + k*agent.nx + j] = lambda_riccati[k*agent.nx + j];
}
// Note: lambda[N-1] (terminal costate) is NOT used
```

**But we have N dynamics constraints!** (After the fix from N-1 to N)

The constraint count is:
```cpp
nC = N * nx  // N dynamics constraints
```

So we should map **N costates** (λ[0] through λ[N-1]), not (N-1)!

### 8. The Fix

**Current code (WRONG):**
```cpp
for (int k = 0; k < agent.N - 1; ++k) {  // Only N-1 costates
    y_riccati[agent.nV + k*agent.nx + j] = lambda_riccati[k*agent.nx + j];
}
```

**Corrected code:**
```cpp
for (int k = 0; k < agent.N; ++k) {  // All N costates
    y_riccati[agent.nV + k*agent.nx + j] = lambda_riccati[k*agent.nx + j];
}
```

This will map:
- λ[0] → constraint 0 (x[1] = A*x[0] + B*u[0])
- λ[1] → constraint 1 (x[2] = A*x[1] + B*u[1])
- ...
- λ[N-1] → constraint N-1 (x[N] = A*x[N-1] + B*u[N-1])

### 9. Why This Causes Control Gradient Mismatch

With only (N-1) costates mapped:
- **Constraint 0** gets dual λ[0] ✓
- **Constraint N-1** gets dual λ[N-2] ✓
- **Constraint N** gets dual 0 ❌ (should be λ[N-1])

The missing λ[N-1] affects the gradient recovery for:
- **u[N-1]:** Missing B'*λ[N-1] term
- **x[N]:** Missing -λ[N-1] term

But wait, the mismatch is at **u[0]** and **x[1]**, not the terminal stage!

### 10. Alternative Hypothesis: Gradient Convention

Another possibility is that `g_original` and the Riccati gradient have **different conventions**:

**Possibility 1:** `g_original` includes ADMM augmentation terms that Riccati doesn't know about

**Possibility 2:** The gradient passed to qpOASES is **not** the same as the gradient used by Riccati

Let me check what `g_original` actually contains...

## Conclusion

The control gradient mismatch likely comes from:

1. **Missing terminal costate mapping** (should map N costates, not N-1)
2. **Different gradient conventions** between ADMM gradient and Riccati gradient
3. **ADMM augmentation terms** not accounted for in Riccati

**Next Steps:**
1. Fix the costate mapping to include all N costates
2. Verify what gradient is passed to qpOASES vs what Riccati uses
3. Check if ADMM consensus terms are properly included
