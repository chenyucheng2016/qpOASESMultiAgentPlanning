# qpOASES Mathematical Treatment of Fixed Bounds

## Overview

This document explains how qpOASES mathematically handles variables with **equality bounds** (fixed variables where `lb[i] = ub[i]`), and how this affects the Hessian, gradient, and auxiliary QP formulation.

## 1. Original QP Problem

The standard QP problem that qpOASES solves is:

```
minimize    (1/2) x'Hx + g'x
subject to  lb ≤ x ≤ ub
            lbA ≤ Ax ≤ ubA
```

## 2. Fixed Variables (Equality Bounds)

When a variable has **equality bounds** (`lb[i] = ub[i] = x̄[i]`), it is **fixed** at value `x̄[i]`.

**Mathematical Interpretation:**
- The variable `x[i]` is no longer a **free optimization variable**
- It becomes a **parameter** with known value `x̄[i]`
- The bound constraint becomes an **equality constraint**: `x[i] = x̄[i]`

## 3. Auxiliary QP Formulation

qpOASES uses an **auxiliary QP** technique where it solves a sequence of QPs with a known optimal solution `x*` (provided by the user or computed via Riccati).

### 3.1 Original QP with Known Solution

Given:
- Original QP: `min (1/2)x'Hx + g'x` subject to constraints
- Known optimal solution: `x*` with dual variables `y*`

### 3.2 Auxiliary QP Construction

qpOASES constructs an **auxiliary QP** where `x*` is the optimal solution by modifying the gradient:

```
Auxiliary QP:  min (1/2)x'Hx + g_aux'x
               subject to: lb_aux ≤ x ≤ ub_aux
                          lbA_aux ≤ Ax ≤ ubA_aux
```

Where the auxiliary gradient is computed from **KKT stationarity**:

```
g_aux = yB - Hx* + A'yC
```

Where:
- `yB` = dual variables for bound constraints
- `yC` = dual variables for linear constraints (costates in MPC)
- `x*` = primal solution (from Riccati)

### 3.3 Auxiliary Bounds Setup

For the auxiliary QP, bounds are set based on the **status** of each variable:

```cpp
case ST_INACTIVE:
    if (bounds.getType(i) == ST_EQUALITY) {
        lb[i] = x[i];  // Fix at current value
        ub[i] = x[i];
    }
    
case ST_LOWER:
    lb[i] = x[i];
    if (bounds.getType(i) == ST_EQUALITY) {
        ub[i] = x[i];  // Also fix upper bound
    }
    
case ST_UPPER:
    ub[i] = x[i];
    if (bounds.getType(i) == ST_EQUALITY) {
        lb[i] = x[i];  // Also fix lower bound
    }
```

**Key Insight:** For equality bounds, both `lb[i]` and `ub[i]` are set to the current value `x[i]`, ensuring the variable remains fixed.

## 4. Mathematical Treatment of Fixed Variables

### 4.1 KKT Conditions for Fixed Variables

For a **fixed variable** `x[i] = x̄[i]`, the KKT stationarity condition is:

```
∂L/∂x[i] = H[i,:]x + g[i] + λ_lower[i] - λ_upper[i] + Σ A[j,i]μ[j] = 0
```

Where:
- `λ_lower[i]` = dual for lower bound `x[i] ≥ x̄[i]`
- `λ_upper[i]` = dual for upper bound `x[i] ≤ x̄[i]`
- Both bounds are **active** (equality constraint)

**Important:** The dual variables `λ_lower[i]` and `λ_upper[i]` are **free** (can be any value) because they represent the **force needed to maintain the fixed value**.

### 4.2 Gradient Recovery for Fixed Variables

The KKT recovery formula:

```
g_aux[i] = yB[i] - (Hx*)[i] + (A'yC)[i]
```

is **INVALID** for fixed variables because:

1. **The gradient `g[i]` is arbitrary** for fixed variables (doesn't affect the solution)
2. **The bound duals `yB[i]` represent the constraint force**, not the objective gradient
3. **The formula assumes the variable is free**, which is not true for fixed variables

### 4.3 Correct Treatment

For fixed variables, the auxiliary QP should:

**Option 1: Preserve Original Gradient**
```
if (lb[i] == ub[i]):  // Fixed variable
    g_aux[i] = g_original[i]  // Keep original gradient
```

**Option 2: Set Gradient to Zero**
```
if (lb[i] == ub[i]):  // Fixed variable
    g_aux[i] = 0  // Gradient doesn't matter for fixed variables
```

**Option 3: Eliminate Fixed Variables (Most Elegant)**

Mathematically, fixed variables can be **eliminated** from the QP by substitution:

1. **Partition variables:** `x = [x_free; x_fixed]`
2. **Substitute fixed values:** `x_fixed = x̄_fixed`
3. **Reduced QP:**
   ```
   minimize    (1/2) x_free' H_ff x_free + (g_f + H_fx x̄_fixed)' x_free + constant
   subject to  lb_f ≤ x_free ≤ ub_f
               lbA ≤ A_f x_free + A_x x̄_fixed ≤ ubA
   ```

Where:
- `H_ff` = Hessian block for free variables
- `H_fx` = Hessian coupling between free and fixed variables
- `g_f` = gradient for free variables
- `A_f` = constraint matrix columns for free variables
- `A_x` = constraint matrix columns for fixed variables

## 5. Why Our Temporary Fix Works

Our temporary fix:

```cpp
for (i = 0; i < nV; ++i) {
    if (fabs(lb_original[i] - ub_original[i]) < 1e-10) {  // Fixed
        g[i] = g_original[i];  // Restore original gradient
    }
}
```

Works because:

1. ✅ **Preserves the original gradient** for fixed variables
2. ✅ **Prevents KKT recovery from corrupting** the gradient with bound duals
3. ✅ **Maintains consistency** between the original QP and auxiliary QP

## 6. Elegant Solution: Variable Elimination

The **most elegant** solution is to eliminate fixed variables before passing to qpOASES:

### 6.1 Preprocessing Step

```cpp
// Identify free and fixed variables
vector<int> free_idx, fixed_idx;
vector<real_t> x_fixed_values;

for (int i = 0; i < nV; ++i) {
    if (fabs(lb[i] - ub[i]) < 1e-10) {
        fixed_idx.push_back(i);
        x_fixed_values.push_back(lb[i]);
    } else {
        free_idx.push_back(i);
    }
}
```

### 6.2 Reduced QP Construction

```cpp
// Extract free variable blocks
H_free = H[free_idx, free_idx]
g_free = g[free_idx] + H[free_idx, fixed_idx] * x_fixed_values

// Update constraint matrix
A_free = A[:, free_idx]
b_fixed = A[:, fixed_idx] * x_fixed_values
lbA_reduced = lbA - b_fixed
ubA_reduced = ubA - b_fixed

// Solve reduced QP
qpOASES_solver.init(H_free, g_free, A_free, lb_free, ub_free, 
                    lbA_reduced, ubA_reduced)
```

### 6.3 Postprocessing Step

```cpp
// Reconstruct full solution
x_full[free_idx] = x_solution_free
x_full[fixed_idx] = x_fixed_values

// Reconstruct full duals
y_full[free_idx] = y_solution_free
y_full[fixed_idx] = 0  // Or compute from KKT if needed
```

## 7. Comparison of Approaches

| Approach | Pros | Cons | Elegance |
|----------|------|------|----------|
| **Temporary Fix** | Simple, works immediately | Not mathematically clean, still passes fixed vars to solver | ⭐⭐ |
| **Preserve Gradient** | Correct for auxiliary QP | Solver still processes fixed vars | ⭐⭐⭐ |
| **Variable Elimination** | Most efficient, mathematically clean, smaller QP | Requires preprocessing/postprocessing | ⭐⭐⭐⭐⭐ |

## 8. Recommendation

For the **most elegant solution**, implement **variable elimination**:

1. **Preprocessing:** Identify and eliminate fixed variables, update Hessian and gradient
2. **Solve:** Pass only free variables to qpOASES
3. **Postprocessing:** Reconstruct full solution with fixed values

This approach:
- ✅ Is mathematically rigorous
- ✅ Reduces QP size (faster solve)
- ✅ Avoids gradient corruption issues
- ✅ Follows standard QP preprocessing techniques

## 9. Implementation Notes

### 9.1 In MPC Context

For MPC with fixed initial state `x[0]`:
- **Fixed variables:** `x[0]` (indices 0 to nx-1)
- **Free variables:** `u[0], x[1], u[1], ..., x[N]`

After elimination:
- **Reduced nV:** `N*(nx+nu) + nx` → `N*(nx+nu)`
- **Updated gradient:** `g_free[0:nu-1] += H[0:nu-1, 0:nx-1] * x0`
- **Updated constraints:** First dynamics constraint becomes `x[1] = A*x0 + B*u[0]` → `x[1] - B*u[0] = A*x0`

### 9.2 Riccati Warm Start Compatibility

Variable elimination is **fully compatible** with Riccati warm start:
- Riccati computes solution for all variables including fixed ones
- Extract free variable solution: `x_free = x_riccati[free_idx]`
- Pass to qpOASES as warm start

## 10. References

- qpOASES Manual: Section on "Auxiliary QP" and "Equality Constraints"
- Nocedal & Wright, "Numerical Optimization": Chapter 16 on Active Set Methods
- Boyd & Vandenberghe, "Convex Optimization": Section 10.2 on Eliminating Equality Constraints
