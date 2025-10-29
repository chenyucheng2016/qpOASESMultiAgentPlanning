# Gradient Recovery Bug Analysis

## Problem Summary

`setupAuxiliaryQPgradient()` incorrectly overwrites the gradient for **fixed variables** (variables with equality bounds `lb[i] == ub[i]`), causing huge mismatches between the original gradient and the recovered gradient.

## Root Cause

The KKT recovery formula:
```
g = yB - H*x + A'*yC
```

is **only valid for FREE variables**. For **FIXED variables** (with active equality bounds), this formula produces incorrect gradients.

## Evidence from Debug Output

### Agent 0 Example:

**Bound Status:**
```
Var[0]: lb=0.000000, ub=0.000000, fixed=YES  (x[0] position x)
Var[1]: lb=4.800000, ub=4.800000, fixed=YES  (x[0] position y)
Var[2]: lb=0.000000, ub=0.000000, fixed=YES  (x[0] velocity x)
Var[3]: lb=0.000000, ub=0.000000, fixed=YES  (x[0] velocity y)
Var[4]: lb=-10.000000, ub=10.000000, fixed=NO  (u[0] control)
Var[5]: lb=-10.000000, ub=10.000000, fixed=NO  (u[0] control)
```

**Gradient Comparison:**
```
Index | g_original    | g_recovered   | Difference    | Fixed?
------+---------------+---------------+---------------+--------
   0  |      0.000000 |    -60.908517 |     60.908517 | YES ❌
   1  |     -4.800000 |    139.176237 |   -143.976237 | YES ❌
   2  |      0.000000 |    -34.572822 |     34.572822 | YES ❌
   3  |      0.000000 |     28.750111 |    -28.750111 | YES ❌
   4  |      0.000000 |     -6.520107 |      6.520107 | NO
   5  |      0.000000 |     -0.017104 |      0.017104 | NO
```

**All 4 fixed variables (x[0]) have HUGE gradient mismatches!**

### Why This Happens

1. **Initial state x[0] is fixed** by setting `lb[0:3] = ub[0:3] = x_init`
2. **x[0] appears in the first dynamics constraint:** `x[1] = A*x[0] + B*u[0]`
3. **When computing `A'*yC`**, the constraint dual λ[1] contributes to the gradient w.r.t. x[0]
4. **This contribution is WRONG** for fixed variables because:
   - Fixed variables have active equality bounds
   - The gradient w.r.t. fixed variables is arbitrary (doesn't affect the solution)
   - The KKT formula assumes free variables

## Why Free Variables Also Have Mismatches

Looking at the free variables (u[0], x[1], etc.), they also have mismatches, but smaller:

```
   4  |      0.000000 |     -6.520107 |      6.520107 | NO
   5  |      0.000000 |     -0.017104 |      0.017104 | NO
   6  |    -23.250000 |      0.000000 |    -23.250000 | NO
   7  |   -148.800000 |   -292.800000 |    144.000000 | NO
```

This suggests there might be additional issues:
- Sign convention mismatch in costates
- Indexing issues in constraint dual mapping
- Different gradient conventions between Riccati and qpOASES

But the **primary issue** is the fixed variables.

## The Solution

### Option 1: Preserve Original Gradient for Fixed Variables

In `setupAuxiliaryQPgradient()`, after computing the KKT recovery formula, restore the original gradient for fixed variables:

```cpp
returnValue QProblem::setupAuxiliaryQPgradient()
{
    // ... existing code to compute g = yB - H*x + A'*yC ...
    
    // Restore original gradient for fixed variables
    for ( int i = 0; i < nV; ++i )
    {
        if ( bounds.getStatus(i) == ST_EQUALITY )  // Fixed by equality bounds
        {
            g[i] = g_original[i];  // Keep original gradient
        }
    }
    
    return SUCCESSFUL_RETURN;
}
```

### Option 2: Skip KKT Recovery for Fixed Variables

Don't apply the KKT recovery formula to fixed variables in the first place:

```cpp
// Only apply KKT recovery to free variables
for ( int i = 0; i < nV; ++i )
{
    if ( bounds.getStatus(i) != ST_EQUALITY )  // Not fixed
    {
        // Apply: g[i] = yB[i] - H*x + A'*yC
    }
    else
    {
        // Keep: g[i] = g_original[i]
    }
}
```

### Option 3: Use Different Gradient for Auxiliary QP

For fixed variables in the auxiliary QP, the gradient doesn't matter because the variable is fixed. We could:
- Set `g[i] = 0` for fixed variables
- Or keep `g[i] = g_original[i]`

## Expected Outcome

After fixing this issue:
- Fixed variables (x[0]) will have `g_recovered == g_original`
- The auxiliary QP will correctly represent the affine LQR problem
- The remaining mismatches in free variables can be investigated separately

## Next Steps

1. **Implement the fix** in `setupAuxiliaryQPgradient()`
2. **Test** to verify fixed variables now match
3. **Investigate** remaining mismatches in free variables (if any)
4. **Verify** that the auxiliary QP gradient matches the original affine LQR gradient

## Technical Notes

- **Fixed variables** are identified by: `bounds.getStatus(i) == ST_EQUALITY` or `lb[i] == ub[i]`
- **In MPC**, the initial state x[0] is always fixed by bounds
- **The gradient w.r.t. fixed variables** is arbitrary and doesn't affect the solution
- **The KKT formula** `g = yB - H*x + A'*yC` assumes all variables are free

## References

- Debug output: Agent 0 and Agent 1 gradient comparisons
- Test file: `test_2agent_rho25.cpp`
- Function: `QProblem::setupAuxiliaryQPgradient()` (lines 2980-3030)
- Function: `QProblem::solveInitialQP()` (lines 1560-1600)
