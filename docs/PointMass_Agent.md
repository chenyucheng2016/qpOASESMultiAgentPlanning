# PointMass Agent Class

## Overview

The `PointMass` class is a derived class from `AgentData` that implements 2D point mass dynamics for multi-agent MPC planning.

## Design Philosophy

- **Base Class (`AgentData`)**: Provides common interface for all agent types (currently implements 1D double integrator)
- **Derived Class (`PointMass`)**: Extends base class with 2D point mass dynamics
- **Extensibility**: Easy to add new agent types (quadrotors, cars, etc.) by deriving from `AgentData`

## State and Control

### State Vector (nx = 4)
```
x = [x, y, vx, vy]
```
- `x, y`: Position in 2D plane (m)
- `vx, vy`: Velocity in 2D plane (m/s)

### Control Vector (nu = 2)
```
u = [ax, ay]
```
- `ax, ay`: Acceleration in 2D plane (m/s²)

## Dynamics

### Continuous-Time Dynamics
```
dx/dt = vx
dy/dt = vy
dvx/dt = ax
dvy/dt = ay
```

### Discrete-Time Dynamics (Euler Integration)
```
x[k+1] = x[k] + dt * vx[k]
y[k+1] = y[k] + dt * vy[k]
vx[k+1] = vx[k] + dt * ax[k]
vy[k+1] = vy[k] + dt * ay[k]
```

### State Transition Matrices
```
A = [1  0  dt  0]    B = [0   0 ]
    [0  1  0  dt]        [0   0 ]
    [0  0  1   0]        [dt  0 ]
    [0  0  0   1]        [0  dt ]
```

## Constructor

```cpp
PointMass(int N = 10, real_t dt = 0.2, real_t a_max = 10.0, real_t v_max = 20.0)
```

### Parameters
- `N`: Prediction horizon (number of time steps)
- `dt`: Time step for discretization (seconds)
- `a_max`: Maximum acceleration magnitude (m/s²) - box constraint
- `v_max`: Maximum velocity magnitude (m/s) - box constraint (negative means no limit)

### Default Values
- Horizon: N = 10 (2 seconds with dt=0.2)
- Time step: dt = 0.2 s
- Max acceleration: a_max = 10.0 m/s²
- Max velocity: v_max = 20.0 m/s

## Methods

### setupDynamics()
Sets up the A and B matrices for 2D point mass dynamics.

### setupBounds()
Configures bounds on states and controls:
- **Control bounds**: `-a_max <= ax, ay <= a_max` (box constraints)
- **Velocity bounds**: `-v_max <= vx, vy <= v_max` (if v_max > 0)
- **Position bounds**: Unbounded by default (can be set via `addWorkspaceBounds()`)

### setupCostMatrices(Q_pos, Q_vel, R_ctrl)
Sets up diagonal cost matrices:
```cpp
Q = diag([Q_pos, Q_pos, Q_vel, Q_vel])  // State cost
R = diag([R_ctrl, R_ctrl])              // Control cost
```

**Default values:**
- `Q_pos = 10.0`: Position tracking cost
- `Q_vel = 1.0`: Velocity tracking cost
- `R_ctrl = 0.01`: Control effort cost

### addWorkspaceBounds(x_min, x_max, y_min, y_max)
Adds rectangular workspace bounds to constrain agent motion:
```cpp
x_min <= x[k] <= x_max
y_min <= y[k] <= y_max
```

### addCircularObstacle(x_obs, y_obs, r_obs, r_agent)
**TODO**: Adds circular obstacle avoidance constraint.
- Currently a placeholder for future implementation
- Requires iterative linearization around current trajectory

## Usage Example

```cpp
// Create a 2D point mass agent
PointMass agent(10, 0.2, 10.0, 20.0);  // N=10, dt=0.2, a_max=10, v_max=20

// Customize cost matrices
agent.setupCostMatrices(10.0, 1.0, 0.01);

// Add workspace bounds
agent.addWorkspaceBounds(0.0, 100.0, 0.0, 100.0);

// Set initial condition
real_t x_init[4] = {0.0, 0.0, 0.0, 0.0};  // Start at origin at rest

// Set reference trajectory (minimum-jerk to target)
// ... (generate reference using 5th-order polynomial)

// Use with TurboADMM for multi-agent planning
TurboADMM admm;
admm.setup(&agent, 1, ...);
```

## Static Inequality Constraints

The `AgentData` base class provides an interface for static inequality constraints:
```cpp
struct AgentData {
    int nG;        // Number of static constraints
    real_t* G;     // Constraint matrix (nG × nV)
    real_t* c;     // Lower bounds (nG × 1)
    real_t* d;     // Upper bounds (nG × 1)
};
```

These can be used for:
- Obstacle avoidance (static objects)
- Workspace boundaries
- No-fly zones
- Custom linear inequality constraints

**Note:** Inter-agent collision avoidance is handled separately by the ADMM framework.

## Integration with ADMM

The `PointMass` class is fully compatible with the `TurboADMM` multi-agent solver:
- Riccati warm start works with any linear dynamics (A, B matrices)
- MPC-aware qpOASES exploits block-diagonal structure
- ADMM handles inter-agent collision avoidance
- Static obstacles handled via inequality constraints

## Future Extensions

Potential derived classes:
- **Quadrotor**: 12-state model with attitude dynamics
- **Unicycle**: Nonholonomic car-like robot
- **Dubins**: Constant-speed vehicle with turning radius
- **DoubleIntegrator**: Explicit 1D double integrator (for backward compatibility)

## Files Modified

- `include/qpOASES/TurboADMM.hpp`: Added `PointMass` class declaration
- `src/TurboADMM.cpp`: Added `PointMass` implementation

## Testing

A test suite for `PointMass` agents will be created in:
- `tests/test_pointmass_2d.cpp`: Single agent 2D navigation
- `tests/test_pointmass_multiagent.cpp`: Multi-agent 2D collision avoidance
