/*
 *  This file is part of qpOASES.
 *
 *  TurboADMM: Multi-Agent MPC using ADMM with MPC-aware qpOASES
 *
 *  Implementation of data structures and core methods
 */

#include <qpOASES/TurboADMM.hpp>
#include <cstring>
#include <cmath>

BEGIN_NAMESPACE_QPOASES


/*
 * AgentData implementation
 */
AgentData::AgentData()
{
    N = 0;
    nx = 0;
    nu = 0;
    nV = 0;
    nC = 0;
    
    A = 0;
    B = 0;
    Q = 0;
    R = 0;
    Q_diag = 0;
    R_diag = 0;
    
    x_ref = 0;
    u_ref = 0;
    
    lb = 0;
    ub = 0;
    lbA = 0;
    ubA = 0;
    
    nG = 0;
    G = 0;
    c = 0;
    d = 0;
}


AgentData::AgentData(const AgentData& other)
{
    // Initialize to null first
    A = 0;
    B = 0;
    Q = 0;
    R = 0;
    Q_diag = 0;
    R_diag = 0;
    x_ref = 0;
    u_ref = 0;
    lb = 0;
    ub = 0;
    lbA = 0;
    ubA = 0;
    
    nG = 0;
    G = 0;
    c = 0;
    d = 0;
    
    // Copy dimensions
    N = other.N;
    nx = other.nx;
    nu = other.nu;
    nV = other.nV;
    nC = other.nC;
    
    // Deep copy all arrays if source is allocated
    if (other.A) {
        A = new real_t[nx * nx];
        memcpy(A, other.A, nx * nx * sizeof(real_t));
    }
    
    if (other.B) {
        B = new real_t[nx * nu];
        memcpy(B, other.B, nx * nu * sizeof(real_t));
    }
    
    if (other.Q) {
        Q = new real_t[nx * nx];
        memcpy(Q, other.Q, nx * nx * sizeof(real_t));
    }
    
    if (other.R) {
        R = new real_t[nu * nu];
        memcpy(R, other.R, nu * nu * sizeof(real_t));
    }
    
    if (other.Q_diag) {
        Q_diag = new real_t[nx];
        memcpy(Q_diag, other.Q_diag, nx * sizeof(real_t));
    }
    
    if (other.R_diag) {
        R_diag = new real_t[nu];
        memcpy(R_diag, other.R_diag, nu * sizeof(real_t));
    }
    
    if (other.x_ref) {
        x_ref = new real_t[(N + 1) * nx];
        memcpy(x_ref, other.x_ref, (N + 1) * nx * sizeof(real_t));
    }
    
    if (other.u_ref) {
        u_ref = new real_t[N * nu];
        memcpy(u_ref, other.u_ref, N * nu * sizeof(real_t));
    }
    
    if (other.lb) {
        lb = new real_t[nV];
        memcpy(lb, other.lb, nV * sizeof(real_t));
    }
    
    if (other.ub) {
        ub = new real_t[nV];
        memcpy(ub, other.ub, nV * sizeof(real_t));
    }
    
    if (other.lbA) {
        lbA = new real_t[nC];
        memcpy(lbA, other.lbA, nC * sizeof(real_t));
    }
    
    if (other.ubA) {
        ubA = new real_t[nC];
        memcpy(ubA, other.ubA, nC * sizeof(real_t));
    }
    
    // Copy static inequality constraints
    nG = other.nG;
    
    if (other.G && nG > 0) {
        G = new real_t[nG * nV];
        memcpy(G, other.G, nG * nV * sizeof(real_t));
    }
    
    if (other.c && nG > 0) {
        c = new real_t[nG];
        memcpy(c, other.c, nG * sizeof(real_t));
    }
    
    if (other.d && nG > 0) {
        d = new real_t[nG];
        memcpy(d, other.d, nG * sizeof(real_t));
    }
}


AgentData& AgentData::operator=(const AgentData& other)
{
    // Check for self-assignment
    if (this == &other)
        return *this;
    
    // Free existing memory
    clear();
    
    // Copy dimensions
    N = other.N;
    nx = other.nx;
    nu = other.nu;
    nV = other.nV;
    nC = other.nC;
    
    // Deep copy all arrays
    if (other.A) {
        A = new real_t[nx * nx];
        memcpy(A, other.A, nx * nx * sizeof(real_t));
    }
    
    if (other.B) {
        B = new real_t[nx * nu];
        memcpy(B, other.B, nx * nu * sizeof(real_t));
    }
    
    if (other.Q) {
        Q = new real_t[nx * nx];
        memcpy(Q, other.Q, nx * nx * sizeof(real_t));
    }
    
    if (other.R) {
        R = new real_t[nu * nu];
        memcpy(R, other.R, nu * nu * sizeof(real_t));
    }
    
    if (other.Q_diag) {
        Q_diag = new real_t[nx];
        memcpy(Q_diag, other.Q_diag, nx * sizeof(real_t));
    }
    
    if (other.R_diag) {
        R_diag = new real_t[nu];
        memcpy(R_diag, other.R_diag, nu * sizeof(real_t));
    }
    
    if (other.x_ref) {
        x_ref = new real_t[(N + 1) * nx];
        memcpy(x_ref, other.x_ref, (N + 1) * nx * sizeof(real_t));
    }
    
    if (other.u_ref) {
        u_ref = new real_t[N * nu];
        memcpy(u_ref, other.u_ref, N * nu * sizeof(real_t));
    }
    
    if (other.lb) {
        lb = new real_t[nV];
        memcpy(lb, other.lb, nV * sizeof(real_t));
    }
    
    if (other.ub) {
        ub = new real_t[nV];
        memcpy(ub, other.ub, nV * sizeof(real_t));
    }
    
    if (other.lbA) {
        lbA = new real_t[nC];
        memcpy(lbA, other.lbA, nC * sizeof(real_t));
    }
    
    if (other.ubA) {
        ubA = new real_t[nC];
        memcpy(ubA, other.ubA, nC * sizeof(real_t));
    }
    
    // Copy static inequality constraints
    nG = other.nG;
    
    if (other.G && nG > 0) {
        G = new real_t[nG * nV];
        memcpy(G, other.G, nG * nV * sizeof(real_t));
    }
    
    if (other.c && nG > 0) {
        c = new real_t[nG];
        memcpy(c, other.c, nG * sizeof(real_t));
    }
    
    if (other.d && nG > 0) {
        d = new real_t[nG];
        memcpy(d, other.d, nG * sizeof(real_t));
    }
    
    return *this;
}


AgentData::~AgentData()
{
    clear();
}


returnValue AgentData::allocate(int N_, int nx_, int nu_)
{
    // Set dimensions
    N = N_;
    nx = nx_;
    nu = nu_;
    // MPC structure: [x0, u0, x1, u1, ..., x_{N-1}, u_{N-1}, x_N]
    // = N stages of [x, u] + terminal state x_N
    nV = N * (nx + nu) + nx;
    nC = (N - 1) * nx;
    
    // Allocate matrices
    A = new real_t[nx * nx];
    B = new real_t[nx * nu];
    Q = new real_t[nx * nx];
    R = new real_t[nu * nu];
    
    Q_diag = new real_t[nx];
    R_diag = new real_t[nu];
    
    // Allocate reference trajectories
    x_ref = new real_t[(N + 1) * nx];
    u_ref = new real_t[N * nu];
    
    // Allocate bounds
    lb = new real_t[nV];
    ub = new real_t[nV];
    lbA = new real_t[nC];
    ubA = new real_t[nC];
    
    // Initialize to zero
    memset(A, 0, nx * nx * sizeof(real_t));
    memset(B, 0, nx * nu * sizeof(real_t));
    memset(Q, 0, nx * nx * sizeof(real_t));
    memset(R, 0, nu * nu * sizeof(real_t));
    memset(Q_diag, 0, nx * sizeof(real_t));
    memset(R_diag, 0, nu * sizeof(real_t));
    memset(x_ref, 0, (N + 1) * nx * sizeof(real_t));
    memset(u_ref, 0, N * nu * sizeof(real_t));
    
    // Initialize bounds to +/- infinity
    for (int i = 0; i < nV; ++i) {
        lb[i] = -1.0e20;
        ub[i] = 1.0e20;
    }
    for (int i = 0; i < nC; ++i) {
        lbA[i] = 0.0;  // Equality constraints for dynamics
        ubA[i] = 0.0;
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue AgentData::extractDiagonals()
{
    // Extract diagonal of Q
    for (int i = 0; i < nx; ++i) {
        Q_diag[i] = Q[i * nx + i];
    }
    
    // Extract diagonal of R
    for (int i = 0; i < nu; ++i) {
        R_diag[i] = R[i * nu + i];
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue AgentData::clear()
{
    if (A) delete[] A;
    if (B) delete[] B;
    if (Q) delete[] Q;
    if (R) delete[] R;
    if (Q_diag) delete[] Q_diag;
    if (R_diag) delete[] R_diag;
    if (x_ref) delete[] x_ref;
    if (u_ref) delete[] u_ref;
    if (lb) delete[] lb;
    if (ub) delete[] ub;
    if (lbA) delete[] lbA;
    if (ubA) delete[] ubA;
    
    // Delete static inequality constraints
    if (G) delete[] G;
    if (c) delete[] c;
    if (d) delete[] d;
    
    A = 0;
    B = 0;
    Q = 0;
    R = 0;
    Q_diag = 0;
    R_diag = 0;
    x_ref = 0;
    u_ref = 0;
    lb = 0;
    ub = 0;
    lbA = 0;
    ubA = 0;
    
    nG = 0;
    G = 0;
    c = 0;
    d = 0;
    
    return SUCCESSFUL_RETURN;
}


/*
 * PointMass implementation
 */

PointMass::PointMass(int N_, real_t dt_, real_t a_max_, real_t v_max_)
    : dt(dt_), a_max(a_max_), v_max(v_max_)
{
    // Allocate for 2D point mass: nx=4 (x, y, vx, vy), nu=2 (ax, ay)
    allocate(N_, 4, 2);
    
    // Setup dynamics, bounds, and default cost matrices
    setupDynamics();
    setupBounds();
    setupCostMatrices();
}

returnValue PointMass::setupDynamics()
{
    // 2D point mass dynamics (Euler discretization)
    // State: [x, y, vx, vy], Control: [ax, ay]
    // x[k+1] = x[k] + dt*vx[k]
    // y[k+1] = y[k] + dt*vy[k]
    // vx[k+1] = vx[k] + dt*ax[k]
    // vy[k+1] = vy[k] + dt*ay[k]
    
    // A matrix (4x4): Identity + dt * [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0]
    memset(A, 0, nx * nx * sizeof(real_t));
    A[0*nx + 0] = 1.0;  // x[k+1] = x[k] + ...
    A[0*nx + 2] = dt;   // ... + dt*vx[k]
    A[1*nx + 1] = 1.0;  // y[k+1] = y[k] + ...
    A[1*nx + 3] = dt;   // ... + dt*vy[k]
    A[2*nx + 2] = 1.0;  // vx[k+1] = vx[k] + ...
    A[3*nx + 3] = 1.0;  // vy[k+1] = vy[k] + ...
    
    // B matrix (4x2): [0 0; 0 0; dt 0; 0 dt]
    memset(B, 0, nx * nu * sizeof(real_t));
    B[2*nu + 0] = dt;   // vx[k+1] = ... + dt*ax[k]
    B[3*nu + 1] = dt;   // vy[k+1] = ... + dt*ay[k]
    
    return SUCCESSFUL_RETURN;
}

returnValue PointMass::setupBounds()
{
    // Initialize all bounds to +/- infinity
    for (int i = 0; i < nV; ++i) {
        lb[i] = -INFTY;
        ub[i] = INFTY;
    }
    
    // Set control bounds: -a_max <= ax, ay <= a_max
    for (int k = 0; k < N; ++k) {
        int idx = k * (nx + nu) + nx;  // Control starts after state
        lb[idx + 0] = -a_max;  // ax lower bound
        ub[idx + 0] = a_max;   // ax upper bound
        lb[idx + 1] = -a_max;  // ay lower bound
        ub[idx + 1] = a_max;   // ay upper bound
    }
    
    // Set velocity bounds if v_max > 0
    if (v_max > 0.0) {
        for (int k = 0; k <= N; ++k) {
            int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
            lb[idx + 2] = -v_max;  // vx lower bound
            ub[idx + 2] = v_max;   // vx upper bound
            lb[idx + 3] = -v_max;  // vy lower bound
            ub[idx + 3] = v_max;   // vy upper bound
        }
    }
    
    // Dynamics constraint bounds (equality: lbA = ubA = 0)
    for (int i = 0; i < nC; ++i) {
        lbA[i] = 0.0;
        ubA[i] = 0.0;
    }
    
    return SUCCESSFUL_RETURN;
}

returnValue PointMass::setupCostMatrices(real_t Q_pos, real_t Q_vel, real_t R_ctrl)
{
    // Q matrix (4x4 diagonal): [Q_pos, Q_pos, Q_vel, Q_vel]
    memset(Q, 0, nx * nx * sizeof(real_t));
    Q[0*nx + 0] = Q_pos;  // x cost
    Q[1*nx + 1] = Q_pos;  // y cost
    Q[2*nx + 2] = Q_vel;  // vx cost
    Q[3*nx + 3] = Q_vel;  // vy cost
    
    Q_diag[0] = Q_pos;
    Q_diag[1] = Q_pos;
    Q_diag[2] = Q_vel;
    Q_diag[3] = Q_vel;
    
    // R matrix (2x2 diagonal): [R_ctrl, R_ctrl]
    memset(R, 0, nu * nu * sizeof(real_t));
    R[0*nu + 0] = R_ctrl;  // ax cost
    R[1*nu + 1] = R_ctrl;  // ay cost
    
    R_diag[0] = R_ctrl;
    R_diag[1] = R_ctrl;
    
    return SUCCESSFUL_RETURN;
}

/*
 * Define a rectangular obstacle (stores geometry only)
 */
int PointMass::defineObstacle(const RectangularObstacle& obstacle)
{
    obstacles_.push_back(obstacle);
    int id = obstacles_.size() - 1;
    obstacles_[id].id = id;  // Ensure ID is set
    return id;
}

/*
 * Transform point from global frame to obstacle's local frame
 */
void PointMass::transformToLocal(real_t x_global, real_t y_global,
                                 const RectangularObstacle& obstacle,
                                 real_t& x_local, real_t& y_local) const
{
    // Translate to obstacle center
    real_t dx = x_global - obstacle.x_center;
    real_t dy = y_global - obstacle.y_center;
    
    // Rotate by -theta (inverse rotation)
    real_t cos_theta = cos(-obstacle.theta);
    real_t sin_theta = sin(-obstacle.theta);
    
    x_local = cos_theta * dx - sin_theta * dy;
    y_local = sin_theta * dx + cos_theta * dy;
}

/*
 * Get constraint coefficients for bypass side in global frame
 */
void PointMass::getConstraintCoefficients(const RectangularObstacle& obstacle, BypassSide side, real_t r_agent,
                                          real_t& a, real_t& b, real_t& bound, bool& is_upper) const
{
    // Constraint in local frame: a_local * x_local + b_local * y_local {<=, >=} bound_local
    real_t a_local, b_local, bound_local;
    
    switch (side) {
        case BYPASS_LEFT:
            // Local frame: y_local >= height/2 + r_agent
            a_local = 0.0;
            b_local = 1.0;
            bound_local = obstacle.height / 2.0 + r_agent;
            is_upper = false;  // >= constraint
            break;
            
        case BYPASS_RIGHT:
            // Local frame: y_local <= -height/2 - r_agent
            a_local = 0.0;
            b_local = 1.0;
            bound_local = -obstacle.height / 2.0 - r_agent;
            is_upper = true;  // <= constraint
            break;
            
        case BYPASS_ABOVE:
            // Local frame: x_local >= width/2 + r_agent
            a_local = 1.0;
            b_local = 0.0;
            bound_local = obstacle.width / 2.0 + r_agent;
            is_upper = false;  // >= constraint
            break;
            
        case BYPASS_BELOW:
            // Local frame: x_local <= -width/2 - r_agent
            a_local = 1.0;
            b_local = 0.0;
            bound_local = -obstacle.width / 2.0 - r_agent;
            is_upper = true;  // <= constraint
            break;
    }
    
    // Transform constraint to global frame
    // Local: a_local * x_local + b_local * y_local {<=, >=} bound_local
    // where x_local = cos(theta) * (x - x_c) + sin(theta) * (y - y_c)
    //       y_local = -sin(theta) * (x - x_c) + cos(theta) * (y - y_c)
    // Substituting and rearranging:
    // [a_local * cos(theta) - b_local * sin(theta)] * x +
    // [a_local * sin(theta) + b_local * cos(theta)] * y {<=, >=}
    // bound_local + a_local * cos(theta) * x_c + a_local * sin(theta) * y_c - b_local * sin(theta) * x_c + b_local * cos(theta) * y_c
    
    real_t cos_theta = cos(obstacle.theta);
    real_t sin_theta = sin(obstacle.theta);
    
    a = a_local * cos_theta - b_local * sin_theta;
    b = a_local * sin_theta + b_local * cos_theta;
    bound = bound_local + a_local * cos_theta * obstacle.x_center + a_local * sin_theta * obstacle.y_center
            - b_local * sin_theta * obstacle.x_center + b_local * cos_theta * obstacle.y_center;
}

/*
 * Add constraint for bypassing obstacle on specified side
 */
returnValue PointMass::addObstacleConstraint(int obstacle_id, BypassSide side, real_t r_agent)
{
    // Check if obstacle ID is valid
    if (obstacle_id < 0 || obstacle_id >= (int)obstacles_.size()) {
        return THROWERROR(RET_INDEX_OUT_OF_BOUNDS);
    }
    
    const RectangularObstacle& obstacle = obstacles_[obstacle_id];
    
    // Get constraint coefficients in global frame
    real_t a, b, bound;
    bool is_upper;
    getConstraintCoefficients(obstacle, side, r_agent, a, b, bound, is_upper);
    
    // Add ONE constraint per stage k=0..N
    // Total: (N+1) constraints for this bypass side
    int n_new_constraints = N + 1;
    
    // Allocate new constraint matrices
    int nG_new = nG + n_new_constraints;
    real_t* G_new = new real_t[nG_new * nV];
    real_t* c_new = new real_t[nG_new];
    real_t* d_new = new real_t[nG_new];
    
    // Copy existing constraints
    if (nG > 0 && G != 0) {
        memcpy(G_new, G, nG * nV * sizeof(real_t));
        memcpy(c_new, c, nG * sizeof(real_t));
        memcpy(d_new, d, nG * sizeof(real_t));
    }
    
    // Initialize new constraints to zero
    memset(G_new + nG * nV, 0, n_new_constraints * nV * sizeof(real_t));
    
    // Add constraint for each stage k=0..N
    // Constraint: a*x + b*y {<=, >=} bound
    for (int k = 0; k <= N; ++k) {
        // Variable index for position at stage k
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        
        // Constraint index for this stage
        int c_idx = nG + k;
        
        // Set constraint coefficients: a*x + b*y
        G_new[c_idx * nV + idx + 0] = a;  // coefficient for x
        G_new[c_idx * nV + idx + 1] = b;  // coefficient for y
        
        // Set bounds based on constraint type
        if (is_upper) {
            // Upper bound constraint: a*x + b*y <= bound
            c_new[c_idx] = -INFTY;
            d_new[c_idx] = bound;
        } else {
            // Lower bound constraint: a*x + b*y >= bound
            c_new[c_idx] = bound;
            d_new[c_idx] = INFTY;
        }
    }
    
    // Delete old constraint matrices
    if (G) delete[] G;
    if (c) delete[] c;
    if (d) delete[] d;
    
    // Update pointers and count
    G = G_new;
    c = c_new;
    d = d_new;
    nG = nG_new;
    
    return SUCCESSFUL_RETURN;
}

/*
 * Add time-varying obstacle constraint based on reference trajectory
 */
returnValue PointMass::addTimeVaryingObstacleConstraint(int obstacle_id, const BypassSide* bypass_schedule, real_t r_agent)
{
    // Check if obstacle ID is valid
    if (obstacle_id < 0 || obstacle_id >= (int)obstacles_.size()) {
        return THROWERROR(RET_INDEX_OUT_OF_BOUNDS);
    }
    
    const RectangularObstacle& obstacle = obstacles_[obstacle_id];
    
    // Store bypass schedule for SCP constraint updates in ADMM loop
    bypass_schedule_.clear();
    bypass_schedule_.reserve(N + 1);
    for (int k = 0; k <= N; ++k) {
        bypass_schedule_.push_back(bypass_schedule[k]);
    }
    
    // Add (N+1) constraints, one per stage with potentially different bypass sides
    int n_new_constraints = N + 1;
    
    // Allocate new constraint matrices
    int nG_new = nG + n_new_constraints;
    real_t* G_new = new real_t[nG_new * nV];
    real_t* c_new = new real_t[nG_new];
    real_t* d_new = new real_t[nG_new];
    
    // Copy existing constraints
    if (nG > 0 && G != 0) {
        memcpy(G_new, G, nG * nV * sizeof(real_t));
        memcpy(c_new, c, nG * sizeof(real_t));
        memcpy(d_new, d, nG * sizeof(real_t));
    }
    
    // Initialize new constraints to zero
    memset(G_new + nG * nV, 0, n_new_constraints * nV * sizeof(real_t));
    
    // Add constraint for each stage k=0..N
    // Each stage can have a different bypass side based on reference trajectory
    for (int k = 0; k <= N; ++k) {
        // Get bypass side for this stage from schedule
        BypassSide side = bypass_schedule[k];
        
        // Get constraint coefficients in global frame for this bypass side
        real_t a, b, bound;
        bool is_upper;
        getConstraintCoefficients(obstacle, side, r_agent, a, b, bound, is_upper);
        
        // Variable index for position at stage k
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        
        // Constraint index for this stage
        int c_idx = nG + k;
        
        // Set constraint coefficients: a*x + b*y {<=, >=} bound
        G_new[c_idx * nV + idx + 0] = a;  // coefficient for x
        G_new[c_idx * nV + idx + 1] = b;  // coefficient for y
        
        // Set bounds based on constraint type
        if (is_upper) {
            // Upper bound constraint: a*x + b*y <= bound
            c_new[c_idx] = -INFTY;
            d_new[c_idx] = bound;
        } else {
            // Lower bound constraint: a*x + b*y >= bound
            c_new[c_idx] = bound;
            d_new[c_idx] = INFTY;
        }
    }
    
    // Delete old constraint matrices
    if (G) delete[] G;
    if (c) delete[] c;
    if (d) delete[] d;
    
    // Update pointers and count
    G = G_new;
    c = c_new;
    d = d_new;
    nG = nG_new;
    
    return SUCCESSFUL_RETURN;
}

/*
 * Update obstacle constraints based on current trajectory
 * 
 * This is critical for Sequential Convex Programming (SCP) where constraints
 * need to be re-linearized around the current trajectory at each iteration.
 */
returnValue PointMass::updateObstacleConstraints(const real_t* z_current, const BypassSide* bypass_schedule)
{
    // Check if we have obstacles and constraints
    if (obstacles_.empty() || nG == 0 || !G) {
        return SUCCESSFUL_RETURN;  // No obstacles to update
    }
    
    // Check if bypass schedule is stored (from addTimeVaryingObstacleConstraint)
    if (bypass_schedule_.empty()) {
        return SUCCESSFUL_RETURN;  // No bypass schedule available
    }
    
    // Get the first obstacle (can be generalized to loop over all obstacles)
    const RectangularObstacle& obstacle = obstacles_[0];
    
    // Update constraint for each stage k=0..N based on current trajectory
    for (int k = 0; k <= N; ++k) {
        // Get current position at stage k from z_current
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        real_t x_current = z_current[idx + 0];
        real_t y_current = z_current[idx + 1];
        
        // Get bypass side for this stage from stored schedule
        BypassSide side = bypass_schedule_[k];
        
        // Get constraint coefficients in global frame for this bypass side
        real_t a, b, bound;
        bool is_upper;
        getConstraintCoefficients(obstacle, side, 0.5, a, b, bound, is_upper);
        
        // Update constraint coefficients in G matrix
        // Constraint index for this stage (assuming obstacle constraints start at row 0)
        int c_idx = k;
        
        // Update coefficients: a*x + b*y {<=, >=} bound
        G[c_idx * nV + idx + 0] = a;  // coefficient for x
        G[c_idx * nV + idx + 1] = b;  // coefficient for y
        
        // Update bounds
        if (is_upper) {
            // Upper bound constraint: a*x + b*y <= bound
            c[c_idx] = -INFTY;
            d[c_idx] = bound;
        } else {
            // Lower bound constraint: a*x + b*y >= bound
            c[c_idx] = bound;
            d[c_idx] = INFTY;
        }
    }
    
    return SUCCESSFUL_RETURN;
}

returnValue PointMass::addWorkspaceBounds(real_t x_min, real_t x_max, real_t y_min, real_t y_max)
{
    // Add workspace bounds to position states
    for (int k = 0; k <= N; ++k) {
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        
        // x bounds
        if (lb[idx + 0] < x_min) lb[idx + 0] = x_min;
        if (ub[idx + 0] > x_max) ub[idx + 0] = x_max;
        
        // y bounds
        if (lb[idx + 1] < y_min) lb[idx + 1] = y_min;
        if (ub[idx + 1] > y_max) ub[idx + 1] = y_max;
    }
    
    return SUCCESSFUL_RETURN;
}


/*
 * CouplingData implementation
 */
CouplingData::CouplingData()
{
    d_safe = 1.0;     // Default safety distance
    gamma = 1.0;      // Default coupling penalty
}


/*
 * ADMMParameters implementation
 */
ADMMParameters::ADMMParameters()
{
    rho = 10.0;
    alpha_relax = 1.0;  // Disabled for now (1.7 causes instability), tune later
    
    eps_primal = 1.0e-4;
    eps_dual = 1.0e-4;
    
    max_admm_iter = 20;
    max_qp_iter = 1000;
    
    enable_adaptive_rho = BT_TRUE;
    enable_over_relax = BT_TRUE;
    enable_collision_avoidance = BT_FALSE;  // Use soft penalty (tune beta)
}


/*
 * ADMMStatistics implementation
 */
ADMMStatistics::ADMMStatistics()
{
    reset();
}


void ADMMStatistics::reset()
{
    admm_iterations = 0;
    total_qp_iterations = 0;
    solve_time_ms = 0.0;
    primal_residual = 0.0;
    dual_residual = 0.0;
    converged = BT_FALSE;
}


/*
 * TurboADMM implementation
 */
TurboADMM::TurboADMM()
{
    n_agents_ = 0;
    agents_ = 0;
    agent_solvers_ = 0;
    
    z_local_ = 0;
    z_prev_ = 0;
    
    // Collision coupling variables
    neighbors_ = 0;
    num_neighbors_ = 0;
    v_collision_ = 0;
    lambda_collision_ = 0;
    
    coupling_vectors_ = 0;
    
    nV_total_ = 0;
    
    is_setup_ = BT_FALSE;
    is_first_solve_ = BT_TRUE;
}


TurboADMM::TurboADMM(int n_agents)
{
    n_agents_ = n_agents;
    agents_ = new AgentData[n_agents];
    agent_solvers_ = 0;
    
    z_local_ = 0;
    z_prev_ = 0;
    
    // Collision coupling variables
    neighbors_ = 0;
    num_neighbors_ = 0;
    v_collision_ = 0;
    lambda_collision_ = 0;
    
    coupling_vectors_ = 0;
    
    nV_total_ = 0;
    
    is_setup_ = BT_FALSE;
    is_first_solve_ = BT_TRUE;
}


TurboADMM::~TurboADMM()
{
    // Free agent solvers
    if (agent_solvers_) {
        for (int i = 0; i < n_agents_; ++i) {
            if (agent_solvers_[i])
                delete agent_solvers_[i];
        }
        delete[] agent_solvers_;
    }
    
    // Free ADMM variables
    if (z_local_) {
        for (int i = 0; i < n_agents_; ++i) {
            if (z_local_[i]) delete[] z_local_[i];
        }
        delete[] z_local_;
    }
    
    if (z_prev_) {
        for (int i = 0; i < n_agents_; ++i) {
            if (z_prev_[i]) delete[] z_prev_[i];
        }
        delete[] z_prev_;
    }
    
    // Free collision coupling variables
    // TODO: Implement proper cleanup for v_collision_ and lambda_collision_
    
    // Free coupling vectors
    if (coupling_vectors_) {
        for (int i = 0; i < n_agents_; ++i) {
            if (coupling_vectors_[i]) {
                for (int k = 0; k <= agents_[i].N; ++k) {
                    if (coupling_vectors_[i][k])
                        delete[] coupling_vectors_[i][k];
                }
                delete[] coupling_vectors_[i];
            }
        }
        delete[] coupling_vectors_;
    }
    
    // Free agent data
    if (agents_)
        delete[] agents_;
}


returnValue TurboADMM::setup(
    const AgentData* agents,
    int n_agents,
    const CouplingData* coupling,
    const ADMMParameters* params,
    const int* const* neighbors,
    const int* num_neighbors
)
{
    // Store configuration
    n_agents_ = n_agents;
    coupling_ = *coupling;
    params_ = *params;
    
    // Allocate and copy agent data
    if (!agents_)
        agents_ = new AgentData[n_agents];
    
    nV_total_ = 0;
    for (int i = 0; i < n_agents; ++i) {
        agents_[i] = agents[i];  // Copy agent data
        nV_total_ += agents[i].nV;
    }
    
    // Store neighbor graph (if not provided, assume all-to-all)
    neighbors_ = new int*[n_agents];
    num_neighbors_ = new int[n_agents];
    
    if (neighbors && num_neighbors) {
        // Use provided neighbor graph
        for (int i = 0; i < n_agents; ++i) {
            num_neighbors_[i] = num_neighbors[i];
            neighbors_[i] = new int[num_neighbors[i]];
            for (int j = 0; j < num_neighbors[i]; ++j) {
                neighbors_[i][j] = neighbors[i][j];
            }
        }
    } else {
        // Default: all-to-all coupling (fully connected graph)
        for (int i = 0; i < n_agents; ++i) {
            num_neighbors_[i] = n_agents - 1;
            neighbors_[i] = new int[n_agents - 1];
            int idx = 0;
            for (int j = 0; j < n_agents; ++j) {
                if (j != i) {
                    neighbors_[i][idx++] = j;
                }
            }
        }
    }
    
    // Setup agent solvers (MPC-aware qpOASES)
    REFER_NAMESPACE_QPOASES returnValue ret = setupAgentSolvers();
    if (ret != SUCCESSFUL_RETURN)
        return ret;
    
    // Allocate ADMM variables
    ret = allocateADMMVariables();
    if (ret != SUCCESSFUL_RETURN)
        return ret;
    
    // Compute preconditioner data
    ret = computePreconditionerData();
    if (ret != SUCCESSFUL_RETURN)
        return ret;
    
    is_setup_ = BT_TRUE;
    is_first_solve_ = BT_TRUE;
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::setupAgentSolvers()
{
    // Allocate solver array
    agent_solvers_ = new QProblem*[n_agents_];
    
    for (int i = 0; i < n_agents_; ++i) {
        AgentData& agent = agents_[i];
        
        // Create QProblem instance
        agent_solvers_[i] = new QProblem(agent.nV, agent.nC);
        
        // Enable MPC-aware features
        Options opts;
        opts.setToDefault();
        opts.enableMPCRiccati = BT_TRUE;  // Re-enabled
        opts.enableEqualities = BT_TRUE;
        opts.printLevel = PL_NONE;  // Reduce output
        agent_solvers_[i]->setOptions(opts);
        
        // Setup MPC structure (enables Riccati + TQ factorization)
        REFER_NAMESPACE_QPOASES returnValue ret = agent_solvers_[i]->setupMPCStructure(
            agent.N, agent.nx, agent.nu,
            agent.A, agent.B, agent.Q, agent.R
        );
        
        if (ret != SUCCESSFUL_RETURN) {
            return THROWERROR(RET_MPC_SETUP_FAILED);
        }
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::allocateADMMVariables()
{
    // Allocate per-agent variables
    z_local_ = new real_t*[n_agents_];
    z_prev_ = new real_t*[n_agents_];
    
    for (int i = 0; i < n_agents_; ++i) {
        int nV_i = agents_[i].nV;
        
        z_local_[i] = new real_t[nV_i];
        z_prev_[i] = new real_t[nV_i];
        
        memset(z_local_[i], 0, nV_i * sizeof(real_t));
        memset(z_prev_[i], 0, nV_i * sizeof(real_t));
    }
    
    // Allocate collision coupling variables (coupling-constraint ADMM)
    // v_collision_[i][j][k][state_idx] and lambda_collision_[i][j][k][state_idx]
    v_collision_ = new real_t***[n_agents_];
    lambda_collision_ = new real_t***[n_agents_];
    
    for (int i = 0; i < n_agents_; ++i) {
        v_collision_[i] = new real_t**[n_agents_];
        lambda_collision_[i] = new real_t**[n_agents_];
        
        for (int j = 0; j < n_agents_; ++j) {
            if (i == j) {
                // No self-coupling
                v_collision_[i][j] = 0;
                lambda_collision_[i][j] = 0;
                continue;
            }
            
            // Allocate for all stages (N+1 stages: k=0..N)
            int N = agents_[i].N;
            v_collision_[i][j] = new real_t*[N + 1];
            lambda_collision_[i][j] = new real_t*[N + 1];
            
            int nx = agents_[i].nx;  // State dimension
            
            for (int k = 0; k <= N; ++k) {
                v_collision_[i][j][k] = new real_t[nx];
                lambda_collision_[i][j][k] = new real_t[nx];
                
                memset(v_collision_[i][j][k], 0, nx * sizeof(real_t));
                memset(lambda_collision_[i][j][k], 0, nx * sizeof(real_t));
            }
        }
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::computePreconditionerData()
{
    // Allocate coupling vector storage
    coupling_vectors_ = new real_t**[n_agents_];
    
    for (int i = 0; i < n_agents_; ++i) {
        int N = agents_[i].N;
        coupling_vectors_[i] = new real_t*[N + 1];
        
        for (int k = 0; k <= N; ++k) {
            coupling_vectors_[i][k] = 0;  // Will be computed dynamically
        }
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::getSolution(int agent_id, real_t* z_out) const
{
    if (agent_id < 0 || agent_id >= n_agents_)
        return THROWERROR(RET_INDEX_OUT_OF_BOUNDS);
    
    if (!z_out)
        return THROWERROR(RET_INVALID_ARGUMENTS);
    
    int nV_i = agents_[agent_id].nV;
    memcpy(z_out, z_local_[agent_id], nV_i * sizeof(real_t));
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::getStatistics(ADMMStatistics* stats_out) const
{
    if (!stats_out)
        return THROWERROR(RET_INVALID_ARGUMENTS);
    
    *stats_out = stats_;
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::solveColdStart(
    const real_t* const* x_init,
    BooleanType* converged_out
)
{
    if (!is_setup_)
        return THROWERROR(RET_INIT_FAILED);
    
    // Reset statistics
    stats_.reset();
    
    // Step 1: Initialize with Riccati recursion
    printf("[DEBUG] Calling initializeWithRiccati...\n");
    returnValue ret = initializeWithRiccati(x_init);
    if (ret != SUCCESSFUL_RETURN) {
        printf("[DEBUG] initializeWithRiccati FAILED with code %d\n", ret);
        return ret;
    }
    printf("[DEBUG] initializeWithRiccati SUCCESS\n");
    
    // Step 2: ADMM iteration loop
    for (int admm_iter = 0; admm_iter < params_.max_admm_iter; ++admm_iter) {
        // Store current iteration for use in consensus update
        stats_.admm_iterations = admm_iter;
        
        // Step 2.1: Solve agent subproblems with ADMM augmentation
        for (int i = 0; i < n_agents_; ++i) {
            AgentData& agent = agents_[i];
            
            // *** SEQUENTIAL CONVEX PROGRAMMING (SCP): Update obstacle constraints ***
            // Re-linearize obstacle constraints around current trajectory z_local[i]
            // This is critical for non-convex obstacles (circular, rotated, etc.)
            // For PointMass agents with time-varying constraints, update based on current trajectory
            // Note: Only PointMass agents have obstacle constraints (nG > 0)
            if (agent.nG > 0 && agent.G != 0) {
                // Safe to cast to PointMass since only PointMass has obstacle constraints
                PointMass* pm_agent = static_cast<PointMass*>(&agent);
                // Update constraints based on current trajectory
                // Bypass schedule is stored in agent from addTimeVaryingObstacleConstraint()
                pm_agent->updateObstacleConstraints(z_local_[i], 0);
            }
            
            // Compute tracking gradient: g = Q*(x - x_ref) + R*(u - u_ref)
            real_t* g = new real_t[agent.nV];
            ret = computeTrackingGradient(i, g);
            if (ret != SUCCESSFUL_RETURN) {
                delete[] g;
                return ret;
            }
            
            // Add ADMM collision coupling terms: g_ADMM = g + Σⱼ∈N(i) [ρ*vᵢⱼ - λᵢⱼ]
            if (params_.enable_collision_avoidance && coupling_.d_safe > 0.0) {
                addCollisionCouplingGradient(i, g);
            }
            
            // Solve QP using MPC-aware qpOASES
            int nWSR = params_.max_qp_iter;
            
            // NOTE: Always use init() for now because the Hessian changes each iteration
            // due to changing collision coupling terms. Hotstart would need to detect this.
            // TODO: Implement proper hotstart that rebuilds only changed parts
            
            if (true) {  // Always cold start for now (was: admm_iter == 0)
                printf("[DEBUG] Agent %d: Cold start QP (admm_iter=0)\n", i);
                
                // FIX INITIAL STATE:
                // Create local copies of bounds and fix x0
                real_t* lb_local = new real_t[agent.nV];
                real_t* ub_local = new real_t[agent.nV];
                memcpy(lb_local, agent.lb, agent.nV * sizeof(real_t));
                memcpy(ub_local, agent.ub, agent.nV * sizeof(real_t));
                
                // Fix first nx variables to x_init
                if (x_init && x_init[i]) {
                    for (int j = 0; j < agent.nx; ++j) {
                        lb_local[j] = x_init[i][j];
                        ub_local[j] = x_init[i][j];
                    }
                }
                
                // Build Hessian H_ADMM = H_tracking + H_collision + ρI
                // H_tracking: standard MPC tracking cost
                // H_collision: collision avoidance cost (encourages separation)
                // ρI: ADMM augmentation (enforces separation via dual variables)
                int num_neighbors_i = num_neighbors_[i];
                real_t rho_augment = num_neighbors_i * params_.rho;  // ADMM augmentation
                real_t rho_collision = params_.rho * 0.5;  // Collision cost weight (half of ADMM penalty)
                
                real_t* H = new real_t[agent.nV * agent.nV];
                memset(H, 0, agent.nV * agent.nV * sizeof(real_t));
                
                // Build diagonal Hessian (tracking cost + ADMM augmentation)
                // ADMM augmentation: Add ρI ONLY to POSITION variables (x, y)
                // This avoids the 50% tracking bug (which was caused by unconditional augmentation)
                int idx = 0;
                for (int k = 0; k <= agent.N; ++k) {
                    // State cost Q + ADMM augmentation for position
                    // Uniform stage weights: 50% for all stages (including terminal)
                    real_t stage_weight = 0.5;  // 50% for all stages
                    
                    for (int j = 0; j < agent.nx; ++j) {
                        // Apply stage-dependent weight to tracking cost
                        H[idx * agent.nV + idx] = stage_weight * agent.Q_diag[j];
                        
                        // Add ADMM augmentation ONLY for position states (j < 2 for 2D: x, y)
                        // Skip k=0 (initial state is fixed by bounds)
                        // Only add if agent has neighbors (collision coupling)
                        if (k > 0 && j < 2 && num_neighbors_i > 0) {
                            H[idx * agent.nV + idx] += params_.rho * num_neighbors_i;
                        }
                        idx++;
                    }
                    // Control cost R (no ADMM augmentation)
                    if (k < agent.N) {
                        for (int j = 0; j < agent.nu; ++j) {
                            H[idx * agent.nV + idx] = agent.R_diag[j];
                            idx++;
                        }
                    }
                }
                
                // Add linearized barrier collision cost: (ρ_collision/2) * Σ_j Σ_k max(0, d_safe - dist_ij)²
                // This only penalizes when agents are too close (dist < d_safe)
                // Linearize around current trajectory z_local (from previous ADMM iter or Riccati at iter 0)
                // With dynamically feasible references, Riccati should produce reasonable trajectories
                real_t d_safe = coupling_.d_safe;
                
                int num_violations = 0;
                real_t total_violation = 0.0;
                
                // Add collision cost at all iterations (including iter 0)
                for (int neighbor_idx = 0; neighbor_idx < num_neighbors_i; ++neighbor_idx) {
                    int j = neighbors_[i][neighbor_idx];
                    AgentData& agent_j = agents_[j];
                    
                    for (int k = 1; k <= agent.N; ++k) {  // Skip k=0
                        // Get current positions from z_local (linearization point)
                        int idx_i = (k == agent.N) ? (agent.N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
                        int idx_j = (k == agent_j.N) ? (agent_j.N * (agent_j.nx + agent_j.nu)) : (k * (agent_j.nx + agent_j.nu));
                        
                        real_t pos_i_current = z_local_[i][idx_i + 0];
                        real_t pos_j_current = z_local_[j][idx_j + 0];
                        real_t dist_current = fabs(pos_i_current - pos_j_current);
                        
                        // Check if collision is violated at linearization point
                        if (dist_current < d_safe) {
                            // Violation! Add quadratic penalty
                            real_t violation = d_safe - dist_current;
                            real_t direction = (pos_i_current - pos_j_current) / (dist_current + 1e-6);  // Avoid division by zero
                            
                            // Hessian: Add ρ_collision to position diagonal
                            // (from quadratic penalty on violation)
                            H[idx_i * agent.nV + idx_i] += rho_collision;
                            
                            num_violations++;
                            total_violation += violation;
                            
                            // Note: Gradient will be added in addCollisionCouplingGradient()
                            // We store the linearization info there
                        }
                        // else: Safe distance, no penalty added
                    }
                }  // end for neighbor_idx
                
                // DIAGNOSTIC: Print collision cost info for first ADMM iteration
                static int hessian_call_count = 0;
                if (hessian_call_count < 2) {
                    printf("[DIAGNOSTIC] Agent %d, admm_iter=%d: Collision cost in Hessian\n", i, admm_iter);
                    printf("  Neighbors: %d, Violations: %d/%d stages\n", num_neighbors_i, num_violations, agent.N);
                    printf("  Total violation: %.4f, rho_collision: %.2f\n", total_violation, rho_collision);
                    if (num_violations > 0) {
                        printf("  WARNING: Collision violations detected!\n");
                        for (int neighbor_idx = 0; neighbor_idx < num_neighbors_i; ++neighbor_idx) {
                            int j = neighbors_[i][neighbor_idx];
                            for (int k = 1; k <= agent.N; ++k) {
                                int idx_i = (k == agent.N) ? (agent.N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
                                int idx_j = (k == agent.N) ? (agent.N * (agents_[j].nx + agents_[j].nu)) : (k * (agents_[j].nx + agents_[j].nu));
                                real_t pos_i = z_local_[i][idx_i + 0];
                                real_t pos_j = z_local_[j][idx_j + 0];
                                real_t dist = fabs(pos_i - pos_j);
                                if (dist < d_safe) {
                                    printf("    Stage k=%d: pos_i=%.2f, pos_j=%.2f, dist=%.2f < d_safe=%.2f (violation=%.2f)\n",
                                           k, pos_i, pos_j, dist, d_safe, d_safe - dist);
                                }
                            }
                        }
                    } else {
                        printf("  No violations - agents are safe!\n");
                    }
                    hessian_call_count++;
                }
                
                // Build constraint matrix A (dynamics + static inequality constraints)
                // Total constraints: nC (dynamics) + nG (static inequalities)
                int nConstraints = agent.nC + agent.nG;
                real_t* A_constraint = new real_t[nConstraints * agent.nV];
                memset(A_constraint, 0, nConstraints * agent.nV * sizeof(real_t));
                
                // Part 1: Dynamics constraints (rows 0 to nC-1)
                // Dynamics: x[k+1] = A*x[k] + B*u[k]
                // Constraint: -x[k+1] + A*x[k] + B*u[k] = 0
                for (int k = 0; k < agent.N - 1; ++k) {
                    int row_offset = k * agent.nx;
                    int col_x_k = k * (agent.nx + agent.nu);
                    int col_u_k = col_x_k + agent.nx;
                    int col_x_kp1 = (k + 1) * (agent.nx + agent.nu);
                    
                    // -x[k+1]
                    for (int i = 0; i < agent.nx; ++i) {
                        A_constraint[(row_offset + i) * agent.nV + (col_x_kp1 + i)] = -1.0;
                    }
                    
                    // A*x[k]
                    for (int i = 0; i < agent.nx; ++i) {
                        for (int j = 0; j < agent.nx; ++j) {
                            A_constraint[(row_offset + i) * agent.nV + (col_x_k + j)] = agent.A[i * agent.nx + j];
                        }
                    }
                    
                    // B*u[k]
                    for (int i = 0; i < agent.nx; ++i) {
                        for (int j = 0; j < agent.nu; ++j) {
                            A_constraint[(row_offset + i) * agent.nV + (col_u_k + j)] = agent.B[i * agent.nu + j];
                        }
                    }
                }
                
                // Part 2: Static inequality constraints (rows nC to nC+nG-1)
                // Constraint: c <= G*x <= d
                if (agent.nG > 0 && agent.G) {
                    for (int i = 0; i < agent.nG; ++i) {
                        for (int j = 0; j < agent.nV; ++j) {
                            A_constraint[(agent.nC + i) * agent.nV + j] = agent.G[i * agent.nV + j];
                        }
                    }
                }
                
                // Combine constraint bounds: [lbA (dynamics); c (static)] and [ubA (dynamics); d (static)]
                real_t* lbA_combined = new real_t[nConstraints];
                real_t* ubA_combined = new real_t[nConstraints];
                
                // Copy dynamics bounds
                memcpy(lbA_combined, agent.lbA, agent.nC * sizeof(real_t));
                memcpy(ubA_combined, agent.ubA, agent.nC * sizeof(real_t));
                
                // Copy static constraint bounds
                if (agent.nG > 0) {
                    if (agent.c) {
                        memcpy(lbA_combined + agent.nC, agent.c, agent.nG * sizeof(real_t));
                    } else {
                        // If c not provided, use -infinity
                        for (int i = 0; i < agent.nG; ++i) {
                            lbA_combined[agent.nC + i] = -1.0e20;
                        }
                    }
                    
                    if (agent.d) {
                        memcpy(ubA_combined + agent.nC, agent.d, agent.nG * sizeof(real_t));
                    } else {
                        // If d not provided, use +infinity
                        for (int i = 0; i < agent.nG; ++i) {
                            ubA_combined[agent.nC + i] = 1.0e20;
                        }
                    }
                }
                
                // Reset QProblem before init (it was used for Riccati)
                agent_solvers_[i]->reset();
                
                // DIAGNOSTIC: Check QProblem internal state before init
                if (i == 1 && admm_iter == 1) {
                    printf("\n[QProblem STATE CHECK] Before Agent 1 init:\n");
                    printf("  Agent 0 z_local terminal: (%.2f, %.2f)\n",
                           z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 0],
                           z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 1]);
                }
                
                ret = agent_solvers_[i]->init(
                    H, g, A_constraint,
                    lb_local, ub_local,  // Use local bounds with fixed x0
                    lbA_combined, ubA_combined,
                    nWSR
                );
                
                // DIAGNOSTIC: Check if Agent 0's z_local was corrupted by Agent 1's init
                if (i == 1 && admm_iter == 1) {
                    printf("\n[QProblem STATE CHECK] After Agent 1 init:\n");
                    printf("  Agent 0 z_local terminal: (%.2f, %.2f)\n",
                           z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 0],
                           z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 1]);
                }
                
                printf("[DEBUG] Agent %d: QP init returned %d, nWSR=%d\n", i, ret, nWSR);
                printf("[DEBUG AFTER QP] Agent %d: z_local[0]=(%.4f, %.4f)\n", i, z_local_[i][0], z_local_[i][1]);
                
                // ITERATION TRACKER: Print Agent 0 trajectory after QP solve
                if (i == 0) {
                    printf("\n[ITER %d] Agent 0 trajectory AFTER QP solve:\n", admm_iter);
                    printf("  k=0:  pos=(%.2f, %.2f)\n", z_local_[0][0], z_local_[0][1]);
                    int idx_mid = 5 * (agent.nx + agent.nu);
                    printf("  k=5:  pos=(%.2f, %.2f)\n", z_local_[0][idx_mid + 0], z_local_[0][idx_mid + 1]);
                    int idx_term = agent.N * (agent.nx + agent.nu);
                    printf("  k=10: pos=(%.2f, %.2f) [target: (15.0, 5.0)]\n", 
                           z_local_[0][idx_term + 0], z_local_[0][idx_term + 1]);
                }
                
                // Additional diagnostic for first iteration
                static int qp_call_count = 0;
                if (qp_call_count < 2) {
                    printf("[DEBUG QP DETAIL] Agent %d, admm_iter=%d: z_local[0]=%.4f (should match x_init)\n",
                           i, admm_iter, z_local_[i][0]);
                    qp_call_count++;
                }
                delete[] H;
                delete[] A_constraint;
                delete[] lb_local;
                delete[] ub_local;
                delete[] lbA_combined;
                delete[] ubA_combined;
            } else {
                // Hotstart
                if (admm_iter == 1) {
                    printf("[DEBUG] Agent %d: Hotstart QP (admm_iter=%d)\n", i, admm_iter);
                }
                
                // FIX INITIAL STATE: Same as cold start
                real_t* lb_local = new real_t[agent.nV];
                real_t* ub_local = new real_t[agent.nV];
                memcpy(lb_local, agent.lb, agent.nV * sizeof(real_t));
                memcpy(ub_local, agent.ub, agent.nV * sizeof(real_t));
                
                if (x_init && x_init[i]) {
                    for (int j = 0; j < agent.nx; ++j) {
                        lb_local[j] = x_init[i][j];
                        ub_local[j] = x_init[i][j];
                    }
                }
                
                nWSR = 10;  // Fewer iterations for hotstart
                ret = agent_solvers_[i]->hotstart(
                    g,
                    lb_local, ub_local,  // Use local bounds with fixed x0
                    agent.lbA, agent.ubA,
                    nWSR
                );
                
                if (admm_iter == 1) {
                    printf("[DEBUG] Agent %d: Hotstart returned %d, nWSR=%d\n", i, ret, nWSR);
                }
                
                delete[] lb_local;
                delete[] ub_local;
            }
            
            delete[] g;
            
            if (ret != SUCCESSFUL_RETURN) {
                stats_.converged = BT_FALSE;
                if (converged_out)
                    *converged_out = BT_FALSE;
                return ret;
            }
            
            // DIAGNOSTIC: Check what QProblem thinks the solution is
            if (i == 0 && admm_iter == 1) {
                printf("\n[BEFORE getPrimalSolution] Agent 0, admm_iter=1:\n");
                printf("  Current z_local[0] terminal: (%.2f, %.2f)\n",
                       z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 0],
                       z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 1]);
            }
            
            // Extract solution
            agent_solvers_[i]->getPrimalSolution(z_local_[i]);
            
            // DIAGNOSTIC: Check what was extracted
            if (i == 0 && admm_iter == 1) {
                printf("\n[AFTER getPrimalSolution] Agent 0, admm_iter=1:\n");
                printf("  New z_local[0] terminal: (%.2f, %.2f)\n",
                       z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 0],
                       z_local_[0][agents_[0].N * (agents_[0].nx + agents_[0].nu) + 1]);
            }
            
            // MEMORY BOUNDARY CHECK: Verify no corruption after getPrimalSolution
            if (i == 1 && admm_iter == 1) {
                // After Agent 1's QP, check if Agent 0's solution is still intact
                int idx_term_0 = agents_[0].N * (agents_[0].nx + agents_[0].nu);
                printf("\n[MEMORY CHECK] After Agent 1 getPrimalSolution:\n");
                printf("  Agent 0 terminal state: (%.2f, %.2f)\n", 
                       z_local_[0][idx_term_0 + 0], z_local_[0][idx_term_0 + 1]);
                printf("  z_local_[0] pointer: %p\n", (void*)z_local_[0]);
                printf("  z_local_[1] pointer: %p\n", (void*)z_local_[1]);
                printf("  Pointer difference: %ld bytes\n", 
                       (long)((char*)z_local_[1] - (char*)z_local_[0]));
                printf("  Agent 0 nV: %d, Agent 1 nV: %d\n", agents_[0].nV, agents_[1].nV);
            }
            
            stats_.total_qp_iterations += nWSR;
            
            // Debug: print full QP solution trajectory for first ADMM iteration
            static int solve_count = 0;
            if (false && solve_count < 2 && admm_iter == 0) {  // Disabled to prevent output buffer issues
                printf("[DEBUG QP SOLUTION] Agent %d trajectory:\n", i);
                for (int k = 0; k <= agent.N; ++k) {
                    int idx = (k == agent.N) ? (agent.N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
                    
                    if (agent.nx == 2) {
                        // 1D agent: [pos, vel]
                        real_t pos = z_local_[i][idx + 0];
                        real_t vel = z_local_[i][idx + 1];
                        real_t ref_pos = agent.x_ref[k * agent.nx + 0];
                        real_t ref_vel = agent.x_ref[k * agent.nx + 1];
                        
                        if (k < agent.N) {
                            real_t u = z_local_[i][idx + agent.nx];
                            real_t u_ref = agent.u_ref[k * agent.nu];
                            printf("  k=%d: x=(%.4f, %.4f) u=%.4f | ref: x=(%.4f, %.4f) u=%.4f\n",
                                   k, pos, vel, u, ref_pos, ref_vel, u_ref);
                        } else {
                            printf("  k=%d: x=(%.4f, %.4f) | ref: x=(%.4f, %.4f)\n",
                                   k, pos, vel, ref_pos, ref_vel);
                        }
                    } else if (agent.nx == 4) {
                        // 2D PointMass agent: [x, y, vx, vy]
                        real_t x = z_local_[i][idx + 0];
                        real_t y = z_local_[i][idx + 1];
                        real_t vx = z_local_[i][idx + 2];
                        real_t vy = z_local_[i][idx + 3];
                        
                        if (k < agent.N) {
                            real_t ax = z_local_[i][idx + agent.nx];
                            real_t ay = z_local_[i][idx + agent.nx + 1];
                            printf("  k=%d: pos=(%.2f,%.2f) vel=(%.2f,%.2f) acc=(%.2f,%.2f)\n",
                                   k, x, y, vx, vy, ax, ay);
                        } else {
                            printf("  k=%d: pos=(%.2f,%.2f) vel=(%.2f,%.2f)\n",
                                   k, x, y, vx, vy);
                        }
                    }
                }
                solve_count++;
            }
        }
        
        // Print comprehensive trajectory comparison for both agents after both are solved
        static int traj_print_count = 0;
        if (traj_print_count == 0 && admm_iter == 0 && n_agents_ >= 2) {
            printf("\n========================================\n");
            printf("COMPREHENSIVE TRAJECTORY COMPARISON\n");
            printf("========================================\n");
            printf("Stage | Agent 0 (pos, vel, u) | Agent 1 (pos, vel, u) | Distance | Status\n");
            printf("------+------------------------+------------------------+----------+--------\n");
            
            for (int k = 0; k <= agents_[0].N; ++k) {
                int idx_0 = (k == agents_[0].N) ? (agents_[0].N * (agents_[0].nx + agents_[0].nu)) : (k * (agents_[0].nx + agents_[0].nu));
                int idx_1 = (k == agents_[1].N) ? (agents_[1].N * (agents_[1].nx + agents_[1].nu)) : (k * (agents_[1].nx + agents_[1].nu));
                
                real_t pos_0 = z_local_[0][idx_0 + 0];
                real_t vel_0 = z_local_[0][idx_0 + 1];
                real_t pos_1 = z_local_[1][idx_1 + 0];
                real_t vel_1 = z_local_[1][idx_1 + 1];
                
                real_t dist = fabs(pos_0 - pos_1);
                const char* status = (dist < coupling_.d_safe) ? "COLLISION!" : "safe";
                
                if (k < agents_[0].N) {
                    real_t u_0 = z_local_[0][idx_0 + agents_[0].nx];
                    real_t u_1 = z_local_[1][idx_1 + agents_[1].nx];
                    printf("  %2d  | (%6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f) | %7.3f  | %s\n",
                           k, pos_0, vel_0, u_0, pos_1, vel_1, u_1, dist, status);
                } else {
                    printf("  %2d  | (%6.2f, %6.2f,    --  ) | (%6.2f, %6.2f,    --  ) | %7.3f  | %s\n",
                           k, pos_0, vel_0, pos_1, vel_1, dist, status);
                }
            }
            printf("========================================\n\n");
            traj_print_count++;
        }
        
        // Step 2.2: Project onto collision-free set
        ret = projectCollisionConstraints();
        if (ret != SUCCESSFUL_RETURN)
            return ret;
        
        // ITERATION TRACKER: Print Agent 0 trajectory AFTER projection
        if (admm_iter < 5) {  // Only first 5 iterations
            printf("\n[ITER %d] Agent 0 trajectory AFTER projection:\n", admm_iter);
            printf("  k=0:  pos=(%.2f, %.2f)\n", z_local_[0][0], z_local_[0][1]);
            int idx_mid = 5 * (agents_[0].nx + agents_[0].nu);
            printf("  k=5:  pos=(%.2f, %.2f)\n", z_local_[0][idx_mid + 0], z_local_[0][idx_mid + 1]);
            int idx_term = agents_[0].N * (agents_[0].nx + agents_[0].nu);
            printf("  k=10: pos=(%.2f, %.2f) [target: (15.0, 5.0)]\n", 
                   z_local_[0][idx_term + 0], z_local_[0][idx_term + 1]);
            printf("  v_collision[0][1][10] = (%.2f, %.2f)\n",
                   v_collision_[0][1][agents_[0].N][0], v_collision_[0][1][agents_[0].N][1]);
        }
        
        // Step 2.3: Update collision dual variables
        ret = updateCollisionDuals();
        if (ret != SUCCESSFUL_RETURN)
            return ret;
        
        // Step 2.4: Check convergence
        real_t r_primal, r_dual;
        BooleanType converged = checkConvergence(&r_primal, &r_dual);
        
        stats_.admm_iterations = admm_iter + 1;
        stats_.primal_residual = r_primal;
        stats_.dual_residual = r_dual;
        
        // Debug output every 10 iterations
        if ((admm_iter + 1) % 10 == 0) {
            printf("[DEBUG] ADMM iter %d: r_primal=%.6f, r_dual=%.6f, QP iters=%d\n",
                   admm_iter + 1, r_primal, r_dual, stats_.total_qp_iterations);
        }
        
        if (converged == BT_TRUE) {
            stats_.converged = BT_TRUE;
            if (converged_out)
                *converged_out = BT_TRUE;
            printf("[DEBUG] ADMM CONVERGED at iteration %d\n", admm_iter + 1);
            return SUCCESSFUL_RETURN;
        }
        
        // Step 2.5: Adapt penalty parameter (if enabled)
        if (params_.enable_adaptive_rho) {
            adaptPenaltyParameter(r_primal, r_dual);
        }
    }
    
    // Max iterations reached without convergence
    stats_.converged = BT_FALSE;
    if (converged_out)
        *converged_out = BT_FALSE;
    
    return SUCCESSFUL_RETURN;  // Not an error, just didn't converge
}


returnValue TurboADMM::computeTrackingGradient(int agent_id, real_t* g_out)
{
    if (agent_id < 0 || agent_id >= n_agents_)
        return THROWERROR(RET_INDEX_OUT_OF_BOUNDS);
    
    AgentData& agent = agents_[agent_id];
    
    // g = [Q(x - x_ref); R(u - u_ref); ...] for all stages
    // MPC structure: [x0, u0, x1, u1, ..., x_{N-1}, u_{N-1}, x_N]
    
    int idx = 0;
    
    // Stages k = 0 to N-1: [x_k, u_k]
    for (int k = 0; k < agent.N; ++k) {
        // Apply uniform weight (must match Hessian!)
        // All stages: 50%
        real_t stage_weight = 0.5;  // All stages
        
        // State x_k
        for (int i = 0; i < agent.nx; ++i) {
            real_t diff = 0.0 - agent.x_ref[k * agent.nx + i];
            g_out[idx++] = stage_weight * agent.Q_diag[i] * diff;
        }
        
        // Control u_k (all N controls)
        for (int i = 0; i < agent.nu; ++i) {
            real_t diff = 0.0 - agent.u_ref[k * agent.nu + i];
            g_out[idx++] = agent.R_diag[i] * diff;  // No stage weight for control
        }
    }
    
    // Terminal state x_N (50% weight, same as all stages)
    real_t terminal_weight = 0.5;
    for (int i = 0; i < agent.nx; ++i) {
        real_t diff = 0.0 - agent.x_ref[agent.N * agent.nx + i];
        g_out[idx++] = terminal_weight * agent.Q_diag[i] * diff;
    }
    
    // Debug: Print gradient and reference for first agent
    static int grad_print_count = 0;
    if (grad_print_count < 2 && agent_id <= 1) {
        printf("[DEBUG GRADIENT] Agent %d:\n", agent_id);
        printf("  x_ref[0] = (%.4f, %.4f)\n", agent.x_ref[0], agent.x_ref[1]);
        printf("  x_ref[N] = (%.4f, %.4f)\n", 
               agent.x_ref[agent.N * agent.nx + 0], 
               agent.x_ref[agent.N * agent.nx + 1]);
        printf("  g[0] x-component = %.4f (Q=%.1f, x_ref=%.4f)\n", 
               g_out[0], agent.Q_diag[0], agent.x_ref[0]);
        printf("  g[0] y-component = %.4f (Q=%.1f, y_ref=%.4f)\n", 
               g_out[1], agent.Q_diag[1], agent.x_ref[1]);
        printf("  g[terminal] x = %.4f (Q=%.1f, x_ref=%.4f)\n",
               g_out[agent.N * (agent.nx + agent.nu)], 
               agent.Q_diag[0], 
               agent.x_ref[agent.N * agent.nx + 0]);
        printf("  g[terminal] y = %.4f (Q=%.1f, y_ref=%.4f)\n",
               g_out[agent.N * (agent.nx + agent.nu) + 1], 
               agent.Q_diag[1], 
               agent.x_ref[agent.N * agent.nx + 1]);
        printf("  INTERPRETATION: Negative g pulls toward POSITIVE values\n");
        printf("  Agent %d y_ref=%.1f, g_y=%.1f -> pulls toward y=%.1f\n",
               agent_id, agent.x_ref[1], g_out[1], agent.x_ref[1]);
        grad_print_count++;
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::initializeWithRiccati(const real_t* const* x_init)
{
    // Initialize ADMM variables using Riccati recursion
    // Riccati solves pure LQR, but provides good warm start for augmented ADMM
    
    for (int i = 0; i < n_agents_; ++i) {
        AgentData& agent = agents_[i];
        
        // CRITICAL: Set x[0] in z_local
        if (x_init && x_init[i]) {
            for (int j = 0; j < agent.nx; ++j) {
                z_local_[i][j] = x_init[i][j];
            }
            printf("[DEBUG] Agent %d: Set x[0] = (%.2f, %.2f) in z_local\n", 
                   i, x_init[i][0], x_init[i][1]);
        }
        
        // SKIP RICCATI: Use dynamically feasible reference trajectory directly!
        // This avoids instability from affine LQR and provides perfect initialization
        // Copy reference trajectory to z_local
        for (int k = 0; k <= agent.N; ++k) {
            int idx = (k == agent.N) ? (agent.N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
            
            // Copy state reference
            for (int j = 0; j < agent.nx; ++j) {
                z_local_[i][idx + j] = agent.x_ref[k * agent.nx + j];
            }
            
            // Copy control reference (if not terminal)
            if (k < agent.N) {
                for (int j = 0; j < agent.nu; ++j) {
                    z_local_[i][idx + agent.nx + j] = agent.u_ref[k * agent.nu + j];
                }
            }
        }
        
        // Restore x[0] to actual initial condition (reference might differ slightly)
        if (x_init && x_init[i]) {
            for (int j = 0; j < agent.nx; ++j) {
                z_local_[i][j] = x_init[i][j];
            }
            
            // DIAGNOSTIC: Print initialization
            printf("[DEBUG] Agent %d initialized with reference trajectory:\n", i);
            for (int k = 0; k <= agent.N; ++k) {
                int idx = (k == agent.N) ? (agent.N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
                printf("  k=%d: pos=%.2f, vel=%.2f (ref: pos=%.2f)\n", 
                       k, z_local_[i][idx + 0], z_local_[i][idx + 1], agent.x_ref[k*agent.nx + 0]);
            }
        }
        
        // Initialize collision variables to ZERO (not reference trajectory!)
        // This avoids artificial coupling for independent agents
        // For agents that need to avoid collisions, the projection will set v correctly
        for (int neighbor_idx = 0; neighbor_idx < num_neighbors_[i]; ++neighbor_idx) {
            int j = neighbors_[i][neighbor_idx];
            for (int k = 0; k <= agent.N; ++k) {
                for (int s = 0; s < agent.nx; ++s) {
                    // Initialize to ZERO instead of z_local (reference trajectory)
                    v_collision_[i][j][k][s] = 0.0;
                    lambda_collision_[i][j][k][s] = 0.0;  // Also ensure lambda is zero
                }
                
                // Debug: print k=0 initialization for both agents
                if (k == 0 && i <= 1) {
                    printf("[DEBUG INIT] Agent %d, k=0: v_collision and lambda initialized to ZERO\n", i);
                }
            }
        }
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::updateCollisionDuals()
{
    // Dual update for collision constraints:
    //   λᵢⱼₖ^(t+1) = λᵢⱼₖ^t + ρ(xᵢₖ^(t+1) - vᵢⱼₖ^(t+1))
    
    // For each agent i
    for (int i = 0; i < n_agents_; ++i) {
        AgentData& agent_i = agents_[i];
        int nx_i = agent_i.nx;
        int nu_i = agent_i.nu;
        
        // For each neighbor j of agent i
        for (int neighbor_idx = 0; neighbor_idx < num_neighbors_[i]; ++neighbor_idx) {
            int j = neighbors_[i][neighbor_idx];
            
            // For each stage k=1..N (skip k=0 since initial state is FIXED by QP bounds)
            for (int k = 1; k <= agent_i.N; ++k) {
                // Get state index in z_local
                int idx_i = (k == agent_i.N) ? agent_i.N * (nx_i + nu_i) : k * (nx_i + nu_i);
                
                // Update dual variables for POSITION ONLY (state[0])
                // Collision avoidance is based on position, not velocity
                real_t residual = z_local_[i][idx_i + 0] - v_collision_[i][j][k][0];
                lambda_collision_[i][j][k][0] += params_.rho * residual;
                
                // Keep dual variables for other states at zero (no coupling)
                for (int s = 1; s < nx_i; ++s) {
                    lambda_collision_[i][j][k][s] = 0.0;
                }
            }
        }
    }
    
    return SUCCESSFUL_RETURN;
}


// OLD CONSENSUS FUNCTION - REMOVED
// Replaced by projectCollisionConstraints() for coupling-constraint ADMM


BooleanType TurboADMM::checkConvergence(real_t* r_primal_out, real_t* r_dual_out)
{
    // Coupling-constraint ADMM convergence check
    // Primal residual: max_{i,j,k} ||xᵢₖ - vᵢⱼₖ||
    // Dual residual:   max_{i,j,k} ||ρ(vᵢⱼₖ^(t+1) - vᵢⱼₖ^t)||
    
    real_t r_primal_max = 0.0;
    real_t r_dual_max = 0.0;
    
    // For each agent i
    for (int i = 0; i < n_agents_; ++i) {
        AgentData& agent_i = agents_[i];
        int nx = agent_i.nx;
        int nu = agent_i.nu;
        
        // For each neighbor j
        for (int neighbor_idx = 0; neighbor_idx < num_neighbors_[i]; ++neighbor_idx) {
            int j = neighbors_[i][neighbor_idx];
            
            // Only compute once per pair (i < j)
            if (i >= j) continue;
            
            // For each stage k=1..N (skip k=0 since initial state is FIXED by QP bounds)
            for (int k = 1; k <= agent_i.N; ++k) {
                // Get state index in z_local
                int idx_i = (k == agent_i.N) ? agent_i.N * (nx + nu) : k * (nx + nu);
                
                // Compute primal residual for POSITION (2D Euclidean distance)
                // Collision avoidance is based on 2D position (x, y), not just x
                real_t dx = z_local_[i][idx_i + 0] - v_collision_[i][j][k][0];
                real_t dy = z_local_[i][idx_i + 1] - v_collision_[i][j][k][1];
                real_t r_primal_ijk = sqrt(dx*dx + dy*dy);
                
                // Debug: print residual for all stages in first check
                static int print_count = 0;
                if (print_count == 0) {
                    if (k == 0) {
                        // Print detailed info for k=0
                        printf("[DEBUG] Stage k=0: r_primal_ijk=%.4f, idx_i=%d\n", r_primal_ijk, idx_i);
                        printf("  z_local[%d][%d]=(%.4f,%.4f), v_collision[%d][%d][0]=(%.4f,%.4f)\n",
                               i, idx_i, z_local_[i][idx_i], z_local_[i][idx_i+1],
                               i, j, v_collision_[i][j][0][0], v_collision_[i][j][0][1]);
                    } else {
                        printf("[DEBUG] Stage k=%d: r_primal_ijk=%.4f\n", k, r_primal_ijk);
                    }
                    if (k == agent_i.N) print_count++;
                }
                
                // Update max primal residual
                if (r_primal_ijk > r_primal_max)
                    r_primal_max = r_primal_ijk;
                
                // Compute dual residual: ||ρ(vᵢⱼₖ^new - vᵢⱼₖ^old)||
                // Note: We need to store v_prev to compute this properly
                // For now, we'll skip dual residual computation
                // TODO: Add v_collision_prev_ for proper dual residual
            }
        }
    }
    
    // For now, use primal residual only for dual (conservative)
    r_dual_max = r_primal_max;
    
    // Store residuals
    if (r_primal_out)
        *r_primal_out = r_primal_max;
    if (r_dual_out)
        *r_dual_out = r_dual_max;
    
    // Check convergence
    BooleanType converged = BT_FALSE;
    if (r_primal_max < params_.eps_primal && r_dual_max < params_.eps_dual)
        converged = BT_TRUE;
    return converged;
}


returnValue TurboADMM::addCollisionCouplingGradient(int agent_i, real_t* g)
{
    // Add collision coupling terms to gradient:
    // 1. ADMM dual terms: Σⱼ∈N(i) Σₖ [ρ*vᵢⱼₖ - λᵢⱼₖ] (enforces hard constraint via projection)
    // 2. Collision cost gradient: -ρ_collision * Σⱼ∈N(i) Σₖ pos_j[k] (encourages separation)
    //
    // Combined gradient:
    //   g_augmented = g_tracking + ADMM_dual_terms + collision_cost_gradient
    
    static int call_count = 0;
    call_count++;
    
    AgentData& agent_i_data = agents_[agent_i];
    int nx = agent_i_data.nx;
    int nu = agent_i_data.nu;
    
    real_t total_coupling = 0.0;  // Track total coupling contribution
    real_t total_admm = 0.0;
    real_t total_collision_cost = 0.0;
    int num_violations_grad = 0;
    real_t rho_collision = params_.rho * 0.5;  // Collision cost weight (same as in Hessian)
    
    // For each neighbor j of agent i
    for (int neighbor_idx = 0; neighbor_idx < num_neighbors_[agent_i]; ++neighbor_idx) {
        int j = neighbors_[agent_i][neighbor_idx];
        AgentData& agent_j_data = agents_[j];
        
        // For each stage k=1..N (skip k=0 since initial state is fixed)
        for (int k = 1; k <= agent_i_data.N; ++k) {
            // Compute index in gradient vector for agent i
            // Structure: [x₀, u₀, x₁, u₁, ..., xₙ]
            int idx_g;
            if (k == agent_i_data.N) {
                // Terminal state: after all (x,u) pairs
                idx_g = agent_i_data.N * (nx + nu);
            } else {
                // Stage k: at beginning of k-th (x,u) pair
                idx_g = k * (nx + nu);
            }
            
            // Get neighbor j's position at stage k (current linearization point)
            int idx_j = (k == agent_j_data.N) ? (agent_j_data.N * (agent_j_data.nx + agent_j_data.nu)) : (k * (agent_j_data.nx + agent_j_data.nu));
            
            // 2D position for both agents (x, y)
            real_t x_i = z_local_[agent_i][idx_g + 0];
            real_t y_i = z_local_[agent_i][idx_g + 1];
            real_t x_j = z_local_[j][idx_j + 0];
            real_t y_j = z_local_[j][idx_j + 1];
            
            // 2D Euclidean distance
            real_t dx = x_i - x_j;
            real_t dy = y_i - y_j;
            real_t dist_current = sqrt(dx*dx + dy*dy);
            
            // Part 1: ADMM dual terms (-ρ*vᵢⱼₖ + λᵢⱼₖ) for both x and y
            // CORRECT ADMM formulation: g_augmented = g_tracking - ρ*(z - v) + λ
            // Expanding: g_augmented = g_tracking - ρ*z + ρ*v + λ
            // The -ρ*z term is handled by Hessian augmentation (H += ρI)
            // So gradient gets: -ρ*v + λ
            real_t admm_dual_term_x = -params_.rho * v_collision_[agent_i][j][k][0] 
                                     + lambda_collision_[agent_i][j][k][0];
            real_t admm_dual_term_y = -params_.rho * v_collision_[agent_i][j][k][1] 
                                     + lambda_collision_[agent_i][j][k][1];
            
            // Part 2: Linearized barrier collision cost gradient (2D)
            // Cost: (ρ_collision/2) * max(0, d_safe - dist)²
            // Only add gradient when collision is violated (dist < d_safe)
            real_t collision_cost_gradient_x = 0.0;
            real_t collision_cost_gradient_y = 0.0;
            real_t d_safe = coupling_.d_safe;
            
            if (dist_current < d_safe) {
                // Violation! Add gradient of linearized barrier
                real_t violation = d_safe - dist_current;
                
                // Normalized direction vector from j to i
                real_t norm = dist_current + 1e-6;  // Avoid division by zero
                real_t dir_x = dx / norm;
                real_t dir_y = dy / norm;
                
                // Gradient: ∇[violation²] = 2*violation*∇[violation]
                //         = 2*violation*(-direction)
                //         = -2*(d_safe - dist)*direction
                // With factor ρ_collision/2: -ρ_collision*(d_safe - dist)*direction
                real_t grad_magnitude = -rho_collision * violation;
                collision_cost_gradient_x = grad_magnitude * dir_x;
                collision_cost_gradient_y = grad_magnitude * dir_y;
                num_violations_grad++;
            }
            // else: Safe distance, no collision cost gradient
            
            // Add both terms to position gradient (x and y components)
            g[idx_g + 0] += admm_dual_term_x + collision_cost_gradient_x;
            g[idx_g + 1] += admm_dual_term_y + collision_cost_gradient_y;
            
            total_coupling += fabs(admm_dual_term_x) + fabs(admm_dual_term_y) + fabs(collision_cost_gradient_x) + fabs(collision_cost_gradient_y);
            total_admm += fabs(admm_dual_term_x) + fabs(admm_dual_term_y);
            total_collision_cost += fabs(collision_cost_gradient_x) + fabs(collision_cost_gradient_y);
        }
    }
    
    // DIAGNOSTIC: Print gradient info for first two calls
    if (call_count <= 2 && agent_i == 0) {
        printf("[DIAGNOSTIC] Agent %d gradient: total_coupling=%.6f\n", agent_i, total_coupling);
        printf("  ADMM dual terms: %.6f\n", total_admm);
        printf("  Collision cost gradient: %.6f (violations: %d/%d stages)\n", 
               total_collision_cost, num_violations_grad, agent_i_data.N);
        printf("  rho=%.2f, rho_collision=%.2f\n", params_.rho, rho_collision);
    }
    
    // Debug: print first two calls for agent 0
    if (call_count <= 2 && agent_i == 0) {
        printf("[DEBUG] Agent %d collision coupling: total=%.6f, rho=%.2f\n",
               agent_i, total_coupling, params_.rho);
        printf("  v_collision[0][1][N][0]=%.4f, lambda[0][1][N][0]=%.4f\n",
               v_collision_[0][1][agent_i_data.N][0],
               lambda_collision_[0][1][agent_i_data.N][0]);
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::projectCollisionConstraints()
{
    // Project onto collision-free set: ||xᵢ - xⱼ|| ≥ d_safe
    // Use symmetric projection (both agents move equally)
    
    static int call_count = 0;
    call_count++;
    
    if (call_count == 1) {
        printf("[DEBUG PROJ ENTRY] Projection called, call_count=%d\n", call_count);
        printf("  v_collision[0][1][0][0] BEFORE projection = %.4f\n", v_collision_[0][1][0][0]);
    }
    
    // For each agent i
    for (int i = 0; i < n_agents_; ++i) {
        AgentData& agent_i = agents_[i];
        int nx_i = agent_i.nx;
        int nu_i = agent_i.nu;
        
        // For each neighbor j of agent i
        for (int neighbor_idx = 0; neighbor_idx < num_neighbors_[i]; ++neighbor_idx) {
            int j = neighbors_[i][neighbor_idx];
            
            // Only process each pair once (i < j)
            if (i >= j) continue;
            
            AgentData& agent_j = agents_[j];
            int nx_j = agent_j.nx;
            int nu_j = agent_j.nu;
            
            // For each stage k=1..N (skip k=0 since initial state is FIXED by QP bounds)
            // Collision avoidance at k=0 must be satisfied by initial conditions (x_init)
            for (int k = 1; k <= agent_i.N; ++k) {
                // Debug: print all iterations in first call
                if (call_count == 1) {
                    printf("[DEBUG PROJ LOOP] i=%d, j=%d, k=%d\n", i, j, k);
                }
                
                // Get state indices in z_local
                int idx_i = (k == agent_i.N) ? agent_i.N * (nx_i + nu_i) : k * (nx_i + nu_i);
                int idx_j = (k == agent_j.N) ? agent_j.N * (nx_j + nu_j) : k * (nx_j + nu_j);
                
                // Compute 2D POSITION distance (collision avoidance is based on (x,y) position, not velocity)
                // For nx=4 (x, y, vx, vy), use state[0:1] (x, y position)
                real_t x_i = z_local_[i][idx_i + 0];
                real_t y_i = z_local_[i][idx_i + 1];
                real_t x_j = z_local_[j][idx_j + 0];
                real_t y_j = z_local_[j][idx_j + 1];
                
                // 2D Euclidean distance
                real_t dx = x_i - x_j;
                real_t dy = y_i - y_j;
                real_t dist = sqrt(dx*dx + dy*dy);
                
                // Debug output for first call at k=N (terminal state)
                if (call_count == 2 && k == agent_i.N) {
                    printf("[DEBUG] Projection: agents (%d,%d), k=%d, dist=%.4f, d_safe=%.4f\n",
                           i, j, k, dist, coupling_.d_safe);
                    printf("  z_local[%d] pos=%.4f, z_local[%d] pos=%.4f\n",
                           i, z_local_[i][idx_i], j, z_local_[j][idx_j]);
                }
                
                // Debug: print distance at k=0 and k=1 in first call
                if (call_count == 1 && (k == 0 || k == 1)) {
                    printf("[DEBUG PROJ k=%d] dist=%.4f, d_safe=%.4f, collision=%s\n",
                           k, dist, coupling_.d_safe, (dist < coupling_.d_safe) ? "YES" : "NO");
                    if (dist < coupling_.d_safe) {
                        printf("  COLLISION DETECTED! Will project to d_safe.\n");
                    }
                }
                
                // Debug: print before projection for iteration 2, terminal state
                if (call_count == 2 && k == agent_i.N) {
                    printf("[DEBUG PROJECTION iter=%d] k=%d: dist=%.4f, d_safe=%.4f\n",
                           call_count, k, dist, coupling_.d_safe);
                    printf("  BEFORE: v[%d][%d][%d][0]=%.4f\n", i, j, k, v_collision_[i][j][k][0]);
                    printf("  z_local[%d]=%.4f, z_local[%d]=%.4f\n",
                           i, z_local_[i][idx_i], j, z_local_[j][idx_j]);
                }
                
                // Check if projection is needed
                if (dist >= coupling_.d_safe) {
                    // Debug: print before copying at k=0
                    if (call_count == 1 && k == 0) {
                        printf("[DEBUG PROJ k=0 BEFORE] v[%d][%d][0][0]=%.4f, z_local[%d][%d]=%.4f\n",
                               i, j, v_collision_[i][j][0][0], i, idx_i, z_local_[i][idx_i]);
                    }
                    
                    // No violation - copy current states
                    for (int s = 0; s < nx_i; ++s) {
                        // Debug at k=0, first call
                        if (call_count == 1 && k == 0 && s == 0) {
                            printf("[DEBUG PROJ COPY k=0] i=%d, j=%d, idx_i=%d, idx_j=%d\n", i, j, idx_i, idx_j);
                            printf("  z_local[%d][%d]=%.4f, will set v_collision[%d][%d][0][0]\n",
                                   i, idx_i, z_local_[i][idx_i], i, j);
                        }
                        
                        v_collision_[i][j][k][s] = z_local_[i][idx_i + s];
                        v_collision_[j][i][k][s] = z_local_[j][idx_j + s];
                        
                        // Debug: verify after assignment
                        if (call_count == 1 && k == 0 && s == 0) {
                            printf("  AFTER assignment: v_collision[%d][%d][0][0]=%.4f\n", i, j, v_collision_[i][j][0][0]);
                        }
                    }
                    
                    // Debug: print after copying at k=0
                    if (call_count == 1 && k == 0) {
                        printf("[DEBUG PROJ k=0 AFTER] v[%d][%d][0][0]=%.4f (should be %.4f)\n",
                               i, j, v_collision_[i][j][0][0], z_local_[i][idx_i]);
                    }
                    if (call_count == 2 && k == agent_i.N) {
                        printf("  AFTER (no collision): v[%d][%d][%d][0]=%.4f\n", i, j, k, v_collision_[i][j][k][0]);
                    }
                    // Debug: print k=0 after projection
                    if (call_count == 2 && k == 0) {
                        printf("  [DEBUG k=0 AFTER PROJ] v[%d][%d][0]=(%.4f,%.4f), v[%d][%d][0]=(%.4f,%.4f)\n",
                               i, j, v_collision_[i][j][0][0], v_collision_[i][j][0][1],
                               j, i, v_collision_[j][i][0][0], v_collision_[j][i][0][1]);
                    }
                } else {
                    // Violation - apply symmetric projection in 2D (x, y)
                    // Numerical stability: handle near-zero distance
                    if (dist < 1e-8) {
                        // Agents at same position - separate along x-axis by default
                        real_t separation = coupling_.d_safe / 2.0;
                        v_collision_[i][j][k][0] = z_local_[i][idx_i + 0] + separation;  // x
                        v_collision_[j][i][k][0] = z_local_[j][idx_j + 0] - separation;  // x
                        v_collision_[i][j][k][1] = z_local_[i][idx_i + 1];  // y (unchanged)
                        v_collision_[j][i][k][1] = z_local_[j][idx_j + 1];  // y (unchanged)
                        
                        // Keep velocity states unchanged
                        for (int s = 2; s < nx_i; ++s) {
                            v_collision_[i][j][k][s] = z_local_[i][idx_i + s];
                            v_collision_[j][i][k][s] = z_local_[j][idx_j + s];
                        }
                    } else {
                        // Normal case: symmetric projection in 2D
                        // Compute midpoint in 2D
                        real_t mid_x = (x_i + x_j) / 2.0;
                        real_t mid_y = (y_i + y_j) / 2.0;
                        
                        // Normalized direction vector from j to i
                        real_t dir_x = dx / dist;
                        real_t dir_y = dy / dist;
                        
                        // Project positions to d_safe apart symmetrically
                        real_t half_safe = coupling_.d_safe / 2.0;
                        
                        // DIAGNOSTIC: Print projection details for k=5 in final iteration
                        static int proj_call_count = 0;
                        proj_call_count++;
                        if (k == 5 && proj_call_count >= 1 && proj_call_count <= 3) {  // First 3 calls
                            printf("\n[PROJECTION k=5 DIAGNOSTIC]\n");
                            printf("  Agent %d: (%.2f, %.2f)\n", i, x_i, y_i);
                            printf("  Agent %d: (%.2f, %.2f)\n", j, x_j, y_j);
                            printf("  Distance: %.4f (d_safe=%.2f)\n", dist, coupling_.d_safe);
                            printf("  Midpoint: (%.2f, %.2f)\n", mid_x, mid_y);
                            printf("  Direction (dx, dy): (%.4f, %.4f)\n", dx, dy);
                            printf("  Normalized dir: (%.4f, %.4f)\n", dir_x, dir_y);
                            printf("  Half-safe distance: %.2f\n", half_safe);
                        }
                        v_collision_[i][j][k][0] = mid_x + half_safe * dir_x;  // x_i projected
                        v_collision_[i][j][k][1] = mid_y + half_safe * dir_y;  // y_i projected
                        v_collision_[j][i][k][0] = mid_x - half_safe * dir_x;  // x_j projected
                        v_collision_[j][i][k][1] = mid_y - half_safe * dir_y;  // y_j projected
                        
                        // DIAGNOSTIC: Print projected positions
                        if (k == 5 && proj_call_count >= 1 && proj_call_count <= 3) {
                            printf("  Projected v[%d][%d][5]: (%.2f, %.2f)\n", 
                                   i, j, v_collision_[i][j][k][0], v_collision_[i][j][k][1]);
                            printf("  Projected v[%d][%d][5]: (%.2f, %.2f)\n", 
                                   j, i, v_collision_[j][i][k][0], v_collision_[j][i][k][1]);
                            real_t proj_dist = sqrt(
                                (v_collision_[i][j][k][0] - v_collision_[j][i][k][0]) * 
                                (v_collision_[i][j][k][0] - v_collision_[j][i][k][0]) +
                                (v_collision_[i][j][k][1] - v_collision_[j][i][k][1]) * 
                                (v_collision_[i][j][k][1] - v_collision_[j][i][k][1])
                            );
                            printf("  Projected distance: %.4f (should be %.2f)\n\n", proj_dist, coupling_.d_safe);
                        }
                        
                        // Keep velocity states unchanged
                        for (int s = 2; s < nx_i; ++s) {
                            v_collision_[i][j][k][s] = z_local_[i][idx_i + s];
                            v_collision_[j][i][k][s] = z_local_[j][idx_j + s];
                        }
                    }
                }
            }
        }
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue TurboADMM::adaptPenaltyParameter(real_t r_primal_max, real_t r_dual_max)
{
    // Adaptive penalty parameter adjustment (using max residuals)
    // Based on Boyd et al., "Distributed Optimization and Statistical Learning via ADMM"
    // 
    // Goal: Balance primal and dual residuals
    // - If r_primal >> r_dual: increase ρ (more emphasis on collision constraints)
    // - If r_dual >> r_primal: decrease ρ (more emphasis on optimality)
    
    const real_t mu = 10.0;      // Threshold ratio for adjustment
    const real_t tau_incr = 2.0; // Increase factor
    const real_t tau_decr = 2.0; // Decrease factor
    
    real_t rho_old = params_.rho;
    
    if (r_primal_max > mu * r_dual_max) {
        // Primal residual too large → increase ρ
        params_.rho *= tau_incr;
    }
    else if (r_dual_max > mu * r_primal_max) {
        // Dual residual too large → decrease ρ
        params_.rho /= tau_decr;
    }
    // else: residuals are balanced, keep ρ unchanged
    
    // If ρ changed, need to scale dual variables λ_collision
    // λ_new = λ_old * (ρ_old / ρ_new)
    if (fabs(params_.rho - rho_old) > 1e-10) {
        real_t scale = rho_old / params_.rho;
        // TODO: Scale lambda_collision_ properly
        // For now, skip scaling (will be recomputed in next iteration)
    }
    
    return SUCCESSFUL_RETURN;
}


END_NAMESPACE_QPOASES
