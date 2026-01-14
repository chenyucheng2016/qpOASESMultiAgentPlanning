% =========================================================================
% LP FEASIBILITY TEST USING GLPK
% =========================================================================
% This script tests if the constraint set is feasible by solving a Linear
% Program with GLPK. We remove the quadratic objective (H) and box bounds
% (lb/ub), focusing purely on whether the equality and inequality
% constraints can be satisfied.
%
% LP Formulation:
%   minimize    sum(x)  (arbitrary objective)
%   subject to  A*x = b              (equality constraints)
%               ld <= C*x <= ud      (inequality constraints)
%               -inf <= x <= inf     (unbounded variables)
%
% If GLPK finds a solution, the constraint set is FEASIBLE.
% If GLPK returns infeasible, then constraints truly conflict.
% =========================================================================

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║         LP FEASIBILITY TEST WITH GLPK (Unbounded)         ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

fprintf('Loading problem data from hpipm_4agent_solve.m...\n');
run('hpipm_4agent_solve.m');
%run('hpipm_4agent_solve_2agent.m');
fprintf('Data loaded successfully.\n\n');

% =========================================================================
% Problem Information
% =========================================================================
fprintf('──────────────────────────────────────────────────────────────\n');
fprintf('ORIGINAL QP PROBLEM:\n');
fprintf('──────────────────────────────────────────────────────────────\n');
fprintf('  Variables:             %d\n', length(g));
fprintf('  Equality constraints:  %d (A*x = b)\n', size(A,1));
fprintf('  Inequality constraints: %d (ld <= C*x <= ud)\n', size(C,1));
fprintf('  Box constraints:       %d (lb <= x <= ub) - WILL BE REMOVED\n\n', length(lb));

fprintf('──────────────────────────────────────────────────────────────\n');
fprintf('LP FEASIBILITY TEST SETUP:\n');
fprintf('──────────────────────────────────────────────────────────────\n');
fprintf('  Objective:     minimize sum(x)  [arbitrary]\n');
fprintf('  Constraints:   A*x = b  (equality)\n');
fprintf('                 ld <= C*x <= ud  (inequality)\n');
fprintf('  Bounds:        -inf <= x <= inf  (UNBOUNDED)\n\n');

% =========================================================================
% Construct LP in GLPK Format
% =========================================================================
fprintf('Constructing LP problem for GLPK...\n');

n_vars = length(g);
n_eq = size(A, 1);
n_ineq = size(C, 1);

% Linear objective: minimize sum of all variables (arbitrary choice)
c_lp = ones(n_vars, 1);

% GLPK uses format: [Aeq; Aineq] * x ~ [beq; bineq]
% with types: 'S' (equality =), 'U' (upper <=), 'L' (lower >=), 'D' (double-bounded)

% Equality constraints: A*x = b
A_glpk = [A; C; C];  % Stack equality and inequality constraints

% For inequality: ld <= C*x <= ud
% We need to convert to GLPK format
b_glpk = [b; ud; ld];  % This will be updated below

% Constraint types and bounds
ctype_glpk = [];
lb_constraints = [];
ub_constraints = [];

% Equality constraints: A*x = b  (type 'S' for equality)
for i = 1:n_eq
    ctype_glpk = [ctype_glpk; 'S'];
end
lb_constraints = [lb_constraints; b];
ub_constraints = [ub_constraints; b];

% Inequality constraints: ld <= C*x <= ud (type 'D' for double-bounded)
for i = 1:n_ineq
    ctype_glpk = [ctype_glpk; 'U'];  % Lower bound only
end
for i = 1:n_ineq
    ctype_glpk = [ctype_glpk; 'L'];  % Lower bound only
end
lb_constraints = [lb_constraints; ld];
ub_constraints = [ub_constraints; ud];

% Variable bounds: UNBOUNDED (as requested)
lb_vars = -inf(n_vars, 1);
ub_vars = inf(n_vars, 1);

fprintf('  LP has %d variables (all unbounded)\n', n_vars);
fprintf('  LP has %d equality constraints\n', n_eq);
fprintf('  LP has %d inequality constraints\n', n_ineq);
fprintf('  Total constraints: %d\n\n', n_eq + n_ineq);

% =========================================================================
% Set up GLPK parameters
% =========================================================================
params.msglev = 3;  % 0=no output, 1=error msgs, 2=normal, 3=full
params.itlim = 10000;  % Iteration limit
params.presol = 1;  % Use presolver
params.dual = 3;
params.lpsolver = 2;
fprintf('GLPK Parameters:\n');
fprintf('  Message level: %d (1=errors only, 2=normal, 3=verbose)\n', params.msglev);
fprintf('  Iteration limit: %d\n', params.itlim);
fprintf('  Presolver: enabled\n\n');

% =========================================================================
% Solve LP with GLPK
% =========================================================================
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('CALLING GLPK SOLVER...\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

tic;
try
    [x_lp, fval, status, extra] = glpk(g, A_glpk, b_glpk, lb_vars, ub_vars, ...
                                       ctype_glpk, [], 1, params);
    solve_time = toc;
    
    fprintf('\n══════════════════════════════════════════════════════════════\n');
    fprintf('GLPK SOLVER COMPLETED (%.2f seconds)\n', solve_time);
    fprintf('══════════════════════════════════════════════════════════════\n\n');
    
    fprintf('GLPK Status Code: %d\n', status);
    fprintf('Status Interpretation:\n');
    fprintf('  1 = undefined\n');
    fprintf('  2 = feasible\n');
    fprintf('  3 = infeasible\n');
    fprintf('  4 = no feasible solution exists\n');
    fprintf('  5 = optimal solution found\n');
    fprintf('  6 = unbounded\n\n');
    
    if status == 0 || status == 2
        % Optimal or feasible solution found
        fprintf('╔════════════════════════════════════════════════════════════╗\n');
        fprintf('║                 ✓ SOLUTION FOUND                           ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
        
        fprintf('Objective value: %.6e\n', fval);
        fprintf('Solution norm: %.6e\n\n', norm(x_lp));
        
        % Verify constraint satisfaction
        fprintf('──────────────────────────────────────────────────────────────\n');
        fprintf('CONSTRAINT SATISFACTION CHECK:\n');
        fprintf('──────────────────────────────────────────────────────────────\n');
        
        % Equality constraints
        eq_residual = A * x_lp - b;
        max_eq_vio = max(abs(eq_residual));
        fprintf('Equality (||Ax - b||_∞):       %.6e\n', max_eq_vio);
        
        % Inequality constraints
        Cx = C * x_lp;
        ineq_lower_vio = max([0; ld(:) - Cx(:)]);
        ineq_upper_vio = max([0; Cx(:) - ud(:)]);
        max_ineq_vio = max(ineq_lower_vio, ineq_upper_vio);
        fprintf('Inequality violations:         %.6e\n', max_ineq_vio);
        
        fprintf('\n');
        if max_eq_vio < 1e-6 && max_ineq_vio < 1e-6
            fprintf('[✓] Solution satisfies all constraints (tol=1e-6)\n\n');
            feasibility_result = 'FEASIBLE';
        else
            fprintf('[⚠] Solution has constraint violations > 1e-6\n');
            fprintf('    This may indicate numerical issues\n\n');
            feasibility_result = 'LOOSELY_FEASIBLE';
        end
        
    elseif status == 6
        fprintf('╔════════════════════════════════════════════════════════════╗\n');
        fprintf('║              ⚠ PROBLEM IS UNBOUNDED                        ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
        fprintf('The constraint set is feasible, but the objective can go to -∞.\n');
        fprintf('This is expected since we removed all box constraints.\n\n');
        feasibility_result = 'FEASIBLE';
        
    elseif status == 3 || status == 4
        fprintf('╔════════════════════════════════════════════════════════════╗\n');
        fprintf('║              ✗ PROBLEM IS INFEASIBLE                       ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
        feasibility_result = 'INFEASIBLE';
        
    else
        fprintf('╔════════════════════════════════════════════════════════════╗\n');
        fprintf('║              ⚠ SOLVER STATUS UNCLEAR                       ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
        feasibility_result = 'UNKNOWN';
    end
    
catch ME
    solve_time = toc;
    fprintf('\n══════════════════════════════════════════════════════════════\n');
    fprintf('GLPK SOLVER ERROR (%.2f seconds)\n', solve_time);
    fprintf('══════════════════════════════════════════════════════════════\n\n');
    fprintf('Error message: %s\n\n', ME.message);
    feasibility_result = 'ERROR';
end

% =========================================================================
% Final Conclusion
% =========================================================================
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                  FINAL CONCLUSION                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

switch feasibility_result
    case 'FEASIBLE'
        fprintf('🟢 CONSTRAINT SET IS FEASIBLE\n\n');
        fprintf('The equality and inequality constraints CAN be satisfied\n');
        fprintf('when variables are unbounded.\n\n');
        fprintf('IMPLICATIONS:\n');
        fprintf('  ✓ The constraint structure itself is consistent\n');
        fprintf('  ✓ A*x = b and ld <= C*x <= ud are compatible\n');
        fprintf('  ? Box constraints (lb, ub) may cause infeasibility\n');
        fprintf('  ? Or HPIPM interface has issues\n\n');
        fprintf('NEXT STEPS:\n');
        fprintf('  1. Test with original box constraints (lb, ub)\n');
        fprintf('  2. If that fails, box constraints conflict with others\n');
        fprintf('  3. If that succeeds, HPIPM interface is the issue\n\n');
        
    case 'LOOSELY_FEASIBLE'
        fprintf('🟡 CONSTRAINT SET IS LOOSELY FEASIBLE\n\n');
        fprintf('GLPK found a solution but with numerical violations.\n');
        fprintf('The constraints may be very tight or nearly conflicting.\n\n');
        fprintf('IMPLICATIONS:\n');
        fprintf('  ~ Constraints are solvable but numerically challenging\n');
        fprintf('  ~ HPIPM may correctly reject as "infeasible"\n\n');
        
    case 'INFEASIBLE'
        fprintf('🔴 CONSTRAINT SET IS INFEASIBLE\n\n');
        fprintf('Even with unbounded variables, GLPK cannot satisfy\n');
        fprintf('the equality and inequality constraints simultaneously.\n\n');
        fprintf('IMPLICATIONS:\n');
        fprintf('  ✗ A*x = b and ld <= C*x <= ud are INCOMPATIBLE\n');
        fprintf('  ✗ HPIPM is CORRECT to return infeasible\n');
        fprintf('  ✗ OSQP must be using different tolerances/constraints\n\n');
        fprintf('DEBUGGING STEPS:\n');
        fprintf('  1. Verify A, b, C, ld, ud were exported correctly from HPIPM\n');
        fprintf('  2. Compare constraint matrices with OSQP formulation\n');
        fprintf('  3. Check if OSQP is solving a different problem\n\n');
        
    case 'UNKNOWN'
        fprintf('🟠 GLPK STATUS UNCLEAR\n\n');
        fprintf('The solver terminated but the status is ambiguous.\n');
        fprintf('Check the GLPK output messages above.\n\n');
        
    case 'ERROR'
        fprintf('🔴 GLPK ENCOUNTERED AN ERROR\n\n');
        fprintf('The LP could not be solved due to a technical issue.\n');
        fprintf('Check the error message above.\n\n');
end

fprintf('──────────────────────────────────────────────────────────────\n');
fprintf('Additional Information:\n');
fprintf('  Problem size: %d variables, %d constraints\n', n_vars, n_eq + n_ineq);
fprintf('  Solve time:   %.2f seconds\n', solve_time);
fprintf('──────────────────────────────────────────────────────────────\n\n');

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('LP feasibility test complete.\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');
