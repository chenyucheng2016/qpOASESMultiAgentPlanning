#include <qpOASES/StageVaryingRiccati.hpp>

#include <algorithm>
#include <cmath>

BEGIN_NAMESPACE_QPOASES

namespace
{

bool solveLinearSystem(
    const std::vector<real_t>& matrix,
    const std::vector<real_t>& rhs,
    int_t n,
    int_t columns,
    std::vector<real_t>& solution
)
{
    std::vector<real_t> augmented(n * (n + columns), 0.0);
    int_t row;
    int_t col;
    int_t pivot;
    for (row = 0; row < n; ++row)
    {
        for (col = 0; col < n; ++col)
            augmented[row * (n + columns) + col] = matrix[row * n + col];
        for (col = 0; col < columns; ++col)
            augmented[row * (n + columns) + n + col] = rhs[row * columns + col];
    }

    for (pivot = 0; pivot < n; ++pivot)
    {
        int_t bestRow = pivot;
        real_t bestValue = std::fabs(augmented[pivot * (n + columns) + pivot]);
        for (row = pivot + 1; row < n; ++row)
        {
            const real_t value = std::fabs(augmented[row * (n + columns) + pivot]);
            if (value > bestValue)
            {
                bestValue = value;
                bestRow = row;
            }
        }
        if (bestValue < 1.0e-12) return false;
        if (bestRow != pivot)
        {
            for (col = pivot; col < n + columns; ++col)
                std::swap(
                    augmented[pivot * (n + columns) + col],
                    augmented[bestRow * (n + columns) + col]
                );
        }
        const real_t diagonal = augmented[pivot * (n + columns) + pivot];
        for (col = pivot; col < n + columns; ++col)
            augmented[pivot * (n + columns) + col] /= diagonal;
        for (row = 0; row < n; ++row)
        {
            if (row == pivot) continue;
            const real_t factor = augmented[row * (n + columns) + pivot];
            for (col = pivot; col < n + columns; ++col)
                augmented[row * (n + columns) + col] -=
                    factor * augmented[pivot * (n + columns) + col];
        }
    }

    solution.assign(n * columns, 0.0);
    for (row = 0; row < n; ++row)
        for (col = 0; col < columns; ++col)
            solution[row * columns + col] =
                augmented[row * (n + columns) + n + col];
    return true;
}

}

StageVaryingLqrProblem::StageVaryingLqrProblem() : horizon(0), nx(0), nu(0) {}

bool StageVaryingLqrProblem::hasValidDimensions() const
{
    if (horizon <= 0 || nx <= 0 || nu <= 0) return false;
    return x0.size() == static_cast<std::size_t>(nx)
        && A.size() == static_cast<std::size_t>(horizon * nx * nx)
        && B.size() == static_cast<std::size_t>(horizon * nx * nu)
        && c.size() == static_cast<std::size_t>(horizon * nx)
        && Q.size() == static_cast<std::size_t>(horizon * nx * nx)
        && R.size() == static_cast<std::size_t>(horizon * nu * nu)
        && q.size() == static_cast<std::size_t>(horizon * nx)
        && r.size() == static_cast<std::size_t>(horizon * nu)
        && Qterminal.size() == static_cast<std::size_t>(nx * nx)
        && qterminal.size() == static_cast<std::size_t>(nx);
}

bool solveStageVaryingLqr(
    const StageVaryingLqrProblem& problem,
    StageVaryingLqrSolution& solution
)
{
    if (!problem.hasValidDimensions()) return false;
    const int_t N = problem.horizon;
    const int_t nx = problem.nx;
    const int_t nu = problem.nu;
    std::vector<real_t> P = problem.Qterminal;
    std::vector<real_t> p = problem.qterminal;
    std::vector<real_t> feedback(N * nu * nx, 0.0);
    std::vector<real_t> feedforward(N * nu, 0.0);
    int_t k;
    int_t i;
    int_t j;
    int_t l;

    for (k = N - 1; k >= 0; --k)
    {
        const real_t* Ak = &problem.A[k * nx * nx];
        const real_t* Bk = &problem.B[k * nx * nu];
        const real_t* ck = &problem.c[k * nx];
        const real_t* Qk = &problem.Q[k * nx * nx];
        const real_t* Rk = &problem.R[k * nu * nu];
        const real_t* qk = &problem.q[k * nx];
        const real_t* rk = &problem.r[k * nu];
        std::vector<real_t> PA(nx * nx, 0.0);
        std::vector<real_t> PB(nx * nu, 0.0);
        std::vector<real_t> pc(p);
        std::vector<real_t> S(nu * nu, 0.0);
        std::vector<real_t> G(nu * nx, 0.0);
        std::vector<real_t> h(nu, 0.0);
        std::vector<real_t> rhs(nu * (nx + 1), 0.0);
        std::vector<real_t> gain;

        for (i = 0; i < nx; ++i)
        {
            for (j = 0; j < nx; ++j)
                for (l = 0; l < nx; ++l)
                    PA[i * nx + j] += P[i * nx + l] * Ak[l * nx + j];
            for (j = 0; j < nu; ++j)
                for (l = 0; l < nx; ++l)
                    PB[i * nu + j] += P[i * nx + l] * Bk[l * nu + j];
            for (j = 0; j < nx; ++j) pc[i] += P[i * nx + j] * ck[j];
        }
        for (i = 0; i < nu; ++i)
        {
            for (j = 0; j < nu; ++j)
            {
                S[i * nu + j] = Rk[i * nu + j];
                for (l = 0; l < nx; ++l)
                    S[i * nu + j] += Bk[l * nu + i] * PB[l * nu + j];
            }
            for (j = 0; j < nx; ++j)
                for (l = 0; l < nx; ++l)
                    G[i * nx + j] += Bk[l * nu + i] * PA[l * nx + j];
            h[i] = rk[i];
            for (l = 0; l < nx; ++l) h[i] += Bk[l * nu + i] * pc[l];
            for (j = 0; j < nx; ++j) rhs[i * (nx + 1) + j] = G[i * nx + j];
            rhs[i * (nx + 1) + nx] = h[i];
        }
        if (!solveLinearSystem(S, rhs, nu, nx + 1, gain)) return false;

        real_t* Kk = &feedback[k * nu * nx];
        real_t* kk = &feedforward[k * nu];
        for (i = 0; i < nu; ++i)
        {
            for (j = 0; j < nx; ++j) Kk[i * nx + j] = -gain[i * (nx + 1) + j];
            kk[i] = -gain[i * (nx + 1) + nx];
        }
        std::vector<real_t> nextP(nx * nx, 0.0);
        std::vector<real_t> nextp(nx, 0.0);
        for (i = 0; i < nx; ++i)
        {
            for (j = 0; j < nx; ++j)
            {
                nextP[i * nx + j] = Qk[i * nx + j];
                for (l = 0; l < nx; ++l)
                    nextP[i * nx + j] += Ak[l * nx + i] * PA[l * nx + j];
                for (l = 0; l < nu; ++l)
                    nextP[i * nx + j] += G[l * nx + i] * Kk[l * nx + j];
            }
            nextp[i] = qk[i];
            for (j = 0; j < nx; ++j) nextp[i] += Ak[j * nx + i] * pc[j];
            for (j = 0; j < nu; ++j) nextp[i] += G[j * nx + i] * kk[j];
        }
        P.swap(nextP);
        p.swap(nextp);
    }

    solution.states.assign((N + 1) * nx, 0.0);
    solution.controls.assign(N * nu, 0.0);
    solution.costates.assign((N + 1) * nx, 0.0);
    std::copy(problem.x0.begin(), problem.x0.end(), solution.states.begin());
    for (k = 0; k < N; ++k)
    {
        const real_t* x = &solution.states[k * nx];
        real_t* u = &solution.controls[k * nu];
        const real_t* Kk = &feedback[k * nu * nx];
        const real_t* kk = &feedforward[k * nu];
        const real_t* Ak = &problem.A[k * nx * nx];
        const real_t* Bk = &problem.B[k * nx * nu];
        real_t* next = &solution.states[(k + 1) * nx];
        for (i = 0; i < nu; ++i)
        {
            u[i] = kk[i];
            for (j = 0; j < nx; ++j) u[i] += Kk[i * nx + j] * x[j];
        }
        for (i = 0; i < nx; ++i)
        {
            next[i] = problem.c[k * nx + i];
            for (j = 0; j < nx; ++j) next[i] += Ak[i * nx + j] * x[j];
            for (j = 0; j < nu; ++j) next[i] += Bk[i * nu + j] * u[j];
        }
    }

    real_t* lambdaN = &solution.costates[N * nx];
    const real_t* xN = &solution.states[N * nx];
    for (i = 0; i < nx; ++i)
    {
        lambdaN[i] = problem.qterminal[i];
        for (j = 0; j < nx; ++j) lambdaN[i] += problem.Qterminal[i * nx + j] * xN[j];
    }
    for (k = N - 1; k >= 0; --k)
    {
        const real_t* x = &solution.states[k * nx];
        const real_t* Ak = &problem.A[k * nx * nx];
        const real_t* nextLambda = &solution.costates[(k + 1) * nx];
        real_t* lambda = &solution.costates[k * nx];
        for (i = 0; i < nx; ++i)
        {
            lambda[i] = problem.q[k * nx + i];
            for (j = 0; j < nx; ++j)
                lambda[i] += problem.Q[k * nx * nx + i * nx + j] * x[j];
            for (j = 0; j < nx; ++j) lambda[i] += Ak[j * nx + i] * nextLambda[j];
        }
    }
    return true;
}

END_NAMESPACE_QPOASES
