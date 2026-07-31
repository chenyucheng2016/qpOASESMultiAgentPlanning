#include <qpOASES/NonlinearTurboADMM.hpp>
#include <qpOASES/Constants.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/SQProblem.hpp>
#include <qpOASES/StageVaryingRiccati.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

BEGIN_NAMESPACE_QPOASES

namespace
{

struct AgentQp
{
    int_t N, nx, nu, np, nV, nC;
    std::vector<real_t> Hbase, gbase, H, g, A, lb, ub, lbA, ubA;
    std::vector<real_t> dynamicsA, dynamicsB, dynamicsC;
    std::vector<real_t> positionC, positionD, solution;
};

struct PairData
{
    int_t first, second;
    std::vector<real_t> normal, firstPosition, secondPosition;
    std::vector<real_t> firstAuxiliary, secondAuxiliary;
    std::vector<real_t> firstDual, secondDual;
    std::vector<real_t> previousFirstAuxiliary, previousSecondAuxiliary;
};

struct SolverPool
{
    std::vector<SQProblem*> solvers;
    ~SolverPool()
    {
        for (std::size_t i = 0; i < solvers.size(); ++i) delete solvers[i];
    }
};

struct CentralContext
{
    SQProblem* solver;
    std::vector<real_t> H, g, A, lb, ub, lbA, ubA;
    CentralContext() : solver(0) {}
    ~CentralContext() { delete solver; }
};

int_t stateIndex(const AgentQp& qp, int_t k)
{
    return k == qp.N ? qp.N * (qp.nx + qp.nu) : k * (qp.nx + qp.nu);
}

int_t controlIndex(const AgentQp& qp, int_t k)
{
    return k * (qp.nx + qp.nu) + qp.nx;
}

real_t optionalBound(const std::vector<real_t>& values, int_t i, real_t fallback)
{
    return values.empty() ? fallback : values[i];
}

void configureSolver(SQProblem& solver)
{
    Options options;
    options.setToMPC();
    options.printLevel = PL_NONE;
    solver.setOptions(options);
}

bool validateProblems(const std::vector<NonlinearAgentProblem>& agents, std::string& error)
{
    if (agents.empty()) { error = "at least one agent is required"; return false; }
    const int_t N = agents[0].horizon;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const NonlinearAgentProblem& p = agents[a];
        if (!p.model) { error = "each agent requires a nonlinear model"; return false; }
        const int_t nx = p.model->stateDimension();
        const int_t nu = p.model->controlDimension();
        if (p.horizon <= 0 || p.horizon != N
            || p.model->positionDimension() <= 0)
        {
            error = "agents require a common positive horizon and position outputs";
            return false;
        }
        if (p.initialState.size() != static_cast<std::size_t>(nx)
            || p.stateReference.size() != static_cast<std::size_t>((N + 1) * nx)
            || p.controlReference.size() != static_cast<std::size_t>(N * nu)
            || p.stateWeights.size() != static_cast<std::size_t>(nx)
            || p.terminalWeights.size() != static_cast<std::size_t>(nx)
            || p.controlWeights.size() != static_cast<std::size_t>(nu))
        {
            error = "trajectory and weight dimensions are inconsistent";
            return false;
        }
        if ((!p.stateLowerBounds.empty() && p.stateLowerBounds.size() != static_cast<std::size_t>(nx))
            || (!p.stateUpperBounds.empty() && p.stateUpperBounds.size() != static_cast<std::size_t>(nx))
            || (!p.controlLowerBounds.empty() && p.controlLowerBounds.size() != static_cast<std::size_t>(nu))
            || (!p.controlUpperBounds.empty() && p.controlUpperBounds.size() != static_cast<std::size_t>(nu))
            || (!p.initialControls.empty() && p.initialControls.size() != static_cast<std::size_t>(N * nu)))
        {
            error = "bound or initial-control dimensions are inconsistent";
            return false;
        }
    }
    return true;
}

int_t worldPositionDimension(
    const std::vector<NonlinearAgentProblem>& agents
)
{
    int_t dimension = 0;
    for (std::size_t a = 0; a < agents.size(); ++a)
        dimension = std::max(
            dimension,
            agents[a].model->positionDimension()
        );
    return dimension;
}

void worldPosition(
    const NonlinearModel& model,
    const real_t* state,
    int_t worldDimension,
    real_t* position
)
{
    for (int_t i = 0; i < worldDimension; ++i) position[i] = 0.0;
    model.position(state, position);
}

void clampControls(const NonlinearAgentProblem& p, std::vector<real_t>& controls)
{
    const int_t nu = p.model->controlDimension();
    for (int_t k = 0; k < p.horizon; ++k)
        for (int_t j = 0; j < nu; ++j)
            controls[k * nu + j] = std::max(
                optionalBound(p.controlLowerBounds, j, -INFTY),
                std::min(optionalBound(p.controlUpperBounds, j, INFTY), controls[k * nu + j])
            );
}

void rollout(
    const NonlinearAgentProblem& p,
    const std::vector<real_t>& controls,
    std::vector<real_t>& states
)
{
    const int_t nx = p.model->stateDimension();
    const int_t nu = p.model->controlDimension();
    states.assign((p.horizon + 1) * nx, 0.0);
    std::copy(p.initialState.begin(), p.initialState.end(), states.begin());
    for (int_t k = 0; k < p.horizon; ++k)
        p.model->dynamics(&states[k * nx], &controls[k * nu], &states[(k + 1) * nx]);
}

void buildAgentQp(
    const NonlinearAgentProblem& p,
    const std::vector<real_t>& nominalStates,
    const std::vector<real_t>& nominalControls,
    real_t trustRegion,
    int_t worldDimension,
    AgentQp& qp
)
{
    qp.N = p.horizon;
    qp.nx = p.model->stateDimension();
    qp.nu = p.model->controlDimension();
    qp.np = worldDimension;
    qp.nV = (qp.N + 1) * qp.nx + qp.N * qp.nu;
    qp.nC = qp.N * qp.nx;
    qp.Hbase.assign(qp.nV * qp.nV, 0.0);
    qp.gbase.assign(qp.nV, 0.0);
    qp.A.assign(qp.nC * qp.nV, 0.0);
    qp.lb.assign(qp.nV, -INFTY); qp.ub.assign(qp.nV, INFTY);
    qp.lbA.assign(qp.nC, 0.0); qp.ubA.assign(qp.nC, 0.0);
    qp.dynamicsA.assign(qp.N * qp.nx * qp.nx, 0.0);
    qp.dynamicsB.assign(qp.N * qp.nx * qp.nu, 0.0);
    qp.dynamicsC.assign(qp.N * qp.nx, 0.0);
    qp.positionC.assign((qp.N + 1) * qp.np * qp.nx, 0.0);
    qp.positionD.assign((qp.N + 1) * qp.np, 0.0);
    qp.solution.assign(qp.nV, 0.0);
    for (int_t i = 0; i < qp.nV; ++i) qp.Hbase[i * qp.nV + i] = 1.0e-9;

    for (int_t k = 0; k <= qp.N; ++k)
    {
        const int_t offset = stateIndex(qp, k);
        const std::vector<real_t>& weights = k == qp.N ? p.terminalWeights : p.stateWeights;
        for (int_t i = 0; i < qp.nx; ++i)
        {
            qp.Hbase[(offset + i) * qp.nV + offset + i] += weights[i];
            qp.gbase[offset + i] -= weights[i] * p.stateReference[k * qp.nx + i];
            qp.lb[offset + i] = optionalBound(p.stateLowerBounds, i, -INFTY);
            qp.ub[offset + i] = optionalBound(p.stateUpperBounds, i, INFTY);
        }
        real_t* C = &qp.positionC[k * qp.np * qp.nx];
        real_t* d = &qp.positionD[k * qp.np];
        const int_t nativeDimension = p.model->positionDimension();
        std::vector<real_t> position(nativeDimension, 0.0);
        std::vector<real_t> nativeC(nativeDimension * qp.nx, 0.0);
        p.model->position(
            &nominalStates[k * qp.nx],
            &position[0]
        );
        p.model->linearizePosition(
            &nominalStates[k * qp.nx],
            &nativeC[0]
        );
        for (int_t i = 0; i < nativeDimension; ++i)
        {
            d[i] = position[i];
            for (int_t j = 0; j < qp.nx; ++j)
            {
                C[i * qp.nx + j] = nativeC[i * qp.nx + j];
                d[i] -= C[i * qp.nx + j]
                    * nominalStates[k * qp.nx + j];
            }
        }
    }
    for (int_t i = 0; i < qp.nx; ++i) qp.lb[i] = qp.ub[i] = p.initialState[i];

    for (int_t k = 0; k < qp.N; ++k)
    {
        const int_t uOffset = controlIndex(qp, k);
        for (int_t i = 0; i < qp.nu; ++i)
        {
            qp.Hbase[(uOffset + i) * qp.nV + uOffset + i] += p.controlWeights[i];
            qp.gbase[uOffset + i] -= p.controlWeights[i] * p.controlReference[k * qp.nu + i];
            real_t lower = optionalBound(p.controlLowerBounds, i, -INFTY);
            real_t upper = optionalBound(p.controlUpperBounds, i, INFTY);
            if (trustRegion > 0.0)
            {
                lower = std::max(lower, nominalControls[k * qp.nu + i] - trustRegion);
                upper = std::min(upper, nominalControls[k * qp.nu + i] + trustRegion);
            }
            qp.lb[uOffset + i] = lower; qp.ub[uOffset + i] = upper;
        }
        real_t* Ak = &qp.dynamicsA[k * qp.nx * qp.nx];
        real_t* Bk = &qp.dynamicsB[k * qp.nx * qp.nu];
        real_t* ck = &qp.dynamicsC[k * qp.nx];
        std::vector<real_t> next(qp.nx, 0.0);
        p.model->dynamics(&nominalStates[k * qp.nx], &nominalControls[k * qp.nu], &next[0]);
        p.model->linearizeDynamics(&nominalStates[k * qp.nx], &nominalControls[k * qp.nu], Ak, Bk);
        for (int_t i = 0; i < qp.nx; ++i)
        {
            ck[i] = next[i];
            for (int_t j = 0; j < qp.nx; ++j) ck[i] -= Ak[i * qp.nx + j] * nominalStates[k * qp.nx + j];
            for (int_t j = 0; j < qp.nu; ++j) ck[i] -= Bk[i * qp.nu + j] * nominalControls[k * qp.nu + j];
            const int_t row = k * qp.nx + i;
            const int_t xOffset = stateIndex(qp, k);
            const int_t nextOffset = stateIndex(qp, k + 1);
            for (int_t j = 0; j < qp.nx; ++j) qp.A[row * qp.nV + xOffset + j] = Ak[i * qp.nx + j];
            for (int_t j = 0; j < qp.nu; ++j) qp.A[row * qp.nV + uOffset + j] = Bk[i * qp.nu + j];
            qp.A[row * qp.nV + nextOffset + i] = -1.0;
            qp.lbA[row] = qp.ubA[row] = -ck[i];
        }
    }
    qp.H = qp.Hbase; qp.g = qp.gbase;
}

void positionFromDecision(
    const AgentQp& qp, int_t k, const std::vector<real_t>& z, real_t* position
)
{
    const real_t* C = &qp.positionC[k * qp.np * qp.nx];
    const real_t* d = &qp.positionD[k * qp.np];
    const int_t offset = stateIndex(qp, k);
    for (int_t i = 0; i < qp.np; ++i)
    {
        position[i] = d[i];
        for (int_t j = 0; j < qp.nx; ++j) position[i] += C[i * qp.nx + j] * z[offset + j];
    }
}

std::vector<PairData> buildPairs(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    int_t np,
    real_t safetyDistance)
{
    std::vector<PairData> pairs;
    const int_t N = agents[0].horizon;
    for (std::size_t first = 0; first < agents.size(); ++first)
    for (std::size_t second = first + 1; second < agents.size(); ++second)
    {
        PairData pair; pair.first = static_cast<int_t>(first); pair.second = static_cast<int_t>(second);
        const int_t count = (N + 1) * np;
        pair.normal.assign(count, 0.0);
        pair.firstPosition.assign(count, 0.0); pair.secondPosition.assign(count, 0.0);
        pair.firstAuxiliary.assign(count, 0.0); pair.secondAuxiliary.assign(count, 0.0);
        pair.firstDual.assign(count, 0.0); pair.secondDual.assign(count, 0.0);
        pair.previousFirstAuxiliary.assign(count, 0.0); pair.previousSecondAuxiliary.assign(count, 0.0);
        for (int_t k = 0; k <= N; ++k)
        {
            const int_t nxf = agents[first].model->stateDimension();
            const int_t nxs = agents[second].model->stateDimension();
            real_t* pf = &pair.firstPosition[k * np]; real_t* ps = &pair.secondPosition[k * np];
            worldPosition(
                *agents[first].model,
                &states[first][k * nxf],
                np,
                pf
            );
            worldPosition(
                *agents[second].model,
                &states[second][k * nxs],
                np,
                ps
            );
            real_t norm = 0.0;
            for (int_t i = 0; i < np; ++i)
            {
                pair.normal[k * np + i] = pf[i] - ps[i];
                norm += pair.normal[k * np + i] * pair.normal[k * np + i];
            }
            norm = std::sqrt(norm);
            if (norm < 1.0e-9)
            {
                if (k > 0) for (int_t i = 0; i < np; ++i)
                    pair.normal[k * np + i] = pair.normal[(k - 1) * np + i];
                else pair.normal[0] = 1.0;
            }
            else for (int_t i = 0; i < np; ++i) pair.normal[k * np + i] /= norm;
            real_t signedDistance = 0.0;
            for (int_t i = 0; i < np; ++i)
                signedDistance += pair.normal[k * np + i] * (pf[i] - ps[i]);
            const real_t correction = 0.5 * std::max(0.0, safetyDistance - signedDistance);
            for (int_t i = 0; i < np; ++i)
            {
                pair.firstAuxiliary[k * np + i] = pf[i] + correction * pair.normal[k * np + i];
                pair.secondAuxiliary[k * np + i] = ps[i] - correction * pair.normal[k * np + i];
            }
        }
        pairs.push_back(pair);
    }
    return pairs;
}

void addAdmmTerms(int_t agentIndex, const std::vector<PairData>& pairs, real_t rho, AgentQp& qp)
{
    for (std::size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex)
    {
        const PairData& pair = pairs[pairIndex];
        const bool isFirst = pair.first == agentIndex;
        if (!isFirst && pair.second != agentIndex) continue;
        const std::vector<real_t>& v = isFirst ? pair.firstAuxiliary : pair.secondAuxiliary;
        const std::vector<real_t>& dual = isFirst ? pair.firstDual : pair.secondDual;
        for (int_t k = 0; k <= qp.N; ++k)
        {
            const int_t offset = stateIndex(qp, k);
            const real_t* C = &qp.positionC[k * qp.np * qp.nx];
            const real_t* d = &qp.positionD[k * qp.np];
            for (int_t i = 0; i < qp.nx; ++i)
            {
                real_t gradient = 0.0;
                for (int_t a = 0; a < qp.np; ++a)
                    gradient += C[a * qp.nx + i] * (d[a] - v[k * qp.np + a] + dual[k * qp.np + a]);
                qp.g[offset + i] += rho * gradient;
                for (int_t j = 0; j < qp.nx; ++j)
                {
                    real_t hessian = 0.0;
                    for (int_t a = 0; a < qp.np; ++a) hessian += C[a * qp.nx + i] * C[a * qp.nx + j];
                    qp.H[(offset + i) * qp.nV + offset + j] += rho * hessian;
                }
            }
        }
    }
}

std::vector<real_t> riccatiWarmStart(const NonlinearAgentProblem& p, const AgentQp& qp)
{
    StageVaryingLqrProblem lqr; StageVaryingLqrSolution solution;
    lqr.horizon = qp.N; lqr.nx = qp.nx; lqr.nu = qp.nu; lqr.x0 = p.initialState;
    lqr.A = qp.dynamicsA; lqr.B = qp.dynamicsB; lqr.c = qp.dynamicsC;
    lqr.Q.assign(qp.N * qp.nx * qp.nx, 0.0); lqr.R.assign(qp.N * qp.nu * qp.nu, 0.0);
    lqr.q.assign(qp.N * qp.nx, 0.0); lqr.r.assign(qp.N * qp.nu, 0.0);
    lqr.Qterminal.assign(qp.nx * qp.nx, 0.0); lqr.qterminal.assign(qp.nx, 0.0);
    for (int_t k = 0; k < qp.N; ++k)
    {
        for (int_t i = 0; i < qp.nx; ++i)
        {
            lqr.Q[k * qp.nx * qp.nx + i * qp.nx + i] = p.stateWeights[i];
            lqr.q[k * qp.nx + i] = -p.stateWeights[i] * p.stateReference[k * qp.nx + i];
        }
        for (int_t i = 0; i < qp.nu; ++i)
        {
            lqr.R[k * qp.nu * qp.nu + i * qp.nu + i] = p.controlWeights[i] + 1.0e-9;
            lqr.r[k * qp.nu + i] = -p.controlWeights[i] * p.controlReference[k * qp.nu + i];
        }
    }
    for (int_t i = 0; i < qp.nx; ++i)
    {
        lqr.Qterminal[i * qp.nx + i] = p.terminalWeights[i];
        lqr.qterminal[i] = -p.terminalWeights[i] * p.stateReference[qp.N * qp.nx + i];
    }
    std::vector<real_t> warm(qp.nV, 0.0);
    if (!solveStageVaryingLqr(lqr, solution)) return warm;
    for (int_t k = 0; k <= qp.N; ++k)
    {
        std::copy(solution.states.begin() + k * qp.nx, solution.states.begin() + (k + 1) * qp.nx,
                  warm.begin() + stateIndex(qp, k));
        if (k < qp.N) std::copy(solution.controls.begin() + k * qp.nu,
            solution.controls.begin() + (k + 1) * qp.nu, warm.begin() + controlIndex(qp, k));
    }
    return warm;
}

bool solveLocalQp(const NonlinearAgentProblem& p, int_t outer, int_t admm,
    const NonlinearTurboOptions& options, SQProblem*& solver, AgentQp& qp,
    NonlinearTurboStatistics& stats)
{
    int_t nWSR = options.maxWorkingSetRecalculations;
    returnValue status; ++stats.qpSolves;
    if (!solver)
    {
        solver = new SQProblem(qp.nV, qp.nC); configureSolver(*solver);
        const std::vector<real_t> warm = riccatiWarmStart(p, qp);
        status = solver->init(&qp.H[0], &qp.g[0], &qp.A[0], &qp.lb[0], &qp.ub[0],
            &qp.lbA[0], &qp.ubA[0], nWSR, 0, &warm[0]);
        ++stats.coldStarts;
    }
    else if (admm == 0)
    {
        status = solver->hotstart(&qp.H[0], &qp.g[0], &qp.A[0], &qp.lb[0], &qp.ub[0],
            &qp.lbA[0], &qp.ubA[0], nWSR); ++stats.matrixHotstarts;
    }
    else
    {
        status = solver->hotstart(&qp.g[0], &qp.lb[0], &qp.ub[0], &qp.lbA[0], &qp.ubA[0], nWSR);
        ++stats.vectorHotstarts;
    }
    stats.qpWorkingSetRecalculations += nWSR;
    if (status != SUCCESSFUL_RETURN && outer > 0 && admm == 0)
    {
        delete solver; solver = new SQProblem(qp.nV, qp.nC); configureSolver(*solver);
        nWSR = options.maxWorkingSetRecalculations;
        status = solver->init(&qp.H[0], &qp.g[0], &qp.A[0], &qp.lb[0], &qp.ub[0],
            &qp.lbA[0], &qp.ubA[0], nWSR); stats.qpWorkingSetRecalculations += nWSR; ++stats.coldStarts;
    }
    return status == SUCCESSFUL_RETURN && solver->getPrimalSolution(&qp.solution[0]) == SUCCESSFUL_RETURN;
}

bool solveDistributed(const std::vector<NonlinearAgentProblem>& agents, int_t outer,
    const NonlinearTurboOptions& options, SolverPool& pool, std::vector<PairData>& pairs,
    std::vector<AgentQp>& qps, NonlinearTurboStatistics& stats)
{
    if (pool.solvers.empty()) pool.solvers.assign(agents.size(), static_cast<SQProblem*>(0));
    for (int_t iteration = 0; iteration < options.maxAdmmIterations; ++iteration)
    {
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            qps[a].H = qps[a].Hbase; qps[a].g = qps[a].gbase;
            addAdmmTerms(static_cast<int_t>(a), pairs, options.rho, qps[a]);
            if (!solveLocalQp(agents[a], outer, iteration, options, pool.solvers[a], qps[a], stats)) return false;
        }
        for (std::size_t p = 0; p < pairs.size(); ++p)
        for (int_t k = 0; k <= qps[pairs[p].first].N; ++k)
        {
            positionFromDecision(qps[pairs[p].first], k, qps[pairs[p].first].solution,
                &pairs[p].firstPosition[k * qps[pairs[p].first].np]);
            positionFromDecision(qps[pairs[p].second], k, qps[pairs[p].second].solution,
                &pairs[p].secondPosition[k * qps[pairs[p].second].np]);
        }
        real_t primal = 0.0, dualResidual = 0.0;
        for (std::size_t p = 0; p < pairs.size(); ++p)
        {
            PairData& pair = pairs[p]; const int_t np = qps[pair.first].np;
            pair.previousFirstAuxiliary = pair.firstAuxiliary;
            pair.previousSecondAuxiliary = pair.secondAuxiliary;
            for (int_t k = 0; k <= qps[pair.first].N; ++k)
            {
                real_t signedDistance = 0.0;
                for (int_t i = 0; i < np; ++i)
                {
                    const int_t x = k * np + i;
                    signedDistance += pair.normal[x]
                        * ((pair.firstPosition[x] + pair.firstDual[x]) - (pair.secondPosition[x] + pair.secondDual[x]));
                }
                const real_t correction = 0.5 * std::max(0.0, options.safetyDistance - signedDistance);
                real_t p1 = 0.0, p2 = 0.0, d1 = 0.0, d2 = 0.0;
                for (int_t i = 0; i < np; ++i)
                {
                    const int_t x = k * np + i;
                    pair.firstAuxiliary[x] = pair.firstPosition[x] + pair.firstDual[x] + correction * pair.normal[x];
                    pair.secondAuxiliary[x] = pair.secondPosition[x] + pair.secondDual[x] - correction * pair.normal[x];
                    const real_t e1 = pair.firstPosition[x] - pair.firstAuxiliary[x];
                    const real_t e2 = pair.secondPosition[x] - pair.secondAuxiliary[x];
                    pair.firstDual[x] += e1; pair.secondDual[x] += e2; p1 += e1 * e1; p2 += e2 * e2;
                    const real_t c1 = pair.firstAuxiliary[x] - pair.previousFirstAuxiliary[x];
                    const real_t c2 = pair.secondAuxiliary[x] - pair.previousSecondAuxiliary[x];
                    d1 += c1 * c1; d2 += c2 * c2;
                }
                primal = std::max(primal, std::max(std::sqrt(p1), std::sqrt(p2)));
                dualResidual = std::max(dualResidual, options.rho * std::max(std::sqrt(d1), std::sqrt(d2)));
            }
        }
        stats.primalResidual = primal; stats.dualResidual = dualResidual; ++stats.admmIterations;
        if (primal <= options.admmPrimalTolerance && dualResidual <= options.admmDualTolerance) break;
    }
    return true;
}

bool solveCentralized(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<PairData>& pairs, int_t outer, const NonlinearTurboOptions& options,
    CentralContext& context, std::vector<AgentQp>& qps, NonlinearTurboStatistics& stats)
{
    std::vector<int_t> offsets(qps.size(), 0);
    int_t nV = 0, dynamicsRows = 0;
    for (std::size_t a = 0; a < qps.size(); ++a)
    {
        offsets[a] = nV; nV += qps[a].nV; dynamicsRows += qps[a].nC;
    }
    const int_t nC = dynamicsRows + static_cast<int_t>(pairs.size()) * qps[0].N;
    std::vector<real_t> H(nV * nV, 0.0), g(nV, 0.0), A(nC * nV, 0.0);
    std::vector<real_t> lb(nV, -INFTY), ub(nV, INFTY), lbA(nC, -INFTY), ubA(nC, INFTY);
    std::vector<real_t> warm(nV, 0.0);
    int_t rowOffset = 0;
    for (std::size_t a = 0; a < qps.size(); ++a)
    {
        const AgentQp& qp = qps[a]; const int_t offset = offsets[a];
        for (int_t i = 0; i < qp.nV; ++i)
        {
            g[offset + i] = qp.gbase[i]; lb[offset + i] = qp.lb[i]; ub[offset + i] = qp.ub[i];
            for (int_t j = 0; j < qp.nV; ++j)
                H[(offset + i) * nV + offset + j] = qp.Hbase[i * qp.nV + j];
        }
        for (int_t i = 0; i < qp.nC; ++i)
        {
            lbA[rowOffset + i] = qp.lbA[i]; ubA[rowOffset + i] = qp.ubA[i];
            for (int_t j = 0; j < qp.nV; ++j)
                A[(rowOffset + i) * nV + offset + j] = qp.A[i * qp.nV + j];
        }
        const std::vector<real_t> localWarm = riccatiWarmStart(agents[a], qp);
        std::copy(localWarm.begin(), localWarm.end(), warm.begin() + offset);
        rowOffset += qp.nC;
    }
    for (std::size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex)
    {
        const PairData& pair = pairs[pairIndex];
        const AgentQp& first = qps[pair.first]; const AgentQp& second = qps[pair.second];
        for (int_t k = 1; k <= first.N; ++k)
        {
            const real_t* normal = &pair.normal[k * first.np];
            const real_t* firstC = &first.positionC[k * first.np * first.nx];
            const real_t* secondC = &second.positionC[k * second.np * second.nx];
            const real_t* firstD = &first.positionD[k * first.np];
            const real_t* secondD = &second.positionD[k * second.np];
            const int_t firstState = offsets[pair.first] + stateIndex(first, k);
            const int_t secondState = offsets[pair.second] + stateIndex(second, k);
            for (int_t j = 0; j < first.nx; ++j)
                for (int_t i = 0; i < first.np; ++i)
                    A[rowOffset * nV + firstState + j] += normal[i] * firstC[i * first.nx + j];
            for (int_t j = 0; j < second.nx; ++j)
                for (int_t i = 0; i < second.np; ++i)
                    A[rowOffset * nV + secondState + j] -= normal[i] * secondC[i * second.nx + j];
            real_t affine = 0.0;
            for (int_t i = 0; i < first.np; ++i) affine += normal[i] * (firstD[i] - secondD[i]);
            lbA[rowOffset] = options.safetyDistance - affine; ubA[rowOffset] = INFTY; ++rowOffset;
        }
    }

    int_t nWSR = options.maxWorkingSetRecalculations; returnValue status; ++stats.qpSolves;
    if (!context.solver)
    {
        context.solver = new SQProblem(nV, nC); configureSolver(*context.solver);
        status = context.solver->init(&H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0],
                                      nWSR, 0, &warm[0]); ++stats.coldStarts;
    }
    else
    {
        status = context.solver->hotstart(&H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0], nWSR);
        ++stats.matrixHotstarts;
    }
    stats.qpWorkingSetRecalculations += nWSR;
    if (status != SUCCESSFUL_RETURN && outer > 0)
    {
        delete context.solver; context.solver = new SQProblem(nV, nC); configureSolver(*context.solver);
        nWSR = options.maxWorkingSetRecalculations;
        status = context.solver->init(&H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0], nWSR);
        stats.qpWorkingSetRecalculations += nWSR; ++stats.coldStarts;
    }
    if (status != SUCCESSFUL_RETURN) return false;
    std::vector<real_t> decision(nV, 0.0);
    if (context.solver->getPrimalSolution(&decision[0]) != SUCCESSFUL_RETURN) return false;
    for (std::size_t a = 0; a < qps.size(); ++a)
        std::copy(decision.begin() + offsets[a], decision.begin() + offsets[a] + qps[a].nV,
                  qps[a].solution.begin());
    context.H.swap(H); context.g.swap(g); context.A.swap(A); context.lb.swap(lb);
    context.ub.swap(ub); context.lbA.swap(lbA); context.ubA.swap(ubA);
    return true;
}

real_t trajectoryCost(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls)
{
    real_t cost = 0.0;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t nu = agents[a].model->controlDimension();
        for (int_t k = 0; k <= agents[a].horizon; ++k)
        {
            for (int_t i = 0; i < nx; ++i)
            {
                const real_t error = states[a][k * nx + i] - agents[a].stateReference[k * nx + i];
                const real_t weight = k == agents[a].horizon ? agents[a].terminalWeights[i] : agents[a].stateWeights[i];
                cost += 0.5 * weight * error * error;
            }
            if (k < agents[a].horizon) for (int_t i = 0; i < nu; ++i)
            {
                const real_t error = controls[a][k * nu + i] - agents[a].controlReference[k * nu + i];
                cost += 0.5 * agents[a].controlWeights[i] * error * error;
            }
        }
    }
    return cost;
}

real_t minimumDistance(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states)
{
    if (agents.size() < 2) return INFTY;
    real_t minimum = INFTY;
    const int_t np = worldPositionDimension(agents);
    for (std::size_t first = 0; first < agents.size(); ++first)
    for (std::size_t second = first + 1; second < agents.size(); ++second)
    {
        const int_t nxf = agents[first].model->stateDimension();
        const int_t nxs = agents[second].model->stateDimension();
        std::vector<real_t> pf(np, 0.0), ps(np, 0.0);
        for (int_t k = 0; k <= agents[first].horizon; ++k)
        {
            worldPosition(
                *agents[first].model,
                &states[first][k * nxf],
                np,
                &pf[0]
            );
            worldPosition(
                *agents[second].model,
                &states[second][k * nxs],
                np,
                &ps[0]
            );
            real_t squared = 0.0;
            for (int_t i = 0; i < np; ++i) squared += (pf[i] - ps[i]) * (pf[i] - ps[i]);
            minimum = std::min(minimum, std::sqrt(squared));
        }
    }
    return minimum;
}

real_t merit(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls,
    const NonlinearTurboOptions& options)
{
    const real_t violation = std::max(0.0, options.safetyDistance - minimumDistance(agents, states));
    return trajectoryCost(agents, states, controls) + options.meritPenalty * violation * violation;
}

real_t dynamicsDefect(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls)
{
    real_t maximum = 0.0;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t nu = agents[a].model->controlDimension();
        std::vector<real_t> next(nx, 0.0);
        for (int_t k = 0; k < agents[a].horizon; ++k)
        {
            agents[a].model->dynamics(&states[a][k * nx], &controls[a][k * nu], &next[0]);
            for (int_t i = 0; i < nx; ++i)
                maximum = std::max(maximum, std::fabs(next[i] - states[a][(k + 1) * nx + i]));
        }
    }
    return maximum;
}

} // anonymous namespace

NonlinearAgentProblem::NonlinearAgentProblem() : model(0), horizon(0) {}

NonlinearTurboOptions::NonlinearTurboOptions()
    : coordinationMethod(NCM_DISTRIBUTED_ADMM), maxScpIterations(8), maxAdmmIterations(40),
      maxWorkingSetRecalculations(300), maxLineSearchSteps(6), rho(30.0), safetyDistance(1.0),
      controlTrustRegion(1.0), admmPrimalTolerance(1.0e-3), admmDualTolerance(1.0e-3),
      scpStepTolerance(1.0e-3), collisionTolerance(1.0e-2), meritPenalty(1.0e5) {}

NonlinearTurboStatistics::NonlinearTurboStatistics()
    : scpIterations(0), admmIterations(0), qpSolves(0), qpWorkingSetRecalculations(0),
      coldStarts(0), matrixHotstarts(0), vectorHotstarts(0), primalResidual(0.0),
      dualResidual(0.0), minimumDistance(INFTY), maximumDynamicsDefect(0.0), objective(0.0),
      solveTimeMilliseconds(0.0) {}

NonlinearTurboResult::NonlinearTurboResult() : success(false), converged(false) {}

NonlinearTurboResult NonlinearTurboADMM::solve(
    const std::vector<NonlinearAgentProblem>& agents,
    const NonlinearTurboOptions& options) const
{
    NonlinearTurboResult result;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::string validationError;
    if (!validateProblems(agents, validationError)) { result.status = validationError; return result; }
    if (options.maxScpIterations <= 0 || options.maxAdmmIterations <= 0
        || options.rho <= 0.0 || options.safetyDistance < 0.0)
    {
        result.status = "invalid solver options"; return result;
    }

    std::vector<std::vector<real_t> > nominalControls(agents.size());
    std::vector<std::vector<real_t> > nominalStates(agents.size());
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        nominalControls[a] = agents[a].initialControls.empty()
            ? agents[a].controlReference : agents[a].initialControls;
        clampControls(agents[a], nominalControls[a]);
        rollout(agents[a], nominalControls[a], nominalStates[a]);
    }

    std::vector<AgentQp> activeQps;
    SolverPool distributedSolvers; CentralContext centralContext;
    bool qpFailed = false;
    const int_t positionDimension = worldPositionDimension(agents);
    for (int_t outer = 0; outer < options.maxScpIterations; ++outer)
    {
        std::vector<AgentQp> qps(agents.size());
        for (std::size_t a = 0; a < agents.size(); ++a)
            buildAgentQp(
                agents[a],
                nominalStates[a],
                nominalControls[a],
                options.controlTrustRegion,
                positionDimension,
                qps[a]
            );
        std::vector<PairData> pairs = buildPairs(
            agents,
            nominalStates,
            positionDimension,
            options.safetyDistance
        );
        const bool solved = options.coordinationMethod == NCM_DISTRIBUTED_ADMM
            ? solveDistributed(agents, outer, options, distributedSolvers, pairs, qps, result.statistics)
            : solveCentralized(agents, pairs, outer, options, centralContext, qps, result.statistics);
        if (!solved) { qpFailed = true; result.status = "a convex SCP subproblem failed"; break; }

        std::vector<std::vector<real_t> > qpControls(agents.size());
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            qpControls[a].assign(qps[a].N * qps[a].nu, 0.0);
            for (int_t k = 0; k < qps[a].N; ++k)
                std::copy(qps[a].solution.begin() + controlIndex(qps[a], k),
                    qps[a].solution.begin() + controlIndex(qps[a], k) + qps[a].nu,
                    qpControls[a].begin() + k * qps[a].nu);
        }

        real_t bestMerit = merit(agents, nominalStates, nominalControls, options);
        real_t bestAlpha = 0.0;
        std::vector<std::vector<real_t> > bestStates = nominalStates;
        std::vector<std::vector<real_t> > bestControls = nominalControls;
        real_t alpha = 1.0;
        for (int_t line = 0; line < options.maxLineSearchSteps; ++line)
        {
            std::vector<std::vector<real_t> > candidateControls(agents.size());
            std::vector<std::vector<real_t> > candidateStates(agents.size());
            for (std::size_t a = 0; a < agents.size(); ++a)
            {
                candidateControls[a].resize(nominalControls[a].size());
                for (std::size_t i = 0; i < candidateControls[a].size(); ++i)
                    candidateControls[a][i] = nominalControls[a][i]
                        + alpha * (qpControls[a][i] - nominalControls[a][i]);
                clampControls(agents[a], candidateControls[a]);
                rollout(agents[a], candidateControls[a], candidateStates[a]);
            }
            const real_t candidateMerit = merit(agents, candidateStates, candidateControls, options);
            if (candidateMerit < bestMerit - 1.0e-10)
            {
                bestMerit = candidateMerit; bestAlpha = alpha;
                bestStates.swap(candidateStates); bestControls.swap(candidateControls);
            }
            alpha *= 0.5;
        }
        real_t maximumStep = 0.0;
        for (std::size_t a = 0; a < agents.size(); ++a)
            for (std::size_t i = 0; i < nominalControls[a].size(); ++i)
                maximumStep = std::max(maximumStep, std::fabs(bestControls[a][i] - nominalControls[a][i]));
        nominalStates.swap(bestStates); nominalControls.swap(bestControls); activeQps.swap(qps);
        result.statistics.scpIterations = outer + 1;
        const real_t distance = minimumDistance(agents, nominalStates);
        if ((maximumStep <= options.scpStepTolerance || bestAlpha == 0.0)
            && distance >= options.safetyDistance - options.collisionTolerance)
        {
            result.converged = true; break;
        }
    }

    result.trajectories.resize(agents.size());
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        result.trajectories[a].states = nominalStates[a];
        result.trajectories[a].controls = nominalControls[a];
    }
    result.statistics.minimumDistance = minimumDistance(agents, nominalStates);
    result.statistics.maximumDynamicsDefect = dynamicsDefect(agents, nominalStates, nominalControls);
    result.statistics.objective = trajectoryCost(agents, nominalStates, nominalControls);
    result.success = !qpFailed && result.statistics.minimumDistance
        >= options.safetyDistance - options.collisionTolerance;
    if (result.status.empty())
        result.status = result.converged ? "converged" : (result.success
            ? "maximum SCP iterations reached with a feasible trajectory"
            : "trajectory remains collision-infeasible");
    const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
    result.statistics.solveTimeMilliseconds = static_cast<real_t>(elapsed.count());
    return result;
}

END_NAMESPACE_QPOASES
