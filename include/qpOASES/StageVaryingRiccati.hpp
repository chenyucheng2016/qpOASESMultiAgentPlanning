#ifndef QPOASES_STAGE_VARYING_RICCATI_HPP
#define QPOASES_STAGE_VARYING_RICCATI_HPP

#include <qpOASES/Types.hpp>
#include <vector>

BEGIN_NAMESPACE_QPOASES

struct StageVaryingLqrProblem
{
    int_t horizon;
    int_t nx;
    int_t nu;
    std::vector<real_t> x0;
    std::vector<real_t> A;
    std::vector<real_t> B;
    std::vector<real_t> c;
    std::vector<real_t> Q;
    std::vector<real_t> R;
    /** Optional state-control cross terms, one nx-by-nu matrix per stage. */
    std::vector<real_t> stateControl;
    std::vector<real_t> q;
    std::vector<real_t> r;
    std::vector<real_t> Qterminal;
    std::vector<real_t> qterminal;

    StageVaryingLqrProblem();
    bool hasValidDimensions() const;
};

struct StageVaryingLqrSolution
{
    std::vector<real_t> states;
    std::vector<real_t> controls;
    std::vector<real_t> costates;
};

bool solveStageVaryingLqr(
    const StageVaryingLqrProblem& problem,
    StageVaryingLqrSolution& solution
);

END_NAMESPACE_QPOASES
#endif
