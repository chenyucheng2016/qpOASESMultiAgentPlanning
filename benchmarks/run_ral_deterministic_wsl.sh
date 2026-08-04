#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${BUILD_DIR:-"${repo_root}/build-ral-wsl"}
mode=${1:-pilot}

case "${mode}" in
    pilot)
        repetitions=1
        protocol_id=ral-deterministic-v1-pilot
        ;;
    final)
        repetitions=10
        protocol_id=ral-deterministic-v1
        ;;
    *)
        echo "usage: $0 [pilot|final]" >&2
        exit 2
        ;;
esac

git_commit=${TURBOADMM_GIT_COMMIT:-unknown}
if [[ "${git_commit}" == "unknown" ]]; then
    git_commit=$(git -C "${repo_root}" rev-parse HEAD 2>/dev/null || true)
fi
if [[ "${mode}" == "final" && -z "${git_commit}" ]]; then
    echo "set TURBOADMM_GIT_COMMIT before a final run" >&2
    exit 2
fi
git_commit=${git_commit:-unknown}

output_dir=${OUTPUT_DIR:-"${build_dir}/results/${protocol_id}"}
cases=easy_open,easy_single_blocker,medium_doorway,hard_heterogeneous_doorway,hard_warehouse,very_hard_maze
methods=full,centralized_osqp,centralized_qpoases,qp_continuation,inner,cold

cmake -S "${repo_root}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DQPOASES_BUILD_EXAMPLES=OFF \
    -DQPOASES_BUILD_NONLINEAR_TESTS=ON \
    -DQPOASES_BUILD_CSDO_COMPARISON=ON \
    -DQPOASES_BUILD_OSQP_BACKEND=ON \
    -DOSQP_ROOT=/usr/local
cmake --build "${build_dir}" -j "$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure)

runner_status=0
python3 "${repo_root}/benchmarks/run_nonlinear_matrix.py" \
    "${build_dir}/bin/nonlinear_benchmark" \
    --suite manual \
    --manual-cases "${cases}" \
    --methods "${methods}" \
    --protocol-id "${protocol_id}" \
    --git-commit "${git_commit}" \
    --output-dir "${output_dir}" \
    --timeout-seconds 300 \
    --repetitions "${repetitions}" \
    --schedule interleaved \
    --schedule-seed 20260804 \
    --fixed-rho \
    --exact-admm || runner_status=$?
python3 "${repo_root}/benchmarks/analyze_nonlinear_benchmark.py" \
    "${output_dir}/results.csv" \
    --execution-status "${output_dir}/execution_status.csv" \
    --summary "${output_dir}/summary.csv" \
    --enriched "${output_dir}/enriched.csv" \
    --method-summary "${output_dir}/method_summary.csv"
exit "${runner_status}"
