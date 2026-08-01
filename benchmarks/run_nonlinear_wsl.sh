#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${BUILD_DIR:-"${repo_root}/build-nonlinear-wsl"}
suite=${1:-ci}
method=${2:-all}
repetitions=${REPETITIONS:-1}
threads=${THREADS:-0}

export OMP_DYNAMIC=FALSE
export OMP_PROC_BIND=close
export OMP_PLACES=cores

cmake -S "${repo_root}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DQPOASES_BUILD_EXAMPLES=OFF \
    -DQPOASES_BUILD_NONLINEAR_TESTS=ON \
    -DQPOASES_BUILD_OSQP_BACKEND=ON \
    -DOSQP_ROOT=/usr/local
cmake --build "${build_dir}" -j "$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure)
output_dir="${build_dir}/results/${suite}_${method}"
method_args=()
runner_args=(--repetitions "${repetitions}")
if [[ "${threads}" -gt 0 ]]; then
    runner_args+=(--threads "${threads}")
fi
if [[ "${method}" != "all" ]]; then
    method_args=(--methods "${method}")
fi
runner_status=0
python3 "${repo_root}/benchmarks/run_nonlinear_matrix.py" \
    "${build_dir}/bin/nonlinear_benchmark" \
    --suite "${suite}" --output-dir "${output_dir}" --timeout-seconds 120 \
    "${method_args[@]}" "${runner_args[@]}" || runner_status=$?
python3 "${repo_root}/benchmarks/analyze_nonlinear_benchmark.py" \
    "${output_dir}/results.csv" \
    --execution-status "${output_dir}/execution_status.csv" \
    --summary "${output_dir}/summary.csv" \
    --enriched "${output_dir}/enriched.csv" \
    --method-summary "${output_dir}/method_summary.csv"
exit "${runner_status}"
