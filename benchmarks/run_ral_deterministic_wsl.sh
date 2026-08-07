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

if [[ -f "${repo_root}/.git" ]]; then
    git_dir=$(sed -n 's/^gitdir: //p' "${repo_root}/.git")
    if [[ "${git_dir}" =~ ^([A-Za-z]):/(.*)$ ]]; then
        drive=${BASH_REMATCH[1],,}
        git_dir="/mnt/${drive}/${BASH_REMATCH[2]}"
    elif [[ "${git_dir}" != /* ]]; then
        git_dir="${repo_root}/${git_dir}"
    fi
    git_command=(git --git-dir="${git_dir}" --work-tree="${repo_root}")
else
    git_command=(git -c "safe.directory=${repo_root}" -C "${repo_root}")
fi
git_commit=$("${git_command[@]}" rev-parse HEAD 2>/dev/null || true)
if [[ -z "${git_commit}" ]]; then
    echo "cannot determine the TurboADMM-NL Git commit" >&2
    exit 2
fi
if [[ -n "${TURBOADMM_GIT_COMMIT:-}" && "${TURBOADMM_GIT_COMMIT}" != "${git_commit}" ]]; then
    echo "TURBOADMM_GIT_COMMIT does not match HEAD" >&2
    exit 2
fi
if [[ "${mode}" == "final" ]]; then
    if [[ -n "$("${git_command[@]}" status --porcelain --untracked-files=no)" ]]; then
        echo "final mode requires a clean tracked worktree" >&2
        exit 2
    fi
fi

output_dir=${OUTPUT_DIR:-"${build_dir}/results/${protocol_id}-${git_commit:0:7}"}
cases=easy_open,easy_single_blocker,medium_doorway,hard_heterogeneous_doorway,hard_warehouse,very_hard_maze
methods=full,centralized_osqp,centralized_qpoases,qp_continuation,inner,cold

mkdir -p "$(dirname "${output_dir}")"
exec 9>"${output_dir}.lock"
if ! flock -n 9; then
    echo "another benchmark process is already using ${output_dir}" >&2
    exit 2
fi

cmake -S "${repo_root}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DQPOASES_BUILD_EXAMPLES=OFF \
    -DQPOASES_BUILD_NONLINEAR_TESTS=ON \
    -DQPOASES_BUILD_CSDO_COMPARISON=ON \
    -DQPOASES_BUILD_OSQP_BACKEND=ON \
    -DOSQP_ROOT=/usr/local
cmake --build "${build_dir}" -j "$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure)

gate_status=0
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
python3 "${repo_root}/benchmarks/check_ral_deterministic_gate.py" \
    "${output_dir}/results.csv" \
    --cases "${cases}" \
    --repetitions "${repetitions}" \
    --maximum-absolute-objective-gap-percent 5.0 || gate_status=$?
if [[ "${gate_status}" -ne 0 ]]; then
    exit "${gate_status}"
fi
exit "${runner_status}"
