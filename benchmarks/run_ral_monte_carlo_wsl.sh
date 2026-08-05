#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${BUILD_DIR:-"${repo_root}/build-ral-wsl"}
mode=${1:-pilot}

all_methods=full,centralized_osqp,centralized_qpoases,qp_continuation,inner,cold
case "${mode}" in
    pilot)
        suite=paper_development
        protocol_id=ral-monte-carlo-primary-pilot
        methods=${METHODS:-full,centralized_osqp}
        scenario_indices=${SCENARIO_INDICES:-0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230}
        ;;
    development)
        suite=paper_development
        protocol_id=ral-monte-carlo-primary-development
        methods=${METHODS:-${all_methods}}
        scenario_indices=${SCENARIO_INDICES:-}
        ;;
    final)
        suite=paper_final
        protocol_id=ral-monte-carlo-primary-final
        methods=${all_methods}
        scenario_indices=
        if [[ -n "${METHODS:-}" || -n "${SCENARIO_INDICES:-}" || -n "${TIMEOUT_SECONDS:-}" ]]; then
            echo "final mode does not accept method, scenario, or timeout overrides" >&2
            exit 2
        fi
        ;;
    *)
        echo "usage: $0 [pilot|development|final]" >&2
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
if [[ "${mode}" == "final" ]]; then
    if [[ -n "$("${git_command[@]}" status --porcelain --untracked-files=no)" ]]; then
        echo "final mode requires a clean tracked worktree" >&2
        exit 2
    fi
    if [[ -n "${TURBOADMM_GIT_COMMIT:-}" && "${TURBOADMM_GIT_COMMIT}" != "${git_commit}" ]]; then
        echo "TURBOADMM_GIT_COMMIT does not match HEAD" >&2
        exit 2
    fi
fi

output_dir=${OUTPUT_DIR:-"${build_dir}/results/${protocol_id}"}
timeout_seconds=${TIMEOUT_SECONDS:-120}

cmake -S "${repo_root}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DQPOASES_BUILD_EXAMPLES=OFF \
    -DQPOASES_BUILD_NONLINEAR_TESTS=ON \
    -DQPOASES_BUILD_CSDO_COMPARISON=ON \
    -DQPOASES_BUILD_OSQP_BACKEND=ON \
    -DOSQP_ROOT=/usr/local
cmake --build "${build_dir}" -j "$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure \
    -R 'nonlinear_paper_(development|final)_inventory')

index_arguments=()
if [[ -n "${scenario_indices}" ]]; then
    index_arguments=(--scenario-indices "${scenario_indices}")
fi

runner_status=0
python3 "${repo_root}/benchmarks/run_nonlinear_matrix.py" \
    "${build_dir}/bin/nonlinear_benchmark" \
    --suite "${suite}" \
    --track primary \
    "${index_arguments[@]}" \
    --methods "${methods}" \
    --protocol-id "${protocol_id}" \
    --git-commit "${git_commit}" \
    --output-dir "${output_dir}" \
    --timeout-seconds "${timeout_seconds}" \
    --repetitions 1 \
    --schedule interleaved \
    --schedule-seed 20260805 \
    --fixed-rho \
    --exact-admm || runner_status=$?

if [[ -f "${output_dir}/results.csv" ]]; then
    python3 "${repo_root}/benchmarks/analyze_nonlinear_benchmark.py" \
        "${output_dir}/results.csv" \
        --execution-status "${output_dir}/execution_status.csv" \
        --summary "${output_dir}/summary.csv" \
        --enriched "${output_dir}/enriched.csv" \
        --method-summary "${output_dir}/method_summary.csv"
    if [[ "${methods}" == *full* && "${methods}" == *centralized_osqp* ]]; then
        python3 "${repo_root}/benchmarks/analyze_ral_paired_scaling.py" \
            "${output_dir}/enriched.csv" \
            --inventory "${output_dir}/inventory.csv" \
            --execution-status "${output_dir}/execution_status.csv" \
            --pairs-output "${output_dir}/paired_results.csv" \
            --summary-output "${output_dir}/paired_summary.csv" \
            --aggregate-output "${output_dir}/paired_aggregate.csv"
    fi
fi
exit "${runner_status}"
