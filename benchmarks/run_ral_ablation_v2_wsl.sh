#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${BUILD_DIR:-"${repo_root}/build-ral-wsl"}
protocol_id=ral-continuation-ablation-development-v2

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
if [[ -n "$("${git_command[@]}" status --porcelain --untracked-files=no)" ]]; then
    echo "ablation protocol requires a clean tracked worktree" >&2
    exit 2
fi

output_dir=${OUTPUT_DIR:-"${build_dir}/results/ral-continuation-ablation-v2-${git_commit:0:7}"}

METHODS=full,inner,qp_continuation \
PROTOCOL_ID=${protocol_id} \
OUTPUT_DIR=${output_dir} \
TIMEOUT_SECONDS=${TIMEOUT_SECONDS:-120} \
    "${repo_root}/benchmarks/run_ral_monte_carlo_wsl.sh" development

comparisons=(
    "full inner"
    "full qp_continuation"
    "qp_continuation inner"
)
for comparison in "${comparisons[@]}"; do
    read -r candidate baseline <<< "${comparison}"
    prefix=${candidate}_vs_${baseline}
    python3 "${repo_root}/benchmarks/analyze_ral_paired_scaling.py" \
        "${output_dir}/enriched.csv" \
        --inventory "${output_dir}/inventory.csv" \
        --execution-status "${output_dir}/execution_status.csv" \
        --candidate "${candidate}" \
        --baseline "${baseline}" \
        --pairs-output "${output_dir}/${prefix}_pairs.csv" \
        --summary-output "${output_dir}/${prefix}_summary.csv" \
        --aggregate-output "${output_dir}/${prefix}_aggregate.csv"
done

python3 "${repo_root}/benchmarks/check_ral_ablation_gate_v2.py" \
    "${output_dir}/full_vs_inner_pairs.csv" \
    "${output_dir}/full_vs_qp_continuation_pairs.csv" \
    "${output_dir}/qp_continuation_vs_inner_pairs.csv" \
    --manifest "${output_dir}/run_manifest.json" \
    --expected-commit "${git_commit}" \
    --expected-protocol "${protocol_id}" \
    --expected-pairs 240 \
    --maximum-absolute-objective-gap-percent 5.0 \
    --minimum-total-runtime-reduction-percent 20.0 \
    --minimum-matrix-runtime-reduction-percent 20.0 \
    --minimum-pair-runtime-reduction-percent 5.0 \
    --minimum-pair-largest-scale-runtime-reduction-percent 10.0 \
    --minimum-total-qp-reduction-percent 20.0 \
    --minimum-pair-qp-reduction-percent 20.0 \
    --maximum-runtime-sign-p-value 0.05 \
    --output "${output_dir}/ablation_gate_v2.json"

echo "RA-L continuation ablation v2 passed: ${output_dir}"
