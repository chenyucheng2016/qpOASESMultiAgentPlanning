#!/usr/bin/env bash
set -euo pipefail

mode="${1:-new}"
if [[ "${mode}" != "new" && "${mode}" != "resume" ]]; then
  echo "usage: $0 [new|resume] [git-commit]" >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

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
head_commit=$("${git_command[@]}" rev-parse HEAD 2>/dev/null || true)
if [[ -z "${head_commit}" ]]; then
  echo "cannot determine the TurboADMM-NL Git commit" >&2
  exit 2
fi
git_commit="${2:-${head_commit}}"
if [[ "${git_commit}" != "${head_commit}" ]]; then
  echo "git-commit does not match HEAD" >&2
  exit 2
fi
if [[ "${mode}" == "new" && -n "$("${git_command[@]}" status --porcelain --untracked-files=no)" ]]; then
  echo "a new frozen evaluation requires a clean tracked worktree" >&2
  exit 2
fi

case_manifest="benchmarks/instances/csdo/passing_bay_sweep/evaluation_manifest.csv"
result_root="build-ral-wsl/results/csdo_ral/evaluation_passing_bay_${git_commit:0:7}"
result_csv="${result_root}/results.csv"
work_root="${result_root}/cases"
csdo_root="../CSDOTrajectoryPlanning"

runner_args=(
  --manifest "${case_manifest}"
  --csdo-executable "${csdo_root}/build-private-overlay/csdo"
  --turbo-executable "build-ral-wsl/bin/csdo_turbo_comparison"
  --root-exporter "build-csdo-frontend-wsl/csdo_root_warmstart_exporter"
  --csdo-config "${csdo_root}/config.yaml"
  --work-root "${work_root}"
  --output-csv "${result_csv}"
  --protocol-id "csdo_passing_bay_evaluation_v1"
  --git-commit "${git_commit}"
  --schedule-seed 20260805
  --corridor-recovery-window 3
  --timeout 120
)
if [[ "${mode}" == "resume" ]]; then
  runner_args+=(--resume)
fi

python3 scripts/run_csdo_congested_suite.py "${runner_args[@]}"
python3 scripts/summarize_csdo_congested_suite.py \
  --manifest "${case_manifest}" \
  --work-root "${work_root}" \
  --status-csv "${result_root}/results_status.csv" \
  --results-csv "${result_csv}" \
  --output-csv "${result_root}/summary.csv"
python3 scripts/analyze_csdo_paired_statistics.py \
  "${result_root}/summary.csv" \
  --output "${result_root}/paired_statistics.csv"
