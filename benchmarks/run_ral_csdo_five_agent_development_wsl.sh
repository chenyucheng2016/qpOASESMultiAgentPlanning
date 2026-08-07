#!/usr/bin/env bash
set -euo pipefail

mode="${1:-new}"
if [[ "${mode}" != "new" && "${mode}" != "resume" ]]; then
  echo "usage: $0 [new|resume] [git-commit]" >&2
  exit 2
fi
git_commit="${2:-}"
if [[ -z "${git_commit}" ]]; then
  git_commit="$(git rev-parse HEAD)"
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

case_manifest="benchmarks/instances/csdo/congested/within_flow_manifest.csv"
result_root="build-ral-wsl/results/csdo_ral/development_five_agent_joint_repair_v2"
result_csv="${result_root}/results.csv"
work_root="${result_root}/cases"
csdo_commit="0a391dbb17a4713b9c8d29a663a64f43b57df707"
csdo_source="build-ral-wsl/csdo-official-${csdo_commit:0:7}-src"
csdo_build="build-ral-wsl/csdo-official-${csdo_commit:0:7}-build"

runner_args=(
  --manifest "${case_manifest}"
  --csdo-executable "${csdo_build}/csdo"
  --turbo-executable "build-ral-wsl/bin/csdo_turbo_comparison"
  --root-exporter "build-csdo-frontend-wsl/csdo_root_warmstart_exporter"
  --csdo-config "${csdo_source}/config.yaml"
  --work-root "${work_root}"
  --output-csv "${result_csv}"
  --protocol-id "csdo_five_agent_development_v4"
  --git-commit "${git_commit}"
  --csdo-source-commit "${csdo_commit}"
  --schedule-seed 20260806
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
