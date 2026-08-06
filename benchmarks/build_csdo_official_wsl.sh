#!/usr/bin/env bash
set -euo pipefail

csdo_commit="0a391dbb17a4713b9c8d29a663a64f43b57df707"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
external_root="${1:-${repo_root}/../CSDOTrajectoryPlanning}"
dependency_prefix="${repo_root}/build-ral-wsl/csdo-deps/prefix"
source_root="${repo_root}/build-ral-wsl/csdo-official-${csdo_commit:0:7}-src"
build_root="${repo_root}/build-ral-wsl/csdo-official-${csdo_commit:0:7}-build"
exporter_build="${repo_root}/build-csdo-frontend-wsl"
source_marker="${source_root}/.csdo_source_commit"

if [[ ! -d "${external_root}/.git" ]]; then
  echo "CSDO checkout not found at ${external_root}" >&2
  exit 2
fi
if ! git -C "${external_root}" cat-file -e "${csdo_commit}^{commit}"; then
  echo "CSDO checkout does not contain ${csdo_commit}" >&2
  exit 2
fi
if [[ ! -f "${dependency_prefix}/lib/cmake/osqp/osqp-config.cmake" ]]; then
  echo "private OSQP 0.6.3 prefix is missing at ${dependency_prefix}" >&2
  exit 2
fi

if [[ ! -d "${source_root}" ]]; then
  mkdir -p "${source_root}"
  git -C "${external_root}" archive "${csdo_commit}" \
    | tar -x -C "${source_root}"
  printf '%s\n' "${csdo_commit}" > "${source_marker}"
elif [[ -f "${source_marker}" ]]; then
  if [[ "$(<"${source_marker}")" != "${csdo_commit}" ]]; then
    echo "clean CSDO source marker does not match ${csdo_commit}" >&2
    exit 2
  fi
else
  for tracked_file in CMakeLists.txt config.yaml csdo.cc; do
    if ! git -C "${external_root}" show \
        "${csdo_commit}:${tracked_file}" \
        | cmp - "${source_root}/${tracked_file}"; then
      echo "existing clean source does not match ${tracked_file}" >&2
      exit 2
    fi
  done
  printf '%s\n' "${csdo_commit}" > "${source_marker}"
fi

private_include="-I${dependency_prefix}/include"
cmake -S "${source_root}" -B "${build_root}" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dosqp_DIR="${dependency_prefix}/lib/cmake/osqp" \
  -DEigen3_DIR="${dependency_prefix}/share/eigen3/cmake" \
  -DCMAKE_PREFIX_PATH="${dependency_prefix}" \
  -DCMAKE_CXX_FLAGS="${private_include}"
cmake --build "${build_root}" -j "$(nproc)"

cmake -S "${repo_root}/benchmarks/csdo_frontend" \
  -B "${exporter_build}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCSDO_SOURCE_DIR="${source_root}" \
  -DEigen3_DIR="${dependency_prefix}/share/eigen3/cmake" \
  -DCMAKE_PREFIX_PATH="${dependency_prefix}" \
  -DCMAKE_CXX_FLAGS="${private_include}"
cmake --build "${exporter_build}" -j "$(nproc)"

if [[ ! -f "${repo_root}/build-ral-wsl/CMakeCache.txt" ]]; then
  echo "configure build-ral-wsl for TurboADMM-NL before building the bridge" >&2
  exit 2
fi
cmake --build "${repo_root}/build-ral-wsl" \
  --target csdo_turbo_comparison -j "$(nproc)"

sha256sum \
  "${build_root}/csdo" \
  "${exporter_build}/csdo_root_warmstart_exporter" \
  "${repo_root}/build-ral-wsl/bin/csdo_turbo_comparison"
