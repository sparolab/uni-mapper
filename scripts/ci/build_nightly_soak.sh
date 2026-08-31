#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 C_COMPILER CXX_COMPILER CONTAINER_DIGEST" >&2
  exit 2
fi

compiler_c=$1
compiler_cxx=$2
container_digest=${3#sha256:}
if [[ ! "$container_digest" =~ ^[0-9a-f]{64}$ ]]; then
  echo "container digest must contain 64 lowercase hexadecimal digits" >&2
  exit 2
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
evidence_root="$repository_root/.ci-build/nightly-soak"
build_root="$evidence_root/build"
if [[ -e "$evidence_root" ]]; then
  echo "nightly evidence path must be clean: $evidence_root" >&2
  exit 1
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

mkdir -p "$evidence_root"
CC="$compiler_c" CXX="$compiler_cxx" cmake \
  -S "$repository_root/open_lmm" -B "$build_root" \
  -DUSE_CCACHE=OFF \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$build_root" --parallel 2 \
  --target open_lmm_soak_metrics_contract_tests \
           open_lmm_soak_aggregate \
           open_lmm_runtime_lifecycle_stress_tests \
           open_lmm_runtime_concurrency_stress_tests \
           open_lmm_transaction_fault_stress_tests \
           open_lmm_visualization_stress_tests \
           open_lmm_plugin_stress_tests \
           open_lmm_resource_stress_tests \
           open_lmm_config_apply_stress_tests

OPEN_LMM_SOAK_REQUIRE_CLEAN=1 \
OPEN_LMM_SOAK_EVIDENCE_ROOT="$evidence_root/reports" \
  "$repository_root/scripts/ci/run_soak_tests.sh" \
    "$build_root" nightly "$container_digest"
