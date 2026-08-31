#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "usage: $0 --build DIR --profile fast|sanitizer|nightly|gpu" \
       "--scenario runtime-lifecycle|all-headless|gpu-real-driver" \
       "--iterations N --warmup N --seed N" \
       "--report NEW_FILE [--container-digest SHA256]" >&2
}

build_root=""
profile=""
scenario=""
iterations=""
warmup=""
seed=""
report=""
container_digest=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) build_root=$2; shift 2 ;;
    --profile) profile=$2; shift 2 ;;
    --scenario) scenario=$2; shift 2 ;;
    --iterations) iterations=$2; shift 2 ;;
    --warmup) warmup=$2; shift 2 ;;
    --seed) seed=$2; shift 2 ;;
    --report) report=$2; shift 2 ;;
    --container-digest) container_digest=${2#sha256:}; shift 2 ;;
    --help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

if [[ -z "$build_root" || -z "$profile" ||
      ( "$scenario" != "runtime-lifecycle" && "$scenario" != "all-headless" &&
        "$scenario" != "gpu-real-driver" ) ||
      ! "$iterations" =~ ^[0-9]+$ || ! "$warmup" =~ ^[0-9]+$ ||
      ! "$seed" =~ ^[0-9]+$ || -z "$report" ]]; then
  usage
  exit 2
fi
case "$profile" in
  fast|sanitizer|nightly) ;;
  gpu)
    if [[ "$scenario" != "gpu-real-driver" ]]; then
      usage
      exit 2
    fi
    ;;
  *) usage; exit 2 ;;
esac
if [[ "$scenario" == "gpu-real-driver" && "$profile" != "gpu" ]]; then
  usage
  exit 2
fi
if [[ -n "$container_digest" &&
      ! "$container_digest" =~ ^[0-9a-f]{64}$ ]]; then
  echo "container digest must contain 64 lowercase hexadecimal digits" >&2
  exit 2
fi
if [[ -e "$report" ]]; then
  echo "report path must not already exist: $report" >&2
  exit 2
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
git_commit=$(git -C "$repository_root" rev-parse HEAD)
git_state=()
if [[ -n "$(git -C "$repository_root" status --porcelain)" ]]; then
  if [[ "${OPEN_LMM_SOAK_REQUIRE_CLEAN:-0}" == "1" ]]; then
    echo "required soak run refuses a dirty source tree" >&2
    exit 1
  fi
  git_state+=(--git-dirty)
fi

if [[ -n "$container_digest" ]]; then
  export OPEN_LMM_CONTAINER_DIGEST="sha256:$container_digest"
fi

# A sourced install workspace may place older OpenLMM DSOs ahead of CMake's
# RUNPATH. Soak runners must exercise the requested build tree as one unit.
build_library_path=""
while IFS= read -r directory; do
  if [[ -z "$build_library_path" ]]; then
    build_library_path=$directory
  else
    build_library_path="$build_library_path:$directory"
  fi
done < <(find "$build_root/src" -type f -name 'libopen_lmm_*.so*' \
  -printf '%h\n' | sort -u)
if [[ -n "$build_library_path" ]]; then
  export LD_LIBRARY_PATH="$build_library_path${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

common_args=(
  --profile "$profile"
  --iterations "$iterations"
  --warmup "$warmup"
  --seed "$seed"
  --git-commit "$git_commit"
  "${git_state[@]}")

if [[ "$scenario" == "gpu-real-driver" ]]; then
  gpu_runner="$build_root/test/open_lmm_gui_driver_soak_tests"
  if [[ ! -x "$gpu_runner" ]]; then
    echo "real-driver GPU soak runner is unavailable: $gpu_runner" >&2
    exit 77
  fi
  "$gpu_runner" "${common_args[@]}" --report "$report"
  exit 0
fi

runner="$build_root/test/open_lmm_runtime_lifecycle_stress_tests"
if [[ ! -x "$runner" ]]; then
  echo "soak runner is unavailable: $runner" >&2
  exit 77
fi

if [[ "$scenario" == "runtime-lifecycle" ]]; then
  "$runner" "${common_args[@]}" --scenario "$scenario" --report "$report"
  exit 0
fi

family_root="${report%.json}.families"
if [[ -e "$family_root" ]]; then
  echo "family evidence path must not already exist: $family_root" >&2
  exit 2
fi

concurrency_runner="$build_root/test/open_lmm_runtime_concurrency_stress_tests"
transaction_runner="$build_root/test/open_lmm_transaction_fault_stress_tests"
visualization_runner="$build_root/test/open_lmm_visualization_stress_tests"
plugin_runner="$build_root/test/open_lmm_plugin_stress_tests"
resource_runner="$build_root/test/open_lmm_resource_stress_tests"
config_runner="$build_root/test/open_lmm_config_apply_stress_tests"
aggregate_runner="$build_root/test/open_lmm_soak_aggregate"
for required_runner in "$concurrency_runner" "$transaction_runner" "$visualization_runner" \
    "$plugin_runner" "$resource_runner" "$config_runner" "$aggregate_runner"; do
  if [[ ! -x "$required_runner" ]]; then
    echo "soak runner is unavailable: $required_runner" >&2
    exit 77
  fi
done
mkdir -p "$family_root"

lifecycle_report="$family_root/runtime-lifecycle.json"
concurrency_report="$family_root/runtime-concurrency.json"
transaction_report="$family_root/transaction-fault.json"
visualization_report="$family_root/visualization.json"
plugin_report="$family_root/plugin.json"
resource_report="$family_root/resource.json"
config_report="$family_root/config-apply.json"

run_logged() {
  local log_file=$1
  shift
  if "$@" >"$log_file" 2>&1; then
    tail -n 1 "$log_file"
  else
    cat "$log_file" >&2
    return 1
  fi
}

run_logged "$family_root/runtime-lifecycle.log" \
  "$runner" "${common_args[@]}" --scenario all-headless \
    --report "$lifecycle_report"
run_logged "$family_root/runtime-concurrency.log" \
  "$concurrency_runner" "${common_args[@]}" --report "$concurrency_report"
run_logged "$family_root/transaction-fault.log" \
  "$transaction_runner" "${common_args[@]}" --report "$transaction_report"
run_logged "$family_root/visualization.log" \
  "$visualization_runner" "${common_args[@]}" --report "$visualization_report"
run_logged "$family_root/plugin.log" "$plugin_runner" \
  "$build_root/test/libopen_lmm_plugin_fixture_valid.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_wrong_abi.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_null_factory.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_missing_destroy.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_no_entry.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_null_kind.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_null_name.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_empty_capability.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_null_capability.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_create_throw.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_entry_throw.so" \
  "$build_root/test/libopen_lmm_plugin_fixture_null_entry.so" \
  "$build_root/libcreate_scan_context.so" \
  "${common_args[@]}" --report "$plugin_report"
run_logged "$family_root/resource.log" \
  "$resource_runner" "${common_args[@]}" --report "$resource_report"
run_logged "$family_root/config-apply.log" \
  "$config_runner" "${common_args[@]}" --report "$config_report"

"$aggregate_runner" --output "$report" \
  "$lifecycle_report" "$concurrency_report" "$transaction_report" \
  "$visualization_report" \
  "$plugin_report" "$resource_report" "$config_report"
