#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 4 || $# -gt 5 ]]; then
  echo "usage: $0 CC CXX FIXTURE_ID CONTAINER_DIGEST [pr|nightly]" >&2
  exit 2
fi

compiler_c=$1
compiler_cxx=$2
fixture_id=$3
container_digest=$4
profile=${5:-nightly}
build_jobs=${OPEN_LMM_BUILD_JOBS:-2}
if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi
if [[ ! "$container_digest" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "container digest must be canonical sha256 metadata" >&2
  exit 2
fi
if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "OPEN_LMM_BUILD_JOBS must be a positive integer" >&2
  exit 2
fi
case "$profile" in
  pr)
    warmups=1
    repetitions=5
    ;;
  nightly)
    warmups=2
    repetitions=10
    ;;
  *)
    echo "benchmark CI profile must be pr or nightly" >&2
    exit 2
    ;;
esac

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
compiler_name=$(basename "$compiler_cxx")
configuration_root="$ci_root/$profile-benchmark-$compiler_name"
build_root="$configuration_root/build"
evidence_root="$configuration_root/evidence"
if [[ -e "$configuration_root" ]]; then
  echo "benchmark CI path must be clean: $configuration_root" >&2
  exit 1
fi

compiler_args=()
if [[ "$compiler_name" == clang++* ]]; then
  compiler_args+=(
    "-DCMAKE_CXX_FLAGS=-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward")
fi
mkdir -p "$configuration_root"
exec > >(tee "$configuration_root/benchmark.log") 2>&1

CC="$compiler_c" CXX="$compiler_cxx" cmake \
  -S "$repository_root/open_lmm" -B "$build_root" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_CCACHE=OFF \
  -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF \
  -DOPEN_LMM_ENABLE_BENCHMARK_WORKFLOW_TEST=ON \
  "${compiler_args[@]}"
cmake --build "$build_root" --parallel "$build_jobs" --target \
  open_lmm_benchmark_generate_fixture \
  open_lmm_benchmark_runner \
  open_lmm_benchmark_resource_owner_runner \
  open_lmm_benchmark_aggregate \
  open_lmm_benchmark_pair \
  open_lmm_artifact_compare
ctest --test-dir "$build_root" --output-on-failure \
  -R '^open_lmm_benchmark_(statistics|options|stage_event_recorder|report_contract|fixture_policy|process_sampler|small_smoke|orchestrator)_tests$'

baseline_args=()
if [[ -n "${OPEN_LMM_PERFORMANCE_BASELINE:-}" ]]; then
  if [[ ! -f "$OPEN_LMM_PERFORMANCE_BASELINE" ]]; then
    echo "configured performance baseline does not exist: $OPEN_LMM_PERFORMANCE_BASELINE" >&2
    exit 1
  fi
  baseline_args=(--baseline "$OPEN_LMM_PERFORMANCE_BASELINE")
elif [[ "$profile" == nightly &&
        -f "$repository_root/docs/post_freeze_results/performance_baseline.json" ]]; then
  baseline_args=(
    --baseline
    "$repository_root/docs/post_freeze_results/performance_baseline.json")
fi

"$repository_root/scripts/benchmark/run_benchmarks.sh" \
  --build "$build_root" \
  --profile "$profile" \
  --fixture "$fixture_id" \
  --scenario all-required \
  --repetitions "$repetitions" \
  --warmup "$warmups" \
  --output "$evidence_root" \
  --container-digest "$container_digest" \
  "${baseline_args[@]}"

echo "$profile benchmark evidence: $evidence_root"
