#!/usr/bin/env bash

set -euo pipefail

build_root=""
profile=""
fixture_id=""
scenario_request=""
repetitions=""
warmups=""
output_root=""
container_digest="${OPEN_LMM_BENCHMARK_CONTAINER_DIGEST:-}"
baseline=""
sanitizer="none"

while [[ $# -gt 0 ]]; do
  option=$1
  shift
  case "$option" in
    --build|--profile|--fixture|--scenario|--repetitions|--warmup|--output|--container-digest|--baseline|--sanitizer)
      if [[ $# -eq 0 ]]; then
        echo "missing value for $option" >&2
        exit 2
      fi
      value=$1
      shift
      ;;
    *)
      echo "unknown option: $option" >&2
      exit 2
      ;;
  esac
  case "$option" in
    --build) build_root=$value ;;
    --profile) profile=$value ;;
    --fixture) fixture_id=$value ;;
    --scenario) scenario_request=$value ;;
    --repetitions) repetitions=$value ;;
    --warmup) warmups=$value ;;
    --output) output_root=$value ;;
    --container-digest) container_digest=$value ;;
    --baseline) baseline=$value ;;
    --sanitizer) sanitizer=$value ;;
  esac
done

if [[ -z "$build_root" || -z "$profile" || -z "$fixture_id" ||
      -z "$scenario_request" || -z "$repetitions" || -z "$warmups" ||
      -z "$output_root" || -z "$container_digest" ]]; then
  echo "usage: $0 --build DIR --profile PROFILE --fixture ID --scenario NAME|map-update-pair|all-required --repetitions N --warmup N --output NEW_DIR --container-digest sha256:HEX [--baseline REVIEWED_CATALOG] [--sanitizer NAME]" >&2
  exit 2
fi
if [[ ! "$repetitions" =~ ^[1-9][0-9]*$ || ! "$warmups" =~ ^[0-9]+$ ]]; then
  echo "repetitions must be positive and warmup must be non-negative" >&2
  exit 2
fi
if [[ ! "$container_digest" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "container digest must be sha256 followed by 64 lowercase hex digits" >&2
  exit 2
fi
case "$profile" in
  contract|pr|nightly) ;;
  external|gpu)
    echo "$profile requires its dedicated asset/driver runner" >&2
    exit 77
    ;;
  *)
    echo "unsupported benchmark profile: $profile" >&2
    exit 2
    ;;
esac
case "$fixture_id" in
  small-v1|medium-v1) ;;
  *)
    echo "unsupported generated fixture: $fixture_id" >&2
    exit 2
    ;;
esac

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
build_root=$(cd "$build_root" && pwd)
fixture_generator="$build_root/test/open_lmm_benchmark_generate_fixture"
runner="$build_root/test/open_lmm_benchmark_runner"
aggregator="$build_root/test/open_lmm_benchmark_aggregate"
owner_runner="$build_root/test/open_lmm_benchmark_resource_owner_runner"
pairer="$build_root/test/open_lmm_benchmark_pair"
artifact_compare="$build_root/test/open_lmm_artifact_compare"
for executable in "$fixture_generator" "$runner" "$aggregator" "$owner_runner" "$pairer" "$artifact_compare"; do
  if [[ ! -x "$executable" ]]; then
    echo "benchmark executable is missing: $executable" >&2
    exit 1
  fi
done
benchmark_library_path=$(find "$build_root" -type f -name '*.so*' \
  -printf '%h\n' | sort -u | paste -sd: -)
runtime_environment=(env "LD_LIBRARY_PATH=$benchmark_library_path:${LD_LIBRARY_PATH:-}")
output_root=$(realpath -m "$output_root")
if [[ -e "$output_root" ]]; then
  echo "benchmark output path must not exist: $output_root" >&2
  exit 1
fi
if [[ -n "$baseline" && ! -f "$baseline" ]]; then
  echo "baseline file does not exist: $baseline" >&2
  exit 2
fi
if [[ "$scenario_request" == "map-update-pair" ]]; then
  scenarios=(map-update-sequential map-update-parallel)
elif [[ "$scenario_request" == "all-required" ]]; then
  if [[ "$profile" == "pr" || "$profile" == "contract" ]]; then
    scenarios=(open data-load map-update-sequential visualization-cold full-pipeline)
  else
    scenarios=(open data-load alignment map-update-sequential map-update-parallel save-fallback visualization-cold visualization-warm full-pipeline cancellation)
  fi
else
  scenarios=("$scenario_request")
fi
for scenario in "${scenarios[@]}"; do
  case "$scenario" in
    open|data-load|alignment|map-update-sequential|map-update-parallel|save-fallback|visualization-cold|visualization-warm|full-pipeline|cancellation) ;;
    *)
      echo "unsupported benchmark scenario: $scenario" >&2
      exit 2
      ;;
  esac
done

pair_requested=false
has_sequential=false
has_parallel=false
for scenario in "${scenarios[@]}"; do
  [[ "$scenario" == map-update-sequential ]] && has_sequential=true
  [[ "$scenario" == map-update-parallel ]] && has_parallel=true
done
if [[ "$has_sequential" == true && "$has_parallel" == true ]]; then
  pair_requested=true
fi
git_commit=$(git -C "$repository_root" rev-parse HEAD)
git_dirty=false
if [[ -n "$(git -C "$repository_root" status --porcelain)" ]]; then
  git_dirty=true
fi
if [[ "$profile" != "contract" && "$git_dirty" == true ]]; then
  echo "$profile benchmark requires a clean worktree" >&2
  exit 1
fi
compiler_executable=$(sed -n 's/^CMAKE_CXX_COMPILER:FILEPATH=//p' \
  "$build_root/CMakeCache.txt" | head -1)
build_type=$(sed -n 's/^CMAKE_BUILD_TYPE:STRING=//p' "$build_root/CMakeCache.txt" | head -1)
if [[ -z "$compiler_executable" || ! -x "$compiler_executable" ||
      -z "$build_type" ]]; then
  echo "compiler/build type metadata is missing from CMakeCache.txt" >&2
  exit 1
fi
compiler_version=$("$compiler_executable" --version)
compiler_version=${compiler_version%%$'\n'*}
if [[ -z "$compiler_version" ]]; then
  echo "compiler version metadata is unavailable: $compiler_executable" >&2
  exit 1
fi
compiler="$compiler_executable :: $compiler_version"

mkdir -p "$output_root"
template_roots=()
pair_root="$output_root/map-update-pair"
pair_artifact_root="$pair_root/artifacts"
for scenario in "${scenarios[@]}"; do
  scenario_root="$output_root/$scenario"
  mkdir -p "$scenario_root/warmup" "$scenario_root/measured"
  map_fixture_args=()
  fixture_mode=default
  case "$scenario" in
    map-update-sequential)
      map_fixture_args=(--map-update sequential)
      fixture_mode=map-update-sequential
      ;;
    map-update-parallel)
      map_fixture_args=(--map-update parallel)
      fixture_mode=map-update-parallel
      ;;
  esac
  fixture_template="$output_root/.fixture-template-$fixture_mode"
  if [[ ! -e "$fixture_template" ]]; then
    "$fixture_generator" "$fixture_id" "$fixture_template" \
      "${map_fixture_args[@]}"
    template_roots+=("$fixture_template")
  fi
  total_runs=$((warmups + repetitions))
  measured_reports=()
  owner_reports=()
  for ((run = 1; run <= total_runs; ++run)); do
    if ((run <= warmups)); then
      run_kind=warmup
      iteration=$run
    else
      run_kind=measured
      iteration=$((run - warmups))
    fi
    fixture_root="$scenario_root/$run_kind/fixture-$iteration"
    report_path="$scenario_root/$run_kind/report-$iteration.json"
    cp -a --reflink=auto "$fixture_template" "$fixture_root"
    runner_args=(
      --profile "$profile"
      --scenario "$scenario"
      --fixture-root "$fixture_root"
      --report "$report_path"
      --iteration "$iteration"
      --git-commit "$git_commit"
      --compiler "$compiler"
      --build-type "$build_type"
      --sanitizer "$sanitizer"
      --container-digest "$container_digest")
    if [[ "$git_dirty" == true ]]; then
      runner_args+=(--git-dirty)
    fi
    "${runtime_environment[@]}" "$runner" "${runner_args[@]}"
    if [[ "$run_kind" == measured ]]; then
      measured_reports+=(--report "$report_path")
      if [[ "$pair_requested" == true &&
            ("$scenario" == map-update-sequential ||
             "$scenario" == map-update-parallel) ]]; then
        mkdir -p "$pair_artifact_root"
        mapfile -t runtime_outputs < <(
          find "$fixture_root/output" -mindepth 1 -maxdepth 1 -type d -print)
        if [[ ${#runtime_outputs[@]} -ne 1 ]]; then
          echo "map-update output must contain exactly one runtime directory" >&2
          exit 1
        fi
        mv "${runtime_outputs[0]}" \
          "$pair_artifact_root/$scenario-$iteration"
      fi
      case "$scenario" in
        data-load|alignment|map-update-sequential|map-update-parallel|save-fallback|visualization-cold|visualization-warm)
          owner_fixture="$scenario_root/owner/fixture-$iteration"
          owner_report="$scenario_root/owner/report-$iteration.json"
          mkdir -p "$scenario_root/owner"
          cp -a --reflink=auto "$fixture_template" "$owner_fixture"
          owner_args=("${runner_args[@]}")
          for ((argument = 0; argument < ${#owner_args[@]}; ++argument)); do
            if [[ "${owner_args[$argument]}" == --fixture-root ]]; then
              owner_args[$((argument + 1))]=$owner_fixture
            elif [[ "${owner_args[$argument]}" == --report ]]; then
              owner_args[$((argument + 1))]=$owner_report
            fi
          done
          "${runtime_environment[@]}" "$owner_runner" "${owner_args[@]}"
          owner_reports+=(--report "$owner_report")
          rm -rf -- "$owner_fixture"
          ;;
      esac
    fi
    rm -rf -- "$fixture_root"
  done
  aggregate_args=(--output "$scenario_root/bundle.json")
  if [[ -n "$baseline" ]]; then
    aggregate_args+=(--baseline "$baseline")
  fi
  aggregate_args+=("${measured_reports[@]}")
  "$aggregator" "${aggregate_args[@]}"
  if [[ ${#owner_reports[@]} -gt 0 ]]; then
    owner_aggregate_args=(--output "$scenario_root/owner-bundle.json")
    if [[ -n "$baseline" ]]; then
      owner_aggregate_args+=(--baseline "$baseline")
    fi
    owner_aggregate_args+=("${owner_reports[@]}")
    "$aggregator" "${owner_aggregate_args[@]}"
  fi
done

if [[ "$pair_requested" == true ]]; then
  mkdir -p "$pair_root"
  parity_args=()
  semantic_failure=false
  for ((iteration = 1; iteration <= repetitions; ++iteration)); do
    parity_log="$pair_root/parity-$iteration.log"
    set +e
    "$artifact_compare" \
      "$pair_artifact_root/map-update-sequential-$iteration" \
      "$pair_artifact_root/map-update-parallel-$iteration" \
      --pcd-only 0 0 0.04 0.4 0.001 >"$parity_log" 2>&1
    parity_status=$?
    set -e
    case "$parity_status" in
      0) parity_result=pass ;;
      1)
        parity_result=fail
        semantic_failure=true
        ;;
      *)
        echo "map-update parity comparator could not validate iteration $iteration" >&2
        exit 1
        ;;
    esac
    parity_args+=(--parity "$iteration:$parity_result:$parity_log")
  done
  set +e
  "$pairer" \
    --sequential-bundle "$output_root/map-update-sequential/bundle.json" \
    --parallel-bundle "$output_root/map-update-parallel/bundle.json" \
    --sequential-owner-bundle \
      "$output_root/map-update-sequential/owner-bundle.json" \
    --parallel-owner-bundle \
      "$output_root/map-update-parallel/owner-bundle.json" \
    --output "$pair_root/report.json" \
    "${parity_args[@]}"
  pair_status=$?
  set -e
  if [[ "$pair_status" -ne 0 || "$semantic_failure" == true ]]; then
    echo "paired map-update correctness failed; evidence retained at $pair_root" >&2
    exit 1
  fi
  rm -rf -- "$pair_artifact_root"
fi
for fixture_template in "${template_roots[@]}"; do
  rm -rf -- "$fixture_template"
done

echo "benchmark evidence: $output_root"
