#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 CONFIGURATION CC CXX calibrate|verify" >&2
  exit 2
fi

configuration_name=$1
compiler_c=$2
compiler_cxx=$3
mode=$4
case "$mode" in
  calibrate|verify) ;;
  *)
    echo "mode must be calibrate or verify, got: $mode" >&2
    exit 2
    ;;
esac

if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi
if ! "$compiler_cxx" --version | grep -Eq 'clang version 15\.'; then
  echo "Goal 08 coverage requires Clang 15" >&2
  "$compiler_cxx" --version >&2
  exit 1
fi

llvm_cov=${OPEN_LMM_LLVM_COV:-$(command -v llvm-cov-15 || true)}
llvm_profdata=${OPEN_LMM_LLVM_PROFDATA:-$(command -v llvm-profdata-15 || true)}
if [[ ! -x "$llvm_cov" || ! -x "$llvm_profdata" ]]; then
  echo "llvm-cov-15 and llvm-profdata-15 are required" >&2
  exit 1
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
source_root="$repository_root/open_lmm"
source_manifest="$source_root/test/quality/coverage/critical_sources.tsv"
test_manifest_source="$source_root/test/quality/coverage/critical_tests.tsv"
baseline=${OPEN_LMM_COVERAGE_BASELINE:-"$source_root/test/quality/coverage/critical_branch_coverage_baseline.json"}
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_root="$ci_root/$configuration_name"
build_root="$configuration_root/build"
artifact_root="$configuration_root/artifacts"
profile_root="$configuration_root/profiles"

reuse_build=${OPEN_LMM_COVERAGE_REUSE_BUILD:-OFF}
if [[ -e "$configuration_root" && "$reuse_build" != ON ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi
mkdir -p "$artifact_root" "$profile_root"
exec > >(tee "$configuration_root/ci.log") 2>&1

git -C "$repository_root" rev-parse HEAD > "$configuration_root/git-head.txt"
git -C "$repository_root" status --short > "$configuration_root/git-status.txt"
"$compiler_cxx" --version > "$configuration_root/compiler-version.txt"
"$llvm_cov" --version > "$configuration_root/llvm-cov-version.txt"

cxx_compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
skip_configure=${OPEN_LMM_COVERAGE_SKIP_CONFIGURE:-OFF}
if [[ "$skip_configure" == ON ]]; then
  if [[ "$reuse_build" != ON || ! -f "$build_root/CMakeCache.txt" ]]; then
    echo "OPEN_LMM_COVERAGE_SKIP_CONFIGURE requires a reusable build" >&2
    exit 1
  fi
else
  cmake -S "$source_root" -B "$build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags -O0 -g1" \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
    -DUSE_CCACHE=OFF \
    -DOPEN_LMM_ENABLE_COVERAGE=ON
fi

mapfile -t tests < <(tail -n +2 "$test_manifest_source" | cut -f1)
mapfile -t targets < <(tail -n +2 "$test_manifest_source" | cut -f2 | sort -u)
if [[ ${#tests[@]} -eq 0 || ${#targets[@]} -eq 0 ]]; then
  echo "critical test manifest is empty" >&2
  exit 1
fi

build_attempt=1
until cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-8}" \
    --target "${targets[@]}"; do
  if [[ $build_attempt -ge 3 ]]; then
    echo "coverage build failed after $build_attempt attempts" >&2
    exit 1
  fi
  build_attempt=$((build_attempt + 1))
  echo "compiler process failed; retrying unchanged incremental coverage build ($build_attempt/3)" >&2
done

generated_manifest="$build_root/test/open_lmm_test_manifest.tsv"
if [[ ! -f "$generated_manifest" ]]; then
  echo "generated test manifest is missing: $generated_manifest" >&2
  exit 1
fi
for test_name in "${tests[@]}"; do
  if ! awk -F '\t' -v expected="$test_name" \
      'NR > 1 && $1 == expected { found = 1 } END { exit !found }' \
      "$generated_manifest"; then
    echo "critical coverage test is not registered: $test_name" >&2
    exit 1
  fi
done

test_regex="^($(IFS='|'; echo "${tests[*]}"))$"
find "$profile_root" -type f -name '*.profraw' -delete
LLVM_PROFILE_FILE="$profile_root/%p.profraw" \
  ctest --test-dir "$build_root" --output-on-failure -R "$test_regex" \
    --output-junit "$artifact_root/critical-coverage-tests.xml"

mapfile -t raw_profiles < <(find "$profile_root" -type f -name '*.profraw' | sort)
if [[ ${#raw_profiles[@]} -eq 0 ]]; then
  echo "coverage test run produced no raw profiles" >&2
  exit 1
fi
"$llvm_profdata" merge -sparse "${raw_profiles[@]}" \
  -o "$artifact_root/critical.profdata"

for target in "${targets[@]}"; do
  executable="$build_root/test/$target"
  if [[ ! -x "$executable" ]]; then
    echo "coverage test executable is missing: $executable" >&2
    exit 1
  fi
done
mapfile -t runtime_libraries < <(
  find "$build_root" -type f -name 'libopen_lmm_map_server.so.*' | sort)
if [[ ${#runtime_libraries[@]} -eq 0 ]]; then
  echo "instrumented runtime library is missing" >&2
  exit 1
fi
runtime_library_index=$((${#runtime_libraries[@]} - 1))
main_object=${runtime_libraries[$runtime_library_index]}
source_paths=()
while IFS=$'\t' read -r owner relative_path; do
  [[ "$owner" == owner ]] && continue
  source_paths+=("$source_root/$relative_path")
done < "$source_manifest"

"$llvm_cov" export "$main_object" \
  --instr-profile="$artifact_root/critical.profdata" \
  "${source_paths[@]}" \
  > "$artifact_root/critical-coverage-export.json"
"$llvm_cov" report "$main_object" \
  --instr-profile="$artifact_root/critical.profdata" \
  "${source_paths[@]}" \
  > "$artifact_root/critical-coverage-report.txt"
"$llvm_cov" show "$main_object" \
  --instr-profile="$artifact_root/critical.profdata" \
  "${source_paths[@]}" \
  > "$artifact_root/critical-coverage-show.txt"

compiler_identity=$($compiler_cxx --version | head -n 1)
llvm_cov_identity=$($llvm_cov --version | head -n 1)
measurement="$artifact_root/critical-coverage-measurement.json"
coverage_args=(
  --export "$artifact_root/critical-coverage-export.json"
  --sources "$source_manifest"
  --tests "$test_manifest_source"
  --source-root "$source_root"
  --repo "$repository_root"
  --output "$measurement"
  --mode "$mode"
  --compiler "$compiler_identity"
  --llvm-cov "$llvm_cov_identity"
  --image-digest "${OPEN_LMM_CI_IMAGE_DIGEST:-local-unpinned}")
if [[ "$mode" == verify ]]; then
  coverage_args+=(--baseline "$baseline")
fi
python3 "$script_dir/critical_coverage.py" "${coverage_args[@]}"

sha256sum "$source_manifest" "$test_manifest_source" "$measurement" \
  > "$configuration_root/artifact-sha256.txt"
printf 'mode\t%s\ntests\t%s\ncritical_owners\t%s\nraw_profiles\t%s\n' \
  "$mode" "${#tests[@]}" "${#source_paths[@]}" "${#raw_profiles[@]}" \
  > "$configuration_root/summary.tsv"
echo "==> critical coverage verified: $configuration_name ($mode)"
