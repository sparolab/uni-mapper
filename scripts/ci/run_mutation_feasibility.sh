#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 CONFIGURATION CC CXX" >&2
  exit 2
fi

configuration_name=$1
compiler_c=$2
compiler_cxx=$3
if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi
if ! "$compiler_cxx" --version | grep -Eq 'clang version 15\.'; then
  echo "Goal 08 mutation feasibility requires Clang 15" >&2
  exit 1
fi
for tool in cmake tar timeout /usr/bin/time python3; do
  if ! command -v "$tool" >/dev/null; then
    echo "mutation pilot dependency is missing: $tool" >&2
    exit 1
  fi
done

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_root="$ci_root/$configuration_name"
artifact_root="$configuration_root/artifacts"
if [[ -e "$configuration_root" ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi
mkdir -p "$artifact_root"
exec > >(tee "$configuration_root/ci.log") 2>&1

start_seconds=$(date +%s)
git_head=$(git -C "$repository_root" rev-parse HEAD)
git -C "$repository_root" status --short > "$configuration_root/git-status.txt"
printf '%s\n' "$git_head" > "$configuration_root/git-head.txt"
"$compiler_cxx" --version > "$configuration_root/compiler-version.txt"
for tool in mull-runner-15 mull-runner mull-cxx-15 mull-cxx; do
  if command -v "$tool" >/dev/null; then
    printf '%s\t%s\n' "$tool" "$(command -v "$tool")"
  else
    printf '%s\tNOT_FOUND\n' "$tool"
  fi
done > "$artifact_root/mutation-tool-availability.tsv"

worktree_root=$(mktemp -d "$configuration_root/worktree.XXXXXX")
cleanup() {
  if [[ "${OPEN_LMM_MUTATION_KEEP_WORKTREE:-OFF}" != ON ]]; then
    cmake -E remove_directory "$worktree_root"
  fi
}
trap cleanup EXIT
mutant_repo="$worktree_root/repository"
mkdir -p "$mutant_repo"
tar -C "$repository_root" \
  --exclude=./.git --exclude=./.ci-build --exclude=./build \
  --exclude=./install --exclude=./log -cf - . | tar -C "$mutant_repo" -xf -

source_file="$mutant_repo/open_lmm/src/runtime/state/runtime_state_store.cpp"
pristine_source="$worktree_root/runtime_state_store.pristine.cpp"
cp "$source_file" "$pristine_source"
build_root="$worktree_root/build"
cxx_compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
cmake -S "$mutant_repo/open_lmm" -B "$build_root" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_COMPILER="$compiler_c" \
  -DCMAKE_CXX_COMPILER="$compiler_cxx" \
  "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
  -DUSE_CCACHE=OFF \
  -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF \
  -DOPEN_LMM_BUILD_PYTHON=OFF \
  -DOPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=OFF \
  -DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=OFF

targets=(open_lmm_runtime_state_store_tests open_lmm_runtime_revision_property_tests)
cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-4}" \
  --target "${targets[@]}"
for target in "${targets[@]}"; do
  "$build_root/test/$target" > "$artifact_root/baseline-$target.log" 2>&1
done

results="$artifact_root/mutation-results.tsv"
printf 'id\tdescription\tstatus\tbuild_seconds\tpeak_rss_kb\ttest_detail\n' \
  > "$results"
limit=${OPEN_LMM_MUTATION_LIMIT:-20}
if [[ ! "$limit" =~ ^[0-9]+$ || $limit -lt 1 || $limit -gt 20 ]]; then
  echo "OPEN_LMM_MUTATION_LIMIT must be in 1..20" >&2
  exit 2
fi

mutant_index=0
while IFS=$'\t' read -r mutant_id description; do
  [[ "$mutant_id" == id ]] && continue
  mutant_index=$((mutant_index + 1))
  [[ $mutant_index -gt $limit ]] && break
  echo "[$mutant_index/$limit] mutation $mutant_id: $description"
  python3 "$script_dir/mutation_pilot.py" apply \
    --mutant "$mutant_id" --source "$pristine_source" --output "$source_file"

  build_log="$artifact_root/$mutant_id-build.log"
  : > "$build_log"
  build_status=1
  build_seconds=0
  peak_rss_kb=0
  build_attempt=0
  while [[ $build_status -ne 0 && $build_attempt -lt 3 ]]; do
    build_attempt=$((build_attempt + 1))
    timing="$artifact_root/$mutant_id-build-time-$build_attempt.txt"
    set +e
    /usr/bin/time -f '%e\t%M' -o "$timing" \
      cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-4}" \
        --target "${targets[@]}" \
        >> "$build_log" 2>&1
    build_status=$?
    set -e
    read -r attempt_seconds attempt_rss_kb < <(tail -n 1 "$timing")
    build_seconds=$(awk -v total="$build_seconds" -v current="$attempt_seconds" \
      'BEGIN { printf "%.2f", total + current }')
    if [[ $attempt_rss_kb -gt $peak_rss_kb ]]; then
      peak_rss_kb=$attempt_rss_kb
    fi
    if [[ $build_status -ne 0 && $build_attempt -lt 3 ]]; then
      echo "mutation build failed; retrying unchanged ($build_attempt/3)" \
        >> "$build_log"
    fi
  done

  status=compile-error
  detail="build-exit-$build_status"
  if [[ $build_status -eq 0 ]]; then
    status=survived
    detail=all-tests-passed
    for target in "${targets[@]}"; do
      set +e
      timeout "${OPEN_LMM_MUTATION_TEST_TIMEOUT_SECONDS:-20}s" \
        "$build_root/test/$target" \
        > "$artifact_root/$mutant_id-$target.log" 2>&1
      test_status=$?
      set -e
      if [[ $test_status -eq 124 ]]; then
        status=timeout
        detail="$target-timeout"
        break
      fi
      if [[ $test_status -ne 0 ]]; then
        status=killed
        detail="$target-exit-$test_status"
        break
      fi
    done
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$mutant_id" "$description" "$status" "$build_seconds" \
    "$peak_rss_kb" "$detail" >> "$results"
done < <(python3 "$script_dir/mutation_pilot.py" list)

end_seconds=$(date +%s)
python3 "$script_dir/mutation_pilot.py" report \
  --results "$results" \
  --output "$artifact_root/mutation-feasibility.json" \
  --source "$repository_root/open_lmm/src/runtime/state/runtime_state_store.cpp" \
  --compiler "$($compiler_cxx --version | head -n 1)" \
  --git-head "$git_head" \
  --elapsed-seconds "$((end_seconds - start_seconds))"

cp "$pristine_source" "$source_file"
sha256sum "$results" "$artifact_root/mutation-feasibility.json" \
  > "$configuration_root/artifact-sha256.txt"
python3 - "$artifact_root/mutation-feasibility.json" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
counts = report["counts"]
print(
    "==> mutation feasibility verified: "
    f"{report['mutants_requested']} mutants, {counts['killed']} killed, "
    f"{counts['survived']} survived, {counts['equivalent']} equivalent, "
    f"{counts['compile-error']} compile errors, "
    f"{counts['timeout']} timeouts (required gate excluded)"
)
PY
