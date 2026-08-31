#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 CONFIGURATION CC CXX smoke|nightly|replay" >&2
  exit 2
fi

configuration_name=$1
compiler_c=$2
compiler_cxx=$3
mode=$4
case "$mode" in
  smoke|nightly|replay) ;;
  *)
    echo "mode must be smoke, nightly or replay, got: $mode" >&2
    exit 2
    ;;
esac

if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi
if ! "$compiler_cxx" --version | grep -Eq 'clang version 15\.'; then
  echo "Goal 08 fuzzing requires Clang 15" >&2
  "$compiler_cxx" --version >&2
  exit 1
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_root="$ci_root/$configuration_name"
build_root="$configuration_root/build"
artifact_root="$configuration_root/artifacts"
corpus_root="$configuration_root/corpus"
source_fuzz_root="$repository_root/open_lmm/test/quality/fuzz"

reuse_build=${OPEN_LMM_FUZZ_REUSE_BUILD:-OFF}
if [[ -e "$configuration_root" && "$reuse_build" != ON ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi
mkdir -p "$artifact_root" "$corpus_root"
exec > >(tee "$configuration_root/ci.log") 2>&1

git -C "$repository_root" rev-parse HEAD > "$configuration_root/git-head.txt"
git -C "$repository_root" status --short > "$configuration_root/git-status.txt"
"$compiler_cxx" --version > "$configuration_root/compiler-version.txt"

cxx_compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
skip_configure=${OPEN_LMM_FUZZ_SKIP_CONFIGURE:-OFF}
if [[ "$skip_configure" == ON ]]; then
  if [[ "$reuse_build" != ON || ! -f "$build_root/CMakeCache.txt" ]]; then
    echo "OPEN_LMM_FUZZ_SKIP_CONFIGURE requires a reusable build" >&2
    exit 1
  fi
else
  cmake -S "$repository_root/open_lmm" -B "$build_root" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
    -DUSE_CCACHE=OFF \
    -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF \
    -DOPEN_LMM_BUILD_PYTHON=OFF \
    -DOPEN_LMM_ENABLE_FUZZING=ON
fi

build_attempt=1
until cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-8}" \
    --target open_lmm_fuzz_targets; do
  if [[ $build_attempt -ge 3 ]]; then
    echo "fuzz build failed after $build_attempt attempts" >&2
    exit 1
  fi
  build_attempt=$((build_attempt + 1))
  echo "compiler process failed; retrying unchanged incremental fuzz build ($build_attempt/3)" >&2
done

if [[ ! -e "$corpus_root/config_schema" ]]; then
  cp -a "$source_fuzz_root/corpus/config_schema" "$corpus_root/"
fi
if [[ ! -e "$corpus_root/agent_id" ]]; then
  cp -a "$source_fuzz_root/corpus/agent_id" "$corpus_root/"
fi

case "$mode" in
  smoke) seconds=${OPEN_LMM_FUZZ_SECONDS:-60} ;;
  nightly) seconds=${OPEN_LMM_FUZZ_SECONDS:-600} ;;
  replay) seconds=0 ;;
esac

run_target() {
  local target=$1
  local corpus=$2
  local dictionary=$3
  local maximum_length=$4
  local executable="$build_root/test/$target"
  local target_artifacts="$artifact_root/$target"
  mkdir -p "$target_artifacts"
  if [[ ! -x "$executable" ]]; then
    echo "fuzz executable is missing: $executable" >&2
    return 1
  fi
  local args=(
    -print_final_stats=1
    -verbosity=0
    -timeout="${OPEN_LMM_FUZZ_INPUT_TIMEOUT_SECONDS:-2}"
    -rss_limit_mb="${OPEN_LMM_FUZZ_RSS_LIMIT_MB:-2048}"
    -max_len="$maximum_length"
    -artifact_prefix="$target_artifacts/"
    -dict="$dictionary")
  if [[ "$mode" == replay ]]; then
    if [[ -z "${OPEN_LMM_FUZZ_REPLAY_INPUT:-}" ||
          ! -f "${OPEN_LMM_FUZZ_REPLAY_INPUT}" ]]; then
      echo "replay mode requires OPEN_LMM_FUZZ_REPLAY_INPUT" >&2
      return 2
    fi
    args+=(-runs=1 "$OPEN_LMM_FUZZ_REPLAY_INPUT")
  else
    args+=(-max_total_time="$seconds" -seed="${OPEN_LMM_FUZZ_SEED:-424242}" "$corpus")
  fi
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "$executable" "${args[@]}" 2>&1 | tee "$target_artifacts/run.log"
}

verify_artifact_replay_contract() {
  local contract_root="$artifact_root/replay-contract"
  local fixture_source="$contract_root/forced_crash_fuzz.cpp"
  local fixture_binary="$contract_root/forced_crash_fuzz"
  local crash_input="$contract_root/trigger"
  local crash_artifact="$contract_root/crash-artifact"
  local seed_corpus="$contract_root/seed-corpus"
  mkdir -p "$contract_root" "$seed_corpus"
  printf '%s\n' \
    '#include <cstddef>' \
    '#include <cstdint>' \
    'extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {' \
    '  if (size == 4 && data[0] == 0x43 && data[1] == 0x52 &&' \
    '      data[2] == 0x53 && data[3] == 0x48) __builtin_trap();' \
    '  return 0;' \
    '}' > "$fixture_source"
  printf 'CRSH' > "$crash_input"
  cp "$crash_input" "$seed_corpus/trigger"
  "$compiler_cxx" -std=c++20 -fsanitize=fuzzer,address,undefined \
    -fno-omit-frame-pointer -fno-pie -no-pie \
    "$fixture_source" -o "$fixture_binary"

  set +e
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
    timeout 10s "$fixture_binary" -runs=1 -exact_artifact_path="$crash_artifact" \
      "$seed_corpus" > "$contract_root/create.log" 2>&1
  local create_status=$?
  set -e
  if [[ $create_status -eq 0 || ! -f "$crash_artifact" ]]; then
    echo "forced fuzz crash did not create a replay artifact" >&2
    return 1
  fi

  set +e
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
    timeout 10s "$fixture_binary" -runs=1 "$crash_artifact" \
      > "$contract_root/replay.log" 2>&1
  local replay_status=$?
  set -e
  if [[ $replay_status -eq 0 ]]; then
    echo "forced fuzz crash artifact did not reproduce" >&2
    return 1
  fi
  printf 'create_status\t%s\nreplay_status\t%s\nartifact_sha256\t%s\n' \
    "$create_status" "$replay_status" \
    "$(sha256sum "$crash_artifact" | cut -d ' ' -f1)" \
    > "$contract_root/summary.tsv"
}

if [[ "$mode" == replay ]]; then
  replay_target=${OPEN_LMM_FUZZ_REPLAY_TARGET:-}
  case "$replay_target" in
    open_lmm_config_schema_fuzz)
      run_target "$replay_target" "$corpus_root/config_schema" \
        "$source_fuzz_root/dictionaries/config_schema.dict" 65536
      ;;
    open_lmm_agent_id_fuzz)
      run_target "$replay_target" "$corpus_root/agent_id" \
        "$source_fuzz_root/dictionaries/agent_id.dict" 128
      ;;
    *)
      echo "OPEN_LMM_FUZZ_REPLAY_TARGET must name a Goal 08 fuzz target" >&2
      exit 2
      ;;
  esac
else
  if [[ "${OPEN_LMM_VERIFY_FUZZ_ARTIFACT_CONTRACT:-ON}" == ON ]]; then
    verify_artifact_replay_contract
  fi
  run_target open_lmm_config_schema_fuzz "$corpus_root/config_schema" \
    "$source_fuzz_root/dictionaries/config_schema.dict" 65536
  run_target open_lmm_agent_id_fuzz "$corpus_root/agent_id" \
    "$source_fuzz_root/dictionaries/agent_id.dict" 128
fi

find "$corpus_root" -type f -print0 | sort -z | xargs -0 sha256sum \
  > "$configuration_root/corpus-sha256.txt"
printf 'mode\t%s\nseconds_per_target\t%s\nseed\t%s\n' \
  "$mode" "$seconds" "${OPEN_LMM_FUZZ_SEED:-424242}" \
  > "$configuration_root/summary.tsv"
echo "==> fuzzing verified: $configuration_name ($mode)"
