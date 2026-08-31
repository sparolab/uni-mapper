#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 NAME ASAN_UBSAN_OR_TSAN CC CXX" >&2
  exit 2
fi

configuration_name=$1
sanitizer=$2
compiler_c=$3
compiler_cxx=$4

case "$sanitizer" in
  ASAN_UBSAN)
    sanitizer_option=OPEN_LMM_ENABLE_ASAN_UBSAN
    sanitizer_label=asan-ubsan
    ;;
  TSAN)
    sanitizer_option=OPEN_LMM_ENABLE_TSAN
    sanitizer_label=tsan
    ;;
  *)
    echo "sanitizer must be ASAN_UBSAN or TSAN, got: $sanitizer" >&2
    exit 2
    ;;
esac

if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi

cxx_compatibility_flags=""
if [[ "$(basename "$compiler_cxx")" == clang++* ]]; then
  cxx_compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_root="$ci_root/$configuration_name"
build_root="$configuration_root/build"

if [[ -e "$configuration_root" ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi

mkdir -p "$configuration_root"
exec > >(tee "$configuration_root/ci.log") 2>&1

cmake -S "$repository_root/open_lmm" -B "$build_root" \
  -DCMAKE_BUILD_TYPE=Debug \
  "-DCMAKE_C_FLAGS_DEBUG=-O0 -g1" \
  "-DCMAKE_CXX_FLAGS_DEBUG=-O0 -g1" \
  -DCMAKE_C_COMPILER="$compiler_c" \
  -DCMAKE_CXX_COMPILER="$compiler_cxx" \
  "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
  -DUSE_CCACHE=OFF \
  -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF \
  -D"$sanitizer_option"=ON

test_manifest="$build_root/test/open_lmm_test_manifest.tsv"
if [[ ! -f "$test_manifest" ]]; then
  echo "generated test manifest is missing: $test_manifest" >&2
  exit 1
fi
mapfile -t targets < <(
  awk -F '\t' -v sanitizer="$sanitizer_label" \
    'NR > 1 && index($8, sanitizer) { print $2 }' "$test_manifest")
if [[ ${#targets[@]} -eq 0 ]]; then
  echo "no tests selected for sanitizer:$sanitizer_label" >&2
  exit 1
fi
cp "$test_manifest" "$configuration_root/open_lmm_test_manifest.tsv"

build_attempt=1
until cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-16}" --target "${targets[@]}"; do
  if [[ $build_attempt -ge 3 ]]; then
    echo "sanitizer build failed after $build_attempt attempts" >&2
    exit 1
  fi
  build_attempt=$((build_attempt + 1))
  echo "compiler process failed; retrying unchanged incremental build ($build_attempt/3)" >&2
done

if [[ "$sanitizer" == ASAN_UBSAN ]]; then
  # LeakSanitizer cannot attach in the containerized/ptrace-restricted CI
  # environment; AddressSanitizer and UBSan remain fully enabled.
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    ctest --test-dir "$build_root" --output-on-failure \
      --output-junit "$configuration_root/ctest.xml" \
      -L "sanitizer:$sanitizer_label"
else
  tsan_runner=()
  if command -v setarch >/dev/null 2>&1; then
    machine_architecture=$(uname -m)
    if setarch "$machine_architecture" -R true >/dev/null 2>&1; then
      # TSan reserves a fixed shadow-memory range.  Disabling ASLR avoids the
      # kernel mapping collision reported as "unexpected memory mapping" on
      # affected Linux runners.
      tsan_runner=(setarch "$machine_architecture" -R)
    fi
  fi
  TSAN_OPTIONS="halt_on_error=1:suppressions=$script_dir/tsan.supp" \
    "${tsan_runner[@]}" ctest --test-dir "$build_root" --output-on-failure \
      --output-junit "$configuration_root/ctest.xml" \
      --timeout "${OPEN_LMM_TSAN_TIMEOUT_SECONDS:-180}" \
      -L "sanitizer:$sanitizer_label"
fi

echo "==> sanitizer verified: $configuration_name ($sanitizer)"
