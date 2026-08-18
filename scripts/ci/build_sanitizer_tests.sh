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
    test_pattern='open_lmm_(safety_regression|pipeline_controller|session_transaction|execution_spec|controller_concurrency|plugin_abi_v2|plugin_selection|self_contained_e2e)_tests'
    targets=(
      open_lmm_safety_regression_tests
      open_lmm_pipeline_controller_tests
      open_lmm_session_transaction_tests
      open_lmm_execution_spec_tests
      open_lmm_controller_concurrency_tests
      open_lmm_plugin_abi_v2_tests
      open_lmm_plugin_selection_tests
      open_lmm_self_contained_e2e_tests
      create_scan_context
      create_free_dom
      open_lmm_plugin_fixture_v2_valid
      open_lmm_plugin_fixture_v2_wrong_major
      open_lmm_plugin_fixture_v2_short_descriptor
      open_lmm_plugin_fixture_v2_partial_open
      open_lmm_plugin_fixture_v2_malformed_result
      open_lmm_plugin_fixture_v2_newer_minor
      open_lmm_plugin_fixture_v2_missing_close
      open_lmm_plugin_fixture_v2_null_handle
      open_lmm_plugin_fixture_v2_unsupported_minor
    )
    ;;
  TSAN)
    sanitizer_option=OPEN_LMM_ENABLE_TSAN
    test_pattern='open_lmm_(bounded_executor|controller_concurrency|runtime_service|session_transaction)_tests'
    targets=(
      open_lmm_bounded_executor_tests
      open_lmm_controller_concurrency_tests
      open_lmm_runtime_service_tests
      open_lmm_session_transaction_tests
    )
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

build_attempt=1
until cmake --build "$build_root" --parallel 1 --target "${targets[@]}"; do
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
      -R "$test_pattern"
else
  TSAN_OPTIONS="halt_on_error=1:suppressions=$script_dir/tsan.supp" \
    ctest --test-dir "$build_root" --output-on-failure \
      --output-junit "$configuration_root/ctest.xml" \
      -R "$test_pattern"
fi

echo "==> sanitizer verified: $configuration_name ($sanitizer)"
