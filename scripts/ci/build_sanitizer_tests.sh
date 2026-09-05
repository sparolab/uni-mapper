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

if [[ "$sanitizer" == TSAN && "$(basename "$compiler_cxx")" != clang++* ]]; then
  echo "ROS TSan requires Clang and LLVM OpenMP/Archer (CI uses clang-15)." >&2
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
core_install_root="$configuration_root/core-install"
python_build_root="$configuration_root/python-build"
gui_build_root="$configuration_root/gui-build"

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
  asan_runner=()
  if command -v setarch >/dev/null 2>&1; then
    machine_architecture=$(uname -m)
    if setarch "$machine_architecture" -R true >/dev/null 2>&1; then
      # Clang's ASan runtime reserves a fixed shadow-memory range before the
      # instrumented process starts. Disable ASLR when the runner permits it
      # so direct tests and any child processes inherit a collision-free map.
      asan_runner=(setarch "$machine_architecture" -R)
    fi
  fi
  # LeakSanitizer cannot attach in the containerized/ptrace-restricted CI
  # environment; AddressSanitizer and UBSan remain fully enabled.
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "${asan_runner[@]}" ctest --test-dir "$build_root" --output-on-failure \
      --output-junit "$configuration_root/ctest.xml" \
      -L "sanitizer:$sanitizer_label"

  # The native bridge is a separate installed-core consumer. Instrument it
  # with the same Clang ASan/UBSan runtime and run the seven C++ bridge
  # contracts without claiming unsupported TSan-instrumented CPython coverage.
  cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-16}"
  cmake --install "$build_root" --prefix "$core_install_root"
  gui_sanitizer_flags="$cxx_compatibility_flags -fsanitize=address,undefined -fno-omit-frame-pointer"
  cmake -S "$repository_root/applications/gui" -B "$gui_build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$gui_sanitizer_flags" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined" \
    -DCMAKE_PREFIX_PATH="$core_install_root" \
    -DOPEN_LMM_GUI_BUILD_IRIDESCENCE=OFF \
    -DOPEN_LMM_GUI_WARNINGS_AS_ERRORS=ON \
    -DBUILD_TESTING=ON
  cmake --build "$gui_build_root" --parallel 2
  # Keep the DSO stress lane's allocator retention bounded while preserving
  # AddressSanitizer instrumentation and its use-after-free quarantine.
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1:quarantine_size_mb=1:thread_local_quarantine_size_kb=64 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "${asan_runner[@]}" ctest --test-dir "$gui_build_root" --output-on-failure \
      --output-junit "$configuration_root/ctest-gui.xml" \
      -L "sanitizer:$sanitizer_label"
  python_sanitizer_flags="$cxx_compatibility_flags -fsanitize=address,undefined -fno-omit-frame-pointer"
  cmake -S "$repository_root/bindings/python" -B "$python_build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$python_sanitizer_flags" \
    "-DCMAKE_MODULE_LINKER_FLAGS=-fsanitize=address,undefined" \
    -DCMAKE_PREFIX_PATH="$core_install_root" \
    -DOPEN_LMM_CORE_PREFIX="$core_install_root" \
    -DOPEN_LMM_PYTHON_WARNINGS_AS_ERRORS=ON \
    -DOPEN_LMM_PYTHON_CORE_CONTRACT_ONLY=ON \
    -DBUILD_TESTING=ON
  cmake --build "$python_build_root" --parallel 2
  asan_runtime=$(
    "$compiler_cxx" -print-file-name=libclang_rt.asan-x86_64.so)
  if [[ ! -f "$asan_runtime" ]]; then
    asan_runtime=$("$compiler_cxx" -print-file-name=libasan.so)
  fi
  if [[ ! -f "$asan_runtime" ]]; then
    echo "Clang ASan runtime is missing: $asan_runtime" >&2
    exit 1
  fi
  cmake -S "$repository_root/bindings/python" -B "$python_build_root" \
    -DOPEN_LMM_PYTHON_TEST_LD_PRELOAD="$asan_runtime" \
    -DOPEN_LMM_PYTHON_TEST_LIBRARY_PATH="$core_install_root/lib"
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  OPEN_LMM_PYTHON_PROCESS_EXIT_TIMEOUT_SECONDS=60 \
    "${asan_runner[@]}" ctest \
      --test-dir "$python_build_root" --output-on-failure \
      --output-junit "$configuration_root/ctest-python.xml" \
      -R '^open_lmm_python_(public_api|error|runtime|config|callback|numpy|lifetime)_tests$'
  cp "$python_build_root/test/open_lmm_python_test_manifest.tsv" \
    "$configuration_root/open_lmm_python_test_manifest.tsv"
else
  cmake --build "$build_root" --parallel "${OPEN_LMM_BUILD_JOBS:-16}"
  cmake --install "$build_root" --prefix "$core_install_root"
  gui_sanitizer_flags="$cxx_compatibility_flags -fsanitize=thread -fno-omit-frame-pointer"
  cmake -S "$repository_root/applications/gui" -B "$gui_build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$gui_sanitizer_flags" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread" \
    -DCMAKE_PREFIX_PATH="$core_install_root" \
    -DOPEN_LMM_GUI_BUILD_IRIDESCENCE=OFF \
    -DOPEN_LMM_GUI_WARNINGS_AS_ERRORS=ON \
    -DBUILD_TESTING=ON
  cmake --build "$gui_build_root" --parallel 2
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
  TSAN_OPTIONS="halt_on_error=1:suppressions=$script_dir/tsan.supp" \
    "${tsan_runner[@]}" ctest --test-dir "$gui_build_root" \
      --output-on-failure \
      --output-junit "$configuration_root/ctest-gui.xml" \
      --timeout "${OPEN_LMM_TSAN_TIMEOUT_SECONDS:-180}" \
      -L "sanitizer:$sanitizer_label"
fi

if [[ "$sanitizer" == TSAN ]]; then
  bash "$script_dir/run_ros_tsan_tests.sh" "$configuration_root/ros-tsan" \
    "$compiler_c" "$compiler_cxx" "$core_install_root"
fi

echo "==> sanitizer verified: $configuration_name ($sanitizer)"
