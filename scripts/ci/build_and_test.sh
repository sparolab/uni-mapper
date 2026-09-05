#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 NAME CC CXX GUI_ON_OR_OFF" >&2
  exit 2
fi

configuration_name=$1
compiler_c=$2
compiler_cxx=$3
gui_enabled=$4

case "$gui_enabled" in
  ON|OFF) ;;
  *)
    echo "GUI setting must be ON or OFF, got: $gui_enabled" >&2
    exit 2
    ;;
esac

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_root="$ci_root/$configuration_name"
core_source_root="$configuration_root/core-source"
build_root="$configuration_root/build"
install_root="$configuration_root/install"
log_root="$configuration_root/log"

if [[ -e "$configuration_root" ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi

if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi

compiler_cmake_args=()
cxx_compatibility_flags=""
if [[ "$(basename "$compiler_cxx")" == clang++* ]]; then
  cxx_compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
  compiler_cmake_args+=(
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags")
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS setup scripts may reference unset variables.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

mkdir -p "$configuration_root"
cmake -E copy_directory "$repository_root/open_lmm" "$core_source_root"
exec > >(tee "$configuration_root/ci.log") 2>&1

echo "==> clean build: $configuration_name"
echo "    CC=$compiler_c"
echo "    CXX=$compiler_cxx"
echo "    GUI=$gui_enabled"
cmake --version

build_attempt=1
until CC="$compiler_c" CXX="$compiler_cxx" \
  colcon --log-base "$log_root" build \
    --base-paths "$core_source_root" \
    --build-base "$build_root" \
    --install-base "$install_root" \
    --cmake-args \
      -DUSE_CCACHE=OFF \
      -DOPEN_LMM_ENABLE_STRICT_WARNINGS=ON \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      "${compiler_cmake_args[@]}"; do
  if [[ $build_attempt -ge 3 ]]; then
    echo "clean build failed after $build_attempt attempts" >&2
    exit 1
  fi
  build_attempt=$((build_attempt + 1))
  echo "compiler process failed; retrying unchanged incremental build ($build_attempt/3)" >&2
done

core_prefix="$install_root/open_lmm"
core_compile_database="$build_root/open_lmm/compile_commands.json"
if [[ ! -f "$core_compile_database" ]]; then
  echo "isolated core compile database is missing: $core_compile_database" >&2
  exit 1
fi
if grep -Fq "$repository_root/open_lmm/" "$core_compile_database" ||
   grep -Fq "$repository_root/applications/" "$core_compile_database" ||
   grep -Fq "$repository_root/bindings/" "$core_compile_database" ||
   grep -Fq "$repository_root/ros/" "$core_compile_database"; then
  echo "isolated core build retained a repository source edge" >&2
  exit 1
fi
gui_source_root="$configuration_root/gui-source"
gui_build_root="$configuration_root/gui-build"
cmake -E copy_directory "$repository_root/applications/gui" "$gui_source_root"
CC="$compiler_c" CXX="$compiler_cxx" cmake \
  -S "$gui_source_root" -B "$gui_build_root" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$core_prefix" \
  -DOPEN_LMM_GUI_REPOSITORY_ROOT="$repository_root" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DOPEN_LMM_GUI_BUILD_IRIDESCENCE="$gui_enabled" \
  -DOPEN_LMM_GUI_WARNINGS_AS_ERRORS=ON \
  -DBUILD_TESTING=ON \
  "${compiler_cmake_args[@]}"
cmake --build "$gui_build_root" --parallel 2
gui_compile_database="$gui_build_root/compile_commands.json"
if [[ ! -f "$gui_compile_database" ]]; then
  echo "staged GUI compile database is missing: $gui_compile_database" >&2
  exit 1
fi
if grep -Fq "$repository_root/applications/gui/" "$gui_compile_database" ||
   grep -Fq "$repository_root/open_lmm/src/" "$gui_compile_database"; then
  echo "staged GUI build retained an original application/core source edge" >&2
  exit 1
fi
if ! grep -Fq "$gui_source_root/src/host/gui_plugin_module.cpp" \
    "$gui_compile_database"; then
  echo "staged GUI build did not compile the Loader-B production source" >&2
  exit 1
fi
CC="$compiler_c" CXX="$compiler_cxx" \
  colcon --log-base "$log_root/ros" build \
    --base-paths "$repository_root/ros" \
    --build-base "$build_root" \
    --install-base "$install_root" \
    --cmake-args \
      -DCMAKE_PREFIX_PATH="$core_prefix" \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      "${compiler_cmake_args[@]}"

ctest --test-dir "$build_root/open_lmm" --output-on-failure \
  --output-junit "$configuration_root/ctest-open_lmm.xml"
ctest --test-dir "$gui_build_root" --output-on-failure \
  --output-junit "$configuration_root/ctest-open_lmm-gui.xml"
cmake --install "$gui_build_root" --prefix "$core_prefix"
ctest --test-dir "$build_root/open_lmm_ros" --output-on-failure \
  --output-junit "$configuration_root/ctest-open_lmm-ros.xml"

# One existing required matrix entry also proves the opposite feature-option
# shape. The main matrix builds all bundled plugins; this focused configure
# disables every optional descriptor/remover DSO and executes the typed
# selection contract without adding or renaming a required GitHub check.
if [[ "$configuration_name" == "gcc12-gui-off" ]]; then
  feature_off_root="$configuration_root/feature-off-build"
  CC="$compiler_c" CXX="$compiler_cxx" cmake \
    -S "$repository_root/open_lmm" -B "$feature_off_root" \
    -DUSE_CCACHE=OFF \
    -DOPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=OFF \
    -DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=OFF \
    -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS=OFF \
    -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP=OFF \
    -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD=OFF \
    -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=OFF \
    -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=OFF \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "${compiler_cmake_args[@]}"
  cmake --build "$feature_off_root" \
    --target open_lmm_plugin_selection_tests --parallel 2
  ctest --test-dir "$feature_off_root" --output-on-failure \
    -R '^open_lmm_plugin_selection_tests$' \
    --output-junit "$configuration_root/ctest-feature-off.xml"
  cp "$feature_off_root/test/open_lmm_test_manifest.tsv" \
    "$configuration_root/open_lmm_test_manifest-feature-off.tsv"
fi

test_manifest="$build_root/open_lmm/test/open_lmm_test_manifest.tsv"
if [[ ! -f "$test_manifest" ]]; then
  echo "generated test manifest is missing: $test_manifest" >&2
  exit 1
fi
cp "$test_manifest" "$configuration_root/open_lmm_test_manifest.tsv"
: > "$configuration_root/test-layer-counts.txt"
for layer in L1 L2 L3 L4 L5 L6; do
  layer_count=$(ctest --test-dir "$build_root/open_lmm" -N \
    -L "layer:$layer" | sed -n 's/^Total Tests: //p')
  printf '%s\t%s\n' "$layer" "${layer_count:-0}" \
    >> "$configuration_root/test-layer-counts.txt"
done
printf 'gpu\tNOT_AVAILABLE\nexternal\tNOT_AVAILABLE\n' \
  > "$configuration_root/test-lane-status.txt"

# ament configuration intentionally excludes the mutating install/consumer
# CTest. Run the same source-free package fixture explicitly for every compiler
# matrix entry so package export cannot silently regress.
cmake \
  -DOPEN_LMM_BUILD_DIR="$build_root/open_lmm" \
  -DOPEN_LMM_SOURCE_DIR="$core_source_root" \
  -DOPEN_LMM_PACKAGE_TEST_ROOT="$configuration_root/package-test" \
  -DOPEN_LMM_CONSUMER_C_COMPILER="$compiler_c" \
  -DOPEN_LMM_CONSUMER_CXX_COMPILER="$compiler_cxx" \
  -DOPEN_LMM_CONSUMER_CXX_FLAGS="$cxx_compatibility_flags" \
  -P "$repository_root/open_lmm/test/package/orchestrator/package_consumer_tests.cmake"

cmake \
  -DOPEN_LMM_REPOSITORY_ROOT="$repository_root" \
  -DOPEN_LMM_CORE_PREFIX="$configuration_root/package-test/install" \
  -DOPEN_LMM_CORE_BUILD_DIR="$build_root/open_lmm" \
  -DOPEN_LMM_CLI_TEST_ROOT="$configuration_root/cli-package-test" \
  -DOPEN_LMM_CLI_C_COMPILER="$compiler_c" \
  -DOPEN_LMM_CLI_CXX_COMPILER="$compiler_cxx" \
  -DOPEN_LMM_CLI_CXX_FLAGS="$cxx_compatibility_flags" \
  -P "$repository_root/applications/cli/test/cli_package_tests.cmake"

cmake \
  -DOPEN_LMM_REPOSITORY_ROOT="$repository_root" \
  -DOPEN_LMM_CORE_PREFIX="$configuration_root/package-test/install" \
  -DOPEN_LMM_CORE_BUILD_DIR="$build_root/open_lmm" \
  -DOPEN_LMM_PYTHON_TEST_ROOT="$configuration_root/python-package-test" \
  -DOPEN_LMM_PYTHON_CXX_COMPILER="$compiler_cxx" \
  -DOPEN_LMM_PYTHON_CXX_FLAGS="$cxx_compatibility_flags" \
  -P "$repository_root/bindings/python/test/package/python_package_tests.cmake"

distribution_build_root="$configuration_root/distribution-build"
cmake -S "$repository_root/distribution" -B "$distribution_build_root" \
  -DOPEN_LMM_DISTRIBUTION_CORE_BUILD_DIR="$build_root/open_lmm" \
  -DOPEN_LMM_DISTRIBUTION_CLI_BUILD_DIR="$configuration_root/cli-package-test/build" \
  -DOPEN_LMM_DISTRIBUTION_GUI_BUILD_DIR="$gui_build_root" \
  -DOPEN_LMM_DISTRIBUTION_ROS_BUILD_DIR="$build_root/open_lmm_ros"
if [[ "$configuration_name" == "gcc12-gui-off" ]]; then
  distribution_label='source-contract|package-composition'
else
  distribution_label='package-composition'
fi
ctest --test-dir "$distribution_build_root" --output-on-failure \
  -L "$distribution_label" \
  --output-junit "$configuration_root/ctest-distribution.xml"

"$repository_root/scripts/ci/inspect_symbol_visibility.sh" \
  "$install_root/open_lmm" "$configuration_root/symbol-visibility"

compiler_cache="$build_root/open_lmm/CMakeCache.txt"
expected_compiler="CMAKE_CXX_COMPILER:FILEPATH=$compiler_cxx"
if ! grep -Fxq "$expected_compiler" "$compiler_cache"; then
  echo "unexpected compiler in $compiler_cache" >&2
  grep '^CMAKE_CXX_COMPILER:' "$compiler_cache" >&2 || true
  exit 1
fi
gui_compiler_cache="$gui_build_root/CMakeCache.txt"
if ! grep -Fxq "$expected_compiler" "$gui_compiler_cache"; then
  echo "unexpected compiler in $gui_compiler_cache" >&2
  grep '^CMAKE_CXX_COMPILER:' "$gui_compiler_cache" >&2 || true
  exit 1
fi

gui_plugin="$install_root/open_lmm/lib/libopen_lmm_iridescence_gui.so"
if [[ "$gui_enabled" == ON && ! -f "$gui_plugin" ]]; then
  echo "GUI build did not install plugin: $gui_plugin" >&2
  exit 1
fi
if [[ "$gui_enabled" == OFF && -e "$gui_plugin" ]]; then
  echo "GUI OFF build unexpectedly installed plugin: $gui_plugin" >&2
  exit 1
fi

echo "==> verified: $configuration_name"
