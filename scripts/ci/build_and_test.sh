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
exec > >(tee "$configuration_root/ci.log") 2>&1

echo "==> clean build: $configuration_name"
echo "    CC=$compiler_c"
echo "    CXX=$compiler_cxx"
echo "    GUI=$gui_enabled"

build_attempt=1
until CC="$compiler_c" CXX="$compiler_cxx" \
  colcon --log-base "$log_root" build \
    --base-paths "$repository_root/open_lmm" "$repository_root/ros" \
    --build-base "$build_root" \
    --install-base "$install_root" \
    --cmake-args \
      -DUSE_CCACHE=OFF \
      -DOPEN_LMM_BUILD_IRIDESCENCE_GUI="$gui_enabled" \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      "${compiler_cmake_args[@]}"; do
  if [[ $build_attempt -ge 3 ]]; then
    echo "clean build failed after $build_attempt attempts" >&2
    exit 1
  fi
  build_attempt=$((build_attempt + 1))
  echo "compiler process failed; retrying unchanged incremental build ($build_attempt/3)" >&2
done

ctest --test-dir "$build_root/open_lmm" --output-on-failure \
  --output-junit "$configuration_root/ctest-open_lmm.xml"
ctest --test-dir "$build_root/open_lmm_ros" --output-on-failure \
  --output-junit "$configuration_root/ctest-open_lmm-ros.xml"

# ament configuration intentionally excludes the mutating install/consumer
# CTest. Run the same source-free package fixture explicitly for every compiler
# matrix entry so package export cannot silently regress.
cmake \
  -DOPEN_LMM_BUILD_DIR="$build_root/open_lmm" \
  -DOPEN_LMM_SOURCE_DIR="$repository_root/open_lmm" \
  -DOPEN_LMM_PACKAGE_TEST_ROOT="$configuration_root/package-test" \
  -DOPEN_LMM_CONSUMER_C_COMPILER="$compiler_c" \
  -DOPEN_LMM_CONSUMER_CXX_COMPILER="$compiler_cxx" \
  -DOPEN_LMM_CONSUMER_CXX_FLAGS="$cxx_compatibility_flags" \
  -P "$repository_root/open_lmm/test/package_consumer_tests.cmake"

compiler_cache="$build_root/open_lmm/CMakeCache.txt"
expected_compiler="CMAKE_CXX_COMPILER:FILEPATH=$compiler_cxx"
if ! grep -Fxq "$expected_compiler" "$compiler_cache"; then
  echo "unexpected compiler in $compiler_cache" >&2
  grep '^CMAKE_CXX_COMPILER:' "$compiler_cache" >&2 || true
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
