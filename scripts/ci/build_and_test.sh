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

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS setup scripts may reference unset variables.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

mkdir -p "$configuration_root"

echo "==> clean build: $configuration_name"
echo "    CC=$compiler_c"
echo "    CXX=$compiler_cxx"
echo "    GUI=$gui_enabled"

CC="$compiler_c" CXX="$compiler_cxx" \
  colcon --log-base "$log_root" build \
    --base-paths "$repository_root/open_lmm" "$repository_root/ros" \
    --build-base "$build_root" \
    --install-base "$install_root" \
    --symlink-install \
    --cmake-args \
      -DUSE_CCACHE=OFF \
      -DOPEN_LMM_BUILD_IRIDESCENCE_GUI="$gui_enabled" \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

ctest --test-dir "$build_root/open_lmm" --output-on-failure
ctest --test-dir "$build_root/open_lmm_ros/open_lmm" --output-on-failure

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
