#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 <python-from-pinned-venv> <installed-core-prefix> <wheel-directory>" >&2
  exit 2
fi

python_executable=$1
core_prefix=$2
wheel_directory=$3
adapter_directory=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [[ ! -x "${python_executable}" ]]; then
  echo "Python executable is missing: ${python_executable}" >&2
  exit 1
fi
python_version=$("${python_executable}" -c \
  'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
if [[ "${python_version}" != "3.10" ]]; then
  echo "OpenLMM local wheel requires CPython 3.10, got: ${python_version}" >&2
  exit 1
fi
if [[ ! -f "${core_prefix}/share/open_lmm/cmake/open_lmmConfig.cmake" ]]; then
  echo "installed OpenLMM core package is missing: ${core_prefix}" >&2
  exit 1
fi

mkdir -p "${wheel_directory}"
shopt -s nullglob
existing_wheels=("${wheel_directory}"/*.whl)
if (( ${#existing_wheels[@]} != 0 )); then
  echo "wheel directory must not contain pre-existing wheels: ${wheel_directory}" >&2
  exit 1
fi
env -u ROS_VERSION -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH \
  "${python_executable}" -m pip wheel \
  --no-build-isolation --no-deps \
  --config-settings \
    "cmake.define.CMAKE_PREFIX_PATH=${core_prefix}" \
  --config-settings \
    "cmake.define.OPEN_LMM_CORE_PREFIX=${core_prefix}" \
  --wheel-dir "${wheel_directory}" "${adapter_directory}"

generated_wheels=("${wheel_directory}"/*.whl)
if (( ${#generated_wheels[@]} != 1 )); then
  echo "wheel build must produce exactly one artifact" >&2
  exit 1
fi
wheel_name=$(basename "${generated_wheels[0]}")
if [[ "${wheel_name}" != open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl ]]; then
  echo "unexpected OpenLMM wheel tag: ${wheel_name}" >&2
  exit 1
fi
