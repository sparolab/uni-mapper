#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <python-from-pinned-venv> <wheel-directory>" >&2
  exit 2
fi

python_executable=$1
wheel_directory=$2
adapter_directory=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

mkdir -p "${wheel_directory}"
env -u ROS_VERSION -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
  "${python_executable}" -m pip wheel \
  --no-build-isolation --no-deps \
  --wheel-dir "${wheel_directory}" "${adapter_directory}"
