#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)

cmake \
  -DOPEN_LMM_SOURCE_DIR="$repository_root/open_lmm" \
  -P "$repository_root/open_lmm/test/architecture/policy/architecture_boundary_tests.cmake"

cmake \
  -DOPEN_LMM_REPOSITORY_ROOT="$repository_root" \
  -P "$repository_root/distribution/test/architecture/application_boundary_inventory_tests.cmake"

cmake \
  -DOPEN_LMM_REPOSITORY_ROOT="$repository_root" \
  -P "$repository_root/distribution/test/release/release_policy_tests.cmake"

cmake \
  -DOPEN_LMM_SOURCE_DIR="$repository_root/open_lmm" \
  -P "$repository_root/distribution/test/architecture/repository_architecture_tests.cmake"

echo "==> architecture and release policy verified"
