#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 NAME CC CXX PINNED_PYTHON" >&2
  exit 2
fi

configuration_name=$1
compiler_c=$2
compiler_cxx=$3
pinned_python=$4
for executable in "$compiler_c" "$compiler_cxx" "$pinned_python"; do
  if [[ ! -x "$executable" ]]; then
    echo "required executable is missing: $executable" >&2
    exit 1
  fi
done

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_root="$ci_root/$configuration_name"
core_build="$configuration_root/core-build"
core_install="$configuration_root/core-install"
wheel_directory="$configuration_root/wheel"
wheel_test_build="$configuration_root/wheel-test-build"

if [[ -e "$configuration_root" ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi
mkdir -p "$configuration_root"
exec > >(tee "$configuration_root/ci.log") 2>&1
install_venv_root=$(mktemp -d "/tmp/openlmm-${configuration_name}-install-XXXXXX")
install_venv="$install_venv_root/venv"
printf '%s\n' "$install_venv" > "$configuration_root/install-venv-path.txt"

"$pinned_python" - <<'PY'
from importlib.metadata import version
import sys

expected = {
    "pip": "24.0",
    "scikit-build-core": "0.10.7",
    "pybind11": "2.13.6",
    "numpy": "1.21.5",
}
if sys.version_info[:2] != (3, 10):
    raise SystemExit(f"wheel lane requires CPython 3.10, got {sys.version.split()[0]}")
actual = {name: version(name) for name in expected}
if actual != expected:
    raise SystemExit(f"wheel build environment is not pinned: {actual} != {expected}")
PY

echo "==> building exact wheel-profile core"
core_path_map_flags="-ffile-prefix-map=$repository_root/open_lmm=. -fmacro-prefix-map=$repository_root/open_lmm=. -ffile-prefix-map=$core_build=. -fmacro-prefix-map=$core_build=."
cmake -S "$repository_root/open_lmm" -B "$core_build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER="$compiler_c" \
  -DCMAKE_CXX_COMPILER="$compiler_cxx" \
  "-DCMAKE_C_FLAGS=$core_path_map_flags" \
  "-DCMAKE_CXX_FLAGS=$core_path_map_flags" \
  -DUSE_CCACHE=OFF \
  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
  -DBUILD_TESTING=OFF \
  -DOPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=ON \
  -DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=ON \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS=ON \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP=ON \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD=ON \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=ON \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=ON
build_attempt=1
until cmake --build "$core_build" --parallel 2; do
  if [[ $build_attempt -ge 3 ]]; then
    echo "wheel-profile core build failed after $build_attempt attempts" >&2
    exit 1
  fi
  build_attempt=$((build_attempt + 1))
  echo "compiler process failed; retrying unchanged incremental wheel-profile build ($build_attempt/3)" >&2
done
cmake --install "$core_build" --prefix "$core_install"

echo "==> building same-image CPython 3.10 wheel"
"$repository_root/bindings/python/build_local_wheel.sh" \
  "$pinned_python" "$core_install" "$wheel_directory"
wheel_files=("$wheel_directory"/*.whl)
wheel_file=${wheel_files[0]}
sha256sum "$wheel_file" > "$configuration_root/wheel.sha256"

"$pinned_python" -m venv "$install_venv"
grep '^numpy==' "$repository_root/bindings/python/build-constraints.txt" \
  > "$configuration_root/runtime-requirements.txt"
"$install_venv/bin/python" -m pip install --require-hashes --no-deps \
  -r "$configuration_root/runtime-requirements.txt"
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  "$install_venv/bin/python" -m pip install \
  --no-index --no-deps "$wheel_file"
"$install_venv/bin/python" - <<'PY'
from importlib.metadata import version
import sys

if sys.version_info[:2] != (3, 10) or version("numpy") != "1.21.5":
    raise SystemExit("fresh wheel environment does not have the pinned CPython/NumPy contract")
PY

cmake -S "$repository_root/bindings/python" -B "$wheel_test_build" \
  -DCMAKE_PREFIX_PATH="$core_install" \
  -DOPEN_LMM_CORE_PREFIX="$core_install" \
  -DOPEN_LMM_PYTHON_WHEEL_ONLY=ON \
  -DOPEN_LMM_PYTHON_WHEEL_TEST_PYTHON="$install_venv/bin/python" \
  -DOPEN_LMM_PYTHON_WHEEL_FILE="$wheel_file" \
  -DOPEN_LMM_REPOSITORY_ROOT="$repository_root" \
  -DOPEN_LMM_CORE_TEST_BUILD_ROOT="$core_build" \
  -DBUILD_TESTING=ON
ctest --test-dir "$wheel_test_build" --output-on-failure \
  --output-junit "$configuration_root/ctest.xml"
cp "$wheel_test_build/test/open_lmm_python_test_manifest.tsv" \
  "$configuration_root/open_lmm_python_test_manifest.tsv"

# Python is a separate wheel/venv namespace. Its uninstall simulation must not
# remove unknown files from that namespace and must remove its console owner.
site_packages=$("$install_venv/bin/python" -c \
  'import sysconfig; print(sysconfig.get_paths()["purelib"])')
python_owner_inventory="$configuration_root/python-owner-inventory.tsv"
python_owner_body="$configuration_root/python-owner-inventory.body"
: > "$python_owner_body"
python_owned_roots=("$site_packages/open_lmm")
for dist_info in "$site_packages"/open_lmm-*.dist-info; do
  if [[ -d "$dist_info" ]]; then
    python_owned_roots+=("$dist_info")
  fi
done
for owned_root in "${python_owned_roots[@]}"; do
  while IFS= read -r -d '' owned_path; do
    relative_path=${owned_path#"$install_venv"/}
    if [[ -L "$owned_path" ]]; then
      file_kind=symlink
      digest=$(printf 'symlink:%s' "$(readlink "$owned_path")" | sha256sum)
    else
      file_kind=file
      digest=$(sha256sum "$owned_path")
    fi
    digest=${digest%% *}
    printf 'python-venv\t%s\tpython\twheel\t%s\t%s\n' \
      "$relative_path" "$file_kind" "$digest" >> "$python_owner_body"
  done < <(find "$owned_root" \( -type f -o -type l \) -print0)
done
printf 'namespace\trelative_path\tartifact_id\tcomponent\tfile_kind\tsha256\n' \
  > "$python_owner_inventory"
sort "$python_owner_body" >> "$python_owner_inventory"
sha256sum "$python_owner_inventory" \
  > "$configuration_root/python-owner-inventory.sha256"
printf 'must survive wheel uninstall\n' > "$site_packages/open_lmm_user_file"
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  "$install_venv/bin/python" -m pip uninstall --yes open-lmm
if [[ ! -f "$site_packages/open_lmm_user_file" ]]; then
  echo "wheel uninstall removed an unknown Python namespace file" >&2
  exit 1
fi
if [[ -e "$site_packages/open_lmm" ]] ||
   compgen -G "$site_packages/open_lmm-*.dist-info" >/dev/null; then
  echo "wheel uninstall retained the venv package owner" >&2
  exit 1
fi

echo "==> Python wheel verified: $wheel_file"
