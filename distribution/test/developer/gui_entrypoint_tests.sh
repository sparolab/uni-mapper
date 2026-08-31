#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 REPOSITORY_MAKEFILE MANIFEST_REMOVER" >&2
  exit 2
fi

source_makefile=$1
source_manifest_remover=$2
if [[ ! -f "$source_makefile" ]]; then
  echo "repository Makefile is missing: $source_makefile" >&2
  exit 1
fi

test_root=$(mktemp -d "/tmp/openlmm-gui-entrypoint-XXXXXX")
trap 'rm -rf -- "$test_root"' EXIT
repository="$test_root/repository with spaces"
fake_bin="$test_root/fake-bin"
operation_log="$test_root/operations.log"
mkdir -p "$repository/open_lmm/config" \
  "$repository/applications/gui" "$repository/applications/cli" \
  "$repository/scripts/dev" "$fake_bin"
cp "$source_makefile" "$repository/Makefile"
cp "$source_manifest_remover" \
  "$repository/scripts/dev/remove_install_manifest.sh"
chmod +x "$repository/scripts/dev/remove_install_manifest.sh"
printf '{}\n' > "$repository/open_lmm/config/config.json"
: > "$operation_log"

cat > "$fake_bin/cmake" <<'FAKE_CMAKE'
#!/usr/bin/env bash
set -euo pipefail

printf '%s' "$1" >> "$OPEN_LMM_TEST_LOG"
printf '|%s' "${@:2}" >> "$OPEN_LMM_TEST_LOG"
printf '\n' >> "$OPEN_LMM_TEST_LOG"

if [[ $1 == -S ]]; then
  source_dir=$2
  shift 2
  build_dir=
  install_prefix=
  while [[ $# -gt 0 ]]; do
    case $1 in
      -B)
        build_dir=$2
        shift 2
        ;;
      -DCMAKE_INSTALL_PREFIX=*)
        install_prefix=${1#*=}
        shift
        ;;
      *) shift ;;
    esac
  done
  mkdir -p "$build_dir"
  printf '%s\n' "$source_dir" > "$build_dir/.fake-source"
  printf '%s\n' "$install_prefix" > "$build_dir/.fake-prefix"
elif [[ $1 == --build ]]; then
  test -f "$2/.fake-prefix"
elif [[ $1 == --install ]]; then
  build_dir=$2
  prefix=$(<"$build_dir/.fake-prefix")
  source_dir=$(<"$build_dir/.fake-source")
  mkdir -p "$prefix/bin" "$prefix/lib" \
    "$prefix/share/open_lmm/cmake"
  if [[ $source_dir == */open_lmm ]]; then
    : > "$prefix/lib/libopen_lmm_client.so"
    : > "$prefix/share/open_lmm/cmake/open_lmmConfig.cmake"
    printf '%s\n' \
      "$prefix/lib/libopen_lmm_client.so" \
      "$prefix/share/open_lmm/cmake/open_lmmConfig.cmake" \
      > "$build_dir/install_manifest.txt"
  elif [[ $source_dir == */applications/gui ]]; then
    cat > "$prefix/bin/open_lmm_gui" <<'FAKE_GUI'
#!/usr/bin/env bash
set -euo pipefail
if [[ ${1:-} == --help ]]; then
  exit 0
fi
printf 'gui-run|%s|%s\n' "$1" "$2" >> "$OPEN_LMM_TEST_LOG"
FAKE_GUI
    chmod +x "$prefix/bin/open_lmm_gui"
    : > "$prefix/lib/libopen_lmm_iridescence_gui.so"
    printf '%s\n' \
      "$prefix/bin/open_lmm_gui" \
      "$prefix/lib/libopen_lmm_iridescence_gui.so" \
      > "$build_dir/install_manifest.txt"
  elif [[ $source_dir == */applications/cli ]]; then
    cat > "$prefix/bin/open_lmm_batch" <<'FAKE_CLI'
#!/usr/bin/env bash
set -euo pipefail
if [[ ${1:-} == --help ]]; then
  exit 0
fi
printf 'cli-run|%s\n' "$1" >> "$OPEN_LMM_TEST_LOG"
FAKE_CLI
    chmod +x "$prefix/bin/open_lmm_batch"
    printf '%s\n' "$prefix/bin/open_lmm_batch" \
      > "$build_dir/install_manifest_Tools.txt"
  fi
elif [[ $1 == -E && $2 == remove_directory ]]; then
  rm -rf -- "$3"
else
  echo "unexpected fake cmake invocation: $*" >&2
  exit 1
fi
FAKE_CMAKE
chmod +x "$fake_bin/cmake"

cat > "$fake_bin/cc" <<'FAKE_TOOL'
#!/usr/bin/env bash
exit 0
FAKE_TOOL
cp "$fake_bin/cc" "$fake_bin/c++"
cat > "$fake_bin/ldd" <<'FAKE_LDD'
#!/usr/bin/env bash
exit 0
FAKE_LDD
chmod +x "$fake_bin/cc" "$fake_bin/c++" "$fake_bin/ldd"

run_make() {
  env PATH="$fake_bin:$PATH" OPEN_LMM_TEST_LOG="$operation_log" \
    make --no-print-directory -C "$repository" \
      CC=cc CXX=c++ JOBS=2 "$@"
}

run_make gui-build
mapfile -t first_operations < "$operation_log"
if [[ ${#first_operations[@]} -ne 6 ]]; then
  printf 'unexpected core/GUI operation count: %s\n' \
    "${#first_operations[@]}" >&2
  exit 1
fi
expected_order=(
  "-S|$repository/open_lmm"
  "--build|$repository/build/dev/core"
  "--install|$repository/build/dev/core"
  "-S|$repository/applications/gui"
  "--build|$repository/build/dev/gui"
  "--install|$repository/build/dev/gui")
for index in "${!expected_order[@]}"; do
  if [[ ${first_operations[$index]} != "${expected_order[$index]}"* ]]; then
    printf 'operation %s is out of order: %s\n' \
      "$index" "${first_operations[$index]}" >&2
    exit 1
  fi
done
if [[ ${first_operations[3]} != *"-DCMAKE_PREFIX_PATH=$repository/install/dev"* ||
      ${first_operations[3]} != *"-DOPEN_LMM_GUI_BUILD_IRIDESCENCE=ON"* ||
      ${first_operations[3]} != *"-DBUILD_TESTING=OFF"* ]]; then
  echo "GUI configure did not consume the installed developer core contract" >&2
  exit 1
fi

# Repeating the target must reuse the same generated roots and remain valid.
run_make gui-build
if [[ $(wc -l < "$operation_log") -ne 12 ]]; then
  echo "incremental developer build did not repeat the same six operations" >&2
  exit 1
fi
operation_count=$(wc -l < "$operation_log")
for invalid_case in \
    "CONFIG=$test_root/missing" \
    "CC=open_lmm_missing_compiler" \
    "JOBS=0" \
    "GUI_USE_SYSTEM_IRIDESCENCE=MAYBE"; do
  if run_make gui-build "$invalid_case" >/dev/null 2>&1; then
    echo "gui-build accepted an invalid override: $invalid_case" >&2
    exit 1
  fi
  if [[ $(wc -l < "$operation_log") -ne $operation_count ]]; then
    echo "gui-build invoked CMake before rejecting: $invalid_case" >&2
    exit 1
  fi
done

custom_config="$test_root/config with spaces"
mkdir -p "$custom_config"
printf '{}\n' > "$custom_config/config.json"
DISPLAY=:99 run_make gui CONFIG="$custom_config"
if ! grep -Fq \
    "gui-run|$custom_config|$repository/install/dev/lib/libopen_lmm_iridescence_gui.so" \
    "$operation_log"; then
  echo "GUI run did not preserve config/plugin paths containing spaces" >&2
  exit 1
fi

cli_start=$(wc -l < "$operation_log")
run_make cli CONFIG="$custom_config"
mapfile -t cli_operations \
  < <(tail -n +$((cli_start + 1)) "$operation_log")
if [[ ${#cli_operations[@]} -ne 7 ||
      ${cli_operations[0]} != "-S|$repository/open_lmm"* ||
      ${cli_operations[3]} != "-S|$repository/applications/cli"* ||
      ${cli_operations[3]} != *"-DCMAKE_PREFIX_PATH=$repository/install/dev"* ||
      ${cli_operations[6]} != "cli-run|$custom_config" ]]; then
  echo "canonical CLI did not build from installed core and run in order" >&2
  printf '%s\n' "${cli_operations[@]}" >&2
  exit 1
fi

if env -u DISPLAY -u WAYLAND_DISPLAY PATH="$fake_bin:$PATH" \
    OPEN_LMM_TEST_LOG="$operation_log" make --no-print-directory \
      -C "$repository" CC=cc CXX=c++ gui-run >/dev/null 2>&1; then
  echo "GUI run accepted a headless environment" >&2
  exit 1
fi
if run_make gui-run CONFIG="$test_root/missing" >/dev/null 2>&1; then
  echo "GUI run accepted a missing config directory" >&2
  exit 1
fi

mkdir -p "$repository/install" "$repository/install/dev"
printf 'preserve\n' > "$repository/install/unknown-user-file"
printf 'preserve\n' > "$repository/install/dev/unknown-user-file"
run_make cli-clean
if [[ -e "$repository/install/dev/bin/open_lmm_batch" ||
      ! -e "$repository/install/dev/bin/open_lmm_gui" ||
      ! -e "$repository/install/dev/lib/libopen_lmm_client.so" ]]; then
  echo "cli-clean violated component ownership" >&2
  exit 1
fi
run_make gui-clean
if [[ -e "$repository/install/dev/bin/open_lmm_gui" ||
      -e "$repository/install/dev/lib/libopen_lmm_iridescence_gui.so" ||
      ! -e "$repository/install/dev/lib/libopen_lmm_client.so" ]]; then
  echo "gui-clean violated component ownership" >&2
  exit 1
fi
run_make core-clean
if [[ -e "$repository/install/dev/lib/libopen_lmm_client.so" ||
      -e "$repository/install/dev/share/open_lmm/cmake/open_lmmConfig.cmake" ]]; then
  echo "core-clean retained core-owned paths" >&2
  exit 1
fi
if [[ ! -f "$repository/install/unknown-user-file" ||
      ! -f "$repository/install/dev/unknown-user-file" ]]; then
  echo "component clean removed an unknown file" >&2
  exit 1
fi
if DISPLAY=:99 run_make gui-run >/dev/null 2>&1; then
  echo "gui-run rebuilt or accepted a missing installed artifact" >&2
  exit 1
fi

unsafe_root="$test_root/unsafe-prefix"
mkdir -p "$unsafe_root"
printf 'preserve\n' > "$unsafe_root/owned"
printf '%s\n%s\n' "$unsafe_root/owned" "$test_root/outside" \
  > "$test_root/unsafe-manifest.txt"
if "$source_manifest_remover" "$unsafe_root" \
    "$test_root/unsafe-manifest.txt" fixture >/dev/null 2>&1; then
  echo "manifest remover accepted a path outside its prefix" >&2
  exit 1
fi
if [[ ! -f "$unsafe_root/owned" ]]; then
  echo "manifest remover mutated files before validating every row" >&2
  exit 1
fi

run_make dev-clean
if [[ -e "$repository/build/dev" || -e "$repository/install/dev" ]]; then
  echo "dev-clean retained a generated developer root" >&2
  exit 1
fi
if [[ ! -f "$repository/install/unknown-user-file" ]]; then
  echo "dev-clean removed a sibling unknown file" >&2
  exit 1
fi

for forbidden in bindings/python /ros/ add_subdirectory; do
  if grep -Fq "$forbidden" "$source_makefile"; then
    echo "developer Makefile acquired a forbidden composition edge: $forbidden" >&2
    exit 1
  fi
done

echo "core + GUI + CLI developer entrypoint verified"
