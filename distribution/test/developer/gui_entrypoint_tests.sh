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
  "$repository/bindings/python/examples" \
  "$repository/ros" "$repository/scripts/dev" "$fake_bin"
cp "$source_makefile" "$repository/Makefile"
cp "$source_manifest_remover" \
  "$repository/scripts/dev/remove_install_manifest.sh"
chmod +x "$repository/scripts/dev/remove_install_manifest.sh"
printf '{}\n' > "$repository/open_lmm/config/config.json"
printf '%s\n' 'pip==24.0' \
  > "$repository/bindings/python/build-constraints.txt"
printf '%s\n' 'print("fake Python example")' \
  > "$repository/bindings/python/examples/basic_runtime.py"
: > "$operation_log"

cat > "$repository/bindings/python/build_local_wheel.sh" <<'FAKE_WHEEL'
#!/usr/bin/env bash
set -euo pipefail
printf 'wheel-build|%s|%s|%s\n' "$1" "$2" "$3" \
  >> "$OPEN_LMM_TEST_LOG"
mkdir -p "$3"
: > "$3/open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl"
FAKE_WHEEL
chmod +x "$repository/bindings/python/build_local_wheel.sh"

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

cat > "$fake_bin/python3.10" <<'FAKE_PYTHON'
#!/usr/bin/env bash
set -euo pipefail
printf 'python|%s\n' "$*" >> "$OPEN_LMM_TEST_LOG"
if [[ ${1:-} == -c ]]; then
  if [[ ${2:-} == *sys.version_info* ]]; then
    printf '%s\n' '3.10'
  fi
  exit 0
fi
if [[ ${1:-} == -m && ${2:-} == venv ]]; then
  target=${@: -1}
  mkdir -p "$target/bin"
  cp "$0" "$target/bin/python"
  chmod +x "$target/bin/python"
  exit 0
fi
if [[ ${1:-} == -m && ${2:-} == pip ]]; then
  exit 0
fi
exit 0
FAKE_PYTHON
chmod +x "$fake_bin/python3.10"

fake_ros_setup="$test_root/ros setup.bash"
printf '%s\n' 'export OPEN_LMM_FAKE_ROS_SETUP=1' > "$fake_ros_setup"

cat > "$fake_bin/colcon" <<'FAKE_COLCON'
#!/usr/bin/env bash
set -euo pipefail

printf 'colcon' >> "$OPEN_LMM_TEST_LOG"
printf '|%s' "$@" >> "$OPEN_LMM_TEST_LOG"
printf '\n' >> "$OPEN_LMM_TEST_LOG"

build_base=
install_base=
while [[ $# -gt 0 ]]; do
  case $1 in
    --build-base)
      build_base=$2
      shift 2
      ;;
    --install-base)
      install_base=$2
      shift 2
      ;;
    *) shift ;;
  esac
done
test -n "$build_base"
test -n "$install_base"
mkdir -p "$build_base/open_lmm_ros" \
  "$install_base/open_lmm_ros/lib/open_lmm_ros"
cat > "$install_base/open_lmm_ros/lib/open_lmm_ros/open_lmm_rosnode" <<'FAKE_NODE'
#!/usr/bin/env bash
exit 0
FAKE_NODE
chmod +x "$install_base/open_lmm_ros/lib/open_lmm_ros/open_lmm_rosnode"
printf '%s\n' 'export OPEN_LMM_FAKE_ROS_OVERLAY=1' > "$install_base/setup.bash"
FAKE_COLCON

cat > "$fake_bin/ros2" <<'FAKE_ROS2'
#!/usr/bin/env bash
set -euo pipefail
printf 'ros2' >> "$OPEN_LMM_TEST_LOG"
printf '|%s' "$@" >> "$OPEN_LMM_TEST_LOG"
printf '|LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-}" >> "$OPEN_LMM_TEST_LOG"
FAKE_ROS2

cat > "$fake_bin/rviz2" <<'FAKE_RVIZ2'
#!/usr/bin/env bash
exit 0
FAKE_RVIZ2
chmod +x "$fake_bin/colcon" "$fake_bin/ros2" "$fake_bin/rviz2"

run_make() {
  env PATH="$fake_bin:$PATH" OPEN_LMM_TEST_LOG="$operation_log" \
    make --no-print-directory -C "$repository" \
      CC=cc CXX=c++ JOBS=2 ROS_SYSTEM_SETUP="$fake_ros_setup" "$@"
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

ros_start=$(wc -l < "$operation_log")
run_make ros-build
mapfile -t ros_build_operations \
  < <(tail -n +$((ros_start + 1)) "$operation_log")
if [[ ${#ros_build_operations[@]} -ne 4 ||
      ${ros_build_operations[0]} != "-S|$repository/open_lmm"* ||
      ${ros_build_operations[3]} != "colcon|--log-base|$repository/build/dev/ros-log|build"* ||
      ${ros_build_operations[3]} != *"|--base-paths|$repository/ros|"* ||
      ${ros_build_operations[3]} != *"|--build-base|$repository/build/dev/ros|"* ||
      ${ros_build_operations[3]} != *"|--install-base|$repository/install/dev/ros-overlay|"* ||
      ${ros_build_operations[3]} != *"|-DCMAKE_PREFIX_PATH=$repository/install/dev|"* ]]; then
  echo "ROS build did not consume only the installed core and ROS leaf" >&2
  printf '%s\n' "${ros_build_operations[@]}" >&2
  exit 1
fi
ros_operation_count=$(wc -l < "$operation_log")
if run_make ros-build ROS_SYSTEM_SETUP="$test_root/missing-ros-setup.bash" \
    >/dev/null 2>&1; then
  echo "ros-build accepted a missing ROS setup" >&2
  exit 1
fi
if [[ $(wc -l < "$operation_log") -ne $ros_operation_count ]]; then
  echo "ros-build invoked a build before rejecting a missing ROS setup" >&2
  exit 1
fi

ros_start=$(wc -l < "$operation_log")
run_make ros CONFIG="$custom_config" ROS_USE_RVIZ=false
mapfile -t ros_operations \
  < <(tail -n +$((ros_start + 1)) "$operation_log")
if [[ ${#ros_operations[@]} -ne 5 ||
      ${ros_operations[3]} != "colcon|"* ||
      ${ros_operations[4]} != "ros2|launch|open_lmm_ros|open_lmm_rviz.launch.py"* ||
      ${ros_operations[4]} != *"|config_path:=$custom_config|"* ||
      ${ros_operations[4]} != *"|use_rviz:=false|"* ||
      ${ros_operations[4]} != *"LD_LIBRARY_PATH=$repository/install/dev/lib"* ]]; then
  echo "canonical ROS target did not build and launch the isolated overlay" >&2
  printf '%s\n' "${ros_operations[@]}" >&2
  exit 1
fi

ros_run_start=$(wc -l < "$operation_log")
run_make ros-run CONFIG="$custom_config" ROS_USE_RVIZ=true
if [[ $(wc -l < "$operation_log") -ne $((ros_run_start + 1)) ||
      $(tail -n 1 "$operation_log") != *"|use_rviz:=true|"* ]]; then
  echo "ros-run rebuilt artifacts or failed to enable RViz" >&2
  exit 1
fi
if run_make ros-run ROS_USE_RVIZ=maybe >/dev/null 2>&1; then
  echo "ros-run accepted an invalid ROS_USE_RVIZ value" >&2
  exit 1
fi
if run_make ros-run CONFIG="$test_root/missing" >/dev/null 2>&1; then
  echo "ros-run accepted a missing config directory" >&2
  exit 1
fi

python_start=$(wc -l < "$operation_log")
run_make python-build PYTHON=python3.10 PYTHON_JOBS=2
mapfile -t python_build_operations \
  < <(tail -n +$((python_start + 1)) "$operation_log")
python_configure=
for operation in "${python_build_operations[@]}"; do
  if [[ $operation == "-S|$repository/open_lmm"* &&
        $operation == *"-B|$repository/build/dev/python-core"* ]]; then
    python_configure=$operation
  fi
done
if [[ -z $python_configure ||
      $python_configure != *"-DCMAKE_INSTALL_PREFIX=$repository/install/dev/python-core"* ||
      $python_configure != *"-DOPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=ON"* ||
      $python_configure != *"-DOPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=ON"* ||
      $python_configure != *"-DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=OFF"* ]]; then
  echo "Python build did not use the exact isolated wheel-core profile" >&2
  printf '%s\n' "${python_build_operations[@]}" >&2
  exit 1
fi
if ! printf '%s\n' "${python_build_operations[@]}" | grep -Fq \
    "wheel-build|$repository/build/dev/python-build-venv/bin/python|$repository/install/dev/python-core|$repository/build/dev/python-wheel"; then
  echo "Python build did not compose the reviewed local wheel helper" >&2
  exit 1
fi

run_make python-install PYTHON=python3.10
if [[ ! -x "$repository/install/dev/python-venv/bin/python" ]]; then
  echo "Python install did not create its isolated developer venv" >&2
  exit 1
fi
python_run_start=$(wc -l < "$operation_log")
run_make python-run CONFIG="$custom_config"
expected_python_run="python|$repository/bindings/python/examples/basic_runtime.py $custom_config"
if [[ $(wc -l < "$operation_log") -ne $((python_run_start + 1)) ||
      $(tail -n 1 "$operation_log") != "$expected_python_run" ]]; then
  echo "python-run rebuilt artifacts or failed to preserve the config path" >&2
  exit 1
fi
if run_make python-run CONFIG="$test_root/missing" >/dev/null 2>&1; then
  echo "python-run accepted a missing config directory" >&2
  exit 1
fi
if run_make python-build PYTHON_JOBS=0 >/dev/null 2>&1; then
  echo "python-build accepted an invalid PYTHON_JOBS value" >&2
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
run_make ros-clean
if [[ -e "$repository/build/dev/ros" ||
      -e "$repository/build/dev/ros-log" ||
      -e "$repository/install/dev/ros-overlay" ||
      ! -e "$repository/install/dev/lib/libopen_lmm_client.so" ]]; then
  echo "ros-clean escaped its generated overlay ownership" >&2
  exit 1
fi
run_make python-clean
if [[ -e "$repository/build/dev/python-core" ||
      -e "$repository/build/dev/python-build-venv" ||
      -e "$repository/build/dev/python-wheel" ||
      -e "$repository/install/dev/python-core" ||
      -e "$repository/install/dev/python-venv" ||
      ! -e "$repository/install/dev/lib/libopen_lmm_client.so" ]]; then
  echo "python-clean escaped its isolated developer namespace" >&2
  exit 1
fi
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

for forbidden in add_subdirectory; do
  if grep -Fq "$forbidden" "$source_makefile"; then
    echo "developer Makefile acquired a forbidden composition edge: $forbidden" >&2
    exit 1
  fi
done

echo "core + GUI + CLI + Python + ROS developer entrypoint verified"
