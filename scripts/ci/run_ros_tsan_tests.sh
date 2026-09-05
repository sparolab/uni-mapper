#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 BUILD_ROOT CC CXX TSAN_CORE_PREFIX" >&2
  exit 2
fi
build_root=$1
compiler_c=$2
compiler_cxx=$3
core_prefix=$4
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
mkdir -p "$build_root"
build_root=$(cd "$build_root" && pwd)
exec > >(tee "$build_root/ci.log") 2>&1

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
tsan_flags="$compatibility_flags -fsanitize=thread -fno-omit-frame-pointer"
llvm_lib=$(cd "$("$compiler_cxx" -print-resource-dir)/../.." && pwd)
# Some distribution Archer DSOs bind annotation stubs locally. Relinking the
# supplied archive preserves interposition by the executable's TSan runtime.
"$compiler_cxx" -shared -Wl,--whole-archive "$llvm_lib/libarcher_static.a" \
  -Wl,--no-whole-archive -o "$build_root/libarcher.so"
export OMP_TOOL_LIBRARIES="$build_root/libarcher.so"
export ARCHER_OPTIONS=verbose=1
# LLVM's documented Archer setting excludes prebuilt runtime internals.
# Core/GUI retain their strict TSan lane; DDS below is explicitly instrumented.
export TSAN_OPTIONS="halt_on_error=1:ignore_noninstrumented_modules=1:suppressions=$script_dir/tsan.supp"
runner=()
if command -v setarch >/dev/null && setarch "$(uname -m)" -R true 2>/dev/null; then
  runner=(setarch "$(uname -m)" -R)
fi
for variant in clean race; do
  probe_flags=()
  if [[ "$variant" == race ]]; then probe_flags=(-DINJECT_RACE); fi
  "$compiler_c" -g -O1 -fopenmp -fsanitize=thread "${probe_flags[@]}" \
    "$script_dir/tsan_openmp_probe.c" -o "$build_root/probe-$variant"
  status=0
  "${runner[@]}" "$build_root/probe-$variant" > "$build_root/probe-$variant.log" 2>&1 || status=$?
  grep -q 'Archer detected OpenMP application with TSan' "$build_root/probe-$variant.log"
  if [[ "$variant" == clean ]]; then
    test "$status" -eq 0
  else
    test "$status" -ne 0
    grep -q 'WARNING: ThreadSanitizer: data race' "$build_root/probe-$variant.log"
  fi
done

# Isolated Humble DDS overlay, never installed over system/production libraries.
# Pin commits as well as tags so upstream tag movement fails closed.
for dependency in Fast-CDR Fast-DDS; do
  if [[ "$dependency" == Fast-CDR ]]; then
    tag=v1.0.29
    revision=959ff6cc637c940d3deca3d8398920242120daee
    dependency_build="$build_root/cdr-build"
  else
    tag=v2.6.11
    revision=87dd60c8f3e8694481ad0279bd4cc8c645050da3
    dependency_build="$build_root/dds-build"
  fi
  source_root="$build_root/$dependency"
  if [[ ! -d "$source_root" ]]; then
    git clone --depth 1 --branch "$tag" "https://github.com/eProsima/$dependency.git" "$source_root"
  fi
  test "$(git -C "$source_root" rev-parse HEAD)" = "$revision"
  "${runner[@]}" cmake -S "$source_root" -B "$dependency_build" \
    -DCMAKE_BUILD_TYPE=Debug '-DCMAKE_CXX_FLAGS_DEBUG=-O0 -g1' \
    -DCMAKE_C_COMPILER="$compiler_c" -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$tsan_flags -std=c++17" -DCMAKE_C_FLAGS=-fsanitize=thread \
    -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread \
    -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread \
    -DCMAKE_INSTALL_PREFIX="$build_root/dds-install" \
    -DCMAKE_PREFIX_PATH="$build_root/dds-install" \
    -DBUILD_TESTING=OFF -DCOMPILE_EXAMPLES=OFF -DSECURITY=ON \
    -DUSE_THIRDPARTY_SHARED_MUTEX=OFF -U SM_RUN_RESULT \
    -DTHIRDPARTY_Asio=ON -DTHIRDPARTY_TinyXML2=ON
  attempt=1
  until cmake --build "$dependency_build" --parallel "${OPEN_LMM_BUILD_JOBS:-2}"; do
    if [[ "$attempt" -ge 3 ]]; then exit 1; fi
    attempt=$((attempt + 1))
    echo "Retrying unchanged DDS build ($attempt/3), as in the core sanitizer lane."
  done
  cmake --install "$dependency_build"
done

# Match the system RMW's public DDS configuration, including mutex layout.
diff -u /opt/ros/humble/include/fastrtps/fastrtps/config.h \
  "$build_root/dds-install/include/fastrtps/config.h"
for library in fastcdr fastrtps; do
  nm -D --undefined-only "$build_root/dds-install/lib/lib$library.so" |
    grep '__tsan_' > "$build_root/$library-tsan-symbols.txt"
done

export LD_LIBRARY_PATH="$build_root/dds-install/lib:${LD_LIBRARY_PATH:-}"
cmake -S "$repository_root/ros" -B "$build_root/ros-build" \
  -DCMAKE_BUILD_TYPE=Debug '-DCMAKE_CXX_FLAGS_DEBUG=-O0 -g1' \
  -DCMAKE_C_COMPILER="$compiler_c" -DCMAKE_CXX_COMPILER="$compiler_cxx" \
  "-DCMAKE_CXX_FLAGS=$tsan_flags" \
  -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread \
  -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread \
  "-DCMAKE_PREFIX_PATH=$build_root/dds-install;$core_prefix" -DBUILD_TESTING=ON
cmake --build "$build_root/ros-build" --parallel "${OPEN_LMM_BUILD_JOBS:-2}"
# rcl loads the selected RMW dynamically, so inspect that DSO as well.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ldd "$build_root/ros-build/open_lmm_ros_runtime_adapter_graph_tests" \
  /opt/ros/humble/lib/librmw_fastrtps_cpp.so > "$build_root/ros-libraries.txt"
grep -Fq "$build_root/dds-install/lib/libfastrtps.so" "$build_root/ros-libraries.txt"
grep -Fq "$build_root/dds-install/lib/libfastcdr.so" "$build_root/ros-libraries.txt"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-173}
"${runner[@]}" ctest --test-dir "$build_root/ros-build" --output-on-failure \
  --output-junit "$build_root/ctest-ros.xml" -L sanitizer:tsan --no-tests=error \
  --repeat until-fail:10
cp "$build_root/ros-build/open_lmm_test_manifest.tsv" "$build_root/open_lmm_ros_test_manifest.tsv"
