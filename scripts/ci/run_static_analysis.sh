#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 required|advisory CC CXX" >&2
  exit 2
fi

mode=$1
compiler_c=$2
compiler_cxx=$3
case "$mode" in
  required|advisory) ;;
  *)
    echo "mode must be required or advisory, got: $mode" >&2
    exit 2
    ;;
esac

if [[ ! -x "$compiler_c" || ! -x "$compiler_cxx" ]]; then
  echo "compiler executable not found: $compiler_c / $compiler_cxx" >&2
  exit 1
fi

clang_tidy=${OPEN_LMM_CLANG_TIDY:-}
if [[ -z "$clang_tidy" ]]; then
  clang_tidy=$(command -v clang-tidy-15 || true)
fi
if [[ -z "$clang_tidy" || ! -x "$clang_tidy" ]]; then
  echo "clang-tidy-15 is required" >&2
  exit 1
fi
if ! "$clang_tidy" --version | grep -Eq 'version 15\.'; then
  echo "Goal 08 requires clang-tidy 15" >&2
  "$clang_tidy" --version >&2
  exit 1
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
ci_root=${OPEN_LMM_CI_ROOT:-"$repository_root/.ci-build"}
configuration_name="quality-static-$mode"
configuration_root="$ci_root/$configuration_name"
build_root="$configuration_root/build"
manifest="$repository_root/open_lmm/test/quality/static/production_sources.tsv"
cli_manifest="$repository_root/applications/cli/test/quality/production_sources.tsv"
gui_manifest="$repository_root/applications/gui/test/quality/production_sources.tsv"
python_manifest="$repository_root/bindings/python/test/quality/production_sources.tsv"
report_root="$configuration_root/reports"
core_install_root="$configuration_root/core-install"
cli_build_root="$configuration_root/cli-build"
gui_build_root="$configuration_root/gui-build"
python_build_root="$configuration_root/python-build"

reuse_build=${OPEN_LMM_STATIC_REUSE_BUILD:-OFF}
if [[ -e "$configuration_root" && "$reuse_build" != ON ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi
if [[ ! -f "$manifest" ]]; then
  echo "static source manifest is missing: $manifest" >&2
  exit 1
fi
if [[ ! -f "$cli_manifest" ]]; then
  echo "CLI static source manifest is missing: $cli_manifest" >&2
  exit 1
fi
if [[ ! -f "$gui_manifest" ]]; then
  echo "GUI static source manifest is missing: $gui_manifest" >&2
  exit 1
fi
if [[ ! -f "$python_manifest" ]]; then
  echo "Python static source manifest is missing: $python_manifest" >&2
  exit 1
fi

mkdir -p "$report_root"
exec > >(tee "$configuration_root/ci.log") 2>&1

git -C "$repository_root" rev-parse HEAD > "$configuration_root/git-head.txt"
git -C "$repository_root" status --short > "$configuration_root/git-status.txt"
"$compiler_cxx" --version > "$configuration_root/compiler-version.txt"
"$clang_tidy" --version > "$configuration_root/clang-tidy-version.txt"

cxx_compatibility_flags=""
if [[ "$(basename "$compiler_cxx")" == clang++* ]]; then
  cxx_compatibility_flags="-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward"
fi

skip_configure=${OPEN_LMM_STATIC_SKIP_CONFIGURE:-OFF}
if [[ "$skip_configure" == ON ]]; then
  if [[ "$reuse_build" != ON ||
        ! -f "$build_root/compile_commands.json" ||
        ! -f "$cli_build_root/compile_commands.json" ||
        ! -f "$gui_build_root/compile_commands.json" ||
        ! -f "$python_build_root/compile_commands.json" ]]; then
    echo "OPEN_LMM_STATIC_SKIP_CONFIGURE requires reusable core, CLI, GUI, and Python compile databases" >&2
    exit 1
  fi
else
  cmake -S "$repository_root/open_lmm" -B "$build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
    -DUSE_CCACHE=OFF
  build_attempt=1
  until cmake --build "$build_root" --parallel 2; do
    if [[ $build_attempt -ge 3 ]]; then
      echo "static-analysis build failed after $build_attempt attempts" >&2
      exit 1
    fi
    build_attempt=$((build_attempt + 1))
    echo "compiler process failed; retrying unchanged incremental static-analysis build ($build_attempt/3)" >&2
  done
  cmake --install "$build_root" --prefix "$core_install_root"
  cmake -S "$repository_root/applications/cli" -B "$cli_build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_PREFIX_PATH="$core_install_root" \
    -DOPEN_LMM_CLI_WARNINGS_AS_ERRORS=ON \
    -DBUILD_TESTING=OFF
  cmake -S "$repository_root/applications/gui" -B "$gui_build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER="$compiler_c" \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_PREFIX_PATH="$core_install_root" \
    -DOPEN_LMM_GUI_BUILD_IRIDESCENCE=ON \
    -DOPEN_LMM_GUI_WARNINGS_AS_ERRORS=ON \
    -DBUILD_TESTING=OFF
  cmake --build "$gui_build_root" --parallel 2
  cmake -S "$repository_root/bindings/python" -B "$python_build_root" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_COMPILER="$compiler_cxx" \
    "-DCMAKE_CXX_FLAGS=$cxx_compatibility_flags" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_PREFIX_PATH="$core_install_root" \
    -DOPEN_LMM_CORE_PREFIX="$core_install_root" \
    -DOPEN_LMM_PYTHON_WARNINGS_AS_ERRORS=ON \
    -DBUILD_TESTING=OFF
  cmake --build "$python_build_root" --parallel 2
fi

compile_commands="$build_root/compile_commands.json"
cli_compile_commands="$cli_build_root/compile_commands.json"
gui_compile_commands="$gui_build_root/compile_commands.json"
python_compile_commands="$python_build_root/compile_commands.json"
if [[ ! -f "$compile_commands" || ! -f "$cli_compile_commands" ||
      ! -f "$gui_compile_commands" ||
      ! -f "$python_compile_commands" ]]; then
  echo "core, CLI, GUI, or Python compile database is missing" >&2
  exit 1
fi
cp "$manifest" "$configuration_root/production_sources.tsv"
cp "$cli_manifest" "$configuration_root/cli-production_sources.tsv"
cp "$gui_manifest" "$configuration_root/gui-production_sources.tsv"
cp "$python_manifest" "$configuration_root/python-production_sources.tsv"

python3 - "$compile_commands" "$configuration_root/compiled-sources.txt" <<'PY'
import json
import pathlib
import sys

database = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
sources = sorted({str(pathlib.Path(entry["file"]).resolve()) for entry in database})
pathlib.Path(sys.argv[2]).write_text("\n".join(sources) + "\n", encoding="utf-8")
PY

python3 - "$cli_compile_commands" "$configuration_root/cli-compiled-sources.txt" <<'PY'
import json
import pathlib
import sys

database = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
sources = sorted({str(pathlib.Path(entry["file"]).resolve()) for entry in database})
pathlib.Path(sys.argv[2]).write_text("\n".join(sources) + "\n", encoding="utf-8")
PY

python3 - "$gui_compile_commands" "$configuration_root/gui-compiled-sources.txt" <<'PY'
import json
import pathlib
import sys

database = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
sources = sorted({str(pathlib.Path(entry["file"]).resolve()) for entry in database})
pathlib.Path(sys.argv[2]).write_text("\n".join(sources) + "\n", encoding="utf-8")
PY

python3 - "$python_compile_commands" "$configuration_root/python-compiled-sources.txt" <<'PY'
import json
import pathlib
import sys

database = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
sources = sorted({str(pathlib.Path(entry["file"]).resolve()) for entry in database})
pathlib.Path(sys.argv[2]).write_text("\n".join(sources) + "\n", encoding="utf-8")
PY

required_checks='-*,bugprone-infinite-loop,bugprone-misplaced-widening-cast,bugprone-sizeof-expression,bugprone-string-integer-assignment,bugprone-suspicious-memory-comparison,bugprone-suspicious-memset-usage,bugprone-undefined-memory-manipulation,bugprone-use-after-move,performance-no-automatic-move,modernize-use-override'
broad_checks='-*,bugprone-*,performance-*,portability-*,modernize-use-override'
checks=$required_checks
warnings_as_errors='bugprone-infinite-loop,bugprone-misplaced-widening-cast,bugprone-sizeof-expression,bugprone-string-integer-assignment,bugprone-suspicious-memory-comparison,bugprone-suspicious-memset-usage,bugprone-undefined-memory-manipulation,bugprone-use-after-move,performance-no-automatic-move,modernize-use-override'
if [[ "$mode" == advisory ]]; then
  checks=$broad_checks
  warnings_as_errors=''
fi

: > "$configuration_root/analyzed-sources.txt"
: > "$configuration_root/skipped-sources.tsv"
: > "$report_root/clang-tidy.txt"
finding_count=0
tool_failure=0
source_index=0
source_total=$(awk -F '\t' 'NR > 1 && $2 == "required" {count++} END {print count}' \
  "$manifest")
cli_source_total=$(awk -F '\t' 'NR > 1 && $2 == "required" {count++} END {print count}' \
  "$cli_manifest")
gui_source_total=$(awk -F '\t' 'NR > 1 {count++} END {print count}' \
  "$gui_manifest")
python_source_total=$(awk -F '\t' 'NR > 1 && $2 == "required" {count++} END {print count}' \
  "$python_manifest")
source_total=$((source_total + cli_source_total + gui_source_total + python_source_total))

while IFS=$'\t' read -r relative_source scope; do
  if [[ "$relative_source" == "path" ]]; then
    continue
  fi
  source_path="$repository_root/open_lmm/$relative_source"
  if [[ ! -f "$source_path" ]]; then
    echo "manifest source is missing: $relative_source" >&2
    exit 1
  fi
  if ! grep -Fxq "$source_path" "$configuration_root/compiled-sources.txt"; then
    if [[ "$scope" == optional-* ]]; then
      printf '%s\t%s\tnot-in-configured-target-graph\n' \
        "$relative_source" "$scope" >> "$configuration_root/skipped-sources.tsv"
      continue
    fi
    echo "required source is absent from compile database: $relative_source" >&2
    exit 1
  fi

  source_index=$((source_index + 1))
  echo "[$source_index/$source_total] clang-tidy: $relative_source"
  printf '%s\n' "$relative_source" >> "$configuration_root/analyzed-sources.txt"
  source_log="$report_root/${relative_source//\//__}.txt"
  tidy_args=(
    -p "$build_root"
    --quiet
    "--checks=$checks")
  if [[ -n "$warnings_as_errors" ]]; then
    tidy_args+=("--warnings-as-errors=$warnings_as_errors")
  fi
  set +e
  "$clang_tidy" "${tidy_args[@]}" "$source_path" > "$source_log" 2>&1
  tidy_status=$?
  set -e
  if [[ -s "$source_log" ]]; then
    printf '===== %s =====\n' "$relative_source" >> "$report_root/clang-tidy.txt"
    sed "s#${repository_root}/##g" "$source_log" \
      >> "$report_root/clang-tidy.txt"
  fi
  source_findings=$(grep -Ec '(^|: )(warning|error):' "$source_log" || true)
  finding_count=$((finding_count + source_findings))
  if [[ $tidy_status -ne 0 && $source_findings -eq 0 ]]; then
    tool_failure=1
  fi
done < "$manifest"

while IFS=$'\t' read -r relative_source scope; do
  if [[ "$relative_source" == "path" ]]; then
    continue
  fi
  source_path="$repository_root/applications/cli/$relative_source"
  if [[ ! -f "$source_path" ]]; then
    echo "CLI manifest source is missing: $relative_source" >&2
    exit 1
  fi
  if ! grep -Fxq "$source_path" "$configuration_root/cli-compiled-sources.txt"; then
    echo "required CLI source is absent from compile database: $relative_source" >&2
    exit 1
  fi

  source_index=$((source_index + 1))
  echo "[$source_index/$source_total] clang-tidy: applications/cli/$relative_source"
  printf 'applications/cli/%s\n' "$relative_source" \
    >> "$configuration_root/analyzed-sources.txt"
  source_log="$report_root/applications__cli__${relative_source//\//__}.txt"
  tidy_args=(
    -p "$cli_build_root"
    --quiet
    "--checks=$checks")
  if [[ -n "$warnings_as_errors" ]]; then
    tidy_args+=("--warnings-as-errors=$warnings_as_errors")
  fi
  set +e
  "$clang_tidy" "${tidy_args[@]}" "$source_path" > "$source_log" 2>&1
  tidy_status=$?
  set -e
  if [[ -s "$source_log" ]]; then
    printf '===== applications/cli/%s =====\n' "$relative_source" \
      >> "$report_root/clang-tidy.txt"
    sed "s#${repository_root}/##g" "$source_log" \
      >> "$report_root/clang-tidy.txt"
  fi
  source_findings=$(grep -Ec '(^|: )(warning|error):' "$source_log" || true)
  finding_count=$((finding_count + source_findings))
  if [[ $tidy_status -ne 0 && $source_findings -eq 0 ]]; then
    tool_failure=1
  fi
done < "$cli_manifest"

while IFS=$'\t' read -r relative_source scope; do
  if [[ "$relative_source" == "path" ]]; then
    continue
  fi
  source_path="$repository_root/applications/gui/$relative_source"
  if [[ ! -f "$source_path" ]]; then
    echo "GUI manifest source is missing: $relative_source" >&2
    exit 1
  fi
  if ! grep -Fxq "$source_path" "$configuration_root/gui-compiled-sources.txt"; then
    echo "required GUI source is absent from compile database: $relative_source" >&2
    exit 1
  fi
  if grep -Fxq "$source_path" "$configuration_root/compiled-sources.txt"; then
    echo "GUI source has a duplicate core compile owner: $relative_source" >&2
    exit 1
  fi

  source_index=$((source_index + 1))
  echo "[$source_index/$source_total] clang-tidy: applications/gui/$relative_source"
  printf 'applications/gui/%s\n' "$relative_source" \
    >> "$configuration_root/analyzed-sources.txt"
  source_log="$report_root/applications__gui__${relative_source//\//__}.txt"
  tidy_args=(
    -p "$gui_build_root"
    --quiet
    "--checks=$checks")
  if [[ -n "$warnings_as_errors" ]]; then
    tidy_args+=("--warnings-as-errors=$warnings_as_errors")
  fi
  set +e
  "$clang_tidy" "${tidy_args[@]}" "$source_path" > "$source_log" 2>&1
  tidy_status=$?
  set -e
  if [[ -s "$source_log" ]]; then
    printf '===== applications/gui/%s =====\n' "$relative_source" \
      >> "$report_root/clang-tidy.txt"
    sed "s#${repository_root}/##g" "$source_log" \
      >> "$report_root/clang-tidy.txt"
  fi
  source_findings=$(grep -Ec '(^|: )(warning|error):' "$source_log" || true)
  finding_count=$((finding_count + source_findings))
  if [[ $tidy_status -ne 0 && $source_findings -eq 0 ]]; then
    tool_failure=1
  fi
done < "$gui_manifest"

while IFS=$'\t' read -r relative_source scope; do
  if [[ "$relative_source" == "path" ]]; then
    continue
  fi
  source_path="$repository_root/bindings/python/$relative_source"
  if [[ ! -f "$source_path" ]]; then
    echo "Python manifest source is missing: $relative_source" >&2
    exit 1
  fi
  if ! grep -Fxq "$source_path" "$configuration_root/python-compiled-sources.txt"; then
    echo "required Python source is absent from compile database: $relative_source" >&2
    exit 1
  fi
  if grep -Fxq "$source_path" "$configuration_root/compiled-sources.txt" ||
     grep -Fxq "$source_path" "$configuration_root/cli-compiled-sources.txt"; then
    echo "Python source has a duplicate compile owner: $relative_source" >&2
    exit 1
  fi

  source_index=$((source_index + 1))
  echo "[$source_index/$source_total] clang-tidy: bindings/python/$relative_source"
  printf 'bindings/python/%s\n' "$relative_source" \
    >> "$configuration_root/analyzed-sources.txt"
  source_log="$report_root/bindings__python__${relative_source//\//__}.txt"
  tidy_args=(
    -p "$python_build_root"
    --quiet
    "--checks=$checks")
  if [[ -n "$warnings_as_errors" ]]; then
    tidy_args+=("--warnings-as-errors=$warnings_as_errors")
  fi
  set +e
  "$clang_tidy" "${tidy_args[@]}" "$source_path" > "$source_log" 2>&1
  tidy_status=$?
  set -e
  if [[ -s "$source_log" ]]; then
    printf '===== bindings/python/%s =====\n' "$relative_source" \
      >> "$report_root/clang-tidy.txt"
    sed "s#${repository_root}/##g" "$source_log" \
      >> "$report_root/clang-tidy.txt"
  fi
  source_findings=$(grep -Ec '(^|: )(warning|error):' "$source_log" || true)
  finding_count=$((finding_count + source_findings))
  if [[ $tidy_status -ne 0 && $source_findings -eq 0 ]]; then
    tool_failure=1
  fi
done < "$python_manifest"

printf 'mode\t%s\nfindings\t%s\ntool_failure\t%s\n' \
  "$mode" "$finding_count" "$tool_failure" \
  > "$configuration_root/summary.tsv"

if [[ $tool_failure -ne 0 ]]; then
  echo "clang-tidy reported a tool/error failure; see $report_root" >&2
  exit 1
fi
if [[ "$mode" == required && $finding_count -ne 0 ]]; then
  echo "required clang-tidy findings: $finding_count" >&2
  exit 1
fi

echo "==> static analysis verified: $mode ($finding_count findings)"
