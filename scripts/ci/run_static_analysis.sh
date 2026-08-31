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
report_root="$configuration_root/reports"

reuse_build=${OPEN_LMM_STATIC_REUSE_BUILD:-OFF}
if [[ -e "$configuration_root" && "$reuse_build" != ON ]]; then
  echo "CI configuration path must be clean: $configuration_root" >&2
  exit 1
fi
if [[ ! -f "$manifest" ]]; then
  echo "static source manifest is missing: $manifest" >&2
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
  if [[ "$reuse_build" != ON || ! -f "$build_root/compile_commands.json" ]]; then
    echo "OPEN_LMM_STATIC_SKIP_CONFIGURE requires a reusable compile database" >&2
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
    -DUSE_CCACHE=OFF \
    -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=ON \
    -DOPEN_LMM_BUILD_PYTHON=OFF
fi

compile_commands="$build_root/compile_commands.json"
if [[ ! -f "$compile_commands" ]]; then
  echo "compile database is missing: $compile_commands" >&2
  exit 1
fi
cp "$manifest" "$configuration_root/production_sources.tsv"

python3 - "$compile_commands" "$configuration_root/compiled-sources.txt" <<'PY'
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
