#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 INSTALL_PREFIX REPORT_ROOT" >&2
  exit 2
fi

install_prefix=$(cd "$1" && pwd)
report_root=$2
library_root="$install_prefix/lib"
if [[ ! -d "$library_root" ]]; then
  echo "installed library directory is missing: $library_root" >&2
  exit 1
fi

for tool in readelf nm c++filt sha256sum; do
  if ! command -v "$tool" >/dev/null; then
    echo "required symbol inspection tool is missing: $tool" >&2
    exit 1
  fi
done

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
project_version=$(sed -n \
  's/^project(open_lmm VERSION \([0-9][0-9.]*\).*/\1/p' \
  "$repository_root/open_lmm/CMakeLists.txt")
project_major=${project_version%%.*}
if [[ -z "$project_version" || -z "$project_major" ]]; then
  echo "could not read the OpenLMM project version" >&2
  exit 1
fi

mkdir -p "$report_root/libraries"
summary="$report_root/summary.tsv"
printf 'library\tsoname\tdynamic_defined_symbols\tprivate_path_hits\tplugin_entry\n' \
  > "$summary"

mapfile -t libraries < <(
  find "$library_root" -maxdepth 1 -type f \
    \( -name 'libopen_lmm_*.so.*' -o -name 'libcreate_*.so.*' \) | sort)
if [[ ${#libraries[@]} -eq 0 ]]; then
  echo "no installed OpenLMM shared libraries found in $library_root" >&2
  exit 1
fi

required_runtime=(
  open_lmm_contracts
  open_lmm_client
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_data_loader
  open_lmm_descriptor
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
  open_lmm_map_server)
for runtime in "${required_runtime[@]}"; do
  expected="$library_root/lib${runtime}.so.${project_version}"
  if [[ ! -f "$expected" ]]; then
    echo "required installed runtime library is missing: $expected" >&2
    exit 1
  fi
done

plugin_count=0
for library in "${libraries[@]}"; do
  base=$(basename "$library")
  report="$report_root/libraries/$base.txt"
  soname=$(readelf -d "$library" | sed -n 's/.*SONAME.*\[\(.*\)\].*/\1/p')
  expected_soname=${base%.${project_version}}.$project_major
  if [[ -z "$soname" || "$soname" != "$expected_soname" ]]; then
    echo "unexpected SONAME for $library: ${soname:-missing}" >&2
    exit 1
  fi

  nm -D --defined-only "$library" > "$report_root/libraries/$base.raw.txt"
  nm -D --defined-only -C "$library" > "$report_root/libraries/$base.demangled.txt"
  symbol_count=$(awk 'NF >= 3 { count++ } END { print count + 0 }' \
    "$report_root/libraries/$base.raw.txt")
  private_path_hits=$(grep -Ec \
    'open_lmm/src/|/src/(runtime|state|config|domain|plugins)/|src/(runtime|state|config|domain|plugins)/' \
    "$report_root/libraries/$base.demangled.txt" || true)
  if [[ $private_path_hits -ne 0 ]]; then
    echo "private source path leaked into exported symbols: $library" >&2
    exit 1
  fi

  plugin_entry=not-applicable
  if [[ "$base" == libcreate_*.so.* ]]; then
    plugin_count=$((plugin_count + 1))
    plugin_entry=present
    if ! awk 'NF >= 3 && $3 == "open_lmm_plugin_entry" { found = 1 }
        END { exit !found }' "$report_root/libraries/$base.raw.txt"; then
      echo "plugin ABI entry symbol is missing: $library" >&2
      exit 1
    fi
  fi

  {
    printf 'path\t%s\n' "$library"
    printf 'sha256\t%s\n' "$(sha256sum "$library" | cut -d ' ' -f1)"
    printf 'soname\t%s\n' "$soname"
    printf 'dynamic_defined_symbols\t%s\n' "$symbol_count"
    printf 'demangled_sample\n'
    awk 'NF >= 3 { print; shown++ } shown == 20 { exit }' \
      "$report_root/libraries/$base.demangled.txt"
  } > "$report"
  printf '%s\t%s\t%s\t%s\t%s\n' \
    "$base" "$soname" "$symbol_count" "$private_path_hits" "$plugin_entry" \
    >> "$summary"
done

if [[ $plugin_count -eq 0 ]]; then
  echo "no installed plugin entry DSO was available for ABI symbol inspection" >&2
  exit 1
fi

readelf --version > "$report_root/readelf-version.txt"
nm --version > "$report_root/nm-version.txt"
git -C "$repository_root" rev-parse HEAD > "$report_root/git-head.txt"
git -C "$repository_root" status --short > "$report_root/git-status.txt"
printf 'project_version\t%s\nlibraries\t%s\nplugins\t%s\n' \
  "$project_version" "${#libraries[@]}" "$plugin_count" \
  > "$report_root/identity.tsv"
echo "==> symbol visibility verified: ${#libraries[@]} libraries, $plugin_count plugins"
