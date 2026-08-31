#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "usage: $0 --runner PATH --case FILE --data-root DIR --config-root DIR" \
       "--output-root DIR --evidence-root DIR --container-digest SHA256" \
       "[--baseline FILE --compare PATH]" >&2
}

runner=""
compare=""
case_manifest=""
data_root=""
config_root=""
output_root=""
evidence_root=""
container_digest=""
baseline=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runner) runner=$2; shift 2 ;;
    --compare) compare=$2; shift 2 ;;
    --case) case_manifest=$2; shift 2 ;;
    --data-root) data_root=$2; shift 2 ;;
    --config-root) config_root=$2; shift 2 ;;
    --output-root) output_root=$2; shift 2 ;;
    --evidence-root) evidence_root=$2; shift 2 ;;
    --container-digest) container_digest=$2; shift 2 ;;
    --baseline) baseline=$2; shift 2 ;;
    --help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

if [[ -z "$runner" || -z "$case_manifest" || -z "$data_root" ||
      -z "$config_root" || -z "$output_root" || -z "$evidence_root" ||
      ! "$container_digest" =~ ^(sha256:)?[0-9a-f]{64}$ ]]; then
  usage
  exit 2
fi
if [[ -n "$baseline" && -z "$compare" ]]; then
  echo "--compare is required with --baseline" >&2
  exit 2
fi
if [[ ! -x "$runner" || ! -f "$case_manifest" || ! -d "$data_root" ||
      ! -d "$config_root" ]]; then
  echo "runner or replay input is unavailable" >&2
  exit 77
fi
if [[ -e "$output_root" || -e "$evidence_root" ]]; then
  echo "output and evidence paths must not already exist" >&2
  exit 2
fi
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
git_commit=$(git -C "$repository_root" rev-parse HEAD)
git_state=()
if [[ -n "$(git -C "$repository_root" status --porcelain)" ]]; then
  git_state+=(--git-dirty)
fi

mkdir -p "$evidence_root"
report="$evidence_root/replay-report.json"

"$runner" \
  --case "$case_manifest" \
  --data-root "$data_root" \
  --config-root "$config_root" \
  --output-root "$output_root" \
  --report "$report" \
  --git-commit "$git_commit" \
  --container-digest "$container_digest" \
  "${git_state[@]}"

if [[ -n "$baseline" ]]; then
  if [[ ! -x "$compare" || ! -f "$baseline" ]]; then
    echo "comparator or baseline is unavailable" >&2
    exit 77
  fi
  "$compare" --baseline "$baseline" --report "$report" \
    --diff "$evidence_root/replay-diff.json"
fi
