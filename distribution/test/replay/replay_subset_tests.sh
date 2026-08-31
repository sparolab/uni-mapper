#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 || ! -x "$1" ]]; then
  echo "usage: $0 BUILD_REPLAY_SUBSET" >&2
  exit 2
fi

builder=$1
fixture=$(mktemp -d)
cleanup() { rm -rf -- "$fixture"; }
trap cleanup EXIT

mkdir -p "$fixture/source/a/Scans" "$fixture/source/b/Scans"
for agent in a b; do
  printf '1 0 0 0 0 1 0 0 0 0 1 0\n' > \
    "$fixture/source/$agent/optimized_poses.txt"
  printf 'pcd-%s\n' "$agent" > "$fixture/source/$agent/Scans/000000.pcd"
done

python3 - "$fixture/source.json" <<'PY'
import json
import sys
json.dump({
    "schema_version": 1,
    "dataset_id": "fixture-source-v1",
    "source": "generated",
    "owner": "test",
    "license": "CC0-1.0",
    "redistributable": True,
    "agents": [
        {"id": "a", "source_directory": "a"},
        {"id": "b", "source_directory": "b"},
    ],
}, open(sys.argv[1], "w", encoding="utf-8"), indent=2)
PY
python3 - "$fixture/selection.json" <<'PY'
import json
import sys
json.dump({
    "schema_version": 1,
    "tier": "tiny",
    "tier_id": "fixture-tiny-v1",
    "source_dataset_id": "fixture-source-v1",
    "generator_version": 1,
    "agents": [
        {"id": "a", "source_directory": "a", "frames": [0]},
        {"id": "b", "source_directory": "b", "frames": [0]},
    ],
}, open(sys.argv[1], "w", encoding="utf-8"), indent=2)
PY
python3 - "$fixture/failure-selection.json" <<'PY'
import json
import sys
json.dump({
    "schema_version": 1,
    "tier": "failure",
    "tier_id": "fixture-failure-v1",
    "source_dataset_id": "fixture-source-v1",
    "generator_version": 1,
    "agents": [
        {"id": "a", "source_directory": "a", "frames": [0]},
        {"id": "b", "source_directory": "b", "frames": [0]},
    ],
    "corruption": {
        "agent_id": "b",
        "frame": 0,
        "method": "truncate",
        "keep_bytes": 3,
    },
}, open(sys.argv[1], "w", encoding="utf-8"), indent=2)
PY

"$builder" lock-source \
  --source-root "$fixture/source" \
  --source-spec "$fixture/source.json" \
  --output "$fixture/source.lock.json" >/dev/null

for run in one two; do
  "$builder" build \
    --source-root "$fixture/source" \
    --source-lock "$fixture/source.lock.json" \
    --selection "$fixture/selection.json" \
    --output "$fixture/$run" \
    --archive "$fixture/$run.tar.gz" >/dev/null
done

test "$(sha256sum "$fixture/one.tar.gz" | cut -d' ' -f1)" = \
  "$(sha256sum "$fixture/two.tar.gz" | cut -d' ' -f1)"
cmp "$fixture/one/a/Scans/000000.pcd" \
  "$fixture/source/a/Scans/000000.pcd"
grep -q '^0 [0-9a-f]\{64\}  a/Scans/000000.pcd$' \
  "$fixture/one/a/scans.sha256"

for run in failure-one failure-two; do
  "$builder" build \
    --source-root "$fixture/source" \
    --source-lock "$fixture/source.lock.json" \
    --selection "$fixture/failure-selection.json" \
    --output "$fixture/$run" \
    --archive "$fixture/$run.tar.gz" >/dev/null
done
test "$(sha256sum "$fixture/failure-one.tar.gz" | cut -d' ' -f1)" = \
  "$(sha256sum "$fixture/failure-two.tar.gz" | cut -d' ' -f1)"
test "$(wc -c < "$fixture/failure-one/b/Scans/000000.pcd")" -eq 3
test "$(sha256sum "$fixture/failure-one/b/Scans/000000.pcd" | cut -d' ' -f1)" = \
  "$(awk 'NR == 1 {print $2}' "$fixture/failure-one/b/scans.sha256")"
grep -q '"method": "truncate"' "$fixture/failure-one/BUNDLE.json"
grep -q 'truncate b frame 0 to 3 bytes' "$fixture/failure-one/ATTRIBUTION.md"

printf 'changed\n' >> "$fixture/source/a/Scans/000000.pcd"
if "$builder" build \
  --source-root "$fixture/source" \
  --source-lock "$fixture/source.lock.json" \
  --selection "$fixture/selection.json" \
  --output "$fixture/rejected" \
  --archive "$fixture/rejected.tar.gz" >/dev/null 2>&1; then
  echo "changed source content was accepted" >&2
  exit 1
fi
test ! -e "$fixture/rejected"
test ! -e "$fixture/rejected.tar.gz"
