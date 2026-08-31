#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 BUILD_DIRECTORY CONTAINER_DIGEST" >&2
  exit 2
fi

build_root=$1
container_digest=$2
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
evidence_root=${OPEN_LMM_REPLAY_EVIDENCE_ROOT:-"$repository_root/.ci-build/replay"}
mkdir -p "$(dirname "$evidence_root")"

ctest --test-dir "$build_root" --output-on-failure \
  -R '^open_lmm_replay_(contract|validate|compare|fetch|subset|runner_e2e)_tests$' \
  --output-junit "$evidence_root-synthetic.xml"

if [[ -z "${OPEN_LMM_REPLAY_CASES:-}" ]]; then
  echo "OPEN_LMM_REPLAY_CASES is required for external replay CI" >&2
  exit 1
fi
if [[ -z "${OPEN_LMM_REPLAY_DATA_ROOT:-}" ||
      -z "${OPEN_LMM_REPLAY_CONFIG_ROOT:-}" ||
      -z "${OPEN_LMM_REPLAY_BASELINE_ROOT:-}" ]]; then
  echo "external replay data/config/baseline roots are required" >&2
  exit 1
fi

mkdir -p "$evidence_root"
case_index=0
for case_manifest in $OPEN_LMM_REPLAY_CASES; do
  case_index=$((case_index + 1))
  case_evidence="$evidence_root/case-$case_index"
  case_name=$(basename "$case_manifest" .json)
  baseline="$OPEN_LMM_REPLAY_BASELINE_ROOT/$case_name.baseline.json"
  "$repository_root/scripts/replay/run_replay_case.sh" \
    --runner "$build_root/test/open_lmm_replay_runner" \
    --compare "$build_root/test/open_lmm_replay_compare" \
    --case "$case_manifest" \
    --data-root "$OPEN_LMM_REPLAY_DATA_ROOT" \
    --config-root "$OPEN_LMM_REPLAY_CONFIG_ROOT" \
    --output-root "$case_evidence-output" \
    --evidence-root "$case_evidence" \
    --container-digest "$container_digest" \
    --baseline "$baseline"
done
