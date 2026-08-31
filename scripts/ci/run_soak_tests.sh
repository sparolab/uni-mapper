#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 BUILD_DIRECTORY fast|nightly CONTAINER_DIGEST" >&2
  exit 2
fi

build_root=$1
profile=$2
container_digest=$3
case "$profile" in
  fast)
    iterations=100
    warmup=10
    ;;
  nightly)
    iterations=1000
    warmup=100
    ;;
  *)
    echo "profile must be fast or nightly" >&2
    exit 2
    ;;
esac

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd "$script_dir/../.." && pwd)
default_evidence_root="$repository_root/.ci-build/soak-$profile"
evidence_root=${OPEN_LMM_SOAK_EVIDENCE_ROOT:-"$default_evidence_root"}
if [[ -e "$evidence_root" ]]; then
  echo "soak evidence path must be clean: $evidence_root" >&2
  exit 1
fi
mkdir -p "$evidence_root"

ctest --test-dir "$build_root" --output-on-failure \
  -R '^open_lmm_soak_metrics_contract_tests$' \
  --output-junit "$evidence_root/metrics-contract.xml"

"$repository_root/scripts/soak/run_soak.sh" \
  --build "$build_root" \
  --profile "$profile" \
  --scenario all-headless \
  --iterations "$iterations" \
  --warmup "$warmup" \
  --seed 104729 \
  --container-digest "$container_digest" \
  --report "$evidence_root/all-headless.json"
