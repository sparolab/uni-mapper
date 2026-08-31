#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 CANDIDATE_DIRECTORY [run_replay_case arguments...]" >&2
  exit 2
fi

candidate_root=$1
shift
if [[ -e "$candidate_root" ]]; then
  echo "candidate directory must not already exist: $candidate_root" >&2
  exit 2
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$candidate_root"

"$script_dir/run_replay_case.sh" "$@" \
  --output-root "$candidate_root/runtime-output" \
  --evidence-root "$candidate_root/evidence"

cat > "$candidate_root/REVIEW_REQUIRED.txt" <<'EOF'
This directory contains a candidate replay report, not an approved baseline.
Review repeated-run stability, provenance, tolerances, and every proposed exact
field before manually authoring or replacing a versioned baseline manifest.
No script in this workflow promotes this candidate automatically.
EOF
