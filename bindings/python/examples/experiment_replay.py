from __future__ import annotations

import argparse
from pathlib import Path

from open_lmm.experiments import SoftwareIdentity
from open_lmm.experiments.replay import ReplayAdapter, ReplayRequest, ReplayToolchain


parser = argparse.ArgumentParser()
for name in ("runner", "comparator", "case", "data_root", "config_root", "baseline", "evidence_root", "commit", "container_digest"):
    parser.add_argument(name)
args = parser.parse_args()

adapter = ReplayAdapter(ReplayToolchain(Path(args.runner), Path(args.comparator)))
result = adapter.run(
    ReplayRequest(
        Path(args.case), Path(args.data_root), Path(args.config_root),
        Path(args.baseline), Path(args.evidence_root),
        SoftwareIdentity(source_kind="git", source_identity=args.commit),
        args.container_digest,
    )
)
print(result.status.value)
