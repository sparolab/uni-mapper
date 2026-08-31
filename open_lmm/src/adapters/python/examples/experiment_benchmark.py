from __future__ import annotations

import argparse
from pathlib import Path

from open_lmm.experiments.benchmark import BenchmarkAdapter, BenchmarkRequest, BenchmarkToolchain


parser = argparse.ArgumentParser()
for name in ("script", "build_root", "baseline", "evidence_root", "container_digest"):
    parser.add_argument(name)
args = parser.parse_args()

adapter = BenchmarkAdapter(
    BenchmarkToolchain(Path(args.script), Path(args.build_root), Path(args.baseline))
)
result = adapter.run(
    BenchmarkRequest(
        "contract", "small-v1", "open", 3, 1,
        Path(args.evidence_root), args.container_digest,
    )
)
print(result.status.value)
