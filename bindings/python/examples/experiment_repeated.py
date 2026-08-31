from __future__ import annotations

import argparse
from dataclasses import replace

from open_lmm.experiments import Experiment


parser = argparse.ArgumentParser()
parser.add_argument("manifest")
parser.add_argument("dataset_root")
parser.add_argument("config_root")
parser.add_argument("evidence_root")
args = parser.parse_args()

loaded = Experiment.from_manifest(
    args.manifest,
    dataset_root=args.dataset_root,
    config_root=args.config_root,
    evidence_root=args.evidence_root,
)
policy = replace(loaded.plan.execution, repetitions=3, seeds=(101, 102, 103))
result = Experiment(
    replace(loaded.plan, execution=policy), evidence_root=args.evidence_root
).run()
print(result.summaries)
