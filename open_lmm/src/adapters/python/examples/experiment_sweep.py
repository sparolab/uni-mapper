from __future__ import annotations

import argparse
from dataclasses import replace

from open_lmm.experiments import ConfigTarget, Experiment, ParameterAxis


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
axis = ParameterAxis(
    "voxel-size",
    ConfigTarget("core/data.json", "/data_loader/voxel_size"),
    (0.2, 0.3, 0.5),
)
result = Experiment(
    replace(loaded.plan, parameter_axes=(axis,)), evidence_root=args.evidence_root
).run()
print(result.result.value, len(result.trials))
