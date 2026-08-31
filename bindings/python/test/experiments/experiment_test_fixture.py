from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from python_test_fixture import RuntimeFixture

from open_lmm.experiments import (
    ConfigSpec,
    ConfigTarget,
    DatasetSpec,
    ExecutionPolicy,
    ExperimentPlan,
    InputLockKind,
    LockedFile,
    MetricPolicy,
    SoftwareIdentity,
    WorkflowSpec,
)
from open_lmm.experiments._canonical import canonical_json_bytes, digest_file


class ExperimentFixture:
    def __init__(self, name: str, *, repetitions: int = 1, timeout: float = 30.0) -> None:
        self.runtime = RuntimeFixture(f"experiment_{name}")
        self.root = self.runtime.root
        self.evidence = self.root / "evidence"
        files = []
        for path in sorted(self.runtime.data.rglob("*")):
            if path.is_file():
                files.append(
                    {
                        "path": path.relative_to(self.runtime.data).as_posix(),
                        "size": path.stat().st_size,
                        "sha256": digest_file(path),
                    }
                )
        self.dataset_index = self.runtime.data / "dataset.index.json"
        self.dataset_index.write_bytes(
            canonical_json_bytes(
                {"schema_version": 1, "dataset_id": "tiny-v1", "files": files}
            )
        )
        self.dataset = DatasetSpec(
            "tiny-v1",
            self.runtime.data,
            InputLockKind.SHA256_INDEX_V1,
            self.dataset_index,
            digest_file(self.dataset_index),
        )
        config_files = tuple(
            LockedFile(
                path.relative_to(self.runtime.config).as_posix(),
                digest_file(path),
                path.stat().st_size,
            )
            for path in sorted(self.runtime.config.rglob("*.json"))
        )
        self.config = ConfigSpec(
            self.runtime.config,
            config_files,
            (ConfigTarget("config.json", "/directory/root_dir_path"),),
        )
        self.software = SoftwareIdentity(
            source_kind="git",
            source_identity="0" * 40,
            dirty=False,
        )
        self.plan = ExperimentPlan(
            "tiny-experiment",
            self.dataset,
            self.config,
            self.software,
            WorkflowSpec(),
            ExecutionPolicy(repetitions=repetitions, seeds=tuple(range(101, 101 + repetitions)), timeout_seconds=timeout),
            MetricPolicy(include_visualization=True),
        )

    def cleanup(self) -> None:
        self.runtime.cleanup()


def minimum_manifest(fixture: ExperimentFixture) -> dict[str, object]:
    return {
        "schema_version": 1,
        "experiment_id": "tiny-experiment",
        "dataset": {
            "id": fixture.dataset.id,
            "lock_kind": fixture.dataset.lock_kind.value,
            "lock_manifest_sha256": fixture.dataset.lock_sha256,
        },
        "config": {
            "files": [
                {"path": item.path, "sha256": item.sha256, "size": item.size}
                for item in fixture.config.files
            ],
            "dataset_bindings": [{"file": "config.json", "pointer": "/directory/root_dir_path"}],
        },
        "software": {
            "open_lmm_version": fixture.software.open_lmm_version,
            "runtime_api_version": 1,
            "experiment_api_version": 1,
            "source": {"kind": "git", "commit": "0" * 40, "dirty": False},
        },
        "workflow": {"kind": "run-all", "stages": []},
        "execution": {
            "mode": "fresh-process",
            "repetitions": 1,
            "seeds": [101],
            "timeout_seconds": 30,
            "failure_policy": "continue",
            "strict_reproducibility": True,
            "max_trials": 64,
        },
        "fixed_patches": [],
        "parameter_axes": [],
        "algorithm_variants": [],
        "metrics": {
            "include_visualization": True,
            "include_points": False,
            "preview_voxel_size_m": None,
            "hash_output_files": True,
        },
    }
