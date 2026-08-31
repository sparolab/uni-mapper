from __future__ import annotations

import os
import json
import hashlib
import importlib.metadata
import importlib.resources
import shutil
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import open_lmm
import open_lmm.experiments as experiments


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


class RuntimeFixture:
    """Tiny wheel-only fixture; intentionally independent of repository imports."""

    def __init__(self, name: str) -> None:
        self.root = Path(tempfile.mkdtemp(prefix=f"open_lmm_wheel_{name}_"))
        self.config = self.root / "config"
        self.data = self.root / "data"
        self.output = self.root / "output"
        self._write()

    def cleanup(self) -> None:
        shutil.rmtree(self.root, ignore_errors=True)

    def _write(self) -> None:
        scans = self.data / "agent1" / "Scans"
        scans.mkdir(parents=True)
        (self.config / "server").mkdir(parents=True)
        (self.config / "core").mkdir(parents=True)
        (scans / "000000.pcd").write_text(
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
            "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\n"
            "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\n"
            "DATA ascii\n10 0 0 1\n",
            encoding="utf-8",
        )
        (self.data / "agent1" / "poses.txt").write_text(
            "1 0 0 0 0 1 0 0 0 0 1 0\n",
            encoding="utf-8",
        )
        documents = {
            self.config / "config.json": {
                "global": {
                    "config_map_server": "server/map.json",
                    "config_data_loader": "core/data.json",
                    "config_loop_detector": "core/loop.json",
                    "config_backend_optimizer": "core/optimizer.json",
                    "config_dynamic_remover": "core/remover.json",
                },
                "directory": {
                    "root_dir_path": str(self.data),
                    "sub_dir_list": ["agent1"],
                    "root_save_dir": str(self.output),
                },
            },
            self.config / "server" / "map.json": {
                "map_server": {
                    "enable_map_updater": False,
                    "anchor_agent_index": 0,
                    "save_voxel_size": 0.2,
                    "parallel_data_load": False,
                    "parallel_map_update": False,
                    "max_parallel_agents": 1,
                }
            },
            self.config / "core" / "data.json": {
                "data_loader": {
                    "data_loader_type": "file_based",
                    "pose_format": "kitti",
                    "pose_file_name": "poses.txt",
                    "extrinsic": [0, 0, 0, 0, 0, 0, 1],
                    "scan_type": "pcd",
                    "scan_dir_name": "Scans",
                    "voxel_size": 0.5,
                    "min_range": 1.0,
                    "max_range": 60.0,
                    "delimiter": " ",
                }
            },
            self.config / "core" / "loop.json": {
                "loop_detector": {
                    "loop_detector_type": "kdtree",
                    "model": "scan_context",
                },
                "database": {
                    "descriptor_vector_dim": 20,
                    "distance_threshold": 0.15,
                    "num_candidates": 3,
                    "rebuild_threshold": 50,
                },
                "alignment": {
                    "pcm_translation_threshold": 10.0,
                    "pcm_rotation_threshold_deg": 20.0,
                    "pcm_solver": "heuristic",
                    "pcm_threads": 1,
                    "pcm_max_candidates": 0,
                },
            },
            self.config / "core" / "optimizer.json": {
                "backend_optimizer": {
                    "backend_optimizer_type": "incremental",
                    "relinearizeThreshold": 0.1,
                    "relinearizeSkip": 1,
                    "isam_extra_updates": 1,
                    "min_loop_frame_gap": 30,
                    "icp_search_num": 1,
                }
            },
            self.config / "core" / "remover.json": {
                "dynamic_remover": {
                    "dynamic_remover_type": "offline",
                    "model": "free_dom",
                }
            },
        }
        for path, document in documents.items():
            path.write_text(json.dumps(document), encoding="utf-8")


class InstalledWheelTests(unittest.TestCase):
    def test_import_is_not_served_from_source_or_build_tree(self) -> None:
        module_path = Path(open_lmm.__file__).resolve()
        self.assertIn("site-packages", module_path.parts)
        self.assertNotIn("/root/workspace", str(module_path))
        self.assertNotIn("openlmm-goal06-pep517-build", str(module_path))
        self.assertEqual(open_lmm.__version__, "3.0.0")
        self.assertEqual(open_lmm.API_VERSION, 1)
        entries = importlib.metadata.distribution("open-lmm").entry_points
        self.assertFalse(any(entry.name == "open-lmm-experiment" for entry in entries))
        schema = importlib.resources.files("open_lmm.experiments").joinpath(
            "schema/experiment_plan.schema.json"
        )
        self.assertTrue(schema.is_file())

    def test_source_free_public_workflow(self) -> None:
        fixture = RuntimeFixture("wheel")
        try:
            with open_lmm.Runtime(1) as runtime:
                runtime.open(fixture.config, label="wheel-e2e")
                runtime.run_all().wait()
                snapshot = runtime.snapshot()
                self.assertEqual(snapshot.label, "wheel-e2e")
                self.assertEqual(snapshot.pipeline.agents, ("agent1",))
                self.assertTrue(snapshot.output_directory.exists())
                visualization = runtime.visualization("agent1")
                self.assertEqual(visualization.points.shape[1], 4)
        finally:
            fixture.cleanup()

    def test_loader_does_not_need_development_paths(self) -> None:
        self.assertFalse(os.environ.get("LD_LIBRARY_PATH"))
        self.assertFalse(os.environ.get("PYTHONPATH"))

    def test_source_free_generic_experiment(self) -> None:
        fixture = RuntimeFixture("experiment")
        try:
            files = []
            for path in sorted(fixture.data.rglob("*")):
                if path.is_file():
                    files.append(
                        {
                            "path": path.relative_to(fixture.data).as_posix(),
                            "size": path.stat().st_size,
                            "sha256": sha256(path),
                        }
                    )
            index = fixture.data / "dataset.index.json"
            index.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "dataset_id": "wheel-tiny-v1",
                        "files": files,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                )
                + "\n",
                encoding="utf-8",
            )
            dataset = experiments.DatasetSpec(
                "wheel-tiny-v1",
                fixture.data,
                experiments.InputLockKind.SHA256_INDEX_V1,
                index,
                sha256(index),
            )
            config = experiments.ConfigSpec(
                fixture.config,
                tuple(
                    experiments.LockedFile(
                        path.relative_to(fixture.config).as_posix(),
                        sha256(path),
                        path.stat().st_size,
                    )
                    for path in sorted(fixture.config.rglob("*.json"))
                ),
                (
                    experiments.ConfigTarget(
                        "config.json", "/directory/root_dir_path"
                    ),
                ),
            )
            plan = experiments.ExperimentPlan(
                "wheel-experiment",
                dataset,
                config,
                experiments.SoftwareIdentity(),
                execution=experiments.ExecutionPolicy(
                    strict_reproducibility=False
                ),
            )
            result = experiments.Experiment(
                plan, evidence_root=fixture.root / "evidence"
            ).run()
            self.assertEqual(result.result, experiments.ExperimentStatus.SUCCEEDED)
            self.assertFalse(result.trials[0].reproducible)
            self.assertTrue((fixture.root / "evidence" / "evidence.sha256").is_file())

            repeated_plan = replace(
                plan,
                id="wheel-repeated",
                execution=replace(
                    plan.execution, repetitions=2, seeds=(101, 102)
                ),
            )
            repeated = experiments.Experiment(
                repeated_plan, evidence_root=fixture.root / "evidence-repeated"
            ).run()
            self.assertEqual(repeated.result, experiments.ExperimentStatus.SUCCEEDED)
            self.assertEqual(len(repeated.trials), 2)
            self.assertEqual(
                tuple(trial.seed for trial in repeated.trials), (101, 102)
            )

            sweep_plan = replace(
                plan,
                id="wheel-sweep",
                parameter_axes=(
                    experiments.ParameterAxis(
                        "voxel-size",
                        experiments.ConfigTarget(
                            "core/data.json", "/data_loader/voxel_size"
                        ),
                        (0.4, 0.6),
                    ),
                ),
            )
            swept = experiments.Experiment(
                sweep_plan, evidence_root=fixture.root / "evidence-sweep"
            ).run()
            self.assertEqual(swept.result, experiments.ExperimentStatus.SUCCEEDED)
            self.assertEqual(len(swept.trials), 2)
        finally:
            fixture.cleanup()


if __name__ == "__main__":
    unittest.main()
