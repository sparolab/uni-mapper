from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from open_lmm.experiments import ExperimentStatus
from open_lmm_experiment.cli import main


class ExperimentCliTests(unittest.TestCase):
    def test_validate_delegates_to_public_sdk_api(self) -> None:
        with patch("open_lmm_experiment.cli.validate_manifest") as validate:
            self.assertEqual(main(["validate", "--manifest", "plan.json"]), 0)
        validate.assert_called_once_with("plan.json")

    def test_invalid_manifest_returns_input_error(self) -> None:
        with patch(
            "open_lmm_experiment.cli.validate_manifest",
            side_effect=ValueError("invalid schema"),
        ):
            self.assertEqual(main(["validate", "--manifest", "plan.json"]), 2)

    def test_run_maps_success_and_failure_results(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "plan.json"
            manifest.write_text("{}", encoding="utf-8")
            arguments = [
                "run",
                "--manifest",
                str(manifest),
                "--dataset-root",
                str(root),
                "--evidence-root",
                str(root / "evidence"),
            ]
            for status, expected in (
                (ExperimentStatus.SUCCEEDED, 0),
                (ExperimentStatus.FAILED, 1),
            ):
                experiment = SimpleNamespace(
                    run=lambda status=status: SimpleNamespace(result=status)
                )
                with self.subTest(status=status), patch(
                    "open_lmm_experiment.cli.Experiment.from_manifest",
                    return_value=experiment,
                ):
                    self.assertEqual(main(arguments), expected)

    def test_missing_dataset_returns_unavailable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "plan.json"
            manifest.write_text(json.dumps({}), encoding="utf-8")
            self.assertEqual(
                main(
                    [
                        "run",
                        "--manifest",
                        str(manifest),
                        "--dataset-root",
                        str(Path(directory) / "missing"),
                        "--evidence-root",
                        str(Path(directory) / "evidence"),
                    ]
                ),
                77,
            )


if __name__ == "__main__":
    unittest.main()
