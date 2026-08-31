from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experiment_test_fixture import ExperimentFixture, minimum_manifest
from open_lmm.experiments._cli import main


class CLITests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = ExperimentFixture("cli")
        self.manifest = self.fixture.root / "experiment.json"
        self.manifest.write_text(json.dumps(minimum_manifest(self.fixture)), encoding="utf-8")

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_validate_and_usage_status(self) -> None:
        self.assertEqual(main(["validate", "--manifest", str(self.manifest)]), 0)
        document = minimum_manifest(self.fixture)
        document["unknown"] = True
        self.manifest.write_text(json.dumps(document), encoding="utf-8")
        self.assertEqual(main(["validate", "--manifest", str(self.manifest)]), 2)


if __name__ == "__main__":
    unittest.main()
