from __future__ import annotations

import unittest

from experiment_test_fixture import ExperimentFixture
from open_lmm.experiments import ConfigPatch
from open_lmm.experiments._canonical import digest_file
from open_lmm.experiments._config import materialize_config


class ConfigMaterializerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = ExperimentFixture("config")

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_replace_copy_preserves_source(self) -> None:
        source_before = {item.path: digest_file(self.fixture.config.directory / item.path) for item in self.fixture.config.files}
        destination = self.fixture.root / "materialized"
        digest, files = materialize_config(
            self.fixture.config,
            self.fixture.dataset,
            destination,
            (ConfigPatch("core/data.json", "/data_loader/voxel_size", 0.25),),
        )
        self.assertTrue(digest.startswith("sha256:"))
        self.assertEqual(len(files), len(self.fixture.config.files))
        self.assertEqual(source_before, {item.path: digest_file(self.fixture.config.directory / item.path) for item in self.fixture.config.files})
        self.assertIn("0.25", (destination / "core/data.json").read_text(encoding="utf-8"))

    def test_missing_and_duplicate_target_rejected(self) -> None:
        with self.assertRaises(ValueError):
            materialize_config(self.fixture.config, self.fixture.dataset, self.fixture.root / "bad", (ConfigPatch("core/data.json", "/missing", 1),))
        with self.assertRaises(ValueError):
            materialize_config(
                self.fixture.config,
                self.fixture.dataset,
                self.fixture.root / "duplicate",
                (ConfigPatch("config.json", "/directory/root_dir_path", "x"),),
            )
        with self.assertRaises(ValueError):
            materialize_config(
                self.fixture.config,
                self.fixture.dataset,
                self.fixture.root / "type-mismatch",
                (ConfigPatch("core/data.json", "/data_loader/voxel_size", "0.5"),),
            )


if __name__ == "__main__":
    unittest.main()
