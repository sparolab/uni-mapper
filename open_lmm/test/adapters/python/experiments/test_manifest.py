from __future__ import annotations

import json
import os
import unittest
from pathlib import Path

from experiment_test_fixture import ExperimentFixture, minimum_manifest
from open_lmm.experiments._canonical import canonical_json_bytes, digest_file, digest_value
from open_lmm.experiments._config import verify_dataset
from open_lmm.experiments._manifest import load_plan, validate_manifest_document


class ManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = ExperimentFixture("manifest")

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_canonical_order_and_closed_manifest(self) -> None:
        self.assertEqual(digest_value({"b": 2, "a": 1}), digest_value({"a": 1, "b": 2}))
        document = minimum_manifest(self.fixture)
        validate_manifest_document(document)
        document["bad"] = 1
        with self.assertRaises(ValueError):
            validate_manifest_document(document)
        with self.assertRaises(ValueError):
            canonical_json_bytes({"value": float("nan")})
        bad_software = minimum_manifest(self.fixture)
        bad_software["software"]["source"]["dirty"] = "false"
        with self.assertRaises(ValueError):
            validate_manifest_document(bad_software)

    def test_dataset_tamper_extra_and_symlink_fail_closed(self) -> None:
        self.assertEqual(verify_dataset(self.fixture.dataset), self.fixture.dataset.lock_sha256)
        extra = self.fixture.runtime.data / "extra.bin"
        extra.write_bytes(b"x")
        with self.assertRaises(ValueError):
            verify_dataset(self.fixture.dataset)
        extra.unlink()
        symlink = self.fixture.runtime.data / "escape"
        symlink.symlink_to("/tmp")
        with self.assertRaises(ValueError):
            verify_dataset(self.fixture.dataset)

    def test_manifest_locator_resolution(self) -> None:
        manifest = self.fixture.root / "plan.json"
        manifest.write_text(json.dumps(minimum_manifest(self.fixture)), encoding="utf-8")
        plan, payload = load_plan(manifest, dataset_root=self.fixture.runtime.data, config_root=self.fixture.runtime.config)
        self.assertEqual(plan.dataset.root, self.fixture.runtime.data.resolve())
        self.assertTrue(payload.endswith(b"\n"))


if __name__ == "__main__":
    unittest.main()
