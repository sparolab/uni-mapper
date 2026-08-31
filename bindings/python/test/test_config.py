from __future__ import annotations

import unittest

import open_lmm

from python_test_fixture import RuntimeFixture


class ConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = RuntimeFixture("config")
        self.runtime = open_lmm.Runtime(1)
        self.runtime.open(self.fixture.config, label="config")

    def tearDown(self) -> None:
        self.runtime.close()
        self.fixture.cleanup()

    def test_apply_config_requires_authoritative_revision(self) -> None:
        before = self.runtime.snapshot()
        document = (self.fixture.config / "core" / "optimizer.json").read_text(
            encoding="utf-8"
        )
        receipt = self.runtime.apply_config(
            open_lmm.ConfigDomain.OPTIMIZER,
            document,
            expected=open_lmm.Revision(
                before.pipeline.runtime_revision,
                before.pipeline.config_revision,
            ),
        )
        after = self.runtime.snapshot()
        self.assertEqual(receipt.runtime_revision, after.pipeline.runtime_revision)
        self.assertEqual(receipt.config_revision, after.pipeline.config_revision)
        self.assertGreater(
            after.pipeline.config_revision,
            before.pipeline.config_revision,
        )

        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            self.runtime.apply_config(
                open_lmm.ConfigDomain.OPTIMIZER,
                document,
                expected=open_lmm.Revision(
                    before.pipeline.runtime_revision,
                    before.pipeline.config_revision,
                ),
            )
        stable = self.runtime.snapshot()
        self.assertEqual(stable.pipeline.config_revision, after.pipeline.config_revision)

    def test_malformed_candidate_does_not_change_committed_revision(self) -> None:
        before = self.runtime.snapshot()
        expected = open_lmm.Revision(
            before.pipeline.runtime_revision,
            before.pipeline.config_revision,
        )
        with self.assertRaises(open_lmm.OpenLMMParseError):
            self.runtime.apply_config(
                open_lmm.ConfigDomain.OPTIMIZER,
                "{not-json",
                expected=expected,
            )
        after = self.runtime.snapshot()
        self.assertEqual(after.pipeline.runtime_revision, before.pipeline.runtime_revision)
        self.assertEqual(after.pipeline.config_revision, before.pipeline.config_revision)


if __name__ == "__main__":
    unittest.main()
