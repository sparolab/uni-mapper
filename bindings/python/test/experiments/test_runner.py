from __future__ import annotations

import unittest
import dataclasses

from experiment_test_fixture import ExperimentFixture
from open_lmm.experiments import ConfigPatch, Experiment, ExperimentStatus, FailurePolicy, SoftwareIdentity


class RunnerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = ExperimentFixture("runner", repetitions=2)

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_repeated_trials_use_fresh_processes_and_publish_evidence(self) -> None:
        result = Experiment(self.fixture.plan, evidence_root=self.fixture.evidence).run()
        self.assertEqual(result.result, ExperimentStatus.SUCCEEDED)
        self.assertEqual(len(result.trials), 2)
        self.assertEqual(len({trial.worker_pid for trial in result.trials}), 2)
        self.assertTrue(all(trial.input_reverified and trial.reproducible for trial in result.trials))
        self.assertTrue(all(trial.runtime_revision_after is not None for trial in result.trials))
        self.assertTrue((self.fixture.evidence / "experiment-result.json").is_file())
        self.assertTrue((self.fixture.evidence / "metrics.csv").is_file())
        self.assertTrue((self.fixture.evidence / "evidence.sha256").is_file())
        with self.assertRaises(FileExistsError):
            Experiment(self.fixture.plan, evidence_root=self.fixture.evidence).run()

    def test_continue_and_stop_preserve_structured_failure(self) -> None:
        failing = dataclasses.replace(
            self.fixture.plan,
            fixed_patches=(ConfigPatch("core/data.json", "/data_loader/data_loader_type", "unknown"),),
        )
        result = Experiment(failing, evidence_root=self.fixture.evidence).run()
        self.assertEqual(result.result, ExperimentStatus.FAILED)
        self.assertEqual(len(result.trials), 2)
        self.assertTrue(all(trial.error and trial.error["kind"] == "open_lmm" for trial in result.trials))

        self.fixture.evidence = self.fixture.root / "stop-evidence"
        stop = dataclasses.replace(
            failing,
            execution=dataclasses.replace(failing.execution, failure_policy=FailurePolicy.STOP),
        )
        stopped = Experiment(stop, evidence_root=self.fixture.evidence).run()
        self.assertEqual(len(stopped.trials), 1)

    def test_strict_dirty_identity_rejected_before_output(self) -> None:
        dirty = dataclasses.replace(
            self.fixture.plan,
            software=SoftwareIdentity(source_kind="git", source_identity="1" * 40, dirty=True),
        )
        with self.assertRaises(ValueError):
            Experiment(dirty, evidence_root=self.fixture.evidence).run()
        self.assertFalse(self.fixture.evidence.exists())


if __name__ == "__main__":
    unittest.main()
