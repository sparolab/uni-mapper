from __future__ import annotations

import dataclasses
import os
import unittest

from experiment_test_fixture import ExperimentFixture
from open_lmm.experiments import ExecutionMode, Experiment, ExperimentStatus, SoftwareIdentity


class ProcessLifetimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = ExperimentFixture("timeout")

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_timeout_is_not_success_and_worker_is_reaped(self) -> None:
        policy = dataclasses.replace(self.fixture.plan.execution, timeout_seconds=0.001)
        plan = dataclasses.replace(self.fixture.plan, execution=policy)
        result = Experiment(plan, evidence_root=self.fixture.evidence).run()
        self.assertEqual(result.result, ExperimentStatus.TIMED_OUT)
        self.assertEqual(result.trials[0].status, "timed_out")
        self.assertIsNotNone(result.trials[0].worker_pid)
        with self.assertRaises(ChildProcessError):
            os.waitpid(result.trials[0].worker_pid, os.WNOHANG)

    def test_in_process_debug_mode_is_explicitly_non_reproducible(self) -> None:
        policy = dataclasses.replace(
            self.fixture.plan.execution,
            mode=ExecutionMode.IN_PROCESS,
            strict_reproducibility=False,
        )
        plan = dataclasses.replace(
            self.fixture.plan,
            execution=policy,
            software=SoftwareIdentity(),
        )
        result = Experiment(plan, evidence_root=self.fixture.evidence).run()
        self.assertEqual(result.result, ExperimentStatus.SUCCEEDED)
        self.assertEqual(result.trials[0].worker_pid, os.getpid())
        self.assertFalse(result.trials[0].reproducible)


if __name__ == "__main__":
    unittest.main()
