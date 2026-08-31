from __future__ import annotations

import dataclasses
import unittest

from experiment_test_fixture import ExperimentFixture
from open_lmm.experiments import ConfigTarget, Experiment, ParameterAxis
from open_lmm.experiments._planner import plan_trials


class PlannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = ExperimentFixture("planner", repetitions=2)

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_stable_cartesian_order_and_ids(self) -> None:
        plan = dataclasses.replace(
            self.fixture.plan,
            parameter_axes=(ParameterAxis("voxel", ConfigTarget("core/data.json", "/data_loader/voxel_size"), (0.2, 0.3)),),
        )
        digest, trials = plan_trials(plan)
        self.assertEqual(len(trials), 4)
        self.assertEqual([trial.seed for trial in trials], [101, 102, 101, 102])
        self.assertEqual([trial.parameters["voxel"] for trial in trials], [0.2, 0.2, 0.3, 0.3])
        self.assertEqual((digest, trials), plan_trials(plan))
        self.assertEqual(len({trial.trial_id for trial in trials}), 4)

    def test_max_trial_guard(self) -> None:
        plan = dataclasses.replace(self.fixture.plan, execution=dataclasses.replace(self.fixture.plan.execution, max_trials=1))
        with self.assertRaises(ValueError):
            plan_trials(plan)
        with self.assertRaises(ValueError):
            Experiment(plan, evidence_root=self.fixture.evidence).run()
        self.assertFalse(self.fixture.evidence.exists())


if __name__ == "__main__":
    unittest.main()
