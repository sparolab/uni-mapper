from __future__ import annotations

import os
import unittest
from pathlib import Path

import open_lmm
import open_lmm.experiments as experiments
from open_lmm.experiments import benchmark, replay


class PublicAPITests(unittest.TestCase):
    def test_golden_exports_and_top_level_stability(self) -> None:
        source = Path(os.environ["OPEN_LMM_BINDING_SOURCE_ROOT"])
        lines = (source / "experiments_public_api_v1.txt").read_text(encoding="utf-8").splitlines()
        sections: dict[str, list[str]] = {}
        active = ""
        for line in lines:
            if not line:
                continue
            if line.startswith("open_lmm"):
                active = line
                sections[active] = []
            else:
                sections[active].append(line)
        self.assertEqual(sorted(experiments.__all__), sorted(sections["open_lmm.experiments"]))
        self.assertEqual(sorted(replay.__all__), sorted(sections["open_lmm.experiments.replay"]))
        self.assertEqual(sorted(benchmark.__all__), sorted(sections["open_lmm.experiments.benchmark"]))
        self.assertEqual(len(open_lmm.__all__), 46)
        self.assertNotIn("experiments", open_lmm.__all__)
        self.assertEqual(experiments.EXPERIMENT_API_VERSION, 1)

    def test_package_does_not_eagerly_require_pandas(self) -> None:
        self.assertNotIn("pandas", __import__("sys").modules)


if __name__ == "__main__":
    unittest.main()
