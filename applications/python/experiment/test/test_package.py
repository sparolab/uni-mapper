from __future__ import annotations

import unittest
from pathlib import Path

import open_lmm_experiment


ROOT = Path(__file__).resolve().parents[1]


class PackageTests(unittest.TestCase):
    def test_console_owner_and_exact_sdk_dependency(self) -> None:
        pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
        self.assertIn('dependencies = ["open-lmm==3.0.0"]', pyproject)
        self.assertIn(
            'open-lmm-experiment = "open_lmm_experiment.cli:main"', pyproject
        )

    def test_source_only_consumes_public_experiment_api(self) -> None:
        source = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "package").rglob("*.py")
        )
        self.assertNotIn("open_lmm._native", source)
        self.assertNotIn("open_lmm.experiments._", source)

    def test_application_package_contains_no_native_binary(self) -> None:
        forbidden = {".so", ".dylib", ".dll", ".a"}
        self.assertFalse(
            [path for path in ROOT.rglob("*") if path.suffix.lower() in forbidden]
        )

    def test_public_surface_is_main_only(self) -> None:
        self.assertEqual(open_lmm_experiment.__all__, ["main"])


if __name__ == "__main__":
    unittest.main()
