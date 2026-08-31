from __future__ import annotations

import inspect
import unittest
from pathlib import Path

import open_lmm


class PublicApiTests(unittest.TestCase):
    def test_exact_public_export_manifest(self) -> None:
        manifest = (
            Path(__file__).parents[1] / "python_public_api_v1.txt"
        )
        expected = tuple(manifest.read_text(encoding="utf-8").splitlines())
        self.assertEqual(tuple(open_lmm.__all__), expected)
        self.assertEqual(open_lmm.API_VERSION, 1)
        self.assertEqual(open_lmm.__version__, "3.0.0")

    def test_public_objects_do_not_expose_native_owners(self) -> None:
        exported = "\n".join(open_lmm.__all__)
        for forbidden in (
            "RuntimeService",
            "PipelineController",
            "StageExecutor",
            "StageCoordinator",
            "RuntimeStateStore",
            "MapServer",
            "PCL",
        ):
            self.assertNotIn(forbidden, exported)
        self.assertTrue(inspect.isclass(open_lmm.Runtime))
        self.assertFalse(hasattr(open_lmm, "Result"))


if __name__ == "__main__":
    unittest.main()
