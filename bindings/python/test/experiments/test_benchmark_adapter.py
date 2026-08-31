from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

from open_lmm.experiments.benchmark import BenchmarkAdapter, BenchmarkRequest, BenchmarkStatus, BenchmarkToolchain


def bundle(status: str) -> dict[str, object]:
    return {
        "schema_version": 1,
        "bundle_id": "fixture",
        "profile": "contract",
        "scenario": "open",
        "key": {"fixture_id": "small-v1"},
        "reports": [],
        "metrics": [{"name": "wall_ns", "summary": {"sample_count": 2, "median": 5, "p95": 7, "mad": 1, "min": 4, "max": 7}}],
        "baseline": None,
        "comparisons": [],
        "comparison": status,
        "failures": [],
    }


class BenchmarkAdapterTests(unittest.TestCase):
    def test_load_preserves_canonical_status(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / "benchmark.sh"
            script.write_text("exit 77\n", encoding="utf-8")
            adapter = BenchmarkAdapter(BenchmarkToolchain(script.resolve(), root.resolve()))
            for status in ("pass", "fail", "uncalibrated", "baseline_mismatch"):
                path = root / f"{status}.json"
                path.write_text(json.dumps(bundle(status)), encoding="utf-8")
                self.assertEqual(adapter.load(path).status.value, status)
            request = BenchmarkRequest("external", "small-v1", "open", 1, 0, root / "evidence", "sha256:" + "3" * 64)
            self.assertEqual(adapter.run(request).status, BenchmarkStatus.NOT_AVAILABLE)

    def test_unknown_schema_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / "benchmark.sh"
            script.write_text("exit 0\n", encoding="utf-8")
            path = root / "bad.json"
            path.write_text('{"schema_version":2}', encoding="utf-8")
            adapter = BenchmarkAdapter(BenchmarkToolchain(script.resolve(), root.resolve()))
            with self.assertRaises(ValueError):
                adapter.load(path)

    def test_existing_goal05_contract_runner(self) -> None:
        repository = Path(os.environ["OPEN_LMM_REPOSITORY_ROOT"])
        build = Path(os.environ["OPEN_LMM_CORE_TEST_BUILD_ROOT"])
        script = (repository / "scripts/benchmark/run_benchmarks.sh").resolve()
        self.assertTrue(script.is_file())
        with tempfile.TemporaryDirectory() as directory:
            request = BenchmarkRequest(
                "contract",
                "small-v1",
                "open",
                1,
                0,
                Path(directory) / "evidence",
                "sha256:" + "6" * 64,
            )
            result = BenchmarkAdapter(
                BenchmarkToolchain(script, build.resolve())
            ).run(request)
            required = (
                "open_lmm_benchmark_generate_fixture",
                "open_lmm_benchmark_runner",
                "open_lmm_benchmark_aggregate",
                "open_lmm_benchmark_resource_owner_runner",
                "open_lmm_benchmark_pair",
                "open_lmm_artifact_compare",
            )
            if all((build / "test" / name).is_file() for name in required):
                self.assertEqual(result.status, BenchmarkStatus.UNCALIBRATED)
                self.assertIsNotNone(result.bundle_sha256)
                self.assertTrue(result.metrics)
            else:
                self.assertEqual(result.status, BenchmarkStatus.FAIL)
                self.assertIsNone(result.bundle_sha256)
                self.assertFalse(result.metrics)


if __name__ == "__main__":
    unittest.main()
