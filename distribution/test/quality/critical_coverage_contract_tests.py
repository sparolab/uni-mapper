#!/usr/bin/env python3
"""Contract tests for the reviewed critical-coverage regression comparator."""

from __future__ import annotations

import copy
import importlib.util
import pathlib
import sys


def load_module(path: pathlib.Path):
    spec = importlib.util.spec_from_file_location("critical_coverage", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load comparator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    if len(sys.argv) != 2:
        raise RuntimeError("usage: critical_coverage_contract_tests.py COMPARATOR")
    comparator = load_module(pathlib.Path(sys.argv[1]))
    owner = {
        "path": "src/runtime/state/runtime_state_store.cpp",
        "source_sha256": "source-hash",
        "branches": {"count": 10, "covered": 8, "rate": 0.8},
        "lines": {"count": 10, "covered": 9},
        "functions": {"count": 3, "covered": 3},
    }
    candidate = {
        "manifests": {
            "critical_sources_sha256": "source-manifest",
            "critical_tests_sha256": "test-manifest",
        },
        "owners": {"RuntimeStateStore": owner},
    }
    baseline = copy.deepcopy(candidate)
    baseline.update({"schema_version": 1, "status": "reviewed"})
    require(
        not comparator.verify_baseline(candidate, baseline),
        "identical reviewed baseline must pass",
    )

    lower = copy.deepcopy(candidate)
    lower["owners"]["RuntimeStateStore"]["branches"].update(
        {"covered": 7, "rate": 0.7}
    )
    failures = comparator.verify_baseline(lower, baseline)
    require(
        any("covered branch count regressed" in failure for failure in failures)
        and any("coverage rate regressed" in failure for failure in failures),
        "branch-count and rate regressions must fail instead of rewriting baseline",
    )

    changed_denominator = copy.deepcopy(candidate)
    changed_denominator["owners"]["RuntimeStateStore"]["branches"]["count"] = 12
    require(
        any(
            "branch denominator changed" in failure
            for failure in comparator.verify_baseline(changed_denominator, baseline)
        ),
        "source branch denominator changes must require review",
    )

    changed_source = copy.deepcopy(candidate)
    changed_source["owners"]["RuntimeStateStore"]["source_sha256"] = "changed"
    require(
        any(
            "critical source changed" in failure
            for failure in comparator.verify_baseline(changed_source, baseline)
        ),
        "critical source changes must require recalibration review",
    )

    unreviewed = copy.deepcopy(baseline)
    unreviewed["status"] = "candidate"
    require(
        comparator.verify_baseline(candidate, unreviewed),
        "candidate baseline must never be accepted by the required gate",
    )
    print("critical coverage comparator contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
