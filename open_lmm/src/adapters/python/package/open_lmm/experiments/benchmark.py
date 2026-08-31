from __future__ import annotations

import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from ._canonical import digest_file, digest_value, load_closed_json, require_keys, require_sha256
from ._models import MetricRecord, MetricSource
from ._subprocess import run_logged, safe_environment


class BenchmarkStatus(str, Enum):
    PASS = "pass"
    FAIL = "fail"
    UNCALIBRATED = "uncalibrated"
    BASELINE_MISMATCH = "baseline_mismatch"
    NOT_AVAILABLE = "not_available"
    INVALID = "invalid"


@dataclass(frozen=True, slots=True)
class BenchmarkToolchain:
    script: Path
    build_root: Path
    baseline: Path | None = None

    def __post_init__(self) -> None:
        script = Path(self.script)
        build = Path(self.build_root)
        if not script.is_absolute() or not script.is_file():
            raise ValueError("benchmark script must be an explicit absolute file path")
        if not build.is_absolute() or not build.is_dir():
            raise ValueError("benchmark build root must be an explicit absolute directory")
        object.__setattr__(self, "script", script.resolve())
        object.__setattr__(self, "build_root", build.resolve())
        if self.baseline is not None:
            baseline = Path(self.baseline)
            if not baseline.is_absolute() or not baseline.is_file():
                raise ValueError("benchmark baseline must be an explicit absolute file path")
            object.__setattr__(self, "baseline", baseline.resolve())


@dataclass(frozen=True, slots=True)
class BenchmarkRequest:
    profile: str
    fixture: str
    scenario: str
    repetitions: int
    warmups: int
    evidence_root: Path
    container_digest: str
    sanitizer: str = "none"
    timeout_seconds: float = 3600.0
    log_limit_bytes: int = 1024 * 1024

    def __post_init__(self) -> None:
        if self.repetitions < 1 or self.warmups < 0:
            raise ValueError("benchmark repetitions/warmups are invalid")
        require_sha256(self.container_digest, where="container digest")
        object.__setattr__(self, "evidence_root", Path(self.evidence_root).resolve())
        if self.timeout_seconds <= 0:
            raise ValueError("benchmark timeout must be positive")
        if self.log_limit_bytes < 1024:
            raise ValueError("benchmark log limit must be at least 1024 bytes")


@dataclass(frozen=True, slots=True)
class BenchmarkResult:
    status: BenchmarkStatus
    bundle: Path | None
    bundle_sha256: str | None
    baseline_sha256: str | None
    key: dict[str, Any] | None
    comparisons: tuple[dict[str, Any], ...]
    metrics: tuple[MetricRecord, ...]
    exit_code: int
    error: str = ""


class BenchmarkAdapter:
    __slots__ = ("_toolchain",)

    def __init__(self, toolchain: BenchmarkToolchain) -> None:
        self._toolchain = toolchain

    def load(self, bundle: Path) -> BenchmarkResult:
        path = Path(bundle).resolve()
        document = load_closed_json(path)
        root = require_keys(
            document,
            required={"schema_version", "bundle_id", "profile", "scenario", "key", "reports", "metrics", "baseline", "comparisons", "comparison", "failures"},
            where="benchmark bundle",
        )
        if root["schema_version"] != 1:
            raise ValueError("unsupported benchmark bundle schema")
        try:
            status = BenchmarkStatus(root["comparison"])
        except ValueError as error:
            raise ValueError("unsupported benchmark comparison status") from error
        records: list[MetricRecord] = []
        if not isinstance(root["metrics"], list):
            raise ValueError("benchmark metrics must be an array")
        for item in root["metrics"]:
            metric = require_keys(item, required={"name", "summary"}, where="benchmark metric")
            summary = require_keys(metric["summary"], required={"sample_count", "median", "p95", "mad", "min", "max"}, where="benchmark summary")
            for statistic in ("sample_count", "median", "p95", "mad", "min", "max"):
                records.append(MetricRecord(f"benchmark.{metric['name']}.{statistic}", summary[statistic], "reported", MetricSource.BENCHMARK_V1, "experiment"))
        baseline_sha = None if root["baseline"] is None else root["baseline"].get("sha256")
        return BenchmarkResult(status, path, digest_file(path), baseline_sha, dict(root["key"]), tuple(dict(item) for item in root["comparisons"]), tuple(records), 0)

    def run(self, request: BenchmarkRequest) -> BenchmarkResult:
        if request.evidence_root.exists():
            raise FileExistsError("benchmark evidence root must not exist")
        request.evidence_root.parent.mkdir(parents=True, exist_ok=True)
        baseline_before = digest_file(self._toolchain.baseline) if self._toolchain.baseline else None
        argv = [
            "/usr/bin/bash",
            str(self._toolchain.script),
            "--build", str(self._toolchain.build_root),
            "--profile", request.profile,
            "--fixture", request.fixture,
            "--scenario", request.scenario,
            "--repetitions", str(request.repetitions),
            "--warmup", str(request.warmups),
            "--output", str(request.evidence_root),
            "--container-digest", request.container_digest,
            "--sanitizer", request.sanitizer,
        ]
        if self._toolchain.baseline is not None:
            argv.extend(("--baseline", str(self._toolchain.baseline)))
        temporary_log = request.evidence_root.parent / f".{request.evidence_root.name}.adapter-{os.getpid()}.log"
        completed = run_logged(
            argv,
            log_path=temporary_log,
            timeout_seconds=request.timeout_seconds,
            log_limit_bytes=request.log_limit_bytes,
            environment=safe_environment(include_home=True),
        )
        if not request.evidence_root.exists():
            request.evidence_root.mkdir(parents=True)
        temporary_log.replace(request.evidence_root / "adapter.log")
        if completed.timed_out:
            return BenchmarkResult(BenchmarkStatus.FAIL, None, None, baseline_before, None, (), (), -1, "benchmark timed out")
        if completed.returncode == 77:
            return BenchmarkResult(BenchmarkStatus.NOT_AVAILABLE, None, None, baseline_before, None, (), (), 77, "benchmark unavailable")
        if self._toolchain.baseline is not None and digest_file(self._toolchain.baseline) != baseline_before:
            return BenchmarkResult(BenchmarkStatus.INVALID, None, None, baseline_before, None, (), (), completed.returncode, "baseline was mutated")
        candidates = sorted(request.evidence_root.glob("*/bundle.json")) if request.evidence_root.exists() else []
        if not candidates:
            return BenchmarkResult(BenchmarkStatus.FAIL if completed.returncode else BenchmarkStatus.INVALID, None, None, baseline_before, None, (), (), completed.returncode, "benchmark produced no bundle")
        try:
            loaded_results = tuple(self.load(candidate) for candidate in candidates)
        except (OSError, ValueError) as error:
            return BenchmarkResult(BenchmarkStatus.INVALID, candidates[0] if len(candidates) == 1 else None, digest_file(candidates[0]) if len(candidates) == 1 else None, baseline_before, None, (), (), completed.returncode, str(error))
        precedence = {
            BenchmarkStatus.PASS: 0,
            BenchmarkStatus.UNCALIBRATED: 1,
            BenchmarkStatus.BASELINE_MISMATCH: 2,
            BenchmarkStatus.FAIL: 3,
            BenchmarkStatus.NOT_AVAILABLE: 4,
            BenchmarkStatus.INVALID: 5,
        }
        status = max((item.status for item in loaded_results), key=precedence.__getitem__)
        if completed.returncode != 0 and status == BenchmarkStatus.PASS:
            status = BenchmarkStatus.FAIL
        one = loaded_results[0] if len(loaded_results) == 1 else None
        bundle_digest = (
            one.bundle_sha256
            if one is not None
            else digest_value([{"path": path.relative_to(request.evidence_root).as_posix(), "sha256": digest_file(path)} for path in candidates])
        )
        baselines = {item.baseline_sha256 for item in loaded_results}
        return BenchmarkResult(
            status,
            None if one is None else one.bundle,
            bundle_digest,
            next(iter(baselines)) if len(baselines) == 1 else None,
            None if one is None else one.key,
            tuple(comparison for item in loaded_results for comparison in item.comparisons),
            tuple(metric for item in loaded_results for metric in item.metrics),
            completed.returncode,
            "benchmark command failed" if completed.returncode != 0 else "",
        )


__all__ = ["BenchmarkAdapter", "BenchmarkRequest", "BenchmarkResult", "BenchmarkStatus", "BenchmarkToolchain"]
