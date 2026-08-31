from __future__ import annotations

import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from ._canonical import digest_file, load_closed_json, require_keys, require_sha256
from ._models import MetricRecord, MetricSource, SoftwareIdentity
from ._subprocess import run_logged, safe_environment


class ReplayStatus(str, Enum):
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    NOT_AVAILABLE = "not_available"
    INVALID = "invalid"


@dataclass(frozen=True, slots=True)
class ReplayToolchain:
    runner: Path
    comparator: Path

    def __post_init__(self) -> None:
        for name in ("runner", "comparator"):
            path = Path(getattr(self, name))
            if not path.is_absolute() or not path.is_file() or not os.access(path, os.X_OK):
                raise ValueError(f"replay {name} must be an explicit absolute executable path")
            object.__setattr__(self, name, path.resolve())


@dataclass(frozen=True, slots=True)
class ReplayRequest:
    case_manifest: Path
    data_root: Path
    config_root: Path
    baseline: Path
    evidence_root: Path
    software: SoftwareIdentity
    container_digest: str
    timeout_seconds: float = 900.0
    log_limit_bytes: int = 1024 * 1024

    def __post_init__(self) -> None:
        require_sha256(self.container_digest, where="container digest")
        for name in ("case_manifest", "baseline"):
            path = Path(getattr(self, name))
            if not path.is_file():
                raise ValueError(f"replay {name} does not exist")
            object.__setattr__(self, name, path.resolve())
        for name in ("data_root", "config_root"):
            path = Path(getattr(self, name))
            if not path.is_dir():
                raise ValueError(f"replay {name} does not exist")
            object.__setattr__(self, name, path.resolve())
        object.__setattr__(self, "evidence_root", Path(self.evidence_root).resolve())
        if self.timeout_seconds <= 0:
            raise ValueError("replay timeout must be positive")
        if self.log_limit_bytes < 1024:
            raise ValueError("replay log limit must be at least 1024 bytes")


@dataclass(frozen=True, slots=True)
class ReplayResult:
    status: ReplayStatus
    report: Path | None
    diff: Path | None
    report_sha256: str | None
    baseline_sha256: str
    diff_sha256: str | None
    runner_exit_code: int
    comparator_exit_code: int | None
    metrics: tuple[MetricRecord, ...]
    error: str = ""


def _flatten_metrics(value: Any, prefix: str = "") -> list[MetricRecord]:
    result: list[MetricRecord] = []
    if isinstance(value, dict):
        for key in sorted(value):
            child = f"{prefix}.{key}" if prefix else str(key)
            result.extend(_flatten_metrics(value[key], child))
    elif value is None or isinstance(value, (bool, int, float, str)):
        result.append(MetricRecord(f"replay.{prefix}", value, "reported", MetricSource.REPLAY_V1, "experiment"))
    return result


class ReplayAdapter:
    __slots__ = ("_toolchain",)

    def __init__(self, toolchain: ReplayToolchain) -> None:
        self._toolchain = toolchain

    def run(self, request: ReplayRequest) -> ReplayResult:
        source = request.software
        if source.source_kind != "git" or not source.source_identity:
            raise ValueError("Goal 03 replay requires an explicit git software identity")
        baseline_before = digest_file(request.baseline)
        root = request.evidence_root
        if root.exists():
            raise FileExistsError("replay evidence root must not exist")
        root.mkdir(parents=True)
        report = root / "report.json"
        diff = root / "diff.json"
        argv = [
            str(self._toolchain.runner),
            "--case", str(request.case_manifest),
            "--data-root", str(request.data_root),
            "--config-root", str(request.config_root),
            "--output-root", str(root / "runtime-output"),
            "--report", str(report),
            "--git-commit", source.source_identity,
            "--container-digest", request.container_digest,
        ]
        if source.dirty:
            argv.append("--git-dirty")
        runner = run_logged(
            argv,
            log_path=root / "runner.log",
            timeout_seconds=request.timeout_seconds,
            log_limit_bytes=request.log_limit_bytes,
            environment=safe_environment(),
        )
        baseline_after_runner = digest_file(request.baseline)
        if baseline_after_runner != baseline_before:
            return ReplayResult(ReplayStatus.INVALID, report if report.exists() else None, None, digest_file(report) if report.exists() else None, baseline_before, None, runner.returncode, None, (), "baseline was mutated")
        if runner.timed_out:
            return ReplayResult(ReplayStatus.FAILED, None, None, None, baseline_before, None, -1, None, (), "runner timed out")
        if runner.returncode == 77:
            return ReplayResult(ReplayStatus.NOT_AVAILABLE, None, None, None, baseline_before, None, 77, None, (), "replay data/tool unavailable")
        if runner.returncode != 0 or not report.is_file():
            return ReplayResult(ReplayStatus.FAILED, report if report.exists() else None, None, digest_file(report) if report.exists() else None, baseline_before, None, runner.returncode, None, (), "runner failed")
        comparator_argv = [str(self._toolchain.comparator), "--baseline", str(request.baseline), "--report", str(report), "--diff", str(diff)]
        comparator = run_logged(
            comparator_argv,
            log_path=root / "comparator.log",
            timeout_seconds=request.timeout_seconds,
            log_limit_bytes=request.log_limit_bytes,
            environment=safe_environment(),
        )
        if comparator.timed_out:
            return ReplayResult(ReplayStatus.FAILED, report, None, digest_file(report), baseline_before, None, runner.returncode, -1, (), "comparator timed out")
        baseline_after = digest_file(request.baseline)
        if baseline_after != baseline_before:
            return ReplayResult(ReplayStatus.INVALID, report, diff if diff.exists() else None, digest_file(report), baseline_before, digest_file(diff) if diff.exists() else None, runner.returncode, comparator.returncode, (), "baseline was mutated")
        document = load_closed_json(report)
        try:
            document = require_keys(
                document,
                required={
                    "schema_version", "case_id", "case_manifest_sha256",
                    "dataset_sha256", "config_sha256", "git", "environment",
                    "agents", "steps", "health", "metrics", "artifacts",
                    "diagnostics", "close_result",
                },
                where="replay report",
            )
        except ValueError:
            return ReplayResult(ReplayStatus.INVALID, report, diff if diff.exists() else None, digest_file(report), baseline_before, digest_file(diff) if diff.exists() else None, runner.returncode, comparator.returncode, (), "unsupported replay report schema")
        if document["schema_version"] != 1:
            return ReplayResult(ReplayStatus.INVALID, report, diff if diff.exists() else None, digest_file(report), baseline_before, digest_file(diff) if diff.exists() else None, runner.returncode, comparator.returncode, (), "unsupported replay report schema")
        metrics = tuple(_flatten_metrics(document.get("metrics", {})))
        status = ReplayStatus.SUCCEEDED if comparator.returncode == 0 else (ReplayStatus.NOT_AVAILABLE if comparator.returncode == 77 else ReplayStatus.FAILED)
        return ReplayResult(status, report, diff if diff.exists() else None, digest_file(report), baseline_before, digest_file(diff) if diff.exists() else None, runner.returncode, comparator.returncode, metrics)


__all__ = ["ReplayAdapter", "ReplayRequest", "ReplayResult", "ReplayStatus", "ReplayToolchain"]
