from __future__ import annotations

import os
import platform
import sys
from pathlib import Path
from typing import Any

import numpy as np

from ._canonical import atomic_write, canonical_json_bytes, digest_file, digest_value, load_closed_json, require_keys
from ._config import materialize_config, verify_config, verify_dataset
from ._export import csv_bytes, result_to_dict, trial_from_dict
from ._manifest import plan_identity
from ._metrics import summarize
from ._models import (
    ExperimentPlan,
    ExperimentResult,
    ExperimentStatus,
    ExecutionMode,
    FailurePolicy,
    TrialResult,
)
from ._planner import PlannedTrial, plan_trials
from ._subprocess import ProcessOutcome, run_logged, safe_environment


def _software_digest(plan: ExperimentPlan) -> str:
    value = plan_identity(plan)["software"]
    return digest_value(value)


def _request(
    plan: ExperimentPlan,
    trial: PlannedTrial,
    *,
    config_directory: Path,
    output_directory: Path,
    config_sha256: str,
) -> dict[str, Any]:
    return {
        "protocol_version": 1,
        "trial_id": trial.trial_id,
        "trial_index": trial.ordinal,
        "seed": trial.seed,
        "variant_id": trial.variant_id,
        "parameters": dict(trial.parameters),
        "config_directory": str(config_directory),
        "output_directory": str(output_directory),
        "config_sha256": config_sha256,
        "software_sha256": _software_digest(plan),
        "dataset": {
            "id": plan.dataset.id,
            "root": str(plan.dataset.root),
            "lock_kind": plan.dataset.lock_kind.value,
            "lock_manifest": str(plan.dataset.lock_manifest),
            "lock_sha256": plan.dataset.lock_sha256,
        },
        "strict_reproducibility": plan.execution.strict_reproducibility,
        "workflow": {
            "kind": plan.workflow.kind.value,
            "stages": [int(item) for item in plan.workflow.stages],
        },
        "metrics": {
            "include_visualization": plan.metrics.include_visualization,
            "include_points": plan.metrics.include_points,
            "preview_voxel_size_m": plan.metrics.preview_voxel_size_m,
            "hash_output_files": plan.metrics.hash_output_files,
        },
        "event_retention": plan.execution.event_retention,
    }


def _timeout_trial(plan: ExperimentPlan, trial: PlannedTrial, config_sha256: str, pid: int) -> TrialResult:
    return TrialResult(
        trial.trial_id,
        trial.ordinal,
        trial.seed,
        "timed_out",
        trial.variant_id,
        trial.parameters,
        config_sha256,
        plan.dataset.lock_sha256,
        _software_digest(plan),
        (),
        error={"kind": "timeout", "message": f"worker exceeded {plan.execution.timeout_seconds} seconds"},
        worker_pid=pid,
        worker_exit_code=-9,
        reproducible=False,
        input_reverified=False,
    )


def _failed_trial(
    plan: ExperimentPlan,
    trial: PlannedTrial,
    config_sha256: str,
    *,
    pid: int | None,
    exit_code: int | None,
    message: str,
) -> TrialResult:
    return TrialResult(
        trial.trial_id,
        trial.ordinal,
        trial.seed,
        "invalid",
        trial.variant_id,
        trial.parameters,
        config_sha256,
        plan.dataset.lock_sha256,
        _software_digest(plan),
        (),
        error={"kind": "worker_protocol", "message": message},
        worker_pid=pid,
        worker_exit_code=exit_code,
        reproducible=False,
        input_reverified=False,
    )


def _run_trial(plan: ExperimentPlan, trial: PlannedTrial, directory: Path) -> TrialResult:
    config_directory = directory / "config"
    output_directory = directory / "runtime-output"
    config_sha256, _ = materialize_config(plan.config, plan.dataset, config_directory, trial.patches)
    request_path = directory / ".worker-request.json"
    response_path = directory / ".worker-response.json"
    request_path.write_bytes(
        canonical_json_bytes(
            _request(
                plan,
                trial,
                config_directory=config_directory,
                output_directory=output_directory,
                config_sha256=config_sha256,
            )
        )
    )
    if plan.execution.mode == ExecutionMode.IN_PROCESS:
        from ._worker import run_worker

        document = run_worker(load_closed_json(request_path))
        response_path.write_bytes(canonical_json_bytes(document))
        (directory / "worker.log").write_bytes(b"")
        outcome = ProcessOutcome(0, os.getpid(), False, 0, False)
    else:
        command = [sys.executable, "-m", "open_lmm.experiments._worker", str(request_path), str(response_path)]
        outcome = run_logged(
            command,
            log_path=directory / "worker.log",
            timeout_seconds=plan.execution.timeout_seconds,
            log_limit_bytes=plan.execution.log_limit_bytes,
            environment=safe_environment(),
        )
    exit_code = outcome.returncode
    request_path.unlink(missing_ok=True)
    if outcome.timed_out:
        result = _timeout_trial(plan, trial, config_sha256, outcome.pid)
    elif exit_code != 0 or not response_path.is_file():
        result = _failed_trial(
            plan,
            trial,
            config_sha256,
            pid=outcome.pid,
            exit_code=exit_code,
            message="worker exited without a valid result",
        )
    else:
        try:
            document = load_closed_json(response_path)
            document = dict(
                require_keys(
                    document,
                    required={
                        "schema_version", "trial_id", "trial_index", "seed",
                        "status", "variant_id", "parameters", "config_sha256",
                        "dataset_sha256", "software_sha256", "metrics",
                        "runtime_revision_before", "runtime_revision_after",
                        "error", "worker_pid", "worker_exit_code",
                        "reproducible", "input_reverified",
                    },
                    where="worker response",
                )
            )
            if document["schema_version"] != 1:
                raise ValueError("unsupported worker response schema")
            document["worker_pid"] = outcome.pid
            document["worker_exit_code"] = exit_code
            result = trial_from_dict(document)
        except (KeyError, TypeError, ValueError) as error:
            result = _failed_trial(
                plan,
                trial,
                config_sha256,
                pid=outcome.pid,
                exit_code=exit_code,
                message=str(error),
            )
    response_path.unlink(missing_ok=True)
    atomic_write(directory / "trial-result.json", canonical_json_bytes({"schema_version": 1, **_trial_dict(result)}))
    return result


def _trial_dict(result: TrialResult) -> dict[str, Any]:
    wrapper = ExperimentResult(
        "trial-wrapper",
        "sha256:" + "0" * 64,
        (result,),
        (),
        (),
        ExperimentStatus.SUCCEEDED,
    )
    return result_to_dict(wrapper)["trials"][0]


def _environment_manifest(plan: ExperimentPlan, plan_digest: str) -> dict[str, Any]:
    affinity: list[int] | None
    try:
        affinity = sorted(os.sched_getaffinity(0))
    except AttributeError:
        affinity = None
    return {
        "schema_version": 1,
        "plan_sha256": plan_digest,
        "reproducible": plan.execution.strict_reproducibility and plan.software.strict_reproducible(),
        "python": {"implementation": platform.python_implementation(), "version": platform.python_version()},
        "open_lmm": {
            "version": plan.software.open_lmm_version,
            "runtime_api_version": plan.software.runtime_api_version,
            "experiment_api_version": plan.software.experiment_api_version,
        },
        "numpy_version": np.__version__,
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "cpu_count": os.cpu_count(),
            "cpu_affinity": affinity,
        },
        "thread_limits": {key: os.environ[key] for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS") if key in os.environ},
        "container_digest": plan.software.container_digest,
        "execution_mode": plan.execution.mode.value,
    }


def _status(trials: tuple[TrialResult, ...]) -> ExperimentStatus:
    statuses = {trial.status for trial in trials}
    if statuses == {"succeeded"}:
        return ExperimentStatus.SUCCEEDED
    if "timed_out" in statuses:
        return ExperimentStatus.TIMED_OUT
    if "succeeded" in statuses:
        return ExperimentStatus.PARTIAL
    if "invalid" in statuses:
        return ExperimentStatus.INVALID
    return ExperimentStatus.FAILED


def _write_evidence_index(root: Path) -> None:
    lines = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.name != "evidence.sha256":
            lines.append(f"{digest_file(path).split(':', 1)[1]}  {path.relative_to(root).as_posix()}\n")
    atomic_write(root / "evidence.sha256", "".join(lines).encode("utf-8"))


def run_experiment(plan: ExperimentPlan, evidence_root: Path) -> ExperimentResult:
    plan_digest, planned = plan_trials(plan)
    if plan.execution.strict_reproducibility and not plan.software.strict_reproducible():
        raise ValueError("strict reproducibility requires a clean immutable software identity")
    verify_config(plan.config)
    verify_dataset(plan.dataset)
    if evidence_root.exists():
        raise FileExistsError("evidence root must not already exist")
    evidence_root.mkdir(parents=True)
    atomic_write(evidence_root / "plan.json", canonical_json_bytes(plan_identity(plan)))
    atomic_write(evidence_root / "manifest.json", canonical_json_bytes(_environment_manifest(plan, plan_digest)))
    trials_root = evidence_root / "trials"
    trials_root.mkdir()
    completed: list[TrialResult] = []
    for trial in planned:
        directory = trials_root / f"{trial.ordinal:04d}-{trial.trial_id[:12]}"
        directory.mkdir()
        result = _run_trial(plan, trial, directory)
        completed.append(result)
        if result.status != "succeeded" and plan.execution.failure_policy == FailurePolicy.STOP:
            break
    trial_results = tuple(completed)
    result = ExperimentResult(
        plan.id,
        plan_digest,
        trial_results,
        (),
        summarize(trial_results),
        _status(trial_results),
    )
    atomic_write(evidence_root / "experiment-result.json", canonical_json_bytes(result_to_dict(result)))
    atomic_write(evidence_root / "metrics.csv", csv_bytes(result))
    _write_evidence_index(evidence_root)
    return result
