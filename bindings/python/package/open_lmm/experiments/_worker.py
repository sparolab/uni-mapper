from __future__ import annotations

import os
import random
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import open_lmm

from ._canonical import canonical_json_bytes, load_closed_json, require_keys
from ._config import verify_dataset
from ._export import _plain
from ._metrics import (
    EventAccumulator,
    alignment_records,
    output_records,
    process_rss_record,
    snapshot_records,
    visualization_records,
)
from ._models import DatasetSpec, InputLockKind, MetricPolicy, MetricRecord, MetricSource


def _error_payload(error: BaseException) -> dict[str, Any]:
    payload: dict[str, Any] = {"kind": "python", "type": type(error).__name__, "message": str(error)}
    if isinstance(error, open_lmm.OpenLMMError):
        payload["kind"] = "open_lmm"
        payload["code"] = int(error.code)
        payload["severity"] = int(error.severity)
        context = _plain(error.context)
        config = context.get("config", "")
        if config and Path(config).is_absolute():
            context["config"] = f"<absolute-path-redacted>/{Path(config).name}"
        payload["context"] = context
    return payload


def run_worker(request: dict[str, Any]) -> dict[str, Any]:
    random.seed(request["seed"])
    np.random.seed(request["seed"] & 0xFFFFFFFF)
    dataset = request["dataset"]
    dataset_spec = DatasetSpec(
        dataset["id"],
        Path(dataset["root"]),
        InputLockKind(dataset["lock_kind"]),
        Path(dataset["lock_manifest"]),
        dataset["lock_sha256"],
    )
    input_reverified = False
    if request["strict_reproducibility"]:
        verify_dataset(dataset_spec)
        input_reverified = True
    metric_policy = MetricPolicy(**request["metrics"])
    accumulator = EventAccumulator(request["event_retention"])
    runtime = open_lmm.Runtime(1)
    subscription = None
    before = None
    after = None
    metrics = []
    status = "failed"
    error_payload = None
    started = time.monotonic_ns()
    try:
        runtime.open(
            request["config_directory"],
            label=request["trial_id"],
            output_root=request["output_directory"],
        )
        subscription = runtime.subscribe_events(accumulator)
        before = runtime.snapshot().pipeline.runtime_revision
        workflow = request["workflow"]
        if workflow["kind"] == "run-all":
            runtime.run_all().wait()
        else:
            for stage in workflow["stages"]:
                runtime.run_stage(open_lmm.Stage(stage)).wait()
        snapshot = runtime.snapshot()
        after = snapshot.pipeline.runtime_revision
        metrics.extend(snapshot_records(snapshot))
        if metric_policy.include_visualization:
            visualizations = []
            for agent in snapshot.pipeline.agents:
                visualization = runtime.visualization(
                    agent,
                    include_points=metric_policy.include_points,
                    preview_voxel_size_m=metric_policy.preview_voxel_size_m,
                )
                visualizations.append(visualization)
                metrics.extend(visualization_records(visualization, metric_policy))
            metrics.extend(alignment_records(visualizations))
        status = "succeeded"
    except BaseException as error:
        error_payload = _error_payload(error)
        try:
            snapshot = runtime.snapshot()
            after = snapshot.pipeline.runtime_revision
            metrics.extend(snapshot_records(snapshot))
        except BaseException:
            pass
    finally:
        if subscription is not None:
            try:
                subscription.close()
            except BaseException as close_error:
                if error_payload is None:
                    error_payload = _error_payload(close_error)
                    status = "failed"
        try:
            runtime.close(cancel_running=True)
        except BaseException as close_error:
            if error_payload is None:
                error_payload = _error_payload(close_error)
                status = "failed"
    metrics.extend(accumulator.records())
    metrics.extend(output_records(Path(request["output_directory"]), metric_policy.hash_output_files))
    metrics.append(process_rss_record())
    metrics.append(
        MetricRecord(
            "workflow.total_wall",
            time.monotonic_ns() - started,
            "ns",
            MetricSource.PYTHON_CALLBACK,
            "trial",
        )
    )
    return {
        "schema_version": 1,
        "trial_id": request["trial_id"],
        "trial_index": request["trial_index"],
        "seed": request["seed"],
        "status": status,
        "variant_id": request.get("variant_id"),
        "parameters": request["parameters"],
        "config_sha256": request["config_sha256"],
        "dataset_sha256": dataset_spec.lock_sha256,
        "software_sha256": request["software_sha256"],
        "metrics": [_plain(item) for item in metrics],
        "runtime_revision_before": before,
        "runtime_revision_after": after,
        "error": error_payload,
        "worker_pid": os.getpid(),
        "worker_exit_code": 0,
        "reproducible": bool(request["strict_reproducibility"] and input_reverified),
        "input_reverified": input_reverified,
    }


def main(argv: list[str] | None = None) -> int:
    arguments = sys.argv[1:] if argv is None else argv
    if len(arguments) != 2:
        print("usage: python -m open_lmm.experiments._worker REQUEST RESULT", file=sys.stderr)
        return 2
    request_path, result_path = map(Path, arguments)
    try:
        request = load_closed_json(request_path)
        request = require_keys(
            request,
            required={
                "protocol_version", "trial_id", "trial_index", "seed",
                "variant_id", "parameters", "config_directory",
                "output_directory", "config_sha256", "software_sha256",
                "dataset", "strict_reproducibility", "workflow", "metrics",
                "event_retention",
            },
            where="worker request",
        )
        if request["protocol_version"] != 1:
            raise ValueError("unsupported worker request protocol")
        result = run_worker(dict(request))
        Path(result_path).write_bytes(canonical_json_bytes(result))
        return 0
    except BaseException as error:
        print(f"worker protocol failure: {type(error).__name__}: {error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
