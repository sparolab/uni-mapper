from __future__ import annotations

import csv
import io
from dataclasses import fields, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping

from ._canonical import canonical_json_bytes
from ._models import (
    ExperimentResult,
    MetricAvailability,
    MetricRecord,
    MetricSource,
    TrialResult,
)


CSV_COLUMNS = (
    "experiment_id",
    "trial_id",
    "trial_index",
    "seed",
    "status",
    "parameter_json",
    "variant_id",
    "metric",
    "unit",
    "value",
    "source",
    "scope",
    "subject",
    "availability",
    "reason",
)


def _plain(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {item.name: _plain(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    if isinstance(value, Path):
        return value.as_posix()
    return value


def result_to_dict(result: ExperimentResult) -> dict[str, Any]:
    return {"schema_version": 1, **_plain(result)}


def metric_from_dict(value: Mapping[str, Any]) -> MetricRecord:
    return MetricRecord(
        value["name"],
        value.get("value"),
        value["unit"],
        MetricSource(value["source"]),
        value["scope"],
        value.get("subject"),
        MetricAvailability(value.get("availability", "available")),
        value.get("reason", ""),
    )


def trial_from_dict(value: Mapping[str, Any]) -> TrialResult:
    return TrialResult(
        value["trial_id"],
        value["trial_index"],
        value["seed"],
        value["status"],
        value.get("variant_id"),
        value.get("parameters", {}),
        value["config_sha256"],
        value["dataset_sha256"],
        value["software_sha256"],
        tuple(metric_from_dict(item) for item in value.get("metrics", [])),
        value.get("runtime_revision_before"),
        value.get("runtime_revision_after"),
        value.get("error"),
        value.get("worker_pid"),
        value.get("worker_exit_code"),
        bool(value.get("reproducible", False)),
        bool(value.get("input_reverified", False)),
    )


def csv_bytes(result: ExperimentResult) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for raw in result.records():
        row = dict(raw)
        for key in ("parameter_json", "value"):
            item = row[key]
            if isinstance(item, (dict, list, tuple)):
                row[key] = canonical_json_bytes(item).decode("utf-8").rstrip("\n")
        writer.writerow(row)
    return stream.getvalue().encode("utf-8")
