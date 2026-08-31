from __future__ import annotations

import math
import os
import resource
import statistics
import time
from collections import Counter, deque
from pathlib import Path
from threading import Lock
from typing import Iterable

import numpy as np
from open_lmm import ArtifactState, EventType, ExecutionEvent, RuntimeSnapshot, VisualizationSnapshot

from ._canonical import digest_file
from ._models import (
    MetricAvailability,
    MetricPolicy,
    MetricRecord,
    MetricSource,
    MetricSummary,
    TrialResult,
)


class EventAccumulator:
    __slots__ = ("_lock", "_recent", "_count", "_dropped", "_starts", "_latencies", "_mismatches")

    def __init__(self, retention: int) -> None:
        self._lock = Lock()
        self._recent: deque[tuple[int, int, int | None, int]] = deque(maxlen=retention)
        self._count: Counter[int] = Counter()
        self._dropped = 0
        self._starts: dict[int, int] = {}
        self._latencies: dict[int, list[int]] = {}
        self._mismatches = 0

    def __call__(self, event: ExecutionEvent) -> None:
        now = time.monotonic_ns()
        with self._lock:
            self._count[int(event.type)] += 1
            if len(self._recent) == self._recent.maxlen:
                self._dropped += 1
            self._recent.append((now, int(event.type), None if event.stage is None else int(event.stage), event.sequence))
            if event.stage is not None and event.type == EventType.STAGE_STARTED:
                self._starts[int(event.stage)] = now
            if event.stage is not None and event.type in {EventType.STAGE_COMPLETED, EventType.STAGE_FAILED}:
                started = self._starts.pop(int(event.stage), None)
                if started is not None:
                    self._latencies.setdefault(int(event.stage), []).append(now - started)
                else:
                    self._mismatches += 1

    def records(self) -> tuple[MetricRecord, ...]:
        with self._lock:
            records = [
                MetricRecord("workflow.event_count", sum(self._count.values()), "count", MetricSource.PYTHON_CALLBACK, "trial"),
                MetricRecord("workflow.event_samples_dropped", self._dropped, "count", MetricSource.PYTHON_CALLBACK, "trial"),
            ]
            for stage, values in sorted(self._latencies.items()):
                for index, value in enumerate(values):
                    records.append(
                        MetricRecord(
                            "workflow.stage_callback_wall",
                            value,
                            "ns",
                            MetricSource.PYTHON_CALLBACK,
                            "stage",
                            f"{stage}:{index}",
                        )
                    )
            for stage in sorted(self._starts):
                records.append(
                    MetricRecord(
                        "workflow.stage_callback_wall",
                        None,
                        "ns",
                        MetricSource.PYTHON_CALLBACK,
                        "stage",
                        str(stage),
                        MetricAvailability.INVALID,
                        "stage started without a terminal callback",
                    )
                )
            if self._mismatches:
                records.append(
                    MetricRecord(
                        "workflow.stage_callback_wall",
                        None,
                        "ns",
                        MetricSource.PYTHON_CALLBACK,
                        "stage",
                        None,
                        MetricAvailability.INVALID,
                        f"{self._mismatches} terminal callbacks had no matching start",
                    )
                )
            return tuple(records)


def snapshot_records(snapshot: RuntimeSnapshot) -> tuple[MetricRecord, ...]:
    states = Counter(item.state for item in snapshot.pipeline.artifacts)
    records = [
        MetricRecord("runtime.runtime_revision", snapshot.pipeline.runtime_revision, "revision", MetricSource.SNAPSHOT, "trial"),
        MetricRecord("runtime.config_revision", snapshot.pipeline.config_revision, "revision", MetricSource.SNAPSHOT, "trial"),
        MetricRecord("runtime.agent_count", len(snapshot.pipeline.agents), "count", MetricSource.SNAPSHOT, "trial"),
        MetricRecord("artifact.count", len(snapshot.pipeline.artifacts), "count", MetricSource.SNAPSHOT, "trial"),
    ]
    for state in ArtifactState:
        records.append(
            MetricRecord(
                "artifact.state_count",
                states[state],
                "count",
                MetricSource.SNAPSHOT,
                "artifact",
                state.name.lower(),
            )
        )
    return tuple(records)


def visualization_records(value: VisualizationSnapshot, policy: MetricPolicy) -> tuple[MetricRecord, ...]:
    records: list[MetricRecord] = [
        MetricRecord("trajectory.pose_count", int(value.poses.shape[0]), "count", MetricSource.VISUALIZATION, "agent", value.agent),
        MetricRecord("map.source_point_count", value.source_point_count, "count", MetricSource.VISUALIZATION, "agent", value.agent),
        MetricRecord("map.displayed_point_count", value.displayed_point_count, "count", MetricSource.VISUALIZATION, "agent", value.agent),
        MetricRecord("map.points_complete", value.points_complete, "bool", MetricSource.VISUALIZATION, "agent", value.agent),
    ]
    poses = value.poses
    if len(poses) > 1:
        translations = poses[:, :3, 3]
        deltas = translations[1:] - translations[:-1]
        path_length = float(((deltas * deltas).sum(axis=1) ** 0.5).sum())
        records.append(MetricRecord("trajectory.path_length", path_length, "m", MetricSource.VISUALIZATION, "agent", value.agent))
    elif len(poses) == 1:
        records.append(MetricRecord("trajectory.path_length", 0.0, "m", MetricSource.VISUALIZATION, "agent", value.agent))
    else:
        records.append(
            MetricRecord("trajectory.path_length", None, "m", MetricSource.VISUALIZATION, "agent", value.agent, MetricAvailability.NOT_AVAILABLE, "no poses")
        )
    if len(poses):
        translations = poses[:, :3, 3]
        extent = translations.max(axis=0) - translations.min(axis=0)
        records.extend(
            MetricRecord("trajectory.aabb_extent", float(extent[index]), "m", MetricSource.VISUALIZATION, "agent", f"{value.agent}:{axis}")
            for index, axis in enumerate(("x", "y", "z"))
        )
    if value.has_bounds:
        extent = value.max_bound - value.min_bound
        records.extend(
            MetricRecord("map.bounds_extent", float(extent[index]), "m", MetricSource.VISUALIZATION, "agent", f"{value.agent}:{axis}")
            for index, axis in enumerate(("x", "y", "z"))
        )
        records.append(MetricRecord("map.bounds_volume", float(extent[0] * extent[1] * extent[2]), "m3", MetricSource.VISUALIZATION, "agent", value.agent))
    else:
        records.append(MetricRecord("map.bounds_volume", None, "m3", MetricSource.VISUALIZATION, "agent", value.agent, MetricAvailability.NOT_AVAILABLE, "visualization has no bounds"))
    if policy.include_points:
        points = value.points
        if points is None or len(points) == 0:
            records.append(MetricRecord("map.point_finite_count", None, "count", MetricSource.VISUALIZATION, "agent", value.agent, MetricAvailability.NOT_AVAILABLE, "point preview unavailable"))
        else:
            mask = np.isfinite(points).all(axis=1)
            finite = int(mask.sum())
            records.append(MetricRecord("map.point_finite_count", finite, "count", MetricSource.VISUALIZATION, "agent", value.agent))
            if finite:
                centroid = [
                    float(np.sum(points[:, index], where=mask) / finite)
                    for index in range(3)
                ]
                records.extend(
                    MetricRecord("map.point_centroid", centroid[index], "m", MetricSource.VISUALIZATION, "agent", f"{value.agent}:{axis}")
                    for index, axis in enumerate(("x", "y", "z"))
                )
                records.append(MetricRecord("map.intensity_min", float(np.min(points[:, 3], where=mask, initial=np.inf)), "intensity", MetricSource.VISUALIZATION, "agent", value.agent))
                records.append(MetricRecord("map.intensity_max", float(np.max(points[:, 3], where=mask, initial=-np.inf)), "intensity", MetricSource.VISUALIZATION, "agent", value.agent))
    return tuple(records)


def alignment_records(values: Iterable[VisualizationSnapshot]) -> tuple[MetricRecord, ...]:
    keys: set[tuple[str, int, str, int, int]] = set()
    for value in values:
        keys.update(
            (edge.from_agent, edge.from_index, edge.to_agent, edge.to_index, int(edge.type))
            for edge in value.edges
        )
    type_counts = Counter(key[4] for key in keys)
    inter = [key for key in keys if key[0] != key[2]]
    affected = sorted({agent for key in keys for agent in (key[0], key[2])})
    records = [
        MetricRecord("alignment.edge_count", len(keys), "count", MetricSource.VISUALIZATION, "trial"),
        MetricRecord("alignment.inter_agent_edge_count", len(inter), "count", MetricSource.VISUALIZATION, "trial"),
        MetricRecord("alignment.affected_agent_count", len(affected), "count", MetricSource.VISUALIZATION, "trial"),
    ]
    records.extend(
        MetricRecord("alignment.edge_type_count", count, "count", MetricSource.VISUALIZATION, "artifact", str(kind))
        for kind, count in sorted(type_counts.items())
    )
    return tuple(records)


def output_records(root: Path, hash_files: bool) -> tuple[MetricRecord, ...]:
    count = 0
    total = 0
    records: list[MetricRecord] = []
    if root.exists():
        for path in sorted(root.rglob("*")):
            if path.is_symlink():
                records.append(MetricRecord("output.file", None, "bytes", MetricSource.OUTPUT_FILE, "artifact", path.name, MetricAvailability.INVALID, "symlink output rejected"))
            elif path.is_file():
                count += 1
                size = path.stat().st_size
                total += size
                subject = path.relative_to(root).as_posix()
                records.append(MetricRecord("output.file_bytes", size, "bytes", MetricSource.OUTPUT_FILE, "artifact", subject))
                if hash_files:
                    records.append(MetricRecord("output.file_sha256", digest_file(path), "sha256", MetricSource.OUTPUT_FILE, "artifact", subject))
    records.extend(
        [
            MetricRecord("output.file_count", count, "count", MetricSource.OUTPUT_FILE, "trial"),
            MetricRecord("output.logical_bytes", total, "bytes", MetricSource.OUTPUT_FILE, "trial"),
        ]
    )
    return tuple(records)


def process_rss_record() -> MetricRecord:
    try:
        value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        bytes_value = int(value * 1024) if os.uname().sysname == "Linux" else int(value)
        return MetricRecord("process.max_rss", bytes_value, "bytes", MetricSource.PROCESS, "trial")
    except (AttributeError, OSError) as error:
        return MetricRecord("process.max_rss", None, "bytes", MetricSource.PROCESS, "trial", availability=MetricAvailability.NOT_AVAILABLE, reason=str(error))


def summarize(trials: Iterable[TrialResult]) -> tuple[MetricSummary, ...]:
    trial_list = tuple(trials)
    groups: dict[tuple[str, str], list[float]] = {}
    missing: Counter[tuple[str, str]] = Counter()
    for trial in trial_list:
        seen: set[tuple[str, str]] = set()
        if trial.status == "succeeded":
            for metric in trial.metrics:
                key = (metric.name, metric.unit)
                seen.add(key)
                if metric.availability == MetricAvailability.AVAILABLE and isinstance(metric.value, (int, float)) and not isinstance(metric.value, bool):
                    groups.setdefault(key, []).append(float(metric.value))
                else:
                    missing[key] += 1
    result: list[MetricSummary] = []
    failures = sum(trial.status != "succeeded" for trial in trial_list)
    for key in sorted(set(groups) | set(missing)):
        values = sorted(groups.get(key, []))
        if values:
            median = statistics.median(values)
            p95 = values[max(0, math.ceil(0.95 * len(values)) - 1)]
            deviations = [abs(value - median) for value in values]
            mad = statistics.median(deviations)
            minimum, maximum = values[0], values[-1]
        else:
            median = p95 = mad = minimum = maximum = None
        result.append(MetricSummary(key[0], key[1], len(values), missing[key], failures, median, p95, mad, minimum, maximum))
    return tuple(result)
