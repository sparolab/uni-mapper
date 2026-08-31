from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, TypeAlias

from open_lmm import API_VERSION, Stage, __version__

from ._canonical import (
    atomic_write,
    canonical_json_bytes,
    canonical_value,
    plain_value,
    require_relative_path,
    require_sha256,
)


JsonValue: TypeAlias = None | bool | int | float | str | tuple["JsonValue", ...] | Mapping[str, "JsonValue"]
EXPERIMENT_API_VERSION = 1
_SLUG = re.compile(r"^[a-z0-9][a-z0-9._-]*$")


def _slug(value: str, where: str) -> str:
    if not isinstance(value, str) or not _SLUG.fullmatch(value):
        raise ValueError(f"{where} must be a non-empty lowercase slug")
    return value


class InputLockKind(str, Enum):
    REPLAY_CASE_V1 = "replay-case-v1"
    SHA256_INDEX_V1 = "sha256-index-v1"


class WorkflowKind(str, Enum):
    RUN_ALL = "run-all"
    STAGES = "stages"


class ExecutionMode(str, Enum):
    FRESH_PROCESS = "fresh-process"
    IN_PROCESS = "in-process"


class FailurePolicy(str, Enum):
    CONTINUE = "continue"
    STOP = "stop"


class AlgorithmSlot(str, Enum):
    DATA_LOADER = "data_loader"
    LOOP_DETECTOR = "loop_detector"
    BACKEND_OPTIMIZER = "backend_optimizer"
    DYNAMIC_REMOVER = "dynamic_remover"


class MetricAvailability(str, Enum):
    AVAILABLE = "available"
    NOT_AVAILABLE = "not_available"
    INVALID = "invalid"


class MetricSource(str, Enum):
    PYTHON_CALLBACK = "python-callback"
    SNAPSHOT = "snapshot"
    VISUALIZATION = "visualization"
    OUTPUT_FILE = "output-file"
    REPLAY_V1 = "replay-v1"
    BENCHMARK_V1 = "benchmark-v1"
    PROCESS = "process"


class ExperimentStatus(str, Enum):
    SUCCEEDED = "succeeded"
    PARTIAL = "partial"
    FAILED = "failed"
    TIMED_OUT = "timed_out"
    INVALID = "invalid"


@dataclass(frozen=True, slots=True)
class LockedFile:
    path: str
    sha256: str
    size: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "path", require_relative_path(self.path))
        object.__setattr__(self, "sha256", require_sha256(self.sha256))
        if self.size is not None and (not isinstance(self.size, int) or self.size < 0):
            raise ValueError("locked file size must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class ConfigTarget:
    file: str
    pointer: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "file", require_relative_path(self.file))
        if not isinstance(self.pointer, str) or not self.pointer.startswith("/"):
            raise ValueError("JSON Pointer must start with '/'")


@dataclass(frozen=True, slots=True)
class ConfigPatch:
    file: str
    pointer: str
    value: JsonValue

    def __post_init__(self) -> None:
        target = ConfigTarget(self.file, self.pointer)
        object.__setattr__(self, "file", target.file)
        object.__setattr__(self, "pointer", target.pointer)
        object.__setattr__(self, "value", canonical_value(self.value, where="patch value"))


@dataclass(frozen=True, slots=True)
class ParameterAxis:
    name: str
    target: ConfigTarget
    values: tuple[JsonValue, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _slug(self.name, "parameter axis name"))
        if not self.values:
            raise ValueError("parameter axis values must not be empty")
        object.__setattr__(self, "values", tuple(canonical_value(v) for v in self.values))


@dataclass(frozen=True, slots=True)
class AlgorithmVariant:
    id: str
    slot: AlgorithmSlot
    patches: tuple[ConfigPatch, ...] = ()
    expected_plugin_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _slug(self.id, "algorithm variant id"))
        object.__setattr__(self, "slot", AlgorithmSlot(self.slot))
        object.__setattr__(self, "patches", tuple(self.patches))
        plugin_ids = tuple(_slug(item, "plugin id") for item in self.expected_plugin_ids)
        if len(plugin_ids) != len(set(plugin_ids)):
            raise ValueError("expected plugin IDs must be unique")
        object.__setattr__(self, "expected_plugin_ids", plugin_ids)


@dataclass(frozen=True, slots=True)
class DatasetSpec:
    id: str
    root: Path
    lock_kind: InputLockKind
    lock_manifest: Path
    lock_sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _slug(self.id, "dataset id"))
        object.__setattr__(self, "root", Path(self.root).absolute())
        object.__setattr__(self, "lock_kind", InputLockKind(self.lock_kind))
        object.__setattr__(self, "lock_manifest", Path(self.lock_manifest).absolute())
        object.__setattr__(self, "lock_sha256", require_sha256(self.lock_sha256))


@dataclass(frozen=True, slots=True)
class ConfigSpec:
    directory: Path
    files: tuple[LockedFile, ...]
    dataset_bindings: tuple[ConfigTarget, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "directory", Path(self.directory).absolute())
        object.__setattr__(self, "files", tuple(self.files))
        object.__setattr__(self, "dataset_bindings", tuple(self.dataset_bindings))
        paths = [item.path for item in self.files]
        if not paths or len(paths) != len(set(paths)):
            raise ValueError("config files must be non-empty and unique")
        bindings = [(item.file, item.pointer) for item in self.dataset_bindings]
        if len(bindings) != len(set(bindings)):
            raise ValueError("dataset bindings must be unique")
        if any(item.file not in set(paths) for item in self.dataset_bindings):
            raise ValueError("dataset binding file must be declared")


@dataclass(frozen=True, slots=True)
class SoftwareIdentity:
    open_lmm_version: str = __version__
    runtime_api_version: int = API_VERSION
    experiment_api_version: int = EXPERIMENT_API_VERSION
    source_kind: str = "distribution"
    source_identity: str = ""
    dirty: bool = False
    container_digest: str | None = None

    def __post_init__(self) -> None:
        if self.runtime_api_version != API_VERSION:
            raise ValueError("runtime API version does not match installed OpenLMM")
        if self.experiment_api_version != EXPERIMENT_API_VERSION:
            raise ValueError("experiment API version does not match this package")
        if self.open_lmm_version != __version__:
            raise ValueError("OpenLMM version does not match installed distribution")
        if self.source_kind not in {"git", "wheel", "distribution", "container"}:
            raise ValueError("unsupported software source kind")
        if self.source_kind == "git" and not re.fullmatch(r"[0-9a-f]{40}", self.source_identity):
            raise ValueError("git software identity must be a 40-character lowercase commit")
        if self.source_kind in {"wheel", "distribution"} and self.source_identity:
            require_sha256(self.source_identity, where="software artifact sha256")
        if self.container_digest is not None:
            require_sha256(self.container_digest, where="container digest")

    def strict_reproducible(self) -> bool:
        if self.dirty:
            return False
        if self.source_kind == "git":
            return bool(self.source_identity)
        if self.source_kind in {"wheel", "distribution"}:
            return bool(self.source_identity)
        return self.container_digest is not None


@dataclass(frozen=True, slots=True)
class WorkflowSpec:
    kind: WorkflowKind = WorkflowKind.RUN_ALL
    stages: tuple[Stage, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", WorkflowKind(self.kind))
        object.__setattr__(self, "stages", tuple(Stage(stage) for stage in self.stages))
        if self.kind == WorkflowKind.RUN_ALL and self.stages:
            raise ValueError("run-all workflow must not list stages")
        if self.kind == WorkflowKind.STAGES and not self.stages:
            raise ValueError("stages workflow requires at least one stage")


@dataclass(frozen=True, slots=True)
class ExecutionPolicy:
    mode: ExecutionMode = ExecutionMode.FRESH_PROCESS
    repetitions: int = 1
    seeds: tuple[int, ...] = ()
    timeout_seconds: float = 900.0
    failure_policy: FailurePolicy = FailurePolicy.CONTINUE
    strict_reproducibility: bool = True
    max_trials: int = 64
    event_retention: int = 256
    log_limit_bytes: int = 1024 * 1024

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", ExecutionMode(self.mode))
        object.__setattr__(self, "failure_policy", FailurePolicy(self.failure_policy))
        object.__setattr__(self, "seeds", tuple(self.seeds))
        if self.repetitions < 1 or self.max_trials < 1 or self.event_retention < 1:
            raise ValueError("execution counts and limits must be positive")
        if self.timeout_seconds <= 0 or self.log_limit_bytes < 1024:
            raise ValueError("timeout and log limit must be positive")
        if self.seeds and len(self.seeds) != self.repetitions:
            raise ValueError("seeds must be empty or match repetitions")
        if any(not isinstance(seed, int) or isinstance(seed, bool) for seed in self.seeds):
            raise ValueError("seeds must be integers")
        if self.mode == ExecutionMode.IN_PROCESS and self.strict_reproducibility:
            raise ValueError("in-process mode cannot claim strict reproducibility")


@dataclass(frozen=True, slots=True)
class MetricPolicy:
    include_visualization: bool = True
    include_points: bool = False
    preview_voxel_size_m: float | None = None
    hash_output_files: bool = True

    def __post_init__(self) -> None:
        if self.preview_voxel_size_m is not None and self.preview_voxel_size_m <= 0:
            raise ValueError("preview voxel size must be positive")
        if self.include_points and self.preview_voxel_size_m is None:
            raise ValueError("point metrics require an explicit preview voxel size")


@dataclass(frozen=True, slots=True)
class ExperimentPlan:
    id: str
    dataset: DatasetSpec
    config: ConfigSpec
    software: SoftwareIdentity
    workflow: WorkflowSpec = field(default_factory=WorkflowSpec)
    execution: ExecutionPolicy = field(default_factory=ExecutionPolicy)
    metrics: MetricPolicy = field(default_factory=MetricPolicy)
    fixed_patches: tuple[ConfigPatch, ...] = ()
    parameter_axes: tuple[ParameterAxis, ...] = ()
    algorithm_variants: tuple[AlgorithmVariant, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _slug(self.id, "experiment id"))
        object.__setattr__(self, "fixed_patches", tuple(self.fixed_patches))
        object.__setattr__(self, "parameter_axes", tuple(self.parameter_axes))
        object.__setattr__(self, "algorithm_variants", tuple(self.algorithm_variants))
        names = [axis.name for axis in self.parameter_axes]
        variants = [variant.id for variant in self.algorithm_variants]
        if len(names) != len(set(names)) or len(variants) != len(set(variants)):
            raise ValueError("axis and variant IDs must be unique")

    @classmethod
    def simple(
        cls,
        *,
        dataset: DatasetSpec,
        config: ConfigSpec | str | Path,
        software: SoftwareIdentity | None = None,
    ) -> "ExperimentPlan":
        if not isinstance(config, ConfigSpec):
            from ._canonical import digest_file

            directory = Path(config).absolute()
            if not directory.is_dir() or directory.is_symlink():
                raise ValueError("simple() config must be a regular directory")
            discovered = []
            for path in sorted(directory.rglob("*")):
                if path.is_symlink() or (not path.is_file() and not path.is_dir()):
                    raise ValueError("simple() config tree contains a symlink or special file")
                if path.is_file():
                    if path.suffix != ".json":
                        raise ValueError("simple() config tree supports JSON files only")
                    discovered.append(
                        LockedFile(
                            path.relative_to(directory).as_posix(),
                            digest_file(path),
                            path.stat().st_size,
                        )
                    )
            config = ConfigSpec(
                directory,
                tuple(discovered),
                (ConfigTarget("config.json", "/directory/root_dir_path"),),
            )
        return cls("experiment", dataset, config, software or SoftwareIdentity())


@dataclass(frozen=True, slots=True)
class MetricRecord:
    name: str
    value: JsonValue
    unit: str
    source: MetricSource
    scope: str
    subject: str | None = None
    availability: MetricAvailability = MetricAvailability.AVAILABLE
    reason: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "source", MetricSource(self.source))
        object.__setattr__(self, "availability", MetricAvailability(self.availability))
        object.__setattr__(self, "value", canonical_value(self.value, where="metric value"))
        if not (
            self.value is None
            or isinstance(self.value, (bool, int, float, str))
        ):
            raise ValueError("metric value must be a scalar JSON value or null")
        if self.availability != MetricAvailability.AVAILABLE and self.value is not None:
            raise ValueError("unavailable/invalid metric values must be null")


@dataclass(frozen=True, slots=True)
class MetricSummary:
    name: str
    unit: str
    available_count: int
    missing_count: int
    failure_count: int
    median: float | None
    p95: float | None
    mad: float | None
    minimum: float | None
    maximum: float | None
    selection: str = "succeeded trials with available numeric metric"


@dataclass(frozen=True, slots=True)
class TrialResult:
    trial_id: str
    trial_index: int
    seed: int
    status: str
    variant_id: str | None
    parameters: Mapping[str, JsonValue]
    config_sha256: str
    dataset_sha256: str
    software_sha256: str
    metrics: tuple[MetricRecord, ...]
    runtime_revision_before: int | None = None
    runtime_revision_after: int | None = None
    error: Mapping[str, JsonValue] | None = None
    worker_pid: int | None = None
    worker_exit_code: int | None = None
    reproducible: bool = False
    input_reverified: bool = False

    def __post_init__(self) -> None:
        if not re.fullmatch(r"[0-9a-f]{64}", self.trial_id):
            raise ValueError("trial ID must be 64 lowercase hex characters")
        if self.trial_index < 1 or not isinstance(self.seed, int) or isinstance(self.seed, bool):
            raise ValueError("trial index and seed are invalid")
        if self.status not in {"succeeded", "failed", "cancelled", "timed_out", "invalid"}:
            raise ValueError("invalid trial status")
        require_sha256(self.config_sha256, where="trial config sha256")
        require_sha256(self.dataset_sha256, where="trial dataset sha256")
        require_sha256(self.software_sha256, where="trial software sha256")
        object.__setattr__(self, "parameters", canonical_value(self.parameters))
        object.__setattr__(self, "metrics", tuple(self.metrics))
        if self.error is not None:
            object.__setattr__(self, "error", canonical_value(self.error))


@dataclass(frozen=True, slots=True)
class ExperimentResult:
    experiment_id: str
    manifest_sha256: str
    trials: tuple[TrialResult, ...]
    metrics: tuple[MetricRecord, ...]
    summaries: tuple[MetricSummary, ...]
    result: ExperimentStatus

    def __post_init__(self) -> None:
        object.__setattr__(self, "result", ExperimentStatus(self.result))
        object.__setattr__(self, "trials", tuple(self.trials))
        object.__setattr__(self, "metrics", tuple(self.metrics))
        object.__setattr__(self, "summaries", tuple(self.summaries))
        require_sha256(self.manifest_sha256, where="manifest sha256")

    def records(self) -> tuple[dict[str, JsonValue], ...]:
        rows: list[dict[str, JsonValue]] = []
        for trial in self.trials:
            for metric in trial.metrics:
                rows.append(
                    {
                        "experiment_id": self.experiment_id,
                        "trial_id": trial.trial_id,
                        "trial_index": trial.trial_index,
                        "seed": trial.seed,
                        "status": trial.status,
                        "parameter_json": canonical_json_bytes(trial.parameters)
                        .decode("utf-8")
                        .rstrip("\n"),
                        "variant_id": trial.variant_id,
                        "metric": metric.name,
                        "unit": metric.unit,
                        "value": plain_value(metric.value),
                        "source": metric.source.value,
                        "scope": metric.scope,
                        "subject": metric.subject,
                        "availability": metric.availability.value,
                        "reason": metric.reason,
                    }
                )
        return tuple(rows)

    def write_json(self, path: str | Path) -> None:
        from ._export import result_to_dict

        atomic_write(path, canonical_json_bytes(result_to_dict(self)))

    def write_csv(self, path: str | Path) -> None:
        from ._export import csv_bytes

        atomic_write(path, csv_bytes(self))

    def to_pandas(self) -> Any:
        import pandas  # type: ignore[import-not-found]

        return pandas.DataFrame(self.records())


@dataclass(frozen=True, slots=True)
class DatasetCatalog:
    datasets: Mapping[str, DatasetSpec]

    def __post_init__(self) -> None:
        object.__setattr__(self, "datasets", MappingProxyType(dict(self.datasets)))

    @classmethod
    def load(cls, path: str | Path) -> "DatasetCatalog":
        from ._manifest import load_dataset_catalog

        return load_dataset_catalog(Path(path))

    def require(self, dataset_id: str) -> DatasetSpec:
        try:
            return self.datasets[dataset_id]
        except KeyError as error:
            raise KeyError(f"dataset catalog has no entry {dataset_id!r}") from error
