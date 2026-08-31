from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Any, Mapping

from ._errors import ErrorInfo, error_info_from_native


class Stage(IntEnum):
    DATA_LOAD = 0
    ALIGNMENT = 1
    MAP_UPDATE = 2
    SAVE = 3


class Node(IntEnum):
    DATA_LOAD = 0
    LOOP_DETECT = 1
    OPTIMIZE = 2
    MAP_UPDATE = 3
    POSE_SAVE = 4
    FALLBACK_MAP_SAVE = 5


class ArtifactType(IntEnum):
    CONFIG_SNAPSHOT = 0
    AGENT_INPUT = 1
    RAW_DATA = 2
    DESCRIPTOR_STATE = 3
    LOOP_CANDIDATES = 4
    MAP_ALIGNMENT = 5
    OPTIMIZER_STATE = 6
    OPTIMIZED_POSES = 7
    GLOBAL_MAP = 8
    POSE_FILE = 9
    PCD_FILE = 10
    PROFILE_RECORD = 11


class ArtifactState(IntEnum):
    MISSING = 0
    READY = 1
    STALE = 2
    FAILED = 3


class RuntimeStatus(IntEnum):
    CREATING = 0
    READY = 1
    RUNNING = 2
    CANCELLING = 3
    FAILED_RECOVERABLE = 4
    FAILED_FATAL = 5
    CLOSING = 6
    CLOSED = 7


class JobState(IntEnum):
    QUEUED = 0
    WAITING_FOR_DEPENDENCY = 1
    RUNNING = 2
    CANCELLING = 3
    WAITING_FOR_ALIGNMENT_FEEDBACK = 4
    SUCCEEDED = 5
    FAILED = 6
    CANCELLED = 7


class EventType(IntEnum):
    JOB_QUEUED = 0
    JOB_STARTED = 1
    STAGE_STARTED = 2
    NODE_STARTED = 3
    PROGRESS_UPDATED = 4
    ARTIFACT_COMMITTED = 5
    ARTIFACT_INVALIDATED = 6
    NODE_FAILED = 7
    STAGE_COMPLETED = 8
    STAGE_FAILED = 9
    CANCELLATION_REQUESTED = 10
    ALIGNMENT_FEEDBACK_REQUESTED = 11
    ALIGNMENT_PROPOSAL_ACCEPTED = 12
    ALIGNMENT_PROPOSAL_REJECTED = 13
    ALIGNMENT_FEEDBACK_CANCELLED = 14
    ALIGNMENT_AGENT_EXCLUDED = 15
    JOB_COMPLETED = 16
    JOB_CANCELLED = 17


class ConfigDomain(IntEnum):
    GLOBAL = 0
    DATA_LOADER = 1
    LOOP_DETECTOR = 2
    OPTIMIZER = 3
    DYNAMIC_REMOVER = 4
    MAP_SAVE = 5


class CancellationMode(IntEnum):
    TOKEN_POLLING = 0
    HOST_SAFE_POINTS = 1
    NON_COOPERATIVE = 2


class AlgorithmProgressPhase(IntEnum):
    ENUMERATE = 0
    READ_AND_FILTER = 1
    BUILD_PREVIEW = 2
    LOAD_REMOVER_INPUT = 3
    READ_AND_RUN_REMOVER = 4
    BUILD_RAW_MAP = 5
    INITIALIZE_REMOVER = 6
    RUN_REMOVER = 7
    BUILD_STATIC_MAP = 8
    DETECT_LOOPS = 9
    OPTIMIZE_GRAPH = 10
    WRITE_OUTPUT = 11
    BUILD_ALIGNMENT_MAP = 12
    DETECT_INTRA_LOOPS = 13
    RUN_KISS_MATCHER = 14
    DETECT_INTER_LOOPS = 15
    BUILD_DESCRIPTOR_CONSENSUS = 16
    WAIT_ALIGNMENT_REVIEW = 17
    BUILD_LOOP_CONSTRAINTS = 18
    REGISTER_INTRA_LOOPS = 19
    REGISTER_INTER_LOOPS = 20
    SOLVE_GRAPH = 21


class VisualizationPhase(IntEnum):
    DATA_LOAD = 0
    LOOP_DETECTION = 1
    OPTIMIZATION = 2
    MAP_UPDATE = 3
    SAVE = 4


class VisualizationPoseKind(IntEnum):
    ODOMETRY = 0
    OPTIMIZED = 1


class VisualizationPointKind(IntEnum):
    FILTERED_SCAN_PREVIEW = 0
    OPTIMIZATION_MAP_PREVIEW = 1
    FINAL_STATIC_MAP = 2


class VisualizationEdgeType(IntEnum):
    TRAJECTORY = 0
    INTRA_LOOP = 1
    INTER_LOOP = 2


@dataclass(frozen=True, slots=True)
class Revision:
    runtime_revision: int
    config_revision: int


@dataclass(frozen=True, slots=True)
class CancellationCapability:
    cooperative: bool
    mode: CancellationMode
    non_interruptible_operations: tuple[str, ...]
    requires_process_isolation: bool


@dataclass(frozen=True, slots=True)
class CancellationTelemetry:
    capability: CancellationCapability
    cancel_requested_at_unix_ns: int | None
    cancel_observed_at_unix_ns: int | None
    cancel_completed_at_unix_ns: int | None

    @property
    def pending(self) -> bool:
        return (
            self.cancel_requested_at_unix_ns is not None
            and self.cancel_completed_at_unix_ns is None
        )


@dataclass(frozen=True, slots=True)
class AlgorithmProgress:
    agent: str
    operation: str
    phase: AlgorithmProgressPhase
    current: int
    total: int | None


@dataclass(frozen=True, slots=True)
class ArtifactMetadata:
    type: ArtifactType
    agent: str | None
    state: ArtifactState
    revision: int
    producer: str
    detail: str
    external_path: str
    fingerprint: str


@dataclass(frozen=True, slots=True)
class ExecutionEvent:
    job_id: int
    type: EventType
    stage: Stage | None
    message: str
    sequence: int
    node: Node | None
    agent: str | None
    progress_current: int
    progress_total: int
    error: ErrorInfo | None
    cancellation: CancellationTelemetry | None
    affected_agents: tuple[str, ...]
    algorithm_progress: AlgorithmProgress | None


@dataclass(frozen=True, slots=True)
class JobSnapshot:
    id: int
    state: JobState
    active_stage: Stage | None
    message: str
    cancellation: CancellationTelemetry


@dataclass(frozen=True, slots=True)
class PipelineSnapshot:
    job: JobSnapshot | None
    runtime_revision: int
    config_revision: int
    agents: tuple[str, ...]
    artifacts: tuple[ArtifactMetadata, ...]
    recent_events: tuple[ExecutionEvent, ...]


@dataclass(frozen=True, slots=True)
class RuntimeSnapshot:
    label: str
    status: RuntimeStatus
    output_directory: Path
    pipeline: PipelineSnapshot


@dataclass(frozen=True, slots=True)
class ConfigApplyReceipt:
    previous_config_revision: int
    config_revision: int
    base_runtime_revision: int
    runtime_revision: int
    affected_agents: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class VisualizationEdge:
    from_agent: str
    from_index: int
    to_agent: str
    to_index: int
    type: VisualizationEdgeType


@dataclass(frozen=True, slots=True)
class VisualizationSnapshot:
    agent: str
    revision: int
    phase: VisualizationPhase
    pose_kind: VisualizationPoseKind
    point_kind: VisualizationPointKind
    poses: Any
    edges: tuple[VisualizationEdge, ...]
    points: Any
    min_bound: Any
    max_bound: Any
    has_bounds: bool
    points_available: bool
    points_complete: bool
    map_available: bool
    displayed_point_count: int
    source_point_count: int


def _cancellation_from_native(value: Mapping[str, Any]) -> CancellationTelemetry:
    capability = value["capability"]
    return CancellationTelemetry(
        CancellationCapability(
            bool(capability["cooperative"]),
            CancellationMode(capability["mode"]),
            tuple(capability["non_interruptible_operations"]),
            bool(capability["requires_process_isolation"]),
        ),
        value.get("cancel_requested_at_unix_ns"),
        value.get("cancel_observed_at_unix_ns"),
        value.get("cancel_completed_at_unix_ns"),
    )


def _progress_from_native(value: Mapping[str, Any] | None) -> AlgorithmProgress | None:
    if value is None:
        return None
    return AlgorithmProgress(
        str(value["agent"]),
        str(value["operation"]),
        AlgorithmProgressPhase(value["phase"]),
        int(value["current"]),
        value.get("total"),
    )


def execution_event_from_native(value: Mapping[str, Any]) -> ExecutionEvent:
    return ExecutionEvent(
        int(value["job_id"]),
        EventType(value["type"]),
        None if value.get("stage") is None else Stage(value["stage"]),
        str(value["message"]),
        int(value["sequence"]),
        None if value.get("node") is None else Node(value["node"]),
        value.get("agent"),
        int(value["progress_current"]),
        int(value["progress_total"]),
        None if value.get("error") is None else error_info_from_native(value["error"]),
        None
        if value.get("cancellation") is None
        else _cancellation_from_native(value["cancellation"]),
        tuple(value["affected_agents"]),
        _progress_from_native(value.get("algorithm_progress")),
    )


def runtime_snapshot_from_native(value: Mapping[str, Any]) -> RuntimeSnapshot:
    pipeline_value = value["pipeline"]
    job_value = pipeline_value.get("job")
    job = None
    if job_value is not None:
        job = JobSnapshot(
            int(job_value["id"]),
            JobState(job_value["state"]),
            None
            if job_value.get("active_stage") is None
            else Stage(job_value["active_stage"]),
            str(job_value["message"]),
            _cancellation_from_native(job_value["cancellation"]),
        )
    artifacts = tuple(
        ArtifactMetadata(
            ArtifactType(item["type"]),
            item.get("agent"),
            ArtifactState(item["state"]),
            int(item["revision"]),
            str(item["producer"]),
            str(item["detail"]),
            str(item["external_path"]),
            str(item["fingerprint"]),
        )
        for item in pipeline_value["artifacts"]
    )
    pipeline = PipelineSnapshot(
        job,
        int(pipeline_value["runtime_revision"]),
        int(pipeline_value["config_revision"]),
        tuple(pipeline_value["agents"]),
        artifacts,
        tuple(execution_event_from_native(item) for item in pipeline_value["recent_events"]),
    )
    return RuntimeSnapshot(
        str(value["label"]),
        RuntimeStatus(value["status"]),
        Path(value["output_directory"]),
        pipeline,
    )


def config_receipt_from_native(value: Mapping[str, Any]) -> ConfigApplyReceipt:
    return ConfigApplyReceipt(
        int(value["previous_config_revision"]),
        int(value["config_revision"]),
        int(value["base_runtime_revision"]),
        int(value["runtime_revision"]),
        tuple(value["affected_agents"]),
    )


def visualization_from_native(value: Mapping[str, Any]) -> VisualizationSnapshot:
    edges = tuple(
        VisualizationEdge(
            str(item["from_agent"]),
            int(item["from_index"]),
            str(item["to_agent"]),
            int(item["to_index"]),
            VisualizationEdgeType(item["type"]),
        )
        for item in value["edges"]
    )
    return VisualizationSnapshot(
        str(value["agent"]),
        int(value["revision"]),
        VisualizationPhase(value["phase"]),
        VisualizationPoseKind(value["pose_kind"]),
        VisualizationPointKind(value["point_kind"]),
        value["poses"],
        edges,
        value["points"],
        value["min_bound"],
        value["max_bound"],
        bool(value["has_bounds"]),
        bool(value["points_available"]),
        bool(value["points_complete"]),
        bool(value["map_available"]),
        int(value["displayed_point_count"]),
        int(value["source_point_count"]),
    )
