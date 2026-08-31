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


class ExecutionScope(IntEnum):
    PER_AGENT = 0
    RUNTIME = 1


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


class AlignmentMethod(IntEnum):
    PENDING = 0
    KISS_MATCHER = 1
    DESCRIPTOR = 2
    MANUAL = 3


class AlignmentDecision(IntEnum):
    ACCEPT = 0
    TRY_KISS_MATCHER = 1
    TRY_DESCRIPTOR = 2
    MANUAL = 3
    EXCLUDE_AGENT = 4
    CANCEL = 5


class AlignmentAttemptState(IntEnum):
    IDLE = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED_RECOVERABLE = 3


class AlignmentAttemptFailure(IntEnum):
    NO_CANDIDATE = 0
    INSUFFICIENT_INLIERS = 1
    NO_CONSISTENT_CLIQUE = 2
    NO_POSE_NEIGHBOR = 3
    PROPOSAL_QUALITY_REJECTED = 4


class AlignmentReviewState(IntEnum):
    ACTIVE = 0
    CANCELLED = 1
    FAILED = 2


@dataclass(frozen=True, slots=True)
class Revision:
    runtime_revision: int
    config_revision: int


@dataclass(frozen=True, slots=True)
class NodeDescriptor:
    id: Node
    name: str
    stage: Stage
    scope: ExecutionScope
    required_artifacts: tuple[ArtifactType, ...]
    produced_artifacts: tuple[ArtifactType, ...]
    ordered: bool
    supports_cancellation: bool


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
class RuntimeReplaceReceipt:
    previous_runtime_revision: int
    previous_config_revision: int
    runtime_revision: int
    config_revision: int


@dataclass(frozen=True, slots=True)
class CommittedConfigDocument:
    domain: ConfigDomain
    canonical_json: str
    selected_document: str | None


@dataclass(frozen=True, slots=True)
class CommittedConfigDocuments:
    runtime_revision: int
    config_revision: int
    documents: tuple[CommittedConfigDocument, ...]


@dataclass(frozen=True, slots=True)
class ConfigDocumentCandidate:
    domain: ConfigDomain
    model: str
    selected_document: str
    canonical_json: str


@dataclass(frozen=True, slots=True)
class ConfigCandidateCatalog:
    runtime_revision: int
    config_revision: int
    candidates: tuple[ConfigDocumentCandidate, ...]


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


@dataclass(frozen=True, slots=True)
class LoopConstraintBuildDiagnostics:
    sampled_source_frames: int
    target_frames: int
    within_radius: int
    nearest_distance_m: float
    threshold_m: float
    search_completed: bool


@dataclass(frozen=True, slots=True)
class AlignmentAttemptStatus:
    method: AlignmentMethod
    state: AlignmentAttemptState
    reason: AlignmentAttemptFailure | None
    message: str
    attempt: int
    constraint_diagnostics: LoopConstraintBuildDiagnostics | None


@dataclass(frozen=True, slots=True)
class AlignmentMetrics:
    correspondence_count: int
    rotation_inliers: int
    final_inliers: int
    consensus_size: int
    fitness: float | None
    overlap_ratio: float | None


@dataclass(frozen=True, slots=True)
class MapAlignmentProposal:
    request_id: int
    target_agent: str
    source_agent: str
    method: AlignmentMethod
    target_T_source: Any
    metrics: AlignmentMetrics


@dataclass(frozen=True, slots=True)
class AlignmentLoopVisualization:
    target: tuple[float, float, float]
    source: tuple[float, float, float]
    inlier: bool


@dataclass(frozen=True, slots=True)
class AlignmentVisualizationData:
    target_trajectory: Any
    source_trajectory: Any
    descriptor_loops: tuple[AlignmentLoopVisualization, ...]


@dataclass(frozen=True, slots=True)
class AlignmentFeedbackSnapshot:
    proposal: MapAlignmentProposal
    target_points: Any
    source_points: Any
    diagnostics: AlignmentVisualizationData
    attempt_status: AlignmentAttemptStatus
    attempt_history: tuple[AlignmentAttemptStatus, ...]
    session_revision: int
    review_state: AlignmentReviewState
    terminal_message: str


@dataclass(frozen=True, slots=True)
class AlignmentResponse:
    request_id: int
    decision: AlignmentDecision
    session_revision: int
    manual_target_T_source: Any | None = None


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


def runtime_replace_receipt_from_native(
    value: Mapping[str, Any],
) -> RuntimeReplaceReceipt:
    return RuntimeReplaceReceipt(
        int(value["previous_runtime_revision"]),
        int(value["previous_config_revision"]),
        int(value["runtime_revision"]),
        int(value["config_revision"]),
    )


def config_documents_from_native(
    value: Mapping[str, Any],
) -> CommittedConfigDocuments:
    return CommittedConfigDocuments(
        int(value["runtime_revision"]),
        int(value["config_revision"]),
        tuple(
            CommittedConfigDocument(
                ConfigDomain(item["domain"]),
                str(item["canonical_json"]),
                item.get("selected_document"),
            )
            for item in value["documents"]
        ),
    )


def config_candidates_from_native(
    value: Mapping[str, Any],
) -> ConfigCandidateCatalog:
    return ConfigCandidateCatalog(
        int(value["runtime_revision"]),
        int(value["config_revision"]),
        tuple(
            ConfigDocumentCandidate(
                ConfigDomain(item["domain"]),
                str(item["model"]),
                str(item["selected_document"]),
                str(item["canonical_json"]),
            )
            for item in value["candidates"]
        ),
    )


def node_descriptors_from_native(value: object) -> tuple[NodeDescriptor, ...]:
    return tuple(
        NodeDescriptor(
            Node(item["id"]),
            str(item["name"]),
            Stage(item["stage"]),
            ExecutionScope(item["scope"]),
            tuple(ArtifactType(artifact) for artifact in item["required_artifacts"]),
            tuple(ArtifactType(artifact) for artifact in item["produced_artifacts"]),
            bool(item["ordered"]),
            bool(item["supports_cancellation"]),
        )
        for item in value
    )


def _constraint_diagnostics_from_native(
    value: Mapping[str, Any] | None,
) -> LoopConstraintBuildDiagnostics | None:
    if value is None:
        return None
    return LoopConstraintBuildDiagnostics(
        int(value["sampled_source_frames"]),
        int(value["target_frames"]),
        int(value["within_radius"]),
        float(value["nearest_distance_m"]),
        float(value["threshold_m"]),
        bool(value["search_completed"]),
    )


def _attempt_status_from_native(value: Mapping[str, Any]) -> AlignmentAttemptStatus:
    reason = value.get("reason")
    return AlignmentAttemptStatus(
        AlignmentMethod(value["method"]),
        AlignmentAttemptState(value["state"]),
        None if reason is None else AlignmentAttemptFailure(reason),
        str(value["message"]),
        int(value["attempt"]),
        _constraint_diagnostics_from_native(value.get("constraint_diagnostics")),
    )


def alignment_feedback_from_native(
    value: Mapping[str, Any] | None,
) -> AlignmentFeedbackSnapshot | None:
    if value is None:
        return None
    proposal_value = value["proposal"]
    metrics_value = proposal_value["metrics"]
    diagnostics_value = value["diagnostics"]
    proposal = MapAlignmentProposal(
        int(proposal_value["request_id"]),
        str(proposal_value["target_agent"]),
        str(proposal_value["source_agent"]),
        AlignmentMethod(proposal_value["method"]),
        proposal_value["target_T_source"],
        AlignmentMetrics(
            int(metrics_value["correspondence_count"]),
            int(metrics_value["rotation_inliers"]),
            int(metrics_value["final_inliers"]),
            int(metrics_value["consensus_size"]),
            metrics_value.get("fitness"),
            metrics_value.get("overlap_ratio"),
        ),
    )
    return AlignmentFeedbackSnapshot(
        proposal,
        value["target_points"],
        value["source_points"],
        AlignmentVisualizationData(
            diagnostics_value["target_trajectory"],
            diagnostics_value["source_trajectory"],
            tuple(
                AlignmentLoopVisualization(
                    tuple(float(component) for component in loop["target"]),
                    tuple(float(component) for component in loop["source"]),
                    bool(loop["inlier"]),
                )
                for loop in diagnostics_value["descriptor_loops"]
            ),
        ),
        _attempt_status_from_native(value["attempt_status"]),
        tuple(_attempt_status_from_native(item) for item in value["attempt_history"]),
        int(value["session_revision"]),
        AlignmentReviewState(value["review_state"]),
        str(value["terminal_message"]),
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
