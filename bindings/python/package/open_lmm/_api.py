from __future__ import annotations

import atexit
import os
import weakref
from typing import Callable

from . import _native
from ._errors import _raise_invalid_argument
from ._models import (
    AlignmentFeedbackSnapshot,
    AlignmentResponse,
    ConfigApplyReceipt,
    ConfigCandidateCatalog,
    ConfigDomain,
    CommittedConfigDocuments,
    ExecutionEvent,
    Node,
    NodeDescriptor,
    Revision,
    RuntimeSnapshot,
    RuntimeReplaceReceipt,
    Stage,
    VisualizationSnapshot,
    config_receipt_from_native,
    config_candidates_from_native,
    config_documents_from_native,
    execution_event_from_native,
    runtime_snapshot_from_native,
    runtime_replace_receipt_from_native,
    visualization_from_native,
    alignment_feedback_from_native,
    node_descriptors_from_native,
)


class Job:
    __slots__ = ("_native_job",)

    def __init__(self, native_job: object) -> None:
        self._native_job = native_job

    @property
    def id(self) -> int:
        return self._native_job.id

    def wait(self) -> None:
        self._native_job.wait()

    def cancel(self) -> None:
        self._native_job.cancel()


class Subscription:
    __slots__ = ("_native_subscription", "_closed", "__weakref__")

    def __init__(self, native_subscription: object) -> None:
        self._native_subscription = native_subscription
        self._closed = False
        _subscriptions.add(self)

    def close(self) -> None:
        if self._closed:
            return
        self._native_subscription.close()
        self._closed = True

    def __enter__(self) -> "Subscription":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


class Runtime:
    __slots__ = ("_native_runtime", "__weakref__")

    def __init__(self, max_agent_tasks: int = 4) -> None:
        self._native_runtime = _native.NativeRuntime(max_agent_tasks)
        _runtimes.add(self)

    def open(
        self,
        config_directory: str | os.PathLike[str],
        *,
        label: str = "",
        output_root: str | os.PathLike[str] | None = None,
    ) -> None:
        self._native_runtime.open(
            os.fspath(config_directory),
            label,
            None if output_root is None else os.fspath(output_root),
        )

    def run_all(self) -> Job:
        return Job(self._native_runtime.run_all())

    def run_stage(self, stage: Stage, *, agent: str | None = None) -> Job:
        try:
            normalized = Stage(stage)
        except (TypeError, ValueError):
            _raise_invalid_argument("stage must be a valid open_lmm.Stage")
        return Job(self._native_runtime.run_stage(int(normalized), agent))

    def run_node(self, node: Node, *, agent: str | None = None) -> Job:
        try:
            normalized = Node(node)
        except (TypeError, ValueError):
            _raise_invalid_argument("node must be a valid open_lmm.Node")
        return Job(self._native_runtime.run_node(int(normalized), agent))

    def optimize_through(self, agent: str) -> Job:
        if not isinstance(agent, str) or not agent:
            _raise_invalid_argument("agent must be a non-empty string")
        return Job(self._native_runtime.optimize_through(agent))

    def snapshot(self) -> RuntimeSnapshot:
        return runtime_snapshot_from_native(self._native_runtime.snapshot())

    def config_documents(self) -> CommittedConfigDocuments:
        return config_documents_from_native(self._native_runtime.config_documents())

    def config_candidates(self) -> ConfigCandidateCatalog:
        return config_candidates_from_native(self._native_runtime.config_candidates())

    def node_descriptors(self) -> tuple[NodeDescriptor, ...]:
        return node_descriptors_from_native(self._native_runtime.node_descriptors())

    def recent_logs(self, max_lines: int = 512) -> tuple[str, ...]:
        if isinstance(max_lines, bool) or not isinstance(max_lines, int):
            _raise_invalid_argument("max_lines must be an integer")
        if max_lines < 1 or max_lines > 512:
            _raise_invalid_argument("max_lines must be between 1 and 512")
        return tuple(self._native_runtime.recent_logs(max_lines))

    def alignment_feedback(self) -> AlignmentFeedbackSnapshot | None:
        return alignment_feedback_from_native(self._native_runtime.alignment_feedback())

    def respond_to_alignment(self, job: Job, response: AlignmentResponse) -> None:
        if not isinstance(job, Job):
            _raise_invalid_argument("job must be an open_lmm.Job")
        if not isinstance(response, AlignmentResponse):
            _raise_invalid_argument(
                "response must be an open_lmm.AlignmentResponse"
            )
        self._native_runtime.respond_to_alignment(
            job._native_job,
            response.request_id,
            int(response.decision),
            response.manual_target_T_source,
            response.session_revision,
        )

    def set_alignment_feedback_enabled(self, enabled: bool) -> None:
        if not isinstance(enabled, bool):
            _raise_invalid_argument("enabled must be a bool")
        self._native_runtime.set_alignment_feedback_enabled(enabled)

    def replace_root_config(
        self,
        document_json: str,
        *,
        expected: Revision,
    ) -> RuntimeReplaceReceipt:
        if not isinstance(document_json, str) or not document_json:
            _raise_invalid_argument("document_json must be a non-empty string")
        if not isinstance(expected, Revision):
            _raise_invalid_argument("expected must be an open_lmm.Revision")
        return runtime_replace_receipt_from_native(
            self._native_runtime.replace_root_config(
                document_json,
                expected.runtime_revision,
                expected.config_revision,
            )
        )

    def visualization(
        self,
        agent: str,
        *,
        include_points: bool = True,
        preview_voxel_size_m: float | None = None,
    ) -> VisualizationSnapshot:
        voxel = 0.0 if preview_voxel_size_m is None else float(preview_voxel_size_m)
        return visualization_from_native(
            self._native_runtime.visualization(agent, include_points, voxel)
        )

    def apply_config(
        self,
        domain: ConfigDomain,
        document_json: str,
        *,
        expected: Revision,
        selected_document: str | os.PathLike[str] | None = None,
    ) -> ConfigApplyReceipt:
        try:
            normalized = ConfigDomain(domain)
        except (TypeError, ValueError):
            _raise_invalid_argument("domain must be a valid open_lmm.ConfigDomain")
        if not isinstance(expected, Revision):
            _raise_invalid_argument("expected must be an open_lmm.Revision")
        result = self._native_runtime.apply_config(
            int(normalized),
            document_json,
            expected.runtime_revision,
            expected.config_revision,
            None if selected_document is None else os.fspath(selected_document),
        )
        return config_receipt_from_native(result)

    def subscribe_events(
        self, callback: Callable[[ExecutionEvent], None]
    ) -> Subscription:
        if not callable(callback):
            _raise_invalid_argument("event callback must be callable")

        def convert_and_call(payload: object) -> None:
            callback(execution_event_from_native(payload))

        return Subscription(self._native_runtime.subscribe_events(convert_and_call))

    def cancel(self, job: Job) -> None:
        if not isinstance(job, Job):
            _raise_invalid_argument("job must be an open_lmm.Job")
        self._native_runtime.cancel(job._native_job)

    def close(self, *, cancel_running: bool = True) -> None:
        self._native_runtime.close(bool(cancel_running))

    def is_open(self) -> bool:
        return self._native_runtime.is_open()

    def __enter__(self) -> "Runtime":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close(cancel_running=True)


_subscriptions: weakref.WeakSet[Subscription] = weakref.WeakSet()
_runtimes: weakref.WeakSet[Runtime] = weakref.WeakSet()


def _shutdown() -> None:
    for subscription in list(_subscriptions):
        try:
            subscription.close()
        except BaseException:
            pass
    for runtime in list(_runtimes):
        try:
            runtime.close(cancel_running=True)
        except BaseException:
            pass


atexit.register(_shutdown)
