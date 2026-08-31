from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class _Command:
    label: str
    submit: Callable[[], Any]


def _name(value: object | None, fallback: str = "-") -> str:
    if value is None:
        return fallback
    return str(getattr(value, "name", value))


def _markdown_code(value: object) -> str:
    return str(value).replace("`", "'").replace("\n", " ")


class RuntimeControlPanel:
    """Single-active-job Viser control surface over the public Python SDK."""

    def __init__(
        self,
        runtime: Any,
        server: Any,
        stages: Sequence[tuple[str, object]],
        nodes: Sequence[tuple[str, object, bool]] = (),
    ) -> None:
        if runtime is None or server is None:
            raise ValueError("runtime and server are required")
        if not stages or any(not label for label, _ in stages):
            raise ValueError("at least one named stage is required")
        self._runtime = runtime
        self._server = server
        self._stages = tuple(stages)
        self._nodes = tuple(nodes)
        self._condition = threading.Condition()
        self._state = "new"
        self._closing = False
        self._dirty = False
        self._submitting = False
        self._pending_cancel = False
        self._generation = 0
        self._terminal_generation = 0
        self._terminal_error: BaseException | None = None
        self._active_job: Any | None = None
        self._active_label = "Idle"
        self._runtime_status = "UNKNOWN"
        self._job_state = "IDLE"
        self._active_stage = "-"
        self._runtime_revision = 0
        self._config_revision = 0
        self._progress_current = 0
        self._progress_total = 0
        self._progress_detail = "-"
        self._message = "Ready"
        self._last_event_sequence = 0
        self._agents: tuple[str, ...] = ()
        self._subscription: Any | None = None
        self._waiter: threading.Thread | None = None
        self._ui_worker: threading.Thread | None = None
        self._handles: list[Any] = []
        self._command_buttons: list[Any] = []
        self._agent_buttons: list[Any] = []
        self._cancel_button: Any | None = None
        self._agent_selector: Any | None = None
        self._status_markdown: Any | None = None

    def start(self) -> None:
        with self._condition:
            if self._state != "new":
                raise RuntimeError("RuntimeControlPanel can only be started once")
            self._state = "started"
        try:
            self._create_gui()
            worker = threading.Thread(
                target=self._run_ui_worker,
                name="open-lmm-viser-control-ui",
                daemon=False,
            )
            worker.start()
            self._ui_worker = worker
            self._subscription = self._runtime.subscribe_events(self._on_event)
            self._synchronize_snapshot()
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        with self._condition:
            if self._state == "closed":
                return
            self._closing = True
            while self._submitting:
                self._condition.wait()
            subscription, self._subscription = self._subscription, None
            job = self._active_job
            waiter = self._waiter
        if subscription is not None:
            try:
                subscription.close()
            except Exception:
                pass
        if job is not None:
            try:
                job.cancel()
            except Exception:
                pass
        if waiter is not None and waiter is not threading.current_thread():
            waiter.join()
        with self._condition:
            self._state = "closed"
            self._dirty = True
            self._condition.notify_all()
            ui_worker, self._ui_worker = self._ui_worker, None
        if ui_worker is not None and ui_worker is not threading.current_thread():
            ui_worker.join()
        for handle in reversed(self._handles):
            try:
                handle.remove()
            except Exception:
                pass
        self._handles.clear()

    def submit_run_all(self) -> int:
        return self._submit(_Command("Run All", self._runtime.run_all))

    def submit_stage(self, label: str, stage: object) -> int:
        return self._submit(
            _Command(f"Stage: {label}", lambda: self._runtime.run_stage(stage))
        )

    def submit_node(self, label: str, node: object, requires_agent: bool) -> int:
        agent = self._selected_agent() if requires_agent else None
        if requires_agent and agent is None:
            raise RuntimeError(f"Node: {label} requires a configured agent")
        suffix = "" if agent is None else f" ({agent})"
        return self._submit(
            _Command(
                f"Node: {label}{suffix}",
                lambda: self._runtime.run_node(node, agent=agent),
            )
        )

    def submit_optimize_through(self) -> int:
        agent = self._selected_agent()
        if agent is None:
            raise RuntimeError("Optimize Through requires a configured agent")
        return self._submit(
            _Command(
                f"Optimize Through ({agent})",
                lambda: self._runtime.optimize_through(agent),
            )
        )

    def refresh_snapshot(self) -> None:
        with self._condition:
            if self._state != "started" or self._closing:
                return
        self._synchronize_snapshot()

    def active_job(self) -> Any | None:
        """Return the exact active Job object, never a reconstructed job id."""
        with self._condition:
            return self._active_job

    def cancel_active(self) -> None:
        with self._condition:
            if self._closing:
                return
            if self._submitting and self._active_job is None:
                self._pending_cancel = True
                self._job_state = "CANCELLING"
                self._message = "Cancellation requested while submitting"
                self._mark_dirty_locked()
                return
            job = self._active_job
            if job is None:
                self._message = "No active job to cancel"
                self._mark_dirty_locked()
                return
            self._pending_cancel = True
            self._job_state = "CANCELLING"
            self._message = f"Cancellation requested for job {job.id}"
            self._mark_dirty_locked()
        try:
            job.cancel()
        except Exception as error:
            with self._condition:
                self._pending_cancel = False
                self._message = f"Cancel failed: {type(error).__name__}: {error}"
                self._mark_dirty_locked()

    def wait(self, generation: int) -> None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._terminal_generation >= generation or self._closing
            )
            if self._terminal_generation < generation:
                raise RuntimeError("control panel closed before the job completed")
            error = self._terminal_error
        if error is not None:
            raise error

    def _create_gui(self) -> None:
        status = self._server.gui.add_markdown("**OpenLMM:** starting")
        self._status_markdown = status
        self._handles.append(status)
        run_all = self._server.gui.add_button("Run All")
        run_all.on_click(lambda _event: self._handle_submit(self.submit_run_all))
        self._command_buttons.append(run_all)
        self._handles.append(run_all)
        for label, stage in self._stages:
            button = self._server.gui.add_button(f"Stage: {label}")
            button.on_click(
                lambda _event, label=label, stage=stage: self._handle_submit(
                    lambda: self.submit_stage(label, stage)
                )
            )
            self._command_buttons.append(button)
            self._handles.append(button)
        selector = self._server.gui.add_dropdown(
            "Agent", options=("No agents available",), disabled=True
        )
        self._agent_selector = selector
        self._handles.append(selector)
        for label, node, requires_agent in self._nodes:
            button = self._server.gui.add_button(f"Node: {label}")
            button.on_click(
                lambda _event, label=label, node=node,
                requires_agent=requires_agent: self._handle_submit(
                    lambda: self.submit_node(label, node, requires_agent)
                )
            )
            self._command_buttons.append(button)
            if requires_agent:
                self._agent_buttons.append(button)
            self._handles.append(button)
        optimize = self._server.gui.add_button("Optimize Through")
        optimize.on_click(
            lambda _event: self._handle_submit(self.submit_optimize_through)
        )
        self._command_buttons.append(optimize)
        self._agent_buttons.append(optimize)
        self._handles.append(optimize)
        cancel = self._server.gui.add_button("Cancel Active Job", color="red")
        cancel.disabled = True
        cancel.on_click(lambda _event: self.cancel_active())
        self._cancel_button = cancel
        self._handles.append(cancel)

    def _handle_submit(self, submit: Callable[[], int]) -> None:
        try:
            submit()
        except Exception as error:
            with self._condition:
                self._message = f"Submit rejected: {type(error).__name__}: {error}"
                self._mark_dirty_locked()

    def _selected_agent(self) -> str | None:
        with self._condition:
            selector = self._agent_selector
            selected = None if selector is None else str(selector.value)
            return selected if selected in self._agents else None

    def _submit(self, command: _Command) -> int:
        with self._condition:
            if self._state != "started" or self._closing:
                raise RuntimeError("control panel is not running")
            if self._submitting or self._active_job is not None:
                raise RuntimeError("another OpenLMM job is active")
            if self._waiter is not None and self._waiter.is_alive():
                raise RuntimeError("previous OpenLMM job is still retiring")
            self._generation += 1
            generation = self._generation
            self._submitting = True
            self._terminal_error = None
            self._active_label = command.label
            self._job_state = "SUBMITTING"
            self._active_stage = "-"
            self._progress_current = 0
            self._progress_total = 0
            self._progress_detail = "-"
            self._message = f"Submitting {command.label}"
            self._mark_dirty_locked()
        try:
            job = command.submit()
        except BaseException as error:
            with self._condition:
                self._submitting = False
                self._job_state = "FAILED"
                self._message = f"Submit failed: {type(error).__name__}: {error}"
                self._terminal_generation = generation
                self._terminal_error = error
                self._condition.notify_all()
                self._mark_dirty_locked()
            raise

        with self._condition:
            self._active_job = job
            self._submitting = False
            cancel_after_submit = self._pending_cancel or self._closing
            self._job_state = "CANCELLING" if cancel_after_submit else "QUEUED"
            self._message = f"Job {job.id} accepted: {command.label}"
            waiter = threading.Thread(
                target=self._wait_job,
                args=(generation, job),
                name=f"open-lmm-viser-job-{job.id}",
                daemon=False,
            )
            self._waiter = waiter
            self._condition.notify_all()
            self._mark_dirty_locked()
        try:
            waiter.start()
        except BaseException as error:
            try:
                job.cancel()
            except Exception:
                pass
            with self._condition:
                self._active_job = None
                self._waiter = None
                self._job_state = "FAILED"
                self._message = f"Waiter start failed: {type(error).__name__}: {error}"
                self._terminal_generation = generation
                self._terminal_error = error
                self._condition.notify_all()
                self._mark_dirty_locked()
            raise
        if cancel_after_submit:
            try:
                job.cancel()
            except Exception as error:
                with self._condition:
                    self._message = (
                        f"Cancel after submit failed: {type(error).__name__}: {error}"
                    )
                    self._mark_dirty_locked()
        return generation

    def _wait_job(self, generation: int, job: Any) -> None:
        error: BaseException | None = None
        try:
            job.wait()
        except BaseException as caught:
            error = caught
        snapshot = None
        try:
            snapshot = self._runtime.snapshot()
        except Exception:
            pass
        with self._condition:
            if self._active_job is job:
                self._active_job = None
            self._pending_cancel = False
            self._terminal_generation = generation
            self._terminal_error = error
            if snapshot is not None:
                self._apply_snapshot_locked(snapshot)
            elif error is None:
                self._job_state = "SUCCEEDED"
                self._message = f"Job {job.id} completed"
            else:
                self._job_state = "FAILED"
                self._message = f"Job {job.id} failed: {type(error).__name__}: {error}"
            self._condition.notify_all()
            self._mark_dirty_locked()

    def _on_event(self, event: object) -> None:
        with self._condition:
            if self._closing or self._active_job is None:
                return
            if getattr(event, "job_id", None) != self._active_job.id:
                return
            sequence = int(getattr(event, "sequence", 0))
            if sequence <= self._last_event_sequence:
                return
            self._last_event_sequence = sequence
            event_name = _name(getattr(event, "type", None), "")
            if event_name in {
                "JOB_STARTED",
                "STAGE_STARTED",
                "NODE_STARTED",
                "PROGRESS_UPDATED",
                "ARTIFACT_COMMITTED",
                "ARTIFACT_INVALIDATED",
                "STAGE_COMPLETED",
            }:
                self._job_state = "RUNNING"
                self._runtime_status = "RUNNING"
            elif event_name == "CANCELLATION_REQUESTED":
                self._job_state = "CANCELLING"
                self._runtime_status = "CANCELLING"
            elif event_name == "JOB_COMPLETED":
                self._job_state = "SUCCEEDED"
            elif event_name == "JOB_CANCELLED":
                self._job_state = "CANCELLED"
            elif event_name in {"NODE_FAILED", "STAGE_FAILED"}:
                self._job_state = "FAILED"
            stage = getattr(event, "stage", None)
            if stage is not None:
                self._active_stage = _name(stage)
            message = str(getattr(event, "message", "") or "")
            if message:
                self._message = message
            algorithm = getattr(event, "algorithm_progress", None)
            if algorithm is not None:
                self._progress_current = int(getattr(algorithm, "current", 0))
                total = getattr(algorithm, "total", None)
                self._progress_total = 0 if total is None else int(total)
                self._progress_detail = " / ".join(
                    part
                    for part in (
                        str(getattr(algorithm, "agent", "") or ""),
                        str(getattr(algorithm, "operation", "") or ""),
                        _name(getattr(algorithm, "phase", None), ""),
                    )
                    if part
                ) or "-"
            else:
                current = int(getattr(event, "progress_current", 0))
                total = int(getattr(event, "progress_total", 0))
                if current or total:
                    self._progress_current = current
                    self._progress_total = total
                elif event_name in {"STAGE_STARTED", "NODE_STARTED"}:
                    self._progress_current = 0
                    self._progress_total = 0
                    self._progress_detail = "-"
            self._mark_dirty_locked()

    def _synchronize_snapshot(self) -> None:
        snapshot = self._runtime.snapshot()
        with self._condition:
            self._apply_snapshot_locked(snapshot)
            self._mark_dirty_locked()

    def _apply_snapshot_locked(self, snapshot: object) -> None:
        self._runtime_status = _name(getattr(snapshot, "status", None), "UNKNOWN")
        pipeline = getattr(snapshot, "pipeline", None)
        if pipeline is None:
            return
        self._runtime_revision = int(getattr(pipeline, "runtime_revision", 0))
        self._config_revision = int(getattr(pipeline, "config_revision", 0))
        self._agents = tuple(str(agent) for agent in getattr(pipeline, "agents", ()))
        job = getattr(pipeline, "job", None)
        if job is None:
            if self._active_job is None:
                self._job_state = "IDLE"
                self._active_stage = "-"
            return
        self._job_state = _name(getattr(job, "state", None), self._job_state)
        self._active_stage = _name(getattr(job, "active_stage", None))
        message = str(getattr(job, "message", "") or "")
        if message:
            self._message = message

    def _mark_dirty_locked(self) -> None:
        self._dirty = True
        self._condition.notify_all()

    def _run_ui_worker(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._dirty or self._state == "closed")
                if self._state == "closed":
                    return
                self._dirty = False
                content, busy, cancellable = self._presentation_locked()
                status = self._status_markdown
                command_buttons = tuple(self._command_buttons)
                agent_buttons = tuple(self._agent_buttons)
                cancel = self._cancel_button
                selector = self._agent_selector
                agents = self._agents
            try:
                if status is not None:
                    status.content = content
                for button in command_buttons:
                    button.disabled = busy
                for button in agent_buttons:
                    button.disabled = busy or not agents
                if selector is not None:
                    previous = str(selector.value)
                    selector.options = agents or ("No agents available",)
                    selector.value = previous if previous in agents else selector.options[0]
                    selector.disabled = busy or not agents
                if cancel is not None:
                    cancel.disabled = not cancellable
            except Exception:
                pass

    def _presentation_locked(self) -> tuple[str, bool, bool]:
        busy = self._submitting or self._active_job is not None
        cancellable = self._active_job is not None and not self._pending_cancel
        if self._progress_total > 0:
            percent = min(
                100.0,
                max(0.0, self._progress_current * 100.0 / self._progress_total),
            )
            progress = (
                f"{self._progress_current}/{self._progress_total} ({percent:.1f}%)"
            )
        elif self._progress_current > 0:
            progress = str(self._progress_current)
        else:
            progress = "-"
        content = "\n".join(
            (
                "### OpenLMM Control",
                f"- **Runtime:** `{_markdown_code(self._runtime_status)}`",
                f"- **Revision:** runtime `{self._runtime_revision}`, config `{self._config_revision}`",
                f"- **Command:** `{_markdown_code(self._active_label)}`",
                f"- **Job:** `{_markdown_code(self._job_state)}`",
                f"- **Stage:** `{_markdown_code(self._active_stage)}`",
                f"- **Progress:** `{_markdown_code(progress)}` — `{_markdown_code(self._progress_detail)}`",
                f"- **Message:** `{_markdown_code(self._message)}`",
            )
        )
        return content, busy, cancellable
