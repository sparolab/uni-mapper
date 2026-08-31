from __future__ import annotations

import math
import threading
from typing import Any, Callable

import numpy as np

from .conversion import agent_color, scene_key


_ROOT = "/open_lmm/alignment/review"
_MIN_GIZMO_SCALE = 0.5
_MAX_GIZMO_SCALE = 20.0


def _enum_name(value: object) -> str:
    return str(getattr(value, "name", value))


def _rotation_to_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            values = (
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            )
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            values = (
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            values = (
                (rotation[1, 0] - rotation[0, 1]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
            )
    norm = math.sqrt(sum(component * component for component in values))
    return tuple(component / norm for component in values)


def _matrix_from_handle(handle: Any) -> np.ndarray:
    w, x, y, z = (float(value) for value in handle.wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("manual alignment quaternion is invalid")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    matrix[:3, 3] = np.asarray(handle.position, dtype=np.float64)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("manual alignment transform contains non-finite values")
    return matrix


def _segments(points: object) -> np.ndarray | None:
    values = np.asarray(points, dtype=np.float32)
    if values.ndim != 2 or values.shape[1:] != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("alignment trajectory must have finite shape (N, 3)")
    if values.shape[0] < 2:
        return None
    return np.ascontiguousarray(np.stack((values[:-1], values[1:]), axis=1))


def _gizmo_scale(*clouds: np.ndarray) -> float:
    extent = 0.0
    for cloud in clouds:
        if cloud.shape[0] == 0:
            continue
        span = np.max(cloud, axis=0) - np.min(cloud, axis=0)
        extent = max(extent, float(np.max(span)))
    return min(max(extent * 0.05, _MIN_GIZMO_SCALE), _MAX_GIZMO_SCALE)


class AlignmentControlPanel:
    """Interactive alignment presentation over the public Python Runtime API."""

    def __init__(
        self,
        runtime: Any,
        server: Any,
        active_job: Callable[[], Any | None],
        *,
        commit_source: Any | None = None,
        panel_handle: Any | None = None,
        poll_interval_s: float = 0.2,
        point_size: float = 0.2,
    ) -> None:
        if runtime is None or server is None or not callable(active_job):
            raise ValueError("runtime, server, and active_job are required")
        if not math.isfinite(poll_interval_s) or poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s must be positive and finite")
        self._runtime = runtime
        self._server = server
        self._active_job = active_job
        self._poll_interval_s = float(poll_interval_s)
        self._point_size = float(point_size)
        self._panel_handle = panel_handle
        self._condition = threading.Condition()
        self._scene_mutex = threading.Lock()
        self._state = "new"
        self._dirty = True
        self._feedback: Any | None = None
        self._key: tuple[int, int] | None = None
        self._subscription: Any | None = None
        self._feedback_enabled = False
        self._worker: threading.Thread | None = None
        self._scene_handles: list[Any] = []
        self._gui_handles: list[Any] = []
        self._buttons: list[Any] = []
        self._normal_buttons: list[Any] = []
        self._manual_buttons: list[Any] = []
        self._manual_entry: Any | None = None
        self._status: Any | None = None
        self._manual_frame: Any | None = None
        self._manual_controls: Any | None = None
        self._manual_mode = False
        self._accepted_source: str | None = None
        self._last_error = ""
        self._commit_source = commit_source
        if commit_source is not None:
            commit_source.add_commit_observer(self._on_presentation_commit)

    def start(self) -> None:
        with self._condition:
            if self._state != "new":
                raise RuntimeError("AlignmentControlPanel can only be started once")
            self._state = "started"
        try:
            self._create_gui()
            self._runtime.set_alignment_feedback_enabled(True)
            self._feedback_enabled = True
            self._subscription = self._runtime.subscribe_events(self._on_event)
            self._worker = threading.Thread(
                target=self._run, name="open-lmm-viser-alignment", daemon=False
            )
            self._worker.start()
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        with self._condition:
            if self._state == "closed":
                return
            self._state = "closed"
            self._condition.notify_all()
        subscription, self._subscription = self._subscription, None
        if subscription is not None:
            try:
                subscription.close()
            except Exception:
                pass
        worker, self._worker = self._worker, None
        if worker is not None and worker is not threading.current_thread():
            worker.join()
        if self._feedback_enabled:
            try:
                self._runtime.set_alignment_feedback_enabled(False)
            except Exception:
                pass
            self._feedback_enabled = False
        commit_source = self._commit_source
        if commit_source is not None:
            try:
                commit_source.remove_commit_observer(
                    self._on_presentation_commit
                )
            except Exception:
                pass
        self._clear_scene()
        self._commit_source = None
        for handle in reversed(self._gui_handles):
            try:
                handle.remove()
            except Exception:
                pass
        self._gui_handles.clear()

    def respond(self, decision: Any, *, manual: bool = False) -> None:
        with self._condition:
            feedback = self._feedback
        if feedback is None:
            raise RuntimeError("there is no alignment review to answer")
        job = self._matching_job()
        if job is None:
            raise RuntimeError("alignment review is read-only without its exact active Job")
        if manual:
            with self._scene_mutex:
                if self._manual_controls is None or not self._manual_mode:
                    raise RuntimeError("manual alignment mode is not active")
                transform = _matrix_from_handle(self._manual_controls)
        else:
            transform = None
        response_type = __import__("open_lmm").AlignmentResponse
        self._runtime.respond_to_alignment(
            job,
            response_type(
                feedback.proposal.request_id,
                decision,
                feedback.session_revision,
                transform,
            ),
        )
        name = _enum_name(decision)
        if name in {"ACCEPT", "MANUAL"}:
            self._accepted_source = feedback.proposal.source_agent
        self._leave_manual_mode(reset=False)
        with self._condition:
            self._dirty = True
            self._condition.notify_all()

    def _matching_job(self) -> Any | None:
        job = self._active_job()
        if job is None:
            return None
        try:
            snapshot_job = self._runtime.snapshot().pipeline.job
        except Exception:
            return None
        return job if snapshot_job is not None and snapshot_job.id == job.id else None

    def _create_gui(self) -> None:
        self._status = self._server.gui.add_markdown("### Alignment Review\nIdle")
        self._gui_handles.append(self._status)
        import open_lmm

        actions = (
            ("Accept", open_lmm.AlignmentDecision.ACCEPT),
            ("Try KISS", open_lmm.AlignmentDecision.TRY_KISS_MATCHER),
            ("Try Descriptor", open_lmm.AlignmentDecision.TRY_DESCRIPTOR),
            ("Manual Align", None),
            ("Exclude Agent", open_lmm.AlignmentDecision.EXCLUDE_AGENT),
            ("Cancel Alignment", open_lmm.AlignmentDecision.CANCEL),
        )
        for label, decision in actions:
            button = self._server.gui.add_button(label, color="red" if label.startswith("Cancel") else None)
            button.disabled = True
            if decision is None:
                self._manual_entry = button
                button.on_click(
                    lambda _event: self._handle_manual_action(
                        self._enter_manual_mode
                    )
                )
            else:
                button.on_click(
                    lambda _event, decision=decision: self._handle_response(
                        decision, False
                    )
                )
            self._buttons.append(button)
            self._normal_buttons.append(button)
            self._gui_handles.append(button)
        manual_actions = (
            (
                "Apply Manual Transform",
                lambda: self.respond(
                    open_lmm.AlignmentDecision.MANUAL, manual=True
                ),
            ),
            ("Reset Proposal", lambda: self._reset_manual_transform(False)),
            ("Reset Identity", lambda: self._reset_manual_transform(True)),
            ("Cancel Manual", lambda: self._leave_manual_mode(reset=True)),
        )
        for label, callback in manual_actions:
            button = self._server.gui.add_button(
                label, color="red" if label == "Cancel Manual" else None
            )
            button.disabled = True
            button.visible = False
            button.on_click(
                lambda _event, callback=callback: self._handle_manual_action(
                    callback
                )
            )
            self._buttons.append(button)
            self._manual_buttons.append(button)
            self._gui_handles.append(button)

    def _handle_response(self, decision: Any, manual: bool) -> None:
        try:
            self.respond(decision, manual=manual)
        except Exception as error:
            self._last_error = f"{type(error).__name__}: {error}"
            self._update_gui()

    def _handle_manual_action(self, callback: Callable[[], None]) -> None:
        try:
            callback()
            self._last_error = ""
            self._update_gui()
        except Exception as error:
            self._last_error = f"{type(error).__name__}: {error}"
            self._update_gui()

    def _enter_manual_mode(self) -> None:
        with self._condition:
            feedback = self._feedback
        if feedback is None:
            raise RuntimeError("there is no alignment review to edit")
        if self._matching_job() is None:
            raise RuntimeError("alignment review is read-only without its exact active Job")
        with self._scene_mutex:
            if self._manual_controls is None:
                raise RuntimeError("manual alignment controls are unavailable")
            self._set_control_transform(feedback.proposal.target_T_source)
            self._manual_controls.visible = True
            self._manual_mode = True

    def _reset_manual_transform(self, identity: bool) -> None:
        with self._condition:
            feedback = self._feedback
        with self._scene_mutex:
            if not self._manual_mode or self._manual_controls is None:
                raise RuntimeError("manual alignment mode is not active")
            if feedback is None:
                raise RuntimeError("there is no alignment review to reset")
            transform = (
                np.eye(4, dtype=np.float64)
                if identity
                else feedback.proposal.target_T_source
            )
            self._set_control_transform(transform)

    def _leave_manual_mode(self, *, reset: bool) -> None:
        with self._condition:
            feedback = self._feedback
        with self._scene_mutex:
            controls = self._manual_controls
            if reset and controls is not None and feedback is not None:
                self._set_control_transform(feedback.proposal.target_T_source)
            if controls is not None:
                controls.visible = False
            self._manual_mode = False

    def _set_control_transform(self, transform: object) -> None:
        matrix = np.asarray(transform, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError("manual alignment proposal transform is invalid")
        wxyz = _rotation_to_wxyz(matrix[:3, :3])
        position = tuple(
            float(value) for value in matrix[:3, 3]
        )
        self._manual_controls.wxyz = wxyz
        self._manual_controls.position = position
        if self._manual_frame is not None:
            self._manual_frame.wxyz = wxyz
            self._manual_frame.position = position

    def _on_event(self, event: object) -> None:
        with self._condition:
            if self._state == "started":
                event_name = _enum_name(getattr(event, "type", ""))
                if (
                    event_name == "ALIGNMENT_PROPOSAL_ACCEPTED"
                    and self._feedback is not None
                ):
                    self._accepted_source = (
                        self._feedback.proposal.source_agent
                    )
                self._dirty = True
                self._condition.notify_all()

    def _run(self) -> None:
        while True:
            with self._condition:
                if self._state == "closed":
                    return
                self._condition.wait(timeout=self._poll_interval_s)
                if self._state == "closed":
                    return
                self._dirty = False
            try:
                feedback = self._runtime.alignment_feedback()
                if feedback is not None:
                    key = (feedback.proposal.request_id, feedback.session_revision)
                    if key != self._key:
                        self._publish(feedback)
                        with self._condition:
                            self._feedback = feedback
                            self._key = key
                self._last_error = ""
            except Exception as error:
                self._last_error = f"{type(error).__name__}: {error}"
            self._update_gui()

    def _publish(self, feedback: Any) -> None:
        catalog = tuple(self._runtime.snapshot().pipeline.agents)
        colors = {agent: agent_color(index) for index, agent in enumerate(catalog)}
        target_color = colors.get(feedback.proposal.target_agent, (51, 166, 255))
        source_color = colors.get(feedback.proposal.source_agent, (255, 166, 26))
        target = np.asarray(feedback.target_points, dtype=np.float32)
        source = np.asarray(feedback.source_points, dtype=np.float32)
        if (
            target.ndim != 2
            or target.shape[1:] != (3,)
            or source.ndim != 2
            or source.shape[1:] != (3,)
        ):
            raise ValueError("alignment review clouds must have shape (N, 3)")
        if target.shape[0] + source.shape[0] > 2_000_000:
            raise ValueError("alignment review exceeds the 2000000-point limit")
        transform = np.asarray(feedback.proposal.target_T_source, dtype=np.float64)
        generation_root = (
            f"{_ROOT}/r{feedback.proposal.request_id}_s{feedback.session_revision}"
        )
        new_handles: list[Any] = []
        new_frame: Any | None = None
        new_controls: Any | None = None
        with self._scene_mutex:
            try:
                with self._server.atomic():
                    new_handles.append(
                        self._server.scene.add_point_cloud(
                            f"{generation_root}/target/"
                            f"{scene_key(feedback.proposal.target_agent)}",
                            points=target,
                            colors=target_color,
                            point_size=self._point_size,
                            precision="float32",
                        )
                    )
                    frame = self._server.scene.add_frame(
                        f"{generation_root}/source_transform",
                        show_axes=False,
                        wxyz=_rotation_to_wxyz(transform[:3, :3]),
                        position=tuple(float(value) for value in transform[:3, 3]),
                    )
                    new_frame = frame
                    new_handles.append(frame)
                    controls = self._server.scene.add_transform_controls(
                        f"{generation_root}/manual_controls",
                        scale=_gizmo_scale(target, source),
                        depth_test=False,
                        wxyz=_rotation_to_wxyz(transform[:3, :3]),
                        position=tuple(float(value) for value in transform[:3, 3]),
                        visible=False,
                    )
                    new_controls = controls
                    new_handles.append(controls)

                    async def synchronize_source_frame(
                        event: Any,
                        *,
                        owned_frame: Any = frame,
                        owned_controls: Any = controls,
                    ) -> None:
                        del event
                        with self._scene_mutex:
                            if (
                                self._manual_frame is not owned_frame
                                or self._manual_controls is not owned_controls
                            ):
                                return
                            owned_frame.wxyz = owned_controls.wxyz
                            owned_frame.position = owned_controls.position

                    controls.on_update(synchronize_source_frame)
                    new_handles.append(
                        self._server.scene.add_point_cloud(
                            f"{generation_root}/source_transform/"
                            f"{scene_key(feedback.proposal.source_agent)}",
                            points=source,
                            colors=source_color,
                            point_size=self._point_size,
                            precision="float32",
                        )
                    )
                    target_segments = _segments(
                        feedback.diagnostics.target_trajectory
                    )
                    source_segments = _segments(
                        feedback.diagnostics.source_trajectory
                    )
                    if target_segments is not None:
                        new_handles.append(
                            self._server.scene.add_line_segments(
                                f"{generation_root}/target_trajectory",
                                points=target_segments,
                                colors=target_color,
                                thickness=0.03,
                                thickness_units="world",
                            )
                        )
                    if source_segments is not None:
                        new_handles.append(
                            self._server.scene.add_line_segments(
                                f"{generation_root}/source_transform/"
                                "source_trajectory",
                                points=source_segments,
                                colors=source_color,
                                thickness=0.03,
                                thickness_units="world",
                            )
                        )
                    loop_groups = (
                        (True, (51, 230, 77), "inliers"),
                        (False, (242, 51, 51), "outliers"),
                    )
                    for inlier, color, label in loop_groups:
                        pairs = [
                            (loop.target, loop.source)
                            for loop in feedback.diagnostics.descriptor_loops
                            if loop.inlier is inlier
                        ]
                        if pairs:
                            new_handles.append(
                                self._server.scene.add_line_segments(
                                    f"{generation_root}/{label}",
                                    points=np.asarray(pairs, dtype=np.float32),
                                    colors=color,
                                    thickness=0.02,
                                    thickness_units="world",
                                )
                            )
            except Exception:
                with self._server.atomic():
                    for handle in reversed(new_handles):
                        try:
                            handle.remove()
                        except Exception:
                            pass
                raise
            set_committed_visible = getattr(
                self._commit_source, "set_presentation_visible", None
            )
            if callable(set_committed_visible):
                set_committed_visible(False)
            old_handles = self._scene_handles
            self._scene_handles = new_handles
            self._manual_frame = new_frame
            self._manual_controls = new_controls
            self._manual_mode = (
                _enum_name(getattr(feedback.proposal, "method", ""))
                == "MANUAL"
                and _enum_name(
                    getattr(getattr(feedback, "attempt_status", None), "state", "")
                )
                != "RUNNING"
            )
            self._manual_controls.visible = self._manual_mode
            with self._server.atomic():
                for handle in reversed(old_handles):
                    try:
                        handle.remove()
                    except Exception:
                        pass

    def _update_gui(self) -> None:
        with self._condition:
            feedback = self._feedback
        matching = self._matching_job() if feedback is not None else None
        running = feedback is not None and _enum_name(feedback.attempt_status.state) == "RUNNING"
        active_review = (
            feedback is not None
            and _enum_name(feedback.review_state) == "ACTIVE"
        )
        if self._panel_handle is not None:
            self._panel_handle.visible = bool(
                active_review or self._accepted_source is not None
            )
        enabled = active_review and matching is not None and not running
        with self._scene_mutex:
            if not enabled:
                self._manual_mode = False
            manual_mode = self._manual_mode
            if self._manual_controls is not None:
                self._manual_controls.visible = manual_mode
        for button in self._normal_buttons:
            button.visible = not manual_mode
            button.disabled = not enabled
        for button in self._manual_buttons:
            button.visible = manual_mode
            button.disabled = not enabled
        if self._status is None:
            return
        if feedback is None:
            content = "### Alignment Review\nIdle"
        else:
            proposal = feedback.proposal
            metrics = proposal.metrics
            mode = "interactive" if matching is not None else "read-only"
            content = "\n".join((
                "### Alignment Review",
                f"- **Agents:** `{proposal.target_agent}` ← `{proposal.source_agent}`",
                f"- **Request/session:** `{proposal.request_id}` / `{feedback.session_revision}`",
                f"- **Method/state:** `{_enum_name(proposal.method)}` / `{_enum_name(feedback.attempt_status.state)}`",
                f"- **Mode:** `{mode}`",
                "- **Display:** target is fixed; source follows the gizmo",
                f"- **Inliers:** rotation `{metrics.rotation_inliers}`, final `{metrics.final_inliers}`, consensus `{metrics.consensus_size}`",
                f"- **Message:** `{feedback.attempt_status.message or feedback.terminal_message or '-'}`",
            ))
        if self._last_error:
            content += f"\n- **Error:** `{self._last_error}`"
        self._status.content = content

    def _on_presentation_commit(self, agent: str, _revision: int) -> None:
        if self._accepted_source == agent:
            self._accepted_source = None
            self._clear_scene()
            with self._condition:
                self._feedback = None
                self._key = None
                self._dirty = True
                self._condition.notify_all()

    def _clear_scene(self) -> None:
        with self._scene_mutex:
            set_committed_visible = getattr(
                self._commit_source, "set_presentation_visible", None
            )
            if callable(set_committed_visible):
                set_committed_visible(True)
            handles, self._scene_handles = self._scene_handles, []
            self._manual_frame = None
            self._manual_controls = None
            self._manual_mode = False
            for handle in reversed(handles):
                try:
                    handle.remove()
                except Exception:
                    pass
