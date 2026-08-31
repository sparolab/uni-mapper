from __future__ import annotations

from collections import deque
import json
import queue
import threading
from typing import Any, Callable
from urllib.parse import quote

import numpy as np

from .conversion import RenderCandidate, agent_color
from .state import EventBuffer, PresentationState, enum_name
from .workers import (
    CommandDispatcher,
    ControlPoller,
    VisualizationRequest,
    VisualizationWorker,
    WorkerResult,
    wait_for_job,
)


class IridescenceApplication:
    """Main-thread Iridescence renderer over the public OpenLMM Runtime API."""

    def __init__(
        self,
        runtime: Any,
        *,
        preview_voxel_size_m: float | None = None,
        auto_run: bool = False,
    ) -> None:
        from pyridescence import glk, guik, imgui

        self.runtime = runtime
        self.glk = glk
        self.guik = guik
        self.imgui = imgui
        self.auto_run = auto_run
        self.results: "queue.SimpleQueue[WorkerResult]" = queue.SimpleQueue()
        self.events = EventBuffer(1024)
        self.presentation = PresentationState()
        self.commands = CommandDispatcher(self.results)
        self.visualizations = VisualizationWorker(runtime, self.results, preview_voxel_size_m)
        self.poller = ControlPoller(runtime, self.results)
        self.viewer: Any | None = None
        self.subscription: Any | None = None
        self.snapshot: Any | None = None
        self.feedback: Any | None = None
        self.logs: tuple[str, ...] = ()
        self.event_history: deque[Any] = deque(maxlen=512)
        self.active_job: Any | None = None
        self.job_waiter: threading.Thread | None = None
        self.message = "Starting"
        self.error = ""
        self._progress_current = 0
        self._progress_total = 0
        self._progress_detail = ""
        self.color_mode = "HEIGHT"
        self.epoch = 0
        self._last_runtime_revision = 0
        self._agents: tuple[str, ...] = ()
        self._node_descriptors: tuple[Any, ...] = ()
        self._agent_colors: dict[str, tuple[float, float, float, float]] = {}
        self._pose_positions: dict[str, np.ndarray] = {}
        self._inter_edges: dict[str, tuple[tuple[str, int, str, int], ...]] = {}
        self._stopping = False
        self._started = False
        self._auto_run_submitted = False
        self._config_domain: Any | None = None
        self._config_documents: Any | None = None
        self._config_candidates: Any | None = None
        self._config_text = ""
        self._config_selected: str | None = None
        self._config_popup_requested = False
        self._alignment_key: tuple[int, int] | None = None
        self._alignment_control: Any | None = None
        self._alignment_accepted_source: str | None = None
        self._picked_point: tuple[float, float, float] | None = None

    def run(self, stop_event: threading.Event | None = None) -> None:
        if self._started:
            raise RuntimeError("application can only be run once")
        self._started = True
        try:
            self.viewer = self.guik.LightViewer.instance(title="OpenLMM Iridescence")
            if not self.viewer.ok():
                raise RuntimeError("Iridescence viewer initialization failed")
            self.viewer.enable_vsync()
            self.viewer.set_point_shape(0.025, True, True)
            self.viewer.register_drawable_filter("openlmm_visibility", self._drawable_visible)
            self.viewer.register_ui_callback("openlmm", self._frame)
            self.commands.start()
            self.visualizations.start()
            self.subscription = self.runtime.subscribe_events(self.events.push)
            self.commands.submit(
                "alignment-feedback-enable",
                lambda: self.runtime.set_alignment_feedback_enabled(True),
            )
            self.commands.submit("node-descriptors", self.runtime.node_descriptors)
            self.poller.start()
            while not self._stopping and (stop_event is None or not stop_event.is_set()):
                if not self.viewer.spin_once():
                    break
        finally:
            self.close()

    def close(self) -> None:
        if self._stopping:
            return
        self._stopping = True
        subscription, self.subscription = self.subscription, None
        if subscription is not None:
            try:
                subscription.close()
            except Exception:
                pass
        if self.active_job is not None:
            try:
                self.active_job.cancel()
            except Exception:
                pass
        self.poller.close()
        self.visualizations.close()
        self.commands.close()
        try:
            if self.runtime.is_open():
                self.runtime.set_alignment_feedback_enabled(False)
        except Exception:
            pass
        if self.job_waiter is not None and self.job_waiter is not threading.current_thread():
            self.job_waiter.join()
        if self.viewer is not None:
            try:
                self.viewer.remove_ui_callback("openlmm")
                self.viewer.remove_drawable_filter("openlmm_visibility")
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
        try:
            self.guik.destroy()
        except Exception:
            pass

    def _frame(self) -> None:
        self._drain_results()
        self._drain_events()
        self._pick_point()
        self._draw_pipeline_panel()
        self._draw_agents_panel()
        self._draw_logs_panel()
        self._draw_event_panel()
        self._draw_config_modal()
        self._draw_alignment_panel()

    def _drain_results(self) -> None:
        while True:
            try:
                result = self.results.get_nowait()
            except queue.Empty:
                return
            if result.error is not None:
                self.error = f"{result.label}: {type(result.error).__name__}: {result.error}"
                if result.kind == "visualization":
                    self.presentation.fail(result.value.token)
                elif result.kind == "job" and self.active_job is result.value:
                    self.active_job = None
                    self.poller.refresh()
                continue
            if result.kind == "snapshot":
                self._install_snapshot(result.value)
            elif result.kind == "alignment":
                self._install_alignment(result.value)
            elif result.kind == "logs":
                self.logs = tuple(result.value)
            elif result.kind == "visualization":
                self._commit_candidate(result.value)
            elif result.kind == "command":
                self._complete_command(result)
            elif result.kind == "job":
                if self.active_job is result.value:
                    self.message = f"Job {result.value.id} completed"
                    self.active_job = None
                self.poller.refresh()

    def _complete_command(self, result: WorkerResult) -> None:
        value = result.value
        if result.label.startswith("config-open:"):
            documents, candidates, domain = value
            self._config_documents = documents
            self._config_candidates = candidates
            self._config_domain = domain
            document = next(item for item in documents.documents if item.domain == domain)
            self._config_text = document.canonical_json
            self._config_selected = document.selected_document
            self._config_popup_requested = True
            return
        if result.label == "node-descriptors":
            self._node_descriptors = tuple(value)
            return
        if hasattr(value, "wait") and hasattr(value, "id"):
            self.active_job = value
            self.message = f"Job {value.id} submitted"
            self.job_waiter = wait_for_job(value, self.results)
        else:
            self.message = f"{result.label} completed"
            self.poller.refresh()

    def _install_snapshot(self, snapshot: Any) -> None:
        previous_revision = self._last_runtime_revision
        revision = int(snapshot.pipeline.runtime_revision)
        if previous_revision and revision < previous_revision:
            self.epoch += 1
        self._last_runtime_revision = revision
        agents = tuple(snapshot.pipeline.agents)
        removed = self.presentation.reset_epoch(self.epoch, agents)
        self.visualizations.remove_agents(removed)
        for agent in removed:
            self._remove_agent(agent)
            self._pose_positions.pop(agent, None)
            self._inter_edges.pop(agent, None)
        catalog_changed = agents != self._agents
        self._agents = agents
        self._agent_colors = {agent: agent_color(index) for index, agent in enumerate(agents)}
        self.snapshot = snapshot
        if catalog_changed or revision != previous_revision:
            for agent in agents:
                self._request_visualization(agent, revision, include_points=True)
        if self.auto_run and not self._auto_run_submitted:
            self._auto_run_submitted = True
            self._submit_job("run-all", self.runtime.run_all)

    def _request_visualization(self, agent: str, revision: int, *, include_points: bool) -> None:
        token = self.presentation.request(agent, revision)
        self.visualizations.request(
            VisualizationRequest(token, self.color_mode, include_points)
        )

    def _commit_candidate(self, candidate: RenderCandidate) -> None:
        if not self.presentation.accepts(candidate.token):
            return
        viewer = self.viewer
        if viewer is None:
            return
        prefix = self._prefix(candidate.agent)
        created: list[tuple[str, Any, Any]] = []
        if candidate.points is not None:
            cloud = self.glk.PointCloudBuffer(candidate.points)
            if (
                self.color_mode == "INTENSITY"
                and candidate.point_kind == "FINAL_STATIC_MAP"
                and candidate.intensity_colors is not None
            ):
                cloud.add_color(candidate.intensity_colors)
                setting = self.guik.VertexColor()
            elif self.color_mode in {"AGENT", "INTENSITY"}:
                setting = self.guik.FlatColor(*self._agent_colors[candidate.agent])
            else:
                setting = self.guik.Rainbow()
                if candidate.metadata.min_bound and candidate.metadata.max_bound:
                    setting.add(
                        "z_range",
                        np.asarray(
                            [candidate.metadata.min_bound[2], candidate.metadata.max_bound[2]],
                            dtype=np.float32,
                        ),
                    )
            setting.set_point_shape(0.025, True, True)
            created.append((prefix + "/points", cloud, setting))
        if candidate.trajectory is not None:
            colors = np.tile(np.asarray(self._agent_colors[candidate.agent], dtype=np.float32), (len(candidate.trajectory), 1))
            created.append((
                prefix + "/trajectory",
                self.glk.ThinLines(candidate.trajectory, colors, line_strip=True, line_width=1.0),
                self.guik.VertexColor(),
            ))
        for suffix, vertices, color in (
            ("intra_loops", candidate.intra_loops, (1.0, 0.55, 0.0, 1.0)),
            ("inter_loops", candidate.inter_loops, (1.0, 0.1, 0.1, 1.0)),
        ):
            if vertices is not None and candidate.phase != "MAP_UPDATE":
                colors = np.tile(np.asarray(color, dtype=np.float32), (len(vertices), 1))
                created.append((
                    prefix + "/" + suffix,
                    self.glk.ThinLines(vertices, colors, line_strip=False, line_width=1.0),
                    self.guik.VertexColor(),
                ))
        for name, drawable, setting in created:
            viewer.update_drawable(name, drawable, setting)
        for name in tuple(viewer.get_drawables()):
            if name.startswith(prefix + "/pose/"):
                viewer.remove_drawable(name)
        for index, transform in enumerate(candidate.pose_axes):
            viewer.update_coord(
                f"{prefix}/pose/{index}",
                self.guik.VertexColor(np.asarray(transform, dtype=np.float32)).scale(0.25),
            )
        if self.presentation.commit(candidate.metadata):
            self._pose_positions[candidate.agent] = candidate.pose_positions
            self._inter_edges[candidate.agent] = (
                () if candidate.phase == "MAP_UPDATE" else candidate.inter_edges
            )
            self._render_inter_loops()
            self._apply_height_range()
            if self._alignment_accepted_source == candidate.agent:
                self._clear_alignment_drawables()
                self._alignment_accepted_source = None
            self.message = f"Displayed {candidate.agent} revision {candidate.token.source_revision}"

    def _install_alignment(self, feedback: Any | None) -> None:
        self.feedback = feedback
        if feedback is None:
            return
        key = (int(feedback.proposal.request_id), int(feedback.session_revision))
        if key == self._alignment_key:
            return
        target = np.asarray(feedback.target_points, dtype=np.float32)
        source = np.asarray(feedback.source_points, dtype=np.float32)
        if target.ndim != 2 or source.ndim != 2 or target.shape[1] < 3 or source.shape[1] < 3:
            self.error = "alignment points have an invalid shape"
            return
        if len(target) + len(source) > 2_000_000:
            self.error = "alignment visualization exceeds the 2,000,000 point limit"
            return
        target_xyz = np.ascontiguousarray(target[:, :3])
        source_xyz = np.ascontiguousarray(source[:, :3])
        target_cloud = self.glk.PointCloudBuffer(target_xyz)
        source_cloud = self.glk.PointCloudBuffer(source_xyz)
        transform = np.asarray(feedback.proposal.target_T_source, dtype=np.float32)
        control = self.guik.ModelControl("openlmm_alignment", transform)
        control.set_gizmo_enabled(True)
        self.viewer.update_drawable(
            "openlmm/alignment/target", target_cloud, self.guik.FlatGreen()
        )
        self.viewer.update_drawable(
            "openlmm/alignment/source", source_cloud, self.guik.FlatRed(transform)
        )
        self._alignment_control = control
        self._alignment_key = key

    def _drain_events(self) -> None:
        for event in self.events.drain(128):
            self.event_history.append(event)
            name = enum_name(event.type).upper()
            if name == "ALIGNMENT_PROPOSAL_ACCEPTED" and self.feedback is not None:
                self._alignment_accepted_source = self.feedback.proposal.source_agent
            if name == "PROGRESS_UPDATED":
                self._progress_current = int(getattr(event, "progress_current", 0))
                self._progress_total = int(getattr(event, "progress_total", 0))
                algorithm = getattr(event, "algorithm_progress", None)
                self._progress_detail = (
                    str(getattr(algorithm, "operation", ""))
                    if algorithm is not None
                    else str(getattr(event, "message", ""))
                )
            affected = tuple(getattr(event, "affected_agents", ()) or ())
            if name in {"ARTIFACT_COMMITTED", "ARTIFACT_INVALIDATED", "STAGE_COMPLETED", "JOB_COMPLETED"}:
                targets = affected or self._agents
                revision = self._last_runtime_revision
                for agent in targets:
                    if agent in self._agents:
                        self._request_visualization(agent, revision, include_points=True)
        if self.events.consume_resync():
            self.message = "Event gap detected; synchronizing authoritative state"
            self.poller.refresh()

    def _submit_job(self, label: str, submit: Callable[[], Any]) -> None:
        if self.active_job is not None:
            self.error = "another OpenLMM job is active"
            return
        try:
            self.commands.submit(label, submit)
            self.message = f"Submitting {label}"
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"

    def _draw_pipeline_panel(self) -> None:
        imgui = self.imgui
        visible, opened = imgui.begin("OpenLMM Pipeline", True)
        if visible:
            snapshot = self.snapshot
            if snapshot is None:
                imgui.text("Waiting for runtime snapshot...")
            else:
                pipeline = snapshot.pipeline
                imgui.text(f"Status: {enum_name(snapshot.status)}")
                imgui.text(f"Runtime revision: {pipeline.runtime_revision}")
                imgui.text(f"Config revision: {pipeline.config_revision}")
                imgui.text(f"Agents: {len(pipeline.agents)}  Artifacts: {len(pipeline.artifacts)}")
                queued, evictions, resync = self.events.diagnostics
                imgui.text(f"Event queue: {queued}/1024  evictions: {evictions}  resync: {resync}")
                imgui.text(f"Visualization cache: {len(self.presentation.metadata)} metadata entries")
                if imgui.button("Run All"):
                    self._submit_job("run-all", self.runtime.run_all)
                imgui.same_line()
                if imgui.button("Cancel") and self.active_job is not None:
                    try:
                        self.active_job.cancel()
                    except Exception as error:
                        self.error = str(error)
                import open_lmm
                for stage in open_lmm.Stage:
                    label = enum_name(stage).replace("_", " ").title()
                    if imgui.button(f"Run {label}"):
                        self._submit_job(
                            f"stage-{enum_name(stage)}",
                            lambda stage=stage: self.runtime.run_stage(stage),
                        )
                    active_stage = pipeline.job is not None and pipeline.job.active_stage == stage
                    if active_stage:
                        fraction = (
                            min(1.0, self._progress_current / self._progress_total)
                            if self._progress_total > 0
                            else 0.0
                        )
                        imgui.progress_bar(
                            fraction,
                            overlay=self._progress_detail or pipeline.job.message,
                        )
                    if stage != open_lmm.Stage.SAVE:
                        imgui.same_line()
                        if imgui.button(f"Configure {label}"):
                            domain = {
                                open_lmm.Stage.DATA_LOAD: open_lmm.ConfigDomain.GLOBAL,
                                open_lmm.Stage.ALIGNMENT: open_lmm.ConfigDomain.LOOP_DETECTOR,
                                open_lmm.Stage.MAP_UPDATE: open_lmm.ConfigDomain.DYNAMIC_REMOVER,
                            }[stage]
                            self._request_config(domain)
                if pipeline.job is not None:
                    job = pipeline.job
                    imgui.text(f"Job {job.id}: {enum_name(job.state)} — {job.message}")
                    cancellation = job.cancellation
                    imgui.text(
                        "Cancellation: requested=%s observed=%s completed=%s mode=%s"
                        % (
                            cancellation.cancel_requested_at_unix_ns is not None,
                            cancellation.cancel_observed_at_unix_ns is not None,
                            cancellation.cancel_completed_at_unix_ns is not None,
                            enum_name(cancellation.capability.mode),
                        )
                    )
                _, selected = imgui.combo("Color", ("HEIGHT", "INTENSITY", "AGENT").index(self.color_mode), ["HEIGHT", "INTENSITY", "AGENT"])
                selected_mode = ("HEIGHT", "INTENSITY", "AGENT")[selected]
                if selected_mode != self.color_mode:
                    self.color_mode = selected_mode
                    for agent in self._agents:
                        self._request_visualization(agent, self._last_runtime_revision, include_points=True)
                for agent in self._agents:
                    changed, enabled = imgui.checkbox(f"{agent}##visible", self.presentation.visible(agent))
                    if changed:
                        self.presentation.set_visible(agent, enabled)
                        self._render_inter_loops()
                        self._apply_height_range()
            if self.message:
                imgui.text_wrapped(self.message)
            if self.error:
                imgui.text_wrapped("Error: " + self.error)
        imgui.end()
        if not opened and self.viewer is not None:
            self.viewer.close()

    def _draw_agents_panel(self) -> None:
        imgui = self.imgui
        visible, _ = imgui.begin("Agents and Artifacts", True)
        if visible:
            imgui.text("Picked point: " + ("-" if self._picked_point is None else "%.3f, %.3f, %.3f" % self._picked_point))
            if self.snapshot is not None:
                for descriptor in self._node_descriptors:
                    requires_agent = enum_name(descriptor.scope).upper() == "PER_AGENT"
                    if requires_agent:
                        for agent in self._agents:
                            if imgui.small_button(f"{descriptor.name} ({agent})"):
                                self._submit_job(
                                    f"node-{descriptor.name}-{agent}",
                                    lambda descriptor=descriptor, agent=agent: self.runtime.run_node(descriptor.id, agent=agent),
                                )
                    elif imgui.small_button(descriptor.name):
                        self._submit_job(
                            f"node-{descriptor.name}",
                            lambda descriptor=descriptor: self.runtime.run_node(descriptor.id),
                        )
                for agent in self._agents:
                    if imgui.small_button(f"Optimize Through ({agent})"):
                        self._submit_job(
                            f"optimize-through-{agent}",
                            lambda agent=agent: self.runtime.optimize_through(agent),
                        )
                imgui.separator()
                for artifact in self.snapshot.pipeline.artifacts:
                    imgui.text_wrapped(
                        f"{enum_name(artifact.type)} [{enum_name(artifact.state)}] "
                        f"agent={artifact.agent or '-'} rev={artifact.revision} {artifact.detail}"
                    )
        imgui.end()

    def _draw_logs_panel(self) -> None:
        imgui = self.imgui
        visible, _ = imgui.begin("Runtime Logs", True)
        if visible:
            for line in self.logs:
                imgui.text_unformatted(line)
        imgui.end()

    def _draw_event_panel(self) -> None:
        imgui = self.imgui
        visible, _ = imgui.begin("Job and Event Log", True)
        if visible:
            for event in tuple(self.event_history)[-128:]:
                imgui.text_wrapped(
                    f"#{event.sequence} job={event.job_id} {enum_name(event.type)} {event.message}"
                )
        imgui.end()

    def _request_config(self, domain: Any) -> None:
        try:
            self.commands.submit(
                f"config-open:{enum_name(domain)}",
                lambda domain=domain: (
                    self.runtime.config_documents(),
                    self.runtime.config_candidates(),
                    domain,
                ),
            )
        except Exception as error:
            self.error = str(error)

    def _draw_config_modal(self) -> None:
        imgui = self.imgui
        if self._config_popup_requested:
            imgui.open_popup("Configuration")
            self._config_popup_requested = False
        if not imgui.begin_popup_modal("Configuration", imgui.WindowFlags_AlwaysAutoResize):
            return
        imgui.text(f"Domain: {enum_name(self._config_domain)}")
        changed, text = imgui.input_text_multiline(
            "Canonical JSON", self._config_text, (720, 420), buffer_size=1_048_576
        )
        if changed:
            self._config_text = text
        candidates = () if self._config_candidates is None else tuple(
            item for item in self._config_candidates.candidates if item.domain == self._config_domain
        )
        if candidates:
            items = [item.selected_document for item in candidates]
            current = items.index(self._config_selected) if self._config_selected in items else 0
            selected_changed, current = imgui.combo("Selected document", current, items)
            if selected_changed:
                candidate = candidates[current]
                self._config_selected = candidate.selected_document
                self._config_text = candidate.canonical_json
        if imgui.button("Apply"):
            try:
                json.loads(self._config_text)
                self.commands.submit("config-apply", self._apply_config)
                imgui.close_current_popup()
            except Exception as error:
                self.error = f"configuration validation: {error}"
        imgui.same_line()
        if imgui.button("Cancel"):
            imgui.close_current_popup()
        imgui.end_popup()

    def _apply_config(self) -> Any:
        import open_lmm
        documents = self._config_documents
        if documents is None or self._config_domain is None:
            raise RuntimeError("configuration draft is unavailable")
        expected = open_lmm.Revision(documents.runtime_revision, documents.config_revision)
        if self._config_domain == open_lmm.ConfigDomain.GLOBAL:
            return self.runtime.replace_root_config(self._config_text, expected=expected)
        return self.runtime.apply_config(
            self._config_domain,
            self._config_text,
            expected=expected,
            selected_document=self._config_selected,
        )

    def _draw_alignment_panel(self) -> None:
        imgui = self.imgui
        visible, _ = imgui.begin("Alignment Review", True)
        if visible:
            feedback = self.feedback
            if feedback is None:
                imgui.text("No active alignment review")
            else:
                proposal = feedback.proposal
                metrics = proposal.metrics
                imgui.text(f"{proposal.target_agent} <- {proposal.source_agent}")
                imgui.text(f"Method: {enum_name(proposal.method)}  Session: {feedback.session_revision}")
                imgui.text(f"Correspondences: {metrics.correspondence_count}")
                imgui.text(f"Rotation/final inliers: {metrics.rotation_inliers}/{metrics.final_inliers}")
                imgui.text(f"Consensus: {metrics.consensus_size}  Fitness: {metrics.fitness}")
                imgui.text_wrapped(feedback.attempt_status.message)
                active = enum_name(feedback.review_state).upper() == "ACTIVE" and self.active_job is not None
                if active:
                    for label, decision in self._alignment_decisions():
                        if imgui.button(label):
                            self._respond_alignment(decision, manual=label == "Apply Manual")
                    if self._alignment_control is not None:
                        if imgui.button("Reset Proposal"):
                            self._alignment_control.set_model_matrix(
                                np.asarray(proposal.target_T_source, dtype=np.float32)
                            )
                        imgui.same_line()
                        if imgui.button("Reset Identity"):
                            self._alignment_control.set_model_matrix(
                                np.eye(4, dtype=np.float32)
                            )
                        self._alignment_control.draw_ui()
                        self._alignment_control.draw_gizmo_ui()
                        self._alignment_control.draw_gizmo()
                        try:
                            setting, _ = self.viewer.find_drawable("openlmm/alignment/source")
                            setting.add("model_matrix", self._alignment_control.model_matrix())
                        except Exception:
                            pass
                else:
                    imgui.text_wrapped(feedback.terminal_message or "Read-only alignment state")
                if feedback.attempt_history:
                    imgui.separator()
                    for attempt in feedback.attempt_history:
                        imgui.text_wrapped(
                            f"#{attempt.attempt} {enum_name(attempt.method)} {enum_name(attempt.state)}: {attempt.message}"
                        )
        imgui.end()

    @staticmethod
    def _alignment_decisions() -> tuple[tuple[str, Any], ...]:
        import open_lmm
        return (
            ("Accept", open_lmm.AlignmentDecision.ACCEPT),
            ("Try KISS", open_lmm.AlignmentDecision.TRY_KISS_MATCHER),
            ("Try Descriptor", open_lmm.AlignmentDecision.TRY_DESCRIPTOR),
            ("Apply Manual", open_lmm.AlignmentDecision.MANUAL),
            ("Exclude Agent", open_lmm.AlignmentDecision.EXCLUDE_AGENT),
            ("Cancel Alignment", open_lmm.AlignmentDecision.CANCEL),
        )

    def _respond_alignment(self, decision: Any, *, manual: bool) -> None:
        import open_lmm
        if self.feedback is None or self.active_job is None:
            self.error = "alignment response requires the exact active Job"
            return
        response = open_lmm.AlignmentResponse(
            self.feedback.proposal.request_id,
            decision,
            self.feedback.session_revision,
            None if not manual or self._alignment_control is None else self._alignment_control.model_matrix(),
        )
        try:
            self.commands.submit(
                f"alignment-{enum_name(decision)}",
                lambda: self.runtime.respond_to_alignment(self.active_job, response),
            )
        except Exception as error:
            self.error = str(error)

    def _pick_point(self) -> None:
        if self.viewer is None:
            return
        try:
            point = self.viewer.pick_point(0, 2)
            if point is not None:
                self._picked_point = tuple(float(value) for value in np.asarray(point)[:3])
        except Exception:
            pass

    def _drawable_visible(self, name: str) -> bool:
        if not name.startswith("openlmm/agent/"):
            return True
        remainder = name[len("openlmm/agent/"):]
        encoded = remainder.split("/", 1)[0]
        for agent in self._agents:
            if quote(agent, safe="-_.~") == encoded:
                return self.presentation.visible(agent)
        return False

    @staticmethod
    def _prefix(agent: str) -> str:
        return "openlmm/agent/" + quote(agent, safe="-_.~")

    def _remove_agent(self, agent: str) -> None:
        if self.viewer is not None:
            prefix = self._prefix(agent)
            for name in tuple(self.viewer.get_drawables()):
                if name.startswith(prefix):
                    self.viewer.remove_drawable(name)

    def _clear_alignment_drawables(self) -> None:
        if self.viewer is not None:
            for name in tuple(self.viewer.get_drawables()):
                if name.startswith("openlmm/alignment/"):
                    self.viewer.remove_drawable(name)
        self._alignment_key = None
        self._alignment_control = None

    def _render_inter_loops(self) -> None:
        if self.viewer is None:
            return
        vertices: list[np.ndarray] = []
        seen: set[tuple[str, int, str, int]] = set()
        for edges in self._inter_edges.values():
            for from_agent, from_index, to_agent, to_index in edges:
                key = (from_agent, from_index, to_agent, to_index)
                reverse = (to_agent, to_index, from_agent, from_index)
                if key in seen or reverse in seen:
                    continue
                seen.add(key)
                if not self.presentation.visible(from_agent) or not self.presentation.visible(to_agent):
                    continue
                from_poses = self._pose_positions.get(from_agent)
                to_poses = self._pose_positions.get(to_agent)
                if (
                    from_poses is None
                    or to_poses is None
                    or from_index >= len(from_poses)
                    or to_index >= len(to_poses)
                ):
                    continue
                vertices.extend((from_poses[from_index], to_poses[to_index]))
        name = "openlmm/inter_loops"
        if not vertices:
            try:
                self.viewer.remove_drawable(name)
            except Exception:
                pass
            return
        points = np.ascontiguousarray(vertices, dtype=np.float32)
        colors = np.tile(np.asarray((1.0, 0.1, 0.1, 1.0), dtype=np.float32), (len(points), 1))
        self.viewer.update_drawable(
            name,
            self.glk.ThinLines(points, colors, line_strip=False, line_width=1.0),
            self.guik.VertexColor(),
        )

    def _apply_height_range(self) -> None:
        if self.viewer is None or self.color_mode != "HEIGHT":
            return
        bounds = tuple(
            metadata
            for metadata in self.presentation.metadata
            if metadata.visible
            and metadata.min_bound is not None
            and metadata.max_bound is not None
        )
        if not bounds:
            return
        z_range = np.asarray(
            [
                min(metadata.min_bound[2] for metadata in bounds),
                max(metadata.max_bound[2] for metadata in bounds),
            ],
            dtype=np.float32,
        )
        if z_range[0] == z_range[1]:
            z_range += np.asarray([-0.5, 0.5], dtype=np.float32)
        for agent in self._agents:
            try:
                setting, _ = self.viewer.find_drawable(self._prefix(agent) + "/points")
                setting.add("z_range", z_range)
            except Exception:
                pass
