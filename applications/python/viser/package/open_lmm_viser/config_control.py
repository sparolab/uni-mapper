from __future__ import annotations

import copy
import json
import math
import os
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


_MAX_DATASET_DIRECTORIES = 1024


@dataclass(frozen=True, slots=True)
class _Candidate:
    model: str
    selected_document: str
    document: dict[str, Any]


@dataclass(frozen=True, slots=True)
class _CommittedDraft:
    runtime_revision: int
    config_revision: int
    root: dict[str, Any]
    alignment: dict[str, Any]
    loop_model: str
    remover_model: str
    loop_candidates: tuple[_Candidate, ...]
    remover_candidates: tuple[_Candidate, ...]


def _discover_dataset_directories(root: str) -> tuple[str, ...]:
    if not root:
        raise ValueError("dataset root is required")
    try:
        with os.scandir(root) as entries:
            directories: list[str] = []
            for entry in entries:
                if not entry.is_dir():
                    continue
                if len(directories) >= _MAX_DATASET_DIRECTORIES:
                    raise ValueError(
                        "dataset root exceeds the 1024-directory UI limit"
                    )
                directories.append(entry.name)
    except OSError as error:
        raise ValueError(
            f"dataset root cannot be scanned: {root}: {error.strerror or error}"
        ) from error
    return tuple(sorted(directories))


class TransactionalConfigPanel:
    """Revision-bound form over RuntimeClient config transactions."""

    def __init__(
        self,
        runtime: Any,
        server: Any,
        *,
        global_domain: object,
        loop_domain: object,
        remover_domain: object,
        revision_factory: Callable[[int, int], object],
        on_commit: Callable[[], None] = lambda: None,
    ) -> None:
        self._runtime = runtime
        self._server = server
        self._global_domain = global_domain
        self._loop_domain = loop_domain
        self._remover_domain = remover_domain
        self._revision_factory = revision_factory
        self._on_commit = on_commit
        self._condition = threading.Condition()
        self._state = "new"
        self._busy = False
        self._dirty = False
        self._refresh_values = False
        self._message = "Not loaded"
        self._draft: _CommittedDraft | None = None
        self._operation: threading.Thread | None = None
        self._ui_worker: threading.Thread | None = None
        self._handles: list[Any] = []
        self._buttons: list[Any] = []
        self._status: Any | None = None
        self._dataset_root: Any | None = None
        self._agent_folder: Any | None = None
        self._agent_order: Any | None = None
        self._agent_handles: dict[str, Any] = {}
        self._selected_agents: list[str] = []
        self._dataset_catalog: tuple[str, ...] = ()
        self._catalog_root: str | None = None
        self._output_root: Any | None = None
        self._kiss_voxel: Any | None = None
        self._kiss_quatro: Any | None = None
        self._pose_nn: Any | None = None
        self._inter_loop_spacing: Any | None = None
        self._loop_model: Any | None = None
        self._remover_model: Any | None = None

    def start(self) -> None:
        with self._condition:
            if self._state != "new":
                raise RuntimeError("TransactionalConfigPanel can only start once")
        draft = self._query_committed()
        with self._condition:
            self._draft = draft
            self._message = "Committed config loaded"
        try:
            self._create_gui(draft)
            worker = threading.Thread(
                target=self._run_ui_worker,
                name="open-lmm-viser-config-ui",
                daemon=False,
            )
            worker.start()
            with self._condition:
                self._ui_worker = worker
                self._state = "started"
                self._mark_dirty_locked(refresh_values=True)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        with self._condition:
            if self._state == "closed":
                return
            operation = self._operation
        if operation is not None and operation is not threading.current_thread():
            operation.join()
        with self._condition:
            self._state = "closed"
            self._dirty = True
            self._condition.notify_all()
            ui_worker, self._ui_worker = self._ui_worker, None
        if ui_worker is not None and ui_worker is not threading.current_thread():
            ui_worker.join()
        if self._agent_order is not None:
            try:
                self._agent_order.remove()
            except Exception:
                pass
            self._agent_order = None
        for handle in self._agent_handles.values():
            try:
                handle.remove()
            except Exception:
                pass
        self._agent_handles.clear()
        for handle in reversed(self._handles):
            try:
                handle.remove()
            except Exception:
                pass
        self._handles.clear()

    def reload(self) -> None:
        self._launch("Reload", self._reload_operation)

    def apply_root(self) -> None:
        values = self._root_values()
        self._launch("Apply Root", lambda: self._apply_root_operation(values))

    def apply_alignment(self) -> None:
        values = self._alignment_values()
        model = str(self._loop_model.value)
        self._launch(
            "Apply Alignment",
            lambda: self._apply_alignment_operation(values, model),
        )

    def apply_remover(self) -> None:
        model = str(self._remover_model.value)
        self._launch(
            "Apply Dynamic Remover",
            lambda: self._apply_remover_operation(model),
        )

    def _create_gui(self, draft: _CommittedDraft) -> None:
        directory = draft.root["directory"]
        alignment = draft.alignment["alignment"]
        self._status = self._add(self._server.gui.add_markdown("**Config:** loading"))
        self._loop_model = self._add(
            self._server.gui.add_dropdown(
                "Loop Detector Model",
                options=tuple(candidate.model for candidate in draft.loop_candidates),
                initial_value=draft.loop_model,
            )
        )
        self._remover_model = self._add(
            self._server.gui.add_dropdown(
                "Dynamic Remover Model",
                options=tuple(
                    candidate.model for candidate in draft.remover_candidates
                ),
                initial_value=draft.remover_model,
            )
        )
        self._dataset_root = self._add(
            self._server.gui.add_text(
                "Dataset Root", str(directory["root_dir_path"])
            )
        )
        refresh_agents = self._button(
            "Refresh Agents", self._refresh_dataset_catalog
        )
        self._agent_folder = self._add(
            self._server.gui.add_folder("Ordered Agents")
        )
        with self._agent_folder:
            self._agent_order = self._server.gui.add_markdown("")
        self._set_agent_catalog(
            str(directory["root_dir_path"]),
            tuple(str(value) for value in directory["sub_dir_list"]),
            preserve_committed=True,
        )
        self._output_root = self._add(
            self._server.gui.add_text(
                "Output Root", str(directory["root_save_dir"])
            )
        )
        self._kiss_voxel = self._add(
            self._server.gui.add_number(
                "KISS Voxel Size", float(alignment["kiss_voxel_size"]), min=0.0
            )
        )
        self._kiss_quatro = self._add(
            self._server.gui.add_checkbox(
                "KISS Use Quatro", bool(alignment["kiss_use_quatro"])
            )
        )
        self._pose_nn = self._add(
            self._server.gui.add_number(
                "Pose NN Maximum Distance",
                float(alignment["pose_nn_distance_threshold"]),
                min=0.0,
            )
        )
        self._inter_loop_spacing = self._add(
            self._server.gui.add_number(
                "Inter-loop Spacing",
                float(alignment["inter_loop_keyframe_spacing_m"]),
                min=0.0,
            )
        )
        reload_button = self._button("Reload Config", self.reload)
        root_button = self._button("Apply Root Config", self.apply_root)
        alignment_button = self._button(
            "Apply Alignment Config", self.apply_alignment
        )
        remover_button = self._button(
            "Apply Dynamic Remover", self.apply_remover
        )
        self._buttons.extend(
            (
                refresh_agents,
                reload_button,
                root_button,
                alignment_button,
                remover_button,
            )
        )

    def _add(self, handle: Any) -> Any:
        self._handles.append(handle)
        return handle

    def _button(self, label: str, callback: Callable[[], None]) -> Any:
        button = self._add(self._server.gui.add_button(label))
        button.on_click(lambda _event: self._handle_callback(callback))
        return button

    def _handle_callback(self, callback: Callable[[], None]) -> None:
        try:
            callback()
        except Exception as error:
            with self._condition:
                self._message = f"Rejected: {type(error).__name__}: {error}"
                self._mark_dirty_locked()

    def _refresh_dataset_catalog(self) -> None:
        root = str(self._dataset_root.value).strip()
        self._set_agent_catalog(root, (), preserve_committed=False)
        with self._condition:
            self._message = f"Found {len(self._dataset_catalog)} agent directories"
            self._mark_dirty_locked()

    def _set_agent_catalog(
        self,
        root: str,
        committed_agents: tuple[str, ...],
        *,
        preserve_committed: bool,
    ) -> None:
        try:
            catalog = _discover_dataset_directories(root)
        except ValueError as error:
            if not preserve_committed:
                raise
            catalog = ()
            with self._condition:
                self._message = f"Dataset scan unavailable: {error}"
        with self._condition:
            if preserve_committed:
                self._selected_agents = list(committed_agents)
            elif root != self._catalog_root:
                self._selected_agents.clear()
            else:
                self._selected_agents = [
                    agent
                    for agent in self._selected_agents
                    if agent in catalog
                ]
            self._dataset_catalog = catalog
            self._catalog_root = root
        self._rebuild_agent_controls()

    def _rebuild_agent_controls(self) -> None:
        for handle in self._agent_handles.values():
            try:
                handle.remove()
            except Exception:
                pass
        self._agent_handles.clear()
        with self._condition:
            selected = tuple(self._selected_agents)
            catalog = self._dataset_catalog
        missing = tuple(agent for agent in selected if agent not in catalog)
        if self._agent_folder is not None:
            with self._agent_folder:
                for agent in (*catalog, *missing):
                    label = agent if agent in catalog else f"{agent} (missing)"
                    handle = self._server.gui.add_checkbox(
                        label, agent in selected
                    )
                    handle.on_update(
                        lambda event, agent=agent: self._select_agent(
                            agent, bool(event.target.value)
                        )
                    )
                    self._agent_handles[agent] = handle
        self._update_agent_order()

    def _select_agent(self, agent: str, selected: bool) -> None:
        with self._condition:
            if selected and agent not in self._selected_agents:
                self._selected_agents.append(agent)
            elif not selected and agent in self._selected_agents:
                self._selected_agents.remove(agent)
        self._update_agent_order()

    def _update_agent_order(self) -> None:
        if self._agent_order is None:
            return
        with self._condition:
            selected = tuple(self._selected_agents)
        order = " → ".join(f"`{agent}`" for agent in selected) or "_None selected_"
        self._agent_order.content = (
            "**Execution order** (uncheck/recheck to move an agent last)  \n"
            + order
        )

    def _launch(self, label: str, operation: Callable[[], None]) -> None:
        with self._condition:
            if self._state != "started":
                raise RuntimeError("config panel is not running")
            if self._busy:
                raise RuntimeError("another config operation is active")
            self._busy = True
            self._message = f"{label} in progress"
            self._mark_dirty_locked()
            worker = threading.Thread(
                target=self._run_operation,
                args=(label, operation),
                name="open-lmm-viser-config-operation",
                daemon=False,
            )
            self._operation = worker
        try:
            worker.start()
        except BaseException:
            with self._condition:
                self._busy = False
                self._operation = None
                self._mark_dirty_locked()
            raise

    def _run_operation(self, label: str, operation: Callable[[], None]) -> None:
        try:
            operation()
        except Exception as error:
            with self._condition:
                self._message = (
                    f"{label} failed: {type(error).__name__}: {error}. "
                    "Reload before retrying if the revision changed."
                )
                self._busy = False
                self._mark_dirty_locked()
            return
        try:
            self._on_commit()
        except Exception:
            pass
        with self._condition:
            self._message = f"{label} committed"
            self._busy = False
            self._mark_dirty_locked(refresh_values=True)

    def _reload_operation(self) -> None:
        draft = self._query_committed()
        with self._condition:
            self._draft = draft
            self._refresh_values = True

    def _apply_root_operation(self, values: tuple[str, tuple[str, ...], str]) -> None:
        with self._condition:
            draft = self._require_draft_locked()
        root = copy.deepcopy(draft.root)
        root["directory"]["root_dir_path"] = values[0]
        root["directory"]["sub_dir_list"] = list(values[1])
        root["directory"]["root_save_dir"] = values[2]
        expected = self._revision_factory(
            draft.runtime_revision, draft.config_revision
        )
        self._runtime.replace_root_config(
            json.dumps(root, ensure_ascii=False, indent=2) + "\n",
            expected=expected,
        )
        self._reload_operation()

    def _apply_alignment_operation(
        self, values: tuple[float, bool, float, float], model: str
    ) -> None:
        with self._condition:
            draft = self._require_draft_locked()
        candidate = self._candidate(draft.loop_candidates, model)
        document = copy.deepcopy(candidate.document)
        alignment = document.setdefault("alignment", {})
        alignment["kiss_voxel_size"] = values[0]
        alignment["kiss_use_quatro"] = values[1]
        alignment["pose_nn_distance_threshold"] = values[2]
        alignment["inter_loop_keyframe_spacing_m"] = values[3]
        expected = self._revision_factory(
            draft.runtime_revision, draft.config_revision
        )
        self._runtime.apply_config(
            self._loop_domain,
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            expected=expected,
            selected_document=candidate.selected_document,
        )
        self._reload_operation()

    def _apply_remover_operation(self, model: str) -> None:
        with self._condition:
            draft = self._require_draft_locked()
        candidate = self._candidate(draft.remover_candidates, model)
        expected = self._revision_factory(
            draft.runtime_revision, draft.config_revision
        )
        self._runtime.apply_config(
            self._remover_domain,
            json.dumps(candidate.document, ensure_ascii=False, indent=2) + "\n",
            expected=expected,
            selected_document=candidate.selected_document,
        )
        self._reload_operation()

    def _root_values(self) -> tuple[str, tuple[str, ...], str]:
        dataset_root = str(self._dataset_root.value).strip()
        output_root = str(self._output_root.value).strip()
        with self._condition:
            agents = tuple(self._selected_agents)
        if not dataset_root or not output_root or not agents:
            raise ValueError("dataset root, output root and agents are required")
        if len(set(agents)) != len(agents):
            raise ValueError("ordered agents must be unique")
        return dataset_root, agents, output_root

    def _alignment_values(self) -> tuple[float, bool, float, float]:
        values = (
            float(self._kiss_voxel.value),
            float(self._pose_nn.value),
            float(self._inter_loop_spacing.value),
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("alignment distances must be positive and finite")
        return values[0], bool(self._kiss_quatro.value), values[1], values[2]

    def _query_committed(self) -> _CommittedDraft:
        documents = self._runtime.config_documents()
        catalog = self._runtime.config_candidates()
        if (
            int(catalog.runtime_revision) != int(documents.runtime_revision)
            or int(catalog.config_revision) != int(documents.config_revision)
        ):
            raise RuntimeError("config documents and candidate catalog revisions differ")
        by_domain = {document.domain: document for document in documents.documents}
        try:
            root = json.loads(by_domain[self._global_domain].canonical_json)
            alignment = json.loads(by_domain[self._loop_domain].canonical_json)
            loop_selected = str(by_domain[self._loop_domain].selected_document)
            remover_selected = str(
                by_domain[self._remover_domain].selected_document
            )
            loop_candidates = self._catalog_candidates(
                catalog.candidates, self._loop_domain
            )
            remover_candidates = self._catalog_candidates(
                catalog.candidates, self._remover_domain
            )
            loop_model = self._selected_model(loop_candidates, loop_selected)
            remover_model = self._selected_model(
                remover_candidates, remover_selected
            )
            directory = root["directory"]
            alignment_values = alignment["alignment"]
            if not isinstance(directory["sub_dir_list"], list):
                raise TypeError("ordered agent list is not an array")
            for key in (
                "kiss_voxel_size",
                "kiss_use_quatro",
                "pose_nn_distance_threshold",
                "inter_loop_keyframe_spacing_m",
            ):
                alignment_values[key]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f"committed config form is unavailable: {error}"
            ) from error
        return _CommittedDraft(
            int(documents.runtime_revision),
            int(documents.config_revision),
            root,
            alignment,
            loop_model,
            remover_model,
            loop_candidates,
            remover_candidates,
        )

    @staticmethod
    def _catalog_candidates(
        candidates: Any, domain: object
    ) -> tuple[_Candidate, ...]:
        result = tuple(
            _Candidate(
                str(candidate.model),
                str(candidate.selected_document),
                json.loads(candidate.canonical_json),
            )
            for candidate in candidates
            if candidate.domain == domain
        )
        if not result:
            raise RuntimeError("trusted config candidate catalog is empty")
        if len({candidate.model for candidate in result}) != len(result):
            raise RuntimeError("trusted config candidate models are not unique")
        return result

    @staticmethod
    def _selected_model(
        candidates: tuple[_Candidate, ...], selected_document: str
    ) -> str:
        for candidate in candidates:
            if candidate.selected_document == selected_document:
                return candidate.model
        raise RuntimeError("committed selector is absent from candidate catalog")

    @staticmethod
    def _candidate(
        candidates: tuple[_Candidate, ...], model: str
    ) -> _Candidate:
        for candidate in candidates:
            if candidate.model == model:
                return candidate
        raise ValueError(f"model is absent from trusted catalog: {model}")

    def _require_draft_locked(self) -> _CommittedDraft:
        if self._draft is None:
            raise RuntimeError("committed config is not loaded")
        return self._draft

    def _mark_dirty_locked(self, *, refresh_values: bool = False) -> None:
        self._dirty = True
        self._refresh_values = self._refresh_values or refresh_values
        self._condition.notify_all()

    def _run_ui_worker(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._dirty or self._state == "closed"
                )
                if self._state == "closed":
                    return
                self._dirty = False
                refresh_values = self._refresh_values
                self._refresh_values = False
                message = self._message
                busy = self._busy
                draft = self._draft
            try:
                self._status.content = self._status_content(message, draft)
                for button in self._buttons:
                    button.disabled = busy
                if refresh_values and draft is not None:
                    self._set_values(draft)
            except Exception:
                pass

    @staticmethod
    def _status_content(message: str, draft: _CommittedDraft | None) -> str:
        revision = (
            "-"
            if draft is None
            else f"runtime `{draft.runtime_revision}`, config `{draft.config_revision}`"
        )
        return "\n".join(
            (
                "### Transactional Config",
                f"- **Revision:** {revision}",
                f"- **Status:** {message}",
            )
        )

    def _set_values(self, draft: _CommittedDraft) -> None:
        directory = draft.root["directory"]
        alignment = draft.alignment["alignment"]
        self._dataset_root.value = str(directory["root_dir_path"])
        self._set_agent_catalog(
            str(directory["root_dir_path"]),
            tuple(str(value) for value in directory["sub_dir_list"]),
            preserve_committed=True,
        )
        self._output_root.value = str(directory["root_save_dir"])
        self._loop_model.options = tuple(
            candidate.model for candidate in draft.loop_candidates
        )
        self._loop_model.value = draft.loop_model
        self._remover_model.options = tuple(
            candidate.model for candidate in draft.remover_candidates
        )
        self._remover_model.value = draft.remover_model
        self._kiss_voxel.value = float(alignment["kiss_voxel_size"])
        self._kiss_quatro.value = bool(alignment["kiss_use_quatro"])
        self._pose_nn.value = float(alignment["pose_nn_distance_threshold"])
        self._inter_loop_spacing.value = float(
            alignment["inter_loop_keyframe_spacing_m"]
        )
