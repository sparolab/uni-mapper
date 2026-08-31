from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import sys
import threading
from typing import TYPE_CHECKING, Any

from .conversion import (
    VisualizationCandidate,
    agent_color,
    scene_key,
    visualization_candidate,
)

if TYPE_CHECKING:
    import open_lmm
    import viser


_REFRESH_EVENTS = {
    "ARTIFACT_COMMITTED",
    "ARTIFACT_INVALIDATED",
    "STAGE_COMPLETED",
    "JOB_COMPLETED",
}


@dataclass(slots=True)
class _SceneHandles:
    points: Any | None = None
    trajectory: Any | None = None


def _positive_finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


def _event_name(event: object) -> str:
    value = getattr(event, "type", None)
    name = getattr(value, "name", value)
    return str(name).upper()


class ViserAdapter:
    def __init__(
        self,
        runtime: "open_lmm.Runtime",
        server: "viser.ViserServer",
        *,
        preview_voxel_size_m: float | None = None,
        point_size: float = 0.03,
        trajectory_thickness: float = 0.03,
    ) -> None:
        if runtime is None or server is None:
            raise ValueError("runtime and server are required")
        self._runtime = runtime
        self._server = server
        self._preview_voxel_size_m = (
            None
            if preview_voxel_size_m is None
            else _positive_finite(preview_voxel_size_m, "preview_voxel_size_m")
        )
        self._point_size = _positive_finite(point_size, "point_size")
        self._trajectory_thickness = _positive_finite(
            trajectory_thickness, "trajectory_thickness"
        )

        self._condition = threading.Condition()
        self._state = "new"
        self._closing = False
        self._stop = False
        self._catalog_refresh = False
        self._generation = 0
        self._catalog: set[str] = set()
        self._colors: dict[str, tuple[int, int, int]] = {}
        self._requested: dict[str, int] = {}
        self._pending: dict[str, int] = {}
        self._displayed: dict[str, int] = {}
        self._handles: dict[str, _SceneHandles] = {}
        self._subscription: Any | None = None
        self._worker: threading.Thread | None = None
        self._commit_observers: list[Callable[[str, int], None]] = []
        self._presentation_visible = True

    def add_commit_observer(self, observer: Callable[[str, int], None]) -> None:
        if not callable(observer):
            raise ValueError("commit observer must be callable")
        with self._condition:
            if self._state == "closed":
                raise RuntimeError("ViserAdapter is closed")
            self._commit_observers.append(observer)

    def remove_commit_observer(self, observer: Callable[[str, int], None]) -> None:
        with self._condition:
            self._commit_observers = [
                candidate
                for candidate in self._commit_observers
                if candidate != observer
            ]

    def set_presentation_visible(self, visible: bool) -> None:
        """Show or hide the committed presentation without discarding it.

        Alignment review is a derived, temporary presentation.  It may hide
        the committed map to avoid presenting two indistinguishable owners,
        but the last valid committed handles stay alive and are restored when
        review ends.
        """
        requested = bool(visible)
        with self._condition:
            if self._state == "closed" or requested == self._presentation_visible:
                return
            handles = tuple(self._handles.values())
            with self._server.atomic():
                for owned in handles:
                    if owned.points is not None:
                        owned.points.visible = requested
                    if owned.trajectory is not None:
                        owned.trajectory.visible = requested
            self._presentation_visible = requested

    def start(self) -> None:
        with self._condition:
            if self._state != "new":
                raise RuntimeError("ViserAdapter can only be started once")
            self._state = "started"
            worker = threading.Thread(
                target=self._run_worker,
                name="open-lmm-viser-refresh",
                daemon=False,
            )
            self._worker = worker
        try:
            worker.start()
            # Subscribe before the initial query so a commit between setup and
            # snapshot cannot disappear. Generation coalescing removes duplicates.
            self._subscription = self._runtime.subscribe_events(self._on_event)
            snapshot = self._runtime.snapshot()
            self._install_catalog(snapshot)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        with self._condition:
            if self._state == "closed":
                return
            self._state = "closed"
            self._closing = True
        subscription, self._subscription = self._subscription, None
        if subscription is not None:
            try:
                subscription.close()
            except Exception as error:
                self._diagnostic("subscription close", error)
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        worker, self._worker = self._worker, None
        if worker is not None and worker is not threading.current_thread():
            worker.join()
        self._remove_all_scenes()

    def __enter__(self) -> "ViserAdapter":
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        del exc_type, exc, traceback
        self.close()

    def _on_event(self, event: object) -> None:
        if _event_name(event) not in _REFRESH_EVENTS:
            return
        affected = tuple(getattr(event, "affected_agents", ()) or ())
        single = getattr(event, "agent", None)
        targets = affected or ((single,) if single else ())
        with self._condition:
            if self._closing:
                return
            if not targets:
                self._catalog_refresh = True
            else:
                for agent in targets:
                    if agent in self._catalog:
                        self._queue_agent_locked(agent)
                    else:
                        self._catalog_refresh = True
            self._condition.notify_all()

    def _queue_agent_locked(self, agent: str) -> None:
        self._generation += 1
        self._requested[agent] = self._generation
        self._pending[agent] = self._generation

    def _install_catalog(self, snapshot: object) -> None:
        pipeline = getattr(snapshot, "pipeline", None)
        agents = tuple(getattr(pipeline, "agents", ()) or ())
        if any(not isinstance(agent, str) or not agent for agent in agents):
            raise ValueError("runtime snapshot contains an invalid agent catalog")
        authoritative = set(agents)
        colors = {agent: agent_color(index) for index, agent in enumerate(agents)}
        with self._condition:
            if self._closing:
                return
            removed = self._catalog - authoritative
            self._catalog = authoritative
            self._colors = colors
            for agent in removed:
                self._pending.pop(agent, None)
                self._requested.pop(agent, None)
                self._displayed.pop(agent, None)
            for agent in agents:
                self._queue_agent_locked(agent)
            self._condition.notify_all()
        for agent in removed:
            self._remove_agent_scene(agent)

    def _run_worker(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._stop or self._catalog_refresh or bool(self._pending)
                )
                if self._stop:
                    return
                if self._catalog_refresh:
                    self._catalog_refresh = False
                    action: tuple[str, str | None, int | None] = (
                        "catalog",
                        None,
                        None,
                    )
                else:
                    agent, generation = self._pending.popitem()
                    action = ("agent", agent, generation)
            if action[0] == "catalog":
                try:
                    self._install_catalog(self._runtime.snapshot())
                except Exception as error:
                    self._diagnostic("catalog refresh", error)
                continue
            agent = action[1]
            generation = action[2]
            assert agent is not None and generation is not None
            self._refresh_agent(agent, generation)

    def _refresh_agent(self, agent: str, generation: int) -> None:
        queried_revision: int | None = None
        try:
            snapshot = self._runtime.visualization(
                agent, preview_voxel_size_m=self._preview_voxel_size_m
            )
            queried_revision = getattr(snapshot, "revision", None)
            with self._condition:
                color = self._colors.get(agent)
            if color is None:
                return
            candidate = visualization_candidate(snapshot, color)
        except Exception as error:
            self._diagnostic(
                f"agent={agent} generation={generation} "
                f"revision={queried_revision} query/conversion",
                error,
            )
            return

        with self._condition:
            if (
                self._closing
                or agent not in self._catalog
                or self._requested.get(agent) != generation
                or candidate.revision < self._displayed.get(agent, 0)
            ):
                return
            try:
                handles = self._publish_candidate(candidate)
            except Exception as error:
                self._diagnostic(
                    f"agent={agent} generation={generation} "
                    f"revision={candidate.revision} scene commit",
                    error,
                )
                return
            self._handles[agent] = handles
            self._displayed[agent] = candidate.revision
            observers = tuple(self._commit_observers)
        for observer in observers:
            try:
                observer(agent, candidate.revision)
            except Exception as error:
                self._diagnostic(f"agent={agent} commit observer", error)

    def _publish_candidate(self, candidate: VisualizationCandidate) -> _SceneHandles:
        key = scene_key(candidate.agent)
        previous = self._handles.get(candidate.agent, _SceneHandles())
        points_handle = previous.points
        trajectory_handle = previous.trajectory
        with self._server.atomic():
            if candidate.point_cloud is not None:
                points_handle = self._server.scene.add_point_cloud(
                    f"/open_lmm/agents/{key}/points",
                    points=candidate.point_cloud.points,
                    colors=candidate.point_cloud.colors,
                    point_size=self._point_size,
                    precision="float32",
                )
                points_handle.visible = self._presentation_visible
            if candidate.trajectory is None:
                if trajectory_handle is not None:
                    trajectory_handle.remove()
                trajectory_handle = None
            else:
                trajectory_handle = self._server.scene.add_line_segments(
                    f"/open_lmm/agents/{key}/trajectory",
                    points=candidate.trajectory,
                    colors=candidate.color,
                    thickness=self._trajectory_thickness,
                    thickness_units="world",
                )
                trajectory_handle.visible = self._presentation_visible
        return _SceneHandles(points_handle, trajectory_handle)

    def _remove_agent_scene(self, agent: str) -> None:
        handles = self._handles.get(agent)
        if handles is None:
            return
        try:
            with self._server.atomic():
                if handles.points is not None:
                    handles.points.remove()
                if handles.trajectory is not None:
                    handles.trajectory.remove()
        except Exception as error:
            self._diagnostic(f"agent={agent} scene removal", error)
            return
        self._handles.pop(agent, None)

    def _remove_all_scenes(self) -> None:
        for agent in tuple(self._handles):
            self._remove_agent_scene(agent)

    @staticmethod
    def _diagnostic(context: str, error: BaseException) -> None:
        print(
            f"open-lmm-viser: refresh failure: {context}: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
