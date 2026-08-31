from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Any, Callable

from .conversion import render_candidate
from .state import PresentationToken


@dataclass(slots=True)
class WorkerResult:
    kind: str
    label: str
    value: Any = None
    error: BaseException | None = None


class CommandDispatcher:
    """Serializes short Runtime mutations without blocking the render thread."""

    def __init__(self, results: "queue.SimpleQueue[WorkerResult]") -> None:
        self._results = results
        self._commands: "queue.Queue[tuple[str, Callable[[], Any]] | None]" = queue.Queue(maxsize=8)
        self._thread = threading.Thread(target=self._run, name="open-lmm-iridescence-command")
        self._started = False

    def start(self) -> None:
        if self._started:
            raise RuntimeError("command dispatcher already started")
        self._started = True
        self._thread.start()

    def submit(self, label: str, command: Callable[[], Any]) -> None:
        if not self._started:
            raise RuntimeError("command dispatcher is not running")
        self._commands.put_nowait((label, command))

    def close(self) -> None:
        if not self._started:
            return
        self._commands.put(None)
        if self._thread is not threading.current_thread():
            self._thread.join()
        self._started = False

    def _run(self) -> None:
        while True:
            item = self._commands.get()
            if item is None:
                return
            label, command = item
            try:
                self._results.put(WorkerResult("command", label, command()))
            except BaseException as error:
                self._results.put(WorkerResult("command", label, error=error))


@dataclass(slots=True)
class VisualizationRequest:
    token: PresentationToken
    color_mode: str
    include_points: bool


class VisualizationWorker:
    """One active query with one newest pending request per agent."""

    def __init__(
        self,
        runtime: Any,
        results: "queue.SimpleQueue[WorkerResult]",
        preview_voxel_size_m: float | None,
    ) -> None:
        self._runtime = runtime
        self._results = results
        self._preview_voxel_size_m = preview_voxel_size_m
        self._condition = threading.Condition()
        self._pending: dict[str, VisualizationRequest] = {}
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="open-lmm-iridescence-visualization")
        self._started = False

    def start(self) -> None:
        self._started = True
        self._thread.start()

    def request(self, request: VisualizationRequest) -> None:
        with self._condition:
            if self._stop:
                return
            previous = self._pending.get(request.token.agent)
            if previous is not None and previous.include_points and not request.include_points:
                return
            self._pending[request.token.agent] = request
            self._condition.notify()

    def remove_agents(self, agents: tuple[str, ...]) -> None:
        with self._condition:
            for agent in agents:
                self._pending.pop(agent, None)

    def close(self) -> None:
        if not self._started:
            return
        with self._condition:
            self._stop = True
            self._pending.clear()
            self._condition.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join()
        self._started = False

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stop or bool(self._pending))
                if self._stop:
                    return
                _, request = self._pending.popitem()
            try:
                snapshot = self._runtime.visualization(
                    request.token.agent,
                    include_points=request.include_points,
                    preview_voxel_size_m=self._preview_voxel_size_m,
                )
                candidate = render_candidate(snapshot, request.token, request.color_mode)
                self._results.put(WorkerResult("visualization", request.token.agent, candidate))
            except BaseException as error:
                self._results.put(WorkerResult("visualization", request.token.agent, request, error))


class ControlPoller:
    def __init__(self, runtime: Any, results: "queue.SimpleQueue[WorkerResult]") -> None:
        self._runtime = runtime
        self._results = results
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._run, name="open-lmm-iridescence-control")
        self._started = False

    def start(self) -> None:
        self._started = True
        self._thread.start()

    def refresh(self) -> None:
        self._wake.set()

    def close(self) -> None:
        if not self._started:
            return
        self._stop.set()
        self._wake.set()
        if self._thread is not threading.current_thread():
            self._thread.join()
        self._started = False

    def _run(self) -> None:
        next_logs = 0.0
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._results.put(WorkerResult("snapshot", "snapshot", self._runtime.snapshot()))
                self._results.put(WorkerResult("alignment", "alignment", self._runtime.alignment_feedback()))
                if started >= next_logs:
                    self._results.put(WorkerResult("logs", "logs", self._runtime.recent_logs()))
                    next_logs = started + 0.25
            except BaseException as error:
                self._results.put(WorkerResult("poll", "poll", error=error))
            remaining = max(0.0, 0.1 - (time.monotonic() - started))
            self._wake.wait(remaining)
            self._wake.clear()


def wait_for_job(job: Any, results: "queue.SimpleQueue[WorkerResult]") -> threading.Thread:
    def wait() -> None:
        try:
            job.wait()
            results.put(WorkerResult("job", str(job.id), job))
        except BaseException as error:
            results.put(WorkerResult("job", str(job.id), job, error))

    thread = threading.Thread(target=wait, name=f"open-lmm-iridescence-job-{job.id}")
    thread.start()
    return thread
