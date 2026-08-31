from __future__ import annotations

import threading
import time
import unittest
from enum import Enum, auto
from types import SimpleNamespace

from open_lmm_viser.control import RuntimeControlPanel
from viser_test_support import FakeSubscription, wait_until


class Named(Enum):
    READY = auto()
    RUNNING = auto()
    DATA_LOAD = auto()
    PROGRESS_UPDATED = auto()
    SUCCEEDED = auto()


class FakeHandle:
    def __init__(self, label: str, **kwargs) -> None:
        self.label = label
        self.content = kwargs.get("content", "")
        self.disabled = bool(kwargs.get("disabled", False))
        self.removed = False
        self.callback = None
        self.options = tuple(kwargs.get("options", ()))
        self.value = kwargs.get(
            "initial_value", self.options[0] if self.options else ""
        )

    def on_click(self, callback):
        self.callback = callback
        return callback

    def click(self) -> None:
        assert self.callback is not None
        self.callback(SimpleNamespace(target=self))

    def remove(self) -> None:
        self.removed = True


class FakeGui:
    def __init__(self) -> None:
        self.handles: list[FakeHandle] = []

    def add_markdown(self, content: str) -> FakeHandle:
        handle = FakeHandle("markdown", content=content)
        self.handles.append(handle)
        return handle

    def add_button(self, label: str, **kwargs) -> FakeHandle:
        handle = FakeHandle(label, **kwargs)
        self.handles.append(handle)
        return handle

    def add_dropdown(self, label: str, **kwargs) -> FakeHandle:
        handle = FakeHandle(label, **kwargs)
        self.handles.append(handle)
        return handle

    def button(self, label: str) -> FakeHandle:
        return next(handle for handle in self.handles if handle.label == label)


class FakeJob:
    def __init__(self, identifier: int, error: BaseException | None = None) -> None:
        self.id = identifier
        self.error = error
        self.entered = threading.Event()
        self.release = threading.Event()
        self.cancel_count = 0

    def wait(self) -> None:
        self.entered.set()
        self.release.wait(2.0)
        if self.error is not None:
            raise self.error

    def cancel(self) -> None:
        self.cancel_count += 1
        self.release.set()


class FakeRuntime:
    def __init__(self) -> None:
        self.callback = None
        self.subscription = FakeSubscription()
        self.jobs: list[FakeJob] = []
        self.calls: list[tuple[str, object | None]] = []
        self.runtime_revision = 3
        self.config_revision = 2

    def queue_job(self, error: BaseException | None = None) -> FakeJob:
        job = FakeJob(len(self.jobs) + 1, error)
        self.jobs.append(job)
        return job

    def run_all(self) -> FakeJob:
        self.calls.append(("all", None))
        return self.jobs[-1]

    def run_stage(self, stage: object) -> FakeJob:
        self.calls.append(("stage", stage))
        return self.jobs[-1]

    def run_node(self, node: object, *, agent: str | None = None) -> FakeJob:
        self.calls.append(("node", (node, agent)))
        return self.jobs[-1]

    def optimize_through(self, agent: str) -> FakeJob:
        self.calls.append(("optimize_through", agent))
        return self.jobs[-1]

    def subscribe_events(self, callback):
        self.callback = callback
        return self.subscription

    def snapshot(self):
        state = Named.RUNNING if self.jobs and not self.jobs[-1].release.is_set() else Named.SUCCEEDED
        job = None
        if self.jobs:
            job = SimpleNamespace(
                id=self.jobs[-1].id,
                state=state,
                active_stage=Named.DATA_LOAD,
                message="snapshot",
            )
        return SimpleNamespace(
            status=Named.READY,
            pipeline=SimpleNamespace(
                runtime_revision=self.runtime_revision,
                config_revision=self.config_revision,
                agents=("agent1", "agent2"),
                job=job,
            ),
        )

    def emit(self, *, sequence: int, job_id: int = 1, current: int = 0, total: int = 0) -> None:
        assert self.callback is not None
        self.callback(
            SimpleNamespace(
                job_id=job_id,
                sequence=sequence,
                type=Named.PROGRESS_UPDATED,
                stage=Named.DATA_LOAD,
                message=f"progress-{sequence}",
                progress_current=current,
                progress_total=total,
                algorithm_progress=None,
            )
        )


class ControlPanelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = FakeRuntime()
        self.server = SimpleNamespace(gui=FakeGui())
        self.panel = RuntimeControlPanel(
            self.runtime,
            self.server,
            (("Data Load", Named.DATA_LOAD), ("Save", "save")),
            (
                ("Data Load", "data_load", True),
                ("Pose Save", "pose_save", False),
            ),
        )
        self.panel.start()

    def tearDown(self) -> None:
        self.panel.close()

    def status(self) -> str:
        return self.server.gui.handles[0].content

    def test_run_all_progress_and_terminal_state(self) -> None:
        job = self.runtime.queue_job()
        generation = self.panel.submit_run_all()
        self.assertTrue(job.entered.wait(1.0))
        wait_until(lambda: self.server.gui.button("Run All").disabled)
        self.runtime.emit(sequence=1, current=2, total=4)
        wait_until(lambda: "2/4 (50.0%)" in self.status())
        self.runtime.emit(sequence=1, current=4, total=4)
        time.sleep(0.02)
        self.assertIn("2/4 (50.0%)", self.status())
        job.release.set()
        self.panel.wait(generation)
        wait_until(lambda: not self.server.gui.button("Run All").disabled)
        self.assertEqual(self.runtime.calls, [("all", None)])
        self.assertIn("runtime `3`, config `2`", self.status())

    def test_stage_button_uses_exact_stage_and_rejects_overlap(self) -> None:
        job = self.runtime.queue_job()
        self.server.gui.button("Stage: Data Load").click()
        self.assertTrue(job.entered.wait(1.0))
        self.server.gui.button("Stage: Save").click()
        wait_until(lambda: "another OpenLMM job is active" in self.status())
        self.assertEqual(self.runtime.calls, [("stage", Named.DATA_LOAD)])
        job.release.set()

    def test_cancel_targets_only_active_job(self) -> None:
        job = self.runtime.queue_job(RuntimeError("cancelled"))
        generation = self.panel.submit_run_all()
        self.assertTrue(job.entered.wait(1.0))
        wait_until(lambda: not self.server.gui.button("Cancel Active Job").disabled)
        self.server.gui.button("Cancel Active Job").click()
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            self.panel.wait(generation)
        self.assertEqual(job.cancel_count, 1)

    def test_agent_node_and_optimize_use_authoritative_selection(self) -> None:
        first = self.runtime.queue_job()
        wait_until(lambda: not self.server.gui.button("Agent").disabled)
        self.server.gui.button("Agent").value = "agent2"
        self.server.gui.button("Node: Data Load").click()
        self.assertTrue(first.entered.wait(1.0))
        first.release.set()
        self.panel.wait(1)
        wait_until(lambda: not self.server.gui.button("Optimize Through").disabled)

        second = self.runtime.queue_job()
        self.server.gui.button("Optimize Through").click()
        self.assertTrue(second.entered.wait(1.0))
        second.release.set()
        self.panel.wait(2)
        self.assertEqual(
            self.runtime.calls,
            [
                ("node", ("data_load", "agent2")),
                ("optimize_through", "agent2"),
            ],
        )

    def test_runtime_node_does_not_forward_agent(self) -> None:
        job = self.runtime.queue_job()
        self.server.gui.button("Agent").value = "agent2"
        self.server.gui.button("Node: Pose Save").click()
        self.assertTrue(job.entered.wait(1.0))
        job.release.set()
        self.panel.wait(1)
        self.assertEqual(self.runtime.calls, [("node", ("pose_save", None))])

    def test_wrong_job_event_is_ignored(self) -> None:
        job = self.runtime.queue_job()
        self.panel.submit_run_all()
        self.assertTrue(job.entered.wait(1.0))
        self.runtime.emit(sequence=1, job_id=999, current=9, total=9)
        time.sleep(0.02)
        self.assertNotIn("9/9", self.status())
        job.release.set()

    def test_close_unsubscribes_cancels_joins_and_removes_gui(self) -> None:
        job = self.runtime.queue_job()
        self.panel.submit_run_all()
        self.assertTrue(job.entered.wait(1.0))
        handles = tuple(self.server.gui.handles)
        self.panel.close()
        self.assertTrue(self.runtime.subscription.closed)
        self.assertEqual(job.cancel_count, 1)
        self.assertTrue(all(handle.removed for handle in handles))


if __name__ == "__main__":
    unittest.main()
