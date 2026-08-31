from __future__ import annotations

import threading
import unittest

import open_lmm

from python_test_fixture import RuntimeFixture


class RuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = RuntimeFixture("runtime")

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_open_run_wait_snapshot_and_close(self) -> None:
        runtime = open_lmm.Runtime(1)
        self.assertFalse(runtime.is_open())
        runtime.open(self.fixture.config, label="python-runtime")
        self.assertTrue(runtime.is_open())

        job = runtime.run_stage(open_lmm.Stage.DATA_LOAD)
        self.assertGreater(job.id, 0)
        job.wait()
        snapshot = runtime.snapshot()
        self.assertEqual(snapshot.label, "python-runtime")
        self.assertEqual(snapshot.status, open_lmm.RuntimeStatus.READY)
        self.assertEqual(snapshot.pipeline.agents, ("agent1",))
        self.assertGreater(snapshot.pipeline.runtime_revision, 0)
        self.assertTrue(snapshot.pipeline.recent_events)

        runtime.close()
        runtime.close()
        self.assertFalse(runtime.is_open())

    def test_foreign_job_is_rejected(self) -> None:
        first = open_lmm.Runtime(1)
        second = open_lmm.Runtime(1)
        first.open(self.fixture.config, label="first")
        other_fixture = RuntimeFixture("foreign")
        try:
            second.open(other_fixture.config, label="second")
            job = first.run_stage(open_lmm.Stage.DATA_LOAD)
            with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
                second.cancel(job)
            job.wait()
        finally:
            first.close()
            second.close()
            other_fixture.cleanup()

    def test_run_all_and_each_public_stage(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="stages")
        for stage in open_lmm.Stage:
            runtime.run_stage(stage).wait()
        runtime.run_all().wait()
        snapshot = runtime.snapshot()
        self.assertEqual(snapshot.status, open_lmm.RuntimeStatus.READY)
        self.assertTrue(snapshot.output_directory.exists())
        runtime.close()

    def test_lifecycle_double_open_reopen_and_terminal_success(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="lifecycle")
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            runtime.open(self.fixture.config, label="duplicate")
        job = runtime.run_stage(open_lmm.Stage.DATA_LOAD)
        job.wait()
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            job.cancel()
        job.wait()
        runtime.close()
        runtime.close()
        runtime.open(self.fixture.config, label="reopened")
        self.assertTrue(runtime.is_open())
        runtime.close()

    def test_cancel_before_commit_is_reported_as_cancelled(self) -> None:
        cancellation_fixture = RuntimeFixture("cancel", scan_count=1000)
        runtime = open_lmm.Runtime(1)
        try:
            runtime.open(cancellation_fixture.config, label="cancel")
            job = runtime.run_stage(open_lmm.Stage.DATA_LOAD)
            job.cancel()
            with self.assertRaises(open_lmm.OpenLMMCancelledError):
                job.wait()
            self.assertEqual(
                runtime.snapshot().status,
                open_lmm.RuntimeStatus.READY,
            )
        finally:
            runtime.close()
            cancellation_fixture.cleanup()

    def test_close_reject_mode_preserves_active_runtime(self) -> None:
        active_fixture = RuntimeFixture("active-close", scan_count=1000)
        runtime = open_lmm.Runtime(1)
        subscription = None
        try:
            runtime.open(active_fixture.config, label="active-close")
            started = threading.Event()

            def observe(event: open_lmm.ExecutionEvent) -> None:
                if event.type == open_lmm.EventType.JOB_STARTED:
                    started.set()

            subscription = runtime.subscribe_events(observe)
            job = runtime.run_stage(open_lmm.Stage.DATA_LOAD)
            self.assertTrue(started.wait(1.0))
            with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
                runtime.close(cancel_running=False)
            self.assertTrue(runtime.is_open())
            job.cancel()
            with self.assertRaises(open_lmm.OpenLMMCancelledError):
                job.wait()
        finally:
            if subscription is not None:
                subscription.close()
            runtime.close()
            active_fixture.cleanup()


if __name__ == "__main__":
    unittest.main()
