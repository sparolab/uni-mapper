from __future__ import annotations

import sys
import threading
import unittest

import open_lmm

from python_test_fixture import RuntimeFixture


class CallbackTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = RuntimeFixture("callbacks")

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_delivery_reentry_and_unsubscribe_drain(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="callbacks")
        terminal = threading.Event()
        events: list[open_lmm.ExecutionEvent] = []

        def callback(event: open_lmm.ExecutionEvent) -> None:
            events.append(event)
            if event.type in {
                open_lmm.EventType.JOB_COMPLETED,
                open_lmm.EventType.JOB_CANCELLED,
            }:
                self.assertGreater(runtime.snapshot().pipeline.runtime_revision, 0)
                terminal.set()

        subscription = runtime.subscribe_events(callback)
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        self.assertTrue(terminal.wait(1.0))
        self.assertEqual(
            sum(event.type == open_lmm.EventType.JOB_COMPLETED for event in events),
            1,
        )
        self.assertEqual(
            [event.sequence for event in events],
            sorted(event.sequence for event in events),
        )
        subscription.close()
        count = len(events)
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        self.assertEqual(len(events), count)
        runtime.close()

    def test_callback_exception_is_unraisable_and_subscription_survives(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="callback-error")
        first_observed = threading.Event()
        second_observed = threading.Event()
        terminal_count = 0
        unraisable = []
        previous_hook = sys.unraisablehook
        sys.unraisablehook = unraisable.append
        try:
            def callback(event: open_lmm.ExecutionEvent) -> None:
                nonlocal terminal_count
                if event.type == open_lmm.EventType.JOB_COMPLETED:
                    terminal_count += 1
                    if terminal_count == 1:
                        first_observed.set()
                    else:
                        second_observed.set()
                    raise RuntimeError("intentional callback failure")

            subscription = runtime.subscribe_events(callback)
            runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
            self.assertTrue(first_observed.wait(1.0))
            self.assertEqual(len(unraisable), 1)
            runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
            self.assertTrue(second_observed.wait(1.0))
            self.assertEqual(len(unraisable), 2)
            subscription.close()
        finally:
            sys.unraisablehook = previous_hook
            runtime.close()

    def test_subscription_can_close_itself_from_callback(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="self-unsubscribe")
        completed = threading.Event()
        terminal_count = 0
        subscription: open_lmm.Subscription | None = None

        def callback(event: open_lmm.ExecutionEvent) -> None:
            nonlocal terminal_count
            if event.type != open_lmm.EventType.JOB_COMPLETED:
                return
            terminal_count += 1
            assert subscription is not None
            subscription.close()
            completed.set()

        subscription = runtime.subscribe_events(callback)
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        self.assertTrue(completed.wait(1.0))
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        self.assertEqual(terminal_count, 1)
        runtime.close()

    def test_runtime_close_from_callback_is_rejected_without_deadlock(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="callback-close")
        attempted = threading.Event()
        failures: list[open_lmm.OpenLMMInvalidArgumentError] = []

        def callback(event: open_lmm.ExecutionEvent) -> None:
            if event.type != open_lmm.EventType.JOB_STARTED:
                return
            try:
                runtime.close()
            except open_lmm.OpenLMMInvalidArgumentError as error:
                failures.append(error)
            finally:
                attempted.set()

        subscription = runtime.subscribe_events(callback)
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        self.assertTrue(attempted.wait(1.0))
        self.assertEqual(len(failures), 1)
        self.assertTrue(runtime.is_open())
        subscription.close()
        runtime.close()


if __name__ == "__main__":
    unittest.main()
