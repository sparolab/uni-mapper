from __future__ import annotations

import gc
import subprocess
import sys
import threading
import unittest
import weakref

import open_lmm

from python_test_fixture import RuntimeFixture


class LifetimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = RuntimeFixture("lifetime", scan_count=300)

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_wait_releases_gil(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="gil")
        job = runtime.run_stage(open_lmm.Stage.DATA_LOAD)
        begin = threading.Event()
        progressed = threading.Event()

        def python_worker() -> None:
            begin.wait()
            progressed.set()

        worker = threading.Thread(target=python_worker)
        worker.start()
        begin.set()
        job.wait()
        worker.join()
        self.assertTrue(progressed.is_set())
        runtime.close()

    def test_job_does_not_keep_runtime_alive(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="weak-job")
        job = runtime.run_stage(open_lmm.Stage.DATA_LOAD)
        job.wait()
        reference = weakref.ref(runtime)
        runtime.close()
        del runtime
        gc.collect()
        self.assertIsNone(reference())
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            job.wait()

    def test_subscription_close_breaks_callback_reference_cycle(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="callback-cycle")
        holder: dict[str, open_lmm.Subscription] = {}

        def callback(event: open_lmm.ExecutionEvent) -> None:
            if event.type == open_lmm.EventType.JOB_COMPLETED:
                holder["subscription"].close()

        callback_reference = weakref.ref(callback)
        subscription = runtime.subscribe_events(callback)
        subscription_reference = weakref.ref(subscription)
        holder["subscription"] = subscription
        subscription.close()
        del callback
        del subscription
        del holder
        gc.collect()
        self.assertIsNone(callback_reference())
        self.assertIsNone(subscription_reference())
        runtime.close()

    def test_process_exit_drains_active_job_and_subscription(self) -> None:
        script = "\n".join(
            (
                "import open_lmm",
                "runtime = open_lmm.Runtime(1)",
                f"runtime.open({str(self.fixture.config)!r}, label='exit')",
                "subscription = runtime.subscribe_events(lambda event: None)",
                "runtime.run_stage(open_lmm.Stage.DATA_LOAD)",
            )
        )
        result = subprocess.run(
            [sys.executable, "-c", script],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
