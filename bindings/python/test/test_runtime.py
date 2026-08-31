from __future__ import annotations

import json
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

    def test_recent_logs_are_bounded_and_require_open_runtime(self) -> None:
        runtime = open_lmm.Runtime(1)
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            runtime.recent_logs()
        runtime.open(self.fixture.config, label="logs")
        self.assertIsInstance(runtime.recent_logs(1), tuple)
        for invalid in (0, 513, True, 1.5):
            with self.subTest(invalid=invalid):
                with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
                    runtime.recent_logs(invalid)
        runtime.close()

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

    def test_each_public_node_and_optimize_through(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="nodes")
        agent = runtime.snapshot().pipeline.agents[0]
        runtime.run_node(open_lmm.Node.DATA_LOAD, agent=agent).wait()
        runtime.run_node(open_lmm.Node.LOOP_DETECT, agent=agent).wait()
        runtime.run_node(open_lmm.Node.OPTIMIZE, agent=agent).wait()
        runtime.optimize_through(agent).wait()
        runtime.run_node(open_lmm.Node.MAP_UPDATE, agent=agent).wait()
        runtime.run_node(open_lmm.Node.POSE_SAVE).wait()
        runtime.run_node(open_lmm.Node.FALLBACK_MAP_SAVE).wait()
        self.assertEqual(runtime.snapshot().status, open_lmm.RuntimeStatus.READY)
        runtime.close()

    def test_node_descriptors_and_alignment_feedback_surface(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="alignment-contract")
        descriptors = runtime.node_descriptors()
        self.assertEqual(tuple(item.id for item in descriptors), tuple(open_lmm.Node))
        self.assertTrue(all(item.name for item in descriptors))
        self.assertTrue(all(isinstance(item.scope, open_lmm.ExecutionScope) for item in descriptors))
        runtime.set_alignment_feedback_enabled(True)
        self.assertIsNone(runtime.alignment_feedback())
        runtime.set_alignment_feedback_enabled(False)
        runtime.close()

    def test_node_and_optimize_arguments_fail_closed(self) -> None:
        runtime = open_lmm.Runtime(1)
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            runtime.run_node(99)
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            runtime.optimize_through("")
        runtime.close()

    def test_committed_config_query_and_root_replacement(self) -> None:
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="config-documents")
        documents = runtime.config_documents()
        self.assertEqual(
            tuple(document.domain for document in documents.documents),
            tuple(open_lmm.ConfigDomain),
        )
        self.assertTrue(
            all(document.canonical_json for document in documents.documents)
        )
        self.assertIsNone(documents.documents[0].selected_document)
        self.assertTrue(
            all(
                document.selected_document
                for document in documents.documents[1:]
            )
        )

        root = json.loads(documents.documents[0].canonical_json)
        replacement_output = self.fixture.root / "replacement-output"
        root["directory"]["root_save_dir"] = str(replacement_output)
        expected = open_lmm.Revision(
            documents.runtime_revision, documents.config_revision
        )
        receipt = runtime.replace_root_config(
            json.dumps(root), expected=expected
        )
        self.assertEqual(receipt.previous_runtime_revision, expected.runtime_revision)
        self.assertEqual(receipt.previous_config_revision, expected.config_revision)
        self.assertEqual(receipt.runtime_revision, expected.runtime_revision + 1)
        self.assertEqual(receipt.config_revision, expected.config_revision + 1)
        committed = runtime.config_documents()
        committed_root = json.loads(committed.documents[0].canonical_json)
        self.assertEqual(
            committed_root["directory"]["root_save_dir"],
            str(replacement_output),
        )
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
            runtime.replace_root_config(json.dumps(root), expected=expected)
        runtime.close()

    def test_config_candidate_catalog_is_revision_bound_and_relative(self) -> None:
        (self.fixture.config / "core" / "legacy-std.json").write_text(
            json.dumps(
                {"loop_detector": {"loop_detector_type": "std"}}
            ),
            encoding="utf-8",
        )
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="config-candidates")
        documents = runtime.config_documents()
        catalog = runtime.config_candidates()
        self.assertEqual(catalog.runtime_revision, documents.runtime_revision)
        self.assertEqual(catalog.config_revision, documents.config_revision)
        self.assertEqual(
            {(candidate.domain, candidate.model) for candidate in catalog.candidates},
            {
                (open_lmm.ConfigDomain.LOOP_DETECTOR, "scan_context"),
                (open_lmm.ConfigDomain.DYNAMIC_REMOVER, "free_dom"),
            },
        )
        for candidate in catalog.candidates:
            self.assertFalse(candidate.selected_document.startswith("/"))
            self.assertTrue(candidate.canonical_json)
        runtime.close()

    def test_catalog_candidates_can_switch_loop_and_remover_transactionally(self) -> None:
        (self.fixture.config / "core" / "solid.json").write_text(
            json.dumps(
                {
                    "loop_detector": {
                        "loop_detector_type": "kdtree",
                        "model": "solid",
                    }
                }
            ),
            encoding="utf-8",
        )
        (self.fixture.config / "core" / "otd.json").write_text(
            json.dumps(
                {
                    "dynamic_remover": {
                        "dynamic_remover_type": "online",
                        "model": "otd",
                    }
                }
            ),
            encoding="utf-8",
        )
        runtime = open_lmm.Runtime(1)
        runtime.open(self.fixture.config, label="candidate-switch")
        catalog = runtime.config_candidates()
        solid = next(candidate for candidate in catalog.candidates if candidate.model == "solid")
        runtime.apply_config(
            solid.domain,
            solid.canonical_json,
            selected_document=solid.selected_document,
            expected=open_lmm.Revision(
                catalog.runtime_revision, catalog.config_revision
            ),
        )
        catalog = runtime.config_candidates()
        otd = next(candidate for candidate in catalog.candidates if candidate.model == "otd")
        runtime.apply_config(
            otd.domain,
            otd.canonical_json,
            selected_document=otd.selected_document,
            expected=open_lmm.Revision(
                catalog.runtime_revision, catalog.config_revision
            ),
        )
        selected = {
            document.domain: document.selected_document
            for document in runtime.config_documents().documents
        }
        self.assertEqual(
            selected[open_lmm.ConfigDomain.LOOP_DETECTOR], "core/solid.json"
        )
        self.assertEqual(
            selected[open_lmm.ConfigDomain.DYNAMIC_REMOVER], "core/otd.json"
        )
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
