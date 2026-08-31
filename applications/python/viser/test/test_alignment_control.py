from __future__ import annotations

import contextlib
import asyncio
import unittest
from types import SimpleNamespace

import numpy as np
import open_lmm

from open_lmm_viser.alignment_control import AlignmentControlPanel


class FakeRuntime:
    def __init__(self, job) -> None:
        self.job = job
        self.responses = []

    def snapshot(self):
        return SimpleNamespace(pipeline=SimpleNamespace(job=SimpleNamespace(id=self.job.id)))

    def respond_to_alignment(self, job, response) -> None:
        self.responses.append((job, response))


class FakeCommitSource:
    def __init__(self) -> None:
        self.visibility = []
        self.observers = []

    def add_commit_observer(self, observer) -> None:
        self.observers.append(observer)

    def remove_commit_observer(self, observer) -> None:
        self.observers.remove(observer)

    def set_presentation_visible(self, visible: bool) -> None:
        self.visibility.append(visible)


class FakeHandle:
    def __init__(self, **values) -> None:
        self.removed = False
        self.label = values.get("label", "")
        self.content = values.get("content", "")
        self.disabled = values.get("disabled", False)
        self.wxyz = values.get("wxyz", (1.0, 0.0, 0.0, 0.0))
        self.position = values.get("position", (0.0, 0.0, 0.0))
        self.visible = values.get("visible", True)
        self.callback = None
        self.update_callback = None

    def on_click(self, callback) -> None:
        self.callback = callback

    def click(self) -> None:
        self.callback(SimpleNamespace(target=self))

    def on_update(self, callback) -> None:
        self.update_callback = callback

    def update(self) -> None:
        result = self.update_callback(SimpleNamespace(target=self, phase="update"))
        if result is not None:
            asyncio.run(result)

    def remove(self) -> None:
        self.removed = True


class FakeScene:
    def __init__(self) -> None:
        self.fail = False
        self.point_calls = []
        self.frame_calls = []
        self.transform_calls = []

    def add_point_cloud(self, name, **values):
        if self.fail:
            raise RuntimeError("scene failure")
        self.point_calls.append((name, values))
        return FakeHandle(**values)

    def add_transform_controls(self, _name, **values):
        self.transform_calls.append((_name, values))
        return FakeHandle(**values)

    def add_frame(self, _name, **values):
        self.frame_calls.append((_name, values))
        return FakeHandle(**values)

    def add_line_segments(self, _name, **values):
        return FakeHandle(**values)


class FakeGui:
    def __init__(self) -> None:
        self.handles = []

    def add_markdown(self, content: str):
        handle = FakeHandle(content=content)
        self.handles.append(handle)
        return handle

    def add_button(self, label: str, **_values):
        handle = FakeHandle(label=label)
        self.handles.append(handle)
        return handle

    def get(self, label: str):
        return next(handle for handle in self.handles if handle.label == label)


class FakeServer:
    def __init__(self) -> None:
        self.scene = FakeScene()
        self.gui = FakeGui()

    @contextlib.contextmanager
    def atomic(self):
        yield


def feedback(request: int = 1):
    return SimpleNamespace(
        proposal=SimpleNamespace(
            request_id=request,
            target_agent="agent1",
            source_agent="agent2",
            target_T_source=np.eye(4),
            method=SimpleNamespace(name="PENDING"),
            metrics=SimpleNamespace(
                rotation_inliers=0,
                final_inliers=0,
                consensus_size=0,
            ),
        ),
        session_revision=1,
        review_state=SimpleNamespace(name="ACTIVE"),
        attempt_status=SimpleNamespace(
            state=SimpleNamespace(name="IDLE"), message=""
        ),
        terminal_message="",
        target_points=np.zeros((1, 3), np.float32),
        source_points=np.ones((1, 3), np.float32),
        diagnostics=SimpleNamespace(
            target_trajectory=np.empty((0, 3), np.float32),
            source_trajectory=np.empty((0, 3), np.float32),
            descriptor_loops=(),
        ),
    )


class AlignmentControlTests(unittest.TestCase):
    def test_manual_button_enters_edit_mode_before_apply_submits(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        runtime.snapshot = lambda: SimpleNamespace(
            pipeline=SimpleNamespace(
                job=SimpleNamespace(id=41), agents=("agent1", "agent2")
            )
        )
        server = FakeServer()
        panel = AlignmentControlPanel(runtime, server, lambda: job)
        panel._create_gui()
        self.assertEqual(
            [handle.label for handle in server.gui.handles if handle.label][:6],
            [
                "Accept",
                "Try KISS",
                "Try Descriptor",
                "Manual Align",
                "Exclude Agent",
                "Cancel Alignment",
            ],
        )
        review = feedback()
        panel._publish(review)
        panel._feedback = review
        panel._update_gui()

        manual = server.gui.get("Manual Align")
        apply = server.gui.get("Apply Manual Transform")
        self.assertTrue(manual.visible)
        self.assertFalse(manual.disabled)
        self.assertFalse(apply.visible)

        manual.click()
        self.assertTrue(panel._manual_controls.visible)
        self.assertFalse(manual.visible)
        self.assertTrue(apply.visible)
        self.assertEqual(runtime.responses, [])

        apply.click()
        self.assertFalse(panel._manual_controls.visible)
        self.assertEqual(len(runtime.responses), 1)

    def test_manual_mode_reveals_gizmo_then_submits_edited_transform(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        runtime.snapshot = lambda: SimpleNamespace(
            pipeline=SimpleNamespace(
                job=SimpleNamespace(id=41), agents=("agent1", "agent2")
            )
        )
        server = FakeServer()
        panel = AlignmentControlPanel(runtime, server, lambda: job)
        review = feedback()
        panel._publish(review)
        panel._feedback = review

        controls = panel._manual_controls
        self.assertFalse(controls.visible)
        _, creation = server.scene.transform_calls[0]
        self.assertFalse(creation["depth_test"])
        self.assertGreaterEqual(creation["scale"], 0.5)

        panel._enter_manual_mode()
        self.assertTrue(controls.visible)
        self.assertEqual(runtime.responses, [])
        controls.position = (1.0, 2.0, 3.0)
        controls.update()
        self.assertEqual(panel._manual_frame.position, (1.0, 2.0, 3.0))
        panel.respond(open_lmm.AlignmentDecision.MANUAL, manual=True)

        self.assertFalse(controls.visible)
        self.assertFalse(panel._manual_mode)
        response = runtime.responses[0][1]
        self.assertEqual(response.decision, open_lmm.AlignmentDecision.MANUAL)
        np.testing.assert_allclose(
            response.manual_target_T_source[:3, 3], (1.0, 2.0, 3.0)
        )

    def test_manual_reset_and_cancel_do_not_submit(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        runtime.snapshot = lambda: SimpleNamespace(
            pipeline=SimpleNamespace(
                job=SimpleNamespace(id=41), agents=("agent1", "agent2")
            )
        )
        server = FakeServer()
        panel = AlignmentControlPanel(runtime, server, lambda: job)
        review = feedback()
        review.proposal.target_T_source[0, 3] = 4.0
        panel._publish(review)
        panel._feedback = review

        panel._enter_manual_mode()
        panel._reset_manual_transform(True)
        self.assertEqual(panel._manual_controls.position, (0.0, 0.0, 0.0))
        panel._leave_manual_mode(reset=True)
        self.assertEqual(panel._manual_controls.position, (4.0, 0.0, 0.0))
        self.assertFalse(panel._manual_controls.visible)
        self.assertEqual(runtime.responses, [])

    def test_review_source_presentation_uses_frame_independent_of_gizmo(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        runtime.snapshot = lambda: SimpleNamespace(
            pipeline=SimpleNamespace(
                job=SimpleNamespace(id=41), agents=("agent1", "agent2")
            )
        )
        server = FakeServer()
        panel = AlignmentControlPanel(runtime, server, lambda: job)
        panel._publish(feedback())

        self.assertTrue(panel._manual_frame.visible)
        self.assertFalse(panel._manual_controls.visible)
        self.assertTrue(
            server.scene.point_calls[1][0].endswith(
                "/source_transform/agent2"
            )
        )
        self.assertEqual(server.scene.point_calls[0][1]["point_size"], 0.2)
        self.assertEqual(server.scene.point_calls[1][1]["point_size"], 0.2)
        panel._manual_controls.position = (5.0, 6.0, 7.0)
        panel._manual_controls.update()
        self.assertEqual(panel._manual_frame.position, (5.0, 6.0, 7.0))

    def test_review_hides_committed_scene_and_clear_restores_it(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        runtime.snapshot = lambda: SimpleNamespace(
            pipeline=SimpleNamespace(
                job=SimpleNamespace(id=41), agents=("agent1", "agent2")
            )
        )
        committed = FakeCommitSource()
        panel = AlignmentControlPanel(
            runtime,
            FakeServer(),
            lambda: job,
            commit_source=committed,
        )

        panel._publish(feedback())
        self.assertEqual(committed.visibility, [False])
        self.assertTrue(panel._scene_handles)

        panel._clear_scene()
        self.assertEqual(committed.visibility, [False, True])
        self.assertEqual(panel._scene_handles, [])

    def test_response_uses_exact_active_job_request_and_revision(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        panel = AlignmentControlPanel(runtime, object(), lambda: job)
        panel._feedback = SimpleNamespace(
            proposal=SimpleNamespace(request_id=17, source_agent="agent2"),
            session_revision=9,
        )
        panel.respond(open_lmm.AlignmentDecision.ACCEPT)
        self.assertEqual(len(runtime.responses), 1)
        sent_job, response = runtime.responses[0]
        self.assertIs(sent_job, job)
        self.assertEqual(response.request_id, 17)
        self.assertEqual(response.session_revision, 9)
        self.assertEqual(response.decision, open_lmm.AlignmentDecision.ACCEPT)

    def test_external_or_replaced_job_is_read_only(self) -> None:
        active = SimpleNamespace(id=41)
        runtime = FakeRuntime(SimpleNamespace(id=42))
        panel = AlignmentControlPanel(runtime, object(), lambda: active)
        panel._feedback = SimpleNamespace(
            proposal=SimpleNamespace(request_id=17, source_agent="agent2"),
            session_revision=9,
        )
        with self.assertRaisesRegex(RuntimeError, "read-only"):
            panel.respond(open_lmm.AlignmentDecision.CANCEL)
        self.assertEqual(runtime.responses, [])

    def test_failed_replacement_preserves_last_valid_review(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        runtime.snapshot = lambda: SimpleNamespace(
            pipeline=SimpleNamespace(
                job=SimpleNamespace(id=41), agents=("agent1", "agent2")
            )
        )
        server = FakeServer()
        panel = AlignmentControlPanel(runtime, server, lambda: job)
        panel._publish(feedback(1))
        visible = tuple(panel._scene_handles)
        self.assertFalse(any(handle.removed for handle in visible))
        first_colors = server.scene.point_calls[0][1]["colors"]
        second_colors = server.scene.point_calls[1][1]["colors"]
        self.assertEqual(first_colors, (230, 51, 51))
        self.assertEqual(second_colors, (51, 166, 255))

        server.scene.fail = True
        with self.assertRaisesRegex(RuntimeError, "scene failure"):
            panel._publish(feedback(2))
        self.assertEqual(tuple(panel._scene_handles), visible)
        self.assertFalse(any(handle.removed for handle in visible))

    def test_panel_visibility_follows_review_lifecycle(self) -> None:
        job = SimpleNamespace(id=41)
        runtime = FakeRuntime(job)
        panel_handle = SimpleNamespace(visible=False)
        panel = AlignmentControlPanel(
            runtime, object(), lambda: job, panel_handle=panel_handle
        )
        panel._feedback = SimpleNamespace(
            review_state=SimpleNamespace(name="ACTIVE"),
            attempt_status=SimpleNamespace(state=SimpleNamespace(name="IDLE")),
        )
        panel._update_gui()
        self.assertTrue(panel_handle.visible)

        panel._feedback.review_state = SimpleNamespace(name="CANCELLED")
        panel._update_gui()
        self.assertFalse(panel_handle.visible)

        panel._accepted_source = "agent2"
        panel._update_gui()
        self.assertTrue(panel_handle.visible)


if __name__ == "__main__":
    unittest.main()
