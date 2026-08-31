from __future__ import annotations

import inspect
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from open_lmm_viser.config_control import (
    TransactionalConfigPanel,
    _discover_dataset_directories,
)


GLOBAL = "global"
LOOP = "loop"
REMOVER = "remover"


def wait_until(predicate, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition was not satisfied")


class FakeHandle:
    def __init__(self, label: str, value=None) -> None:
        self.label = label
        self.value = value
        self.content = "" if value is None else str(value)
        self.disabled = False
        self.callback = None
        self.update_callback = None
        self.removed = False

    def on_click(self, callback):
        self.callback = callback

    def click(self) -> None:
        self.callback(SimpleNamespace(target=self))

    def on_update(self, callback):
        self.update_callback = callback

    def update(self, value) -> None:
        self.value = value
        self.update_callback(SimpleNamespace(target=self))

    def remove(self) -> None:
        self.removed = True

    def __enter__(self):
        return self

    def __exit__(self, *args) -> None:
        del args


class FakeGui:
    def __init__(self) -> None:
        self.handles: list[FakeHandle] = []

    def _add(self, label: str, value=None) -> FakeHandle:
        handle = FakeHandle(label, value)
        self.handles.append(handle)
        return handle

    def add_markdown(self, content: str) -> FakeHandle:
        return self._add("markdown", content)

    def add_folder(self, label: str):
        return self._add(label)

    def add_text(self, label: str, initial_value: str, **kwargs) -> FakeHandle:
        return self._add(label, initial_value)

    def add_number(self, label: str, initial_value: float, **kwargs) -> FakeHandle:
        return self._add(label, initial_value)

    def add_checkbox(self, label: str, initial_value: bool, **kwargs) -> FakeHandle:
        return self._add(label, initial_value)

    def add_dropdown(
        self, label: str, *, options, initial_value, **kwargs
    ) -> FakeHandle:
        handle = self._add(label, initial_value)
        handle.options = tuple(options)
        return handle

    def add_button(self, label: str, **kwargs) -> FakeHandle:
        return self._add(label)

    def get(self, label: str) -> FakeHandle:
        return next(
            handle
            for handle in reversed(self.handles)
            if handle.label == label and not handle.removed
        )


class FakeRuntime:
    def __init__(self) -> None:
        self.runtime_revision = 4
        self.config_revision = 7
        self.root = {
            "global": {
                "config_loop_detector": "core/loop_detector/scan_context.json"
            },
            "directory": {
                "root_dir_path": "/data",
                "sub_dir_list": ["agent1", "agent2"],
                "root_save_dir": "/output",
            },
            "unknown": {"preserve": True},
        }
        self.loop = {
            "loop_detector": {"model": "scan_context"},
            "alignment": {
                "kiss_voxel_size": 2.0,
                "kiss_use_quatro": False,
                "pose_nn_distance_threshold": 10.0,
                "inter_loop_keyframe_spacing_m": 10.0,
                "unknown_alignment": 99,
            },
        }
        self.remover = {
            "dynamic_remover": {"model": "free_dom", "unknown": 42}
        }
        self.root_calls = []
        self.alignment_calls = []
        self.remover_calls = []
        self.failure: Exception | None = None

    def config_documents(self):
        return SimpleNamespace(
            runtime_revision=self.runtime_revision,
            config_revision=self.config_revision,
            documents=(
                SimpleNamespace(
                    domain=GLOBAL,
                    canonical_json=json.dumps(self.root),
                    selected_document=None,
                ),
                SimpleNamespace(
                    domain=LOOP,
                    canonical_json=json.dumps(self.loop),
                    selected_document="core/loop_detector/scan_context.json",
                ),
                SimpleNamespace(
                    domain=REMOVER,
                    canonical_json=json.dumps(self.remover),
                    selected_document="core/dynamic_remover/free_dom.json",
                ),
            ),
        )

    def config_candidates(self):
        candidates = (
            SimpleNamespace(
                domain=LOOP,
                model="scan_context",
                selected_document="core/loop_detector/scan_context.json",
                canonical_json=json.dumps(self.loop),
            ),
            SimpleNamespace(
                domain=LOOP,
                model="solid",
                selected_document="core/loop_detector/solid.json",
                canonical_json=json.dumps(
                    {"loop_detector": {"model": "solid"}, "candidate": True}
                ),
            ),
            SimpleNamespace(
                domain=REMOVER,
                model="free_dom",
                selected_document="core/dynamic_remover/free_dom.json",
                canonical_json=json.dumps(self.remover),
            ),
            SimpleNamespace(
                domain=REMOVER,
                model="otd",
                selected_document="core/dynamic_remover/otd.json",
                canonical_json=json.dumps(
                    {"dynamic_remover": {"model": "otd", "unknown": 7}}
                ),
            ),
        )
        return SimpleNamespace(
            runtime_revision=self.runtime_revision,
            config_revision=self.config_revision,
            candidates=candidates,
        )

    def replace_root_config(self, document_json, *, expected):
        if self.failure is not None:
            raise self.failure
        self.root_calls.append((json.loads(document_json), expected))
        self.root = json.loads(document_json)
        self.runtime_revision += 1
        self.config_revision += 1

    def apply_config(
        self, domain, document_json, *, expected, selected_document=None
    ):
        if self.failure is not None:
            raise self.failure
        call = (domain, json.loads(document_json), expected, selected_document)
        if domain == LOOP:
            self.alignment_calls.append(call)
            self.loop = json.loads(document_json)
            self.root["global"]["config_loop_detector"] = selected_document
        else:
            self.remover_calls.append(call)
            self.remover = json.loads(document_json)
        self.runtime_revision += 1
        self.config_revision += 1


class ConfigControlTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.dataset_root = Path(self.temporary.name) / "data"
        self.dataset_root.mkdir()
        (self.dataset_root / "agent1").mkdir()
        (self.dataset_root / "agent2").mkdir()
        (self.dataset_root / "not-an-agent.txt").write_text("ignored")
        self.runtime = FakeRuntime()
        self.runtime.root["directory"]["root_dir_path"] = str(self.dataset_root)
        self.gui = FakeGui()
        self.commits = 0
        self.panel = TransactionalConfigPanel(
            self.runtime,
            SimpleNamespace(gui=self.gui),
            global_domain=GLOBAL,
            loop_domain=LOOP,
            remover_domain=REMOVER,
            revision_factory=lambda runtime, config: (runtime, config),
            on_commit=self._committed,
        )
        self.panel.start()
        wait_until(
            lambda: "Committed config loaded" in self.panel._status.content
        )

    def tearDown(self) -> None:
        self.panel.close()
        self.temporary.cleanup()

    def _committed(self) -> None:
        self.commits += 1

    def test_alignment_apply_is_revision_bound_and_preserves_unknown_keys(self) -> None:
        self.gui.get("KISS Voxel Size").value = 0.5
        self.gui.get("KISS Use Quatro").value = True
        self.gui.get("Pose NN Maximum Distance").value = 8.0
        self.gui.get("Inter-loop Spacing").value = 12.0
        self.gui.get("Apply Alignment Config").click()
        wait_until(lambda: len(self.runtime.alignment_calls) == 1)
        domain, document, expected, selected = self.runtime.alignment_calls[0]
        self.assertEqual(domain, LOOP)
        self.assertEqual(expected, (4, 7))
        self.assertEqual(selected, "core/loop_detector/scan_context.json")
        self.assertEqual(document["alignment"]["kiss_voxel_size"], 0.5)
        self.assertEqual(document["alignment"]["unknown_alignment"], 99)
        self.assertEqual(
            tuple(document["alignment"]),
            (
                "kiss_voxel_size",
                "kiss_use_quatro",
                "pose_nn_distance_threshold",
                "inter_loop_keyframe_spacing_m",
                "unknown_alignment",
            ),
        )
        wait_until(lambda: self.commits == 1)

    def test_selector_switch_uses_only_catalog_document(self) -> None:
        self.gui.get("Loop Detector Model").value = "solid"
        self.gui.get("KISS Voxel Size").value = 0.8
        self.gui.get("Apply Alignment Config").click()
        wait_until(lambda: len(self.runtime.alignment_calls) == 1)
        _, document, expected, selected = self.runtime.alignment_calls[0]
        self.assertEqual(expected, (4, 7))
        self.assertEqual(selected, "core/loop_detector/solid.json")
        self.assertTrue(document["candidate"])
        self.assertEqual(document["alignment"]["kiss_voxel_size"], 0.8)

    def test_dynamic_remover_switch_preserves_catalog_document(self) -> None:
        self.gui.get("Dynamic Remover Model").value = "otd"
        self.gui.get("Apply Dynamic Remover").click()
        wait_until(lambda: len(self.runtime.remover_calls) == 1)
        domain, document, expected, selected = self.runtime.remover_calls[0]
        self.assertEqual(domain, REMOVER)
        self.assertEqual(expected, (4, 7))
        self.assertEqual(selected, "core/dynamic_remover/otd.json")
        self.assertEqual(document["dynamic_remover"]["unknown"], 7)

    def test_root_apply_preserves_unknown_keys_and_agent_order(self) -> None:
        new_root = Path(self.temporary.name) / "new-data"
        new_root.mkdir()
        for agent in ("agent1", "agent2", "agent3"):
            (new_root / agent).mkdir()
        self.gui.get("Dataset Root").value = str(new_root)
        self.gui.get("Refresh Agents").click()
        for agent in ("agent2", "agent1", "agent3"):
            self.gui.get(agent).update(True)
        self.gui.get("Output Root").value = "/new-output"
        self.gui.get("Apply Root Config").click()
        wait_until(lambda: len(self.runtime.root_calls) == 1)
        document, expected = self.runtime.root_calls[0]
        self.assertEqual(expected, (4, 7))
        self.assertEqual(
            document["directory"]["sub_dir_list"],
            ["agent2", "agent1", "agent3"],
        )
        self.assertEqual(document["unknown"], {"preserve": True})
        self.assertEqual(tuple(document), ("global", "directory", "unknown"))

    def test_agent_catalog_lists_only_directories_and_tracks_selection_order(self) -> None:
        self.assertFalse(any(handle.label == "not-an-agent.txt" for handle in self.gui.handles))
        self.gui.get("agent1").update(False)
        self.gui.get("agent1").update(True)
        self.assertIn(
            "`agent2` → `agent1`",
            str(self.panel._agent_order.content),
        )

    def test_agent_catalog_is_bounded(self) -> None:
        root = Path(self.temporary.name) / "large"
        root.mkdir()
        for index in range(1025):
            (root / f"agent-{index:04d}").mkdir()
        with self.assertRaisesRegex(ValueError, "1024-directory UI limit"):
            _discover_dataset_directories(str(root))

    def test_failed_agent_refresh_preserves_selection(self) -> None:
        self.gui.get("Dataset Root").value = str(
            Path(self.temporary.name) / "missing"
        )
        self.gui.get("Refresh Agents").click()
        wait_until(lambda: "cannot be scanned" in self.panel._status.content)
        self.gui.get("Dataset Root").value = str(self.dataset_root)
        self.gui.get("Apply Root Config").click()
        wait_until(lambda: len(self.runtime.root_calls) == 1)
        document, _ = self.runtime.root_calls[0]
        self.assertEqual(
            document["directory"]["sub_dir_list"], ["agent1", "agent2"]
        )

    def test_failed_apply_keeps_draft_and_requires_reload(self) -> None:
        self.runtime.failure = RuntimeError("revision conflict")
        self.gui.get("KISS Voxel Size").value = 0.7
        self.gui.get("Apply Alignment Config").click()
        wait_until(lambda: "revision conflict" in self.panel._status.content)
        self.assertEqual(self.gui.get("KISS Voxel Size").value, 0.7)
        self.assertEqual(self.commits, 0)

    def test_close_removes_all_handles(self) -> None:
        handles = tuple(self.gui.handles)
        self.panel.close()
        self.assertTrue(all(handle.removed for handle in handles))

    def test_config_panel_only_scans_dataset_directory(self) -> None:
        source = inspect.getsource(TransactionalConfigPanel)
        self.assertNotIn("open(", source)
        self.assertNotIn("Path(", source)
        self.assertIn("_discover_dataset_directories", source)


if __name__ == "__main__":
    unittest.main()
