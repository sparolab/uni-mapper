from __future__ import annotations

import json
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import open_lmm
import open_lmm_viser
from open_lmm_viser.config_control import TransactionalConfigPanel
from open_lmm_viser.control import RuntimeControlPanel


ROOT = Path(__file__).resolve().parents[1]


class Subscription:
    def close(self) -> None:
        pass


class PackageTests(unittest.TestCase):
    def test_public_surface_is_application_components_only(self) -> None:
        self.assertEqual(
            open_lmm_viser.__all__, ["AlignmentControlPanel", "ViserAdapter"]
        )

    def test_source_has_no_private_sdk_or_core_dependency(self) -> None:
        source = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "package").rglob("*.py")
        )
        self.assertNotIn("open_lmm._native", source)
        self.assertNotIn("open_lmm.core", source)
        self.assertNotIn("plugins.host", source)

    def test_application_package_contains_no_native_binary(self) -> None:
        forbidden = {".so", ".dylib", ".dll", ".a"}
        self.assertFalse(
            [path for path in ROOT.rglob("*") if path.suffix.lower() in forbidden]
        )

    def test_runtime_dependencies_do_not_include_viser(self) -> None:
        sdk = ROOT.parents[2] / "bindings" / "python" / "pyproject.toml"
        core = ROOT.parents[2] / "open_lmm" / "CMakeLists.txt"
        self.assertNotIn("viser", sdk.read_text(encoding="utf-8").lower())
        self.assertNotIn("viser", core.read_text(encoding="utf-8").lower())

    def test_installed_viser_server_bootstrap_and_one_agent_scene(self) -> None:
        try:
            import viser
        except ImportError:
            self.skipTest("installed-package smoke requires viser")

        class Runtime:
            def subscribe_events(self, callback):
                self.callback = callback
                return Subscription()

            def snapshot(self):
                return SimpleNamespace(
                    pipeline=SimpleNamespace(agents=("agent1",))
                )

            def visualization(self, agent, *, preview_voxel_size_m=None):
                poses = np.repeat(
                    np.eye(4, dtype=np.float32)[None, :, :], 2, axis=0
                )
                poses[1, 0, 3] = 1.0
                return SimpleNamespace(
                    agent=agent,
                    revision=1,
                    points_available=True,
                    points=np.array([[1.0, 2.0, 3.0, 4.0]], np.float32),
                    poses=poses,
                )

        try:
            server = viser.ViserServer(
                host="127.0.0.1", port=0, verbose=False
            )
        except PermissionError:
            self.skipTest("sandbox does not permit loopback server sockets")
        adapter = open_lmm_viser.ViserAdapter(Runtime(), server)
        try:
            adapter.start()
            deadline = time.monotonic() + 2.0
            while not adapter._displayed and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(adapter._displayed, {"agent1": 1})
        finally:
            adapter.close()
            server.stop()

    def test_installed_viser_control_panel_bootstrap(self) -> None:
        try:
            import viser
        except ImportError:
            self.skipTest("installed-package smoke requires viser")

        class Runtime:
            def subscribe_events(self, callback):
                self.callback = callback
                return Subscription()

            def snapshot(self):
                return SimpleNamespace(
                    status=SimpleNamespace(name="READY"),
                    pipeline=SimpleNamespace(
                        runtime_revision=1,
                        config_revision=1,
                        agents=("agent1", "agent2"),
                        job=None,
                    ),
                )

            def config_documents(self):
                root = {
                    "directory": {
                        "root_dir_path": "/data",
                        "sub_dir_list": ["agent1", "agent2"],
                        "root_save_dir": "/output",
                    }
                }
                loop = {
                    "alignment": {
                        "kiss_voxel_size": 2.0,
                        "kiss_use_quatro": False,
                        "pose_nn_distance_threshold": 10.0,
                        "inter_loop_keyframe_spacing_m": 10.0,
                    }
                }
                remover = {"dynamic_remover": {"model": "free_dom"}}
                return SimpleNamespace(
                    runtime_revision=1,
                    config_revision=1,
                    documents=(
                        SimpleNamespace(
                            domain=open_lmm.ConfigDomain.GLOBAL,
                            canonical_json=json.dumps(root),
                        ),
                        SimpleNamespace(
                            domain=open_lmm.ConfigDomain.LOOP_DETECTOR,
                            canonical_json=json.dumps(loop),
                            selected_document="core/loop_detector/scan_context.json",
                        ),
                        SimpleNamespace(
                            domain=open_lmm.ConfigDomain.DYNAMIC_REMOVER,
                            canonical_json=json.dumps(remover),
                            selected_document="core/dynamic_remover/free_dom.json",
                        ),
                    ),
                )

            def config_candidates(self):
                loop = {
                    "alignment": {
                        "kiss_voxel_size": 2.0,
                        "kiss_use_quatro": False,
                        "pose_nn_distance_threshold": 10.0,
                        "inter_loop_keyframe_spacing_m": 10.0,
                    }
                }
                return SimpleNamespace(
                    runtime_revision=1,
                    config_revision=1,
                    candidates=(
                        SimpleNamespace(
                            domain=open_lmm.ConfigDomain.LOOP_DETECTOR,
                            model="scan_context",
                            selected_document="core/loop_detector/scan_context.json",
                            canonical_json=json.dumps(loop),
                        ),
                        SimpleNamespace(
                            domain=open_lmm.ConfigDomain.DYNAMIC_REMOVER,
                            model="free_dom",
                            selected_document="core/dynamic_remover/free_dom.json",
                            canonical_json=json.dumps(
                                {"dynamic_remover": {"model": "free_dom"}}
                            ),
                        ),
                    ),
                )

        try:
            server = viser.ViserServer(
                host="127.0.0.1", port=0, verbose=False
            )
        except PermissionError:
            self.skipTest("sandbox does not permit loopback server sockets")
        runtime = Runtime()
        panel = RuntimeControlPanel(
            runtime,
            server,
            (("Data Load", 0),),
            (("Data Load", 0, True),),
        )
        config_panel = TransactionalConfigPanel(
            runtime,
            server,
            global_domain=open_lmm.ConfigDomain.GLOBAL,
            loop_domain=open_lmm.ConfigDomain.LOOP_DETECTOR,
            remover_domain=open_lmm.ConfigDomain.DYNAMIC_REMOVER,
            revision_factory=open_lmm.Revision,
        )
        try:
            panel.start()
            config_panel.start()
            self.assertIsNotNone(panel._status_markdown)
            self.assertEqual(len(panel._command_buttons), 4)
            self.assertIsNotNone(panel._cancel_button)
            deadline = time.monotonic() + 2.0
            while (
                tuple(panel._agent_selector.options) != ("agent1", "agent2")
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertEqual(
                tuple(panel._agent_selector.options), ("agent1", "agent2")
            )
            self.assertEqual(config_panel._dataset_root.value, "/data")
            self.assertEqual(config_panel._kiss_voxel.value, 2.0)
        finally:
            config_panel.close()
            panel.close()
            server.stop()


if __name__ == "__main__":
    unittest.main()
