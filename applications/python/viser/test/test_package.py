from __future__ import annotations

import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import open_lmm_viser


ROOT = Path(__file__).resolve().parents[1]


class PackageTests(unittest.TestCase):
    def test_public_surface_is_adapter_only(self) -> None:
        self.assertEqual(open_lmm_viser.__all__, ["ViserAdapter"])

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

        class Subscription:
            def close(self) -> None:
                pass

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


if __name__ == "__main__":
    unittest.main()
