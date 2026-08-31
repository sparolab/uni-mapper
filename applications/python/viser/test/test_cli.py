from __future__ import annotations

import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from open_lmm_viser import cli


class FakeJob:
    def __init__(self, error: Exception | None = None) -> None:
        self.error = error

    def wait(self) -> None:
        if self.error is not None:
            raise self.error


class FakeRuntime:
    def __init__(self, *, open_error=None, job_error=None) -> None:
        self.open_error = open_error
        self.job_error = job_error
        self.closed = False
        self.run_calls = 0

    def open(self, *args, **kwargs) -> None:
        if self.open_error is not None:
            raise self.open_error

    def run_all(self):
        self.run_calls += 1
        return FakeJob(self.job_error)

    def close(self) -> None:
        self.closed = True


class FakeServer:
    def __init__(self) -> None:
        self.scene = SimpleNamespace(set_up_direction=lambda value: None)
        self.stopped = False

    def get_port(self) -> int:
        return 8080

    def stop(self) -> None:
        self.stopped = True


class FakeAdapter:
    def __init__(self, runtime, server, **kwargs) -> None:
        self.closed = False

    def start(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class FakeControl:
    def __init__(self, runtime, server, stages, nodes) -> None:
        del server, stages, nodes
        self.runtime = runtime
        self.job = None
        self.closed = False

    def start(self) -> None:
        pass

    def submit_run_all(self) -> int:
        self.job = self.runtime.run_all()
        return 1

    def wait(self, generation: int) -> None:
        self.job.wait()

    def close(self) -> None:
        self.closed = True

    def refresh_snapshot(self) -> None:
        pass

    def active_job(self):
        return self.job


class FakeTab:
    def __init__(self, label: str) -> None:
        self.label = label

    def __enter__(self):
        return self

    def __exit__(self, *args) -> None:
        del args


class FakePanel:
    def __init__(self, *, visible: bool) -> None:
        self.visible = visible
        self.tabs = []
        self.placement = None
        self.width = None
        self.removed = False

    def add_tab(self, label: str):
        tab = FakeTab(label)
        self.tabs.append(tab)
        return tab

    def dock_left(self) -> None:
        self.placement = "left"

    def dock_right(self) -> None:
        self.placement = "right"

    def set_width(self, width: float) -> None:
        self.width = width

    def remove(self) -> None:
        self.removed = True


class FakeGui:
    def __init__(self) -> None:
        self.panels = []

    def add_panel(self, *, order: float, visible: bool):
        del order
        panel = FakePanel(visible=visible)
        self.panels.append(panel)
        return panel


class FakePanelServer(FakeServer):
    def __init__(self) -> None:
        super().__init__()
        self.gui = FakeGui()


class FakeAlignmentControl:
    instances = []

    def __init__(self, runtime, server, active_job, **kwargs) -> None:
        del runtime, server, active_job
        self.panel = kwargs["panel_handle"]
        self.started = False
        self.closed = False
        self.instances.append(self)

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.closed = True


class FakeConfigControl:
    instances = []

    def __init__(self, runtime, server, **kwargs) -> None:
        del runtime, server
        self.kwargs = kwargs
        self.started = False
        self.closed = False
        self.instances.append(self)

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.closed = True


class CliTests(unittest.TestCase):
    def arguments(self):
        return cli._parser().parse_args(["config"])

    def test_defaults_use_loopback(self) -> None:
        arguments = self.arguments()
        self.assertEqual(arguments.host, "127.0.0.1")
        self.assertEqual(arguments.port, 8080)
        self.assertFalse(arguments.auto_run)

    def test_auto_run_requires_explicit_opt_in(self) -> None:
        arguments = cli._parser().parse_args(["config", "--auto-run"])
        self.assertTrue(arguments.auto_run)

    def test_invalid_port_and_voxel_exit_two(self) -> None:
        invalid_options = (
            ("--port", "0"),
            ("--port", "65536"),
            ("--preview-voxel-size-m", "nan"),
        )
        for options in invalid_options:
            with self.subTest(options=options), self.assertRaises(SystemExit) as raised:
                cli._parser().parse_args(["config", *options])
            self.assertEqual(raised.exception.code, 2)

    def test_success_after_pipeline_exits_zero_on_interrupt(self) -> None:
        runtime = FakeRuntime()
        server = FakeServer()
        with patch.object(cli, "ViserAdapter", FakeAdapter):
            result = cli._execute(
                self.arguments(),
                runtime_factory=lambda: runtime,
                server_factory=lambda **kwargs: server,
                wait_forever=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
                control_factory=FakeControl,
            )
        self.assertEqual(result, 0)
        self.assertTrue(runtime.closed)
        self.assertTrue(server.stopped)

    def test_runtime_open_and_pipeline_failures_exit_one(self) -> None:
        for runtime in (
            FakeRuntime(open_error=RuntimeError("open")),
            FakeRuntime(job_error=RuntimeError("job")),
        ):
            arguments = self.arguments()
            arguments.auto_run = True
            with self.subTest(runtime=runtime), patch.object(
                cli, "ViserAdapter", FakeAdapter
            ), contextlib.redirect_stderr(io.StringIO()):
                result = cli._execute(
                    arguments,
                    runtime_factory=lambda runtime=runtime: runtime,
                    server_factory=lambda **kwargs: FakeServer(),
                    control_factory=FakeControl,
                )
            self.assertEqual(result, 1)

    def test_server_bootstrap_failure_exits_one(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            result = cli._execute(
                self.arguments(),
                runtime_factory=FakeRuntime,
                server_factory=lambda **kwargs: (_ for _ in ()).throw(
                    OSError("bind")
                ),
                control_factory=FakeControl,
            )
        self.assertEqual(result, 1)

    def test_no_auto_run_opens_idle_until_interrupt(self) -> None:
        runtime = FakeRuntime()
        arguments = self.arguments()
        arguments.auto_run = False
        with patch.object(cli, "ViserAdapter", FakeAdapter):
            result = cli._execute(
                arguments,
                runtime_factory=lambda: runtime,
                server_factory=lambda **kwargs: FakeServer(),
                wait_forever=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
                control_factory=FakeControl,
            )
        self.assertEqual(result, 0)
        self.assertEqual(runtime.run_calls, 0)

    def test_config_panel_is_composed_and_closed(self) -> None:
        FakeConfigControl.instances.clear()
        runtime = FakeRuntime()
        arguments = self.arguments()
        arguments.auto_run = False
        with patch.object(cli, "ViserAdapter", FakeAdapter):
            result = cli._execute(
                arguments,
                runtime_factory=lambda: runtime,
                server_factory=lambda **kwargs: FakeServer(),
                wait_forever=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
                control_factory=FakeControl,
                config_factory=FakeConfigControl,
                config_domains=("global", "loop", "remover"),
                revision_factory=lambda runtime, config: (runtime, config),
            )
        self.assertEqual(result, 0)
        self.assertEqual(len(FakeConfigControl.instances), 1)
        instance = FakeConfigControl.instances[0]
        self.assertTrue(instance.started)
        self.assertTrue(instance.closed)
        self.assertEqual(instance.kwargs["global_domain"], "global")

    def test_runtime_alignment_and_config_use_separate_panels(self) -> None:
        FakeAlignmentControl.instances.clear()
        FakeConfigControl.instances.clear()
        runtime = FakeRuntime()
        server = FakePanelServer()
        with patch.object(cli, "ViserAdapter", FakeAdapter):
            result = cli._execute(
                self.arguments(),
                runtime_factory=lambda: runtime,
                server_factory=lambda **kwargs: server,
                wait_forever=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
                control_factory=FakeControl,
                alignment_factory=FakeAlignmentControl,
                config_factory=FakeConfigControl,
                config_domains=("global", "loop", "remover"),
                revision_factory=lambda runtime, config: (runtime, config),
            )
        self.assertEqual(result, 0)
        self.assertEqual(
            [panel.tabs[0].label for panel in server.gui.panels],
            ["Runtime Control", "Alignment Review", "Configuration"],
        )
        self.assertEqual(
            [panel.placement for panel in server.gui.panels],
            ["left", "right", "right"],
        )
        self.assertFalse(FakeAlignmentControl.instances[0].panel.visible)
        self.assertTrue(all(panel.removed for panel in server.gui.panels))


if __name__ == "__main__":
    unittest.main()
