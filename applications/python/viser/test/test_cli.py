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

    def open(self, *args, **kwargs) -> None:
        if self.open_error is not None:
            raise self.open_error

    def run_all(self):
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


class CliTests(unittest.TestCase):
    def arguments(self):
        return cli._parser().parse_args(["config"])

    def test_defaults_use_loopback(self) -> None:
        arguments = self.arguments()
        self.assertEqual(arguments.host, "127.0.0.1")
        self.assertEqual(arguments.port, 8080)

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
            )
        self.assertEqual(result, 0)
        self.assertTrue(runtime.closed)
        self.assertTrue(server.stopped)

    def test_runtime_open_and_pipeline_failures_exit_one(self) -> None:
        for runtime in (
            FakeRuntime(open_error=RuntimeError("open")),
            FakeRuntime(job_error=RuntimeError("job")),
        ):
            with self.subTest(runtime=runtime), patch.object(
                cli, "ViserAdapter", FakeAdapter
            ), contextlib.redirect_stderr(io.StringIO()):
                result = cli._execute(
                    self.arguments(),
                    runtime_factory=lambda runtime=runtime: runtime,
                    server_factory=lambda **kwargs: FakeServer(),
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
            )
        self.assertEqual(result, 1)


if __name__ == "__main__":
    unittest.main()
