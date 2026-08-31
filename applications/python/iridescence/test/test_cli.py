from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from open_lmm_iridescence.cli import _execute, _parser


class Runtime:
    def __init__(self) -> None:
        self.opened = False
        self.closed = False

    def open(self, *_args, **_kwargs) -> None:
        self.opened = True

    def close(self) -> None:
        self.closed = True


class Application:
    def __init__(self, runtime, **options) -> None:
        self.runtime = runtime
        self.options = options
        self.closed = False

    def run(self, _stop_event) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class CliTests(unittest.TestCase):
    def test_parser_defaults_and_positive_voxel(self) -> None:
        args = _parser().parse_args(["config"])
        self.assertEqual(args.label, "iridescence")
        self.assertFalse(args.auto_run)
        with self.assertRaises(SystemExit):
            _parser().parse_args(["config", "--preview-voxel-size-m", "0"])

    def test_execute_validates_opens_runs_and_closes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "config.json").write_text("{}", encoding="utf-8")
            runtime = Runtime()
            args = argparse.Namespace(
                config_dir=directory,
                label="test",
                output_root=None,
                preview_voxel_size_m=0.2,
                auto_run=True,
            )
            with mock.patch.dict(os.environ, {"DISPLAY": ":99"}):
                code = _execute(
                    args,
                    runtime_factory=lambda: runtime,
                    application_factory=Application,
                )
            self.assertEqual(code, 0)
            self.assertTrue(runtime.opened)
            self.assertTrue(runtime.closed)

    def test_missing_display_is_a_diagnostic_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "config.json").write_text("{}", encoding="utf-8")
            args = argparse.Namespace(
                config_dir=directory,
                label="test",
                output_root=None,
                preview_voxel_size_m=None,
                auto_run=False,
            )
            with mock.patch.dict(os.environ, {}, clear=True):
                self.assertEqual(
                    _execute(args, runtime_factory=Runtime, application_factory=Application),
                    1,
                )


if __name__ == "__main__":
    unittest.main()
