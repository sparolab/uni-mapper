from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import sys
import threading
from typing import Any, Callable

from .application import IridescenceApplication


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="open-lmm-iridescence")
    parser.add_argument("config_dir", metavar="CONFIG_DIR")
    parser.add_argument("--label", default="iridescence")
    parser.add_argument("--output-root")
    parser.add_argument("--preview-voxel-size-m", type=_positive_float)
    parser.add_argument("--auto-run", action="store_true")
    parser.add_argument("--no-auto-run", action="store_false", dest="auto_run", help=argparse.SUPPRESS)
    parser.set_defaults(auto_run=False)
    return parser


def _validate(arguments: argparse.Namespace) -> None:
    config_dir = Path(arguments.config_dir)
    if not config_dir.is_dir():
        raise ValueError(f"configuration directory does not exist: {config_dir}")
    config_file = config_dir / "config.json"
    if not config_file.is_file():
        raise ValueError(f"root configuration does not exist: {config_file}")
    if not arguments.label:
        raise ValueError("label must not be empty")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise RuntimeError("DISPLAY or WAYLAND_DISPLAY is required for Iridescence")
    if arguments.output_root is not None:
        output = Path(arguments.output_root)
        parent = output if output.exists() else output.parent
        if not parent.exists():
            raise ValueError(f"output parent does not exist: {parent}")


def _execute(
    arguments: argparse.Namespace,
    *,
    runtime_factory: Callable[[], Any],
    application_factory: Callable[..., Any] = IridescenceApplication,
) -> int:
    runtime: Any | None = None
    application: Any | None = None
    stop_event = threading.Event()
    previous_handlers: dict[int, Any] = {}

    def request_stop(_number: int, _frame: Any) -> None:
        stop_event.set()

    try:
        _validate(arguments)
        for number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[number] = signal.signal(number, request_stop)
        runtime = runtime_factory()
        runtime.open(
            arguments.config_dir,
            label=arguments.label,
            output_root=arguments.output_root,
        )
        application = application_factory(
            runtime,
            preview_voxel_size_m=arguments.preview_voxel_size_m,
            auto_run=arguments.auto_run,
        )
        application.run(stop_event)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"open-lmm-iridescence: {error}", file=sys.stderr)
        return 1
    finally:
        if application is not None:
            try:
                application.close()
            except Exception as error:
                print(f"open-lmm-iridescence: viewer close failed: {error}", file=sys.stderr)
        if runtime is not None:
            try:
                runtime.close()
            except Exception as error:
                print(f"open-lmm-iridescence: runtime close failed: {error}", file=sys.stderr)
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    import open_lmm
    return _execute(arguments, runtime_factory=open_lmm.Runtime)
