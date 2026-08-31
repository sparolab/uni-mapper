from __future__ import annotations

import argparse
import math
import sys
import threading
from collections.abc import Callable
from typing import Any

from .adapter import ViserAdapter


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def _port(value: str) -> int:
    parsed = int(value)
    if parsed < 1 or parsed > 65535:
        raise argparse.ArgumentTypeError("must be in the range 1..65535")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="open-lmm-viser")
    parser.add_argument("config_dir", metavar="CONFIG_DIR")
    parser.add_argument("--label", default="viser")
    parser.add_argument("--output-root")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8080, type=_port)
    parser.add_argument("--preview-voxel-size-m", type=_positive_float)
    return parser


def _wait_forever() -> None:
    threading.Event().wait()


def _execute(
    arguments: argparse.Namespace,
    *,
    runtime_factory: Callable[[], Any],
    server_factory: Callable[..., Any],
    wait_forever: Callable[[], None] = _wait_forever,
) -> int:
    runtime: Any | None = None
    server: Any | None = None
    adapter: ViserAdapter | None = None
    pipeline_completed = False
    try:
        server = server_factory(host=arguments.host, port=arguments.port)
        server.scene.set_up_direction("+z")
        runtime = runtime_factory()
        runtime.open(
            arguments.config_dir,
            label=arguments.label,
            output_root=arguments.output_root,
        )
        adapter = ViserAdapter(
            runtime,
            server,
            preview_voxel_size_m=arguments.preview_voxel_size_m,
        )
        adapter.start()
        host = arguments.host
        port = server.get_port() if hasattr(server, "get_port") else arguments.port
        print(f"OpenLMM Viser: http://{host}:{port}")
        job = runtime.run_all()
        job.wait()
        pipeline_completed = True
        wait_forever()
        return 0
    except KeyboardInterrupt:
        return 0 if pipeline_completed else 1
    except Exception as error:
        print(f"open-lmm-viser: {error}", file=sys.stderr)
        return 1
    finally:
        if adapter is not None:
            adapter.close()
        if runtime is not None:
            try:
                runtime.close()
            except Exception as error:
                print(f"open-lmm-viser: runtime close failed: {error}", file=sys.stderr)
        if server is not None:
            try:
                server.stop()
            except Exception as error:
                print(f"open-lmm-viser: server stop failed: {error}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        import open_lmm
        import viser
    except ImportError as error:
        print(
            f"open-lmm-viser: required dependency is unavailable: {error}",
            file=sys.stderr,
        )
        return 1
    return _execute(
        arguments,
        runtime_factory=open_lmm.Runtime,
        server_factory=viser.ViserServer,
    )


if __name__ == "__main__":
    raise SystemExit(main())
