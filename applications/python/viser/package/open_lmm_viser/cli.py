from __future__ import annotations

import argparse
import math
import sys
import threading
from collections.abc import Callable, Sequence
from typing import Any

from .adapter import ViserAdapter
from .alignment_control import AlignmentControlPanel
from .config_control import TransactionalConfigPanel
from .control import RuntimeControlPanel


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
    parser.add_argument(
        "--auto-run",
        action="store_true",
        dest="auto_run",
        help="run the full pipeline immediately after opening the viewer",
    )
    parser.add_argument(
        "--no-auto-run",
        action="store_false",
        dest="auto_run",
        help=argparse.SUPPRESS,
    )
    parser.set_defaults(auto_run=False)
    return parser


def _wait_forever() -> None:
    threading.Event().wait()


def _application_panel(
    server: Any,
    label: str,
    *,
    order: float,
    visible: bool = True,
) -> tuple[Any | None, Any | None]:
    gui = getattr(server, "gui", None)
    add_panel = getattr(gui, "add_panel", None)
    if not callable(add_panel):
        return None, None
    panel = add_panel(order=order, visible=visible)
    return panel, panel.add_tab(label)


def _place_panel(panel: Any | None, side: str, width: float) -> None:
    if panel is None:
        return
    if side == "left":
        panel.dock_left()
    else:
        panel.dock_right()
    panel.set_width(width)


def _execute(
    arguments: argparse.Namespace,
    *,
    runtime_factory: Callable[[], Any],
    server_factory: Callable[..., Any],
    wait_forever: Callable[[], None] = _wait_forever,
    control_factory: Callable[..., Any] = RuntimeControlPanel,
    stages: Sequence[tuple[str, object]] = (),
    nodes: Sequence[tuple[str, object, bool]] = (),
    config_factory: Callable[..., Any] = TransactionalConfigPanel,
    config_domains: tuple[object, object, object] | None = None,
    revision_factory: Callable[[int, int], object] | None = None,
    alignment_factory: Callable[..., Any] | None = None,
) -> int:
    runtime: Any | None = None
    server: Any | None = None
    adapter: ViserAdapter | None = None
    control: RuntimeControlPanel | None = None
    config_control: TransactionalConfigPanel | None = None
    alignment_control: AlignmentControlPanel | None = None
    panels: list[Any] = []
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
        control = control_factory(
            runtime,
            server,
            stages,
            nodes,
        )
        runtime_panel, runtime_tab = _application_panel(
            server, "Runtime Control", order=0.0
        )
        if runtime_panel is not None:
            panels.append(runtime_panel)
            with runtime_tab:
                control.start()
            _place_panel(runtime_panel, "left", 340.0)
        else:
            control.start()
        if alignment_factory is not None:
            alignment_panel, alignment_tab = _application_panel(
                server, "Alignment Review", order=1.0, visible=False
            )
            if alignment_panel is not None:
                panels.append(alignment_panel)
            alignment_control = alignment_factory(
                runtime,
                server,
                control.active_job,
                commit_source=adapter,
                panel_handle=alignment_panel,
            )
            if alignment_tab is not None:
                with alignment_tab:
                    alignment_control.start()
                _place_panel(alignment_panel, "right", 380.0)
            else:
                alignment_control.start()
        if config_domains is not None and revision_factory is not None:
            config_panel, config_tab = _application_panel(
                server, "Configuration", order=2.0
            )
            if config_panel is not None:
                panels.append(config_panel)
            config_control = config_factory(
                runtime,
                server,
                global_domain=config_domains[0],
                loop_domain=config_domains[1],
                remover_domain=config_domains[2],
                revision_factory=revision_factory,
                on_commit=control.refresh_snapshot,
            )
            if config_tab is not None:
                with config_tab:
                    config_control.start()
                _place_panel(config_panel, "right", 380.0)
            else:
                config_control.start()
        host = arguments.host
        port = server.get_port() if hasattr(server, "get_port") else arguments.port
        print(f"OpenLMM Viser: http://{host}:{port}")
        if arguments.auto_run:
            generation = control.submit_run_all()
            control.wait(generation)
            pipeline_completed = True
        else:
            pipeline_completed = True
        wait_forever()
        return 0
    except KeyboardInterrupt:
        return 0 if pipeline_completed else 1
    except Exception as error:
        print(f"open-lmm-viser: {error}", file=sys.stderr)
        return 1
    finally:
        if alignment_control is not None:
            alignment_control.close()
        if config_control is not None:
            config_control.close()
        if control is not None:
            control.close()
        if adapter is not None:
            adapter.close()
        for panel in reversed(panels):
            try:
                panel.remove()
            except Exception:
                pass
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
        stages=(
            ("Data Load", open_lmm.Stage.DATA_LOAD),
            ("Alignment", open_lmm.Stage.ALIGNMENT),
            ("Map Update", open_lmm.Stage.MAP_UPDATE),
            ("Save", open_lmm.Stage.SAVE),
        ),
        nodes=(
            ("Data Load", open_lmm.Node.DATA_LOAD, True),
            ("Loop Detect", open_lmm.Node.LOOP_DETECT, True),
            ("Optimize", open_lmm.Node.OPTIMIZE, True),
            ("Map Update", open_lmm.Node.MAP_UPDATE, True),
            ("Pose Save", open_lmm.Node.POSE_SAVE, False),
            ("Fallback Map Save", open_lmm.Node.FALLBACK_MAP_SAVE, False),
        ),
        config_domains=(
            open_lmm.ConfigDomain.GLOBAL,
            open_lmm.ConfigDomain.LOOP_DETECTOR,
            open_lmm.ConfigDomain.DYNAMIC_REMOVER,
        ),
        revision_factory=open_lmm.Revision,
        alignment_factory=AlignmentControlPanel,
    )


if __name__ == "__main__":
    raise SystemExit(main())
