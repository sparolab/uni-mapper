from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ._api import Experiment
from ._manifest import validate_manifest_document
from ._canonical import load_closed_json
from ._models import ExperimentStatus


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="open-lmm-experiment")
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser("validate", help="validate a closed experiment manifest")
    validate.add_argument("--manifest", required=True)
    run = commands.add_parser("run", help="run a locked local experiment")
    run.add_argument("--manifest", required=True)
    run.add_argument("--dataset-root", required=True)
    run.add_argument("--config-root")
    run.add_argument("--evidence-root", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    try:
        arguments = parser.parse_args(argv)
        if arguments.command == "validate":
            validate_manifest_document(load_closed_json(Path(arguments.manifest)))
            return 0
        if not Path(arguments.dataset_root).is_dir() or (
            arguments.config_root is not None
            and not Path(arguments.config_root).is_dir()
        ):
            print("open-lmm-experiment: dataset/config input is not available", file=sys.stderr)
            return 77
        experiment = Experiment.from_manifest(
            arguments.manifest,
            dataset_root=arguments.dataset_root,
            config_root=arguments.config_root,
            evidence_root=arguments.evidence_root,
        )
        result = experiment.run()
        return 0 if result.result == ExperimentStatus.SUCCEEDED else 1
    except (FileNotFoundError, FileExistsError, TypeError, ValueError) as error:
        print(f"open-lmm-experiment: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
