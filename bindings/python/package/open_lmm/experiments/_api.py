from __future__ import annotations

from os import PathLike
from pathlib import Path

from ._canonical import load_closed_json
from ._manifest import load_plan, validate_manifest_document
from ._models import ExperimentPlan, ExperimentResult
from ._runner import run_experiment


def validate_manifest(manifest: str | PathLike[str]) -> None:
    """Validate a closed experiment manifest without materializing a run."""
    validate_manifest_document(load_closed_json(Path(manifest).resolve()))


class Experiment:
    __slots__ = ("_plan", "_evidence_root")

    def __init__(self, plan: ExperimentPlan, *, evidence_root: str | PathLike[str]) -> None:
        if not isinstance(plan, ExperimentPlan):
            raise TypeError("plan must be an ExperimentPlan")
        self._plan = plan
        self._evidence_root = Path(evidence_root).resolve()

    @property
    def plan(self) -> ExperimentPlan:
        return self._plan

    @classmethod
    def from_manifest(
        cls,
        manifest: str | PathLike[str],
        *,
        dataset_root: str | PathLike[str],
        config_root: str | PathLike[str] | None = None,
        evidence_root: str | PathLike[str],
    ) -> "Experiment":
        manifest_path = Path(manifest).resolve()
        plan, _ = load_plan(
            manifest_path,
            dataset_root=Path(dataset_root).resolve(),
            config_root=None if config_root is None else Path(config_root).resolve(),
        )
        return cls(plan, evidence_root=evidence_root)

    def run(self) -> ExperimentResult:
        return run_experiment(self._plan, self._evidence_root)
