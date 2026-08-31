from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from open_lmm import Stage

from ._canonical import (
    canonical_json_bytes,
    digest_file,
    load_closed_json,
    require_keys,
    require_relative_path,
    require_sha256,
)
from ._models import (
    AlgorithmSlot,
    AlgorithmVariant,
    ConfigPatch,
    ConfigSpec,
    ConfigTarget,
    DatasetCatalog,
    DatasetSpec,
    ExecutionMode,
    ExecutionPolicy,
    ExperimentPlan,
    FailurePolicy,
    InputLockKind,
    LockedFile,
    MetricPolicy,
    ParameterAxis,
    SoftwareIdentity,
    WorkflowKind,
    WorkflowSpec,
)


_STAGES = {
    "data-load": Stage.DATA_LOAD,
    "alignment": Stage.ALIGNMENT,
    "map-update": Stage.MAP_UPDATE,
    "save": Stage.SAVE,
}


def _list(value: Any, where: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{where} must be an array")
    return value


def _positive_int(value: Any, where: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ValueError(f"{where} must be a positive integer")
    return value


def _boolean(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{where} must be a boolean")
    return value


def _positive_number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{where} must be a positive finite number")
    return float(value)


def _locked_file(value: Any, where: str) -> LockedFile:
    item = require_keys(value, required={"path", "sha256"}, optional={"size"}, where=where)
    return LockedFile(item["path"], item["sha256"], item.get("size"))


def _target(value: Any, where: str) -> ConfigTarget:
    item = require_keys(value, required={"file", "pointer"}, where=where)
    return ConfigTarget(item["file"], item["pointer"])


def _patch(value: Any, where: str) -> ConfigPatch:
    item = require_keys(value, required={"file", "pointer", "value"}, where=where)
    return ConfigPatch(item["file"], item["pointer"], item["value"])


def validate_manifest_document(document: Any) -> Mapping[str, Any]:
    root = require_keys(
        document,
        required={
            "schema_version",
            "experiment_id",
            "dataset",
            "config",
            "software",
            "workflow",
            "execution",
            "fixed_patches",
            "parameter_axes",
            "algorithm_variants",
            "metrics",
        },
        where="experiment manifest",
    )
    if root["schema_version"] != 1:
        raise ValueError("experiment manifest schema_version must be 1")
    require_keys(
        root["dataset"],
        required={"id", "lock_kind", "lock_manifest_sha256"},
        where="dataset",
    )
    config = require_keys(
        root["config"], required={"files", "dataset_bindings"}, where="config"
    )
    for index, item in enumerate(_list(config["files"], "config.files")):
        _locked_file(item, f"config.files[{index}]")
    for index, item in enumerate(_list(config["dataset_bindings"], "config.dataset_bindings")):
        _target(item, f"config.dataset_bindings[{index}]")
    software = require_keys(
        root["software"],
        required={"open_lmm_version", "runtime_api_version", "experiment_api_version", "source"},
        optional={"container_digest"},
        where="software",
    )
    source = require_keys(
        software["source"],
        required={"kind", "dirty"},
        optional={"identity", "commit", "sha256"},
        where="software.source",
    )
    _boolean(source["dirty"], "software.source.dirty")
    kind = source["kind"]
    identity_fields = [key for key in ("identity", "commit", "sha256") if source.get(key)]
    if len(identity_fields) != 1:
        raise ValueError("software.source must declare exactly one immutable identity")
    identity = source[identity_fields[0]]
    if kind == "git":
        if identity_fields[0] not in {"identity", "commit"} or not isinstance(identity, str) or len(identity) != 40 or any(c not in "0123456789abcdef" for c in identity):
            raise ValueError("git source identity must be a 40-character lowercase commit")
    elif kind in {"wheel", "distribution"}:
        require_sha256(identity, where="software source identity")
    elif kind == "container":
        require_sha256(software.get("container_digest", identity), where="container source identity")
    else:
        raise ValueError("unsupported software source kind")
    workflow = require_keys(root["workflow"], required={"kind", "stages"}, where="workflow")
    _list(workflow["stages"], "workflow.stages")
    execution = require_keys(
        root["execution"],
        required={
            "mode",
            "repetitions",
            "seeds",
            "timeout_seconds",
            "failure_policy",
            "strict_reproducibility",
            "max_trials",
        },
        optional={"event_retention", "log_limit_bytes"},
        where="execution",
    )
    _positive_int(execution["repetitions"], "execution.repetitions")
    _positive_int(execution["max_trials"], "execution.max_trials")
    _positive_number(execution["timeout_seconds"], "execution.timeout_seconds")
    _boolean(execution["strict_reproducibility"], "execution.strict_reproducibility")
    _list(execution["seeds"], "execution.seeds")
    for index, item in enumerate(_list(root["fixed_patches"], "fixed_patches")):
        _patch(item, f"fixed_patches[{index}]")
    for index, raw in enumerate(_list(root["parameter_axes"], "parameter_axes")):
        axis = require_keys(raw, required={"name", "target", "values"}, where=f"parameter_axes[{index}]")
        _target(axis["target"], f"parameter_axes[{index}].target")
        _list(axis["values"], f"parameter_axes[{index}].values")
    for index, raw in enumerate(_list(root["algorithm_variants"], "algorithm_variants")):
        variant = require_keys(
            raw,
            required={"id", "slot", "patches", "expected_plugin_ids"},
            where=f"algorithm_variants[{index}]",
        )
        for patch_index, item in enumerate(_list(variant["patches"], "variant.patches")):
            _patch(item, f"algorithm_variants[{index}].patches[{patch_index}]")
        _list(variant["expected_plugin_ids"], "variant.expected_plugin_ids")
    metric_values = require_keys(
        root["metrics"],
        required={
            "include_visualization",
            "include_points",
            "preview_voxel_size_m",
            "hash_output_files",
        },
        where="metrics",
    )
    _boolean(metric_values["include_visualization"], "metrics.include_visualization")
    _boolean(metric_values["include_points"], "metrics.include_points")
    _boolean(metric_values["hash_output_files"], "metrics.hash_output_files")
    if metric_values["preview_voxel_size_m"] is not None:
        _positive_number(metric_values["preview_voxel_size_m"], "metrics.preview_voxel_size_m")
    canonical_json_bytes(root)
    return root


def load_plan(
    manifest: Path,
    *,
    dataset_root: Path,
    config_root: Path | None,
) -> tuple[ExperimentPlan, bytes]:
    document = validate_manifest_document(load_closed_json(manifest))
    dataset_value = document["dataset"]
    lock_kind = InputLockKind(dataset_value["lock_kind"])
    lock_name = "dataset.index.json" if lock_kind == InputLockKind.SHA256_INDEX_V1 else "replay-case.json"
    dataset = DatasetSpec(
        dataset_value["id"],
        dataset_root,
        lock_kind,
        dataset_root / lock_name,
        dataset_value["lock_manifest_sha256"],
    )
    config_value = document["config"]
    config_directory = (config_root if config_root is not None else manifest.parent).resolve()
    config = ConfigSpec(
        config_directory,
        tuple(_locked_file(item, "config file") for item in config_value["files"]),
        tuple(_target(item, "dataset binding") for item in config_value["dataset_bindings"]),
    )
    source = document["software"]["source"]
    identity = source.get("identity", "")
    if not identity:
        identity = source.get("commit", source.get("sha256", ""))
    software = SoftwareIdentity(
        document["software"]["open_lmm_version"],
        document["software"]["runtime_api_version"],
        document["software"]["experiment_api_version"],
        source["kind"],
        identity,
        source["dirty"],
        document["software"].get("container_digest", identity if source["kind"] == "container" else None),
    )
    workflow_value = document["workflow"]
    try:
        stages = tuple(_STAGES[item] for item in workflow_value["stages"])
    except KeyError as error:
        raise ValueError(f"unknown public workflow stage {error.args[0]!r}") from error
    workflow = WorkflowSpec(WorkflowKind(workflow_value["kind"]), stages)
    execution_value = document["execution"]
    execution = ExecutionPolicy(
        ExecutionMode(execution_value["mode"]),
        execution_value["repetitions"],
        tuple(execution_value["seeds"]),
        float(execution_value["timeout_seconds"]),
        FailurePolicy(execution_value["failure_policy"]),
        bool(execution_value["strict_reproducibility"]),
        execution_value["max_trials"],
        execution_value.get("event_retention", 256),
        execution_value.get("log_limit_bytes", 1024 * 1024),
    )
    metrics_value = document["metrics"]
    metrics = MetricPolicy(
        bool(metrics_value["include_visualization"]),
        bool(metrics_value["include_points"]),
        metrics_value["preview_voxel_size_m"],
        bool(metrics_value["hash_output_files"]),
    )
    axes = []
    for item in document["parameter_axes"]:
        axes.append(ParameterAxis(item["name"], _target(item["target"], "axis target"), tuple(item["values"])))
    variants = []
    for item in document["algorithm_variants"]:
        variants.append(
            AlgorithmVariant(
                item["id"],
                AlgorithmSlot(item["slot"]),
                tuple(_patch(patch, "variant patch") for patch in item["patches"]),
                tuple(item["expected_plugin_ids"]),
            )
        )
    plan = ExperimentPlan(
        document["experiment_id"],
        dataset,
        config,
        software,
        workflow,
        execution,
        metrics,
        tuple(_patch(item, "fixed patch") for item in document["fixed_patches"]),
        tuple(axes),
        tuple(variants),
    )
    return plan, canonical_json_bytes(document)


def load_dataset_catalog(path: Path) -> DatasetCatalog:
    document = require_keys(
        load_closed_json(path), required={"schema_version", "datasets"}, where="dataset catalog"
    )
    if document["schema_version"] != 1:
        raise ValueError("dataset catalog schema_version must be 1")
    datasets: dict[str, DatasetSpec] = {}
    for index, raw in enumerate(_list(document["datasets"], "datasets")):
        item = require_keys(
            raw,
            required={"id", "root", "lock_kind", "lock_manifest", "lock_sha256"},
            where=f"datasets[{index}]",
        )
        root_text = require_relative_path(item["root"], where="catalog dataset root")
        manifest_text = require_relative_path(item["lock_manifest"], where="catalog lock manifest")
        root = (path.parent / root_text).resolve()
        spec = DatasetSpec(
            item["id"], root, InputLockKind(item["lock_kind"]), root / manifest_text, item["lock_sha256"]
        )
        if spec.id in datasets:
            raise ValueError(f"duplicate dataset ID {spec.id!r}")
        datasets[spec.id] = spec
    return DatasetCatalog(datasets)


def plan_identity(plan: ExperimentPlan) -> Mapping[str, Any]:
    return {
        "schema_version": 1,
        "experiment_id": plan.id,
        "dataset": {
            "id": plan.dataset.id,
            "lock_kind": plan.dataset.lock_kind.value,
            "lock_manifest_sha256": plan.dataset.lock_sha256,
        },
        "config": {
            "files": [
                {"path": item.path, "sha256": item.sha256, **({"size": item.size} if item.size is not None else {})}
                for item in plan.config.files
            ],
            "dataset_bindings": [
                {"file": item.file, "pointer": item.pointer} for item in plan.config.dataset_bindings
            ],
        },
        "software": {
            "open_lmm_version": plan.software.open_lmm_version,
            "runtime_api_version": plan.software.runtime_api_version,
            "experiment_api_version": plan.software.experiment_api_version,
            "source": {
                "kind": plan.software.source_kind,
                "identity": plan.software.source_identity,
                "dirty": plan.software.dirty,
            },
            **({"container_digest": plan.software.container_digest} if plan.software.container_digest else {}),
        },
        "workflow": {
            "kind": plan.workflow.kind.value,
            "stages": [next(name for name, stage in _STAGES.items() if stage == value) for value in plan.workflow.stages],
        },
        "execution": {
            "mode": plan.execution.mode.value,
            "repetitions": plan.execution.repetitions,
            "seeds": list(plan.execution.seeds),
            "timeout_seconds": plan.execution.timeout_seconds,
            "failure_policy": plan.execution.failure_policy.value,
            "strict_reproducibility": plan.execution.strict_reproducibility,
            "max_trials": plan.execution.max_trials,
            "event_retention": plan.execution.event_retention,
            "log_limit_bytes": plan.execution.log_limit_bytes,
        },
        "fixed_patches": [_patch_dict(item) for item in plan.fixed_patches],
        "parameter_axes": [
            {
                "name": item.name,
                "target": {"file": item.target.file, "pointer": item.target.pointer},
                "values": list(item.values),
            }
            for item in plan.parameter_axes
        ],
        "algorithm_variants": [
            {
                "id": item.id,
                "slot": item.slot.value,
                "patches": [_patch_dict(patch) for patch in item.patches],
                "expected_plugin_ids": list(item.expected_plugin_ids),
            }
            for item in plan.algorithm_variants
        ],
        "metrics": {
            "include_visualization": plan.metrics.include_visualization,
            "include_points": plan.metrics.include_points,
            "preview_voxel_size_m": plan.metrics.preview_voxel_size_m,
            "hash_output_files": plan.metrics.hash_output_files,
        },
    }


def _patch_dict(item: ConfigPatch) -> Mapping[str, Any]:
    return {"file": item.file, "pointer": item.pointer, "value": item.value}
