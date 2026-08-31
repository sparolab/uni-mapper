from __future__ import annotations

import hashlib
import itertools
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

from ._canonical import digest_value, plain_value
from ._config import validate_patch_targets
from ._manifest import plan_identity
from ._models import AlgorithmVariant, ConfigPatch, ExperimentPlan, JsonValue


@dataclass(frozen=True, slots=True)
class PlannedTrial:
    trial_id: str
    ordinal: int
    seed: int
    variant_id: str | None
    parameters: Mapping[str, JsonValue]
    patches: tuple[ConfigPatch, ...]


def _derived_seed(plan_digest: str, repetition: int) -> int:
    payload = f"{plan_digest}:{repetition}".encode("ascii")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big") & 0x7FFFFFFF


def plan_trials(plan: ExperimentPlan) -> tuple[str, tuple[PlannedTrial, ...]]:
    identity = plan_identity(plan)
    plan_digest = digest_value(identity)
    axes = tuple(sorted(plan.parameter_axes, key=lambda item: item.name))
    variants: tuple[AlgorithmVariant | None, ...] = plan.algorithm_variants or (None,)
    counts = len(variants) * plan.execution.repetitions
    for axis in axes:
        counts *= len(axis.values)
    if counts > plan.execution.max_trials:
        raise ValueError(f"planned trial count {counts} exceeds max_trials={plan.execution.max_trials}")
    all_targets: list[ConfigPatch] = list(plan.fixed_patches)
    all_targets.extend(
        ConfigPatch(axis.target.file, axis.target.pointer, axis.values[0]) for axis in axes
    )
    validate_patch_targets(plan.config, all_targets)
    trials: list[PlannedTrial] = []
    value_products = tuple(itertools.product(*(axis.values for axis in axes))) if axes else ((),)
    for variant in variants:
        base = list(plan.fixed_patches)
        if variant is not None:
            validate_patch_targets(plan.config, tuple(base) + variant.patches)
            base.extend(variant.patches)
        for values in value_products:
            parameters = MappingProxyType({axis.name: value for axis, value in zip(axes, values)})
            axis_patches = tuple(
                ConfigPatch(axis.target.file, axis.target.pointer, value)
                for axis, value in zip(axes, values)
            )
            validate_patch_targets(plan.config, tuple(base) + axis_patches)
            for repetition in range(plan.execution.repetitions):
                seed = (
                    plan.execution.seeds[repetition]
                    if plan.execution.seeds
                    else _derived_seed(plan_digest, repetition)
                )
                ordinal = len(trials) + 1
                trial_semantics = {
                    "plan": plan_digest,
                    "ordinal": ordinal,
                    "seed": seed,
                    "variant": None if variant is None else variant.id,
                    "parameters": plain_value(parameters),
                }
                trial_id = digest_value(trial_semantics).split(":", 1)[1]
                trials.append(
                    PlannedTrial(
                        trial_id,
                        ordinal,
                        seed,
                        None if variant is None else variant.id,
                        parameters,
                        tuple(base) + axis_patches,
                    )
                )
    return plan_digest, tuple(trials)
