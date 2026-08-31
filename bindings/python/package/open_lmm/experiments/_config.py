from __future__ import annotations

import json
import re
import shutil
from pathlib import Path
from typing import Iterable, Mapping

from ._canonical import canonical_json_bytes, digest_file, digest_value, require_keys, require_relative_path
from ._models import ConfigPatch, ConfigSpec, DatasetSpec, InputLockKind, LockedFile


def _safe_regular(root: Path, relative: str, *, where: str) -> Path:
    relative = require_relative_path(relative, where=where)
    path = root / relative
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{where} must name a regular non-symlink file: {relative}")
    if path.resolve().parent != root.resolve() and root.resolve() not in path.resolve().parents:
        raise ValueError(f"{where} escapes its root: {relative}")
    return path


def verify_config(spec: ConfigSpec) -> str:
    root = spec.directory
    if not root.is_dir() or root.is_symlink():
        raise ValueError("config root must be a regular directory")
    declared = {item.path: item for item in spec.files}
    actual: set[str] = set()
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"config tree contains symlink: {path.relative_to(root)}")
        if path.is_file():
            actual.add(path.relative_to(root).as_posix())
        elif not path.is_dir():
            raise ValueError(f"config tree contains special file: {path.relative_to(root)}")
    if actual != set(declared):
        raise ValueError(f"config file set mismatch: missing={sorted(set(declared)-actual)}, extra={sorted(actual-set(declared))}")
    for relative, item in declared.items():
        path = _safe_regular(root, relative, where="config file")
        if item.size is not None and path.stat().st_size != item.size:
            raise ValueError(f"config size mismatch: {relative}")
        if digest_file(path) != item.sha256:
            raise ValueError(f"config digest mismatch: {relative}")
    return digest_value([{"path": item.path, "sha256": item.sha256} for item in sorted(spec.files, key=lambda x: x.path)])


def verify_dataset(spec: DatasetSpec) -> str:
    root = spec.root
    if not root.is_dir() or root.is_symlink():
        raise ValueError("dataset root must be a regular directory")
    manifest = spec.lock_manifest
    if manifest.is_symlink() or not manifest.is_file() or root.resolve() not in manifest.resolve().parents:
        raise ValueError("dataset lock manifest must be a regular file inside dataset root")
    if digest_file(spec.lock_manifest) != spec.lock_sha256:
        raise ValueError("dataset lock manifest digest mismatch")
    if spec.lock_kind == InputLockKind.REPLAY_CASE_V1:
        raise ValueError("replay-case-v1 input must be executed through ReplayAdapter")
    document = json.loads(
        spec.lock_manifest.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_invalid_number(token)),
    )
    index = require_keys(document, required={"schema_version", "dataset_id", "files"}, where="dataset index")
    if index["schema_version"] != 1 or index["dataset_id"] != spec.id or not isinstance(index["files"], list):
        raise ValueError("dataset index identity/schema mismatch")
    declared: dict[str, LockedFile] = {}
    for position, raw in enumerate(index["files"]):
        item = require_keys(raw, required={"path", "size", "sha256"}, where=f"dataset index files[{position}]")
        locked = LockedFile(item["path"], item["sha256"], item["size"])
        if locked.path in declared:
            raise ValueError(f"duplicate dataset path {locked.path!r}")
        declared[locked.path] = locked
    if list(declared) != sorted(declared):
        raise ValueError("dataset index paths must be sorted")
    actual: set[str] = set()
    manifest_resolved = spec.lock_manifest.resolve()
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"dataset contains symlink: {path.relative_to(root)}")
        if path.is_file() and path.resolve() != manifest_resolved:
            actual.add(path.relative_to(root).as_posix())
        elif not path.is_file() and not path.is_dir():
            raise ValueError(f"dataset contains special file: {path.relative_to(root)}")
    if actual != set(declared):
        raise ValueError(f"dataset file set mismatch: missing={sorted(set(declared)-actual)}, extra={sorted(actual-set(declared))}")
    for relative, item in declared.items():
        path = _safe_regular(root, relative, where="dataset file")
        if path.stat().st_size != item.size or digest_file(path) != item.sha256:
            raise ValueError(f"dataset file lock mismatch: {relative}")
    return spec.lock_sha256


def _pointer_tokens(pointer: str) -> list[str]:
    if pointer == "":
        raise ValueError("root replacement is not supported")
    tokens = pointer[1:].split("/")
    result = []
    for token in tokens:
        if re.search(r"~(?![01])", token):
            raise ValueError(f"invalid JSON Pointer escape in {pointer!r}")
        token = token.replace("~1", "/").replace("~0", "~")
        result.append(token)
    return result


def _replace(document: object, pointer: str, value: object) -> None:
    tokens = _pointer_tokens(pointer)
    parent = document
    for token in tokens[:-1]:
        if isinstance(parent, dict) and token in parent:
            parent = parent[token]
        elif isinstance(parent, list) and token.isdigit() and int(token) < len(parent):
            parent = parent[int(token)]
        else:
            raise ValueError(f"JSON Pointer target does not exist: {pointer}")
    final = tokens[-1]
    if isinstance(parent, dict) and final in parent:
        _require_compatible_type(parent[final], value, pointer)
        parent[final] = value
    elif isinstance(parent, list) and final.isdigit() and int(final) < len(parent):
        _require_compatible_type(parent[int(final)], value, pointer)
        parent[int(final)] = value
    else:
        raise ValueError(f"JSON Pointer target does not exist: {pointer}")


def _require_compatible_type(previous: object, replacement: object, pointer: str) -> None:
    def category(value: object) -> str:
        if value is None:
            return "null"
        if isinstance(value, bool):
            return "boolean"
        if isinstance(value, (int, float)):
            return "number"
        if isinstance(value, str):
            return "string"
        if isinstance(value, (list, tuple)):
            return "array"
        if isinstance(value, Mapping):
            return "object"
        return type(value).__name__

    if category(previous) != category(replacement):
        raise ValueError(
            f"JSON Pointer replacement type mismatch at {pointer}: "
            f"{category(previous)} != {category(replacement)}"
        )


def validate_patch_targets(
    spec: ConfigSpec,
    patches: Iterable[ConfigPatch],
    *,
    include_bindings: bool = True,
) -> None:
    declared = {item.path for item in spec.files}
    seen: set[tuple[str, str]] = set()
    if include_bindings:
        seen.update((item.file, item.pointer) for item in spec.dataset_bindings)
    for patch in patches:
        if patch.file not in declared:
            raise ValueError(f"patch file is not declared: {patch.file}")
        key = (patch.file, patch.pointer)
        if key in seen:
            raise ValueError(f"multiple writers for config target {patch.file}{patch.pointer}")
        seen.add(key)


def materialize_config(
    spec: ConfigSpec,
    dataset: DatasetSpec,
    destination: Path,
    patches: tuple[ConfigPatch, ...],
) -> tuple[str, tuple[LockedFile, ...]]:
    verify_config(spec)
    validate_patch_targets(spec, patches)
    if destination.exists():
        raise FileExistsError("trial config destination already exists")
    destination.mkdir(parents=True)
    for item in spec.files:
        source = _safe_regular(spec.directory, item.path, where="config source")
        target = destination / item.path
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as input_stream, target.open("xb") as output_stream:
            shutil.copyfileobj(input_stream, output_stream, 1024 * 1024)
    grouped: dict[str, list[ConfigPatch]] = {}
    for binding in spec.dataset_bindings:
        grouped.setdefault(binding.file, []).append(ConfigPatch(binding.file, binding.pointer, str(dataset.root)))
    for patch in patches:
        grouped.setdefault(patch.file, []).append(patch)
    for relative, file_patches in grouped.items():
        target = destination / relative
        with target.open("r", encoding="utf-8") as stream:
            document = json.load(stream, parse_constant=lambda token: (_invalid_number(token)))
        for patch in file_patches:
            _replace(document, patch.pointer, patch.value)
        payload = canonical_json_bytes(document)
        target.write_bytes(payload)
    locked = tuple(
        LockedFile(item.path, digest_file(destination / item.path), (destination / item.path).stat().st_size)
        for item in sorted(spec.files, key=lambda value: value.path)
    )
    aggregate = digest_value([{"path": item.path, "sha256": item.sha256, "size": item.size} for item in locked])
    return aggregate, locked


def _invalid_number(token: str) -> None:
    raise ValueError(f"config must not contain {token}")
