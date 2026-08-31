from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any, Mapping


_SHA256_PREFIX = "sha256:"


def canonical_value(value: Any, *, where: str = "value") -> Any:
    """Return an owned, immutable JSON value or fail closed."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{where} must not contain NaN or Infinity")
        if value == 0.0 and math.copysign(1.0, value) < 0:
            raise ValueError(f"{where} must not contain negative zero")
        return value
    if isinstance(value, (list, tuple)):
        return tuple(
            canonical_value(item, where=f"{where}[{index}]")
            for index, item in enumerate(value)
        )
    if isinstance(value, Mapping):
        owned: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{where} object keys must be strings")
            owned[key] = canonical_value(item, where=f"{where}.{key}")
        return MappingProxyType(dict(sorted(owned.items())))
    raise ValueError(f"{where} contains unsupported JSON type {type(value).__name__}")


def plain_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: plain_value(item) for key, item in sorted(value.items())}
    if isinstance(value, tuple):
        return [plain_value(item) for item in value]
    return value


def canonical_json_bytes(value: Any) -> bytes:
    normalized = plain_value(canonical_value(value))
    return (
        json.dumps(
            normalized,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def digest_bytes(payload: bytes) -> str:
    return _SHA256_PREFIX + hashlib.sha256(payload).hexdigest()


def digest_value(value: Any) -> str:
    return digest_bytes(canonical_json_bytes(value))


def digest_file(path: str | os.PathLike[str]) -> str:
    hasher = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return _SHA256_PREFIX + hasher.hexdigest()


def require_sha256(value: str, *, where: str = "sha256") -> str:
    if not isinstance(value, str):
        raise ValueError(f"{where} must be a string")
    if len(value) != 71 or not value.startswith(_SHA256_PREFIX):
        raise ValueError(f"{where} must be sha256:<64 lowercase hex>")
    suffix = value[len(_SHA256_PREFIX) :]
    if any(character not in "0123456789abcdef" for character in suffix):
        raise ValueError(f"{where} must be sha256:<64 lowercase hex>")
    return value


def require_relative_path(value: str, *, where: str = "path") -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{where} must be a non-empty relative path")
    if "\\" in value:
        raise ValueError(f"{where} must use portable '/' separators")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in path.parts):
        raise ValueError(f"{where} must not be absolute or traverse its root")
    if path.as_posix() != value:
        raise ValueError(f"{where} must use canonical relative path syntax")
    return path.as_posix()


def load_closed_json(path: str | os.PathLike[str]) -> Any:
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream, parse_constant=lambda token: (_raise_nonfinite(token)))


def _raise_nonfinite(token: str) -> None:
    raise ValueError(f"JSON must not contain {token}")


def require_keys(
    value: Any,
    *,
    required: set[str],
    optional: set[str] = frozenset(),
    where: str,
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{where} must be an object")
    keys = set(value)
    missing = required - keys
    unknown = keys - required - optional
    if missing:
        raise ValueError(f"{where} is missing keys: {', '.join(sorted(missing))}")
    if unknown:
        raise ValueError(f"{where} has unknown keys: {', '.join(sorted(unknown))}")
    return value


def atomic_write(path: str | os.PathLike[str], payload: bytes) -> None:
    target = Path(path)
    if target.exists():
        raise FileExistsError(f"refusing to overwrite {target.name}")
    temporary = target.with_name(f".{target.name}.tmp-{os.getpid()}")
    try:
        descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, target)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
