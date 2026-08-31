from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping


class ErrorCode(IntEnum):
    FILE_NOT_FOUND = 0
    PARSE_ERROR = 1
    INVALID_ARGUMENT = 2
    PLUGIN_LOAD_FAILED = 3
    REGISTRATION_FAILED = 4
    OPTIMIZATION_FAILED = 5
    IO_ERROR = 6
    CANCELLED = 7
    AGENT_EXCLUDED = 8
    RESOURCE_EXHAUSTED = 9
    INTERNAL = 255


class ErrorSeverity(IntEnum):
    RECOVERABLE = 0
    FATAL_RUNTIME = 1


@dataclass(frozen=True, slots=True)
class ErrorContext:
    runtime_revision: int | None
    stage: str
    node: str
    agent: str | None
    plugin: str
    config: str
    json_pointer: str
    expected: str
    actual: str
    schema_version: int | None


@dataclass(frozen=True, slots=True)
class ErrorInfo:
    code: ErrorCode
    message: str
    severity: ErrorSeverity
    context: ErrorContext


class OpenLMMError(RuntimeError):
    def __init__(self, info: ErrorInfo) -> None:
        super().__init__(info.message)
        self.code = info.code
        self.severity = info.severity
        self.context = info.context
        self.info = info


class OpenLMMFileNotFoundError(OpenLMMError):
    pass


class OpenLMMParseError(OpenLMMError):
    pass


class OpenLMMInvalidArgumentError(OpenLMMError):
    pass


class OpenLMMPluginLoadError(OpenLMMError):
    pass


class OpenLMMRegistrationError(OpenLMMError):
    pass


class OpenLMMOptimizationError(OpenLMMError):
    pass


class OpenLMMIOError(OpenLMMError):
    pass


class OpenLMMCancelledError(OpenLMMError):
    pass


class OpenLMMAgentExcludedError(OpenLMMError):
    pass


class OpenLMMResourceExhaustedError(OpenLMMError):
    pass


class OpenLMMInternalError(OpenLMMError):
    pass


_EXCEPTION_BY_CODE: dict[ErrorCode, type[OpenLMMError]] = {
    ErrorCode.FILE_NOT_FOUND: OpenLMMFileNotFoundError,
    ErrorCode.PARSE_ERROR: OpenLMMParseError,
    ErrorCode.INVALID_ARGUMENT: OpenLMMInvalidArgumentError,
    ErrorCode.PLUGIN_LOAD_FAILED: OpenLMMPluginLoadError,
    ErrorCode.REGISTRATION_FAILED: OpenLMMRegistrationError,
    ErrorCode.OPTIMIZATION_FAILED: OpenLMMOptimizationError,
    ErrorCode.IO_ERROR: OpenLMMIOError,
    ErrorCode.CANCELLED: OpenLMMCancelledError,
    ErrorCode.AGENT_EXCLUDED: OpenLMMAgentExcludedError,
    ErrorCode.RESOURCE_EXHAUSTED: OpenLMMResourceExhaustedError,
    ErrorCode.INTERNAL: OpenLMMInternalError,
}


def error_info_from_native(payload: Mapping[str, Any]) -> ErrorInfo:
    context = payload.get("context", {})
    return ErrorInfo(
        code=ErrorCode(payload["code"]),
        message=str(payload["message"]),
        severity=ErrorSeverity(payload["severity"]),
        context=ErrorContext(
            runtime_revision=context.get("runtime_revision"),
            stage=str(context.get("stage", "")),
            node=str(context.get("node", "")),
            agent=context.get("agent"),
            plugin=str(context.get("plugin", "")),
            config=str(context.get("config", "")),
            json_pointer=str(context.get("json_pointer", "")),
            expected=str(context.get("expected", "")),
            actual=str(context.get("actual", "")),
            schema_version=context.get("schema_version"),
        ),
    )


def _raise_from_native(payload: Mapping[str, Any]) -> None:
    info = error_info_from_native(payload)
    raise _EXCEPTION_BY_CODE[info.code](info)


def _raise_invalid_argument(message: str) -> None:
    info = ErrorInfo(
        ErrorCode.INVALID_ARGUMENT,
        message,
        ErrorSeverity.RECOVERABLE,
        ErrorContext(None, "", "", None, "", "", "", "", "", None),
    )
    raise OpenLMMInvalidArgumentError(info)


def _raise_internal(message: str) -> None:
    info = ErrorInfo(
        ErrorCode.INTERNAL,
        message,
        ErrorSeverity.FATAL_RUNTIME,
        ErrorContext(None, "", "", None, "", "", "", "", "", None),
    )
    raise OpenLMMInternalError(info)
