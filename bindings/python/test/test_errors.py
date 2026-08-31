from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import open_lmm
from open_lmm import _errors


class ErrorTests(unittest.TestCase):
    def test_every_native_error_code_maps_to_its_public_subclass(self) -> None:
        expected = {
            open_lmm.ErrorCode.FILE_NOT_FOUND: open_lmm.OpenLMMFileNotFoundError,
            open_lmm.ErrorCode.PARSE_ERROR: open_lmm.OpenLMMParseError,
            open_lmm.ErrorCode.INVALID_ARGUMENT: open_lmm.OpenLMMInvalidArgumentError,
            open_lmm.ErrorCode.PLUGIN_LOAD_FAILED: open_lmm.OpenLMMPluginLoadError,
            open_lmm.ErrorCode.REGISTRATION_FAILED: open_lmm.OpenLMMRegistrationError,
            open_lmm.ErrorCode.OPTIMIZATION_FAILED: open_lmm.OpenLMMOptimizationError,
            open_lmm.ErrorCode.IO_ERROR: open_lmm.OpenLMMIOError,
            open_lmm.ErrorCode.CANCELLED: open_lmm.OpenLMMCancelledError,
            open_lmm.ErrorCode.AGENT_EXCLUDED: open_lmm.OpenLMMAgentExcludedError,
            open_lmm.ErrorCode.INTERNAL: open_lmm.OpenLMMInternalError,
        }
        for code, exception_type in expected.items():
            payload = {
                "code": int(code),
                "message": f"mapped-{code.name}",
                "severity": int(open_lmm.ErrorSeverity.FATAL_RUNTIME),
                "context": {
                    "runtime_revision": 17,
                    "stage": "MapUpdate",
                    "node": "MapUpdate",
                    "agent": "agent1",
                    "plugin": "fixture",
                    "config": "core/test.json",
                    "json_pointer": "/fixture",
                    "expected": "expected",
                    "actual": "actual",
                    "schema_version": 3,
                },
            }
            with self.subTest(code=code), self.assertRaises(exception_type) as caught:
                _errors._raise_from_native(payload)
            self.assertEqual(caught.exception.code, code)
            self.assertEqual(
                caught.exception.severity,
                open_lmm.ErrorSeverity.FATAL_RUNTIME,
            )
            self.assertEqual(caught.exception.context.runtime_revision, 17)
            self.assertEqual(caught.exception.context.agent, "agent1")

    def test_invalid_constructor_uses_structured_exception(self) -> None:
        with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError) as caught:
            open_lmm.Runtime(0)
        self.assertEqual(caught.exception.code, open_lmm.ErrorCode.INVALID_ARGUMENT)
        self.assertEqual(
            caught.exception.severity, open_lmm.ErrorSeverity.RECOVERABLE
        )
        self.assertIsNone(caught.exception.context.runtime_revision)

    def test_missing_config_preserves_native_error(self) -> None:
        runtime = open_lmm.Runtime(1)
        missing = Path(tempfile.gettempdir()) / "open_lmm_python_missing_config"
        with self.assertRaises(open_lmm.OpenLMMError) as caught:
            runtime.open(missing)
        self.assertIn(
            caught.exception.code,
            {
                open_lmm.ErrorCode.FILE_NOT_FOUND,
                open_lmm.ErrorCode.INVALID_ARGUMENT,
            },
        )
        self.assertTrue(str(caught.exception))
        runtime.close()

    def test_python_argument_error_uses_same_hierarchy(self) -> None:
        runtime = open_lmm.Runtime(1)
        try:
            with self.assertRaises(open_lmm.OpenLMMInvalidArgumentError):
                runtime.subscribe_events(None)
        finally:
            runtime.close()


if __name__ == "__main__":
    unittest.main()
