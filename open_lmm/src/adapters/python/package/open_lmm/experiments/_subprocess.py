from __future__ import annotations

import os
import signal
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True, slots=True)
class ProcessOutcome:
    returncode: int
    pid: int
    timed_out: bool
    output_bytes: int
    output_truncated: bool


def safe_environment(*, include_home: bool = False) -> dict[str, str]:
    keys = (
        "LANG",
        "LC_ALL",
        "LD_LIBRARY_PATH",
        "PYTHONPATH",
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
    )
    result = {key: os.environ[key] for key in keys if key in os.environ}
    if include_home and "HOME" in os.environ:
        result["HOME"] = os.environ["HOME"]
    result["PATH"] = os.defpath
    return result


def run_logged(
    argv: Sequence[str],
    *,
    log_path: Path,
    timeout_seconds: float,
    log_limit_bytes: int,
    environment: Mapping[str, str] | None = None,
) -> ProcessOutcome:
    if log_path.exists():
        raise FileExistsError(f"log path already exists: {log_path.name}")
    process = subprocess.Popen(
        list(argv),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None if environment is None else dict(environment),
        start_new_session=True,
    )
    total = 0
    retained = 0
    reader_error: list[BaseException] = []
    lock = threading.Lock()

    def drain() -> None:
        nonlocal total, retained
        assert process.stdout is not None
        try:
            with process.stdout, log_path.open("xb") as log:
                for block in iter(lambda: process.stdout.read(64 * 1024), b""):
                    with lock:
                        total += len(block)
                    remaining = log_limit_bytes - retained
                    if remaining > 0:
                        payload = block[:remaining]
                        log.write(payload)
                        retained += len(payload)
        except BaseException as error:
            reader_error.append(error)

    thread = threading.Thread(target=drain, name="openlmm-log-drain")
    thread.start()
    timed_out = False
    try:
        returncode = process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        timed_out = True
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            returncode = process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            returncode = process.wait()
    thread.join()
    if reader_error:
        raise RuntimeError(f"failed to drain subprocess log: {reader_error[0]}")
    truncated = total > log_limit_bytes
    if truncated:
        marker = f"\n[open_lmm log truncated: original_bytes={total}]\n".encode("utf-8")
        with log_path.open("r+b") as log:
            keep = max(0, log_limit_bytes - len(marker))
            log.truncate(keep)
            log.seek(keep)
            log.write(marker[: log_limit_bytes - keep])
    return ProcessOutcome(returncode, process.pid, timed_out, total, truncated)
