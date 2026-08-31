from __future__ import annotations

import csv
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import zipfile


def fail(message: str) -> None:
    raise SystemExit(message)


def dynamic_section(path: Path) -> str:
    result = subprocess.run(
        ["readelf", "-d", str(path)],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if result.returncode:
        fail(f"readelf failed for {path}:\n{result.stdout}")
    return result.stdout


def main() -> None:
    if len(sys.argv) != 4:
        fail("usage: wheel_audit.py WHEEL REPOSITORY_ROOT CORE_BUILD_ROOT")
    wheel = Path(sys.argv[1]).resolve()
    repository_root = Path(sys.argv[2]).resolve()
    core_build_root = Path(sys.argv[3]).resolve()
    if not wheel.is_file():
        fail(f"wheel is missing: {wheel}")

    with tempfile.TemporaryDirectory(prefix="open_lmm_wheel_audit_") as temp:
        root = Path(temp)
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(root)

        package = root / "open_lmm"
        library_dir = package / ".libs"
        manifests = list(package.glob("open_lmm-python-runtime-closure.tsv"))
        if len(manifests) != 1:
            fail("wheel must contain one runtime closure manifest")
        with manifests[0].open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream, delimiter="\t"))
        required_columns = {
            "kind",
            "logical_name",
            "source_relative_path",
            "wheel_relative_path",
            "required",
            "license",
        }
        if not rows or set(rows[0]) != required_columns:
            fail("runtime closure manifest schema changed")
        logical_names = [row["logical_name"] for row in rows]
        if logical_names != sorted(logical_names) or len(set(logical_names)) != 23:
            fail("runtime closure must contain 23 unique sorted logical DSOs")
        for row in rows:
            logical = row["logical_name"]
            if (
                row["source_relative_path"] != f"lib/{logical}"
                or row["wheel_relative_path"] != f"open_lmm/.libs/{logical}"
                or row["required"] != "true"
                or row["license"] != "GPL-3.0-only"
            ):
                fail(f"invalid runtime closure row: {row}")

        expected_files = {
            f"{logical}{suffix}"
            for logical in logical_names
            for suffix in ("", ".3", ".3.0.0")
        }
        actual_files = {path.name for path in library_dir.iterdir() if path.is_file()}
        if actual_files != expected_files:
            fail(
                "wheel runtime closure changed\n"
                f"missing={sorted(expected_files - actual_files)}\n"
                f"extra={sorted(actual_files - expected_files)}"
            )

        native_modules = list(package.glob("_native*.so"))
        if len(native_modules) != 1:
            fail(f"wheel must contain one native module: {native_modules}")
        root_dsos = [path for path in package.glob("*.so*") if path != native_modules[0]]
        if root_dsos:
            fail(f"native DSOs escaped .libs: {root_dsos}")
        native_dynamic = dynamic_section(native_modules[0])
        if not re.search(r"\((?:RPATH|RUNPATH)\).*\[\$ORIGIN/\.libs\]", native_dynamic):
            fail(f"native module RUNPATH changed:\n{native_dynamic}")

        owned_sonames = {
            name
            for name in expected_files
            if name.endswith(".so.3") or name.endswith(".so")
        }
        elf_paths = [native_modules[0]] + sorted(library_dir.glob("*.so.3.0.0"))
        for elf in elf_paths:
            section = dynamic_section(elf)
            if elf.parent == library_dir and not re.search(
                r"\((?:RPATH|RUNPATH)\).*\[\$ORIGIN\]", section
            ):
                fail(f"staged DSO RUNPATH changed: {elf.name}\n{section}")
            for needed in re.findall(r"Shared library: \[([^]]+)\]", section):
                if needed.startswith(
                    (
                        "libopen_lmm_",
                        "libcreate_",
                        "libdufomap",
                        "liberasor",
                        "libfree_dom",
                        "libhmm_mos",
                        "libotd",
                    )
                ):
                    if needed not in owned_sonames:
                        fail(f"{elf.name} needs an unstaged OpenLMM DSO: {needed}")

        ldd_environment = os.environ.copy()
        ldd_environment["LD_LIBRARY_PATH"] = str(library_dir)
        plugin_closure = [
            row["logical_name"] for row in rows if row["kind"] != "runtime"
        ]
        for logical in plugin_closure:
            result = subprocess.run(
                ["ldd", str(library_dir / logical)],
                check=False,
                text=True,
                env=ldd_environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            if result.returncode or "not found" in result.stdout:
                fail(f"selected plugin closure is unresolved: {logical}\n{result.stdout}")

        for metadata in (
            "LICENCE",
            "RELEASE_POLICY.md",
            "THIRD_PARTY_NOTICES.md",
            "LOCAL_WHEEL_POLICY.md",
            "open_lmm-python-runtime-closure.tsv",
        ):
            if not (package / metadata).is_file():
                fail(f"wheel metadata is missing: {metadata}")

        forbidden = (
            str(repository_root).encode(),
            str(core_build_root).encode(),
            b"open_lmm/src/",
        )
        for path in root.rglob("*"):
            if path.is_file():
                contents = path.read_bytes()
                for marker in forbidden:
                    if marker and marker in contents:
                        fail(f"wheel retained a source/build path in {path.relative_to(root)}")

    print("wheel audit passed: 23 logical DSOs / 69 runtime paths")


if __name__ == "__main__":
    main()
